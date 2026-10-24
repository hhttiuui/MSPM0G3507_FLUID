/**
 * @file vcmd.c
 * @brief 串口下行文本命令解析（设计说明见 vcmd.h）
 *
 * 数据流：
 *   UART1 RX → DMA_CH0 → s_rx[128] → 主循环每圈取走新增字节
 *              → 按 '\n'/'\r' 切行 → 解析 → DBG_CmdSubmit()（写命令邮箱）
 *
 * 解析本身不碰任何外设和控制逻辑，只把文本翻译成"命令号 + 4 个整数参数"，
 * 所以新增命令只需要在 debug.h 里加一行 #define + 这里加一条别名，
 * 执行代码在 app.c 的 App_CmdTask()（全局命令）和当前场景的 cmd()（场景命令）。
 */

#include "vcmd.h"
#include <stdbool.h>
#include "debug.h"
#include "ti_msp_dl_config.h"

/* 一行最长字节数。够放下 "px 15 16 255" 这种，再长就判为无效行。 */
#define VCMD_LINE_MAX       (32u)

/* RX DMA 一次武装能收多少字节。
 * ⚠ 115200 下 128 字节 = 11 ms，比一帧（21.5 ms）短，所以正常粘贴不会溢出。
 *   再大就会跨越多个主循环周期，中途满了以后到达的字节会丢。 */
#define VCMD_RX_SIZE        (128u)

static char     s_rx[VCMD_RX_SIZE];
static char     s_line[VCMD_LINE_MAX];
static uint8_t  s_len;
static uint8_t  s_overflow;      /* 当前行超长，丢弃到下一个换行 */
static uint16_t s_rd;            /* 已消费到 s_rx 的哪个位置 */
static uint16_t s_seen;          /* 已计入 rx_bytes 统计的位置（≠ s_rd）*/
static uint8_t  s_full;          /* RX DMA 已收满并自动停下 */

/* ---- 别名表：手打方便。命令号见 debug.h 的 DBG_CMD_* ----
 * 顺序无所谓，查找是线性扫描（只在收到命令时跑一次，几微秒）。 */
typedef struct {
    const char *name;
    uint16_t    id;
} vcmd_alias_t;

static const vcmd_alias_t s_alias[] = {
    /* ---- 系统 ---- */
    { "div",    DBG_CMD_SET_DIV        },   /* div 0~7       遥测分频 */
    { "mask",   DBG_CMD_SET_MASK       },   /* mask 15       通道掩码 */
    { "reset",  DBG_CMD_RESET_STATS    },   /* reset         清零统计 */

    /* ---- 流体 ---- */
    { "dt",     DBG_CMD_FLUID_SET_DT   },   /* dt 3200       dt = 0.032 s */
    { "iter",   DBG_CMD_FLUID_SET_ITER },   /* iter 12       压力迭代 */
    { "shake",  DBG_CMD_FLUID_SET_SHAKE},   /* shake 150     摇晃增益 1.5 */
    { "pause",  DBG_CMD_FLUID_PAUSE    },   /* pause 1       暂停物理 */
    { "reinit", DBG_CMD_FLUID_REINIT   },   /* reinit        粒子归位 */
    { "full",   DBG_CMD_FLUID_SET_FULL },   /* full 60       满亮度密度 = 静止密度×60% */
    { "edge",   DBG_CMD_FLUID_EDGE     },   /* edge 15 50    边缘分段映射（抗锯齿）*/

    /* ---- LED 点阵（亮度标定）---- */
    { "bc",     DBG_CMD_LED_SET_BC     },   /* bc 128        全局亮度 0x37 */
    { "cc",     DBG_CMD_LED_SET_CC     },   /* cc 7          恒流档 0x36 */
    { "fill",   DBG_CMD_LED_FILL       },   /* fill 200      整屏亮度 */
    { "off",    DBG_CMD_LED_OFF        },   /* off           全灭 */
    { "px",     DBG_CMD_LED_PX         },   /* px 3 5 200    单点 */
    { "ramp",   DBG_CMD_LED_RAMP       },   /* ramp          14 级线性阶梯（标定用）*/
    { "rampg",  DBG_CMD_LED_RAMP_GAMMA },   /* rampg         14 级 gamma 阶梯（看观感）*/
    { "lock",   DBG_CMD_LED_LOCK       },   /* lock 1        冻结流体渲染 */
    { "gam",    DBG_CMD_LED_GAMMA      },   /* gam 1         流体渲染走 gamma2.2 */

    /* 场景切换 */
    { "mode",   DBG_CMD_APP_MODE       },   /* mode 1        切到指定场景 */
    { "next",   DBG_CMD_APP_NEXT       },   /* next          下一个场景 */
    { "prev",   DBG_CMD_APP_PREV       },   /* prev          上一个场景 */
};

#define VCMD_ALIAS_N  (sizeof(s_alias) / sizeof(s_alias[0]))

/* ==================== 小工具 ==================== */

static bool is_digit(char c)
{
    return (c >= '0') && (c <= '9');
}

/* 跳过空格、等号、逗号：三种都当分隔符，这样
 * "px 3 5 200" / "px=3" / "513,200" 都能解析。 */
static char *skip_sep(char *p)
{
    while ((*p == ' ') || (*p == '=') || (*p == ',')) {
        p++;
    }
    return p;
}

/* 解析一个十进制整数（可带负号）。成功返回新指针，失败返回 0。 */
static char *parse_int(char *p, int32_t *out)
{
    bool neg = false;
    bool any = false;
    int32_t v = 0;

    if (*p == '-') {
        neg = true;
        p++;
    }

    while (is_digit(*p)) {
        /* 超过 8 位就饱和，避免回绕成负数 */
        if (v > 200000000) {
            v = 200000000;
        } else {
            v = (v * 10) + (int32_t) (*p - '0');
        }
        any = true;
        p++;
    }

    if (!any) {
        return 0;
    }

    *out = (neg ? -v : v);
    return p;
}

/* 在别名表里查 nlen 个字符，命中返回命令号，否则 0xFFFF。 */
static uint16_t find_alias(const char *p, uint8_t nlen)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t) VCMD_ALIAS_N; i++) {
        const char *n = s_alias[i].name;
        uint8_t k = 0u;

        while ((n[k] != '\0') && (k < nlen) && (n[k] == p[k])) {
            k++;
        }
        if ((k == nlen) && (n[k] == '\0')) {
            return s_alias[i].id;
        }
    }
    return 0xFFFFu;
}

/* ==================== 解析并提交 ==================== */

static void vcmd_exec(char *line)
{
    char *p = skip_sep(line);
    char *q;
    uint8_t nlen;
    uint16_t id;
    int32_t a[4] = { 0, 0, 0, 0 };
    uint8_t k;

    if (*p == '\0') {
        return;                  /* 空行，忽略 */
    }

    /* ---- 先看是不是别名 ---- */
    q = p;
    while ((*q != '\0') && (*q != ' ') && (*q != '=') && (*q != ',')) {
        q++;
    }
    nlen = (uint8_t) (q - p);

    id = find_alias(p, nlen);

    if (id == 0xFFFFu) {
        /* ---- 不是别名：把第一段当命令号 ---- */
        int32_t v;

        q = parse_int(p, &v);
        if ((q == 0) || (v < 0) || (v > 0xFFFF)) {
            dbg.uart.bad++;
            return;
        }
        id = (uint16_t) v;
    }

    /* ---- 后面最多再取 4 个参数 ---- */
    p = q;
    for (k = 0u; k < 4u; k++) {
        char *r = parse_int(skip_sep(p), &a[k]);
        if (r == 0) {
            break;
        }
        p = r;
    }

    DBG_CmdSubmit(id, a[0], a[1], a[2], a[3]);
    dbg.uart.cmds++;
}

/* 逐字节喂进行缓冲。返回 1 表示这一行已经完整（可以执行了）。 */
static uint8_t vcmd_feed(char c)
{
    if ((c == '\n') || (c == '\r')) {
        if (s_overflow == 0u) {
            s_line[s_len] = '\0';
            vcmd_exec(s_line);
        }
        s_len = 0u;
        s_overflow = 0u;
        return 1u;
    }

    if (s_overflow != 0u) {
        return 0u;               /* 已经超长，丢到下一个换行为止 */
    }

    if (s_len >= (uint8_t) (VCMD_LINE_MAX - 1u)) {
        s_overflow = 1u;
        dbg.uart.bad++;
        return 0u;
    }

    s_line[s_len++] = c;
    return 0u;
}

/* ==================== DMA ==================== */

static void rx_arm(void)
{
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
                      (uint32_t) (uintptr_t) &UART_0_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID,
                       (uint32_t) (uintptr_t) &s_rx[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, (uint32_t) VCMD_RX_SIZE);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}

void VCMD_Init(void)
{
    s_len      = 0u;
    s_overflow = 0u;
    s_rd       = 0u;
    s_seen     = 0u;
    s_full     = 0u;

    /* 先把可能残留的通道停掉再重新武装 —— 初始化期间串口上可能有杂波，
     * 脏数据会在这里被丢掉。 */
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    rx_arm();
}

void VCMD_Tick(void)
{
    uint16_t n;

    if (DL_DMA_isChannelEnabled(DMA, DMA_CH0_CHAN_ID) != false) {
        /* 还在收：已写入字节数 = 总长 − 剩余 */
        n = (uint16_t) (VCMD_RX_SIZE -
                        DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID));
        if (n <= s_rd) {
            return;              /* 没有新字节，这是平时唯一的花费 */
        }
    } else {
        /* 通道已停 = 缓冲收满 */
        n      = (uint16_t) VCMD_RX_SIZE;
        s_full = 1u;
    }

    /* ⚠ 统计用的是 s_seen 而不是 s_rd：下面一圈只处理一条命令，
     * s_rd 会分好几圈才追上 n，用 s_rd 算会把同一批字节重复累加。 */
    if (n > s_seen) {
        dbg.uart.rx_bytes += (uint32_t) (n - s_seen);
        s_seen = n;
    }

    /* ⚠ 一圈最多执行一条命令。
     * 邮箱只有一个槽位，同一圈里连续提交多条，前面的会被后面的直接覆盖 ——
     * 症状是"粘贴一串命令，只有最后一条生效"，而且 cmds 还在正常增长，
     * 极难察觉。一条一圈 = 最多 21.5 ms 延迟，人手完全感觉不到。 */
    while (s_rd < n) {
        if (vcmd_feed(s_rx[s_rd++]) != 0u) {
            break;               /* 这条执行完了，剩下的下一圈继续 */
        }
    }

    /* 只有"缓冲读空 + DMA 已收满"才重新武装。
     * 没读空时让 DMA 继续往同一个缓冲里写，避免中途重装丢字节。 */
    if ((s_rd >= n) && (s_full != 0u)) {
        s_rd   = 0u;
        s_seen = 0u;
        s_full = 0u;
        rx_arm();
    }
}
