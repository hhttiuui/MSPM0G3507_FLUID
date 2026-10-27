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
#include "clock.h"          /* tick_ms：二进制半包超时用 */
#include "app.h"            /* App_ShowDirectFrame() */
#include "glyph_store.h"    /* 自定义滚动字形：接收 → 校验 → 存 Flash */
#include "scenes.h"         /* FrameScene_StoreBegin/Data/End（图片槽）*/
#include "ti_msp_dl_config.h"

/* 一行最长字节数。
 * 要放得下 "text 室内温度25℃"（5 + 3×6 = 23 字节），所以给到 64。
 * 再长就判为无效行（bad++），不会截断成半句去执行。 */
#define VCMD_LINE_MAX       (64u)

/* 带字符串命令（目前只有 text）的参数缓冲。
 * 4 个 int32 装不下 UTF-8 字符串，所以单独存一份，命令只投递 id。 */
#define VCMD_STR_MAX        (64u)

/* RX DMA 一次武装能收多少字节。
 * ⚠ 115200 下 512 字节 = 44 ms，比一帧（21.5 ms）长一倍多，所以：
 *    · 文本命令：一次粘贴 512 B 以内不会溢出（原先是 128）
 *    · 二进制帧：一整包 246 B（6 B 头 + 240 B 数据）能**一次装下**，
 *      不会被 DMA 缓冲边界切断。这是单帧直显能工作的前提。
 *    · 连续快速发多帧时仍可能丢字节（DMA 满了以后到达的会掉进 4 级 FIFO），
 *      所以上位机每发一包应当留 ≥ 50 ms 间隔。 */
#define VCMD_RX_SIZE        (512u)

static char     s_rx[VCMD_RX_SIZE];
static char     s_line[VCMD_LINE_MAX];
static uint8_t  s_len;
static uint8_t  s_overflow;      /* 当前行超长，丢弃到下一个换行 */
static uint16_t s_rd;            /* 已消费到 s_rx 的哪个位置 */
static uint16_t s_seen;          /* 已计入 rx_bytes 统计的位置（≠ s_rd）*/
static uint8_t  s_full;          /* RX DMA 已收满并自动停下 */

/* 最近一次带字符串命令的参数（见 VCMD_GetString 的注释） */
static char     s_str[VCMD_STR_MAX];

/* ==================== 二进制帧（上位机直显）====================
 *
 * 首字节 0xA5 用来和文本命令区分。但**只在行首**才认它（s_len == 0）——
 * 因为 0xA5 同时也是 UTF-8 汉字的中间字节："好" = E5 A5 BD。
 * 如果整行都在找包头，`text 你好世界` 会在"好"的第二个字节处被拦腰截断，
 * 后半行被当成二进制包吞掉（踩过：发"你好世界"显示成"你口de 2"）。
 * 文本命令都以可打印 ASCII 开头，所以"行首 0xA5"不会和文本冲突。
 *
 * 包格式（和 pc/index.html 的"帧数据"完全一致）：
 *   [0] 0xA5  magic
 *   [1] 1     版本
 *   [2] flags bit[1:0] 位深（0 = 8bit，1 = 1bit）
 *   [3] 16    行数
 *   [4] 15    列数
 *   [5] len   负载字节数（8bit = 240，1bit = 32）
 *   [6..] 负载
 *
 * 8bit：240 B，行优先 idx = row×15 + col，值 = 亮度 0~255
 * 1bit：32 B = 16 行 × 2 B；每行 byte0 装 x1~x8（bit7 = x1），
 *       byte1 装 x9~x15（bit7 = x9，bit0 恒 0）
 *
 * ⚠ 没有 CRC：单帧直显属于"发一次、看一眼"，丢包就重发，不值得加校验。
 *   等做 Flash 槽上传（多帧、不可重来）时再加 CRC16。
 */
#define VCMD_BIN_MAGIC      (0xA5u)
#define VCMD_BIN_HDR        (6u)
/* 6 B 头 + 最多 400 B 负载。
 * 400 是给"字形条带"留的：10 个 12×12 字形 ≈ 10 × (2 + 24) = 260 B。
 * ⚠ 不要超过 RX 缓冲（512 B），否则一个包会跨 DMA 重装边界、丢字节。 */
#define VCMD_BIN_MAX        (VCMD_BIN_HDR + 400u)
#define VCMD_BIN_TIMEOUT_MS (200u)      /* 半包超过这么久没后续 → 丢弃 */

static uint8_t  s_bin[VCMD_BIN_MAX];
static uint16_t s_binLen;               /* 0 = 当前不在收包 */
static uint16_t s_binNeed;              /* 0 = 还没解析出总长 */
static uint32_t s_binTick;              /* 收包起始时刻（超时用） */

/* 1 bit 解包用的临时缓冲。**用 static 不用局部**：
 * 240 B 放栈上，在 8 KB 栈里太冒险了。 */
static uint8_t  s_binPx[240];

/* 一个完整二进制包到了：解析 + 显示 */
static void vcmd_bin_dispatch(void)
{
    uint8_t ver   = s_bin[1];
    uint8_t flags = s_bin[2];
    uint8_t rows  = s_bin[3];
    uint8_t cols  = s_bin[4];
    uint8_t len   = s_bin[5];
    uint8_t bpp   = (uint8_t) (flags & 0x03u);
    const uint8_t *src = &s_bin[VCMD_BIN_HDR];

    /* ---- 图片/动图槽：三包协议（阶段 3）----
     * BEGIN : A5|5|slot|frames.lo|frames.hi|len=4| bpp, fps.lo, fps.hi, gain
     * DATA  : A5|6|seq |0   |0    |len   | 帧数据（len 必须是 8 的倍数）
     * END   : A5|7|0   |0   |0    |len=2 | crc.lo, crc.hi
     *
     * 和字形三包一个套路：先擦头（槽失效）→ 边收边写 → CRC 通过才写头。
     * DATA 一包一帧最省事：8bit = 240 B，4bit = 120 B，都 ≤ 248 且是 8 的倍数。
     */
    if ((ver >= 5u) && (ver <= 7u)) {
        if (ver == 5u) {                        /* BEGIN */
            uint16_t frames;
            uint16_t fps;

            if (len != 4u) {
                dbg.uart.bad++;
                return;
            }

            frames = (uint16_t) ((uint16_t) s_bin[3] | ((uint16_t) s_bin[4] << 8));
            fps    = (uint16_t) ((uint16_t) s_bin[7] | ((uint16_t) s_bin[8] << 8));

            if (FrameScene_StoreBegin(s_bin[2], s_bin[6], frames, fps, s_bin[9]) == 0u) {
                dbg.uart.bad++;
            } else {
                dbg.uart.bin_ok++;
            }
            return;
        }

        if (ver == 6u) {                        /* DATA */
            if (FrameScene_StoreData(src, len) == 0u) {
                dbg.uart.bad++;
            } else {
                dbg.uart.bin_ok++;
            }
            return;
        }

        /* ver == 7：END */
        {
            uint16_t crc = (uint16_t) ((uint16_t) s_bin[6] |
                                       ((uint16_t) s_bin[7] << 8));

            if ((len != 2u) || (FrameScene_StoreEnd(crc) == 0u)) {
                dbg.uart.bad++;
            } else {
                dbg.uart.bin_ok++;
                dbg.uart.cmds++;
            }
        }
        return;
    }

    /* ---- 自定义滚动字形：三包协议（接收 → 校验 → 存 Flash）----
     * BEGIN : A5|2|count|spacing|flags|len=4| step, ms.lo, ms.hi, lev
     * GLYPH : A5|3|idx  |w    |h    |len  | bitmap
     * END   : A5|4|0    |0    |0    |len=2| crc.lo, crc.hi
     *
     * 三包分开是因为 8 位字形一个字就 256 B，一包装不下；
     * 而且 MCU 可以在 GLYPH 阶段就做长度校验，不用先攒完再看。
     */
    if ((ver == 2u) || (ver == 3u) || (ver == 4u)) {
        if (ver == 2u) {                        /* BEGIN */
            uint8_t  step, lev;
            uint16_t ms;

            if ((len < 4u) || (s_bin[2] == 0u)) {
                dbg.uart.bad++;
                return;
            }

            step = s_bin[6];
            ms   = (uint16_t) ((uint16_t) s_bin[7] | ((uint16_t) s_bin[8] << 8));
            lev  = s_bin[9];

            if (GlyphStore_Begin(s_bin[2], (int8_t) s_bin[3], step, ms, lev,
                                 (uint8_t) (s_bin[4] & 0x01u)) == 0u) {
                dbg.uart.bad++;
            }
            return;
        }

        if (ver == 3u) {                        /* GLYPH */
            uint8_t w    = s_bin[3];
            uint8_t h    = s_bin[4];
            uint8_t roww = (uint8_t) ((w + 7u) / 8u);
            uint8_t wh[2];

            if ((w == 0u) || (w > 120u) || (h == 0u) || (h > 16u) ||
                (len != (uint8_t) (roww * h))) {
                dbg.uart.bad++;
                return;
            }

            wh[0] = w;
            wh[1] = h;

            /* 按 [w][h][bitmap] 的顺序进缓冲，CRC 顺序也就跟着对了 */
            if ((GlyphStore_Append(wh, 2u) == 0u) ||
                (GlyphStore_Append(src, (uint16_t) len) == 0u)) {
                dbg.uart.bad++;
            }
            return;
        }

        /* ver == 4：END —— 校验 CRC → 落盘 → 立刻播放 */
        {
            uint16_t crc = (uint16_t) ((uint16_t) s_bin[6] |
                                       ((uint16_t) s_bin[7] << 8));

            if ((len != 2u) || (GlyphStore_CrcOk(crc) == 0u) ||
                (GlyphStore_Commit() == 0u) ||
                (App_ApplyStoredGlyphs() == 0u)) {
                dbg.uart.bad++;
            } else {
                dbg.uart.bin_ok++;
                dbg.uart.cmds++;
            }
        }
        return;
    }

    if ((ver != 1u) || (rows != 16u)) {
        dbg.uart.bad++;
        return;
    }

    /* ---- 条带（flags bit2）：cols 可变，MCU 自己横向滚动 ----
     * 1 bit：负载 = ceil(cols/8) × 16 字节，行优先、MSB = 最左。
     *
     * ⚠ 宽度上限 120 px 是帧头算出来的，不是随便定的：帧头 [5] 只有 1 字节
     *   长度，ceil(120/8)=15，15×16=240 ≤ 255 装得下；121 px 就变 16×16=256
     *   溢出成 0。再宽必须把 [5] 扩成 2 字节（协议改动）。 */
    if ((flags & 0x04u) != 0u) {
        uint16_t roww = (uint16_t) (((uint16_t) cols + 7u) / 8u);

        if ((cols == 0u) || (cols > 120u) || (bpp != 1u) ||
            ((uint16_t) len != (uint16_t) (roww * 16u))) {
            dbg.uart.bad++;
            return;
        }

        if (App_ShowStrip(src, (uint16_t) cols) == 0u) {
            dbg.uart.bad++;
        } else {
            dbg.uart.bin_ok++;
            dbg.uart.cmds++;
        }
        return;
    }

    if (cols != 15u) {
        dbg.uart.bad++;
        return;
    }

    if ((bpp == 0u) && (len == 240u)) {
        App_ShowDirectFrame(src);
        dbg.uart.bin_ok++;
        dbg.uart.cmds++;
        return;
    }

    if ((bpp == 1u) && (len == 32u)) {
        uint8_t r, i;

        for (r = 0u; r < 16u; r++) {
            uint8_t b0 = src[(uint16_t) r * 2u];
            uint8_t b1 = src[(uint16_t) r * 2u + 1u];

            for (i = 0u; i < 8u; i++) {             /* x1..x8 */
                s_binPx[(uint16_t) r * 15u + i] =
                    (((b0 >> (uint8_t) (7u - i)) & 1u) != 0u) ? 0xFFu : 0u;
            }
            for (i = 0u; i < 7u; i++) {             /* x9..x15 */
                s_binPx[(uint16_t) r * 15u + 8u + i] =
                    (((b1 >> (uint8_t) (7u - i)) & 1u) != 0u) ? 0xFFu : 0u;
            }
        }

        App_ShowDirectFrame(s_binPx);
        dbg.uart.bin_ok++;
        dbg.uart.cmds++;
        return;
    }

    dbg.uart.bad++;                                 /* 位深/长度不认识 */
}

/* ---- 别名表：手打方便。命令号见 debug.h 的 DBG_CMD_* ----
 * 顺序无所谓，查找是线性扫描（只在收到命令时跑一次，几微秒）。 */
typedef struct {
    const char *name;
    uint16_t    id;
    uint8_t     str;        /* 1 = 命令名后面整段是字符串参数，存进 s_str */
} vcmd_alias_t;

static const vcmd_alias_t s_alias[] = {
    /* ---- 系统 ---- */
    { "div",    DBG_CMD_SET_DIV, 0 },   /* div 0~7       遥测分频 */
    { "mask",   DBG_CMD_SET_MASK, 0 },   /* mask 15       通道掩码 */
    { "reset",  DBG_CMD_RESET_STATS, 0 },   /* reset         清零统计 */
    { "ftest",  DBG_CMD_FLASH_TEST,  0 },   /* ftest         Flash 擦/写/读回自检 */

    /* ---- 流体 ---- */
    { "dt",     DBG_CMD_FLUID_SET_DT, 0 },   /* dt 3200       dt = 0.032 s */
    { "iter",   DBG_CMD_FLUID_SET_ITER, 0 },   /* iter 12       压力迭代 */
    { "shake",  DBG_CMD_FLUID_SET_SHAKE, 0 },   /* shake 150     摇晃增益 1.5 */
    { "pause",  DBG_CMD_FLUID_PAUSE, 0 },   /* pause 1       暂停物理 */
    { "reinit", DBG_CMD_FLUID_REINIT, 0 },   /* reinit        粒子归位 */
    { "full",   DBG_CMD_FLUID_SET_FULL, 0 },   /* full 60       满亮度密度 = 静止密度×60% */
    { "edge",   DBG_CMD_FLUID_EDGE, 0 },   /* edge 15 50    边缘分段映射（抗锯齿）*/

    /* ---- LED 点阵（亮度标定）---- */
    { "bc",     DBG_CMD_LED_SET_BC, 0 },   /* bc 128        全局亮度 0x37 */
    { "cc",     DBG_CMD_LED_SET_CC, 0 },   /* cc 7          恒流档 0x36 */
    { "fill",   DBG_CMD_LED_FILL, 0 },   /* fill 200      整屏亮度 */
    { "off",    DBG_CMD_LED_OFF, 0 },   /* off           全灭 */
    { "px",     DBG_CMD_LED_PX, 0 },   /* px 3 5 200    单点 */
    { "ramp",   DBG_CMD_LED_RAMP, 0 },   /* ramp          14 级线性阶梯（标定用）*/
    { "rampg",  DBG_CMD_LED_RAMP_GAMMA, 0 },   /* rampg         14 级 gamma 阶梯（看观感）*/
    { "lock",   DBG_CMD_LED_LOCK, 0 },   /* lock 1        冻结流体渲染 */
    { "gam",    DBG_CMD_LED_GAMMA, 0 },   /* gam 1         流体渲染走 gamma2.2 */
    { "pat",    DBG_CMD_LED_PAT,   0 },   /* pat           方向测试图 */

    /* 场景切换 */
    { "mode",   DBG_CMD_APP_MODE, 0 },   /* mode 1        切到指定场景 */
    { "next",   DBG_CMD_APP_NEXT, 0 },   /* next          下一个场景 */
    { "prev",   DBG_CMD_APP_PREV, 0 },   /* prev          上一个场景 */

    /* ---- 场景 2：文字滚动 ---- */
    { "text",   DBG_CMD_TEXT_SET,  1 },   /* text 你好世界  换文字（带字符串）*/
    { "tstep",  DBG_CMD_TEXT_STEP, 0 },   /* tstep 1       每帧移动像素 1~8 */
    { "tms",    DBG_CMD_TEXT_MS,   0 },   /* tms 40        帧间隔 ms */
    { "tlev",   DBG_CMD_TEXT_LEV,  0 },   /* tlev 255      文字亮度 */
    { "tpos",   DBG_CMD_TEXT_POS,  0 },   /* tpos 30       跳到滚动位置（定格看）*/
    { "torient",DBG_CMD_TEXT_ORIENT,0 },  /* torient 4     文字方向：0~3 固定 / 4 自动 */

    /* ---- 文字预设（存 Flash，PC 端是主、板子是副本）---- */
    { "tpclr",  DBG_CMD_TEXT_TPCLR, 0 },  /* tpclr         清空预设区 */
    { "tpadd",  DBG_CMD_TEXT_TPADD, 0 },  /* tpadd         把当前文字+参数追加成一条 */
    /* ⚠ 第三列是 "整段当字符串" 的标志，不是参数个数！
     *   写成 1 会让 vcmd 把行尾当字符串塞进 s_str，数字参数全部丢失 ——
     *   tinv/tpload 一开始就是这么写错的：arg[0] 恒为 0，反色永远关着、
     *   tpload 永远跳到第 0 条。数字类命令一律写 0（对照 tlev/torient）。 */
    { "tpload", DBG_CMD_TEXT_TPLOAD,0 },  /* tpload 2      应用第 2 条预设 */
    { "tinv",   DBG_CMD_TEXT_INV,   0 },  /* tinv 1        文字反色（亮底黑字）*/

    /* ---- 几何动画（场景 4）---- */
    { "ganim",  DBG_CMD_GEOM_ANIM,  0 },  /* ganim 1       0 弹跳球 / 1 旋转立方体 */
    { "gtail",  DBG_CMD_GEOM_TAIL,  0 },  /* gtail 2       拖尾 0~8，0 = 不拖尾 */
    { "gball",  DBG_CMD_GEOM_BALL,  0 },  /* gball 3       球半径 1~6 像素 */
    { "gkick",  DBG_CMD_GEOM_KICK,  0 },  /* gkick 3       碰边弹起速度 1~8 px/帧 */
    { "gdamp",  DBG_CMD_GEOM_DAMP,  0 },  /* gdamp 2       水平阻尼 0~8，0 = 关 */

    /* ---- 刹车高亮（brake.c）---- */
    { "brk",     DBG_CMD_BRAKE_ENABLE, 0 },  /* brk 1          刹车检测开关 */
    { "brkf",    DBG_CMD_BRAKE_FORCE,  0 },  /* brkf 1         强制刹车态（演示/拍摄）*/
    { "brkthr",  DBG_CMD_BRAKE_THR,    0 },  /* brkthr 150     触发阈值 mg（30~800）*/
    { "brkax",   DBG_CMD_BRAKE_AXIS,   0 },  /* brkax 0        0=屏幕 x / 1=屏幕 y */
    { "brksgn",  DBG_CMD_BRAKE_SIGN,   0 },  /* brksgn -1      行进方向对应哪个符号 */
    { "brkdisp", DBG_CMD_BRAKE_DISP,   0 },  /* brkdisp 0      0 全亮 / 1 下半条 */
    { "brkmode", DBG_CMD_BRAKE_MODE,   0 },  /* brkmode 1      0 幅值(默认) / 1 变化率边沿 */
    { "brkdthr", DBG_CMD_BRAKE_DTHR,   0 },  /* brkdthr 1000   变化率阈值 mg/s */
    { "brkhold", DBG_CMD_BRAKE_HOLD,   0 },  /* brkhold 1200   边沿触发后保持 ms */
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

/* 在别名表里查 nlen 个字符。命中返回**下标**（要拿 str 标志），否则 -1。
 * 返回下标而不是命令号，是因为 text 这种命令还要知道"后面是不是字符串参数"。 */
static int8_t find_alias(const char *p, uint8_t nlen)
{
    uint8_t i;

    for (i = 0u; i < (uint8_t) VCMD_ALIAS_N; i++) {
        const char *n = s_alias[i].name;
        uint8_t k = 0u;

        while ((n[k] != '\0') && (k < nlen) && (n[k] == p[k])) {
            k++;
        }
        if ((k == nlen) && (n[k] == '\0')) {
            return (int8_t) i;
        }
    }
    return -1;
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
    int8_t ai;

    if (*p == '\0') {
        return;                  /* 空行，忽略 */
    }

    /* ---- 先看是不是别名 ---- */
    q = p;
    while ((*q != '\0') && (*q != ' ') && (*q != '=') && (*q != ',')) {
        q++;
    }
    nlen = (uint8_t) (q - p);

    ai = find_alias(p, nlen);

    if (ai < 0) {
        /* ---- 不是别名：把第一段当命令号 ---- */
        int32_t v;

        q = parse_int(p, &v);
        if ((q == 0) || (v < 0) || (v > 0xFFFF)) {
            dbg.uart.bad++;
            return;
        }
        id = (uint16_t) v;
    } else {
        id = s_alias[ai].id;

        /* ---- 带字符串参数的命令（text）----
         * 4 个 int32 装不下 UTF-8，所以把命令名后面**整段**（只跳过分隔符，
         * 中间的空格保留）存进 s_str，命令本身只投递 id。
         * 场景的 cmd() 里用 VCMD_GetString() 取。 */
        if (s_alias[ai].str != 0u) {
            char   *sp = skip_sep(q);
            uint8_t i  = 0u;

            while ((sp[i] != '\0') && (i < (uint8_t) (VCMD_STR_MAX - 1u))) {
                s_str[i] = sp[i];
                i++;
            }
            s_str[i] = '\0';

            DBG_CmdSubmit(id, 0, 0, 0, 0);
            dbg.uart.cmds++;
            return;
        }
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
    /* ---- 二进制包优先（0xA5 开头）----
     * 这里**永远返回 0**：一个包可能在一次 Tick 里就收完，不该打断本圈消费。 */
    if (s_binLen != 0u) {
        if (s_binLen >= (uint16_t) VCMD_BIN_MAX) {
            s_binLen = 0u;              /* 不该发生：总长已校验过 */
            s_binNeed = 0u;
            dbg.uart.bad++;
            return 0u;
        }

        s_bin[s_binLen] = (uint8_t) c;
        s_binLen++;

        /* 凑够 6 字节头就算出总长，顺便校验 */
        if ((s_binNeed == 0u) && (s_binLen >= (uint16_t) VCMD_BIN_HDR)) {
            s_binNeed = (uint16_t) (VCMD_BIN_HDR + s_bin[5]);
            if ((s_binNeed > (uint16_t) VCMD_BIN_MAX) ||
                (s_binNeed <= (uint16_t) VCMD_BIN_HDR)) {
                s_binLen  = 0u;
                s_binNeed = 0u;
                dbg.uart.bad++;
                return 0u;
            }
        }

        if ((s_binNeed != 0u) && (s_binLen >= s_binNeed)) {
            vcmd_bin_dispatch();
            s_binLen  = 0u;
            s_binNeed = 0u;
        }
        return 0u;
    }

    if ((s_len == 0u) && (s_overflow == 0u) &&
        ((uint8_t) c == VCMD_BIN_MAGIC)) {
        s_bin[0]  = (uint8_t) c;
        s_binLen  = 1u;
        s_binNeed = 0u;
        s_binTick = tick_ms;
        return 0u;
    }

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

const char *VCMD_GetString(void)
{
    return &s_str[0];
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
    s_str[0]   = '\0';

    /* 先把可能残留的通道停掉再重新武装 —— 初始化期间串口上可能有杂波，
     * 脏数据会在这里被丢掉。 */
    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    rx_arm();
}

void VCMD_Tick(void)
{
    uint16_t n;

    /* 半包超时：上位机发到一半断了（拔线 / 关页面），把残留丢掉，
     * 否则下一次正常数据首字节会被当成包体，一直错位。 */
    if ((s_binLen != 0u) &&
        ((uint32_t) (tick_ms - s_binTick) > VCMD_BIN_TIMEOUT_MS)) {
        s_binLen  = 0u;
        s_binNeed = 0u;
        dbg.uart.bad++;
    }

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
