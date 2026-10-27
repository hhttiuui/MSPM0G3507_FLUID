/**
 * @file vofa.c
 * @brief 串口上行遥测（FireWater + UART1 TX DMA，设计说明见 vofa.h）
 *
 * 数据流：
 *   dbg.* → frame_build() 拼进 s_frame[] → DMA_CH1 搬进 UART1 TXDATA
 *          → 立刻返回，CPU 不参与搬运
 */

#include "vofa.h"
#include <stdbool.h>
#include "debug.h"
#include "ti_msp_dl_config.h"

/* 一帧最长字节数。8 通道 × 最多 11 字符 + 7 个逗号 + '\n' ≈ 96。
 * 115200 下 96 字节 = 8.3 ms，仍短于一帧物理（21.5 ms）。 */
#define VOFA_FRAME_MAX      (96u)

/* DMA 看门狗：一帧最长 96 字节，115200 下也就 8.3 ms（不到半帧）。
 * 超过 8 圈（约 170 ms）还挂着，说明通道卡住了（UART 的 DMA 触发丢了），
 * 强制释放 —— 不释放的话后面每一帧都会被判"忙"而丢掉，上报永久停摆。 */
#define VOFA_STUCK_FRAMES   (8u)

static char     s_frame[VOFA_FRAME_MAX];
static uint16_t s_mask;
static uint8_t  s_div_shift;
static uint8_t  s_div_cnt;
static uint8_t  s_busy;
static uint8_t  s_busy_frames;

/* ==================== 整数转文本 ====================
 * 不用 printf：会拖进整个格式化库（约 3 KB Flash），而且这里只需要十进制整数。 */
static const uint32_t s_pow10[10] = {
    1u, 10u, 100u, 1000u, 10000u,
    100000u, 1000000u, 10000000u, 100000000u, 1000000000u
};

static char *put_i32(char *p, int32_t v)
{
    uint32_t u;
    int8_t   k = 9;

    if (v < 0) {
        *p++ = '-';
        u = (uint32_t) (-(v + 1)) + 1u;   /* 这样写 INT32_MIN 也不会溢出 */
    } else {
        u = (uint32_t) v;
    }

    while ((k > 0) && (u < s_pow10[k])) {
        k--;
    }

    for (; k >= 0; k--) {
        uint32_t d = 0u;

        while (u >= s_pow10[k]) {
            u -= s_pow10[k];
            d++;
        }
        *p++ = (char) ('0' + d);
    }
    return p;
}

/* ==================== 拼帧 ==================== */
static uint16_t frame_build(void)
{
    char   *p = &s_frame[0];
    uint8_t sent = 0u;
    uint8_t ch;

    for (ch = 0u; ch < 16u; ch++) {
        int32_t v;

        if ((s_mask & (uint16_t) (1u << ch)) == 0u) {
            continue;
        }

        switch (ch) {
            case VOFA_CH_SIM:      v = (int32_t) dbg.fluid.sim_ms_x10;   break;
            case VOFA_CH_FPS:      v = (int32_t) dbg.fluid.fps;          break;
            case VOFA_CH_AX10:     v = dbg.fluid.ax10;                   break;
            case VOFA_CH_AY10:     v = dbg.fluid.ay10;                   break;
            case VOFA_CH_LIT:      v = (int32_t) dbg.fluid.dens_lit;     break;
            case VOFA_CH_BAT_MV:   v = (int32_t) dbg.bat.mv;             break;
            case VOFA_CH_SHAKE:    v = (int32_t) dbg.fluid.shake_pk10;   break;
            case VOFA_CH_DENS_MAX: v = (int32_t) dbg.fluid.dens_max;     break;
            /* 场景号：切换后这一路立刻变，是"按键到底有没有生效"最直接的证据 */
            case VOFA_CH_SCENE:    v = (int32_t) dbg.app.scene;          break;
            /* 旋转方向：确认自动判定到底转没转、转对没转 */
            case VOFA_CH_ROT:      v = (int32_t) dbg.app.rot;            break;
            /* 刹车高亮：标定时看它跟不跟 val_mg 走 */
            case VOFA_CH_BRAKE:    v = (int32_t) dbg.brake.active;       break;
            /* 两个轴的原始加速度 —— 选轴 / 定符号就看这两路 */
            case VOFA_CH_RAWX:     v = (int32_t) dbg.brake.rawx_mg;      break;
            case VOFA_CH_RAWY:     v = (int32_t) dbg.brake.rawy_mg;      break;
            /* 减速度：正 = 正在减速。阈值就是拿它比的 */
            case VOFA_CH_DECEL:    v = (int32_t) dbg.brake.val_mg;       break;
            /* 法线轴原始值 + 基线：标定默认轴就看这两路 */
            case VOFA_CH_RAWZ:     v = (int32_t) dbg.brake.rawz_mg;      break;
            default:               v = (int32_t) dbg.brake.base_mg;      break;
        }

        if (sent != 0u) {
            *p++ = ',';
        }
        p = put_i32(p, v);
        sent++;
    }

    /* 一个通道都没开：发一个空行，VOFA+ 会当成"0 通道"忽略掉 */
    *p++ = '\n';

    return (uint16_t) (p - &s_frame[0]);
}

/* ==================== DMA 启动 ====================
 * DMA_CH1 在 syscfg 里已经配好（触发源 = UART1 TX，源自增、目的固定、字节宽），
 * 这里只需要每次给新的源地址和长度，然后 enable。
 * 传输完成后硬件会自己把通道的 EN 位清掉，所以忙闲判断就是读 EN。 */
static void dma_start(uint16_t len)
{
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID,
                      (uint32_t) (uintptr_t) &s_frame[0]);
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID,
                       (uint32_t) (uintptr_t) &UART_0_INST->TXDATA);
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, (uint32_t) len);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);

    dbg.uart.tx_frames++;
}

/* ==================== 对外接口 ==================== */

void VOFA_Init(void)
{
    s_div_cnt     = 0u;
    s_div_shift   = VOFA_DIV_OFF;
    s_mask        = VOFA_CH_DEFAULT;
    s_busy        = 0u;
    s_busy_frames = 0u;

    dbg.uart.div_shift = s_div_shift;
    dbg.uart.mask      = s_mask;
}

void VOFA_SetDivShift(uint32_t n)
{
    if (n > 7u) {
        n = 7u;                 /* 1/128 → 0.36 Hz，再低就没意义了 */
    }

    s_div_shift = (uint8_t) n;
    s_div_cnt   = 0u;

    dbg.uart.div_shift = s_div_shift;
}

void VOFA_SetMask(uint32_t mask)
{
    s_mask = (uint16_t) (mask & 0xFFFFu);

    dbg.uart.mask = s_mask;
}

void VOFA_Tick(void)
{
    uint16_t len;

    /* ---- DMA 看门狗：即使上报关着也要把通道收干净 ----
     * 每圈都查，不跟分频走：一旦通道卡住，分频点可能很久才来一次。 */
    if (s_busy != 0u) {
        if (DL_DMA_isChannelEnabled(DMA, DMA_CH1_CHAN_ID) == false) {
            s_busy = 0u;            /* 正常搬完 */
        } else if (++s_busy_frames >= (uint8_t) VOFA_STUCK_FRAMES) {
            DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
            s_busy = 0u;
            dbg.uart.tx_stuck++;
        }
    }

    if (s_div_shift == (uint8_t) VOFA_DIV_OFF) {
        return;                     /* 关着的时候一点开销都不花 */
    }

    /* 分频：每 (1 << s_div_shift) 圈发一帧 */
    if ((s_div_cnt & (uint8_t) ((1u << s_div_shift) - 1u)) != 0u) {
        s_div_cnt++;
        return;
    }
    s_div_cnt++;

    /* 上一帧还没搬完就丢掉这一帧 —— 绝不等待，绝不能拖慢主循环。
     * dbg.uart.tx_drops 持续增长说明波特率不够 / 通道太多 / 分频太小。 */
    if (s_busy != 0u) {
        dbg.uart.tx_drops++;
        return;
    }

    len = frame_build();
    dma_start(len);

    s_busy        = 1u;
    s_busy_frames = 0u;
}
