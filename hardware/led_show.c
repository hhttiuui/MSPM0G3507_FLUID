/*
 * led_show.c —— 综合演示序列的状态机实现
 *
 * 时间轴用 tick_ms（SysTick，1ms 一拍）做基准，各段时长是 led_show.h 里的编译期常量：
 * 每段都有自己的时长（见 led_show.h）。
 *
 * 并发上的讲究：LedShow_Start()/Stop() 只写**一个字节的标志位**，真正的序列复位
 * 放在 LedShow_Task() 里做 —— 这样以后从串口中断里调 Start() 也不会和外层的
 * Task() 抢那几个 32 位状态量（Cortex-M0+ 上 32 位写不是原子的）。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "debug.h"
#include "power.h"
#include "led_matrix.h"
#include "led_show.h"

static volatile uint8_t s_restartReq;   /* Start() 置位，Task() 里消费 */
static volatile uint8_t s_active;       /* 1 = 在播放 */
static uint8_t  s_stage;                /* 当前段（只在 Task 里改） */
static uint8_t  s_dirty;                /* 1 = 本段还没画 */
static uint32_t s_stageStart;           /* 本段开始的 tick_ms */
static uint32_t s_nextStep;             /* 游走段下一步的 tick_ms */
static uint16_t s_walkIdx;              /* 游走段格号 0..239 */

static void ShowAll(uint8_t on)
{
    LedMatrix_Clear();

    if (on != 0u) {
        LedMatrix_FillAll(1u);          /* 走映射填满，不碰 CA19 那些没有 LED 的槽 */
    }

    LedMatrix_Flush();
}

void LedShow_ShowPattern(void)
{
    uint8_t i;

    LedMatrix_Clear();

    /* 外框：用应用坐标画，所以它必须正好落在面板的整个边框上 */
    for (i = 1u; i <= LED_APP_COLS; i++) {
        LedMatrix_SetCell(i, 1u);
        LedMatrix_SetCell(i, LED_APP_ROWS);
    }
    for (i = 1u; i <= LED_APP_ROWS; i++) {
        LedMatrix_SetCell(1u, i);
        LedMatrix_SetCell(LED_APP_COLS, i);
    }

    /* 主对角线：外框是矩形，转置对不对一眼可见 */
    for (i = 1u; i <= LED_APP_COLS; i++) {
        LedMatrix_SetCell(i, i);
    }

    /* 左上角 2×2 实心块：外框和对角线都左右对称，靠它区分镜像方向。
     * 正确时它应该落在设备的左上角（应用坐标 (1,1)）。 */
    LedMatrix_SetCell(1u, 1u);
    LedMatrix_SetCell(2u, 1u);
    LedMatrix_SetCell(1u, 2u);
    LedMatrix_SetCell(2u, 2u);

    LedMatrix_Flush();
}

void LedShow_Start(void)
{
    s_restartReq = 1u;                  /* 只写一个字节 → 任何上下文调用都安全 */
    s_active     = 1u;
}

void LedShow_Stop(void)
{
    s_active = 0u;
    /* s_stage / s_restartReq 交给 Task() 收尾，避免从这里动 32 位状态 */
}

uint8_t LedShow_IsActive(void)
{
    return s_active;
}

void LedShow_Run(void)
{
    LedMatrix_Init();                   /* 芯片初始化 + 清屏（里面含 HT16D33A_Init） */
    LedShow_Start();

    for (;;) {
        LedShow_Task();                 /* 推进段落 + 刷屏 */

        /* 一键开关机：长按 USER1 后由 TIMA0 中断拉低 PA22，这里只负责停下来 */
        PowerKey_CheckShutdown();

        /* 这里没有别的活，睡到下一个中断（SysTick 1ms）再判断，别空转烧电 */
        __WFI();
    }
}

void LedShow_Task(void)
{
    uint32_t elapsed;

    if (s_restartReq != 0u) {
        s_restartReq       = 0u;
        s_stage            = LED_SHOW_STAGE_ON;
        s_stageStart       = tick_ms;
        s_nextStep         = tick_ms;
        s_walkIdx          = 0u;
        s_dirty            = 1u;
        s_active           = 1u;
        dbg.led.show_stage = LED_SHOW_STAGE_ON;
        dbg.led.step       = 0u;
    }

    if (s_active == 0u) {
        return;
    }

    elapsed = tick_ms - s_stageStart;

    switch (s_stage) {
    case LED_SHOW_STAGE_ON:
        if (s_dirty != 0u) {
            s_dirty = 0u;
            ShowAll(1u);
        }
        if (elapsed >= LED_SHOW_ON_MS) {
            s_stage            = LED_SHOW_STAGE_OFF;
            s_stageStart       = tick_ms;
            s_dirty            = 1u;
            dbg.led.show_stage = LED_SHOW_STAGE_OFF;
        }
        break;

    case LED_SHOW_STAGE_OFF:
        if (s_dirty != 0u) {
            s_dirty = 0u;
            ShowAll(0u);
        }
        if (elapsed >= LED_SHOW_OFF_MS) {
            s_stage            = LED_SHOW_STAGE_WALK;
            s_stageStart       = tick_ms;
            s_nextStep         = tick_ms;
            s_walkIdx          = 0u;
            dbg.led.show_stage = LED_SHOW_STAGE_WALK;
        }
        break;

    case LED_SHOW_STAGE_WALK:
        if ((int32_t) (tick_ms - s_nextStep) >= 0) {
            s_nextStep = tick_ms + LED_SHOW_STEP_MS;

            /* 按映射表的 (行,列) 光栅顺序走一格 */
            LedMatrix_Clear();
            LedMatrix_SetPanel((uint8_t) (s_walkIdx % LED_PANEL_COLS + 1u),
                               (uint8_t) (s_walkIdx / LED_PANEL_COLS + 1u));
            LedMatrix_Flush();

            dbg.led.step = s_walkIdx;

            s_walkIdx++;
            if (s_walkIdx >= (uint16_t) (LED_PANEL_COLS * LED_PANEL_ROWS)) {
                s_stage            = LED_SHOW_STAGE_PATTERN;
                s_stageStart       = tick_ms;
                s_dirty            = 1u;
                dbg.led.show_stage = LED_SHOW_STAGE_PATTERN;
            }
        }
        break;

    case LED_SHOW_STAGE_PATTERN:
    default:
        if (s_dirty != 0u) {
            s_dirty = 0u;
            LedShow_ShowPattern();
        }
        if (elapsed >= LED_SHOW_PATTERN_MS) {
            s_stage            = LED_SHOW_STAGE_ON;     /* 循环回第 1 段 */
            s_stageStart       = tick_ms;
            s_dirty            = 1u;
            dbg.led.show_stage = LED_SHOW_STAGE_ON;
        }
        break;
    }
}
