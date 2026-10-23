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
#include "battery.h"
#include "power.h"
#include "key.h"
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

        /* 低电检查调度。
         * ⚠ 本模式没有实现"熄屏采样窗口"，所以低电保护在这里不会触发；
         *   它是调试模式，生产跑的是流体模式（见 fluid_demo.c）。 */
        Battery_Task();

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

/* ==================== 灰度验证序列（上电自检）==================== */

/* 本次自检是否被要求跳过。
 *
 * ⚠ 这里特意用 Key_TookPress()（"按下"**边沿**）而不是 Key_IsDown()（电平）：
 *   实测本板 USER2(PA24) **一直读成低电平**（dbg.key.raw 恒为 2）。
 *   该脚在 syscfg 里是 DL_GPIO_RESISTOR_NONE —— 只有 USER1 配了内部上拉，
 *   USER2/USER3 靠板上外部上拉，而 USER2 这一路看起来没起作用、脚悬空读低。
 *   用 Key_IsDown 判"跳过"会让自检一进去就立刻退出，现象就是
 *   **"上电根本看不到测试，直接进流体"**（踩过）。
 *   边沿判据天生免疫：卡住的键不会产生新的按下边沿。
 *   三个键都要读一遍，把边沿标志清干净，免得留给后面的程序。 */
static uint8_t GraySkipRequested(void)
{
    uint8_t hit = 0u;

    if (Key_TookPress(KEY_USER1) != 0u) { hit = 1u; }
    if (Key_TookPress(KEY_USER2) != 0u) { hit = 1u; }
    if (Key_TookPress(KEY_USER3) != 0u) { hit = 1u; }

    return hit;
}

/* 等 ms 毫秒。用 tick_ms 计时（SysTick 1ms 一拍），期间 __WFI 省电。
 * 返回 1 = 用户按了键要求跳过；0 = 正常等满。 */
static uint8_t GrayWait(uint32_t ms)
{
    uint32_t t0 = tick_ms;

    while ((uint32_t) (tick_ms - t0) < ms) {
        if (GraySkipRequested() != 0u) {
            return 1u;
        }
        __WFI();
    }
    return 0u;
}

/* 亮度阶梯：应用坐标的 2..15 行铺 14 级台阶（0 → 255），四周再套一圈全亮外框。
 *
 * 为什么必须套外框（这是补的，第一版忘了写）：
 *   面板一共 16 列，台阶只占 14 列的话，剩下 2 列**一直是灭的** ——
 *   看上去就像坏点或者映射错。套上外框就没有"来路不明的黑边"了，
 *   顺带还再验一次映射能画到面板最边上一列/一行。
 *
 * 台阶分母取 LED_APP_ROWS-3 = 13，保证最暗那级正好 0、最亮那级正好 255
 * （两端都取到端点，好判断灰度是否真的是 8 位而不是两态）。
 *
 * 判读：应当看到"外框全亮 + 内部一条由暗到亮的连续渐变"。
 *   · 内部看不出台阶、或整屏一样亮 → 灰度没生效（还在二进制模式）
 *   · 渐变方向反了 → 只是行列方向问题，映射本身没错，不影响使用 */
void LedShow_ShowGrayRamp(void)
{
    uint8_t x, y;

    LedMatrix_Clear();

    /* ① 内部 14 级台阶 */
    for (y = 2u; y <= (uint8_t) (LED_APP_ROWS - 1u); y++) {
        uint8_t lv = (uint8_t) (((uint32_t) (y - 2u) * (uint32_t) LED_MATRIX_MAX_LEVEL)
                                / (uint32_t) (LED_APP_ROWS - 3u));

        for (x = 1u; x <= LED_APP_COLS; x++) {
            LedMatrix_SetCellLevel(x, y, lv);
        }
    }

    /* ② 外框：全亮度，盖在台阶上（这样 16 列/16 行全都画到了）*/
    for (x = 1u; x <= LED_APP_COLS; x++) {
        LedMatrix_SetCellLevel(x, 1u, LED_MATRIX_MAX_LEVEL);
        LedMatrix_SetCellLevel(x, (uint8_t) LED_APP_ROWS, LED_MATRIX_MAX_LEVEL);
    }
    for (y = 1u; y <= LED_APP_ROWS; y++) {
        LedMatrix_SetCellLevel(1u, y, LED_MATRIX_MAX_LEVEL);
        LedMatrix_SetCellLevel((uint8_t) LED_APP_COLS, y, LED_MATRIX_MAX_LEVEL);
    }

    LedMatrix_Flush();
}

/* 单点游走一遍：按面板 (列,行) 光栅顺序，每次只亮一颗，全亮度。
 * 这是最藏不住映射错误的一项 —— 顺序错、错位、跳行都会直接看出来。
 * 返回 1 = 用户要求跳过。 */
static uint8_t GrayWalk(void)
{
    uint16_t i;
    const uint16_t total = (uint16_t) (LED_PANEL_COLS * LED_PANEL_ROWS);

    for (i = 0u; i < total; i++) {
        LedMatrix_Clear();
        LedMatrix_SetPanel((uint8_t) (i % LED_PANEL_COLS + 1u),
                           (uint8_t) (i / LED_PANEL_COLS + 1u));
        LedMatrix_Flush();
        dbg.led.step = i;

        if (GrayWait(LED_GRAY_TEST_STEP_MS) != 0u) {
            return 1u;
        }
    }
    return 0u;
}

void LedShow_GrayTest(void)
{
    uint32_t t0;

    LedMatrix_Init();       /* 芯片初始化：按当前模式（现在是灰度）配置 + 清屏 */

    t0 = tick_ms;
    while ((uint32_t) (tick_ms - t0) < LED_GRAY_TEST_HOLD_MS) {
        ShowAll(1u);                                    /* ① 全亮 */
        if (GrayWait(LED_GRAY_TEST_ON_MS) != 0u) { break; }

        ShowAll(0u);                                    /* ② 全灭 */
        if (GrayWait(LED_GRAY_TEST_OFF_MS) != 0u) { break; }

        LedShow_ShowPattern();                          /* ③ 方向图案（全亮度）*/
        if (GrayWait(LED_GRAY_TEST_PAT_MS) != 0u) { break; }

        LedShow_ShowGrayRamp();                         /* ④ 亮度阶梯 */
        if (GrayWait(LED_GRAY_TEST_RAMP_MS) != 0u) { break; }

        if (GrayWalk() != 0u) { break; }                /* ⑤ 单点游走 */
    }

    ShowAll(0u);            /* 交出去之前先灭屏，别留一帧残影 */
}
