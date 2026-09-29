/*
 * clock_selftest.c —— CPUCLK 80MHz 实测自检
 *
 * 原理：MSPM0 内置 FCC（Frequency Clock Counter）可以"用一段时间窗口去数
 *       某个时钟的周期数"。窗口由触发源（这里选 LFCLK = 32.768kHz）的
 *       N 个周期决定：
 *
 *           count = f_target × N / 32768
 *
 *       取 N = 8（DL_SYSCTL_FCC_TRIG_CNT_08），则
 *
 *           f_target = count × 4096        ← 纯移位，无除法误差
 *
 *       分别测三路：
 *         MCLK        = 就是 CPUCLK 本身      期望 80000000 Hz → count ≈ 19531
 *         SYSPLLCLK2X = MCLK 的来源           期望 80000000 Hz → count ≈ 19531
 *         SYSOSC      = PLL 的参考源          期望 32000000 Hz → count ≈  7812
 *       并计算 SYSPLLCLK2X / SYSOSC × 1000，期望 2500（即 2.5 倍）。
 *
 * 交叉验证：PA7(LED) 以 1Hz 翻转。它的计时来自 SysTick 的 1ms 中断，
 *       而 SysTick 直接跑在 CPUCLK 上（SysTick_Init 里选了 CLKSOURCE=CPU），
 *       所以用示波器量方波周期应为 1.000s —— 若 CPUCLK 其实是 32MHz，
 *       会变成 2.5s，肉眼都能看出来。
 */
#include "ti_msp_dl_config.h"
#include "clock.h"
#include "oled_hardware_i2c.h"
#include "clock_selftest.h"

/* FCC 计数窗口：N 个 LFCLK 周期 */
#define CLKST_FCC_WINDOW      DL_SYSCTL_FCC_TRIG_CNT_08
#define CLKST_FCC_WINDOW_N    8u
/* f_target(Hz) = count × 32768 / N ；N=8 时即 ×4096 */
#define CLKST_HZ_PER_COUNT    (32768u / CLKST_FCC_WINDOW_N)

/* 板载 LED：GPIO8 PIN_1 = PA7（SysConfig 里已配成输出） */
#define CLKST_LED_PORT        LED_PORT
#define CLKST_LED_PIN         LED_PIN_1_PIN

/* 80MHz 时 count 的期望值，仅用于注释/对照 */
/* #define CLKST_EXPECT_CNT_MCLK   (19531u) */

static uint32_t s_mclkCnt;
static uint32_t s_pll2xCnt;
static uint32_t s_sysoscCnt;

/* 用 FCC 数一个时钟源在 N 个 LFCLK 周期内的周期数 */
static uint32_t ClockSelfTest_CountClock(DL_SYSCTL_FCC_CLOCK_SOURCE src)
{
    uint32_t timeout = 0u;

    DL_SYSCTL_setFCCPeriods(CLKST_FCC_WINDOW);
    DL_SYSCTL_configFCC(DL_SYSCTL_FCC_TRIG_TYPE_RISE_RISE,
                        DL_SYSCTL_FCC_TRIG_SOURCE_LFCLK,
                        src);
    DL_SYSCTL_startFCC();
    while (DL_SYSCTL_isFCCDone() == 0u) {
        delay_cycles(977u);     /* 约 30us @32MHz 参考节拍，仅作超时步进 */
        if (++timeout > 65u) {
            break;              /* 超时保护，避免死等 */
        }
    }
    return DL_SYSCTL_readFCC();
}

/* count → kHz（四舍五入） */
static uint32_t ClockSelfTest_CountToKHz(uint32_t count)
{
    return (count * CLKST_HZ_PER_COUNT + 500u) / 1000u;
}

/* 按实测 MCLK 给出结论字符串 */
static char *ClockSelfTest_Verdict(uint32_t mclkHz)
{
    if (mclkHz >= 64000u && mclkHz <= 96000u) {
        return "MCLK 80MHz PASS";
    }
    if (mclkHz >= 25600u && mclkHz <= 38400u) {
        return "MCLK 32MHz FAIL";
    }
    return "MCLK UNKNOWN";
}

void ClockSelfTest_Run(void)
{
    uint32_t mclkHz;
    uint32_t ratioX1000;
    uint32_t lastBlink = 0u;
    uint8_t  ledState  = 0u;

    /* ---- 静态标签只画一次，之后每轮只刷新数字，减少 I2C 流量 ---- */
    OLED_Clear();
    OLED_ShowString(0,   0, (uint8_t *)"CLK SELFTEST 80M", 8);

    OLED_ShowString(0,   1, (uint8_t *)"MCLK  ", 8);
    OLED_ShowString(84,  1, (uint8_t *)"kHz", 8);

    OLED_ShowString(0,   2, (uint8_t *)"PLL2X ", 8);
    OLED_ShowString(84,  2, (uint8_t *)"kHz", 8);

    OLED_ShowString(0,   3, (uint8_t *)"SYSOSC", 8);
    OLED_ShowString(84,  3, (uint8_t *)"kHz", 8);

    OLED_ShowString(0,   4, (uint8_t *)"RATIO ", 8);
    OLED_ShowString(78,  4, (uint8_t *)"x1000", 8);

    OLED_ShowString(0,   5, (uint8_t *)"CNT8  ", 8);

    OLED_ShowString(0,   6, (uint8_t *)"TICK  ", 8);
    OLED_ShowString(84,  6, (uint8_t *)"ms", 8);

    for (;;) {
        /* ---- 1. 三路时钟实测 ---- */
        s_mclkCnt   = ClockSelfTest_CountClock(DL_SYSCTL_FCC_CLOCK_SOURCE_MCLK);
        s_pll2xCnt  = ClockSelfTest_CountClock(DL_SYSCTL_FCC_CLOCK_SOURCE_SYSPLLCLK2X);
        s_sysoscCnt = ClockSelfTest_CountClock(DL_SYSCTL_FCC_CLOCK_SOURCE_SYSOSC);

        mclkHz = ClockSelfTest_CountToKHz(s_mclkCnt);

        if (s_sysoscCnt != 0u) {
            ratioX1000 = (s_pll2xCnt * 1000u) / s_sysoscCnt;
        } else {
            ratioX1000 = 0u;
        }

        /* ---- 2. 刷新数字 ---- */
        OLED_ShowNum(48, 1, mclkHz, 5, 8);
        OLED_ShowNum(48, 2, ClockSelfTest_CountToKHz(s_pll2xCnt), 5, 8);
        OLED_ShowNum(48, 3, ClockSelfTest_CountToKHz(s_sysoscCnt), 5, 8);
        OLED_ShowNum(48, 4, ratioX1000, 4, 8);
        OLED_ShowNum(48, 5, s_mclkCnt, 5, 8);
        OLED_ShowNum(48, 6, (uint32_t)(tick_ms % 100000u), 5, 8);
        OLED_ShowString(0, 7, (uint8_t *)ClockSelfTest_Verdict(mclkHz), 8);

        /* ---- 3. LED 1Hz 心跳：由 SysTick(CPUCLK) 计时，供示波器/肉眼核对 ---- */
        if ((tick_ms - lastBlink) >= 500u) {
            lastBlink += 500u;
            ledState = (uint8_t)(ledState == 0u ? 1u : 0u);
            if (ledState != 0u) {
                DL_GPIO_setPins(CLKST_LED_PORT, CLKST_LED_PIN);
            } else {
                DL_GPIO_clearPins(CLKST_LED_PORT, CLKST_LED_PIN);
            }
        }
    }
}
