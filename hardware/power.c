/*
 * power.c —— 一键开关机的按键状态机（长按 2 秒关机）
 *
 * 为什么单独一个状态机：开机那次按键**还按着**的时候，USER1 就是低电平，
 * 和"用户想关机"在电平上完全一样。唯一的区分办法是**顺序**：
 *   ① 上电后必须先看见 USER1 松开过一次  → 才允许关机（PK_WAIT_RELEASE）
 *   ② 之后要"持续按住 2 秒"才关机         → 防误触（PK_ARMED → PK_COUNTING）
 * 少了第 ① 步的症状就是：一上电立刻关机，永远开不了机。
 *
 * 本文件只通过 Key_IsDown(KEY_USER1) 读按键 —— 去抖、上拉极性都在 key.c 里，
 * 这里不碰任何 GPIO 输入。
 */

#include "ti_msp_dl_config.h"
#include "key.h"
#include "power.h"
#include "debug.h"

typedef enum {
    PK_WAIT_RELEASE = 0,    /* 开机键还按着 / 还没确认松开 */
    PK_ARMED,               /* 已就绪：监视下一次长按 */
    PK_COUNTING,            /* 长按计时中 */
    PK_SHUTDOWN             /* 已释放自锁，等松手断电 */
} pk_state_t;

/* s_state 由 TIMA0 中断改、被主循环（PowerKey_CheckShutdown）读，
 * 所以必须 volatile，理由同 key.c 里的 s_stable。s_holdMs 只在中断里用。 */
static volatile pk_state_t s_state  = PK_WAIT_RELEASE;
static uint32_t            s_holdMs = 0u;

void PowerKey_Task(void)        /* 每 5ms 调一次 */
{
    if (s_state == PK_SHUTDOWN) {
        return;                 /* 已经拉低 PA22 了，不再动作 */
    }

    if (s_state == PK_WAIT_RELEASE) {
        /* 去抖已经在 key.c 做完了，这里直接信 Key_IsDown()：
         * 松开过一次就进入"可关机"状态。 */
        if (Key_IsDown(KEY_USER1) == 0u) {
            s_state = PK_ARMED;
        }
    } else if (s_state == PK_ARMED) {
        if (Key_IsDown(KEY_USER1) != 0u) {
            s_state  = PK_COUNTING;             /* 开始长按计时 */
            s_holdMs = KEY_TICK_MS;
        }
    } else {    /* PK_COUNTING */
        if (Key_IsDown(KEY_USER1) != 0u) {
            s_holdMs += KEY_TICK_MS;

            if (s_holdMs >= POWER_OFF_HOLD_MS) {
                Power_Release();                /* PA22 → 低：释放自锁，松手即断电 */
                s_state = PK_SHUTDOWN;
            }
        } else {
            s_state  = PK_ARMED;                /* 中途松手 → 取消，重新等 */
            s_holdMs = 0u;
        }
    }

    dbg.pwr.state   = (uint8_t) s_state;
    dbg.pwr.hold_ms = s_holdMs;
}

void PowerKey_CheckShutdown(void)
{
    if (s_state != PK_SHUTDOWN) {
        return;
    }

    /* 已经拉低 PA22。此时按键很可能还按着、系统靠按键供电，所以这里什么都
     * 不做，只等松手断电（__WFI 让 CPU 空转，省点电也避免再执行任何逻辑）。 */
    for (;;) {
        __WFI();
    }
}
