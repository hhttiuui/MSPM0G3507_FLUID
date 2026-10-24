/*
 * key.h —— 3 路按键扫描 + 去抖（挂 TIMA0 中断，5ms 一拍）
 *
 * ==================== 硬件 ====================
 * 三根脚都在 syscfg 的 GPIO_USERKEY 组里，**按下 = 低电平**：
 *   USER1 = PA25   内部上拉（板上无外部上拉）
 *   USER2 = PA24   **本板没装按键，引脚悬空** —— 已改成内部上拉，读数稳定
 *   USER3 = PA23   外部上拉（syscfg 里内部电阻 = NONE）
 *
 * ⚠ 悬空输入一定要有上拉（内部或外部），否则经过去抖器会被随机承认成
 *   "按下"，表现为功能莫名其妙自己触发。本板就是踩过这个：USER2 原本是
 *   NONE，而板上既没按键也没外部上拉。
 *
 * 改接线/换板上拉方式时，syscfg 的 internalResistor 和 KEY_ACTIVE_LOW 要一起看。
 *
 * ==================== 节拍 ====================
 * 扫一次由 TIMA0 的 5ms 周期中断驱动（见 main() 的 NVIC_EnableIRQ），
 * 所以 KEY_TICK_MS 必须和 syscfg 里 TIMER1.timerPeriod 保持一致。
 * ⚠ TIMA0 是 16 位计数器，5ms 不能只改周期：必须同时把 TIMER1.timerClkDiv
 *   从 2 改成 8（40MHz → 10MHz），否则 5ms 需要 200000 拍、超出 16 位范围，
 *   SysConfig 会直接报 out of range。改完 LOAD_VALUE 应该是 49999。
 *
 * ==================== 去抖 ====================
 * 计数式、按下/松开两边对称：原始电平连续 KEY_DEBOUNCE_TICKS 拍都是同一个新
 * 值，才承认状态变化。20ms / 5ms = 4 拍。
 * 好处是"按下"和"松开"用同一套逻辑，且中途抖动只会把计数清零、不会误判。
 *
 * ==================== 使用 ====================
 * 主循环/其它模块只读状态，不要自己去读引脚：
 *   if (Key_IsDown(KEY_USER1)) { ... }      // 去抖后的电平
 *   if (Key_TookPress(KEY_USER3)) { ... }   // "按下"边沿，读后清（切场景用）
 */

#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>

/* TIMA0 的周期，必须和 syscfg 里 TIMER1.timerPeriod 一致（见文件头） */
#define KEY_TICK_MS         (5u)
#define KEY_DEBOUNCE_MS     (20u)
#define KEY_DEBOUNCE_TICKS  (KEY_DEBOUNCE_MS / KEY_TICK_MS)

typedef enum {
    KEY_USER1 = 0,      /* PA25，一键开关机的那个键 */
    KEY_USER2,          /* PA24，本板未装按键（悬空，靠内部上拉）*/
    KEY_USER3,          /* PA23 */
    KEY_NUM
} key_id_t;

/* 用当前电平做初值（避免把"开机时正按着的键"误判成一次新按下）。 */
void Key_Init(void);

/* 每 KEY_TICK_MS 调一次：采样 3 个引脚 + 去抖 + 更新观测量。放 TIMA0 中断里。 */
void Key_Task(void);

/* 去抖后的状态：1 = 现在按着 */
uint8_t Key_IsDown(key_id_t k);

/* "按下"边沿标志，读后自动清零（不读就一直累积着） */
uint8_t Key_TookPress(key_id_t k);

#endif /* __KEY_H */
