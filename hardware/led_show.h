/*
 * led_show.h —— LED 点阵"综合演示"：把几项自检串成一个循环播放的序列
 *
 * 顺序（一轮循环）：
 *   ① 全亮     LED_SHOW_ON_MS      (500ms)
 *   ② 全黑     LED_SHOW_OFF_MS     (500ms)
 *   ③ 逐位游走 LED_SHOW_STEP_MS × 240 步 (20ms/步 ≈ 4.8s)
 *              —— 按**映射表**的 (行,列) 顺序走（走 led_matrix.c），是干净的光栅
 *   ④ 图案     LED_SHOW_PATTERN_MS (2000ms)
 *              —— 外框 + 主对角线 + 左上角 2×2 实心块（顺便当方向标定图案）
 * 一轮 ≈ 7.8 秒。
 *
 * ==================== 调用接口（给以后的串口指令用）====================
 *   LedShow_Start();        // 从头开始播放。只置一个标志位，**从串口中断里调也安全**
 *   LedShow_Stop();         // 立即停（画面停在最后一帧）
 *   LedShow_IsActive();     // 1 = 正在播放
 *   LedShow_Task();         // ⚠ 必须被周期性调用（≥1ms 一次，放主循环里）
 *
 * 典型用法：
 *   串口命令处理：if (cmd == CMD_LED_SHOW) { LedShow_Start(); }
 *   主循环每圈：  LedShow_Task();
 *   若主循环里还有别的刷屏逻辑（比如流体），用
 *       if (LedShow_IsActive() == 0u) { 正常刷屏 }
 *   让出显示权，免得两边抢同一块显示 RAM。
 *
 * 观测量：dbg.led.show_stage（当前段）、dbg.led.step（游走格号）、dbg.led.xfer。
 */

#ifndef __LED_SHOW_H
#define __LED_SHOW_H

#include <stdint.h>

/* 1 = 用"综合演示"当 main() 的主程序（进入后不返回）; 0 = 不用（让位给流体）*/
#define LED_SHOW_ENABLE        0

/* 各段时长（ms）—— 想改节奏改这里 */
#define LED_SHOW_ON_MS          (500u)      /* ① 全亮 */
#define LED_SHOW_OFF_MS         (500u)      /* ② 全黑 */
#define LED_SHOW_STEP_MS        (20u)       /* ③ 游走每步 */
#define LED_SHOW_PATTERN_MS     (2000u)     /* ④ 图案停留 */

/* dbg.led.show_stage 的取值 */
#define LED_SHOW_STAGE_IDLE     (0u)
#define LED_SHOW_STAGE_ON       (1u)
#define LED_SHOW_STAGE_OFF      (2u)
#define LED_SHOW_STAGE_WALK     (3u)
#define LED_SHOW_STAGE_PATTERN  (4u)

/* 从头开始播放（只置标志，真正复位在下一次 LedShow_Task() 里做） */
void LedShow_Start(void);

/* 立即停止（画面保持最后一帧） */
void LedShow_Stop(void);

/* 1 = 正在播放 */
uint8_t LedShow_IsActive(void);

/* 初始化芯片 → 开始播放 → 死循环推进（**永不返回**）。
 * 用在 main() 里当独立演示程序；要接进别的应用（比如流体）就改用上面那套
 * Start / Task / Stop 接口，别调这个。 */
void LedShow_Run(void);

/* 周期任务：推进段落 + 刷屏。≥1ms 调一次，放主循环里，**不要放中断里**（里面有 SPI）*/
void LedShow_Task(void);

/* 单独画一次"方向图案"（外框 + 对角线 + 左上角 2×2 块）并推屏 */
void LedShow_ShowPattern(void);

#endif /* __LED_SHOW_H */
