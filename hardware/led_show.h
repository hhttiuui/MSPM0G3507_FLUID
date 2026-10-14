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

/* ==================== 灰度验证序列（上电自检）====================
 * 为什么单独做一套：把显示切到灰度是**驱动层**的改动（RAM 打包方式 + 每点 1 字节），
 * 先把它单独验掉，再谈"把流体密度映射成亮度" —— 否则出问题分不清是哪一层的锅。
 *
 * 一轮循环（约 8.5 秒）：
 *   ① 全亮     0.6s  —— 确认 LED 使能/扫描没坏
 *   ② 全灭     0.3s
 *   ③ 方向图案 2.0s  —— 外框 + 主对角线 + 左上角 2×2（全亮度）
 *                       → 验**地址映射与方向**
 *   ④ 亮度阶梯 2.0s  —— 整屏 14 级灰度台阶（0 → 255）
 *                       → 验**"每点 1 字节"的灰度是否真的生效**
 *   ⑤ 单点游走 15ms × 240 步（3.6s）—— 按面板 (列,行) 顺序走一遍
 *                       → 验**地址顺序**（映射错了这个最藏不住）
 *
 * 判读（对不上时按这个顺序怀疑）：
 *   · 外框不成矩形 / 对角线歪了 → 地址映射或方向错（先试翻 LED_ORIENT_*）
 *   · ④ 是一条均匀亮带、看不出台阶 → 灰度没生效，还在二进制模式（查 0x31 的 BGS 位）
 *   · ⑤ 走位顺序乱跳 → LedMatrix_RamAddr() 的打包公式不对
 *   · 都对 → 灰度链路通了，可以接流体密度 → 亮度
 *
 * 上电默认就跑（LED_GRAY_TEST_ENABLE=1），跑完自动进流体 Demo；
 * 中途**按一下**任一按键（USER1/2/3）即可跳过。
 *   ⚠ 判据用的是"按下边沿"而不是"按住电平" —— 本板 USER2(PA24) 悬空读低，
 *     用电平判会一进自检就退出（踩过这个坑，详见 led_show.c 的 GraySkipRequested）。 */
/* 1 = 上电先跑一遍灰度自检（验证用）; 0 = 直接进流体 Demo（**当前**）。
 * 灰度已经在 2026-10 实测通过（映射/顺序/亮度分级都对），所以默认关掉；
 * 以后改了映射层或换了面板，把它改回 1 就能重新验一遍。 */
#define LED_GRAY_TEST_ENABLE    (0)

/* 整个自检最多跑多久（ms）；到点自动让位给流体 Demo，不会卡在测试里 */
#define LED_GRAY_TEST_HOLD_MS   (20000u)
#define LED_GRAY_TEST_ON_MS     (600u)      /* ① 全亮 */
#define LED_GRAY_TEST_OFF_MS    (300u)      /* ② 全灭 */
#define LED_GRAY_TEST_PAT_MS    (2000u)     /* ③ 方向图案 */
#define LED_GRAY_TEST_RAMP_MS   (2000u)     /* ④ 亮度阶梯 */
#define LED_GRAY_TEST_STEP_MS   (15u)       /* ⑤ 游走每步 */

/* 画一次亮度阶梯（整屏台阶）并推屏 */
void LedShow_ShowGrayRamp(void);

/* 跑一遍灰度自检序列（阻塞，跑完返回）。上电由 main() 调用。 */
void LedShow_GrayTest(void);

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
