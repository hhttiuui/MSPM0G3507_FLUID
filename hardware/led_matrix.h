/*
 * led_matrix.h —— 把"面板坐标"翻译成 HT16D33A 的显示 RAM（映射层）
 *
 * ==================== 分层 ====================
 *   fluid.c        纯算法：15 列 × 16 行格子，零外设依赖
 *   fluid_demo.c   胶水层：现在推给 OLED，以后推给这里
 *   led_matrix.c   本层：面板 (列,行) → 芯片 (扫描通道, 数据通道) → (RAM 地址, 位)
 *   ht16d33a.c     最底层：CSB 帧 / 命令 / 写 RAM
 *
 * 为什么需要这一层：手册只给了"逻辑通道 → RAM 位"的公式（addr = 2×(CA-1)+(COM-1)/8），
 * 而**这块面板的引脚和逻辑通道之间还差两次旋转**（阶梯形走线 + 扫描通道轮转，
 * 推导过程见 led_matrix.c 顶部）。把这两次旋转收在这里，上层就只认 (列, 行)。
 *
 * ==================== 尺寸 ====================
 * 面板实测 **16 列 × 15 行 = 240 颗**；流体网格是 **15 列 × 16 行 = 240 格**，
 * 两者正好转置 —— 所以面板挂上去之后要选一次方向（见下面的 LED_ORIENT_*）。
 *
 * ==================== 用法 ====================
 *   LedMatrix_Init();                 // 初始化芯片（含清屏）
 *   LedMatrix_Clear();                // 清本地帧缓冲
 *   LedMatrix_SetCell(x, y);          // 按流体网格坐标点一个格子
 *   LedMatrix_Flush();                // 整帧推给芯片（两个 CSB 帧，~144µs）
 */

#ifndef __LED_MATRIX_H
#define __LED_MATRIX_H

#include <stdint.h>

/* 面板物理尺寸（实测） */
#define LED_PANEL_COLS      16u     /* 横向 16 列 */
#define LED_PANEL_ROWS      15u     /* 纵向 15 行 */

/* 应用（流体）网格尺寸：和面板正好转置 */
#define LED_APP_COLS        15u
#define LED_APP_ROWS        16u

/* ==================== 方向开关 ====================
 * 面板和流体网格是转置关系，具体哪个方向对，取决于面板怎么装在板子上。
 * 调法：看综合演示（hardware/led_show.c）的**图案段**（外框 + 对角线 + 左上角 2×2 块），
 *   ① 外框/对角线不像矩形四边 → SWAP_XY 选错
 *   ② 2×2 实心块出现在**右上/右下/左下角** → 对应地把 FLIP_X / FLIP_Y 翻一下
 * 目标是：**实心块落在图案的左上角**（= 应用坐标 (1,1) 应该落在设备左上角那一格），
 * 这样板子倾斜时液体流向才和手机上的直觉一致。
 * ⚠ 只影响 LedMatrix_SetCell()，LedMatrix_SetPanel() 永远是面板物理坐标。 */
#define LED_ORIENT_SWAP_XY  1u      /* 1 = 面板 16 宽 ↔ 应用 16 高（转置） */
#define LED_ORIENT_FLIP_X   0u      /* 1 = 左右镜像（沿面板列方向翻） */
#define LED_ORIENT_FLIP_Y   0u      /* 1 = 上下镜像（沿面板行方向翻） */

/* 初始化芯片并清屏。内部会调 HT16D33A_Init()，所以不用再单独调它。 */
void LedMatrix_Init(void);

/* 只清本地帧缓冲，不动芯片（要和 LedMatrix_Flush() 配对使用） */
void LedMatrix_Clear(void);

/* ==================== 亮度（灰度模式）============================
 * HT16D33A 有两种显示模式，见 ht16d33a.h 的 HT16D33A_USE_BINARY_MODE：
 *   二进制：1 bit/点  → 本层只有"亮/灭"
 *   灰度  ：1 字节/点 → 本层多出 0~255 的逐点亮度
 * 下面带 Level 的三个函数在两种模式下都能用（二进制模式下非 0 的 level = 满亮）。
 * ⚠ 灰度模式每帧 SPI 是二进制的 8 倍（256 vs 32 字节），代价见 ht16d33a.h。 */
#define LED_MATRIX_MAX_LEVEL    (0xFFu)

/* 按**面板物理坐标**设一点亮度：col 1..16（左→右）、row 1..15（上→下）。
 * 越界（面板上本来就没这颗）直接丢掉，不报错 —— 这样上层不用做边界判断。 */
void LedMatrix_SetPanelLevel(uint8_t col, uint8_t row, uint8_t level);

/* 按**应用坐标**设一点亮度：x 1..15、y 1..16，内部按 LED_ORIENT_* 转到面板坐标 */
void LedMatrix_SetCellLevel(uint8_t x, uint8_t y, uint8_t level);

/* 直写显示 RAM 里的某个字节（诊断用）。灰度模式下就是这一字节的亮度 */
void LedMatrix_SetRawLevel(uint8_t addr, uint8_t level);

/* 上面两者的"满亮"快捷方式，等价于 level = LED_MATRIX_MAX_LEVEL */
void LedMatrix_SetPanel(uint8_t col, uint8_t row);
void LedMatrix_SetCell(uint8_t x, uint8_t y);

/* 一键全亮/全灭：on=1 点亮面板上**所有真实存在**的 LED（走映射，不会去碰 CA19
 * 那些没有 LED 的槽）；on=0 等价于 LedMatrix_Clear()。 */
void LedMatrix_FillAll(uint8_t on);

/* 把整帧推给芯片：0xFD 选页 + 0x80 写 32 字节（一帧原子更新，不会半屏新半屏旧）*/
void LedMatrix_Flush(void);

#endif /* __LED_MATRIX_H */
