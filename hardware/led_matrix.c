/*
 * led_matrix.c —— 面板坐标 → 芯片显示 RAM 的映射实现
 *
 * 全部依据来自手册 + 本板的实测（实测数据写在下面每一步里，改代码前先看）。
 *
 * ============================ 推导 ============================
 * ① 引脚按手册 p7 的顺序编号成 S1..S17
 *    （16×16 模式的有效脚，CA9/CA11 是高阻、跳过）：
 *        S1..S8  = CA1..CA8
 *        S9      = CA10
 *        S10..S16= CA12..CA18
 *        S17     = CA19
 *    **本面板完全没用到 S17(CA19)** —— 实测 16 个相位里它既不当阳极也没有 LED，
 *    所以 CA19 悬空是正确的（手册也允许未用脚开路）。
 *
 * ② 面板是"阶梯形"走线：第 r 行的**阴极一律是 S(c)**；阳极绝大多数是 S(17-r)，
 *    但 col > 16-r 的那一小段改用 S(16-r)，也就是切点每往下一行左移一格。实测：
 *        第 1 行：前 15 颗阳极 CA18(S16) → 阴极 S1..S15；第 16 颗阳极 CA17(S15) → 阴极 S16
 *        第 2 行：前 14 颗阳极 CA17(S15) → 阴极 S1..S14；第 15/16 颗阳极 CA16(S14) → S15、S16
 *    12 个实测点全部吻合。这也解释了"线性走索引看起来在斜着走"——
 *    一个相位(一个阳极)的 15 颗灯横跨两行，本来就是阶梯形的。
 *
 * ③ 芯片的扫描通道 n 与阳极引脚是**一一对应**：通道 n ↔ 阳极 S(n)，即 n = a
 *    （a = 阳极的 S 序号）。
 *    ⚠ 这里踩过一个大坑：一开始误以为是"反序轮转"（S16→n=1、S15→n=2 …），
 *      现象是 mode 7 的光栅走到第 15 步时，从"第一行最后一颗"跳到"最后一行最后一列"——
 *      因为通道 1 的阳极被错认成了 S15（实际是 S1）。
 *    ⚠ 提请注意：**"整屏一起亮一个数据通道"那种图案验证不出这个顺序**，因为它同时
 *      点亮全部 16 个相位，哪个相位对哪根脚完全区分不出来。定位顺序必须用
 *      "一次只点亮一个相位"的测试（按 (行,列) 走光栅，或者逐位游走）。
 *
 * ④ 数据通道 m：手册规定"本相位的那个脚不参与数据"，所以 m = 阴极在 S1..S17 里
 *    **去掉本相位阳极**之后的序号，也就是  m = col - (col > a ? 1 : 0)。
 *    实测确认：n=1（a=15）时阴极 S1..S14 → m=1..14、S16 → m=15、S17 无 LED → m=16。
 *
 * ⑤ 最后套手册公式：addr = 2×(n-1) + (m-1)/8、bit = (m-1)%8。
 *
 * 于是每个相位恰好 15 颗，16 相位 × 15 = 240 = 面板总点数 ✓（自洽性检查）
 * ==============================================================
 */

#include "ti_msp_dl_config.h"
#include "ht16d33a.h"
#include "led_matrix.h"

/* 一帧显示数据（二进制模式 32 字节 = 256 个 LED，1 bit/LED）*/
static uint8_t s_frame[HT16D33A_RAM_BYTES];

/* 面板 (列 col, 行 row) → (RAM 地址 addr, 数据位 bit)。见文件顶部推导。 */
static void LedMatrix_Map(uint8_t col, uint8_t row, uint8_t *addr, uint8_t *bit)
{
    uint8_t a;      /* 本格的阳极引脚在 S1..S17 里的序号 */
    uint8_t n;      /* 芯片扫描通道 1..16 */
    uint8_t m;      /* 芯片数据通道 1..16 */

    if (col <= (uint8_t) (LED_PANEL_COLS - row)) {
        a = (uint8_t) (LED_PANEL_COLS + 1u - row);      /* 主阳极 S(17-r) */
    } else {
        a = (uint8_t) (LED_PANEL_COLS - row);           /* 阶梯切点之后的 S(16-r) */
    }

    n = a;                                              /* 通道 n ↔ 阳极 S(n) */

    m = (uint8_t) (col - ((col > a) ? 1u : 0u));

    *addr = (uint8_t) (2u * (n - 1u) + (m - 1u) / 8u);
    *bit  = (uint8_t) ((m - 1u) % 8u);
}

void LedMatrix_Clear(void)
{
    uint16_t i;

    for (i = 0u; i < (uint16_t) HT16D33A_RAM_BYTES; i++) {
        s_frame[i] = 0x00u;
    }
}

void LedMatrix_SetPanel(uint8_t col, uint8_t row)
{
    uint8_t addr, bit;

    /* 越界 = 面板上本来就没这颗，直接丢掉，让上层不用做边界判断 */
    if ((col == 0u) || (col > LED_PANEL_COLS) || (row == 0u) || (row > LED_PANEL_ROWS)) {
        return;
    }

    LedMatrix_Map(col, row, &addr, &bit);

    if ((uint16_t) addr < (uint16_t) HT16D33A_RAM_BYTES) {
        s_frame[addr] |= (uint8_t) (1u << bit);
    }
}

void LedMatrix_SetRaw(uint8_t addr, uint8_t bit)
{
    if (((uint16_t) addr < (uint16_t) HT16D33A_RAM_BYTES) && (bit < 8u)) {
        s_frame[addr] |= (uint8_t) (1u << bit);
    }
}

void LedMatrix_FillAll(uint8_t on)
{
    uint8_t c, r;

    if (on == 0u) {
        LedMatrix_Clear();
        return;
    }

    /* 逐个真实格子点亮（240 次 ≈ 几十 µs），比"32 字节填 0xFF"多几行，
     * 但不会去依赖"CA19 那些位没有 LED"这个前提。 */
    for (r = 1u; r <= LED_PANEL_ROWS; r++) {
        for (c = 1u; c <= LED_PANEL_COLS; c++) {
            LedMatrix_SetPanel(c, r);
        }
    }
}

void LedMatrix_SetCell(uint8_t x, uint8_t y)
{
    uint8_t col, row;

#if LED_ORIENT_SWAP_XY
    col = y;                        /* 面板 16 宽 ↔ 应用 16 高 */
    row = x;
#else
    col = x;
    row = y;
#endif

#if LED_ORIENT_FLIP_X
    col = (uint8_t) (LED_PANEL_COLS + 1u - col);
#endif
#if LED_ORIENT_FLIP_Y
    row = (uint8_t) (LED_PANEL_ROWS + 1u - row);
#endif

    LedMatrix_SetPanel(col, row);
}

void LedMatrix_Flush(void)
{
    HT16D33A_WriteRam(0x00u, s_frame, (uint16_t) HT16D33A_RAM_BYTES);
}

void LedMatrix_Init(void)
{
    (void) HT16D33A_Init();         /* 选页 → 复位 → 模式 → 电流/亮度 → OSC → 清屏 → 使能全 1 → 开显示 */

    LedMatrix_Clear();
    LedMatrix_Flush();
}
