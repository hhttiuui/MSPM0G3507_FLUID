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

/* 一帧显示数据。大小随显示模式变（见 ht16d33a.h 的 HT16D33A_USE_BINARY_MODE）：
 *   二进制 32 字节（1 bit/点，只有亮灭） / 灰度 256 字节（1 字节/点，0~255 亮度）*/
static uint8_t s_frame[HT16D33A_RAM_BYTES];

/* 面板 (列 col, 行 row) → 芯片 (扫描通道 n, 数据通道 m)。见文件顶部推导。
 * ⚠ 这一步与显示模式**无关**：两种模式下 (n,m) ↔ 物理灯的对应关系是同一套，
 *   灰度模式只是把 (n,m) 打包进 RAM 的方式变了（见 LedMatrix_RamAddr）。 */
static void LedMatrix_MapNM(uint8_t col, uint8_t row, uint8_t *n, uint8_t *m)
{
    uint8_t a;      /* 本格的阳极引脚在 S1..S17 里的序号 */

    if (col <= (uint8_t) (LED_PANEL_COLS - row)) {
        a = (uint8_t) (LED_PANEL_COLS + 1u - row);      /* 主阳极 S(17-r) */
    } else {
        a = (uint8_t) (LED_PANEL_COLS - row);           /* 阶梯切点之后的 S(16-r) */
    }

    *n = a;                                             /* 通道 n ↔ 阳极 S(n) */
    *m = (uint8_t) (col - ((col > a) ? 1u : 0u));
}

/* (n,m) → 显示 RAM 地址。两种模式的打包方式不同（手册 p19~23 / p24~26）：
 *   二进制：每个扫描通道 2 字节，8 个数据通道挤进 1 字节
 *   灰度  ：每个扫描通道 16 字节，一个数据通道独占 1 字节（该字节 = 亮度） */
static uint16_t LedMatrix_RamAddr(uint8_t n, uint8_t m)
{
#if HT16D33A_USE_BINARY_MODE
    return (uint16_t) (2u * ((uint16_t) n - 1u) + ((uint16_t) m - 1u) / 8u);
#else
    return (uint16_t) (16u * ((uint16_t) n - 1u) + ((uint16_t) m - 1u));
#endif
}

void LedMatrix_Clear(void)
{
    uint16_t i;

    for (i = 0u; i < (uint16_t) HT16D33A_RAM_BYTES; i++) {
        s_frame[i] = 0x00u;
    }
}

/* ==================== gamma 2.2 查表 ====================
 *
 * 表值 = round(255 × (i/255)^2.2)。
 *
 * 为什么需要：DDRAM 是**物理线性**的（手机传感器拍出来就是均匀梯度），
 * 但人眼感知 ≈ 亮度^0.45。所以线性映射下中间调全部偏亮、层次被压扁 ——
 * 实测 14 级线性台阶里，前 6 级就吃掉了 75% 的感知变化。
 *
 * 谁用它：
 *   · 流体渲染的"密度 → 亮度"（fluid_demo.c，`gam 1` 打开）
 *   · 以后播放自定义画面时，如果 PC 端没做 gamma，也可以在这里补
 *
 * ⚠ 它只改**中间调**。已经截断到 255 的格子查表后还是 255 ——
 *   想让液体内核不糊成一片白，要调的是 fluid_demo.c 的 `full <n>`
 *   （满量程密度百分比），不是 gamma。 */
static const uint8_t s_gamma22[256] = {
      0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   0u,   1u,
      1u,   1u,   1u,   1u,   1u,   1u,   1u,   1u,   1u,   2u,   2u,   2u,   2u,   2u,   2u,   2u,
      3u,   3u,   3u,   3u,   3u,   4u,   4u,   4u,   4u,   5u,   5u,   5u,   5u,   6u,   6u,   6u,
      6u,   7u,   7u,   7u,   8u,   8u,   8u,   9u,   9u,   9u,  10u,  10u,  11u,  11u,  11u,  12u,
     12u,  13u,  13u,  13u,  14u,  14u,  15u,  15u,  16u,  16u,  17u,  17u,  18u,  18u,  19u,  19u,
     20u,  20u,  21u,  22u,  22u,  23u,  23u,  24u,  25u,  25u,  26u,  26u,  27u,  28u,  28u,  29u,
     30u,  30u,  31u,  32u,  33u,  33u,  34u,  35u,  35u,  36u,  37u,  38u,  39u,  39u,  40u,  41u,
     42u,  43u,  43u,  44u,  45u,  46u,  47u,  48u,  49u,  49u,  50u,  51u,  52u,  53u,  54u,  55u,
     56u,  57u,  58u,  59u,  60u,  61u,  62u,  63u,  64u,  65u,  66u,  67u,  68u,  69u,  70u,  71u,
     73u,  74u,  75u,  76u,  77u,  78u,  79u,  81u,  82u,  83u,  84u,  85u,  87u,  88u,  89u,  90u,
     91u,  93u,  94u,  95u,  97u,  98u,  99u, 100u, 102u, 103u, 105u, 106u, 107u, 109u, 110u, 111u,
    113u, 114u, 116u, 117u, 119u, 120u, 121u, 123u, 124u, 126u, 127u, 129u, 130u, 132u, 133u, 135u,
    137u, 138u, 140u, 141u, 143u, 145u, 146u, 148u, 149u, 151u, 153u, 154u, 156u, 158u, 159u, 161u,
    163u, 165u, 166u, 168u, 170u, 172u, 173u, 175u, 177u, 179u, 181u, 182u, 184u, 186u, 188u, 190u,
    192u, 194u, 196u, 197u, 199u, 201u, 203u, 205u, 207u, 209u, 211u, 213u, 215u, 217u, 219u, 221u,
    223u, 225u, 227u, 229u, 231u, 234u, 236u, 238u, 240u, 242u, 244u, 246u, 248u, 251u, 253u, 255u,
};

uint8_t LedMatrix_Gamma(uint8_t linear)
{
    return s_gamma22[linear];
}

void LedMatrix_SetPanelLevel(uint8_t col, uint8_t row, uint8_t level)
{
    uint8_t  n, m;
    uint16_t addr;

    /* 越界 = 面板上本来就没这颗，直接丢掉，让上层不用做边界判断 */
    if ((col == 0u) || (col > LED_PANEL_COLS) || (row == 0u) || (row > LED_PANEL_ROWS)) {
        return;
    }

    LedMatrix_MapNM(col, row, &n, &m);

    addr = LedMatrix_RamAddr(n, m);
    if (addr >= (uint16_t) HT16D33A_RAM_BYTES) {
        return;
    }

#if HT16D33A_USE_BINARY_MODE
    if (level != 0u) {
        s_frame[addr] |= (uint8_t) (1u << ((m - 1u) % 8u));
    }
#else
    s_frame[addr] = level;              /* 灰度：整字节就是这一点的亮度 */
#endif
}

void LedMatrix_SetPanel(uint8_t col, uint8_t row)
{
    LedMatrix_SetPanelLevel(col, row, LED_MATRIX_MAX_LEVEL);
}

void LedMatrix_SetRawLevel(uint8_t addr, uint8_t level)
{
    if ((uint16_t) addr >= (uint16_t) HT16D33A_RAM_BYTES) {
        return;
    }

#if HT16D33A_USE_BINARY_MODE
    /* 二进制下一个字节管 8 个点，没有"中间亮度"：非 0 就按整字节点亮 */
    s_frame[addr] = (level != 0u) ? 0xFFu : 0x00u;
#else
    s_frame[addr] = level;
#endif
}

void LedMatrix_FillAll(uint8_t on)
{
    if (on == 0u) {
        LedMatrix_Clear();
        return;
    }

#if HT16D33A_USE_BINARY_MODE
    {
        uint8_t c, r;

        /* 逐个真实格子点亮（240 次 ≈ 几十 µs），比"32 字节填 0xFF"多几行，
         * 但不会去依赖"CA19 那些位没有 LED"这个前提。 */
        for (r = 1u; r <= LED_PANEL_ROWS; r++) {
            for (c = 1u; c <= LED_PANEL_COLS; c++) {
                LedMatrix_SetPanel(c, r);
            }
        }
    }
#else
    {
        uint16_t i;

        /* 灰度：整屏拉到最亮。面板只有 240 颗真灯，而 RAM 有 256 个槽，
         * 多写的 16 槽对应 CA19（本板没接 LED），写了也不会亮，无害。 */
        for (i = 0u; i < (uint16_t) HT16D33A_RAM_BYTES; i++) {
            s_frame[i] = LED_MATRIX_MAX_LEVEL;
        }
    }
#endif
}

/* 应用坐标 (x,y) → 面板坐标 (col,row)：套 LED_ORIENT_* 那两下旋转。
 * 只在这里做一次，SetCell / SetCellLevel 共用，免得两边推导走偏。 */
static void LedMatrix_CellToPanel(uint8_t x, uint8_t y, uint8_t *col, uint8_t *row)
{
    uint8_t c, r;

#if LED_ORIENT_SWAP_XY
    c = y;                          /* 面板 16 宽 ↔ 应用 16 高 */
    r = x;
#else
    c = x;
    r = y;
#endif

#if LED_ORIENT_FLIP_X
    c = (uint8_t) (LED_PANEL_COLS + 1u - c);
#endif
#if LED_ORIENT_FLIP_Y
    r = (uint8_t) (LED_PANEL_ROWS + 1u - r);
#endif

    *col = c;
    *row = r;
}

void LedMatrix_SetCellLevel(uint8_t x, uint8_t y, uint8_t level)
{
    uint8_t col, row;

    LedMatrix_CellToPanel(x, y, &col, &row);
    LedMatrix_SetPanelLevel(col, row, level);
}

void LedMatrix_SetCell(uint8_t x, uint8_t y)
{
    LedMatrix_SetCellLevel(x, y, LED_MATRIX_MAX_LEVEL);
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
