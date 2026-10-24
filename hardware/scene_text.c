/*
 * scene_text.c —— 场景 2：中文文字滚动（脱机，不依赖上位机）
 *
 * ==================== 干什么 ====================
 *
 *   把一句 UTF-8 文字用 12×12 点阵字库渲染成一条 1bpp 的横向条带，
 *   然后每帧截 15 列宽的窗口刷到点阵上 —— 看起来就是"字从右边进来、
 *   往左走、从左边出去"。
 *
 * ==================== 为什么用位图字库而不是 TTF ====================
 *
 *   12~13 像素这个尺寸下，TTF 缩下来笔画会糊（很多横竖只剩半格灰），
 *   再二值化就变成断续的点。字库是**逐像素手工调**的文泉驿点阵宋，
 *   每一笔都落在整格上 —— 详见 hardware/wqy12_common.h 顶部。
 *
 *   ⚠ 许可证：文泉驿是 GPL v2 **+ 字体嵌入例外**。嵌进固件不要求固件
 *     本身也开源，但字库文件仍受 GPL 约束（分发要保留声明，header 里已带）。
 *
 * ==================== 数据结构 ====================
 *
 *   条带：s_scratch.text_px[16][32]，1bpp，每行 32 字节 = 256 像素宽。
 *         放共享 union 里（见 app.h 的规矩）—— 只有本场景用，切走就作废。
 *   文字：s_text[] 是普通静态量，**不放 union**，这样切走再切回来文字还在。
 *
 *   256 像素 ≈ 19 个汉字（步进 13）。超了截断，不会越界。
 *
 * ==================== 纵向怎么对 ====================
 *
 *   字库单元 12 宽 × 14 高，CJK 字身占 cell 的 0~11 行，12~13 行是下伸区。
 *   **条带里不烘竖直偏移**（glyph 固定在条带行 0..13），偏移在 DrawFrame
 *   里按当前方向算 —— 因为可用高度会随方向变（16 行 or 15 行）：
 *
 *     rot 0/2：可用高 16 → 偏移 2，字身落在 3~14 行（上下各留 2 行）
 *     rot 1/3：可用高 15 → 偏移 1，字身落在 2~13 行（不裁下伸区）
 *
 * ==================== 方向（自动跟随设备摆放）====================
 *
 *   设备侧着放时，字也要跟着转，观看者才始终能正着读。方向由重力判定：
 *
 *     (ax, ay) ≈ (0, +9.8)  → rot 0  正放
 *     (+9.8, 0)             → rot 1  顺时针 90°
 *     (0, -9.8)             → rot 2  倒放
 *     (-9.8, 0)             → rot 3  逆时针 90°
 *
 *   ⚠ 旋转后屏幕的**宽高会换**：应用网格 15 宽 16 高，正放时物理宽 15、
 *     高 16；侧放时物理宽 16、高 15。所以滚动窗口宽度要跟着 rot 变
 *     （见 WinWidth()），字高 12 在两种情况下都放得下（最短边 15）。
 *
 *   ⚠ 45° 附近两根轴差不多大，判定会来回跳 → 必须"迟滞 + 稳定确认"：
 *     主导轴要超过次轴 1.4 倍（≈±35° 以内），且新方向连续保持 500 ms。
 *
 * ==================== 帧率 ====================
 *
 *   自己按 in->now_ms 记节拍（主循环不做帧率控制，见 app.h）。
 *   默认 40 ms 一帧（25 fps）× 1 像素 = 25 像素/秒。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"           /* tick_ms：Restart() 里重置节拍基准 */
#include "debug.h"
#include "scenes.h"
#include "led_matrix.h"
#include "glyph_store.h"        /* 上电恢复的自定义字形 */
#include "ht16d33a.h"
#include "vcmd.h"           /* VCMD_GetString()：text 命令带过来的字符串 */
#include "wqy12_common.h"   /* 12×12 点阵字库（WQF_Find / WQF_PIXEL）*/

/* ==================== 方向 ====================
 *
 * 条带里 glyph 固定在行 0..13（不烘竖直偏移），偏移在 DrawFrame 里按 rot 加：
 *   rot 0/2 可用高 16 → 偏移 2
 *   rot 1/3 可用高 15 → 偏移 1
 */
#define TEXT_VOFF_EVEN      2u      /* rot 0/2 的竖直偏移 */
#define TEXT_VOFF_ODD       1u      /* rot 1/3 的竖直偏移 */

/* 自动判定方向：主导轴要超过次轴多少倍才认（1.4 ≈ ±35°，避开 45° 抖动）*/
#define TEXT_ORIENT_RATIO   _IQ(1.4f)

/* 新方向要连续保持多久才生效（ms）。太短会在 45° 附近来回翻 */
#define TEXT_ORIENT_HOLD_MS 500u

/* 一帧移动几像素。0 = 定格（配合 tpos 用来静态检查某个位置的字）*/
#define TEXT_STEP_DEF       1u
#define TEXT_STEP_MIN       0u
#define TEXT_STEP_MAX       8u

/* 两帧间隔（ms）。40 ms = 25 fps；调小更顺但更费 CPU */
#define TEXT_MS_DEF         40u
#define TEXT_MS_MIN         10u
#define TEXT_MS_MAX         500u

/* 文字亮度（0~255）。文字是 1bpp 的，所以每格只有"亮"和"灭"两态，
 * 这里定的是"亮"那格的灰度 —— 配合全局亮度 bc 可以压暗整条消息。 */
#define TEXT_LEV_DEF        255u

/* 条带最多 256 像素，对应 32 字节/行 */
#define TEXT_MAX_PX         ((uint16_t) (APP_SCRATCH_ROW_BYTES * 8u))

/* 文字最多 32 字节 UTF-8：约 10 个汉字，或 32 个 ASCII。
 * 和 VCMD_LINE_MAX 对得上（"text " 占 5 字节，剩 59，这里取 32 留余量）。 */
#define TEXT_MAX_BYTES      32u

/* ==================== 状态 ==================== */

/* 默认文字。UTF-8，编译期常量；串口 `text` 可以换掉。 */
static char     s_text[TEXT_MAX_BYTES + 1u] = "你好世界";

static uint8_t  s_step = TEXT_STEP_DEF;
static uint16_t s_ms   = TEXT_MS_DEF;
static uint8_t  s_lev  = TEXT_LEV_DEF;

static uint16_t s_cols;         /* 条带实际宽度（像素），0 = 没内容 */
static uint16_t s_pos;          /* 滚动位置，见 DrawFrame() 的说明 */

/* 方向：0/1/2/3 = 0°/90°/180°/270°，由重力自动判定，也可 torient 手动固定 */
static uint8_t  s_rot;
static uint8_t  s_rotCand;      /* 候选方向 */
static uint32_t s_rotSince;     /* 候选开始稳定的时刻 */
static uint8_t  s_orientAuto = 1u;   /* 1 = 自动跟随重力（默认）*/
static uint32_t s_lastMs;

/* 条带来源：0 = 内置字库（RenderStrip），1 = 上位机发来的字形（ComposeGlyphs）。
 * 用来让 Restart() 在切回本场景时**不要**用字库重画、把上位机的条带覆盖掉。 */
static uint8_t  s_ext;

/* ==================== UTF-8 解码 ==================== */

/* 这个字节作为首字节时，整个字符占几字节。
 * 野续字节（0x80~0xBF）返回 1 —— 当单字节吞掉，否则拷贝循环会死循环。 */
static uint8_t Utf8SeqLen(uint8_t b)
{
    if ((b & 0x80u) == 0u)    { return 1u; }
    if ((b & 0xE0u) == 0xC0u) { return 2u; }
    if ((b & 0xF0u) == 0xE0u) { return 3u; }
    if ((b & 0xF8u) == 0xF0u) { return 4u; }

    return 1u;
}

/* 解一个码位，返回下一个字节的位置。
 * 只处理 1~3 字节（GB2312 里的字和 ASCII 都在 BMP 内）；
 * 4 字节（emoji 等）母库没有，吞掉整个序列并返回 U+FFFD。 */
static const char *Utf8Next(const char *p, uint32_t *cp)
{
    uint8_t c = (uint8_t) *p++;

    if ((c & 0x80u) == 0u) {
        *cp = (uint32_t) c;
        return p;
    }

    if ((c & 0xE0u) == 0xC0u) {
        *cp = (uint32_t) (c & 0x1Fu) << 6;
        if (*p != '\0') {
            *cp |= (uint32_t) ((uint8_t) *p++ & 0x3Fu);
        }
        return p;
    }

    if ((c & 0xF0u) == 0xE0u) {
        *cp = (uint32_t) (c & 0x0Fu) << 12;
        if (*p != '\0') {
            *cp |= (uint32_t) ((uint8_t) *p++ & 0x3Fu) << 6;
        }
        if (*p != '\0') {
            *cp |= (uint32_t) ((uint8_t) *p++ & 0x3Fu);
        }
        return p;
    }

    /* 4 字节序列：跳过最多 3 个后续字节 */
    for (uint8_t i = 0u; (i < 3u) && (((uint8_t) *p & 0xC0u) == 0x80u); i++) {
        p++;
    }
    *cp = 0xFFFDu;
    return p;
}

/* ==================== 渲染条带 ==================== */

/* 把 s_text 画成 1bpp 条带，宽度写进 s_cols。
 * 只做一次（文字变了才重画），每帧只负责"截窗口"。 */
static void RenderStrip(void)
{
    uint16_t x = 0u;
    const char *p = s_text;
    uint16_t row;
    uint16_t i;

    for (row = 0u; row < APP_SCRATCH_ROWS; row++) {
        for (i = 0u; i < APP_SCRATCH_ROW_BYTES; i++) {
            s_scratch.text_px[row][i] = 0u;
        }
    }
    s_cols = 0u;

    while (*p != '\0') {
        uint32_t cp;
        const wqf_glyph_t *g;
        uint8_t gy, gx;

        p = Utf8Next(p, &cp);
        g = WQF_Find(cp);

        if (g == 0) {
            /* 母库没有这个字：画一个**空心方框**（豆腐块）。
             * 为什么不直接留空：留空的话"缺字"和"排版出错"长得一模一样，
             * 排查时根本分不清。画个框一眼就知道"这个字字库里没有"。
             *
             * ⚠ 574 字的常用库覆盖不全（实测"温"就没有）。要显示的字提前
             *   用 font/tools/subset_font.py 从母库裁一个专用字库最稳。 */
            for (gy = 3u; gy <= 11u; gy++) {
                uint16_t py = (uint16_t) gy;

                if (py >= APP_SCRATCH_ROWS) {
                    break;
                }
                for (gx = 1u; gx <= 8u; gx++) {
                    uint16_t px = (uint16_t) (x + gx);

                    if (px >= TEXT_MAX_PX) {
                        break;
                    }
                    if ((gy == 3u) || (gy == 11u) || (gx == 1u) || (gx == 8u)) {
                        s_scratch.text_px[py][px >> 3] |= (uint8_t) (0x80u >> (px & 7u));
                    }
                }
            }
            x = (uint16_t) (x + 10u);
        } else {
            for (gy = 0u; gy < (uint8_t) WQF_H; gy++) {
                uint16_t py = (uint16_t) gy;    /* 不烘偏移，DrawFrame 再加 */

                if (py >= APP_SCRATCH_ROWS) {
                    break;
                }

                for (gx = 0u; gx < g->adv; gx++) {
                    uint16_t px = (uint16_t) (x + gx);

                    if (px >= TEXT_MAX_PX) {
                        break;
                    }
                    /* WQF_PIXEL 的 gx 可以取到 adv-1（13），比 WQF_W(12) 大 ——
                     * 但每行有 2 字节（16 位），高位是补零的，不会越界 */
                    if (WQF_PIXEL(g->bm, gx, gy) != 0u) {
                        s_scratch.text_px[py][px >> 3] |= (uint8_t) (0x80u >> (px & 7u));
                    }
                }
            }
            x = (uint16_t) (x + g->adv);
        }

        if (x >= TEXT_MAX_PX) {
            break;                      /* 条带满了，后面的字放不下 */
        }
    }

    s_cols = x;
}

/* ==================== 画一帧（按方向旋转）====================
 *
 * 物理坐标 (u, v)：u = 观看者看到的横向（阅读方向），v = 观看者看到的纵向。
 * 应用坐标 (x, y)：固定绑在设备上，x 1..15、y 1..16。
 *
 * 正放时物理宽 15、高 16；侧放时宽 16、高 15（因为应用网格是 15×16，
 * 面板是 16×15，转置关系见 led_matrix.h）。所以**窗口宽度随 rot 变**。
 *
 * 四个方向下 (u,v) → (x,y) 的换算 —— 这是最容易写错的地方，逐条推过：
 *
 *   rot 0（正放）        ：x = 1+u    y = 1+v
 *   rot 1（顺 90°，x 朝下）：x = 1+v    y = 16-u
 *   rot 2（倒放）        ：x = 15-u   y = 16-v
 *   rot 3（逆 90°，x 朝上）：x = 15-v   y = 1+u
 *
 * 滚动位置 s_pos 的含义（uw = 当前窗口宽度）：
 *
 *   物理第 u 列显示条带第 (s_pos - uw + u) 列
 *   s_pos = 0        → 全黑（字刚完全出去）
 *   s_pos = uw       → 条带第 0 列落在最左（字刚进来）
 *   s_pos = cols+uw  → 全黑（字完全出去）
 */

/* 当前方向下的窗口宽度（物理横向能放几格）*/
static uint8_t WinWidth(void)
{
    return ((s_rot & 1u) != 0u) ? (uint8_t) LED_PANEL_COLS : (uint8_t) LED_APP_COLS;
}

/* 当前方向下的窗口高度 */
static uint8_t WinHeight(void)
{
    return ((s_rot & 1u) != 0u) ? (uint8_t) LED_APP_COLS : (uint8_t) LED_APP_ROWS;
}

/* 竖直偏移：字库单元 14 行，可用高度 16 或 15，各留一点边 */
static uint8_t WinVOffset(void)
{
    return ((s_rot & 1u) != 0u) ? (uint8_t) TEXT_VOFF_ODD : (uint8_t) TEXT_VOFF_EVEN;
}

static void DrawFrame(void)
{
    uint8_t uw = WinWidth();
    uint8_t vh = WinHeight();
    /* 内置字库单元只有 12 行（放在 14 行的框里，所以要往下偏一点）；
     * 上位机发来的字形是**整屏高度**渲染的，再偏就丢行，所以不偏、行数放到 16。 */
    uint8_t voff = (s_ext != 0u) ? 0u : WinVOffset();
    uint8_t rows = (s_ext != 0u) ? (uint8_t) APP_SCRATCH_ROWS : (uint8_t) WQF_H;
    uint8_t u, v;

    LedMatrix_Clear();

    if (s_cols != 0u) {
        for (u = 0u; u < uw; u++) {
            /* 用有符号算，负数才是"还没进来" */
            int32_t sx = (int32_t) s_pos - (int32_t) uw + (int32_t) u;
            uint16_t bit;

            if ((sx < 0) || (sx >= (int32_t) s_cols)) {
                continue;
            }
            bit = (uint16_t) (0x80u >> ((uint16_t) sx & 7u));

            for (v = 0u; v < vh; v++) {
                /* 条带行 = 物理 v − 竖直偏移；v < voff 时是负数，必须用有符号 */
                int16_t r = (int16_t) ((int16_t) v - (int16_t) voff);
                uint8_t x, y;

                if ((r < 0) || (r >= (int16_t) rows)) {
                    continue;
                }
                if ((s_scratch.text_px[r][(uint16_t) sx >> 3] & bit) == 0u) {
                    continue;
                }

                switch (s_rot) {
                    case 0u:                    /* 正放 */
                        x = (uint8_t) (1u + u);
                        y = (uint8_t) (1u + v);
                        break;

                    case 1u:                    /* 顺时针 90°：物理右 = 应用 -y */
                        x = (uint8_t) (1u + v);
                        y = (uint8_t) (LED_APP_ROWS - u);
                        break;

                    case 2u:                    /* 倒放 */
                        x = (uint8_t) (LED_APP_COLS - u);
                        y = (uint8_t) (LED_APP_ROWS - v);
                        break;

                    default:                    /* 逆时针 90°：物理右 = 应用 +y */
                        x = (uint8_t) (LED_APP_COLS - v);
                        y = (uint8_t) (1u + u);
                        break;
                }

                LedMatrix_SetCellLevel(x, y, s_lev);
            }
        }
    }

    LedMatrix_Flush();
}

/* ==================== 方向判定 ==================== */

/* 把 s_pos 钳到当前方向允许的范围内（切方向时窗口宽度变了，不钳会跳）*/
static void ClampPos(void)
{
    uint16_t hi = (uint16_t) (s_cols + WinWidth());

    if (s_pos > hi) {
        s_pos = hi;
    }
}

/* 每帧调一次（不受 s_ms 节拍限制，要跟得上手）。
 *
 * 判定规则：
 *   ① 主导轴必须超过次轴 TEXT_ORIENT_RATIO(1.4) 倍，否则当成"说不清"，
 *      直接把计时器重置 —— 这就是迟滞，45° 附近不会来回翻
 *   ② 候选方向要连续保持 TEXT_ORIENT_HOLD_MS(500ms) 才生效
 *
 * ⚠ IMU 不可用时（假重力每 2 s 换方向）必须关掉自动，否则字会跟着转圈。
 */
static void TrackOrient(const app_in_t *in)
{
    int32_t axa, aya;
    uint8_t c;

    if ((s_orientAuto == 0u) || (dbg.fluid.imu_use == 0u)) {
        return;
    }

    axa = (in->ax_slow < 0) ? -(int32_t) in->ax_slow : (int32_t) in->ax_slow;
    aya = (in->ay_slow < 0) ? -(int32_t) in->ay_slow : (int32_t) in->ay_slow;

    /* ① 两根轴差不多大 → 方向不明，不更新候选 */
    if ((axa < _IQmpy(aya, TEXT_ORIENT_RATIO)) &&
        (aya < _IQmpy(axa, TEXT_ORIENT_RATIO))) {
        s_rotSince = in->now_ms;
        return;
    }

    if (aya >= axa) {
        c = (in->ay_slow >= 0) ? 0u : 2u;
    } else {
        c = (in->ax_slow >= 0) ? 1u : 3u;
    }

    if (c != s_rotCand) {
        s_rotCand  = c;
        s_rotSince = in->now_ms;
        return;
    }
    if (c == s_rot) {
        return;                             /* 已经是它，不用动 */
    }

    /* ② 稳定够久才真的切 */
    if ((uint32_t) (in->now_ms - s_rotSince) >= TEXT_ORIENT_HOLD_MS) {
        s_rot = c;
        dbg.app.rot = s_rot;
        ClampPos();
        DrawFrame();
    }
}

/* 重画条带 + 从"字刚好在最左边"开始，并立刻出一帧。
 * 文字/参数改完之后调它，改完马上能看到，不用等下一次 tick。 */
static void Restart(void)
{
    if (s_ext == 0u) {
        RenderStrip();              /* 内置字库：重新排版 */
    }
    /* s_ext != 0：条带上位机已经拼好了，切回来直接沿用，不要再覆盖 */

    s_pos      = (uint16_t) WinWidth();   /* 一进来就能看见字，不是一片黑 */
    s_lastMs   = tick_ms;
    s_rotSince = tick_ms;
    s_rotCand  = s_rot;
    dbg.app.rot = s_rot;
    DrawFrame();
}

/* ==================== 上位机字形拼接 ====================
 *
 * 数据格式（一个包，见 vcmd.c 的 ver=2）：
 *   [w][h][bitmap] × count
 *   bitmap：ceil(w/8) × h 字节，行优先，**MSB = 最左**（和内部条带一致）
 *
 * 拼接规则：字形按顺序首尾相接，每个字后面留 spacing 像素空白
 * （spacing 为负 = 下一个字压到上一个字上，页面的"字间距"允许 -4）。
 * 拼好的条带直接进 s_scratch.text_px，之后完全复用本文件的滚动 + 旋转逻辑
 * （DrawFrame 会按 s_rot 重新映射坐标，所以重力朝向自动生效）。
 *
 * 返回 1 = 成功，0 = 数据不合法（长度不够 / w/h 越界）。
 *
 * ⚠ 高度 ≤16 = 整屏。上位机按屏幕高度渲染，DrawFrame 对这种条带不加竖直偏移，
 *   所以字形上下边距由上位机自己控制（页面上的"偏移 Y"）。
 */
uint8_t TextScene_ComposeGlyphs(const uint8_t *p, uint16_t len,
                                uint8_t count, int8_t spacing)
{
    int32_t  cursor = 0;        /* 可为负：spacing < 0 时是"倒回去收紧" */
    int32_t  endX   = 0;        /* 已画内容的最右边界（不含末尾间距）*/
    uint16_t row, i;
    uint8_t  n;

    if ((p == 0) || (count == 0u)) {
        return 0u;
    }

    for (row = 0u; row < APP_SCRATCH_ROWS; row++) {
        for (i = 0u; i < APP_SCRATCH_ROW_BYTES; i++) {
            s_scratch.text_px[row][i] = 0u;
        }
    }

    for (n = 0u; n < count; n++) {
        uint8_t  w, h, roww;
        uint16_t need;
        uint8_t  gy, gx;

        if (len < 2u) {
            return 0u;
        }

        w = p[0];
        h = p[1];
        /* 宽度允许到 120：字号调大时一个汉字可能比屏幕还宽，靠滚动露出全貌；
         * 上位机的"整条条带"也复用这个格式（w = 条带宽度，count = 1）。
         * 上限 120 是帧头 [5] 的 1 字节长度定的：ceil(120/8)×16 = 240 ≤ 255。 */
        if ((w == 0u) || (w > 120u) || (h == 0u) || (h > 16u)) {
            return 0u;
        }

        roww = (uint8_t) ((w + 7u) / 8u);
        need = (uint16_t) (2u + ((uint16_t) roww * (uint16_t) h));
        if (len < need) {
            return 0u;
        }

        for (gy = 0u; gy < h; gy++) {
            for (gx = 0u; gx < w; gx++) {
                uint8_t b = p[2u + ((uint16_t) gy * (uint16_t) roww) + (uint8_t) (gx >> 3)];

                if ((b & (uint8_t) (0x80u >> (gx & 7u))) != 0u) {
                    int32_t x = cursor + (int32_t) gx;

                    if ((x >= 0) && (x < (int32_t) TEXT_MAX_PX)) {
                        uint16_t ux = (uint16_t) x;

                        s_scratch.text_px[gy][ux >> 3] |=
                            (uint8_t) (0x80u >> (ux & 7u));
                    }
                }
            }
        }

        p    += need;
        len   = (uint16_t) (len - need);

        cursor += (int32_t) w;
        if (cursor > endX) {
            endX = cursor;                  /* 记住最右边界：末尾不再加间距 */
        }

        if (cursor >= (int32_t) TEXT_MAX_PX) {
            break;                          /* 条带满了 */
        }
        cursor += (int32_t) spacing;        /* 负数 = 下一个字压过来 */
        if (cursor < 0) {
            cursor = 0;
        }
    }

    s_ext      = 1u;                        /* 明确条带来源是上位机 */
    s_cols     = (uint16_t) endX;
    s_pos      = (uint16_t) WinWidth();
    s_lastMs   = tick_ms;
    s_rotSince = tick_ms;
    s_rotCand  = s_rot;
    dbg.app.rot = s_rot;
    ClampPos();
    DrawFrame();

    return 1u;
}

/* ==================== 上位机条带（整条长图）====================
 *
 * 数据：bits = 16 行 × ceil(cols/8) 字节，行优先，MSB = 最左。
 * 直接把条带灌进 s_scratch.text_px，之后完全复用本文件的滚动 + 重力朝向
 * （s_cols / s_pos / s_step / s_ms / DrawFrame 全都现成）。
 *
 * 和 TextScene_ComposeGlyphs 的区别：那个是"按字拼"，这个是"整条照搬"，
 * 排版已经在 PC 上做完了。
 *
 * 返回 1 = 成功，0 = 参数不合法。
 */
uint8_t TextScene_LoadStrip(const uint8_t *bits, uint16_t cols)
{
    uint8_t  roww, row;
    uint16_t i;

    if ((bits == 0) || (cols == 0u) || (cols > TEXT_MAX_PX)) {
        return 0u;
    }

    roww = (uint8_t) ((cols + 7u) / 8u);

    for (row = 0u; row < APP_SCRATCH_ROWS; row++) {
        for (i = 0u; i < APP_SCRATCH_ROW_BYTES; i++) {
            s_scratch.text_px[row][i] =
                (i < (uint16_t) roww) ? bits[((uint16_t) row * roww) + i] : 0u;
        }
    }

    s_ext      = 1u;                        /* 条带来源是上位机 */
    s_cols     = cols;
    s_pos      = (uint16_t) WinWidth();
    s_lastMs   = tick_ms;
    s_rotSince = tick_ms;
    s_rotCand  = s_rot;
    dbg.app.rot = s_rot;
    ClampPos();
    DrawFrame();

    return 1u;
}

/* ==================== 场景入口 ==================== */

void TextScene_Enter(void)
{
    /* 有存 Flash 的自定义字形时优先用它（掉电保留的那条消息）。
     * 否则用内置字库的默认文字。 */
    if (GlyphStore_HasData() != 0u) {
        const glyph_meta_t *m = GlyphStore_Meta();

        TextScene_SetScroll(m->step, m->ms, m->lev);

        if (TextScene_ComposeGlyphs(GlyphStore_Data(), GlyphStore_DataLen(),
                                    m->count, m->spacing) != 0u) {
            return;
        }
    }

    /* App_Goto 已经清过屏。这里重画条带 —— 切走再切回来时 union 可能被
     * 别的场景踩过，不能假设 px 还是好的。文字本身在静态区，没丢。 */
    Restart();
}

void TextScene_SetScroll(uint8_t step, uint16_t ms, uint8_t lev)
{
    if ((step >= (uint8_t) TEXT_STEP_MIN) && (step <= (uint8_t) TEXT_STEP_MAX)) {
        s_step = step;
    }
    if ((ms >= (uint16_t) TEXT_MS_MIN) && (ms <= (uint16_t) TEXT_MS_MAX)) {
        s_ms = ms;
    }

    s_lev    = lev;
    s_lastMs = tick_ms;
}

void TextScene_Exit(void)
{
    /* 什么都不用做：App_Goto 会统一清屏 */
}

void TextScene_Tick(const app_in_t *in)
{
    /* 方向判定每帧都跑（不受 s_ms 节拍限制）—— 侧过来要马上有反应 */
    TrackOrient(in);

    if (s_cols == 0u) {
        return;                             /* 没字可滚 */
    }

    if ((uint32_t) (in->now_ms - s_lastMs) < (uint32_t) s_ms) {
        return;                             /* 还没到下一帧 */
    }
    s_lastMs = in->now_ms;

    if (s_step == 0u) {
        return;                             /* tstep 0 = 定格，不推进 */
    }

    s_pos = (uint16_t) (s_pos + s_step);

    if (s_pos >= (uint16_t) (s_cols + WinWidth())) {
        s_pos = 0u;                         /* 整条完全滚出去之后从头再来 */
    }

    DrawFrame();
}

/* ==================== 命令 ==================== */

uint8_t TextScene_Cmd(uint16_t id, const int32_t *arg)
{
    switch (id) {
        case DBG_CMD_TEXT_SET:
            /* text <UTF-8 字符串>：字符串由 vcmd 存好，这里只取指针。
             * 超长会被 vcmd 截断（VCMD_LINE_MAX），这里再兜一次。 */
            {
                const char *src = VCMD_GetString();
                uint8_t i = 0u;
                uint8_t k;

                if ((src == 0) || (*src == '\0')) {
                    dbg.cmd.result = 2u;    /* 空字符串：不改，报参数错 */
                    return 1u;
                }

                /* ⚠ 必须**按字符边界**截断。直接按字节数切会把一个 3 字节的
                 * 汉字切成两半，剩下半个序列会被解码成乱码 —— 症状是最后
                 * 一个字变成豆腐块或几个莫名其妙的点。 */
                while (src[i] != '\0') {
                    uint8_t n = Utf8SeqLen((uint8_t) src[i]);

                    if ((uint16_t) ((uint16_t) i + (uint16_t) n) >
                        (uint16_t) TEXT_MAX_BYTES) {
                        break;              /* 放不下整个字就不放 */
                    }
                    i = (uint8_t) (i + n);
                }

                for (k = 0u; k < i; k++) {
                    s_text[k] = src[k];
                }
                s_text[i] = '\0';
            }
            s_ext = 0u;                 /* 切回内置字库渲染 */
            Restart();
            return 1u;

        case DBG_CMD_TEXT_STEP:
            if ((arg[0] >= (int32_t) TEXT_STEP_MIN) &&
                (arg[0] <= (int32_t) TEXT_STEP_MAX)) {
                s_step = (uint8_t) arg[0];
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_TEXT_MS:
            if ((arg[0] >= (int32_t) TEXT_MS_MIN) &&
                (arg[0] <= (int32_t) TEXT_MS_MAX)) {
                s_ms = (uint16_t) arg[0];
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_TEXT_POS:
            /* 直接跳到某个滚动位置并重画 —— 配合 `tstep 0` 定格检查。
             * 范围 0 ~ cols+15，超出按边界截。 */
            if (s_cols != 0u) {
                int32_t lim = (int32_t) s_cols + (int32_t) WinWidth();

                if (arg[0] < 0) {
                    s_pos = 0u;
                } else if (arg[0] > lim) {
                    s_pos = (uint16_t) lim;
                } else {
                    s_pos = (uint16_t) arg[0];
                }
                DrawFrame();
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_TEXT_ORIENT:
            /* torient 0~3：手动固定方向（锁死，不再跟重力）
             * torient 4  ：恢复自动跟随
             * 板子固定安装后锁死比较稳，免得你转个角度看它就跟着翻。 */
            if ((arg[0] >= 0) && (arg[0] <= 3)) {
                s_orientAuto = 0u;
                if (s_rot != (uint8_t) arg[0]) {
                    s_rot = (uint8_t) arg[0];
                    dbg.app.rot = s_rot;
                    ClampPos();
                }
                DrawFrame();
            } else if (arg[0] == 4) {
                s_orientAuto = 1u;
                s_rotCand  = s_rot;
                s_rotSince = tick_ms;
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_TEXT_LEV:
            if ((arg[0] >= 0) && (arg[0] <= 255)) {
                s_lev = (uint8_t) arg[0];
                DrawFrame();                /* 亮度改了立刻重画一帧 */
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        default:
            return 0u;
    }
}
