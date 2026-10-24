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
 *   字库单元 12 宽 × 14 高，其中 CJK 字身占 cell 的 0~11 行，
 *   12~13 行是下伸区（g j p q y 用）。屏幕 16 行，所以：
 *
 *     条带行 = cell 行 + TEXT_TOP(2)      条带行 2..15 → 屏幕行 3..16
 *
 *   于是 CJK 字身落在屏幕 3~14 行，上下各留 2 行 —— 正好居中。
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
#include "ht16d33a.h"
#include "vcmd.h"           /* VCMD_GetString()：text 命令带过来的字符串 */
#include "wqy12_common.h"   /* 12×12 点阵字库（WQF_Find / WQF_PIXEL）*/

/* 条带行 → 屏幕行：CJK 字身居中 */
#define TEXT_TOP            2u

/* 一帧移动几像素（1 = 最顺滑）*/
#define TEXT_STEP_DEF       1u
#define TEXT_STEP_MIN       1u
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
static uint32_t s_lastMs;

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
                uint16_t py = (uint16_t) (gy + TEXT_TOP);

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
                uint16_t py = (uint16_t) (gy + TEXT_TOP);

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

/* ==================== 画一帧 ==================== */

/* 滚动位置 s_pos 的含义：
 *
 *   屏幕第 x 列（0..14）显示条带第  (s_pos - 15 + x)  列
 *
 *   s_pos = 0            → 全都 < 0，屏幕全黑（字刚完全出去）
 *   s_pos = 15           → 条带第 0 列正好落在屏幕最左列（字刚进来）
 *   s_pos = s_cols + 15  → 全都 >= s_cols，屏幕全黑（字完全出去）
 *
 * 所以循环范围是 [0, s_cols + 15]，到顶回 0 就是"再滚一遍"。
 */
static void DrawFrame(void)
{
    uint8_t x, y;

    LedMatrix_Clear();

    if (s_cols != 0u) {
        for (x = 0u; x < (uint8_t) LED_APP_COLS; x++) {
            /* 用有符号算，负数才是"还没进来" */
            int32_t sx = (int32_t) s_pos - (int32_t) LED_APP_COLS + (int32_t) x;

            if ((sx < 0) || (sx >= (int32_t) s_cols)) {
                continue;
            }

            for (y = 0u; y < (uint8_t) APP_SCRATCH_ROWS; y++) {
                if ((s_scratch.text_px[y][(uint16_t) sx >> 3] &
                     (uint8_t) (0x80u >> ((uint16_t) sx & 7u))) != 0u) {
                    LedMatrix_SetCellLevel((uint8_t) (x + 1u), (uint8_t) (y + 1u), s_lev);
                }
            }
        }
    }

    LedMatrix_Flush();
}

/* 重画条带 + 从"字刚好在最左边"开始，并立刻出一帧。
 * 文字/参数改完之后调它，改完马上能看到，不用等下一次 tick。 */
static void Restart(void)
{
    RenderStrip();
    s_pos    = (uint16_t) LED_APP_COLS;   /* 一进来就能看见字，不是一片黑 */
    s_lastMs = tick_ms;
    DrawFrame();
}

/* ==================== 场景入口 ==================== */

void TextScene_Enter(void)
{
    /* App_Goto 已经清过屏。这里重画条带 —— 切走再切回来时 union 可能被
     * 别的场景踩过，不能假设 px 还是好的。文字本身在静态区，没丢。 */
    Restart();
}

void TextScene_Exit(void)
{
    /* 什么都不用做：App_Goto 会统一清屏 */
}

void TextScene_Tick(const app_in_t *in)
{
    if (s_cols == 0u) {
        return;                             /* 没字可滚 */
    }

    if ((uint32_t) (in->now_ms - s_lastMs) < (uint32_t) s_ms) {
        return;                             /* 还没到下一帧 */
    }
    s_lastMs = in->now_ms;

    s_pos = (uint16_t) (s_pos + s_step);

    if (s_pos >= (uint16_t) (s_cols + LED_APP_COLS)) {
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
