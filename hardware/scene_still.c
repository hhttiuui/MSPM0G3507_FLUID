/*
 * scene_still.c —— 场景 1：静态图（亮度标定 / 手工画面）
 *
 * ==================== 这个场景干什么 ====================
 *
 *   · 亮度标定：画 14 级阶梯，探头逐行读，得到 DDRAM → 实际光输出的物理曲线
 *   · 手工画面：px 单点 / fill 整屏填充 / off 全灭
 *
 * tick() **什么都不做** —— DDRAM 是持久的，画完就一直显示，不需要每帧重绘。
 * 这是"场景"这个概念最省事的一种：只有一个 enter 和一组命令。
 *
 * ==================== 为什么进来先画阶梯 ====================
 *
 * 不画的话进来是一片黑，你分不清"切场景成功了"还是"板子卡死了"。
 * 阶梯是信息量最大的一帧：能看出亮度层次、能看出映射对不对、能直接当标尺用。
 * 反正标定流程本来也是 `ramp`，重复画一次无害。
 *
 * ==================== 和以前的工作流不一样了 ====================
 *
 * 以前 px/fill/ramp 是在流体场景里发的，靠 `lock 1` 冻结流体渲染才留得住。
 * 现在必须**先切到这个场景**（串口 `mode 1` 或按 USER2），然后发 px/fill/ramp。
 * 好处是 `lock` 这个概念不需要了 —— 场景本身就不会被别人覆盖。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"          /* tick_ms：几何动画的帧节拍 */
#include "debug.h"
#include "scenes.h"
#include "led_matrix.h"
#include "ht16d33a.h"

/* ==================== 亮度阶梯（两种） ====================
 *
 * 两条命令画的**内容完全相同**，只有亮度取值不同，专门用来对比"线性 vs gamma"：
 *
 *   线性（ramp） ：0 19 39 58 78 98 117 137 156 176 196 215 235 255
 *   gamma2.2（rampg）：0  1  4 10 19 31  47  65  88 114 143 177 214 255
 *
 * 为什么需要 gamma：LED 的 PWM 占空比是**物理线性**的（手机传感器测出来就是
 * 均匀梯度），但人眼感知约等于 `亮度^0.45`。所以线性台阶里，前 6 级就吃掉了
 * 75% 的感知变化，后 8 级看起来几乎一样亮 —— 实测就是这样。
 *
 * 用途分工：
 *   ramp  —— **亮度标定**用。每行一个已知 DDRAM 值，探头逐行读，
 *            得到 DDRAM → 实际光输出的物理曲线（这条曲线应该是线性的）。
 *   rampg —— **观感验收**用。肉眼看上去才是均匀的 14 级。
 *
 * 表长必须等于台阶数（LED_APP_ROWS - 2 = 14），下标 = y - 2。 */
static const uint8_t s_rampLinear[LED_APP_ROWS - 2u] = {
    0u, 19u, 39u, 58u, 78u, 98u, 117u, 137u, 156u, 176u, 196u, 215u, 235u, 255u
};

static const uint8_t s_rampGamma[LED_APP_ROWS - 2u] = {
    0u, 1u, 4u, 10u, 19u, 31u, 47u, 65u, 88u, 114u, 143u, 177u, 214u, 255u
};

/* 按给定的 14 级表画阶梯（第 2~15 行每行一个亮度），外框全亮。 */
static void DrawRampFrom(const uint8_t *levels)
{
    uint8_t x, y;

    LedMatrix_Clear();

    for (y = 2u; y <= (uint8_t) (LED_APP_ROWS - 1u); y++) {
        uint8_t lv = levels[y - 2u];

        for (x = 1u; x <= LED_APP_COLS; x++) {
            LedMatrix_SetCellLevel(x, y, lv);
        }
    }

    /* 外框全亮，这样 16 列 / 16 行全都画到 */
    for (x = 1u; x <= LED_APP_COLS; x++) {
        LedMatrix_SetCellLevel(x, 1u, 255u);
        LedMatrix_SetCellLevel(x, (uint8_t) LED_APP_ROWS, 255u);
    }
    for (y = 1u; y <= LED_APP_ROWS; y++) {
        LedMatrix_SetCellLevel(1u, y, 255u);
        LedMatrix_SetCellLevel((uint8_t) LED_APP_COLS, y, 255u);
    }

    LedMatrix_Flush();
}

static void DrawRamp(void)      { DrawRampFrom(s_rampLinear); }
static void DrawRampGamma(void) { DrawRampFrom(s_rampGamma);  }

/* ==================== 场景入口 ==================== */

void StillScene_Enter(void)
{
    /* App_Goto 已经清过屏并 Flush 了，这里直接画内容 */
    DrawRamp();
}

void StillScene_Tick(const app_in_t *in)
{
    (void) in;                  /* DDRAM 持久，不需要每帧重绘 */
}

void StillScene_Exit(void)
{
    /* 什么都不用做：App_Goto 会统一清屏 */
}

uint8_t StillScene_Cmd(uint16_t id, const int32_t *arg)
{
    uint8_t x, y;

    switch (id) {
        case DBG_CMD_LED_FILL:
            if ((arg[0] >= 0) && (arg[0] <= 255)) {
                LedMatrix_Clear();
                for (x = 1u; x <= LED_APP_COLS; x++) {
                    for (y = 1u; y <= LED_APP_ROWS; y++) {
                        LedMatrix_SetCellLevel(x, y, (uint8_t) arg[0]);
                    }
                }
                LedMatrix_Flush();
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_LED_OFF:
            LedMatrix_Clear();
            LedMatrix_Flush();
            return 1u;

        case DBG_CMD_LED_PX:
            /* px <x> <y> <lv>：单点，不整屏重绘 —— 标定时可以一点一点加 */
            if ((arg[0] >= 1) && (arg[0] <= (int32_t) LED_APP_COLS) &&
                (arg[1] >= 1) && (arg[1] <= (int32_t) LED_APP_ROWS) &&
                (arg[2] >= 0) && (arg[2] <= 255)) {
                LedMatrix_SetCellLevel((uint8_t) arg[0], (uint8_t) arg[1],
                                       (uint8_t) arg[2]);
                LedMatrix_Flush();
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_LED_PAT:
            /* 方向测试图。用途：确认"应用坐标 (x,y) 在面板上到底朝哪"。
             *
             * 画的是：四边外框 + **左上角内侧 2×2 实心块**。
             * 判断方法：实心块应该出现在**左上角**。
             *   跑到右上 → LED_ORIENT_FLIP_X 反了
             *   跑到左下 → LED_ORIENT_FLIP_Y 反了
             *   跑到右下 → 两个都反了
             *   外框长宽比看着"高比宽大"→ 屏幕是竖的，正常（应用网格 15 宽 16 高）
             *
             * ⚠ 文字看不清时先发这个：如果 2×2 块不在左上角，说明整个应用坐标系
             *   和面板对不上，那不是文字场景的问题，是 led_matrix.h 的方向开关。 */
            LedMatrix_Clear();

            for (x = 1u; x <= LED_APP_COLS; x++) {
                LedMatrix_SetCellLevel(x, 1u, 255u);
                LedMatrix_SetCellLevel(x, (uint8_t) LED_APP_ROWS, 255u);
            }
            for (y = 1u; y <= LED_APP_ROWS; y++) {
                LedMatrix_SetCellLevel(1u, y, 255u);
                LedMatrix_SetCellLevel((uint8_t) LED_APP_COLS, y, 255u);
            }

            LedMatrix_SetCellLevel(2u, 2u, 255u);
            LedMatrix_SetCellLevel(3u, 2u, 255u);
            LedMatrix_SetCellLevel(2u, 3u, 255u);
            LedMatrix_SetCellLevel(3u, 3u, 255u);

            LedMatrix_Flush();
            return 1u;

        case DBG_CMD_LED_RAMP:
            DrawRamp();
            return 1u;

        case DBG_CMD_LED_RAMP_GAMMA:
            DrawRampGamma();
            return 1u;

        default:
            return 0u;          /* 不是我的命令，交给 app 报 result = 3 */
    }
}


/* ==================== 几何动画（阶段 4）====================
 *
 * 一层极小的渲染层 + 一张动画表（见文件末尾的 s_anims[]）。
 *
 * ==================== 为什么要"超采样" ====================
 * 15×16 直接画线只有两种结果：要么 1 px 宽的硬线（斜线全是锯齿），
 * 要么完全画不出来。这里先用 3 倍超采样在 45×48 的缓冲上画，
 * 再按 3×3 平均降回 15×16 —— 每个输出格有 9 级"覆盖度"，
 * 于是斜线变成灰度渐变，看着就"清晰"了。
 *
 * 代价：超采样缓冲 45×48 = 2160 B RAM。想省 RAM 就把 GEO_SUP 改成 2
 *      （30×32 = 960 B，但只剩 4 级覆盖度，锯齿会回来一些）。
 *
 * 定点：全部 Q8.8（int16，1.0 = 256）；三角函数查 256 项表（M0+ 没有 FPU）。
 *
 * 动画（ganim，编号 = s_anims[] 的下标）：
 *   0 弹跳球     **吃 IMU 重力**（倾斜板子，球就往低处滚）+ 可调半径
 *   1 旋转立方体 3D→2D 投影 + 超采样抗锯齿的 12 条棱
 *   2 扩散环     直接在 15×16 上按距离场算，不走超采样
 *
 * ==================== 怎么加一个新动画 ====================
 *   ① 写一个 `static void anim_xxx_frame(const app_in_t *in)`，
 *      **自己负责整条流水线**：超采样类先 h_clear()、画完 geo_compose()；
 *      直接类（像扩散环）就画 s_gbuf。
 *   ② 有状态要重置就再写个 `static void anim_xxx_reset(void)`，没有填 0。
 *   ③ 在 s_anims[] 末尾加一行 { anim_xxx_frame, anim_xxx_reset }。
 * 就这三步。不用改 switch、不用改 app.c —— route_item_count() 直接取
 * GeomScene_AnimCount()，项数跟着表自动变。
 */

#define GEO_W          ((uint8_t) LED_APP_COLS)          /* 15 */
#define GEO_H          ((uint8_t) LED_APP_ROWS)          /* 16 */
#define GEO_N          ((uint16_t) (GEO_W * GEO_H))      /* 240 */
#define GEO_FPS_MS     (40u)                             /* 25 fps */
#define GEO_SUP        (3u)                              /* 超采样倍数 */
#define GEO_HW         ((uint16_t) (GEO_W * GEO_SUP))    /* 45 */
#define GEO_HH         ((uint16_t) (GEO_H * GEO_SUP))    /* 48 */
#define GEO_HN         ((uint16_t) (GEO_HW * GEO_HH))    /* 2160 */
#define GEO_SUP2       ((uint16_t) (GEO_SUP * GEO_SUP))  /* 9 */
#define GEO_BALL_RMAX  (6u)
#define GEO_BALL_RDEF  (2u)
#define GEO_KICK_MAX   (8u)
#define GEO_KICK_DEF   (3u)                              /* 碰边后固定的弹起速度 px/帧 */
#define GEO_DAMP_MAX   (8u)
#define GEO_DAMP_DEF   (2u)                              /* 每横向撞一次减 s_damp*4/256 */
#define GEO_HDAMP_MIN  (64u)                             /* 水平弹性最低保留 25% */

/* ---- 扩散环（ganim 2）---- */
/* 最大半径 9.4 px：再大屏幕就只剩四个角的弧了（对角线约 10.6 px），
 * 到这里就重来一圈，看起来是"一轮一轮地往外撞"。 */
#define GEO_RING_MAX   (2400)
#define GEO_RING_STEP  (90)                              /* 每帧半径增量 ≈ 0.35 px */
#define GEO_RING_W     (300)                             /* 环宽 ≈ 1.2 px（Q8.8）*/

/* Q8.8 正弦表：sin(2πi/256) × 256，峰值夹到 ±255（256 会让 (v+256)>>1 溢出）*/
static const int16_t s_geo_sin[256] = {
        0,    6,   13,   19,   25,   31,   38,   44,   50,   56,   62,   68,
       74,   80,   86,   92,   98,  104,  109,  115,  121,  126,  132,  137,
      142,  147,  152,  157,  162,  167,  172,  177,  181,  185,  190,  194,
      198,  202,  206,  209,  213,  216,  220,  223,  226,  229,  231,  234,
      237,  239,  241,  243,  245,  247,  248,  250,  251,  252,  253,  254,
      255,  255,  255,  255,  255,  255,  255,  255,  255,  254,  253,  252,
      251,  250,  248,  247,  245,  243,  241,  239,  237,  234,  231,  229,
      226,  223,  220,  216,  213,  209,  206,  202,  198,  194,  190,  185,
      181,  177,  172,  167,  162,  157,  152,  147,  142,  137,  132,  126,
      121,  115,  109,  104,   98,   92,   86,   80,   74,   68,   62,   56,
       50,   44,   38,   31,   25,   19,   13,    6,    0,   -6,  -13,  -19,
      -25,  -31,  -38,  -44,  -50,  -56,  -62,  -68,  -74,  -80,  -86,  -92,
      -98, -104, -109, -115, -121, -126, -132, -137, -142, -147, -152, -157,
     -162, -167, -172, -177, -181, -185, -190, -194, -198, -202, -206, -209,
     -213, -216, -220, -223, -226, -229, -231, -234, -237, -239, -241, -243,
     -245, -247, -248, -250, -251, -252, -253, -254, -255, -255, -255, -255,
     -255, -255, -255, -255, -255, -254, -253, -252, -251, -250, -248, -247,
     -245, -243, -241, -239, -237, -234, -231, -229, -226, -223, -220, -216,
     -213, -209, -206, -202, -198, -194, -190, -185, -181, -177, -172, -167,
     -162, -157, -152, -147, -142, -137, -132, -126, -121, -115, -109, -104,
      -98,  -92,  -86,  -80,  -74,  -68,  -62,  -56,  -50,  -44,  -38,  -31,
      -25,  -19,  -13,   -6,
};

static uint8_t  s_gbuf[GEO_N];      /* 输出缓冲（15×16） */
static uint8_t  s_hbuf[GEO_HN];     /* 超采样缓冲（45×48） */
static uint8_t  s_anim;             /* 0 球 / 1 立方体 */
static uint8_t  s_tail = 2u;        /* 拖尾：0 = 不拖尾，1..8 越大尾巴越长 */
static uint8_t  s_ballR = GEO_BALL_RDEF;
static uint8_t  s_kick  = GEO_KICK_DEF;

/* 水平方向阻尼：竖直方向永远给满力（球要一直有精神），
 * 横向则每撞一次衰减一点 —— 否则左右来回弹像打砖块。
 * s_hdamp 是 Q8.8 的"水平弹性"：256 = 100%，最低 GEO_HDAMP_MIN。
 * 不撞墙时会缓慢恢复，所以只是"横向慢慢安静下来"，不是永久变懒。 */
static uint16_t s_hdamp = 256u;
static uint8_t  s_damp  = GEO_DAMP_DEF;
static uint16_t s_frames;
static uint16_t s_ringR;                /* 扩散环半径（Q8.8） */
static uint16_t s_phase;
static uint32_t s_nextMs;
static uint8_t  s_geoInited;

/* 球状态（Q8.8，单位 = 像素） */
static int16_t s_bx, s_by, s_bvx, s_bvy;

static int16_t geo_sin(uint16_t a) { return s_geo_sin[a & 0xFFu]; }
static int16_t geo_cos(uint16_t a) { return s_geo_sin[(a + 64u) & 0xFFu]; }

static void geo_clear(void)
{
    uint16_t i;

    for (i = 0u; i < GEO_N; i++) {
        s_gbuf[i] = 0u;
    }
}

static void h_clear(void)
{
    uint16_t i;

    for (i = 0u; i < GEO_HN; i++) {
        s_hbuf[i] = 0u;
    }
}

/* 在超采样缓冲上点一个子像素（坐标是**子像素整数**） */
static void h_plot(int16_t x, int16_t y, uint8_t lv)
{
    uint8_t *p;

    if ((x < 0) || (y < 0) || (x >= (int16_t) GEO_HW) || (y >= (int16_t) GEO_HH)) {
        return;
    }
    p = &s_hbuf[((uint16_t) y * GEO_HW) + (uint16_t) x];
    if (lv > *p) {
        *p = lv;
    }
}

/* 画一个"粗点"：GEO_SUP×GEO_SUP 的小方块 —— 3 倍超采样下正好约 1 个输出像素宽，
 * 既不会因为太细而变暗（1/9 覆盖度只有 28 灰），边缘又保留抗锯齿。 */
static void h_dot(int16_t x, int16_t y, uint8_t lv)
{
    int16_t dx, dy;

    for (dy = 0; dy < (int16_t) GEO_SUP; dy++) {
        for (dx = 0; dx < (int16_t) GEO_SUP; dx++) {
            h_plot((int16_t) (x + dx), (int16_t) (y + dy), lv);
        }
    }
}

/* Bresenham 直线（子像素坐标） */
static void h_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint8_t lv)
{
    int16_t dx = (int16_t) ((x1 > x0) ? (x1 - x0) : (x0 - x1));
    int16_t dy = (int16_t) ((y1 > y0) ? (y0 - y1) : (y1 - y0));
    int16_t sx = (int16_t) ((x0 < x1) ? 1 : -1);
    int16_t sy = (int16_t) ((y0 < y1) ? 1 : -1);
    int16_t err = (int16_t) (dx + dy);

    for (;;) {
        int16_t e2;

        h_dot(x0, y0, lv);
        if ((x0 == x1) && (y0 == y1)) {
            break;
        }
        e2 = (int16_t) (err << 1);
        if (e2 >= dy) {
            err = (int16_t) (err + dy);
            x0  = (int16_t) (x0 + sx);
        }
        if (e2 <= dx) {
            err = (int16_t) (err + dx);
            y0  = (int16_t) (y0 + sy);
        }
    }
}

/* 实心圆（子像素坐标，Q8.8），带 1 子像素的软边 */
static void h_disc(int32_t cx, int32_t cy, int32_t r, uint8_t lv)
{
    int32_t x0 = (cx - r) >> 8;
    int32_t x1 = (cx + r) >> 8;
    int32_t y0 = (cy - r) >> 8;
    int32_t y1 = (cy + r) >> 8;
    int32_t r2 = r * r;
    int32_t edge = 2 * r + 256;
    int16_t x, y;

    if (x0 < 0) { x0 = 0; }
    if (y0 < 0) { y0 = 0; }
    if (x1 >= (int32_t) GEO_HW) { x1 = (int32_t) GEO_HW - 1; }
    if (y1 >= (int32_t) GEO_HH) { y1 = (int32_t) GEO_HH - 1; }

    for (y = (int16_t) y0; y <= (int16_t) y1; y++) {
        for (x = (int16_t) x0; x <= (int16_t) x1; x++) {
            int32_t dx = ((int32_t) x << 8) - cx;
            int32_t dy = ((int32_t) y << 8) - cy;
            int32_t d2 = (dx * dx) + (dy * dy);

            if (d2 <= r2) {
                h_plot(x, y, lv);
            } else if (d2 <= (r2 + edge)) {
                h_plot(x, y, (uint8_t) (lv / 3u));
            }
        }
    }
}

/* 超采样缓冲 → 15×16：3×3 平均，得到 9 级覆盖度 */
static void geo_compose(void)
{
    uint8_t x, y, dx, dy;

    for (y = 0u; y < GEO_H; y++) {
        for (x = 0u; x < GEO_W; x++) {
            uint8_t *dst = &s_gbuf[((uint16_t) y * GEO_W) + x];
            uint16_t sum = 0u;
            uint8_t  v;

            for (dy = 0u; dy < (uint8_t) GEO_SUP; dy++) {
                const uint8_t *row =
                    &s_hbuf[(((uint16_t) y * GEO_SUP + dy) * GEO_HW) + ((uint16_t) x * GEO_SUP)];

                for (dx = 0u; dx < (uint8_t) GEO_SUP; dx++) {
                    sum = (uint16_t) (sum + row[dx]);
                }
            }

            v = (uint8_t) (sum / GEO_SUP2);
            if (v > *dst) {
                *dst = v;
            }
        }
    }
}

/* 拖尾：每帧 buf -= buf >> tail；tail = 0 直接清空 */
static void geo_fade(void)
{
    uint16_t i;
    uint8_t  k = s_tail;

    if (k == 0u) {
        geo_clear();
        return;
    }
    for (i = 0u; i < GEO_N; i++) {
        uint8_t v = s_gbuf[i];

        s_gbuf[i] = (uint8_t) (v - (uint8_t) (v >> k));
    }
}

static void geo_flush(void)
{
    uint8_t x, y, lv;

    LedMatrix_Clear();
    for (y = 0u; y < GEO_H; y++) {
        for (x = 0u; x < GEO_W; x++) {
            lv = s_gbuf[((uint16_t) y * GEO_W) + x];
            if (lv != 0u) {
                LedMatrix_SetCellLevel((uint8_t) (x + 1u), (uint8_t) (y + 1u), lv);
            }
        }
    }
    LedMatrix_Flush();
}

/* ---- 0：弹跳球（吃 IMU 重力）---- */
static void geo_reset_ball(void)
{
    s_bx    = (int16_t) (7 * 256);
    s_by    = (int16_t) (2 * 256);
    s_bvx   = 0;
    s_bvy   = 0;
    s_hdamp = 256u;                     /* 重新开始：水平弹性满格 */
}

static void geo_ball(const app_in_t *in)
{
    /* IMU 重力：in->ax_slow 是 Q24 的"模拟单位"，1g ≈ 9.8（见 app.c 的 App_MgToSim）。
     * 右移 16 位得到 Q8.8（9.8 → 2509），再除以 40 ≈ 0.24 px/帧² 的加速度。
     * IMU 没通时上层给的是"假重力"（同样 9.8 量级、方向会变），所以照样能弹。 */
    int32_t gx = (int32_t) (in->ax_slow >> 16);
    int32_t gy = (int32_t) (in->ay_slow >> 16);
    int16_t r  = (int16_t) (s_ballR * 256);
    int16_t lo = r;
    int16_t hi_x = (int16_t) (((GEO_W - 1u) * 256) - r);
    int16_t hi_y = (int16_t) (((GEO_H - 1u) * 256) - r);

    s_bvx = (int16_t) (s_bvx + (gx / 40));
    s_bvy = (int16_t) (s_bvy + (gy / 40));

    /* 空气阻力：否则来回弹会越弹越快 */
    s_bvx = (int16_t) (s_bvx - (s_bvx >> 6));
    s_bvy = (int16_t) (s_bvy - (s_bvy >> 6));

    s_bx = (int16_t) (s_bx + s_bvx);
    s_by = (int16_t) (s_by + s_bvy);

    /* 撞墙：**给固定的力**弹起（不是简单的速度反向）。
     *
     * 为什么必须这样：球现在吃 IMU 重力，倾斜时会被持续压向低位那面墙。
     * 单纯"反向 + 损失 20%"会越弹越小，最后贴在角上不动 —— 看起来像卡死。
     * 这里每次接触都赋一个固定速度（gkick，1..8 px/帧），保证一直有精神。
     */
    {
        int16_t kick = (int16_t) (s_kick * 256);
        int16_t hx   = (int16_t) (((int32_t) kick * (int32_t) s_hdamp) >> 8);

        if (s_bx < lo) {
            s_bx  = lo;
            s_bvx = hx;                     /* 横向：按当前弹性衰减后的力 */
            if ((s_damp != 0u) && (s_hdamp > GEO_HDAMP_MIN)) {
                uint16_t d = (uint16_t) (s_damp * 4u);

                s_hdamp = (s_hdamp > (GEO_HDAMP_MIN + d))
                          ? (uint16_t) (s_hdamp - d) : GEO_HDAMP_MIN;
            }
        } else if (s_bx > hi_x) {
            s_bx  = hi_x;
            s_bvx = (int16_t) (-hx);
            if ((s_damp != 0u) && (s_hdamp > GEO_HDAMP_MIN)) {
                uint16_t d = (uint16_t) (s_damp * 4u);

                s_hdamp = (s_hdamp > (GEO_HDAMP_MIN + d))
                          ? (uint16_t) (s_hdamp - d) : GEO_HDAMP_MIN;
            }
        }

        if (s_by < lo) {
            s_by  = lo;
            s_bvy = kick;                   /* 竖直：永远给满力 */
        } else if (s_by > hi_y) {
            s_by  = hi_y;
            s_bvy = (int16_t) (-kick);
        }
    }

    /* 一段时间没横向撞墙就慢慢恢复弹性 */
    if ((s_hdamp < 256u) && ((s_frames & 0x0Fu) == 0u)) {
        s_hdamp = (uint16_t) (s_hdamp + 4u);
        if (s_hdamp > 256u) {
            s_hdamp = 256u;
        }
    }

    h_disc((int32_t) s_bx * (int32_t) GEO_SUP,
           (int32_t) s_by * (int32_t) GEO_SUP,
           (int32_t) r * (int32_t) GEO_SUP, 255);
}

/* ---- 2：中心扩散环 + 方框内不断反弹 ----
 *
 * 做法是**镜像法**：不存速度、不做碰撞检测，只让半径一直涨，
 * 然后把"源点到像素的距离"按边框折叠（billiard folding）：
 *
 *     t = (px - sx) mod 2L;  t > L 时 t = 2L - t
 *
 * 这样距离场天然在边框上折返，看起来就是"环撞到方框又弹回来"。
 * 一维折叠后再合成二维距离 —— 因为镜像源是二维网格，
 * min_grid(a²+b²) = min_a(a²) + min_b(b²)，所以折叠可以分轴做。
 */

/* 整数平方根（Q16.16 → Q8.8），逐位逼近，不用除法 */
static uint32_t geo_isqrt(uint32_t v)
{
    uint32_t res = 0u;
    uint32_t bit = (uint32_t) 1u << 30;

    while (bit > v) {
        bit >>= 2;
    }
    while (bit != 0u) {
        if (v >= (res + bit)) {
            v -= (res + bit);
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

/* 一维折叠：把 |px - 源| 折进 [0, L] */
static int32_t geo_fold(int32_t d, int32_t twoL)
{
    int32_t t = d % twoL;

    if (t < 0) {
        t += twoL;
    }
    if (t > (twoL / 2)) {
        t = twoL - t;
    }
    return t;
}

/* 往输出缓冲（15×16）点一个像素，取最亮 */
static void geo_plot_px(uint8_t x, uint8_t y, uint8_t lv)
{
    uint8_t *p;

    if ((x >= GEO_W) || (y >= GEO_H)) {
        return;
    }
    p = &s_gbuf[((uint16_t) y * GEO_W) + x];
    if (lv > *p) {
        *p = lv;
    }
}

static void geo_ring(void)
{
    int32_t lx2 = (int32_t) (((GEO_W - 1u) << 8) * 2);   /* 2L，Q8.8 */
    int32_t ly2 = (int32_t) (((GEO_H - 1u) << 8) * 2);
    int32_t sx  = (int32_t) (7 * 256);                   /* 圆心：中间线 */
    int32_t sy  = (int32_t) (7 * 256 + 128);
    int32_t r   = (int32_t) s_ringR;
    uint8_t x, y;

    for (y = 0u; y < GEO_H; y++) {
        for (x = 0u; x < GEO_W; x++) {
            int32_t px = (int32_t) x << 8;
            int32_t py = (int32_t) y << 8;
            int32_t dx = geo_fold(px - sx, lx2);
            int32_t dy = geo_fold(py - sy, ly2);
            int32_t d  = (int32_t) geo_isqrt((uint32_t) ((dx * dx) + (dy * dy)));
            int32_t diff = d - r;

            if (diff < 0) {
                diff = -diff;
            }
            if (diff <= GEO_RING_W) {
                geo_plot_px(x, y, (uint8_t) (255 - ((diff * 255) / GEO_RING_W)));
            }
        }
    }

    s_ringR = (uint16_t) ((s_ringR + GEO_RING_STEP) % GEO_RING_MAX);
}

/* ---- 1：旋转立方体 ---- */
static const int16_t s_cubeV[8][3] = {
    { -1024, -1024, -1024 }, { 1024, -1024, -1024 },
    {  1024,  1024, -1024 }, { -1024,  1024, -1024 },
    { -1024, -1024,  1024 }, { 1024, -1024,  1024 },
    {  1024,  1024,  1024 }, { -1024,  1024,  1024 }
};

static const uint8_t s_cubeE[12][2] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 0 },
    { 4, 5 }, { 5, 6 }, { 6, 7 }, { 7, 4 },
    { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 }
};

static void geo_cube(void)
{
    int16_t spx[8];
    int16_t spy[8];
    int16_t cyw = geo_cos(s_phase);
    int16_t syw = geo_sin(s_phase);
    int16_t cpt = geo_cos((uint16_t) (s_phase >> 1));
    int16_t spt = geo_sin((uint16_t) (s_phase >> 1));
    uint8_t i;

    for (i = 0u; i < 8u; i++) {
        int32_t x  = s_cubeV[i][0];
        int32_t y  = s_cubeV[i][1];
        int32_t z  = s_cubeV[i][2];
        int32_t x1 = (x * cyw - z * syw) >> 8;
        int32_t z1 = (x * syw + z * cyw) >> 8;
        int32_t y2 = (y * cpt - z1 * spt) >> 8;
        int32_t z2 = (y * spt + z1 * cpt) >> 8;
        int32_t sc = 1024 + (z2 >> 2);              /* 轻微透视：近大远小 */
        int32_t px = (int32_t) (7 * 256) + ((x1 * sc) >> 10);
        int32_t py = (int32_t) (7 * 256) + 128 + ((y2 * sc) >> 10);

        /* Q8.8 像素 → 子像素（乘超采样倍数），h_line 收整数子像素 */
        spx[i] = (int16_t) (((px * (int32_t) GEO_SUP) + 128) >> 8);
        spy[i] = (int16_t) (((py * (int32_t) GEO_SUP) + 128) >> 8);
    }

    for (i = 0u; i < 12u; i++) {
        uint8_t a = s_cubeE[i][0];
        uint8_t b = s_cubeE[i][1];

        h_line(spx[a], spy[a], spx[b], spy[b], 255);
    }
}

/* ==================== 动画表 ====================
 *
 * 每个 frame 函数**自己负责整条流水线**：
 *   超采样类（球 / 立方体）：h_clear() → 画 → geo_compose()
 *   直接类（扩散环）：直接画 s_gbuf（它本身就是连续的距离场，超采样没意义）
 *
 * 这样 tick 里不再有 "if (s_anim == 2)" 这种特例，也不会出现"加了新动画
 * 却忘了加 case、被 default 静默当成立方体"的情况 —— 表是定长的，
 * 越界在下标检查那一步就被拒绝了。
 *
 * 表里只有两个函数指针，const 放 Flash，RAM 开销 0。
 */
typedef struct {
    void (*frame)(const app_in_t *in);   /* 画一帧（含自己的 clear / compose）*/
    void (*reset)(void);                 /* 进入/切换时重置，0 = 无状态 */
} geo_anim_t;

static void anim_ball_frame(const app_in_t *in)
{
    h_clear();
    geo_ball(in);
    geo_compose();
}

static void anim_cube_frame(const app_in_t *in)
{
    (void) in;                           /* 立方体不吃重力 */
    h_clear();
    geo_cube();
    geo_compose();
}

static void geo_reset_ring(void)
{
    s_ringR = 0u;
}

static void anim_ring_frame(const app_in_t *in)
{
    (void) in;
    geo_ring();                          /* 直接画 s_gbuf，不超采样 */
}

static const geo_anim_t s_anims[] = {
    { anim_ball_frame, geo_reset_ball },   /* 0 弹跳球（吃 IMU 重力）*/
    { anim_cube_frame, 0              },   /* 1 旋转立方体 */
    { anim_ring_frame, geo_reset_ring },   /* 2 扩散环 */
};

#define GEO_ANIM_COUNT  ((uint8_t) (sizeof(s_anims) / sizeof(s_anims[0])))

/* 给 app.c 的 route_item_count() 用：几何大类的项数 = 表长，加动画自动跟着变 */
uint8_t GeomScene_AnimCount(void) { return GEO_ANIM_COUNT; }

/* ==================== 场景 4 接口 ==================== */

void GeomScene_Select(uint8_t n)
{
    if (n >= GEO_ANIM_COUNT) {           /* 越界拒绝，不再靠 default 兜底 */
        return;
    }
    s_anim      = n;
    s_geoInited = 0u;
    geo_clear();
}

void GeomScene_Enter(void)
{
    s_geoInited = 0u;
    geo_clear();
    s_nextMs = tick_ms;
}

void GeomScene_Tick(const app_in_t *in)
{
    if (s_geoInited == 0u) {
        s_geoInited = 1u;
        s_phase     = 0u;
        s_frames    = 0u;
        if (s_anims[s_anim].reset != 0) {
            s_anims[s_anim].reset();
        }
        s_nextMs = in->now_ms;
    }

    if ((int32_t) (in->now_ms - s_nextMs) < 0) {
        return;                         /* 还没到下一帧 */
    }
    s_nextMs += GEO_FPS_MS;

    geo_fade();
    s_anims[s_anim].frame(in);          /* 动画自己负责 clear / compose */
    geo_flush();

    s_phase = (uint16_t) ((s_phase + 2u) & 0xFFu);
    s_frames++;
}

void GeomScene_Exit(void)
{
    /* 缓冲留着，切回来接着画 */
}

uint8_t GeomScene_Cmd(uint16_t id, const int32_t *arg)
{
    switch (id) {
        case DBG_CMD_GEOM_ANIM:
            if ((arg[0] >= 0) && (arg[0] < (int32_t) GEO_ANIM_COUNT)) {
                GeomScene_Select((uint8_t) arg[0]);
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_GEOM_TAIL:
            if ((arg[0] >= 0) && (arg[0] <= 8)) {
                s_tail = (uint8_t) arg[0];
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_GEOM_BALL:
            if ((arg[0] >= 1) && (arg[0] <= (int32_t) GEO_BALL_RMAX)) {
                s_ballR = (uint8_t) arg[0];
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_GEOM_KICK:
            if ((arg[0] >= 1) && (arg[0] <= (int32_t) GEO_KICK_MAX)) {
                s_kick = (uint8_t) arg[0];
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_GEOM_DAMP:
            if ((arg[0] >= 0) && (arg[0] <= (int32_t) GEO_DAMP_MAX)) {
                s_damp = (uint8_t) arg[0];
                if (s_damp == 0u) {
                    s_hdamp = 256u;         /* 关掉阻尼：立刻恢复满弹性 */
                }
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        default:
            return 0u;
    }
}