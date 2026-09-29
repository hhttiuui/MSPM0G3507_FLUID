/*
 * oled_selftest.c —— OLED 批量刷屏 + 网格映射验证
 *
 * 关键映射（后续流体渲染会原样沿用）：
 *   OLED 一页 = 8 像素高，一个格子 = 4 像素高、4 像素宽，
 *   所以「一页正好装 2 行格子」：
 *       格子行 r 所在的页   = r / 2
 *       格子行 r 占页字节   = r 偶数 -> 低 4 位(0x0F)；r 奇数 -> 高 4 位(0xF0)
 *       格子列 c 占的列     = 4c ~ 4c+3
 *
 * ⚠️ SSD1306 的 bit0 是否对应"最上面一行"、col0 是否在左边，
 *    取决于 OLED_Init 里的 0xA1/0xC8(列/行镜像)设置 —— 本自检就是用来
 *    把这件事一次性确认掉的：看到的是外框+左上到右下斜线，就说明映射正确。
 */
#include "ti_msp_dl_config.h"
#include "clock.h"
#include "oled_hardware_i2c.h"
#include "oled_selftest.h"

/* 画布：15 列 × 16 行格子，每格 4×4 px → 60×64 px */
#define TEST_COLS     15u
#define TEST_ROWS     16u
#define CANVAS_COLS   60u
#define CANVAS_PAGES   8u

/* HUD 文本区（右侧 64..127） */
#define HUD_LABEL_X   64u
#define HUD_VALUE_X   92u

/* 页面缓冲：一页画布 = 60 字节 */
static uint8_t s_pageBuf[CANVAS_COLS];

/* 测试用的"格子"数组，等价于流体里的 cellType[]：非 0 = 点亮 */
static uint8_t s_cells[TEST_COLS * TEST_ROWS];

static uint32_t s_benchOnePageMs;   /* 100 次单页推送总耗时 */
static uint32_t s_benchFrameMs;     /* 20 帧完整画布总耗时   */
static uint32_t s_avgX10;           /* 平均帧耗时 ×10 (0.1ms) */
static uint32_t s_fps;
static uint32_t s_errCount;

/* 组装一个有确定形状的图案：外框 + 45 度斜线 + 一个小球 */
static void TestBuildCells(uint8_t ballC, uint8_t ballR)
{
    uint8_t c, r;

    for (r = 0u; r < TEST_ROWS; r++) {
        for (c = 0u; c < TEST_COLS; c++) {
            uint8_t on = 0u;

            if ((r == 0u) || (r == (TEST_ROWS - 1u)) ||
                (c == 0u) || (c == (TEST_COLS - 1u))) {
                on = 1u;                        /* 外框 */
            }
            if (c == r) {
                on = 1u;                        /* 左上 → 右下 对角线 */
            }
            if ((c == ballC) && (r == ballR)) {
                on = 1u;                        /* 移动的小球 */
            }

            s_cells[(uint16_t)r * TEST_COLS + c] = on;
        }
    }
}

/* 把 s_cells 渲染成 8 页缓冲并逐页推送（这就是流体渲染的骨架） */
static void OledDraw_Grid(void)
{
    uint8_t page, c, r0, r1, lo, hi;

    for (page = 0u; page < CANVAS_PAGES; page++) {
        r0 = (uint8_t)(page * 2u);          /* 本页上半个字节对应的格子行 */
        r1 = (uint8_t)(page * 2u + 1u);     /* 本页下半个字节对应的格子行 */

        for (c = 0u; c < TEST_COLS; c++) {
            lo = (s_cells[(uint16_t)r0 * TEST_COLS + c] != 0u) ? 0x0Fu : 0x00u;
            hi = (s_cells[(uint16_t)r1 * TEST_COLS + c] != 0u) ? 0xF0u : 0x00u;

            s_pageBuf[c * 4u + 0u] = (uint8_t)(lo | hi);
            s_pageBuf[c * 4u + 1u] = (uint8_t)(lo | hi);
            s_pageBuf[c * 4u + 2u] = (uint8_t)(lo | hi);
            s_pageBuf[c * 4u + 3u] = (uint8_t)(lo | hi);
        }

        if (OLED_PushPageRange(page, 0u, (uint8_t)(CANVAS_COLS - 1u), s_pageBuf) != 0) {
            s_errCount++;
        }
    }
}

/* ---------------- P1：刷屏耗时基准 ---------------- */
static void OledBench_Run(void)
{
    uint16_t i;
    uint8_t  page;
    uint32_t t0, t1;

    /* 缓冲清零，基准测试期间屏幕上先保持干净 */
    for (i = 0u; i < CANVAS_COLS; i++) {
        s_pageBuf[i] = 0x00u;
    }

    /* 1) 单页推送 × 100 */
    t0 = tick_ms;
    for (i = 0u; i < 100u; i++) {
        if (OLED_PushPageRange(0u, 0u, (uint8_t)(CANVAS_COLS - 1u), s_pageBuf) != 0) {
            s_errCount++;
        }
    }
    t1 = tick_ms;
    s_benchOnePageMs = t1 - t0;

    /* 2) 完整画布（8 页）× 20 */
    t0 = tick_ms;
    for (i = 0u; i < 20u; i++) {
        for (page = 0u; page < CANVAS_PAGES; page++) {
            if (OLED_PushPageRange(page, 0u, (uint8_t)(CANVAS_COLS - 1u), s_pageBuf) != 0) {
                s_errCount++;
            }
        }
    }
    t1 = tick_ms;
    s_benchFrameMs = t1 - t0;
}

/* ---------------- HUD ---------------- */
static void OledHud_Static(void)
{
    OLED_ShowString(HUD_LABEL_X, 0, (uint8_t *)"OLED BURST", 8);
    OLED_ShowString(HUD_LABEL_X, 1, (uint8_t *)"PG1=", 8);
    OLED_ShowString(HUD_LABEL_X, 2, (uint8_t *)"FRM=", 8);
    OLED_ShowString(HUD_LABEL_X, 3, (uint8_t *)"AVG=", 8);
    OLED_ShowString(HUD_LABEL_X, 4, (uint8_t *)"FPS=", 8);
    OLED_ShowString(HUD_LABEL_X, 5, (uint8_t *)"ERR=", 8);
    OLED_ShowString(HUD_LABEL_X, 6, (uint8_t *)"60x64PX", 8);
    OLED_ShowString(HUD_LABEL_X, 7, (uint8_t *)"15x16GRD", 8);
}

static void OledHud_Dynamic(void)
{
    OLED_ShowNum(HUD_VALUE_X, 1, s_benchOnePageMs, 4, 8);
    OLED_ShowNum(HUD_VALUE_X, 2, s_benchFrameMs,   4, 8);
    OLED_ShowNum(HUD_VALUE_X, 3, s_avgX10,         4, 8);
    OLED_ShowNum(HUD_VALUE_X, 4, s_fps,            4, 8);
    OLED_ShowNum(HUD_VALUE_X, 5, s_errCount,       4, 8);
}

/* ---------------- 入口 ---------------- */
void OledDrawTest_Run(void)
{
    uint32_t frameCnt = 0u;
    uint32_t tStart;
    uint8_t  moveDiv = 0u;
    int      ballC = 3, ballR = 3;
    int      dxStep = 1, dyStep = 1;

    OledBench_Run();
    OledHud_Static();
    OledHud_Dynamic();

    tStart = tick_ms;

    for (;;) {
        /* 76fps 下小球动得太快看不清，每 6 帧走一格 */
        if (++moveDiv >= 6u) {
            moveDiv = 0u;

            ballC += dxStep;
            ballR += dyStep;

            if (ballC <= 1)                      { ballC = 1;                     dxStep = -dxStep; }
            if (ballC >= (int)TEST_COLS - 2)     { ballC = (int)TEST_COLS - 2;    dxStep = -dxStep; }
            if (ballR <= 1)                      { ballR = 1;                     dyStep = -dyStep; }
            if (ballR >= (int)TEST_ROWS - 2)     { ballR = (int)TEST_ROWS - 2;    dyStep = -dyStep; }
        }

        TestBuildCells((uint8_t)ballC, (uint8_t)ballR);
        OledDraw_Grid();

        /* 每 20 帧统计一次：平均帧耗时(0.1ms) 与 FPS */
        frameCnt++;
        if (frameCnt >= 20u) {
            uint32_t elapsed = tick_ms - tStart;

            if (elapsed > 0u) {
                s_avgX10 = (elapsed * 10u) / frameCnt;
                s_fps    = (frameCnt * 1000u) / elapsed;
            }

            frameCnt = 0u;
            tStart   = tick_ms;
            OledHud_Dynamic();
        }
    }
}
