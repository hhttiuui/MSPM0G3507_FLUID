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
