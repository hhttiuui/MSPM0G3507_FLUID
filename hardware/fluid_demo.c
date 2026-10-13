/*
 * fluid_demo.c —— 流体模拟 → LED 点阵（**暂时不接 IMU**）
 *
 * 职责划分：
 *   fluid.c        纯算法，零外设依赖（可以原样丢到 PC 上跑）
 *   led_matrix.c   面板映射层：(列,行) → 芯片显示 RAM 位
 *   led_show.c     点阵综合演示（另一个可选程序，见 LED_SHOW_ENABLE）
 *   fluid_demo.c   本文件：算法 ↔ 点阵的胶水层
 *                  ① cellType[] → LedMatrix → 推屏
 *                  ② 假重力：方向每 DEMO_TILT_MS 换一次，让液体四个方向都流一遍
 *                  ③ 观测量全部在 dbg.fluid（没有屏，参数只能这么看）
 *
 * ⚠ 还没接 IMU：现在靠"假重力"驱动。等确认了 IMU 身份（看 dbg.who.id 是 0x6A
 *   还是 0x24）再接真实读数 —— 接的时候只需替换下面取 (ax, ay) 那几行。
 *
 * ⚠ 尺寸对得上：流体可视区 15 列 × 16 行，面板 16 列 × 15 行，两者是转置关系，
 *   由 led_matrix.h 的 LED_ORIENT_SWAP_XY 处理（方向和机械安装有关，看图案段调）。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "debug.h"
#include "fluid.h"
#include "fluid_demo.h"
#include "led_matrix.h"
#include "power.h"

/* ==================== 参数 ==================== */

/* 假重力方向保持多久（ms）：4 个方向 下→左→上→右 循环 */
#define DEMO_TILT_MS        2000u

/* 时间步长：原版实测 0.032 时手感最好（一步物理约 28ms，dt 推 32ms ≈ 1.1 倍实时）*/
#define DEMO_DT             _IQ(0.032f)

/* 压力求解迭代次数（越大液体越"硬"、也越慢）*/
#define DEMO_GRID_ITER      12u

/* 每多少帧统计一次 dbg.fluid 的耗时/帧率 */
#define DEMO_STAT_FRAMES    20u

/* 假重力的 4 个方向（模拟单位）。约定：x 正 = 屏幕右，y 正 = 屏幕下 */
static const _iq s_tiltAcc[4][2] = {
    { _IQ(0.0f),  _IQ(9.8f) },      /* 0 下 */
    { _IQ(-9.8f), _IQ(0.0f) },      /* 1 左 */
    { _IQ(0.0f),  _IQ(-9.8f) },     /* 2 上 */
    { _IQ(9.8f),  _IQ(0.0f) },      /* 3 右 */
};

/* ==================== 渲染 ==================== */

/* cellType[] → 一帧显示缓冲 → 推屏。
 * 非空气（液体 + 固体边框）都点亮，和原来的显示约定一致。 */
static void FluidDemo_Push(void)
{
    const uint8_t *cellType = Fluid_GetCellType();
    uint8_t x, y;

    LedMatrix_Clear();

    for (x = 1u; x <= LED_APP_COLS; x++) {          /* 15 列 */
        for (y = 1u; y <= LED_APP_ROWS; y++) {      /* 16 行 */
            if (cellType[CELL_INDEX((uint32_t) x, (uint32_t) y)] != AIR_CELL) {
                LedMatrix_SetCell(x, y);
            }
        }
    }

    LedMatrix_Flush();      /* 整帧一次发完（FD 00 + 80 00 ×32，约 144µs）*/
}

/* 模拟单位 ×10 取整，方便在 dbg 里当整数看 */
static int32_t FluidDemo_SimX10(_iq v)
{
    return (int32_t) (((int64_t) v * 10) / IQ_ONE);
}

/* ==================== 数学层自检 ==================== */

/* 先确认 _IQmpy/_IQdiv/_IQsqrt 都是对的，再让仿真跑，免得白排查。
 * 失败没法在屏幕上写字了，所以用"整屏点亮 + 停下"当报错信号，结果也记在
 * dbg.fluid.iq_ok 里。 */
static uint8_t FluidDemo_IqSelfTest(void)
{
    const _iq tol = (IQ_ONE / 1024);        /* ≈1e-3，Q24 下实际误差远小于此 */
    _iq t;

    t = _IQmpy(_IQ(2.0f), _IQ(3.0f));                 /* 乘法 */
    if (t < (_IQ(6.0f) - tol) || t > (_IQ(6.0f) + tol)) return 0u;

    t = _IQdiv(_IQ(6.0f), _IQ(3.0f));                 /* 除法 */
    if (t < (_IQ(2.0f) - tol) || t > (_IQ(2.0f) + tol)) return 0u;

    t = _IQsqrt(_IQ(9.0f));                           /* 开方 */
    if (t < (_IQ(3.0f) - tol) || t > (_IQ(3.0f) + tol)) return 0u;

    t = _IQmpy(_IQ(-0.5f), _IQ(4.0f));                /* 负数乘法 */
    if (t < (_IQ(-2.0f) - tol) || t > (_IQ(-2.0f) + tol)) return 0u;

    t = _IQdiv(_IQ(-6.0f), _IQ(3.0f));                /* 负数除法 */
    if (t < (_IQ(-2.0f) - tol) || t > (_IQ(-2.0f) + tol)) return 0u;

    if (_IQI(5) != _IQ(5.0f)) return 0u;              /* 整数转换 */
    if (-_IQI(3) != _IQ(-3.0f)) return 0u;
    if (_IQint(_IQ(7.5f)) != 7) return 0u;

    return 1u;
}

/* ==================== 入口 ==================== */

void FluidDemo_Run(void)
{
    uint32_t tiltStart;
    uint32_t statStart;
    uint32_t frameCnt = 0u;
    uint32_t accSim = 0u;
    uint8_t  tilt = 0u;
    _iq      ax, ay;
    uint32_t t0, t1;

    LedMatrix_Init();                       /* 芯片初始化 + 清屏 */

    /* 数学层自检：坏了就把整屏点亮停住（还能长按 USER1 关机）*/
    dbg.fluid.iq_ok = FluidDemo_IqSelfTest();
    if (dbg.fluid.iq_ok == 0u) {
        LedMatrix_FillAll(1u);
        LedMatrix_Flush();
        for (;;) {
            PowerKey_CheckShutdown();
            __WFI();
        }
    }

    /* 流体初始化 + 预热一步（和原版 main 对齐）*/
    Fluid_Init();
    Fluid_SetDt(DEMO_DT);
    Fluid_SetGridIter(DEMO_GRID_ITER);
    Fluid_Step(_IQ(0.0f), _IQ(9.8f));

    dbg.fluid.tilt = tilt;
    tiltStart = tick_ms;
    statStart = tick_ms;

    for (;;) {
        /* 一键开关机：USER1 长按 2s 后由 TIMA0 中断拉低 PA22（释放电源自锁），
         * 这里只需要停下来别再干活，安静等硬件断电 */
        PowerKey_CheckShutdown();

        /* ---- 假重力：每 DEMO_TILT_MS 换一个方向（接 IMU 后就换成真实读数）---- */
        if ((uint32_t) (tick_ms - tiltStart) >= DEMO_TILT_MS) {
            tiltStart += DEMO_TILT_MS;
            tilt = (uint8_t) ((tilt + 1u) & 3u);
            dbg.fluid.tilt = tilt;
        }

        ax = s_tiltAcc[tilt][0];
        ay = s_tiltAcc[tilt][1];

        /* ---- 物理一步 ---- */
        t0 = tick_ms;
        Fluid_Step(ax, ay);
        t1 = tick_ms;
        accSim += (t1 - t0);

        /* ---- 渲染 + 推屏（不计入物理耗时）---- */
        FluidDemo_Push();

        /* ---- 每 DEMO_STAT_FRAMES 帧统计一次（原来这些数在 OLED 的 HUD 上）---- */
        frameCnt++;
        if (frameCnt >= DEMO_STAT_FRAMES) {
            uint32_t elapsed = tick_ms - statStart;

            dbg.fluid.sim_ms_x10 = (accSim * 10u) / frameCnt;
            dbg.fluid.fps        = (elapsed > 0u) ? ((frameCnt * 1000u) / elapsed) : 0u;
            dbg.fluid.frames     = Fluid_GetFrameCount();
            dbg.fluid.ax10       = FluidDemo_SimX10(ax);
            dbg.fluid.ay10       = FluidDemo_SimX10(ay);

            accSim   = 0u;
            frameCnt = 0u;
            statStart = tick_ms;

            if (dbg.test.heartbeat != 0u) {
                /* 翻指示灯做心跳：主循环卡住时它就不闪了，一眼能看出来 */
                DL_GPIO_togglePins(LED_PORT, LED_PIN_LED_PIN);
            }
        }
    }
}
