/*
 * fluid_demo.c —— 流体模拟 + OLED 显示 Demo
 *
 * 职责划分：
 *   fluid.c        纯算法，零外设依赖（可单独丢到 PC 上跑）
 *   fluid_demo.c   本文件：算法 ↔ OLED 的胶水层
 *                  ① cellType[] → 8 页显存缓冲 → OLED_PushPageRange()
 *                  ② 读 IMU660RB 的加速度计 → 换算 + 低通 → 驱动流体
 *                  ③ 右侧 HUD：三轴原始值 / 送入流体的值 / 帧耗时 / 帧率
 *
 * 渲染映射（已在 P2 用测试图案验证过方向正确）：
 *   一个格子 = 4×4 px，一页(8px)装 2 行格子
 *       格子行 r → 页 = r/2
 *       r 偶 → 页字节低 4 位(0x0F)；r 奇 → 高 4 位(0xF0)
 *   内部格子 (x, y) x∈[1,15] y∈[1,16] 对应画布列 c=x-1、行 r=y-1
 */
#include "ti_msp_dl_config.h"
#include "clock.h"
#include "oled_hardware_i2c.h"
#include "fluid.h"
#include "fluid_demo.h"
#include "IMU660RB/imu660rb.h"

/* 画布：15 列 × 16 行格子 × 4px = 60×64 px */
#define CANVAS_COLS   60u
#define CANVAS_PAGES   8u

/* HUD 文本区（右侧 64..127） */
#define HUD_LABEL_X   64u
#define HUD_VALUE_X   88u

/* ==================== 加速度计接入 ====================
   这里有两层符号，分开写是因为原因不同：

   【第一层】硬件安装方向：
     IMU660RB 是倒装的（/装在板子背面），传感器三轴读数相对板面坐标全部反向。
     所以统一乘一个符号，后面拿到的就是"板面坐标下的真实加速度"。
     换了安装方向（或换回正面）只需改 ACC_MOUNT_SIGN 一个宏。

   【第二层】板面坐标 → 屏幕坐标：
     取哪个轴、要不要再反号，取决于 IMU 在板面上的朝向，靠猜不准，要靠实测。
     **本板子的实测结论已经写死在 ACC_PERP_AXIS / ACC_SRC_* / ACC_SIGN_* 里了。**
     换板子或换安装方向后的重测流程：
     1) 开机平放（屏幕朝上）静止，看 HUD 的 AX/AY/AZ —— 哪个接近 ±1000，
         那个就是垂直于屏幕平面的轴（它在屏幕平面里没有重力分量，不能拿来用）。
         把它的序号填到 ACC_PERP_AXIS。
     2) ACC_SRC_X / ACC_SRC_Y 填剩下的两个面内轴
     3) 拿起来往某一侧压低，看 X?/Y? 的值往哪边走、方向对不对；
         不对就翻 ACC_SIGN_X / ACC_SIGN_Y。目标是：
         **压低哪一边，液体就往哪一边流**；把板子翻过来拿（屏幕朝下）时，
         液体应该跑到屏幕上方
   约定：流体 ax > 0 = 往屏幕右流；ay > 0 = 往屏幕下流。

   最终：sim_x = ACC_MOUNT_SIGN × ACC_SIGN_X × 传感器 rawX */
#define ACC_MOUNT_SIGN  (-1)   /* -1 = 倒装，三轴全反；正面安装则填 +1 */

/* 垂直于屏幕平面的那个轴（它在屏幕平面内没有重力分量，不能用来驱动液体）。
   实测值：**这块板子是 1（AY）**。
   换板子/换安装方向后怎么重测：开机平放（屏幕朝上），看 HUD 的 AX/AY/AZ ——
   哪个接近 ±1000，那个就是垂轴，把它的序号填到这里；
   然后把 ACC_SRC_X / ACC_SRC_Y 改成剩下的两个面内轴。 */
#define ACC_PERP_AXIS  1

/* 屏幕 x / y 分别取自哪个传感器轴 —— 必须是上面那两个面内轴（≠ ACC_PERP_AXIS）。
   实测结论：垂轴是 AY(1)，所以面内轴是 AX(0) 和 AZ(2)。 */
#define ACC_SRC_X     0        /* 屏幕 x 取自: 0=AX 1=AY 2=AZ */
#define ACC_SRC_Y     2        /* 屏幕 y 取自: 0=AX 1=AY 2=AZ */
#define ACC_SIGN_X   (-1)      /* 符号: +1 或 -1，均为实测值 */
#define ACC_SIGN_Y   (+1)      /* 上下方向反了就把这里翻一下；
                                  不要去动 ACC_MOUNT_SIGN，那个会把 x 也一起翻掉 */

/* 1g = 1000mg ↔ 9.8 个模拟单位（与原版对齐） */
#define ACC_MG_PER_G  1000.0f
#define ACC_G_TO_SIM  9.8f

/* 零偏固定为 0：**不做开机标定**。理由见 FluidDemo_Run 里的注释。
   以后如果量到了实际零偏，直接填到 s_accOfs[] 的初始化列表（单位 mg，顺序 AX/AY/AZ） */

/* 一阶低通：s += (new - s) * alpha。alpha 越小越平滑、但越滞后 */
#define ACC_LPF_ALPHA _IQ(0.35f)

/* 一阶低通：s += (new - s) * alpha。alpha 越小越平滑、但越滞后 */
#define ACC_LPF_ALPHA _IQ(0.35f)

/* ---- 时间步长 dt ----
   实测 dt = 0.032 时手感最好：一步物理约 28ms，dt 推进 32ms → 约 1.1 倍实时。
   要改直接改这里。 */
#define DEMO_DT       _IQ(0.032f)

/* 压力求解迭代次数（越大液体越"硬"、也越慢） */
#define DEMO_GRID_ITER  12u

/* 每多少帧统计并刷新一次 HUD（HUD 用的是慢速逐字节接口，不宜每帧刷） */
#define HUD_UPDATE_FRAMES  20u

static uint8_t s_pageBuf[CANVAS_COLS];

static uint32_t s_err;
static uint32_t s_simMs;          /* 上一统计窗口的平均单步物理耗时 (0.1ms) */
static uint32_t s_fps;

/* 加速度计：三轴原始值(mg，已扣零偏)、零偏、以及滤波后的值(模拟单位) */
static int32_t s_accRaw[3];
static int32_t s_accOfs[3] = { 0, 0, 0 };   /* 固定值，不再开机标定 */
static _iq     s_axFilt;
static _iq     s_ayFilt;

/* 最终生效的源轴（可能被选轴纠错改过，与 ACC_SRC_* 不一定相同）。
   屏幕第 3/4 行的标签 X?/Y? 显示的就是它们，可以当配置回头用。 */
static uint8_t s_srcX = ACC_SRC_X;
static uint8_t s_srcY = ACC_SRC_Y;

/* 垂直于屏幕平面的那个轴（开机平放时 |均值| 最大的轴，即抱走全部重力的轴） */
static uint8_t s_perpAxis;
static uint8_t s_shownPerp = 0xFFu;

/* 上次显示过的值，用来跳过没变化的项（HUD 走慢速逐字节接口） */
static int32_t  s_shownRaw[3] = { 0x7FFFFFFF, 0x7FFFFFFF, 0x7FFFFFFF };
static int32_t  s_shownXs     = 0x7FFFFFFF;
static int32_t  s_shownYs     = 0x7FFFFFFF;
static uint32_t s_shownSim    = 0xFFFFFFFFu;
static uint32_t s_shownFps    = 0xFFFFFFFFu;
static uint8_t  s_hudErrShown;

/* ==================== 渲染 ==================== */

/* 把一个页缓冲组装出来：低半字节 = 偶数行，高半字节 = 奇数行 */
static void FluidDemo_BuildPage(uint8_t page, const uint8_t *cellType)
{
    uint8_t c;
    uint8_t r0 = (uint8_t)(page * 2u);
    uint8_t r1 = (uint8_t)(page * 2u + 1u);

    for (c = 0u; c < 15u; c++) {
        uint8_t lo = (cellType[CELL_INDEX((uint32_t)(c + 1u), (uint32_t)(r0 + 1u))] != AIR_CELL)
                     ? 0x0Fu : 0x00u;
        uint8_t hi = (cellType[CELL_INDEX((uint32_t)(c + 1u), (uint32_t)(r1 + 1u))] != AIR_CELL)
                     ? 0xF0u : 0x00u;

        s_pageBuf[c * 4u + 0u] = (uint8_t)(lo | hi);
        s_pageBuf[c * 4u + 1u] = (uint8_t)(lo | hi);
        s_pageBuf[c * 4u + 2u] = (uint8_t)(lo | hi);
        s_pageBuf[c * 4u + 3u] = (uint8_t)(lo | hi);
    }
}

/* 组装 8 页并推送。返回失败的页数 */
static uint32_t FluidDemo_Push(void)
{
    const uint8_t *cellType = Fluid_GetCellType();
    uint32_t err = 0u;
    uint8_t page;

    for (page = 0u; page < CANVAS_PAGES; page++) {
        FluidDemo_BuildPage(page, cellType);
        if (OLED_PushPageRange(page, 0u, (uint8_t)(CANVAS_COLS - 1u), s_pageBuf) != 0) {
            err++;
        }
    }

    return err;
}

/* ==================== HUD ==================== */

/* 取某个轴的加速度（已扣零偏、已翻转，mg）→ 模拟单位(Q24)，带符号 */
static _iq FluidDemo_AccelToSim(uint8_t axis, int sign)
{
    float v = (float)s_accRaw[axis];

    /* 这里必须写"乘倒数"而不是"除以 1000"：
       clang 在没有 -ffast-math 时不会把浮点常量除法折成乘法，会保留成一次真正的
       __aeabi_fdiv（约 700 周期，还会把 divsf3 拖进镜像）。写成 (1.0f / 常量)
       让编译器在编译期就折成一个常量。 */
    v = (v * (1.0f / ACC_MG_PER_G)) * ACC_G_TO_SIM * 16777216.0f;

    return (sign > 0) ? (_iq)v : (_iq)(-v);
}

/* 模拟单位 ×10 取整，方便当整数显示 */
static int32_t FluidDemo_SimX10(_iq v)
{
    return (int32_t)(((int64_t)v * 10) / IQ_ONE);
}

/* 选轴纠错：ACC_SRC_* 万一选中了垂轴（那上面根本没有面内重力分量，
   而且它还挂着 1g 的均值），就自动换成另一个面内轴。
   三轴索引 0+1+2=3，所以"除 a/b 之外的那个轴" = 3 - a - b。 */
static void FluidDemo_FixAxis(void)
{
    if (s_srcX == s_perpAxis) {
        s_srcX = (uint8_t)(3u - (uint32_t)s_perpAxis - (uint32_t)s_srcY);
    }
    if (s_srcY == s_perpAxis) {
        s_srcY = (uint8_t)(3u - (uint32_t)s_perpAxis - (uint32_t)s_srcX);
    }
}

static void FluidDemo_HudStatic(void)
{
    OLED_ShowString(HUD_LABEL_X, 0, (uint8_t *)"AX", 8);
    OLED_ShowString(HUD_LABEL_X, 1, (uint8_t *)"AY", 8);
    OLED_ShowString(HUD_LABEL_X, 2, (uint8_t *)"AZ", 8);
    OLED_ShowString(HUD_LABEL_X, 5, (uint8_t *)"SIM", 8);
    OLED_ShowString(HUD_LABEL_X, 6, (uint8_t *)"FPS", 8);
    OLED_ShowString(HUD_LABEL_X, 7, (uint8_t *)"V", 8);
}

/* 第 3/4 行标签显示屏幕 x / 屏幕 y 实际取自哪个传感器轴（X0 = 取自 AX） */
static void FluidDemo_HudAxisLabels(void)
{
    uint8_t buf[3];

    buf[0] = (uint8_t)'X';
    buf[1] = (uint8_t)('0' + s_srcX);
    buf[2] = 0u;
    OLED_ShowString(HUD_LABEL_X, 3, buf, 8);

    buf[0] = (uint8_t)'Y';
    buf[1] = (uint8_t)('0' + s_srcY);
    OLED_ShowString(HUD_LABEL_X, 4, buf, 8);
}

/* AX/AY/AZ = 原始 mg（已扣零偏、已翻转）；
   X?/Y?    = 实际送入流体的值（模拟单位×10）和它的来源轴；
   SIM = 单步物理平均耗时(0.1ms)；FPS = 帧率；
   V   = 垂直于屏幕平面的轴号（0/1/2）—— 这个轴不能用来驱动液体。 */
static void FluidDemo_HudDynamic(void)
{
    uint32_t i;
    int32_t  xs = FluidDemo_SimX10(s_axFilt);
    int32_t  ys = FluidDemo_SimX10(s_ayFilt);

    for (i = 0u; i < 3u; i++) {
        if (s_accRaw[i] != s_shownRaw[i]) {
            OLED_ShowSignedNum(HUD_VALUE_X, (uint8_t)i, s_accRaw[i], 4, 8);
            s_shownRaw[i] = s_accRaw[i];
        }
    }

    if (xs != s_shownXs) {
        OLED_ShowSignedNum(HUD_VALUE_X, 3, xs, 4, 8);
        s_shownXs = xs;
    }

    if (ys != s_shownYs) {
        OLED_ShowSignedNum(HUD_VALUE_X, 4, ys, 4, 8);
        s_shownYs = ys;
    }

    if (s_simMs != s_shownSim) {
        OLED_ShowNum(HUD_VALUE_X, 5, s_simMs, 4, 8);
        s_shownSim = s_simMs;
    }

    if (s_fps != s_shownFps) {
        OLED_ShowNum(HUD_VALUE_X, 6, s_fps, 4, 8);
        s_shownFps = s_fps;
    }

    /* 第 7 行：正常显示垂轴号；一旦出现 I2C 错误就改成显示 ERR（优先） */
    if (s_err > 0u) {
        if (s_hudErrShown == 0u) {
            OLED_ShowString(HUD_LABEL_X, 7, (uint8_t *)"ERR", 8);
            s_hudErrShown = 1u;
        }
        OLED_ShowNum(HUD_VALUE_X, 7, s_err, 4, 8);
    } else if (s_perpAxis != s_shownPerp) {
        OLED_ShowNum(HUD_VALUE_X, 7, (uint32_t)s_perpAxis, 1, 8);
        s_shownPerp = s_perpAxis;
    }
}

/* ==================== 数学层自检 ==================== */

/* 先确认 _IQmpy/_IQdiv/_IQsqrt 都是对的，再让仿真跑，免得白排查 */
static uint8_t FluidDemo_IqSelfTest(void)
{
    /* 容差 IQ_ONE/1024 ≈ 1e-3，Q24 下实际误差应远小于此 */
    const _iq tol = (IQ_ONE / 1024);
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
    uint32_t frameCnt = 0u;
    uint32_t lastHud;
    uint32_t accSim = 0u;
    uint32_t t0, t1;
    uint32_t i;
    _iq      ax, ay;

    /* 1) 数学层自检：验证 Q24 乘法 / 浮点除法 / 开方。失败就把结果留在屏幕上 */
    if (FluidDemo_IqSelfTest() == 0u) {
        OLED_ShowString(0, 0, (uint8_t *)"IQ SELFTEST", 8);
        OLED_ShowString(0, 1, (uint8_t *)"FAILED", 8);
        for (;;) { }
    }

    /* 2) 先画 HUD 标签，这样 IMU 初始化那 ~450ms 屏幕不是全黑 */
    FluidDemo_HudStatic();

    /* 3) 加速度计初始化。**不做零偏标定**，s_accOfs[] 固定为 0。
          好处：开机瞬间完成，不要求板子静止/放平，开机姿态对结果完全无影响。
          代价：LSM6DSR 的加速度零偏（±40mg 以内）会原样留下，相当于液面
                自带不到 2° 的斜度 —— 拿在手里倾斜时完全淹没在真实重力里，可忽略。
          如果以后发现板子水平放着液体还慢慢往一边漂，再加一个"按键触发重新
          置零"比开机标定更好用（不用重启，也不挑开机姿态）。 */
    IMU660RB_Init();

    /* 垂轴号直接用实测写死的常量，开机不做任何检测 */
    s_perpAxis = (uint8_t)ACC_PERP_AXIS;

    /* 万一 ACC_SRC_* 正好选中了垂轴，自动换成另一个面内轴，
       并把最终生效的轴显示在第 3/4 行的标签上（X?/Y?）。 */
    FluidDemo_FixAxis();
    FluidDemo_HudAxisLabels();

    /* 4) 流体初始化 + 预热一步（与原版 main 对齐） */
    Fluid_Init();
    Fluid_SetDt(DEMO_DT);
    Fluid_SetGridIter(DEMO_GRID_ITER);
    Fluid_Step(_IQ(0.0f), _IQ(9.8f));

    /* 5) 用一帧真实读数给低通赋初值，
          否则会从 0 重力开始慢慢爬，开局那几十帧液体会像失重一样乱飘 */
    Read_IMU660RB();
    for (i = 0u; i < 3u; i++) {
        s_accRaw[i] = ACC_MOUNT_SIGN * ((int32_t)acceleration_mg[i] - s_accOfs[i]);
    }
    s_axFilt = FluidDemo_AccelToSim(s_srcX, ACC_SIGN_X);
    s_ayFilt = FluidDemo_AccelToSim(s_srcY, ACC_SIGN_Y);

    lastHud = tick_ms;

    for (;;) {
        /* ---- 读加速度计 → 扣零偏 → 按安装方向翻转到板面坐标 ---- */
        Read_IMU660RB();
        for (i = 0u; i < 3u; i++) {
            s_accRaw[i] = ACC_MOUNT_SIGN * ((int32_t)acceleration_mg[i] - s_accOfs[i]);
        }

        /* ---- 映射到屏幕坐标 → 一阶低通 ---- */
        ax = FluidDemo_AccelToSim(s_srcX, ACC_SIGN_X);
        ay = FluidDemo_AccelToSim(s_srcY, ACC_SIGN_Y);

        s_axFilt += _IQmpy(ax - s_axFilt, ACC_LPF_ALPHA);
        s_ayFilt += _IQmpy(ay - s_ayFilt, ACC_LPF_ALPHA);

        /* ---- 物理一步 ---- */
        t0 = tick_ms;
        Fluid_Step(s_axFilt, s_ayFilt);
        t1 = tick_ms;
        accSim += (t1 - t0);

        /* ---- 渲染 + 推屏（不计入物理耗时） ---- */
        s_err += FluidDemo_Push();

        /* ---- 每 HUD_UPDATE_FRAMES 帧统计并刷新一次 HUD ---- */
        frameCnt++;
        if (frameCnt >= HUD_UPDATE_FRAMES) {
            uint32_t elapsed = tick_ms - lastHud;

            s_simMs = (accSim * 10u) / frameCnt;
            s_fps   = (elapsed > 0u) ? ((frameCnt * 1000u) / elapsed) : 0u;

            accSim   = 0u;
            frameCnt = 0u;
            lastHud  = tick_ms;

            FluidDemo_HudDynamic();
        }
    }
}
