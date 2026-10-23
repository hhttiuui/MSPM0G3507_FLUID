/*
 * fluid_demo.c —— 流体模拟 → LED 点阵（IMU 重力驱动 / 假重力可切换）
 *
 * 职责划分：
 *   fluid.c        纯算法，零外设依赖（可以原样丢到 PC 上跑）
 *   led_matrix.c   面板映射层：(列,行) → 芯片显示 RAM 位
 *   led_show.c     点阵综合演示（另一个可选程序，见 LED_SHOW_ENABLE）
 *   lsm6ds3trc.c   LSM6DS3TR-C 六轴 IMU 的 SPI 驱动（只提供原始/物理量）
 *   fluid_demo.c   本文件：算法 ↔ 点阵的胶水层，以及"重力来源"的切换
 *                  ① particleDensity[] → 亮度 → LedMatrix → 推屏
 *                     （灰度：内核亮、表层暗；二进制模式下自动退回阈值+滞回）
 *                  ② 重力来源：真实 IMU 加速度（默认）或假重力（fallback）
 *                  ③ 观测量全部在 dbg.fluid（没有屏，参数只能这么看）
 *
 * ==================== 重力来源与坐标映射 ====================
 *
 * FLUID_USE_IMU = 1（默认）：用 LSM6DS3TR-C 的加速度计读数当重力。
 *   加速度计静止时测的是"重力反作用力"，所以**平放时某轴读 +1000mg**。
 *
 *   关键：IMU 的芯片坐标系 ≠ 屏幕坐标系，必须做一次坐标变换。
 *   本板已完成标定（见下面 FLUID_IMU_* 宏处的三点实测推导），结论是
 *   **两轴对调 + 都取负**。若换了板子或改了 IMU 贴装方向，需要重新标定：
 *
 *     把屏幕按"正着放 → 顺时针90° → 顺时针180°"各停一次，记下液体落在哪边，
 *     和"应该落在哪边"（下 → 右 → 上）对比，即可解出这 4 个宏。
 *     快速口诀：
 *       液体流向与倾斜方向相反   → 该轴 SIGN 取反
 *       液体左右流但你是上下倾斜 → 两个 SRC 对调
 *
 *   ⚠ 这个映射对**摇晃**同样成立：加速度计测的是比力，"运动加速度"和重力反作用
 *     走的是同一条换算链，所以左右摇时给出的方向自动就是"惯性滞后"的方向
 *     （向右推板子 → 液体往左堆），不需要为摇晃另加符号。
 *
 *   dbg.fluid.ax10 / ay10 是**送进流体后**的值（模拟单位 ×10），
 *   可以直接确认映射对不对：正着放时应为 (0, +98) 即"向下"。
 *
 * FLUID_USE_IMU = 0：退回原来的"假重力"（每 DEMO_TILT_MS 换一个方向）。
 *   用于 IMU 没焊 / 没通 / 想对比效果时。IMU 初始化失败会自动走这条路。
 *
 * ==================== 数值处理 ====================
 *   原始读数有噪声，直接喂流体会让液体抖。这里做三件事：
 *     ① 死区：|读数| 小于阈值时当 0，避免静止时液体自己慢慢漂
 *     ② 一阶低通：α 越小越平滑但越迟钝
 *     ③ 双通路分解：把读数按频率拆成"慢 = 重力/倾角"和"快 = 摇晃/惯性"，
 *        于是"摇一摇液体跟着晃"和"静止不抖"可以同时成立 —— 详见下面的
 *        《重力 / 摇晃：双通路分解》
 *   都是 Q24 定点运算，不引入软浮点。
 *
 * ⚠ 尺寸对得上：流体可视区 15 列 × 16 行，面板 16 列 × 15 行，两者是转置关系，
 *   由 led_matrix.h 的 LED_ORIENT_SWAP_XY 处理（方向和机械安装有关，看图案段调）。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "debug.h"
#include "battery.h"
#include "fluid.h"
#include "fluid_demo.h"
#include "led_matrix.h"
#include "ht16d33a.h"       /* 渲染要按 HT16D33A_USE_BINARY_MODE 分支（灰度 or 二进制）*/
#include "lsm6ds3trc.h"
#include "power.h"
#include "vcmd.h"           /* 串口下行命令：DMA RX + 文本解析 */
#include "vofa.h"           /* 串口上行遥测：FireWater + DMA TX */

/* ==================== 配置 ==================== */

/* 1 = 用真实 IMU 的加速度计驱动重力；0 = 用假重力（编译期强制退回）*/
#define FLUID_USE_IMU       1

/* IMU 初始化失败时是否允许自动退回假重力（1 = 允许，程序不会卡住）*/
#define FLUID_IMU_FALLBACK  1

/* ---- 轴向映射（已按本板实测标定）----
 * 选轴：0 = AX, 1 = AY, 2 = AZ
 * 符号：+1 或 -1
 *
 * 本板实测（IMU 与屏同向竖装）：把"喂进流体的值"当 (ax, ay) 看，三种姿态下
 *   屏幕正着放    → 液体落在【左】(-1, 0)，应为【下】(0,+1)
 *   顺时针 90°    → 液体落在【上】(0,-1)，应为【右】(+1,0)
 *   顺时针 180°   → 液体落在【右】(+1,0)，应为【上】(0,-1)
 * 三点解出唯一线性映射：
 *   ax_屏 = -ay_IMU
 *   ay_屏 = -ax_IMU
 * 即【两轴对调 + 都取负】。
 *
 * ⚠ 注意这个映射矩阵行列式 = -1，是"旋转 + 镜像"而不是纯旋转
 *   （纯旋转行列式应为 +1）。也就是说本板 IMU 的 X-Y 平面相对屏幕是**镜像**的，
 *   常见成因：芯片贴在 PCB 背面、或封装轴向与屏幕轴向手性相反。
 *   这不影响使用，但换板/换贴装方向后必须重新用上面三点法标定。
 *
 * 标定口诀（只需改这 4 个宏）：
 *   液体流向与倾斜方向相反   → 该轴 SIGN 取反
 *   液体左右流但你是上下倾斜 → 两个 SRC 对调 */
#define FLUID_IMU_AX_SRC    1     /* 屏幕 x 取 AY */
#define FLUID_IMU_AY_SRC    0     /* 屏幕 y 取 AX */
#define FLUID_IMU_AX_SIGN   (-1)
#define FLUID_IMU_AY_SIGN   (-1)

/* ==================== 重力 / 摇晃：双通路分解 ====================
 *
 * 问题：一个低通没法同时满足两件事 ——
 *   静态平滑（α 小）：静止 / 慢倾斜时液体不抖，但手摇 1~3Hz 的左右晃动会被
 *                     衰减到只剩两三成，液体只是"慢慢歪过去"，不"晃"；
 *   动态跟手（α 大）：摇起来有反应，但静止时噪声直通，液体表面自己抖。
 *   实测：把 FLUID_IMU_LPF_ALPHA 从 0.10 调到 0.90，摇晃立刻明显 —— 但也正因
 *   为"调大就抖"，这件事不能靠调 α 解决。
 *
 * 解法：把加速度计读数按频率拆成两条通路。加速度计测的是**比力**，本身就等于
 *       "重力反作用 + 自身运动加速度"，所以两条通路就是同一次读数的分解：
 *
 *                     ┌─ 死区 40mg → 慢通路低通 α_slow ─────────────────┐
 *   raw ─(选轴/符号)──┤                                                  ├→ 相加 → 限幅 → Fluid_Step
 *                     └─ (raw − 慢) 高通 → α_fast 低通 → 死区 90mg ──────┘（× 增益）
 *
 *   · 慢通路 = 原来的行为，**逐位不变**（同一个 α、同一个死区）⇒ 静态手感不会
 *     和以前有任何差别
 *   · 快通路 = 读数里"变化的那部分"。静止时它恒为 0（增益再大也不起作用），
 *     一摇就出来 ⇒ 摇晃强度可以独立调，不用拿静态平滑去换
 *   · 高通天然"回中"：停止摇晃后快通路自己衰减到 0，液体回到只受重力，
 *     不需要任何状态机 / 计时器
 *   · 附带好处：快速倾斜时也会有真实的惯性滞后（快通路给出瞬时误差、慢通路
 *     随后追上），比原来"纯跟随"更像液体
 *
 * ⚠ 这里**有两个死区，作用不同，不能合并**（30fps 离线仿真实测）：
 *
 *   FLUID_IMU_DEADBAND_MG (40mg)：**慢通路输入**的门槛，小于它就当 0，避免静止
 *     时液体自己慢慢漂。但它只在"信号本身很小"时才管用 —— 板子有较大静态倾斜时
 *     （比如某轴持续 800mg），骑在上面的 ±5~10mg 噪声是**穿过**这个死区进到后面去的。
 *
 *   FLUID_SHAKE_DEADBAND (90mg)：**快通路输出后**的门槛。只有它能挡住两种假摇晃：
 *     ① 倾斜态下的噪声（快通路把它看成"变化"）；
 *     ② 信号压在 40mg 死区边界上时，输入会在 0 和 40mg 之间反复跳，快通路会把这
 *        种方波当成摇晃。
 *     实测：去掉它，静止倾斜时快通路输出 ≈0.13 模拟单位的抖动，和当年单通路
 *     α=0.90 的抖动量级一样（也就是白改）。加上它，静止时抖动回到 α=0.10 的水平。
 *
 *   ⇒ 所以快通路的输入必须用**未加死区的原始值**（避开②的跳变），门槛留给输出端。
 *
 * ⚠ 两条通路的 α 都是"每帧一次"的定义，所以等效截止频率会跟着帧率走
 *   （fps ≈ 30 时把 0.10 记成 ≈0.5Hz、0.60 记成 ≈2.9Hz）。本工程主循环是定长的
 *   （单步物理 ≈28ms），帧率很稳，所以不做 dt 补偿；以后如果帧长抖动明显，
 *   可以用 tick_ms 实测 Δt 折算 α' = α × Δt/33ms（上限 1）。
 */

/* 死区（mg）：小于这个值当 0。静止时加速度计噪声约 ±10mg，取 40 足够稳。
 * ⚠ 这是**慢通路（重力）**的门槛，只作用于慢通路 */
#define FLUID_IMU_DEADBAND_MG   40

/* ---- 慢通路：重力 / 倾角 ---- */
/* 一阶低通系数 α（Q24）。越小越平滑、越迟钝。
 * 0.10 ≈ 时间常数 10 帧；液体跟手但不抖。
 * ⚠ 现在有快通路了，这里**不要再为了摇晃去调大** —— 调大只会让静止的液体跟着
 *   噪声抖，摇晃那一半应该交给 FLUID_SHAKE_ALPHA / FLUID_SHAKE_GAIN */
#define FLUID_IMU_LPF_ALPHA     _IQ(0.10f)

/* ---- 快通路：摇晃 / 惯性 ---- */
/* 快通路的低通系数 α（Q24），只负责吃掉高通残留的噪声。
 * 0.60 ≈ 时间常数 1.7 帧（fps≈30 时截止 ≈2.9Hz）：2Hz 的手摇通过 ~0.82，
 * 再乘下面的增益，观感比当初 α=0.90 的单通路还活跃一点。
 * 调大更跟手也更毛糙，调小更稳但摇不动。 */
#define FLUID_SHAKE_ALPHA       _IQ(0.60f)

/* 摇晃增益（Q24）。1.0 = 物理正确（摇晃 0.3g 就是 30% 的重力）。
 * 上面那条低通会吃掉一部分幅度，所以默认给 1.5，"摇"的观感才和物理相当。
 * 想让液体更疯就往上加（2.0 已经很夸张），觉得太晃就减到 1.0。 */
#define FLUID_SHAKE_GAIN        _IQ(1.5f)

/* 摇晃死区（模拟单位）：90mg = 90 × 9.8/1000 = 0.882。
 * 为什么必须有它、为什么不能和 FLUID_IMU_DEADBAND_MG 合并，见上面的《双通路分解》。
 * 想摇得更灵敏就往下调（60mg 也还能用），想更稳就往上调（120mg）。
 * ⚠ 它是在 α_fast 低通**之后**生效的，所以改它会同时影响灵敏度与静态抖动。 */
#define FLUID_SHAKE_DEADBAND    _IQ(0.882f)

/* 复合之后的限幅（模拟单位，9.8 = 1g）。
 * 加速度计本身在 ±2g 就削顶了，这里纯粹是防"磕桌子"那种单帧尖峰被放大后把
 * 液体一下甩到边上（Q24 量程 ±127 不会溢出，只是手感问题）。
 * 想保留最猛的冲击感可以调到 _IQ(39.2f)（= 4g，等于不限）。 */
#define FLUID_ACC_CLAMP         _IQ(24.5f)

/* 假重力方向保持多久（ms）：4 个方向 下→左→上→右 循环 */
#define DEMO_TILT_MS        2000u

/* 时间步长：原版实测 0.032 时手感最好（一步物理约 28ms，dt 推 32ms ≈ 1.1 倍实时）*/
#define DEMO_DT             _IQ(0.032f)

/* 压力求解迭代次数（越大液体越"硬"、也越慢）*/
#define DEMO_GRID_ITER      12u

/* 每多少帧统计一次 dbg.fluid 的耗时/帧率 */
#define DEMO_STAT_FRAMES    20u

/* 加速度计 1g 对应的 mg 值（换算成模拟单位 9.8 用）*/
#define DEMO_MG_PER_G       1000

/* ---- 运行期可调状态（串口命令改这些，见 FluidDemo_CmdTask）----
 * 上面那些 *_GAIN / DEMO_* 宏是"默认值"，这里才是真正生效的变量。
 * 为什么不直接用宏：串口调参的价值就在于不改代码、不重编译。 */
static _iq      s_shakeGain = FLUID_SHAKE_GAIN;   /* shake <n×100> 改 */
static uint8_t  s_paused;                         /* pause 1 = 冻结物理 */
static uint8_t  s_dispLock;                       /* lock  1 = 冻结流体渲染 */

/* 用户设定的全局亮度（0x37）。低电安静窗口熄屏采样后要恢复成**这个值**，
 * 不能写死 MAX —— 否则串口把 bc 调到 64，每 2 秒就被安静窗口打回 255。 */
static uint8_t  s_userBc = HT16D33A_BRIGHTNESS_MAX;

/* 假重力的 4 个方向（模拟单位）。约定：x 正 = 屏幕右，y 正 = 屏幕下 */
static const _iq s_tiltAcc[4][2] = {
    { _IQ(0.0f),  _IQ(9.8f) },      /* 0 下 */
    { _IQ(-9.8f), _IQ(0.0f) },      /* 1 左 */
    { _IQ(0.0f),  _IQ(-9.8f) },     /* 2 上 */
    { _IQ(9.8f),  _IQ(0.0f) },      /* 3 右 */
};

/* 慢通路状态（重力/倾角的低通输出，Q24，单位：模拟单位）*/
static _iq s_axSlow;
static _iq s_aySlow;

/* 快通路状态（摇晃分量，Q24，单位：模拟单位。**不含增益**，增益在合成时乘）*/
static _iq s_sxFast;
static _iq s_syFast;

/* 滤波器是否已用第一次读数播种（1 = 已播种）。
 * 不播种的话，滤波器从 0 开始爬，开机头几百毫秒液体会先"自由落体"再稳住。 */
static uint8_t s_filtSeeded;

/* ==================== 重力来源：IMU ==================== */

/* 死区（mg）：小于阈值的当 0。
 * ⚠ **只给慢通路（重力）用**：快通路的输入必须用未加死区的原始值，否则信号压在
 *   死区边界上时会 0/40mg 反复跳，那种方波会被快通路误当成摇晃（见《双通路分解》）*/
static int32_t FluidDemo_Deadband(int32_t mg)
{
    if (mg > -FLUID_IMU_DEADBAND_MG && mg < FLUID_IMU_DEADBAND_MG) {
        return 0;
    }

    return mg;
}

/* 从 dbg.phy 里取出指定轴、指定符号的加速度值（mg）。**不做死区** */
static int32_t FluidDemo_PickAxis(uint8_t src, int8_t sign)
{
    int32_t v;

    switch (src) {
        case 0:  v = dbg.phy.ax_mg; break;
        case 1:  v = dbg.phy.ay_mg; break;
        case 2:  v = dbg.phy.az_mg; break;
        default: v = 0;             break;
    }

    return (sign < 0) ? -v : v;
}

/* mg → 模拟单位（9.8 对应 1000mg）。
 * 用 Q24 定点：result = mg * 9.8 / 1000 = mg * _IQ(9.8) / 1000
 * 先乘后除，避免整除丢精度。 */
static _iq FluidDemo_MgToSim(int32_t mg)
{
    int64_t t = (int64_t) mg * (int64_t) _IQ(9.8f);

    t = t / DEMO_MG_PER_G;

    return (_iq) t;
}

/* 读一次 IMU：选轴 → 符号 → 双通路分解（慢=重力 / 快=摇晃）→ 相加 → 限幅，
   输出屏幕坐标系下"该喂给流体的总加速度"（模拟单位）。
   参数与原理见文件上方《重力 / 摇晃：双通路分解》。 */
static void FluidDemo_ReadGravity(_iq *ax, _iq *ay)
{
    int32_t rawXmg, rawYmg;
    _iq     slowInX, slowInY;
    _iq     totalX, totalY;

    /* 一次突发读拿六轴（约十几 µs），结果落在 dbg.phy / dbg.raw */
    LSM6DS3TRC_ReadAll();

    /* 选轴 + 符号。**这里不加死区** —— 死区只属于慢通路，见下 */
    rawXmg = FluidDemo_PickAxis(FLUID_IMU_AX_SRC, FLUID_IMU_AX_SIGN);
    rawYmg = FluidDemo_PickAxis(FLUID_IMU_AY_SRC, FLUID_IMU_AY_SIGN);

    /* 慢通路的输入 = 死区之后的值（和改动前逐位一致）*/
    slowInX = FluidDemo_MgToSim(FluidDemo_Deadband(rawXmg));
    slowInY = FluidDemo_MgToSim(FluidDemo_Deadband(rawYmg));

    if (s_filtSeeded == 0u) {
        /* 第一次：慢通路直接赋当前值（避免从 0 慢慢爬上去，开机时液体乱冲一下）；
         * 快通路清零 —— 此刻还没有"变化"，别把开机瞬间当成摇晃 */
        s_axSlow = slowInX;
        s_aySlow = slowInY;
        s_sxFast = 0;
        s_syFast = 0;
        s_filtSeeded = 1u;
    } else {
        /* 慢通路：一阶低通 y[n] = y[n-1] + α·(x[n] − y[n-1]) */
        s_axSlow += _IQmpy(FLUID_IMU_LPF_ALPHA, slowInX - s_axSlow);
        s_aySlow += _IQmpy(FLUID_IMU_LPF_ALPHA, slowInY - s_aySlow);

        /* 快通路：先高通（raw − 慢，把"变化量"挑出来），再低通吃噪声。
         * 这里的"慢"是本帧刚更新过的值，所以 (raw − 慢) 天然就是高通的结果。
         * ⚠ 高通用的是**未加死区的 raw**，这样信号压在 40mg 死区边界上时
         *   （输入在 0 和 40mg 之间跳）不会在快通路里造出假摇晃。 */
        s_sxFast += _IQmpy(FLUID_SHAKE_ALPHA,
                           (FluidDemo_MgToSim(rawXmg) - s_axSlow) - s_sxFast);
        s_syFast += _IQmpy(FLUID_SHAKE_ALPHA,
                           (FluidDemo_MgToSim(rawYmg) - s_aySlow) - s_syFast);
    }

    /* 摇晃死区：平滑完还是小值的当 0（挡"倾斜态下的噪声"）。
     * 这里清的是**状态**而不是只清输出，这样 dbg 里看到的 fast 就是"实际贡献了多少"，
     * 静止时读数为 0 一目了然。 */
    if (s_sxFast > -FLUID_SHAKE_DEADBAND && s_sxFast < FLUID_SHAKE_DEADBAND) {
        s_sxFast = 0;
    }
    if (s_syFast > -FLUID_SHAKE_DEADBAND && s_syFast < FLUID_SHAKE_DEADBAND) {
        s_syFast = 0;
    }

    /* 合成：重力 + 摇晃×增益，然后限幅（防磕碰尖峰）*/
    totalX = s_axSlow + _IQmpy(s_shakeGain, s_sxFast);
    totalY = s_aySlow + _IQmpy(s_shakeGain, s_syFast);

    if (totalX > FLUID_ACC_CLAMP) {
        totalX = FLUID_ACC_CLAMP;
    } else if (totalX < -FLUID_ACC_CLAMP) {
        totalX = -FLUID_ACC_CLAMP;
    }
    if (totalY > FLUID_ACC_CLAMP) {
        totalY = FLUID_ACC_CLAMP;
    } else if (totalY < -FLUID_ACC_CLAMP) {
        totalY = -FLUID_ACC_CLAMP;
    }

    *ax = totalX;
    *ay = totalY;
}

/* ==================== 渲染 ==================== */

/*
 * 抗闪烁：密度阈值 + 幅度滞回（amplitude hysteresis）
 *
 * 现象：静止时液体**顶层及接近顶层**的格子无规律乱闪。
 *
 * 根因（不在 IMU，在渲染用的数据源）：
 *   fluid.c 的 particles_to_grid() 生成 cellType[] 时是**点采样**：
 *       xi = _IQint(particlePos * invertSpacing);   // 只看粒子中心落在哪格
 *       cellType[INDEX(xi, yi)] = FLUID_CELL;
 *   液体内部格子挤着多个粒子，掉一个不影响；但**表层粒子稀疏**，很多格子
 *   只有 0~1 个粒子。静止时粒子在格子边界微抖（压力求解残差 + 粒子互斥的
 *   微小位移），中心点就在相邻两格间反复跳：
 *     离开的格子 → AIR_CELL（灭）
 *     进入的格子 → FLUID_CELL（亮）
 *   ⇒ 顶层乱闪。
 *
 * ❌ 曾用方案：时间迟滞（要求连续 N 帧同状态才翻转）
 *   实测**粘滞感太强，观感不好**。原因：它只看"持续了几帧"，**不看变化幅度**，
 *   所以分不清"单帧噪声"和"真实液面移动"——后者也被无谓推迟 N 帧。
 *   而且 N 小压不住闪、N 大更粘，是个死结。
 *
 * ✅ 现方案：换用**密度场**（particleDensity[]）+ 幅度滞回
 *   density_update() 里每个粒子按双线性权重贡献给最近 4 格，权重连续变化、
 *   四者之和恒为 1。粒子在边界微抖时，相邻两格密度是"此消彼长"的连续量，
 *   **两格都保持 > 0**，不会瞬间翻转 ⇒ 从源头消除了阶跃。
 *
 *   再配一对**上下阈值**（滞回）吃掉残余的小幅摆动：
 *     密度 > ON  才点亮
 *     密度 < OFF 才熄灭
 *   ON/OFF 之间是"保持原状"的缓冲区。
 *
 *   为什么这个不粘滞：密度变化是**物理上真实的**——液面真涨上来，密度就真的
 *   越过阈值，**当帧立即亮**，零延迟；而抖动时密度在阈值附近小幅摆动，被滞回
 *   区间直接吃掉。用**幅度**滞回对付噪声，而不是用**时间**延迟。
 *
 * 阈值怎么定：单位是"粒子数当量"（1 个粒子满权重 = 1.0）。
 *   液体内部格子通常 2~5，表层 0.1~0.5。
 *   所以取 ON ≈ 0.35、OFF ≈ 0.22 能干净分开"有液/无液"，且表层不抖。
 *   ⚠ 换了粒子数 / 网格尺寸后这两个值要重新试。
 */
#define FLUID_RENDER_DENSITY    1       /* 1 = 密度阈值渲染（推荐，零延迟） */
                                        /* 0 = 退回点采样 cellType[]（会闪，仅供对比） */

/* ==================== 渲染：密度 → 亮度（灰度）====================
 *
 * 驱动层已经切到灰度模式（见 ht16d33a.h 的 HT16D33A_USE_BINARY_MODE=0）：
 * 每个点 1 字节 = 0~255 亮度。于是"阈值 + 幅度滞回"那一整套都可以扔掉：
 *
 *   阈值方案的死结：显示只有两个状态。密度在阈值附近摆动就是整格翻转 ——
 *     阈值贴太近会闪，调高又会把液体的薄边整条削掉。
 *   灰度方案：亮度是连续量，同样的摆动只造成 1/256 的亮度变化，肉眼看不见；
 *     而真实液面移动**当帧**就体现为亮度渐变，零延迟、零粘滞感。
 *
 * 映射：level = density / DENS_FULL × 255，超出 255 截断
 *
 *   DENS_FULL 取"静止密度"（第一帧标定的液体平均密度，实测 ≈2.18）再乘一个
 *   百分比。好处是**换粒子数 / 网格尺寸后自适应**，不用像阈值方案那样重新标定：
 *     PCT=100 → 内核刚好拉满，内部层次最丰富
 *     PCT= 60 → 更亮更满（当前）：内核一片白，只有外圈是渐变
 *     PCT=150 → 更暗，内部层次最多
 *   "想让液体看起来更满"就往下调 PCT；"想要更多层次"就往上调。
 *
 * 代价：每格 1 次 Q24 乘法（MATHACL 硬件，约 35 周期），240 格 ≈ 0.12ms；
 *       每帧另加 1 次 64 位除法算比例系数。相对物理步可忽略。 */
#define FLUID_RENDER_DENSITY    1       /* 1 = 密度→亮度渲染（推荐） */
                                        /* 0 = 退回点采样 cellType[]（会闪，仅供对比） */

/* 满亮度密度 = 静止密度 × 本百分比 / 100 —— 整体亮度的唯一旋钮 */
#define FLUID_DENS_FULL_PCT     (60)
/* 静止密度拿不到（<=0）时的兜底满量程 */
#define FLUID_DENS_FULL_FALLBACK    _IQ(2.2f)

/* 亮度上限（灰度 8 位） */
#define FLUID_LEVEL_MAX         ((int32_t) LED_MATRIX_MAX_LEVEL)
/* 低于这个亮度的格子肉眼基本看不见，统计"亮了多少格"时不算它 */
#define FLUID_LEVEL_VISIBLE_MIN (8)

/* 二进制模式的阈值（只有在 HT16D33A_USE_BINARY_MODE=1 时才用得上，
 * 但一直定义着，好让 FluidDemo_Run 里上报 dbg 的代码两种模式都能编译）*/
#define FLUID_DENSITY_ON        _IQ(0.15f)
#define FLUID_DENSITY_OFF       _IQ(0.09f)

#if HT16D33A_USE_BINARY_MODE
/* ---- 二进制模式的退路：阈值 + 幅度滞回 ----
 * 保留这套是为了能一键对比（把 ht16d33a.h 的 HT16D33A_USE_BINARY_MODE 改回 1）。
 * 定标依据（2026-10 实测，130 粒子 / 15×16 网格）：内核密度 2.6、最表层 0.066，
 * 取 ON=0.15（表层的 2.3 倍、内核的 1/17）能干净分开"有液/无液"。
 * 调参：灯太少 → ON 往下；表层开始闪 → ON 往上。 */
static uint8_t s_ledOn[LED_APP_COLS][LED_APP_ROWS];

/* 幅度滞回：给定本帧密度和上一帧状态，返回本帧该亮还是灭。
 * 零时间延迟——密度一越过阈值当帧就翻转。 */
static uint8_t FluidDemo_DensityHysteresis(uint8_t ix, uint8_t iy, _iq density)
{
    uint8_t *on = &s_ledOn[ix][iy];

    if (*on != 0u) {
        /* 当前亮着：只有密度掉到 OFF 以下才灭 */
        if (density < FLUID_DENSITY_OFF) {
            *on = 0u;
        }
    } else {
        /* 当前灭着：只有密度涨到 ON 以上才亮 */
        if (density > FLUID_DENSITY_ON) {
            *on = 1u;
        }
    }

    return *on;
}
#endif /* HT16D33A_USE_BINARY_MODE */

/* 一帧显示缓冲 → 推屏。
 * 顺便统计密度/亮度分布，供 dbg.fluid 观察（开销可忽略）。 */
static void FluidDemo_Push(void)
{
    uint8_t x, y;

    /* ---- 低电检查的"安静窗口"（每 BAT_CHECK_MS 一次）----
     * LED 满载 360mA 会把电池读数压低 200~335mV（实测），所以必须先把屏熄掉
     * 再采；否则一颗健康的 3.8V 电池会被读成 3.43V，直接误判低电关机。
     *
     * 用全局亮度 0x37 = 0 熄屏：一条命令立即生效，不清帧缓冲、也不等扫描走完，
     * 采样后写回 MAX 就能原样恢复画面。
     * 熄屏时长 ≈ BAT_SETTLE_MS + 一次采样(≈0.5ms) ≈ 1.5ms，肉眼不可见。 */
    if (Battery_QuietBegin() != 0u) {
        HT16D33A_SetBrightness(HT16D33A_BRIGHTNESS_OFF);
        mspm0_delay_ms(BAT_SETTLE_MS);
        Battery_QuietSample();                  /* 低电确认时这里可能直接关机（不返回）*/
        HT16D33A_SetBrightness(s_userBc);       /* 恢复用户设定的亮度，不是写死 MAX */
    }

    /* ---- 显示锁：串口 `lock 1` 之后流体不再改画面 ----
     * 用途是亮度标定 —— 需要一块**完全静止**的画面才能稳定读数。
     * 冻结的是"重建帧 + 推屏"这两步，芯片显示 RAM 里的内容会一直保持
     * （HT16D33A 自带扫描，不需要 MCU 持续刷新）。
     * 低电安静窗口仍然照跑（在上面），所以锁着的时候电池保护依然有效。 */
    if (s_dispLock != 0u) {
        return;
    }

    LedMatrix_Clear();

#if FLUID_RENDER_DENSITY
    const _iq *density = Fluid_GetParticleDensity();
    _iq dMax = 0;
    uint32_t lit     = 0u;      /* 亮度达到"看得见"的格子数 */
    uint32_t litCore = 0u;      /* 亮度过半的格子数 ≈ 液体内核 */

#if HT16D33A_USE_BINARY_MODE
    for (x = 1u; x <= LED_APP_COLS; x++) {          /* 15 列 */
        for (y = 1u; y <= LED_APP_ROWS; y++) {      /* 16 行 */
            _iq d = density[CELL_INDEX((uint32_t) x, (uint32_t) y)];

            if (d > dMax) {
                dMax = d;
            }

            if (FluidDemo_DensityHysteresis((uint8_t) (x - 1u), (uint8_t) (y - 1u), d) != 0u) {
                LedMatrix_SetCell(x, y);
                lit++;
                litCore++;
            }
        }
    }
#else
    /* 灰度：每帧先算一次比例系数 k，再逐格乘。
     *
     *   k = 255 × 2^24 × 100 / (PCT × DENS_FULL)
     *
     * 注意 k 是**普通整数、不是 Q24 值**：这样 _IQmpy(d, k) = (d×k)>>24 的结果
     * 直接就是 0..255 的亮度。
     * ⚠ 千万别把"亮度"本身当 Q24 值来算 —— Q24 的量程只有 ±127.99，
     *   255 根本表示不了，中间结果会溢出 int32（这个坑踩过）。 */
    _iq densFull = Fluid_GetRestDensity();
    int32_t k;
    int32_t lv;

    if (densFull <= _IQ(0.0f)) {
        densFull = FLUID_DENS_FULL_FALLBACK;
    }
    k = (int32_t) ((((int64_t) FLUID_LEVEL_MAX) << IQ_SHIFT) * 100
                   / ((int64_t) FLUID_DENS_FULL_PCT * (int64_t) densFull));

    for (x = 1u; x <= LED_APP_COLS; x++) {          /* 15 列 */
        for (y = 1u; y <= LED_APP_ROWS; y++) {      /* 16 行 */
            _iq d = density[CELL_INDEX((uint32_t) x, (uint32_t) y)];

            if (d > dMax) {
                dMax = d;
            }

            lv = (d > 0) ? _IQmpy(d, k) : 0;
            if (lv > FLUID_LEVEL_MAX) {
                lv = FLUID_LEVEL_MAX;       /* 内核超出满量程就截断（会一片白）*/
            }

            if (lv > 0) {
                LedMatrix_SetCellLevel(x, y, (uint8_t) lv);
                if (lv >= FLUID_LEVEL_VISIBLE_MIN) {
                    lit++;
                }
                if (lv >= (FLUID_LEVEL_MAX / 2)) {
                    litCore++;
                }
            }
        }
    }

    dbg.fluid.dens_full_x1000 = (uint32_t)(((int64_t) densFull * 1000) / IQ_ONE);
#endif

    /* 换算成 ×1000 的整数，方便在 CCS 里直接读（Q24 → 实际值 = x1000/1000）*/
    dbg.fluid.dens_max      = (uint32_t)(((int64_t) dMax * 1000) / IQ_ONE);
    dbg.fluid.dens_lit      = lit;
    dbg.fluid.dens_lit_core = litCore;
    dbg.fluid.rest_dens     = (int32_t) Fluid_GetRestDensity();  /* 应为正；是负数就说明累加溢出了 */
#else
    const uint8_t *cellType = Fluid_GetCellType();

    for (x = 1u; x <= LED_APP_COLS; x++) {          /* 15 列 */
        for (y = 1u; y <= LED_APP_ROWS; y++) {      /* 16 行 */
            if (cellType[CELL_INDEX((uint32_t) x, (uint32_t) y)] != AIR_CELL) {
                LedMatrix_SetCell(x, y);
            }
        }
    }
#endif

    LedMatrix_Flush();      /* 灰度下 256 字节分 8 个 burst，约 1.2ms @2MHz SPI */
}

/* 模拟单位 ×10 取整，方便在 dbg 里当整数看 */
static int32_t FluidDemo_SimX10(_iq v)
{
    return (int32_t) (((int64_t) v * 10) / IQ_ONE);
}

/* 窗口内摇晃分量（已含增益）的峰值 ×10，只给 dbg.fluid.shake_pk10 看。
 * 没屏了，这是"摇晃到底有没有进来"最直观的一个数：静止 ≈ 0，摇起来几百。 */
static uint32_t s_shakePk10;

static void FluidDemo_TrackShakePeak(void)
{
    int32_t  fx = FluidDemo_SimX10(_IQmpy(s_shakeGain, s_sxFast));
    int32_t  fy = FluidDemo_SimX10(_IQmpy(s_shakeGain, s_syFast));
    uint32_t m  = (uint32_t) ((fx < 0) ? -fx : fx);
    uint32_t my = (uint32_t) ((fy < 0) ? -fy : fy);

    if (my > m) {
        m = my;
    }
    if (m > s_shakePk10) {
        s_shakePk10 = m;
    }
}

/* ==================== 数学层自检 ==================== */

/* 先确认 _IQmpy/_IQdiv/_IQsqrt 都是对的，再让仿真跑，免得白排查。
 * 失败没法在屏幕上写字了，所以用"整屏点亮 + 停下"当报错信号，结果也记在
 * dbg.fluid.iq_ok 里。 */
static uint8_t FluidDemo_IqSelfTest(void)
{
    const _iq tol = (IQ_ONE / 1024);        /* ≈1e-3，Q24 下实际误差远小于此 */
    _iq t;

#if IQ_MPY_MODE == 2
    /* ---- 硬件乘法自检（必须放在最前面：后面所有 _IQmpy 都依赖它的结论）----
     * 覆盖位置/权重/密度/重力/负值这些真实量级，和精确软件参考逐点比对。
     *   通过 → g_iqMpyUseHw = 1，走 MATHACL；
     *   不通过 → 自动退回软件乘法，**数值照样正确**，只是慢。
     * dbg.fluid.iq_mpy_ok  = 1 硬件可用 / 0 已退回软件
     * dbg.fluid.iq_mpy_err = 最大误差（Q24 LSB），正确时应为 0~1 */
    {
        int32_t mpyErr = 0;

        if (IQ_MpySelfTest(&mpyErr) == 0) {
            dbg.fluid.iq_mpy_ok = 1u;
        } else {
            dbg.fluid.iq_mpy_ok = 0u;
        }
        dbg.fluid.iq_mpy_err = mpyErr;
    }
#else
    dbg.fluid.iq_mpy_ok = 0xFFu;        /* 0xFF = 编译期就是软件实现 */
#endif

#if IQ_SQRT_MODE
    /* ---- 硬件 SQRT 自检 ----
     * 配置照抄 TI 的 __IQNsqrt_MathACL()（详见 fluid_iqmath.h 顶部注释，
     * 那里有 2026-10 的实测数据表）。
     * 拿 sqrt(4)=2、sqrt(9)=3 两个已知点验一遍：
     *   通过 → 后续 _IQsqrt 走硬件，快（TI 基准 109 周期 vs 软件 553）；
     *   不通过 → 自动退回软件开方，**仿真照样跑**，只是慢一点。
     * 结果都能在 dbg 里看到：
     *   dbg.fluid.iq_sqrt_ok = 1 硬件可用 / 0 已退回软件 / 0xFF 本来就是软件
     *   dbg.fluid.iq_sqrt_sf / iq_sqrt_it = 自检点用到的参数，供以后排查
     *   dbg.fluid.iq_sqrt_err = 实测误差，期望 ≈ -1024 LSB（硬件迭代的量化误差）*/
    {
        int32_t sqrtErr = 0;

        if (IQ_SqrtSelfTest(&sqrtErr) == 0) {
            dbg.fluid.iq_sqrt_ok = 1u;
        } else {
            dbg.fluid.iq_sqrt_ok = 0u;
        }
        dbg.fluid.iq_sqrt_err = sqrtErr;
    }
    dbg.fluid.iq_sqrt_sf = g_iqSqrtSfactor;
    dbg.fluid.iq_sqrt_it = g_iqSqrtIter;
#else
    dbg.fluid.iq_sqrt_ok = 0xFFu;       /* 0xFF = 用的是软件实现，无需自检 */
#endif

    t = _IQmpy(_IQ(2.0f), _IQ(3.0f));                 /* 乘法 */
    if (t < (_IQ(6.0f) - tol) || t > (_IQ(6.0f) + tol)) return 0u;

    /* 非整数乘法：专门验 MATHACL MPY32 的 Q24 定标和截断。
     * 若硬件没做 >>24（或定标不对），这里会差好几个数量级，一下就暴露。 */
    t = _IQmpy(_IQ(0.35f), _IQ(0.35f));               /* 0.1225 */
    if (t < (_IQ(0.1225f) - tol) || t > (_IQ(0.1225f) + tol)) return 0u;

    t = _IQmpy(_IQ(1.9f), _IQ(0.123f));               /* 0.2337，压力求解里的真实量级 */
    if (t < (_IQ(0.2337f) - tol) || t > (_IQ(0.2337f) + tol)) return 0u;

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

/* ==================== 低电关机动作（动画接口的示范实现）====================
 *
 * battery.c 只负责"判断低电"，具体怎么收场交给这里。
 * 现在 = **熄屏 → 立即关机**（用户要求"直接关机"，不播动画）。
 *
 * 以后要做"先播一段低电动画再关机"，把函数体换成：
 *     LedShow_PlayLowBattery();     // 播完自动返回
 *     Battery_ShutdownNow();
 * 即可，其它地方一行都不用改（接口见 battery.h）。
 *
 * ⚠ 熄屏这一步不能省：电源自锁释放后，整机还要靠保持电容撑一小会儿，
 *   不熄屏的话最后那一帧会冻在屏上。 */
static void FluidDemo_LowBatteryAction(void)
{
    LedMatrix_Clear();
    LedMatrix_Flush();              /* 立即熄屏（约 1.2ms）*/

    Battery_ShutdownNow();          /* 释放电源自锁；不返回 */
}

/* ==================== 入口 ==================== */

/* ==================== 串口命令执行 ====================
 *
 * vcmd.c 只负责把文本翻译成"命令号 + 4 个参数"投进 dbg.cmd 邮箱；
 * 真正的动作在这里、在主循环的固定位置执行。这样命令生效的时机可预测，
 * 不会在 SPI 推屏中途改显示状态。
 *
 * 命令号见 debug.h 的 DBG_CMD_*，别名见 vcmd.c 的 s_alias[]。
 * 执行结果写 dbg.cmd.result：1 = 成功，2 = 参数越界，3 = 未知命令。
 */

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
 *            以后给自定义画面做 gamma 表，就按这条命令的取值为基准。
 *
 * 表长必须等于台阶数（LED_APP_ROWS - 2 = 14），下标 = y - 2。 */
static const uint8_t s_rampLinear[LED_APP_ROWS - 2u] = {
    0u, 19u, 39u, 58u, 78u, 98u, 117u, 137u, 156u, 176u, 196u, 215u, 235u, 255u
};

static const uint8_t s_rampGamma[LED_APP_ROWS - 2u] = {
    0u, 1u, 4u, 10u, 19u, 31u, 47u, 65u, 88u, 114u, 143u, 177u, 214u, 255u
};

/* 按给定的 14 级表画阶梯（第 2~15 行每行一个亮度），外框全亮。 */
static void FluidDemo_DrawRampFrom(const uint8_t *levels)
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

static void FluidDemo_DrawRamp(void)      { FluidDemo_DrawRampFrom(s_rampLinear); }
static void FluidDemo_DrawRampGamma(void) { FluidDemo_DrawRampFrom(s_rampGamma);  }

static void FluidDemo_CmdTask(void)
{
    uint16_t id;
    int32_t  a0, a1, a2;

    if (dbg.cmd.pending == 0u) {
        return;
    }

    id = dbg.cmd.id;
    a0 = dbg.cmd.arg[0];
    a1 = dbg.cmd.arg[1];
    a2 = dbg.cmd.arg[2];

    dbg.cmd.pending = 0u;
    dbg.cmd.result  = 1u;

    switch (id) {
        /* ---------------- 系统 ---------------- */
        case DBG_CMD_SET_DIV:
            VOFA_SetDivShift((uint32_t) a0);
            break;

        case DBG_CMD_SET_MASK:
            VOFA_SetMask((uint32_t) a0);
            break;

        case DBG_CMD_RESET_STATS:
            dbg.uart.rx_bytes  = 0u;
            dbg.uart.cmds      = 0u;
            dbg.uart.bad       = 0u;
            dbg.uart.tx_frames = 0u;
            dbg.uart.tx_drops  = 0u;
            dbg.uart.tx_stuck  = 0u;
            break;

        /* ---------------- 流体 ---------------- */
        case DBG_CMD_FLUID_SET_DT:
            /* 参数是 dt × 1e5（3200 = 0.032 s）。整数换算成 Q24，不引入浮点。 */
            if ((a0 >= 100) && (a0 <= 10000)) {
                Fluid_SetDt((_iq) (((int64_t) a0 << IQ_SHIFT) / 100000));
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_FLUID_SET_ITER:
            if ((a0 >= 0) && (a0 <= 32)) {
                Fluid_SetGridIter((uint32_t) a0);
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_FLUID_SET_SHAKE:
            /* 参数是增益 ×100（150 = 1.5） */
            if ((a0 >= 0) && (a0 <= 1000)) {
                s_shakeGain = (_iq) (((int64_t) a0 * IQ_ONE) / 100);
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_FLUID_PAUSE:
            s_paused = (a0 != 0) ? 1u : 0u;
            break;

        case DBG_CMD_FLUID_REINIT:
            Fluid_Init();
            break;

        /* ---------------- LED 点阵（亮度标定） ----------------
         * ⚠ 画静态画面的三条命令（fill / off / px / ramp）都会**自动置
         *   s_dispLock**：不锁的话下一帧就被流体覆盖了，根本来不及测。 */
        case DBG_CMD_LED_SET_BC:
            if ((a0 >= 0) && (a0 <= 255)) {
                s_userBc = (uint8_t) a0;
                HT16D33A_SetBrightness(s_userBc);
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_LED_SET_CC:
            if ((a0 >= 0) && (a0 <= 15)) {
                HT16D33A_WriteCmd(HT16D33A_CMD_SET_CURRENT, (uint8_t) a0);
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_LED_FILL:
            if ((a0 >= 0) && (a0 <= 255)) {
                uint8_t x, y;

                s_dispLock = 1u;
                LedMatrix_Clear();
                for (x = 1u; x <= LED_APP_COLS; x++) {
                    for (y = 1u; y <= LED_APP_ROWS; y++) {
                        LedMatrix_SetCellLevel(x, y, (uint8_t) a0);
                    }
                }
                LedMatrix_Flush();
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_LED_OFF:
            s_dispLock = 1u;
            LedMatrix_Clear();
            LedMatrix_Flush();
            break;

        case DBG_CMD_LED_PX:
            if ((a0 >= 1) && (a0 <= (int32_t) LED_APP_COLS) &&
                (a1 >= 1) && (a1 <= (int32_t) LED_APP_ROWS) &&
                (a2 >= 0) && (a2 <= 255)) {
                s_dispLock = 1u;
                LedMatrix_SetCellLevel((uint8_t) a0, (uint8_t) a1, (uint8_t) a2);
                LedMatrix_Flush();
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_LED_RAMP:
            s_dispLock = 1u;
            FluidDemo_DrawRamp();
            break;

        case DBG_CMD_LED_RAMP_GAMMA:
            s_dispLock = 1u;
            FluidDemo_DrawRampGamma();
            break;

        case DBG_CMD_LED_LOCK:
            s_dispLock = (a0 != 0) ? 1u : 0u;
            break;

        default:
            dbg.cmd.result = 3u;
            break;
    }
}

void FluidDemo_Run(void)
{
#if FLUID_USE_IMU
    uint32_t imuInitRes;
#endif
    uint32_t tiltStart = 0u;
    uint32_t statStart;
    uint32_t frameCnt = 0u;
    uint32_t accSim = 0u;
    uint8_t  tilt = 0u;
    uint8_t  useImu = 0u;
    _iq      ax = 0, ay = 0;
    uint32_t t0, t1;

    /* ---- 串口调试通道：**第一个初始化** ----
     * 后面任何一步失败（数学层自检、IMU、LED 芯片）都还能靠串口把原因报出来，
     * 比"整屏点亮当报错信号"有用得多。 */
    VCMD_Init();                            /* 武装 RX DMA */
    VOFA_Init();                            /* 遥测默认关闭，发 div 1 才开 */

    LedMatrix_Init();                       /* 芯片初始化 + 清屏 */

    /* 注册低电关机动作（熄屏 + 关机）。不注册则默认直接关机、不熄屏。 */
    Battery_SetLowAction(FluidDemo_LowBatteryAction);

    /* 数学层自检：坏了就把整屏点亮停住（还能长按 USER1 关机）*/
    dbg.fluid.iq_ok = FluidDemo_IqSelfTest();
    if (dbg.fluid.iq_ok == 0u) {
        LedMatrix_FillAll(1u);
        LedMatrix_Flush();
        for (;;) {
            PowerKey_CheckShutdown();
            VCMD_Tick();                    /* 至少还能从串口看到 rx_bytes / last_id */
            VOFA_Tick();
            __WFI();
        }
    }

#if FLUID_USE_IMU
    /* ---- 初始化 IMU，决定这次跑"真实重力"还是"假重力" ----
     * 初始化返回码写进 dbg.fluid.imu_ok，方便对照 lsm6ds3trc.h：
     *   0 = OK，1 = WHO_AM_I 不对（可能装的是 LSM6DSR 0x6B），2 = 寄存器读不回来
     * 失败时（若允许 fallback）退回假重力，程序照常跑，不会卡死。 */
    imuInitRes = LSM6DS3TRC_Init();
    dbg.fluid.imu_ok = (uint8_t) imuInitRes;

    if (imuInitRes == LSM6DS3TRC_OK) {
        useImu = 1u;
    } else {
#if FLUID_IMU_FALLBACK
        useImu = 0u;                        /* 退回假重力 */
#else
        /* 不允许退回：整屏点亮 + 停住，提示 IMU 有问题 */
        LedMatrix_FillAll(1u);
        LedMatrix_Flush();
        for (;;) {
            PowerKey_CheckShutdown();
            __WFI();
        }
#endif
    }
#else
    dbg.fluid.imu_ok = 0xFFu;               /* 0xFF = 编译期就没启用 IMU */
#endif

    dbg.fluid.imu_use = useImu;

    /* 流体初始化 + 预热一步（和原版 main 对齐）。
     * 预热用"向下"的重力，保证初态和以前一致，不受 IMU 朝向影响。 */
    Fluid_Init();
    Fluid_SetDt(DEMO_DT);
    Fluid_SetGridIter(DEMO_GRID_ITER);
    Fluid_Step(_IQ(0.0f), _IQ(9.8f));

    /* 滤波器复位：第一次读 IMU 时会用当时的读数播种（见 FluidDemo_ReadGravity），
     * 这样开机瞬间液体就朝真实重力方向，不会先"自由落体"一下 */
    s_axSlow     = 0;
    s_aySlow     = 0;
    s_sxFast     = 0;
    s_syFast     = 0;
    s_shakePk10  = 0u;
    s_filtSeeded = 0u;

#if HT16D33A_USE_BINARY_MODE
    /* 二进制模式才有的"上帧显示状态"：复位成全灭，避免开机用脏数据决定首帧。
     * 灰度模式没有状态（亮度每帧直接从密度算出来），不需要复位。 */
    {
        uint8_t ix, iy;

        for (ix = 0u; ix < LED_APP_COLS; ix++) {
            for (iy = 0u; iy < LED_APP_ROWS; iy++) {
                s_ledOn[ix][iy] = 0u;
            }
        }
    }
#endif

    dbg.fluid.tilt = tilt;
    tiltStart = tick_ms;
    statStart = tick_ms;

    for (;;) {
        /* 一键开关机：USER1 长按 2s 后由 TIMA0 中断拉低 PA22（释放电源自锁），
         * 这里只需要停下来别再干活，安静等硬件断电 */
        PowerKey_CheckShutdown();

        /* 低电检查调度：每 BAT_CHECK_MS 置一次"该熄屏采样了"。
         * 真正的采样在 FluidDemo_Push() 的安静窗口里做（见那里的注释），
         * 低电确认后会在那里直接关机。 */
        Battery_Task();

        /* ---- 串口：收命令（不阻塞，没数据时只读两个寄存器）---- */
        VCMD_Tick();

        /* ---- 执行命令：固定在这里生效，不会在推屏中途改显示状态 ---- */
        FluidDemo_CmdTask();

        if (useImu != 0u) {
            /* ---- 真实 IMU：读 → 选轴/符号/死区 → 双通路分解（重力 + 摇晃）---- */
            FluidDemo_ReadGravity(&ax, &ay);
            FluidDemo_TrackShakePeak();     /* 仅供 dbg.fluid.shake_pk10 观测 */
        } else {
            /* ---- 假重力：每 DEMO_TILT_MS 换一个方向（IMU 不可用时的退路）---- */
            if ((uint32_t) (tick_ms - tiltStart) >= DEMO_TILT_MS) {
                tiltStart += DEMO_TILT_MS;
                tilt = (uint8_t) ((tilt + 1u) & 3u);
                dbg.fluid.tilt = tilt;
            }

            ax = s_tiltAcc[tilt][0];
            ay = s_tiltAcc[tilt][1];
        }

        /* ---- 物理一步（`pause 1` 之后冻结，方便静态观察 / 测量）---- */
        if (s_paused == 0u) {
            t0 = tick_ms;
            Fluid_Step(ax, ay);
            t1 = tick_ms;
            accSim += (t1 - t0);
        }

        /* ---- 渲染 + 推屏（不计入物理耗时）---- */
        FluidDemo_Push();

        /* ---- 串口：发遥测（DMA 忙就丢帧，绝不等待）---- */
        VOFA_Tick();

        /* ---- 每 DEMO_STAT_FRAMES 帧统计一次（原来这些数在 OLED 的 HUD 上）---- */
        frameCnt++;
        if (frameCnt >= DEMO_STAT_FRAMES) {
            uint32_t elapsed = tick_ms - statStart;

            dbg.fluid.sim_ms_x10 = (accSim * 10u) / frameCnt;
            dbg.fluid.fps        = (elapsed > 0u) ? ((frameCnt * 1000u) / elapsed) : 0u;
            dbg.fluid.frames     = Fluid_GetFrameCount();
            dbg.fluid.ax10       = FluidDemo_SimX10(ax);
            dbg.fluid.ay10       = FluidDemo_SimX10(ay);

            /* 双通路分解的观测值（原理见文件顶部《重力 / 摇晃：双通路分解》）
             * 静止时应为：slow ≈ ±98（1g 向下）、fast ≈ 0；
             * 左右摇时 fast 大幅摆动、slow 基本不动。 */
            dbg.fluid.ax_slow10  = FluidDemo_SimX10(s_axSlow);
            dbg.fluid.ay_slow10  = FluidDemo_SimX10(s_aySlow);
            dbg.fluid.ax_fast10  = FluidDemo_SimX10(_IQmpy(s_shakeGain, s_sxFast));
            dbg.fluid.ay_fast10  = FluidDemo_SimX10(_IQmpy(s_shakeGain, s_syFast));
            dbg.fluid.shake_pk10 = s_shakePk10;
            s_shakePk10 = 0u;       /* 窗口量：发布完清零重算 */

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
