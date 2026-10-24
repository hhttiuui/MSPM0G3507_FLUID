/*
 * fluid_demo.c —— 流体模拟 → LED 点阵（IMU 重力驱动 / 假重力可切换）
 *
 * 职责划分：
 *   fluid.c        纯算法，零外设依赖（可以原样丢到 PC 上跑）
 *   led_matrix.c   面板映射层：(列,行) → 芯片显示 RAM 位
 *   led_show.c     点阵综合演示（另一个可选程序，见 LED_SHOW_ENABLE）
 *   lsm6ds3trc.c   LSM6DS3TR-C 六轴 IMU 的 SPI 驱动（只提供原始/物理量）
 *   app.c          场景框架 + 输入层（IMU 双通路 / 按键 / 场景切换）
 *   scene_still.c  场景 1：静态图（px / fill / ramp，亮度标定用）
 *   fluid_demo.c   本文件 = **场景 0：流体**。只做算法 ↔ 点阵的胶水：
 *                  ① particleDensity[] → 亮度 → LedMatrix → 推屏
 *                     （灰度：内核亮、表层暗；二进制模式下自动退回阈值+滞回）
 *                  ② 观测量全部在 dbg.fluid（没有屏，参数只能这么看）
 *
 * ⚠ 重力 / 摇晃 / 按键 / 主循环都搬到 app.c 了。本文件**不再读 IMU**，
 *   加速度从 app_in_t 传进来 —— 这样所有场景共用同一次采样与同一套滤波，
 *   不会出现"两个场景各读一次把滤波器打乱"的问题。
 *
 * ==================== 加速度从哪来 ====================
 *
 * 轴向映射怎么标定、双通路（重力 / 摇晃）为什么这么拆、死区为什么有两个，
 * **全部在 app.c 顶部**。本文件只是消费者：`in->ax / in->ay` 已经是
 * 屏幕坐标系下、合成并限幅好的模拟单位。
 *
 * dbg.fluid.ax10 / ay10 是本文件**送进流体前**的值（模拟单位 ×10），
 * 可以直接确认映射对不对：正着放时应为 (0, +98) 即"向下"。
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
#include "app.h"            /* 场景框架：app_in_t / App_Brightness */
#include "scenes.h"         /* 本文件的 FluidScene_* 声明 */

/* ==================== 配置 ==================== */

/* ---- IMU / 重力 / 摇晃的全部参数与推导都搬到了 app.c ----
 * 这里只留**流体自己**的参数。要看轴向映射怎么标定、双通路为什么这么拆，
 * 见 app.c 顶部的《重力来源与坐标映射》和《重力 / 摇晃：双通路分解》。 */

/* 时间步长：原版实测 0.032 时手感最好（一步物理约 28ms，dt 推 32ms ≈ 1.1 倍实时）*/
#define DEMO_DT             _IQ(0.032f)

/* 压力求解迭代次数（越大液体越"硬"、也越慢）*/
#define DEMO_GRID_ITER      12u

/* 每多少帧统计一次 dbg.fluid 的耗时/帧率 */
#define DEMO_STAT_FRAMES    20u

/* ---- 运行期可调状态（串口命令改这些，见 FluidScene_Cmd）----
 * 上面那些 DEMO_* 宏是"默认值"，这里才是真正生效的变量。
 * 为什么不直接用宏：串口调参的价值就在于不改代码、不重编译。 */
static uint8_t  s_paused;                         /* pause 1 = 冻结物理 */
static uint8_t  s_dispLock;                       /* lock  1 = 冻结渲染 */



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

/* 渲染的两个"观感"旋钮，串口可调，默认值 = 原来写死的宏（行为不变）：
 *   s_gammaOn     ：0 = 线性映射（原样）；1 = 走 led_matrix 的 gamma2.2 表
 *   s_densFullPct ：满亮度对应的密度 = 静止密度 × 这个百分比 / 100
 *
 * 为什么两个都要：gamma 只改中间调，**救不了已经截断到 255 的内核**。
 * 内核一片白是因为 s_densFullPct = 60 太低 —— 液体内部密度普遍超过
 * 静止密度的 60%，全被截到 255 了。想把层次拉出来得调大这个值。 */
static uint8_t  s_gammaOn      = 0u;
static uint16_t s_densFullPct  = FLUID_DENS_FULL_PCT;

/* ---- 边缘分段映射（"抗锯齿"）----
 *
 * 15×16 的网格上没有亚像素可用，所以"抗锯齿"只能是**把边缘那段密度单独拉开**：
 * 边缘格子的密度只有 0~0.3，线性映射算出来是 0~40，而人眼几乎分不出 0 和 40，
 * 边界看起来就是"突然出现"的一格。
 *
 * 映射曲线（两段，在 dEdge 处**连续**，不会有台阶）：
 *
 *       255 ┤                    ╭────────────  d ≥ dKnee：内核，不动
 *           │                 ╱
 *   offBody ┤──────────────╱                   dEdge ≤ d < dKnee：接到 255
 *           │            ╱
 *         0 ┴───────────╯                       d < dEdge：边缘段，斜率单独设
 *           0        dEdge              dKnee
 *
 *   dEdge   = dKnee × s_edgePct / 100
 *   offBody = 255 × s_edgeGain / 100     ← 边缘段最亮到哪
 *
 * ⚠ 方向由 gain 和 pct 的**相对大小**决定，不是单独看 gain（最容易搞混的一点）：
 *
 *   gain > pct   → 边缘比线性更亮、过渡更宽，像一圈柔光（抗锯齿感）
 *                  例 edge 15 50：t=0.05 处从 13 提到 43
 *   gain < pct   → 边缘比线性更暗，边界更硬，像描了一圈黑边
 *                  例 edge 30 15：t=0.05 处从 13 压到 6
 *   gain == pct  → 与线性**完全逐点一致**（退化成原样，可用来校验参数）
 *                  例 edge 15 15
 *
 * 原因是两段在 dEdge 处的值必须相等才不会出现台阶：
 *   边缘段终点 = 255×gain/100，线性映射在 dEdge 处 = 255×pct/100
 * 两者相等 ⟺ gain == pct，此时两段斜率也相同，整体就是原来的线性映射。
 *
 * 默认 s_edgeOn = 0，完全走原来的单段线性映射，行为不变。 */
static uint8_t  s_edgeOn       = 0u;
static uint8_t  s_edgePct      = 15u;   /* 边缘段占满量程的比例 % */
static uint8_t  s_edgeGain     = 50u;   /* 边缘段最亮 = 255 × n% */

/* 二进制模式的阈值（只有在 HT16D33A_USE_BINARY_MODE=1 时才用得上，
 * 但一直定义着，好让 FluidScene_Tick 里上报 dbg 的代码两种模式都能编译）*/
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
        HT16D33A_SetBrightness(App_Brightness());  /* 恢复用户设定的亮度，不是写死 MAX */
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
    _iq     dKnee    = 0;       /* 满亮度密度（Q24）*/
    _iq     dEdge    = 0;       /* 边缘段终点（Q24）*/
    int32_t kEdge    = 0;       /* 边缘段斜率 */
    int32_t kBody    = 0;       /* 内核段斜率 */
    int32_t offBody  = 0;       /* 内核段起点亮度 */

    if (densFull <= _IQ(0.0f)) {
        densFull = FLUID_DENS_FULL_FALLBACK;
    }
    k = (int32_t) ((((int64_t) FLUID_LEVEL_MAX) << IQ_SHIFT) * 100
                   / ((int64_t) s_densFullPct * (int64_t) densFull));

    /* 每帧只算一次，逐格只做查表级的两三次乘加，开销可忽略 */
    if (s_edgeOn != 0u) {
        int32_t peak;

        dKnee = (_iq) (((int64_t) densFull * (int64_t) s_densFullPct) / 100);
        peak  = (int32_t) (((int64_t) FLUID_LEVEL_MAX * s_edgeGain) / 100);

        dEdge   = (_iq) (((int64_t) dKnee * s_edgePct) / 100);
        offBody = peak;

        if (dEdge > 0) {
            kEdge = (int32_t) (((int64_t) peak << IQ_SHIFT) / dEdge);
        }
        if (dKnee > dEdge) {
            kBody = (int32_t) (((int64_t) (FLUID_LEVEL_MAX - peak) << IQ_SHIFT)
                               / (dKnee - dEdge));
        }
    }

    for (x = 1u; x <= LED_APP_COLS; x++) {          /* 15 列 */
        for (y = 1u; y <= LED_APP_ROWS; y++) {      /* 16 行 */
            _iq d = density[CELL_INDEX((uint32_t) x, (uint32_t) y)];

            if (d > dMax) {
                dMax = d;
            }

            if (s_edgeOn != 0u) {
                /* 两段连续映射：边缘段单独斜率，内核段接到 255 */
                if (d < dEdge) {
                    lv = _IQmpy(d, kEdge);
                } else if (d < dKnee) {
                    lv = offBody + _IQmpy(d - dEdge, kBody);
                } else {
                    lv = FLUID_LEVEL_MAX;
                }
            } else {
                lv = (d > 0) ? _IQmpy(d, k) : 0;
                if (lv > FLUID_LEVEL_MAX) {
                    lv = FLUID_LEVEL_MAX;   /* 内核超出满量程就截断（会一片白）*/
                }
            }

            if (lv > 0) {
                /* gamma 只作用于**送进显示器的值**；下面两个统计仍用线性 lv，
                 * 这样 dbg.fluid.dens_lit / dens_lit_core 在 gam 开关前后可比。 */
                uint8_t out = (uint8_t) lv;

                if (s_gammaOn != 0u) {
                    out = LedMatrix_Gamma(out);
                }
                LedMatrix_SetCellLevel(x, y, out);

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

/* 窗口内摇晃幅度的峰值 ×10，只给 dbg.fluid.shake_pk10 看。
 * 输入层（app.c）给的是**瞬时值** in->shake10，累计成窗口峰值是场景自己的事 ——
 * 这样"多快算一次峰"由场景决定，输入层不用猜。 */
static uint32_t s_shakePk10;

/* ==================== 数学层自检 ==================== */

/* 先确认 _IQmpy/_IQdiv/_IQsqrt 都是对的，再让仿真跑，免得白排查。
 * 失败没法在屏幕上写字了，所以用"整屏点亮 + 停下"当报错信号，结果也记在
 * dbg.fluid.iq_ok 里。 */
uint8_t FluidDemo_IqSelfTest(void)
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

/* 低电关机动作搬到 app.c 了（App_LowBatteryAction）—— 它是系统级动作，
 * 不属于任何场景。battery.c 的接口没变，见 battery.h。 */

/* ==================== 场景 0：流体 的命令 ====================
 *
 * 只处理**流体自己**的命令。全局命令（遥测 / 亮度 / 恒流 / 摇晃增益 / 场景切换）
 * 在 app.c 的 App_CmdTask 里就处理掉了，根本到不了这里。
 *
 * 返回值：1 = 已处理，0 = 不是我的（app 会填 dbg.cmd.result = 3）。
 * 参数越界时自己填 result = 2。
 */
uint8_t FluidScene_Cmd(uint16_t id, const int32_t *arg)
{
    switch (id) {
        case DBG_CMD_FLUID_SET_DT:
            /* 参数是 dt × 1e5（3200 = 0.032 s）。整数换算成 Q24，不引入浮点。 */
            if ((arg[0] >= 100) && (arg[0] <= 10000)) {
                Fluid_SetDt((_iq) (((int64_t) arg[0] << IQ_SHIFT) / 100000));
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_FLUID_SET_ITER:
            if ((arg[0] >= 0) && (arg[0] <= 32)) {
                Fluid_SetGridIter((uint32_t) arg[0]);
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_FLUID_PAUSE:
            s_paused = (arg[0] != 0) ? 1u : 0u;
            return 1u;

        case DBG_CMD_FLUID_REINIT:
            Fluid_Init();
            return 1u;

        case DBG_CMD_FLUID_SET_FULL:
            /* 满亮度密度 = 静止密度 × arg0 / 100。
             * 调大 → 只有最密的内核才到 255，层次更丰富但整体变暗；
             * 调小 → 更容易截断，整片发白。 */
            if ((arg[0] >= 20) && (arg[0] <= 300)) {
                s_densFullPct = (uint16_t) arg[0];
            } else {
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_FLUID_EDGE:
            /* edge <pct> <gain>：pct = 边缘段占满量程的比例，gain = 边缘段最亮百分比。
             * edge 0 → 关闭，回到单段线性。 */
            if (arg[0] == 0) {
                s_edgeOn = 0u;
            } else if ((arg[0] > 0) && (arg[0] < (int32_t) s_densFullPct) &&
                       (arg[1] >= 1) && (arg[1] <= 100)) {
                s_edgePct  = (uint8_t) arg[0];
                s_edgeGain = (uint8_t) arg[1];
                s_edgeOn   = 1u;
            } else {
                /* pct 必须小于 full；gain 必须给（漏参数会被当成 0）*/
                dbg.cmd.result = 2u;
            }
            return 1u;

        case DBG_CMD_LED_LOCK:
            /* 冻结 / 恢复流体渲染。切场景时会自动清掉（见 FluidScene_Enter）。 */
            s_dispLock = (arg[0] != 0) ? 1u : 0u;
            return 1u;

        case DBG_CMD_LED_GAMMA:
            s_gammaOn = (arg[0] != 0) ? 1u : 0u;
            return 1u;

        default:
            return 0u;
    }
}

/* ==================== 场景 0：流体 ====================
 *
 * enter 的语义：**第一次**进来做完整初始化（Fluid_Init + 预热一步），
 * 之后每次切回来只复位开关和统计，**不动粒子** —— 液体保持上次的形态接着流。
 *
 * 为什么不每次 Fluid_Init：
 *   ① Fluid_Init 要重排 170 个粒子，代价不小；
 *   ② 切回来看到"液体瞬移重排"很出戏，保持原样才像"暂停了一下"。
 * 想强制重来，串口发 `reinit` 即可。
 */

static uint8_t  s_inited;           /* 0 = 还没做过完整初始化 */
static uint32_t s_statStart;
static uint32_t s_frameCnt;
static uint32_t s_accSim;

void FluidScene_Enter(void)
{
    if (s_inited == 0u) {
        Fluid_Init();
        Fluid_SetDt(DEMO_DT);
        Fluid_SetGridIter(DEMO_GRID_ITER);

        /* 预热一步，用"向下"的重力，保证初态和以前一致，不受 IMU 朝向影响 */
        Fluid_Step(_IQ(0.0f), _IQ(9.8f));
        s_inited = 1u;
    }

#if HT16D33A_USE_BINARY_MODE
    /* 二进制模式才有的"上帧显示状态"：复位成全灭，避免用脏数据决定首帧。
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

    /* pause / lock 是"这一次访问"的开关，切场景时复位 ——
     * 否则上次 `lock 1` 冻住画面，切回来还以为板子卡死了 */
    s_paused    = 0u;
    s_dispLock  = 0u;
    s_frameCnt  = 0u;
    s_accSim    = 0u;
    s_shakePk10 = 0u;
    s_statStart = tick_ms;
}

void FluidScene_Exit(void)
{
    /* 什么都不用做：App_Goto 会统一清屏，粒子状态留着下次接着用 */
}

void FluidScene_Tick(const app_in_t *in)
{
    uint32_t t0, t1;

    /* 窗口摇晃峰值：输入层给瞬时值，这里累计成"每 DEMO_STAT_FRAMES 帧一个峰" */
    if ((uint32_t) in->shake10 > s_shakePk10) {
        s_shakePk10 = (uint32_t) in->shake10;
    }

    /* ---- 物理一步（`pause 1` 之后冻结，方便静态观察 / 测量）---- */
    if (s_paused == 0u) {
        t0 = tick_ms;
        Fluid_Step(in->ax, in->ay);
        t1 = tick_ms;
        s_accSim += (t1 - t0);
    }

    /* ---- 渲染 + 推屏（不计入物理耗时）---- */
    FluidDemo_Push();

    /* ---- 每 DEMO_STAT_FRAMES 帧统计一次（原来这些数在 OLED 的 HUD 上）---- */
    s_frameCnt++;
    if (s_frameCnt >= DEMO_STAT_FRAMES) {
        uint32_t elapsed = tick_ms - s_statStart;

        dbg.fluid.sim_ms_x10 = (s_accSim * 10u) / s_frameCnt;
        dbg.fluid.fps        = (elapsed > 0u) ? ((s_frameCnt * 1000u) / elapsed) : 0u;
        dbg.fluid.frames     = Fluid_GetFrameCount();
        dbg.fluid.ax10       = FluidDemo_SimX10(in->ax);
        dbg.fluid.ay10       = FluidDemo_SimX10(in->ay);

        /* 双通路分解的观测值（原理见 app.c 的《双通路分解》）
         * 静止时应为：slow ≈ ±98（1g 向下）、fast ≈ 0；
         * 左右摇时 fast 大幅摆动、slow 基本不动。 */
        dbg.fluid.ax_slow10  = FluidDemo_SimX10(in->ax_slow);
        dbg.fluid.ay_slow10  = FluidDemo_SimX10(in->ay_slow);
        dbg.fluid.ax_fast10  = FluidDemo_SimX10(in->shx);
        dbg.fluid.ay_fast10  = FluidDemo_SimX10(in->shy);
        dbg.fluid.shake_pk10 = s_shakePk10;
        s_shakePk10 = 0u;       /* 窗口量：发布完清零重算 */

        s_accSim    = 0u;
        s_frameCnt  = 0u;
        s_statStart = tick_ms;

        if (dbg.test.heartbeat != 0u) {
            /* 翻指示灯做心跳：主循环卡住时它就不闪了，一眼能看出来 */
            DL_GPIO_togglePins(LED_PORT, LED_PIN_LED_PIN);
        }
    }
}
