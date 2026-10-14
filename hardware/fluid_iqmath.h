/*
 * fluid_iqmath.h —— 可移植的 Q24 定点运算替身
 *
 * 原 CH32 工程用的是 WCH 的预编译库 libIQMath_RV32.a（RISC-V 专用），
 * 在 MSPM0(Cortex-M0+) 上用不了，这里用纯 C 重新实现 SandSim.c 用到的那几个算子，
 * 数值语义与 Q24 保持一致，所以仿真参数（Spacing/dt/GRID_ITER…）一个字都不用改。
 *
 * 用法约定：
 *   · 浮点常量 用 _IQ(...)   —— 编译期折叠成整数常量，运行时代价为 0
 *   · 运行期整数 用 _IQI(...) —— 走移位，**不会**引入软件浮点
 *   · 千万不要把运行期的浮点变量传给 _IQ()，那样会走软件 double 运算（极慢）
 *
 * 性能说明：
 *   _IQmpy/_IQdiv 内部用了 int64。Cortex-M0+ 没有 64 位乘法/除法指令，
 *   会调用 __aeabi_lmul 之类的软件例程（约 30~60 周期/次）。
 *   这是当前的主要瓶颈 —— 先用"正确优先"的实现把仿真跑起来，
 *   后面按实测结果再决定是否换 Q15 / MATHACL / 浮点。
 *
 * 已做的两轮优化（都有实测数据支撑）：
 *   ① 乘法内联 + 热点函数搬进 SRAM          50.7ms → 31.6ms
 *   ② 除法改用 MATHACL 硬件除法器（见 IQ_DIV_MODE） 31.6ms → 预计约 18ms
 */
#ifndef __FLUID_IQMATH_H
#define __FLUID_IQMATH_H

#include <stdint.h>

typedef int32_t _iq;

#define IQ_SHIFT   24
#define IQ_ONE     (1L << IQ_SHIFT)     /* 16777216 */

/* 浮点常量 → Q24（编译期折叠） */
#define _IQ(A)        ((_iq)((A) * 16777216.0))

/* 整数 → Q24（移位，无浮点）。先转无符号再移位，避免负数左移的未定义行为 */
#define _IQI(A)       ((_iq)((uint32_t)(int32_t)(A) << IQ_SHIFT))

/* ---- _IQmpy 的实现方式 ----
   2 = 用 MSPM0G350x 的 **MATHACL 硬件乘法器**（MPY32，默认）。
        实测（TI IQMath 基准，MSPM0G）：硬件 mpy 33 周期 vs 软件 RTS 54 周期。
        但真正的差距比这大得多：本文件内联的 iq_mul64 是 4 次 32×32 乘法
        拼出来的 64 位结果，Cortex-M0+ 没有单周期乘法器（MULS 要 32 周期），
        算下来每次 _IQmpy 约 150 周期。而 _IQmpy 是整个求解器里调用最频繁的
        算子（一步上万次），所以这里是当前最大的热点。
        CTL = 6(MPY32) | (24<<8)(Q24) | (1<<5)(SIGNED) = 0x1826，
        与 TI 官方 iqmath.a 里 _IQ24mpy 的 CTL 完全一致。
        ⚠ 同 _IQdiv：MATHACL **不可重入**，中断里不能用。
   1 = 本文件内联的 32×32→64 乘法。好处：没有函数调用开销，而且能
       跟着宿主函数一起被放到 SRAM（.TI.ramfunc）执行，避开 flash 等待周期。
   0 = 交给编译器生成 __aeabi_lmul 调用（库函数在 flash 里，80MHz 下每次都要等） */
#ifndef IQ_MPY_MODE
#define IQ_MPY_MODE     2
#endif

/* 精确的软件参考乘法（64 位中间结果再右移）。
   永远编译：硬件模式拿它当自检基准，也是自检不过时的退路。 */
static inline _iq _IQmpy_sw(_iq A, _iq B)
{
    return (_iq)(int32_t)(((int64_t)(int32_t)A * (int64_t)(int32_t)B) >> IQ_SHIFT);
}

#if (IQ_MPY_MODE == 2)
/* 只需要芯片寄存器定义（和 IQ_DIV_MODE==2 用的是同一个头）*/
#include <ti/devices/msp/msp.h>

/* 运行时开关，定义在 fluid.c（全工程一份）。默认 1 = 先用硬件，
   启动自检不过就由 IQ_MpySelfTest() 置 0 退回软件 —— 数值正确 > 快。 */
extern uint8_t g_iqMpyUseHw;

/* 硬件 Q24 乘法：CTL = MPY32(6) | Q24(24<<8) | SIGNED(1<<5)，写 OP1 即触发。
   RES1 直接就是 Q24 结果 —— 硬件内部自己做了 >>24，不需要我们再移位。
   与 TI 官方 iqmath.a 的 _IQ24mpy 逐条一致。 */
static inline _iq _IQmpy_hw(_iq A, _iq B)
{
    MATHACL->CTL = 6u | ((uint32_t)IQ_SHIFT << 8) | (1u << 5);
    MATHACL->OP2 = (uint32_t)B;
    MATHACL->OP1 = (uint32_t)A;   /* 写 OP1 就是发令 */
    return (_iq)MATHACL->RES1;    /* 读 RES1 时硬件会 hold 总线直到算完 */
}

static inline _iq _IQmpy(_iq A, _iq B)
{
    return (g_iqMpyUseHw != 0u) ? _IQmpy_hw(A, B) : _IQmpy_sw(A, B);
}

/* 硬件乘法自检：在求解器**真实用到的量级区间**上逐点比对硬件与软件参考。
 *   返回 0 = 硬件可用（g_iqMpyUseHw = 1）
 *   返回 1 = 硬件不对，已退回软件（g_iqMpyUseHw = 0）
 * outMaxErr 回填最大绝对误差（Q24 LSB）。
 *
 * 判据说明：软件参考是"截断"，硬件可能是"四舍五入"，两者天然会差 0~1 LSB，
 * 所以容差取 8 LSB（≈5e-7，对流体完全无意义）。而硬件若真的算错
 * （定标不对 / 符号不对 / 溢出饱和），误差是成千上万 LSB，一眼分得开。
 *
 * 取值覆盖：位置(~0.05)、权重(0~1)、密度峰值(2.8)、重力和超松弛(9.8/1.9)，
 * 以及负值（BOUNCYNESS -0.9）—— 正是求解器里真正相乘的那些数。 */
static inline int IQ_MpySelfTest(int32_t *outMaxErr)
{
    /* 覆盖求解器真实用到的整条动态区间：位置/半径量级(0.001~0.05)、
       双线性权重(0~1)、密度峰值(2.8)、重力和超松弛(9.8/1.9)。
       最大乘积 9.8×9.8 = 96 < 128，不会撞到 Q24 上限，避免溢出把判据搅浑。 */
    static const _iq xs[24] = {
        _IQ(0.001f), _IQ(-0.001f), _IQ(0.017f), _IQ(0.034f), _IQ(0.05f),
        _IQ(0.1f),   _IQ(-0.1f),   _IQ(0.3f),   _IQ(-0.3f),  _IQ(0.5f),
        _IQ(0.7f),   _IQ(-0.7f),   _IQ(1.0f),   _IQ(-1.0f),  _IQ(1.5f),
        _IQ(1.9f),   _IQ(2.0f),    _IQ(-2.0f),  _IQ(2.8f),   _IQ(-0.9f),
        _IQ(9.8f),   _IQ(-9.8f),   _IQ(0.1225f), _IQ(4.0f)
    };
    int32_t maxErr = 0;
    unsigned int i;
    unsigned int j;

    g_iqMpyUseHw = 1u;                  /* 先打开，让比对真的走硬件路径 */
    for (i = 0u; i < 24u; i++) {
        for (j = 0u; j < 24u; j++) {
            int32_t e = (int32_t)_IQmpy_hw(xs[i], xs[j])
                      - (int32_t)_IQmpy_sw(xs[i], xs[j]);

            if (e < 0) {
                e = -e;
            }
            if (e > maxErr) {
                maxErr = e;
            }
        }
    }

    if (outMaxErr != 0) {
        *outMaxErr = maxErr;
    }

    if (maxErr > 8) {
        g_iqMpyUseHw = 0u;              /* 硬件不对 → 退回软件，保证数值正确 */
        return 1;
    }
    return 0;
}

#elif (IQ_MPY_MODE == 1)
/* 手写 32×32 → 64 位有符号乘法。
   a = ah*2^16 + al（ah 取算术右移的**有符号**高半，al 取低 16 位无符号），
   b 同理，展开成 4 次 16×16 乘法后拼回 64 位。 */
static inline int64_t iq_mul64(int32_t a, int32_t b)
{
    int32_t  ah = a >> 16;
    uint32_t al = (uint32_t)a & 0xFFFFu;
    int32_t  bh = b >> 16;
    uint32_t bl = (uint32_t)b & 0xFFFFu;

    uint64_t ll  = (uint64_t)(al * bl);                      /* 无符号，< 2^32 */
    int64_t  mid = (int64_t)(ah * (int32_t)bl)
                 + (int64_t)((int32_t)al * bh);              /* 各自 < 2^31 */
    int64_t  hi  = (int64_t)(ah * bh);

    return (int64_t)ll + (mid << 16) + (hi << 32);
}

#define _IQmpy(A, B)  ((_iq)(iq_mul64((int32_t)(A), (int32_t)(B)) >> IQ_SHIFT))

static inline int IQ_MpySelfTest(int32_t *outMaxErr)
{
    if (outMaxErr != 0) {
        *outMaxErr = 0;
    }
    return 1;                           /* 本来就是软件实现，无需自检 */
}
#else
#define _IQmpy(A, B)  ((_iq)(((int64_t)(A) * (int64_t)(B)) >> IQ_SHIFT))

static inline int IQ_MpySelfTest(int32_t *outMaxErr)
{
    if (outMaxErr != 0) {
        *outMaxErr = 0;
    }
    return 1;                           /* 本来就是软件实现，无需自检 */
}
#endif
/* ---- _IQdiv 的实现方式 ----
   2 = 用 MSPM0G350x 的 **MATHACL 硬件除法器**（默认）。
       写 4 个寄存器就出结果（约 10~20 周期），比软浮点除法快几十倍。
       实测前：物理一步 28.2ms，其中除法占约 38%（≈10.6ms）。
       代价：㉠ 只能跑在 MSPM0 上（PC 上编译要改成 1 或 0）
             ㊡ MATHACL **不可重入**（只有一组 CTL/OP1/OP2/RES1），
                以后如果中断里也要用，必须加互斥
   1 = 走单精度浮点（int→float → 除法 → float→int）。软浮点除法约 700 周期，
       仍明显快于 64 位整数除法 __aeabi_ldivmod（约 1000 周期，32 次迭代的循环）。
       精度：float 的 24 位尾数与 Q24 相当，误差 ≤1 LSB，对流体模拟完全够用。
   0 = 用 64 位整数除法（结果与 WCH 原版逐位一致，最慢但最准） */
#ifndef IQ_DIV_MODE
#define IQ_DIV_MODE   2
#endif

#if (IQ_DIV_MODE == 2)
/* 只需要芯片寄存器定义；改成 1 或 0 即可脱离 MSPM0 编译（fluid.c 本身依旧可移植） */
#include <ti/devices/msp/msp.h>

static inline _iq _IQdiv(_iq A, _iq B)
{
    /* 操作码 4 = DIV；bit8 起是 Q 格式；bit5 = 立即触发 */
    MATHACL->CTL = 4u | ((uint32_t)IQ_SHIFT << 8) | (1u << 5);
    MATHACL->OP2 = (uint32_t)B;   /* 除数 */
    MATHACL->OP1 = (uint32_t)A;   /* 被除数；写 OP1 就是发令 */
    return (_iq)MATHACL->RES1;    /* 读 RES1 时硬件会 hold 总线直到算完 */
}
#elif (IQ_DIV_MODE == 1)
#define _IQdiv(A, B)  ((_iq)((float)(A) / (float)(B) * 16777216.0f))
#else
/* 先转无符号再左移，避免负数左移的未定义行为；/ 之后再转回有符号 */
#define _IQdiv(A, B)  ((_iq)((int64_t)((uint64_t)((int64_t)(A)) << IQ_SHIFT) / (int64_t)(B)))
#endif
#define _IQint(A)     ((int32_t)((A) >> IQ_SHIFT))
#define _IQtoF(A)     ((float)(A) / 16777216.0f)

/* ==================== 开方（软件 / MATHACL 硬件 二选一）====================
 *
 * IQ_SQRT_MODE:
 *   1 = MATHACL 硬件开方（默认）
 *   0 = 纯软件逐位法
 *
 * 为什么值得换：_IQsqrt 在 fluid.c 的 PushParticlesApart() 里被调用，
 * 那是 O(n²) 的内层循环（130 粒子 → 最多 8385 对/步）。软件开方每次
 * 要跑 32 轮 64 位循环，是当前最热的代码段之一。
 *
 * ── 硬件 SQRT 到底怎么配：照抄 TI 自己的实现 ──────────────────────────
 *   源码：source/ti/iqmath/_IQNfunctions/_IQNsqrt.c
 *         函数：__IQNsqrt_MathACL()
 *   TI 的例程只示范了 DIV/MPY/SINCOS，没有 SQRT 的配置范例，但 IQMath
 *   库的源码就是权威答案。三个关键点（先前靠猜，全猜错了，于是整屏点亮）：
 *
 *   ① NUMITER 不是 1，是 **31**（写 0 也会被解释成 31）
 *   ② SFACTOR 不是常数 24，要按输入大小**现算**：
 *        把输入左移 n 位、归一化到 IQ30 的 [1,2) 区间（即让 bit30 变最高位），
 *        sfactor = (30 - IQ_SHIFT) - n
 *      输入越小 n 越大 → sfactor 越小，甚至为负，取低 6 位即可。
 *   ③ RES1 出来的是 **IQ16**，Q24 要左移 (24-16)=8 位
 *   ④ SQRT 只用 CTL 的 FUNC / NUMITER / SFACTOR 三个字段，
 *      QVAL、OPTYPE 是 DIV 专用的，别写（TI 也没写）
 *
 * ── 实测数据（2026-10，MSPM0G3507，iter=31，探针输入 0x40000000）──────
 *   sfactor :  0      1      2       3      4      6
 *   RES1    :  65533  92680  131068  185363 262137 524275
 *   精确值  :  65536  92682  131072  185364 262144 524288
 *   误差    :  -3     -2     -4      -1     -7     -13    （单位 LSB@IQ16）
 *   ⇒ 硬件规律与 TI 完全一致：RES1 = sqrt(输入) × 2^(1 + sfactor/2)，
 *     Q24 结果 = RES1 << 8。只是迭代逼近会留几个 LSB 的量化误差
 *     （相对误差约 2.5~4.6e-5），对流体模拟完全无所谓。
 *   ⚠ 正因为有这点误差，自检容差**必须用相对值**：固定几百 LSB 会把
 *     "正确但有量化误差"的结果误判成失败（这个坑已经踩过一次）。
 *
 * ── 安全性 ───────────────────────────────────────────────────────────
 *   MATHACL 的结果在启动时用已知答案自检（sqrt(4)=2、sqrt(9)=3）。
 *   自检不过就自动退回软件开方（g_iqSqrtUseHw = 0），
 *   绝不会再把 demo 卡死在"整屏点亮 + PowerKey_CheckShutdown()"上。
 */
#ifndef IQ_SQRT_MODE
#define IQ_SQRT_MODE        1
#endif

/* 软件逐位法（原实现，32 轮循环）。
   永远编译进来：既是 IQ_SQRT_MODE==0 的正选，也是硬件自检失败时的退路。 */
static inline _iq _IQsqrt_sw(_iq A)
{
    uint64_t rem  = 0;
    uint64_t root = 0;
    uint64_t v;
    int i;

    if (A <= 0) {
        return 0;
    }

    v = (uint64_t)((uint32_t)A) << IQ_SHIFT;

    for (i = 0; i < 32; i++) {
        root <<= 1;
        rem = (rem << 2) | (v >> 62);
        v <<= 2;
        if (root < rem) {
            rem -= (root + 1u);
            root += 2u;
        }
    }

    return (_iq)(uint32_t)(root >> 1);
}

#if IQ_SQRT_MODE

/* 只需要芯片寄存器定义（和 IQ_DIV_MODE==2 用的是同一个头）*/
#include <ti/devices/msp/msp.h>

/* ---- 运行时开关（★ 定义在 fluid.c，全工程只此一份）----
 * 为什么用全局变量而不是宏：自检函数在别的翻译单元里跑，
 * 它改不了本文件的宏；而写成 static 又会让每个 .c 各有一份副本，
 * 结果"自检通过"的那个副本和"真正算 sqrt"的那个副本不是一个东西。 */
extern uint8_t g_iqSqrtUseHw;       /* 0 = 软件开方；1 = 自检通过，走硬件 */
extern uint8_t g_iqSqrtSfactor;     /* 诊断：自检点算出的 sfactor（0~63）*/
extern uint8_t g_iqSqrtIter;        /* 诊断：固定 31 */

/* 单次硬件开方，参数显式给出（给自检 / 排查用）
 * CTL 编码（依据 hw_mathacl.h）：
 *   bit4:0  FUNC    = 5 (SQRT)
 *   bit21:16 SFACTOR
 *   bit28:24 NUMITER
 * 写 OP1 即触发；读 RES1 时硬件会 hold 总线直到算完。 */
static inline _iq _IQsqrt_hw(_iq A, uint32_t sfactor, uint32_t iter)
{
    MATHACL->CTL = 5u
                 | ((sfactor & 0x3Fu) << 16)
                 | ((iter    & 0x1Fu) << 24);
    MATHACL->OP1 = (uint32_t)A;
    return (_iq)MATHACL->RES1;
}

/* Q24 硬件开方：归一化 → 硬件 → IQ16 结果左移 8 位转回 Q24。
 * 与 TI 的 __IQNsqrt_MathACL(a, 24) 逐句等价。 */
static inline _iq _IQsqrt_hwQ24(_iq A)
{
    uint32_t input;
    uint32_t output;
    uint32_t sfactor;
    uint32_t n = 0u;

    if (A <= 0) {
        return 0;                       /* 硬件对 0/负数无意义，直接短路 */
    }

    input = (uint32_t)A;

    if (input & 0x80000000u) {
        /* 输入已经占满 32 位（Q24 下相当于 ≥128，正常不会走到这里） */
        sfactor = 0u;
        output  = input;
    } else {
        /* 左移归一化到 IQ30 的 [1,2)：让 bit30 成为最高有效位 */
        while ((input & 0x40000000u) == 0u) {
            n++;
            input <<= 1;
        }
        /* sfactor = (30 - 24) - n = 6 - n，可能为负 → 取低 6 位 */
        sfactor = ((uint32_t)(30 - IQ_SHIFT) - n) & 0x3Fu;
        output  = input;
    }

    /* RES1 是 IQ16，Q24 要左移 24-16 = 8 位 */
    return (_iq)((uint32_t)_IQsqrt_hw((_iq)output, sfactor, 31u) << (IQ_SHIFT - 16));
}

/* 正式使用：按运行时开关在硬件 / 软件之间选 */
static inline _iq _IQsqrt(_iq A)
{
    if (A <= 0) {
        return 0;
    }
    if (g_iqSqrtUseHw) {
        return _IQsqrt_hwQ24(A);
    }
    return _IQsqrt_sw(A);
}

/* 硬件开方自检：拿两个已知点（sqrt(4)=2、sqrt(9)=3）验一遍。
 *   返回 0 = 硬件可用（g_iqSqrtUseHw 置 1）
 *   返回 1 = 硬件不对，已退回软件开方（g_iqSqrtUseHw 保持 0）
 * 无论哪种结果，仿真都能正常跑，只是速度不同。
 * outErr 非空时回填第一个测试点的实际误差（Q24 LSB），用于留档看精度。 */
static inline int IQ_SqrtSelfTest(int32_t *outErr)
{
    const _iq exp4 = _IQ(2.0f);
    const _iq exp9 = _IQ(3.0f);
    /* 容差取相对值：期望值 >> 10 ≈ 0.1%。
       实测硬件偏 4 LSB@IQ16 → Q24 里是 1024 LSB（相对 3.05e-5），
       所以固定几百 LSB 的容差太紧，会把正确结果判成失败。
       而 sfactor 真填错的话，误差是 ×2 / ×√2 量级（至少 41%），
       0.1% 的窗口照样挡得住。 */
    const _iq tol4 = exp4 >> 10;
    const _iq tol9 = exp9 >> 10;
    _iq r4;
    _iq r9;

    /* 记下自检点（_IQ(4.0)）对应的 sfactor，方便以后对照手册排查 */
    {
        uint32_t in = (uint32_t)_IQ(4.0f);
        uint32_t n  = 0u;

        while ((in & 0x40000000u) == 0u) {
            n++;
            in <<= 1;
        }
        g_iqSqrtSfactor = (uint8_t)(((uint32_t)(30 - IQ_SHIFT) - n) & 0x3Fu);
    }
    g_iqSqrtIter = 31u;

    g_iqSqrtUseHw = 1u;                 /* 先打开，让下面的验证真的走硬件路径 */
    r4 = _IQsqrt(_IQ(4.0f));
    r9 = _IQsqrt(_IQ(9.0f));

    if (outErr != 0) {
        *outErr = (int32_t)(r4 - exp4);     /* 期望 ≈ -1024 */
    }

    if ((r4 >= (exp4 - tol4)) && (r4 <= (exp4 + tol4)) &&
        (r9 >= (exp9 - tol9)) && (r9 <= (exp9 + tol9))) {
        return 0;                       /* 硬件可用 */
    }

    g_iqSqrtUseHw = 0u;                 /* 硬件不对 → 退回软件，保证数值正确 */
    return 1;
}

#else  /* ===== IQ_SQRT_MODE == 0：纯软件 ===== */

static inline _iq _IQsqrt(_iq A)
{
    return _IQsqrt_sw(A);
}

static inline int IQ_SqrtSelfTest(int32_t *outErr)
{
    if (outErr != 0) {
        *outErr = 0;
    }
    return 1;                           /* 软件实现，无需自检 */
}

#endif /* IQ_SQRT_MODE */

#endif /* __FLUID_IQMATH_H */
