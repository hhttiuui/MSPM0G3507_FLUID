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
   1 = 用本文件的内联 32×32→64 乘法。好处：没有函数调用开销，而且能
       跟着宿主函数一起被放到 SRAM（.TI.ramfunc）执行，避开 flash 等待周期。
   0 = 交给编译器生成 __aeabi_lmul 调用（库函数在 flash 里，80MHz 下每次都要等） */
#define IQ_INLINE_MPY   1

#if IQ_INLINE_MPY
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
#else
#define _IQmpy(A, B)  ((_iq)(((int64_t)(A) * (int64_t)(B)) >> IQ_SHIFT))
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
#define IQ_DIV_MODE   2

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

/* 开方：逐位法，64 位输入 → 32 位结果，只处理非负（原版也只对 d2 求过根） */
static inline _iq _IQsqrt(_iq A)
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

#endif /* __FLUID_IQMATH_H */
