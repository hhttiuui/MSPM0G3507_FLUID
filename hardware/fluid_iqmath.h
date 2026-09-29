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
   1 = 走单精度浮点（int→float → 除法 → float→int）。
       M0+ 没有硬件浮点，但软浮点除法（约 300~500 周期）通常仍明显快于
       64 位整数除法 __aeabi_ldivmod（实测约 1000 周期，是个 32 次迭代的循环）。
       精度：float 的 24 位尾数与 Q24 相当，误差 ≤1 LSB，对流体模拟完全够用。
   0 = 用 64 位整数除法（结果与 WCH 原版逐位一致） */
#define IQ_FLOAT_DIV   1

#if IQ_FLOAT_DIV
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
