/*
 * fluid_demo.h —— 场景 0：流体 → LED 点阵
 *
 * 实现是 hardware/fluid_demo.c：
 *   ① 加速度（由 app.c 的输入层给）驱动 fluid.c 的算法
 *   ② particleDensity[] → 亮度 → LedMatrix（映射层）→ 推屏
 *   ③ 观测量全在 dbg.fluid（没有屏，参数只能这么看）
 * 算法本身在 hardware/fluid.c，纯计算、零外设依赖。
 *
 * 场景的 enter/tick/exit/cmd 声明在 scenes.h —— 那个才是对外接口。
 * 这里只剩一个启动自检，因为它是"数学层是否可信"的检查，要在进场景之前跑。
 */
#ifndef __FLUID_DEMO_H
#define __FLUID_DEMO_H

/* 数学层自检（_IQmpy/_IQdiv/_IQsqrt + MATHACL 硬件乘法）。
 * 由 app.c 在初始化时调用；结果同时写进 dbg.fluid.iq_ok。
 * 返回 1 = 通过，0 = 失败（调用方应该整屏点亮停住）。 */
uint8_t FluidDemo_IqSelfTest(void);

#endif /* __FLUID_DEMO_H */
