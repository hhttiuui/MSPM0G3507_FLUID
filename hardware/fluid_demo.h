/*
 * fluid_demo.h —— 流体模拟 → LED 点阵（暂不接 IMU）
 *
 * 实现是 hardware/fluid_demo.c（胶水层）：
 *   ① 假重力（方向每 2s 换一次）驱动 fluid.c 的算法
 *   ② cellType[] → LedMatrix（映射层）→ 推屏
 *   ③ 观测量全在 dbg.fluid（没有屏，参数只能这么看）
 * 算法本身在 hardware/fluid.c，纯计算、零外设依赖。
 */
#ifndef __FLUID_DEMO_H
#define __FLUID_DEMO_H

/* 进入主循环，永不返回 */
void FluidDemo_Run(void);

#endif /* __FLUID_DEMO_H */
