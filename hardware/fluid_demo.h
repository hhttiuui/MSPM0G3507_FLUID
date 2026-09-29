/*
 * fluid_demo.h —— 流体模拟 + OLED 显示
 *
 * 实现是 hardware/fluid_demo.c（胶水层）：
 *   ① IMU 加速度计 → 屏幕坐标，驱动 fluid.c 的算法
 *   ② cellType[] → 8 页显存缓冲 → OLED_PushPageRange() 批量推屏
 *   ③ 右侧 HUD：三轴读数 / 送入流体的值 / 帧耗时 / 帧率
 */
#ifndef __FLUID_DEMO_H
#define __FLUID_DEMO_H

/* 进入主循环，永不返回 */
void FluidDemo_Run(void);

#endif /* __FLUID_DEMO_H */
