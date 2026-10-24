/*
 * scenes.h —— 所有场景的入口声明
 *
 * 一个场景 = enter / tick / exit / cmd 四个函数，实现在各自的 .c 里。
 * app.c 的 s_scenes[] 按这里的顺序建表，**顺序即场景号**，改顺序等于改串口
 * `mode <n>` 的编号，别随便调。
 *
 * 现有场景：
 *   0 流体     fluid_demo.c    算法 → 密度 → 灰度 → 点阵（IMU 重力驱动）
 *   1 静态图   scene_still.c   亮度阶梯 / 单点 / 填充，亮度标定用
 */
#ifndef __SCENES_H
#define __SCENES_H

#include "app.h"

/* ---------------- 0 流体 ---------------- */
void    FluidScene_Enter(void);
void    FluidScene_Tick(const app_in_t *in);
void    FluidScene_Exit(void);
uint8_t FluidScene_Cmd(uint16_t id, const int32_t *arg);

/* ---------------- 1 静态图 ----------------
 *
 * 进入时默认画 14 级线性亮度阶梯（否则是一片黑，看不出切没切成功）。
 * 命令：px / fill / off / ramp / rampg
 * 这个场景 tick 什么都不做 —— DDRAM 是持久的，画完就一直显示。
 */
void    StillScene_Enter(void);
void    StillScene_Tick(const app_in_t *in);
void    StillScene_Exit(void);
uint8_t StillScene_Cmd(uint16_t id, const int32_t *arg);

#endif /* __SCENES_H */
