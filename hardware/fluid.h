/*
 * fluid.h —— FLIP/PIC 流体求解器（从 CH32 版 SandSim.c 移植）
 *
 * 算法来自经典的 FLIP/PIC 流体教学实现（Ten Minute Physics 那一套），
 * 每个时间步的流程：
 *     粒子受力积分 → 粒子互斥 → 粒子→网格 → 密度统计
 *     → 网格压力求解(迭代) → 网格→粒子
 *
 * 本文件只有 <string.h> 和定点头文件两个依赖，**不含任何外设/显示依赖**，
 * 所以整个 fluid.c 可以原样丢到 PC 上编译调试。
 */
#ifndef __FLUID_H
#define __FLUID_H

#include <stdint.h>
#include "fluid_iqmath.h"

/* ===== 与原版一致的参数：不要随意改，改了要重新标定手感 ===== */
#define NumberOfParticles      130U
#define ParticleRadius         _IQ(0.017f)
#define Spacing                _IQ(0.05f)
#define CellNumX               17U
#define CellNumY               18U
#define CellCount              (CellNumX * CellNumY)
#define FLUID_DT               _IQ(0.016f)
#define BOUNCYNESS             _IQ(-0.9f)
#define overRelaxiation        _IQ(1.9f)
#define stiffnessCoefficient   _IQ(1.0f)

/* 单元格类型（显示层据此判断亮/灭，务必保持一致） */
#define FLUID_CELL  0U
#define AIR_CELL    1U
#define SOLID_CELL  2U

/* 网格索引；内部可视区域是 x∈[1, CellNumX-2]，y∈[1, CellNumY-2] */
#define CELL_INDEX(x, y)   ((x) * CellNumY + (y))

/* 初始化粒子与网格（上电调用一次） */
void Fluid_Init(void);

/* 推进一个时间步。xAcceleration/yAcceleration 是"模拟单位"下的加速度，
   正 y 方向对应屏幕下方 */
void Fluid_Step(_iq xAcceleration, _iq yAcceleration);

/* 单独调用各个阶段（Fluid_Step 内部也是这个顺序）。
   上层可以拿它们做分阶段耗时统计。 */
void ParticleIntegrate(_iq xAcceleration, _iq yAcceleration);   /* 1 粒子积分+撞墙 */
void PushParticlesApart(unsigned int nIters);                   /* 2 粒子互斥 */
void particles_to_grid(void);                                   /* 3 粒子→网格 */
void density_update(void);                                      /* 4 密度统计 */
void compute_grid_forces(unsigned int nIters);                  /* 5 压力求解 */
void grid_to_particles(void);                                   /* 6 网格→粒子 */

/* 取内部 cellType[] 数组首地址，长度 CellCount（显示层只读使用） */
const uint8_t *Fluid_GetCellType(void);

/* 已推进的步数 */
uint32_t Fluid_GetFrameCount(void);

/* 运行期设置时间步长（秒，Q24）。默认 FLUID_DT = 0.016。
   调大 = 液体运动更快、更接近实时；太大（>0.05 量级）可能发散。 */
void Fluid_SetDt(_iq dt);

/* 运行期设置压力求解迭代次数（默认 12）。
   设为 0 表示跳过整个压力求解，只用于测"固定开销"。
   注意：迭代次数越低液体越"软"，减到 4 以下会明显不像液体。 */
void Fluid_SetGridIter(uint32_t nIters);

#endif /* __FLUID_H */
