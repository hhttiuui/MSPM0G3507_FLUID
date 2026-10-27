/*
 * brake.h —— 刹车高亮：检测 + 全屏满亮 + 可替换的刹车显示
 *
 * ==================== 为什么需要它 ====================
 *
 * 自行车尾灯的国标逻辑很简单：**刹车时必须明显变亮**。但我们的板子上
 * 没有任何"刹车线"信号可取 —— 没有和刹车把手相连的线，也没有霍尔。
 *
 * 所以只能从加速度计上"读"刹车：刹车 = 沿行进方向**持续的减速度**。
 * 这个判定比想象中好做，因为骑行时的加速度只有三种来源：
 *   ① 蹬踏加速（0.1~0.2 g，短促、周期性）
 *   ② 刹车减速（0.2~0.5 g，持续 0.5~2 s）← 只有它是"大且持续"的
 *   ③ 路面颠簸 / 摇车（高频抖动）
 * 所以：**低通 90 ms 去掉 ③，阈值 + 持续时间去掉 ①**。
 *
 * ==================== 判定流程 ====================
 *
 *   raw（屏幕坐标，未滤波）→ 选轴/符号 → 一阶低通 τ≈90ms
 *        ↓
 *   |v| ≥ thr 持续 ON_MS(180ms)     → 进入刹车态
 *   |v| ≤ rel 持续 OFF_MS(350ms)    → 退出（且已亮够 MIN_ON_MS）
 *
 * 三个设计要点，缺一个手感就不对：
 *   · **滞回**：进入用 thr(150mg)、退出用 rel(60mg)。同一个阈值会在边界上
 *     反复开关，看起来像接触不良。
 *   · **最短点亮时间**：点刹只有 0.3 s，不设下限会"闪一下就没"。
 *   · **退出要慢**：刹车结束后尾灯保持高亮 350 ms，后车才有反应时间。
 *
 * ==================== 刹车时怎么显示 ====================
 *
 * 默认 = **全部 LED 满亮 + 全局亮度拉满**（就是用户要的效果）。
 * 但这里做成可替换的，因为"刹车该显示什么"是最容易被改的东西：
 *   · BRAKE_DISP_SOLID  全屏满亮（默认）
 *   · BRAKE_DISP_BAR    下半屏满亮（预留示例，证明接口可用）
 *   · Brake_SetDisplayHook(fn)  外部注册任意画法（预留，见下）
 *
 * 显示函数只负责**画进帧缓冲**，不要自己 Flush —— 统一由本模块推。
 *
 * ==================== 为什么必须"接管显示" ====================
 *
 * 场景（流体 / 文字 / 动图）每帧都会重画 DDRAM，刹车画面会被直接盖掉。
 * 所以刹车态下的显示权在**场景之后**：app.c 的 App_Tick 顺序是
 *
 *     场景 tick（画它自己的）→ Brake_Tick（若在刹车，盖成全屏满亮）
 *
 * 代价是刹车期间每帧多一次 256 B 推送（约 1.2 ms），可以接受。
 *
 * 退出时要把画面**原样还回去** —— 文字场景定格（tstep 0）、动图场景、
 * 静态图场景都不重绘，不还的话屏幕会一直停在白屏上。做法是进刹车时
 * 快照 256 B 帧缓冲（LedMatrix_Snapshot），退出时写回 + Flush。
 * 场景在刹车期间被切走的话，App_Goto 会调 Brake_NotifySceneChanged()
 * 重新快照，避免还原出"上一个场景的画面"。
 *
 * ==================== 标定（换板 / 换安装方向必做）====================
 *
 * 1. 装到车上，正着放，静止时看 `dbg.brake.val_mg` 应该在 ±30 mg 内漂。
 *    偏得厉害说明轴选错了：`brkax 0`（屏幕 x）/ `brkax 1`（屏幕 y）。
 * 2. 推车加速 → 记下 val_mg 的符号；捏刹车减速 → 应该是**相反符号**。
 *    刹车那一下的符号如果不对，发 `brksgn 1` / `brksgn -1` 翻过来。
 * 3. 阈值：`brkthr 150` 起步。太灵敏（平路自己亮）就加，太钝就减。
 * 4. 验证：`brkf 1` 强制进入刹车态（拍摄 / 演示用），`brkf 0` 退出。
 *
 * ⚠ 屏幕坐标 ≠ 物理方向：ax_raw/ay_raw 已经过 app.c 的轴向映射，
 *   和流体用的是同一套。所以换 IMU 贴装方向后，这里也要重新标。
 *
 * ==================== 与低电保护的冲突 ====================
 *
 * battery.c 每 2 s 会熄屏 1 ms 采样电压，采完把亮度恢复成**用户设定值**。
 * 如果刹车高亮只设一次 BC，就会被打回正常亮度。所以刹车期间**每帧重设**
 * BC = 255（一次命令写约 15 µs，忽略不计）。
 */
#ifndef __BRAKE_H
#define __BRAKE_H

#include <stdint.h>
#include "app.h"

/* ---- 刹车时显示什么 ---- */
#define BRAKE_DISP_SOLID    0u      /* 全屏满亮（默认，也是用户要的效果）*/
#define BRAKE_DISP_BAR      1u      /* 下半屏满亮（预留示例）*/
#define BRAKE_DISP_HOOK     0xFFu   /* 用 Brake_SetDisplayHook() 注册的画法 */

/* 刹车画面绘制函数：**只画进帧缓冲，不要 Flush**（本模块统一推屏）。
 * 用法：
 *     static void my_brake_draw(void) {
 *         LedMatrix_Clear();
 *         ... 画点什么 ...
 *     }
 *     Brake_SetDisplayHook(my_brake_draw);
 *     Brake_SetDisplay(BRAKE_DISP_HOOK);
 */
typedef void (*brake_draw_fn)(void);

/* 初始化（清零状态、同步 dbg）。在 App_Init 里调，LedMatrix_Init 之后。 */
void Brake_Init(void);

/* 每帧一次，**必须在当前场景 tick 之后**调（否则会被场景盖掉）。
 * 内部按 15 ms 节流，静止场景下不会把主循环拖满。 */
void Brake_Tick(const app_in_t *in);

/* 场景被切换时调用（App_Goto 末尾）。刹车态下重新快照，避免还原错画面。 */
void Brake_NotifySceneChanged(void);

uint8_t Brake_IsActive(void);

/* ---- 运行期配置（串口命令改这些）---- */
void    Brake_SetEnabled(uint8_t on);       /* brk 0/1    检测开关 */
void    Brake_SetForced(uint8_t on);        /* brkf 0/1   强制进入刹车态 */
uint8_t Brake_SetThreshold(uint16_t mg);    /* brkthr     触发阈值，30~800 */
void    Brake_SetAxis(uint8_t axis);        /* brkax 0/1  0=屏幕 x，1=屏幕 y */
void    Brake_SetSign(int8_t sign);         /* brksgn ±1  方向 */
uint8_t Brake_SetDisplay(uint8_t id);       /* brkdisp    0 全亮 / 1 下半条 */
void    Brake_SetDisplayHook(brake_draw_fn fn);

#endif /* __BRAKE_H */
