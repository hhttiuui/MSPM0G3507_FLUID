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
 *   2 文字滚动 scene_text.c    12×12 点阵字库 → 条带 → 横向滚动
 *   3 单帧画面 scene_frame.c   上位机直显：240 B 像素 → 点阵
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

/* ---------------- 2 文字滚动 ----------------
 *
 * 脱机显示中文：用 hardware/wqy12_common.h 的 12×12 点阵字库渲染成 1bpp 条带
 * （放 s_scratch.text_px），每帧截 15 列宽的窗口刷屏。
 *
 * 命令：text <字符串> / tstep <1-8> / tms <10-500> / tlev <0-255>
 * 文字和参数存在本场景的静态区，**切走再切回来还在**；只有条带在 union 里，
 * 切回来会重画。
 */
void    TextScene_Enter(void);
void    TextScene_Tick(const app_in_t *in);
void    TextScene_Exit(void);
uint8_t TextScene_Cmd(uint16_t id, const int32_t *arg);

/* 用上位机发来的字形拼一条条带（覆盖字库渲染结果），随后自动滚动 + 跟随重力。
 * 数据格式：count × { w(1), h(1), bitmap[ceil(w/8)×h] }，MSB = 最左。
 * 返回 1 = 成功，0 = 数据不合法。 */
uint8_t TextScene_ComposeGlyphs(const uint8_t *p, uint16_t len,
                                uint8_t count, int8_t spacing);

/* 上位机条带（整条长图）：bits = 16 行 × ceil(cols/8) 字节，行优先，MSB = 最左。
 * 灌进条带缓冲后交给本场景自己滚（复用 s_step/s_ms 和重力朝向）。
 * 返回 1 = 成功。 */
uint8_t TextScene_LoadStrip(const uint8_t *bits, uint16_t cols);

/* 设置滚动参数（存 Flash 的消息上电恢复时用）。
 * 超出范围的参数会被忽略（保持当前值）。 */
void    TextScene_SetScroll(uint8_t step, uint16_t ms, uint8_t lev);

/* ---------------- 3 单帧画面 ----------------
 *
 * 上位机直显：240 字节（行优先 idx = row×15 + col，值 = 亮度 0~255）。
 * 收到新帧后标 dirty，下一次 tick 重画并推屏。
 *
 * 用法：FrameScene_SetPixels(px) → App_Goto(3)。也可以直接用
 *       App_ShowDirectFrame(px)（它把这两步包在一起）。 */
void    FrameScene_Enter(void);
void    FrameScene_Tick(const app_in_t *in);
void    FrameScene_Exit(void);
void    FrameScene_SetPixels(const uint8_t *px);

#endif /* __SCENES_H */
