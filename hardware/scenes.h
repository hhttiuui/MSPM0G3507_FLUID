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

/* ---------------- 文字预设（存 Flash，PC 端是主、板子是副本）----------------
 *
 * 上位机流程：`tpclr` 清空 → 对每一组文字调 text/tstep/tms/tlev/torient →
 * 各发一条 `tpadd`。板子只做顺序追加，不擦单条（flash 只能整扇区擦）。
 *
 * 与两级按键的关系：文字大类（ROUTE_CAT_TEXT）的项数 = StoreCount()；
 * 某项被选中时调用 StoreApply(item) 把文字和参数一起套上去。
 */
uint8_t TextScene_StoreCount(void);          /* 当前存了几条（0 = 空）*/
uint8_t TextScene_StoreClear(void);          /* 擦空整块并写头，返回 1 = 成功 */
uint8_t TextScene_StoreAdd(void);            /* 追加当前文字+参数，返回 1 = 成功 */
uint8_t TextScene_StoreApply(uint8_t i);     /* 应用第 i 条，返回 1 = 成功 */

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

/* ---------------- 图片/动图槽（阶段 3，存 Flash）----------------
 *
 * 槽 0 = FLASH_REG_ANIM0（16 KB），槽 1 = FLASH_REG_ANIM1（8 KB）；
 * 每个槽头部独占 1 KB，数据紧跟其后。位深 8bit（240 B/帧）或 4bit（120 B/帧）。
 * 上传走 vcmd 的 ver 5/6/7 三包：BEGIN → DATA… → END（头部最后写）。
 */
uint8_t FrameScene_SlotCount(void);          /* 有几个有效槽 */
uint8_t FrameScene_PlaySlot(uint8_t idx);    /* 播第 idx 个**有效槽**（0 基）*/

uint8_t FrameScene_StoreBegin(uint8_t slot, uint8_t bpp, uint16_t frames,
                              uint16_t fpsX10, uint8_t gain);
uint8_t FrameScene_StoreData(const uint8_t *p, uint8_t len);   /* len 必须是 8 的倍数 */
uint8_t FrameScene_StoreEnd(uint16_t crc);

/* ---------------- 4 几何动画（阶段 4）----------------
 *
 * 一张 240 B 灰度缓冲 + 绘制原语 + 每帧衰减（拖尾），动画由
 * scene_still.c 末尾的 s_anims[] 表驱动：
 *   0 弹跳球 / 1 旋转立方体 / 2 扩散环
 * 纯代码、不碰 Flash；实现放在 scene_still.c（本工程不便新增 .c，见该文件注释）。
 *
 * **加动画只改 scene_still.c**：写 frame/reset 函数 + 表里加一行即可，
 * 本头文件和 app.c 都不用动（项数走 GeomScene_AnimCount()）。
 */
void    GeomScene_Enter(void);
void    GeomScene_Tick(const app_in_t *in);
void    GeomScene_Exit(void);
uint8_t GeomScene_Cmd(uint16_t id, const int32_t *arg);

/* 直接选第 n 个动画（路由切换用），n = 0 .. AnimCount()-1 */
void    GeomScene_Select(uint8_t n);

/* 动画总数 = s_anims[] 表长。app.c 的 route_item_count() 直接取这个值。 */
uint8_t GeomScene_AnimCount(void);

#endif /* __SCENES_H */
