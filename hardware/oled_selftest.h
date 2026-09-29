/*
 * oled_selftest.h —— OLED 批量刷屏 + 网格映射验证（临时诊断用）
 *
 * 这一步对应移植计划的 P1 + P2：
 *   P1  用新加的 OLED_PushPageRange() 实测"一页一事务"的真实耗时
 *   P2  用与流体完全相同的"格子 → 半字节 → 页缓冲"路径画一个已知图案，
 *       验证 SS D1306 的页/位/列方向是否与我们的映射一致
 *
 * 屏幕上应该看到（左侧 60x64 画布）：
 *   · 一圈 4px 粗的外框
 *   · 一条从左上到右下的 45 度斜线（15 个格子）
 *   · 一个在框内斜向弹跳的 4x4 方块（每 6 帧动一格）
 * 右侧是耗时统计：
 *   PG1= 100 次单页推送总耗时(ms)，期望约 140  (即 1.4ms/页)
 *   FRM= 20 帧完整画布(8页)总耗时(ms)，期望约 260 (即 13ms/帧)
 *   AVG= 每帧平均耗时，单位 0.1ms
 *   FPS= 实测帧率
 *   ERR= I2C 超时次数，应为 0
 *
 * ⚠️ 用完把 OLED_SELFTEST_ENABLE 改回 0（或删掉本文件）。
 */
#ifndef __OLED_SELFTEST_H
#define __OLED_SELFTEST_H

/* 1 = 使能自检（进入后不返回）; 0 = 关闭 */
#define OLED_SELFTEST_ENABLE   0

/* 进入自检死循环，永不返回 */
void OledDrawTest_Run(void);

#endif /* __OLED_SELFTEST_H */
