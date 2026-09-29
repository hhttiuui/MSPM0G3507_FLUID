/*
 * clock_selftest.h —— CPUCLK 80MHz 实测自检（临时诊断用，验证完可整体删除）
 *
 * 用途：不需要示波器、不需要串口终端，用芯片内置的 FCC（频率时钟计数器）
 *       以 LFCLK(32.768kHz) 为基准实测 MCLK / SYSPLLCLK2X / SYSOSC，
 *       结果直接画在 OLED 上；同时让 PA7(LED) 以 1Hz 翻转做交叉验证。
 *
 * 使用方法：
 *   1. 把下面的 CLOCK_SELFTEST_ENABLE 改成 1
 *   2. 编译下载运行
 *   3. main() 会在 OLED 初始化之后进入自检死循环，不再进入菜单
 *   4. 验证完成后改回 0（或直接删掉 clock_selftest.c/.h 并从
 *      Debug/hardware/subdir_vars.mk 里移除对应条目）
 *
 * ⚠️ 须知：
 *   - 自检期间电机/舵机/编码器已经初始化但保持停止；建议把小车架空
 *     或断开电机、舵机电源后再跑。
 *   - 精度受内部 LFOSC 限制（约 ±百分之几），本自检能明确区分
 *     80 / 40 / 32 MHz 这类量级差别，但不代表绝对精度；
 *     要绝对精度需外接 LFXT 32.768kHz 晶振。
 *   - 如果 SYSPLL 根本没锁上，程序会在启动阶段
 *     （SYSCFG_DL_SYSCTL_SYSPLL_init 的看门狗循环）死掉，
 *     OLED 什么都不显示 —— 这本身就是一种诊断信号。
 */
#ifndef __CLOCK_SELFTEST_H
#define __CLOCK_SELFTEST_H

/* 1 = 使能自检（进入后不返回）; 0 = 正常跑小车程序 */
#define CLOCK_SELFTEST_ENABLE   0

/* 进入自检死循环，永不返回 */
void ClockSelfTest_Run(void);

#endif /* __CLOCK_SELFTEST_H */
