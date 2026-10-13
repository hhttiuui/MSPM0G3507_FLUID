/*
 * imu_selftest.h —— LSM6DS3TR-C 陀螺仪读取自检（无屏版本）
 *
 * 本工程已砍掉 OLED 显示（显示部分后续只用 LED），所以自检**不画任何东西**：
 * 只跑"初始化 → 连续读六轴 → 定期发布窗口统计 + 翻一次心跳 LED"这一个死循环，
 * 参数全部落在 debug.c 的 dbg 里，用 CCS 的 Expressions 展开 dbg 看。
 *
 * 上电后看这几项就够判断陀螺仪通没通（都在 dbg 下）：
 *   dbg.who.init_res   0 = 芯片识别 + 寄存器读写都正常（1 = ID 不对，2 = 读不回来）
 *   dbg.who.id         实测 WHO_AM_I（LSM6DS3TR-C 应为 0x6A）
 *   dbg.spi.rx         不恒为 0x00 / 0xFF → MISO 确实被器件驱动
 *   dbg.raw.cnt        在涨 → 采样循环在跑
 *   dbg.raw.* / phy.*  平放时某个轴约 ±1000 mg；转动板子时陀螺 mdps 正负跟着变
 *   dbg.perf.read_us   单次 14 字节突发读的耗时（约十几 us）
 *   dbg.fault.cnt      应为 0（>0 说明进过 HardFault）
 *
 * ⚠ 须知：自检期间流体模拟不再运行（fluid_demo.c 只是没被调用，代码仍在工程里）。
 *   验证完把下面 IMU_SELFTEST_ENABLE 改回 0，main() 就恢复跑流体。
 */
#ifndef __IMU_SELFTEST_H
#define __IMU_SELFTEST_H

/* 1 = 使能自检（进入后不返回）; 0 = 正常跑流体模拟 */
#define IMU_SELFTEST_ENABLE     0

/* 进入自检死循环，永不返回 */
void ImuSelfTest_Run(void);

#endif /* __IMU_SELFTEST_H */
