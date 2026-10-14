/*
 * lsm6ds3trc.h —— LSM6DS3TR-C 六轴 IMU 的最简 SPI 驱动（本工程只用到三轴数据）
 *
 * ==================== 器件与接线 ====================
 * LSM6DS3TR-C：ST 六轴 IMU（3 轴加速度 + 3 轴陀螺）。SPI/I2C 双接口，
 * 本工程用系统里已经配好的硬件 SPI1（syscfg 里叫 SPI_LSM6DS3TRC）：
 *   SCLK = PA17   PICO/MOSI = PA18   POCI/MISO = PA16   CS = PA15（GPIO 手动控制）
 *   Mode0(POL0/PHA0)、8bit、MSB first、8MHz（= 80MHz / ((1+4)*2)）
 *
 * ⚠ 别和已有的 hardware/IMU660RB/ 搞混：那套驱动是针对 LSM6DSR 的
 *   （WHO_AM_I(0x0F) = 0x6B），本芯片是 0x6A。两者基础寄存器地址大体相同，
 *   但**陀螺量程（CTRL2_G 的 FS_G 字段）编码不一样**，混用会按错量程换算。
 *
 * ==================== SPI 时序（datasheet 5.2 节） ====================
 *   写：CS↓ → 发 reg(bit7=0)  → 发 data       → CS↑
 *   读：CS↓ → 发 reg|0x80     → 收 N 字节     → CS↑
 *   注意：地址字节之后**紧跟**数据，没有 dummy byte。
 *   多字节连续读靠 CTRL3_C.IF_INC=1 的地址自增。
 *
 * ==================== 使用 ====================
 *   if (LSM6DS3TRC_Init() == LSM6DS3TRC_OK) { ... }
 *   循环里调 LSM6DS3TRC_ReadAll();   // 六轴 + 温度都写进 dbg
 *
 * 兼容性说明：ID 不匹配时 Init 不会停在外面不干活，而是把实测 ID 写进
 *   dbg.who.id 后**继续按 LSM6DS3TR-C 的寄存器配置**跑一遍，并把返回码
 *   置成 LSM6DS3TRC_ERR_ID。这样即使板子上其实是别的芯片，也能从 dbg 里
 *   一眼看出"读到的 ID 是多少、后面的读取有没有数据"。
 */

#ifndef __LSM6DS3TRC_H
#define __LSM6DS3TRC_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ==================== 器件常量 ==================== */

/* WHO_AM_I(0x0F) 固定回读值。LSM6DS3TR-C 与 LSM6DSL 相同 = 0x6A；
 * 逐飞 IMU660RB 用的 LSM6DSR 是 0x6B，别混。 */
#define LSM6DS3TRC_ID                   (0x6Au)

/* 初始化返回码（也是 dbg.who.init_res 的取值） */
#define LSM6DS3TRC_OK                   (0u)
#define LSM6DS3TRC_ERR_ID               (1u)   /* WHO_AM_I 不等于 LSM6DS3TRC_ID */
#define LSM6DS3TRC_ERR_READBACK         (2u)   /* CTRL 寄存器写进去读不回来 ⇒ 读通路坏 */

/* ==================== 关键寄存器地址 ==================== */
#define LSM6DS3TRC_WHO_AM_I             (0x0Fu)
#define LSM6DS3TRC_CTRL1_XL             (0x10u)   /* 加速度：ODR[7:4] + FS_XL[3:2] */
#define LSM6DS3TRC_CTRL2_G              (0x11u)   /* 陀螺：  ODR[7:4] + FS_G[3:0]  */
#define LSM6DS3TRC_CTRL3_C              (0x12u)   /* BDU / IF_INC / SW_RESET      */
#define LSM6DS3TRC_STATUS_REG           (0x1Eu)   /* bit0 XLDA / bit1 GDA 数据就绪 */
#define LSM6DS3TRC_OUT_TEMP_L           (0x20u)   /* 温度 (2 字节)                 */
#define LSM6DS3TRC_OUTX_L_G             (0x22u)   /* 陀螺 X/Y/Z (6 字节)           */
#define LSM6DS3TRC_OUTX_L_XL            (0x28u)   /* 加速度 X/Y/Z (6 字节)         */

/* ==================== 采样配置 ====================
 * ODR 字段（CTRL1_XL/CTRL2_G 的 bit7:4）：0 = 关断，1..10 依次对应
 *   12.5 / 26 / 52 / 104 / 208 / 416 / 833 / 1660 / 3330 / 6660 Hz
 *   （陀螺最高只到 6660Hz 的那两档里，实际 6660 只有加速度支持）
 * 这里取 416Hz：比屏刷新（~400ms）快得多，读取循环跑满也不会重复采到同一拍。 */
#define LSM6DS3TRC_ODR_XL               (0x6u)    /* 416 Hz */
#define LSM6DS3TRC_ODR_G                (0x6u)    /* 416 Hz */

/* 量程（改动后必须同步改下面的换算系数，否则读出来的物理量全是错的）
 *   FS_XL(CTRL1_XL[3:2])：0 = ±2g，1 = ±16g，2 = ±4g，3 = ±8g
 *   FS_G (CTRL2_G[3:0]) ：0 = ±125dps，2 = ±250，4 = ±500，8 = ±1000，12 = ±2000 */
#define LSM6DS3TRC_FS_XL                (0x0u)    /* ±2 g  */
#define LSM6DS3TRC_FS_G                 (0xCu)    /* ±2000 dps */

/* 灵敏度（datasheet 第 2 章）。整数化后：
 *   ±2g   → 0.061 mg/LSB  ⇒ 用 61 µg/LSB 参与整数运算
 *   ±2000dps → 70 mdps/LSB（正好是整数，不用取整）
 *
 * 温度（datasheet Table 5 "Temperature sensor characteristics"，DocID030071 Rev 3）：
 *   TSen = 256 LSB/°C  ⇒ 1 LSB = 1000/256 = 3.90625 m°C
 *   Toff：0 LSB (typ.) 对应 25 °C
 * ⚠ 早先这里误写成 62.5 m°C/LSB（即 16 LSB/°C），比实际灵敏 16 倍，
 *   导致室温被算成 167 °C。用分数形式换算可避免取整误差。 */
#define LSM6DS3TRC_ACCEL_UG_PER_LSB     (61)      /* µg/LSB，对应 FS_XL = ±2g   */
#define LSM6DS3TRC_GYRO_MDPS_PER_LSB    (70)      /* mdps/LSB，对应 FS_G = ±2000 */
#define LSM6DS3TRC_TEMP_MDEG_NUM        (1000)    /* 温度换算分子：m°C = raw × 1000 / 256 */
#define LSM6DS3TRC_TEMP_MDEG_DEN        (256)     /* 温度换算分母（256 LSB/°C） */
#define LSM6DS3TRC_TEMP_OFFSET_MDEGC    (25000)   /* 0 LSB 对应 25 °C = 25000 m°C */

/* ==================== 接口 ==================== */

/* 初始化：软复位 → BDU/IF_INC → ODR/量程 → 回读校验。
 * 返回取值为 LSM6DS3TRC_OK / _ERR_ID / _ERR_READBACK，
 * 同时把 WHO_AM_I 实测值与返回码写进 dbg.who。 */
uint8_t LSM6DS3TRC_Init(void);

/* 读一次温度 + 六轴（0x20 起 14 字节突发读），
 * 结果写进 dbg.raw（原始 LSB）、dbg.phy（mg / mdps / m°C），
 * 并把这次读取的耗时累加进观测窗口（DBG_PerfAcc）。 */
void LSM6DS3TRC_ReadAll(void);

/* 通用寄存器访问（留给 bring-up 时逐寄存器排查用） */
uint8_t LSM6DS3TRC_ReadReg(uint8_t reg);
void    LSM6DS3TRC_WriteReg(uint8_t reg, uint8_t val);
void    LSM6DS3TRC_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);

#endif /* __LSM6DS3TRC_H */
