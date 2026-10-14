/*
 * lsm6ds3trc.c —— LSM6DS3TR-C 六轴 IMU 的 SPI 读取实现
 *
 * 分层：本文件只做"寄存器读写 + 原始值换算"，把观测量写进 dbg；
 *       采样节奏、屏刷新、错误提示都在 hardware/imu_selftest.c 里。
 *
 * SPI 事务由 CS/SCLK 上的一问一答完成（SPI_LSM6DS3TRC 已在 SYSCFG_DL_init
 * 里初始化好，这里不再碰 SPI 配置，避免和 syscfg 打架）。
 */

#include "lsm6ds3trc.h"
#include "debug.h"
#include "clock.h"

/* 片选：syscfg 里是普通 GPIO（因为 SPI1 用的是 Motorola 3 线格式，
 * 没有硬件 CS 信号），所以要自己拉低/拉高。 */
#define CS_PORT     GPIO_LSM6DS3TRC_PORT
#define CS_PIN      GPIO_LSM6DS3TRC_PIN_LSM6DS3TRC_CS_PIN

/* 超时保护：一次 8bit 移位只要几十个循环，给个宽松上限，
 * 防止 SPI 意外卡住时把整个主循环死等在这里。 */
#define SPI_WAIT_LIMIT      (100000u)

/* 复位自清等待的上限（ms） */
#define RESET_WAIT_LIMIT_MS (100u)

/* ==================== 最底层：收发一个字节 ====================
 * 收完立刻读 RXFIFO，再等 BUSY 清零，保证下一次 CS 拉低前总线真的空闲。
 * dbg.spi.tx/rx/xfer/timeout 在这里统一维护（所有寄存器访问都会经过这里）。 */
static uint8_t SpiTransferByte(uint8_t data)
{
    uint32_t guard;
    uint8_t  rd;

    DL_SPI_transmitData8(SPI_LSM6DS3TRC_INST, data);

    guard = SPI_WAIT_LIMIT;
    while (DL_SPI_isRXFIFOEmpty(SPI_LSM6DS3TRC_INST) && (--guard != 0u)) {
        /* Wait for received byte */
    }
    if (guard == 0u) {
        dbg.spi.timeout++;      /* 收到东西之前就超时了：MISO 没被驱动 */
    }

    rd = DL_SPI_receiveData8(SPI_LSM6DS3TRC_INST);

    guard = SPI_WAIT_LIMIT;
    while (DL_SPI_isBusy(SPI_LSM6DS3TRC_INST) && (--guard != 0u)) {
        /* Wait for SPI transfer completion */
    }
    if (guard == 0u) {
        dbg.spi.timeout++;
    }

    dbg.spi.xfer++;
    dbg.spi.tx = data;
    dbg.spi.rx = rd;

    return rd;
}

/* ==================== 寄存器访问 ==================== */

uint8_t LSM6DS3TRC_ReadReg(uint8_t reg)
{
    uint8_t v;

    dbg.spi.last_reg = reg;
    dbg.spi.rw       = 1u;

    DL_GPIO_clearPins(CS_PORT, CS_PIN);
    dbg.spi.cmd = SpiTransferByte((uint8_t)(reg | 0x80u));   /* bit7 = 1 → 读 */
    v = SpiTransferByte(0xFFu);
    DL_GPIO_setPins(CS_PORT, CS_PIN);

    return v;
}

void LSM6DS3TRC_WriteReg(uint8_t reg, uint8_t val)
{
    dbg.spi.last_reg = reg;
    dbg.spi.rw       = 0u;

    DL_GPIO_clearPins(CS_PORT, CS_PIN);
    dbg.spi.cmd = SpiTransferByte(reg);                      /* bit7 = 0 → 写 */
    SpiTransferByte(val);
    DL_GPIO_setPins(CS_PORT, CS_PIN);
}

void LSM6DS3TRC_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    dbg.spi.last_reg = reg;
    dbg.spi.rw       = 1u;

    DL_GPIO_clearPins(CS_PORT, CS_PIN);
    dbg.spi.cmd = SpiTransferByte((uint8_t)(reg | 0x80u));

    /* 整个突发期间 CS 一直保持低电平，器件按 IF_INC 自动递增地址 */
    while (len > 0u) {
        *buf = SpiTransferByte(0xFFu);
        buf++;
        len--;
    }

    DL_GPIO_setPins(CS_PORT, CS_PIN);
}

/* ==================== 初始化 ==================== */

uint8_t LSM6DS3TRC_Init(void)
{
    uint32_t t0;
    uint32_t guard;
    uint8_t  cfg_xl;
    uint8_t  chk;

    cfg_xl = (uint8_t)((LSM6DS3TRC_ODR_XL << 4) | (LSM6DS3TRC_FS_XL << 2));

    dbg.who.want = LSM6DS3TRC_ID;
    t0 = tick_ms;

    /* 上电后器件需要一点时间才能响应（datasheet: 10ms boot time） */
    mspm0_delay_ms(10);

    /* 1) 认芯片：把实测 ID 记下来，不匹配也不 return —— 留证据比"静默失败"有用 */
    dbg.who.id    = LSM6DS3TRC_ReadReg(LSM6DS3TRC_WHO_AM_I);
    dbg.who.match = (dbg.who.id == LSM6DS3TRC_ID) ? 1u : 0u;

    /* 2) 软复位：回确定的默认状态（CTRL3_C.bit0 自清） */
    LSM6DS3TRC_WriteReg(LSM6DS3TRC_CTRL3_C, 0x01u);
    guard = RESET_WAIT_LIMIT_MS;
    while (((LSM6DS3TRC_ReadReg(LSM6DS3TRC_CTRL3_C) & 0x01u) != 0u) &&
           (--guard != 0u)) {
        mspm0_delay_ms(1);
    }

    /* 3) BDU(bit6)=1：输出寄存器不会在读取过程中被刷新，六轴数据不会"半新半旧"
     *    IF_INC(bit2)=1：多字节连续读时地址自动递增（0x20 起一次读 14 字节就靠它） */
    LSM6DS3TRC_WriteReg(LSM6DS3TRC_CTRL3_C, 0x44u);

    /* 4) 采样率 + 量程 */
    LSM6DS3TRC_WriteReg(LSM6DS3TRC_CTRL1_XL, cfg_xl);
    LSM6DS3TRC_WriteReg(LSM6DS3TRC_CTRL2_G,
                        (uint8_t)((LSM6DS3TRC_ODR_G << 4) | LSM6DS3TRC_FS_G));

    /* 5) 回读校验：写进去的值读不回来 = SPI 读通路有问题（MISO 没接好/MISO/POCI 选错） */
    chk = LSM6DS3TRC_ReadReg(LSM6DS3TRC_CTRL1_XL);

    if (dbg.who.match == 0u) {
        dbg.who.init_res = LSM6DS3TRC_ERR_ID;
    } else if (chk != cfg_xl) {
        dbg.who.init_res = LSM6DS3TRC_ERR_READBACK;
    } else {
        dbg.who.init_res = LSM6DS3TRC_OK;
    }

    dbg.who.init_ms = tick_ms - t0;

    return dbg.who.init_res;
}

/* ==================== 读取 ==================== */

/* 小工具：两个字节（L/H）拼成有符号 16 位 */
static int16_t RawLe16(const uint8_t *p)
{
    return (int16_t)(((uint16_t) p[1] << 8) | (uint16_t) p[0]);
}

/* 四舍五入的整数除法（被除数可能是负数，所以不能直接用 +500） */
static int32_t DivRound1000(int32_t v)
{
    if (v < 0) {
        return (v - 500) / 1000;
    }
    return (v + 500) / 1000;
}

void LSM6DS3TRC_ReadAll(void)
{
    uint8_t  buf[14];
    uint16_t t0;

    t0 = DBG_Mark();

    /* 从 OUT_TEMP_L(0x20) 起连续读 14 字节：
     *   0x20/0x21 温度、0x22~0x27 陀螺 X/Y/Z、0x28~0x2D 加速度 X/Y/Z
     * 一次突发读拿到同一时刻的整组数据（配合 BDU=1 不会读到撕裂值），
     * 也不用发 3 次地址，比分开读省一半时间。 */
    LSM6DS3TRC_ReadRegs(LSM6DS3TRC_OUT_TEMP_L, buf, 14u);

    dbg.perf.read_us = DBG_TicksToUs(DBG_Elapsed(t0));
    DBG_PerfAcc(dbg.perf.read_us);

    /* ---- 原始值 ---- */
    dbg.raw.gx = RawLe16(&buf[2]);
    dbg.raw.gy = RawLe16(&buf[4]);
    dbg.raw.gz = RawLe16(&buf[6]);
    dbg.raw.ax = RawLe16(&buf[8]);
    dbg.raw.ay = RawLe16(&buf[10]);
    dbg.raw.az = RawLe16(&buf[12]);
    dbg.raw.cnt++;

    /* ---- 换算成物理量（整数定点，量程见 lsm6ds3trc.h）---- */
    dbg.phy.gx_mdps = (int32_t) dbg.raw.gx * LSM6DS3TRC_GYRO_MDPS_PER_LSB;
    dbg.phy.gy_mdps = (int32_t) dbg.raw.gy * LSM6DS3TRC_GYRO_MDPS_PER_LSB;
    dbg.phy.gz_mdps = (int32_t) dbg.raw.gz * LSM6DS3TRC_GYRO_MDPS_PER_LSB;

    dbg.phy.ax_mg = DivRound1000((int32_t) dbg.raw.ax * LSM6DS3TRC_ACCEL_UG_PER_LSB);
    dbg.phy.ay_mg = DivRound1000((int32_t) dbg.raw.ay * LSM6DS3TRC_ACCEL_UG_PER_LSB);
    dbg.phy.az_mg = DivRound1000((int32_t) dbg.raw.az * LSM6DS3TRC_ACCEL_UG_PER_LSB);

    /* 温度（datasheet Table 5）：256 LSB/°C、0 LSB = 25 °C
     *   m°C = 25000 + raw × 1000 / 256
     * 先乘后除，保留精度；raw 是 16 位有符号，乘 1000 后约 ±3.3e7，int32 装得下。 */
    dbg.phy.temp_mdegc = LSM6DS3TRC_TEMP_OFFSET_MDEGC
                       + ((int32_t) RawLe16(&buf[0]) * LSM6DS3TRC_TEMP_MDEG_NUM)
                         / LSM6DS3TRC_TEMP_MDEG_DEN;
}
