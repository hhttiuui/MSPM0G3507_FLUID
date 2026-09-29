#include "imu660rb.h"
#include "lsm6dsr_reg.h"

#include "ti_msp_dl_config.h"
#include "clock.h"

#define BOOT_TIME        (10)
#define OFFSET_CAL_TIME  (50)

#define ODR_COEFF_12Hz5  (512)
#define ODR_COEFF_26Hz   (256)
#define ODR_COEFF_52Hz   (128)
#define ODR_COEFF_104Hz  (64)
#define ODR_COEFF_208Hz  (32)
#define ODR_COEFF_416Hz  (16)
#define ODR_COEFF_833Hz  (8)
#define ODR_COEFF_1667Hz (4)
#define ODR_COEFF_3333Hz (2)
#define ODR_COEFF_6667Hz (1)

static stmdev_ctx_t dev_ctx;

float acceleration_mg[3];
float angular_rate_mdps[3];

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static uint8_t whoamI;
static uint8_t rst;
static float samplePeriod;
static float sampleRate;

/*
 * FusionMatrix 内部包含二维数组。
 * 使用 .array 指定成员并补齐二维花括号，避免 missing-braces 警告。
 */
static const FusionMatrix gyroscopeMisalignment = {
    .array = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    }
};

/*
 * FusionVector 内部包含一维数组。
 * 使用 .array 显式初始化。
 */
static const FusionVector gyroscopeSensitivity = {
    .array = {1.0f, 1.0f, 1.0f}
};

static FusionVector gyroscopeOffset = {
    .array = {0.0f, 0.0f, 0.0f}
};

FusionAhrs ahrs;
FusionEuler euler;
static FusionOffset offset;

static int32_t platform_write(
    void *handle,
    uint8_t reg,
    const uint8_t *bufp,
    uint16_t len);

static int32_t platform_read(
    void *handle,
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len);

static void platform_delay(uint32_t ms);

void IMU660RB_Init(void)
{
    lsm6dsr_pin_int1_route_t int1_route;
    uint8_t offset_cnt;
    int8_t freq_fine;

    /* Initialize MEMS driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;

    /* Wait for sensor boot */
    platform_delay(BOOT_TIME);

    /* Check device ID */
    lsm6dsr_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != LSM6DSR_ID) {
        return;
    }

    /* Restore default configuration */
    lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);

    do {
        lsm6dsr_reset_get(&dev_ctx, &rst);
    } while (rst != 0U);

    /* Disable I3C interface */
    lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);

    /* Enable Block Data Update */
    lsm6dsr_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Set Output Data Rate */
    lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_52Hz);
    lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_208Hz);

    /* Set full scale */
    lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
    lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_2000dps);

    /* Enable gyroscope low-pass filter */
    lsm6dsr_gy_filter_lp1_set(&dev_ctx, 1);

    /* Configure data-ready interrupt route */
    lsm6dsr_pin_int1_route_get(&dev_ctx, &int1_route);
    int1_route.int1_ctrl.int1_drdy_xl = PROPERTY_ENABLE;
    lsm6dsr_pin_int1_route_set(&dev_ctx, &int1_route);
    lsm6dsr_data_ready_mode_set(&dev_ctx, LSM6DSR_DRDY_PULSED);

    /* Calculate actual sample rate and period */
    lsm6dsr_odr_cal_reg_get(&dev_ctx, &freq_fine);

    sampleRate =
        (6667.0f + ((0.0015f * (float)freq_fine) * 6667.0f)) /
        (float)ODR_COEFF_208Hz;

    samplePeriod = 1.0f / sampleRate;

    /*
     * INT1 数据就绪中断已在模块端配置好。
     * 当前采用轮询方式，不在 MCU 端使能 GPIO 中断。
     */

    FusionAhrsInitialise(&ahrs);
    FusionOffsetInitialise(&offset, sampleRate);

    platform_delay(200);

    /*
     * 启动时进行静态零偏校准。
     * 校准期间应保持传感器静止。
     */
    offset_cnt = OFFSET_CAL_TIME;

    while (offset_cnt > 0U) {
        uint8_t reg = 0U;

        lsm6dsr_gy_flag_data_ready_get(&dev_ctx, &reg);

        if (reg != 0U) {
            offset_cnt--;

            lsm6dsr_angular_rate_raw_get(
                &dev_ctx,
                data_raw_angular_rate);

            angular_rate_mdps[0] =
                lsm6dsr_from_fs2000dps_to_mdps(
                    data_raw_angular_rate[0]);

            angular_rate_mdps[1] =
                lsm6dsr_from_fs2000dps_to_mdps(
                    data_raw_angular_rate[1]);

            angular_rate_mdps[2] =
                lsm6dsr_from_fs2000dps_to_mdps(
                    data_raw_angular_rate[2]);

            gyroscopeOffset.array[0] +=
                angular_rate_mdps[0] / 1000.0f;

            gyroscopeOffset.array[1] +=
                angular_rate_mdps[1] / 1000.0f;

            gyroscopeOffset.array[2] +=
                angular_rate_mdps[2] / 1000.0f;
        }
    }

    gyroscopeOffset.array[0] /= (float)OFFSET_CAL_TIME;
    gyroscopeOffset.array[1] /= (float)OFFSET_CAL_TIME;
    gyroscopeOffset.array[2] /= (float)OFFSET_CAL_TIME;
}

/*
 * 仅读取 SPI 原始数据，不执行姿态解算。
 * 适合高频调用。
 */
void Read_IMU660RB(void)
{
    uint8_t raw[12];

    /*
     * 从陀螺仪 X 轴低字节寄存器开始连续读取 12 字节：
     * 0x22～0x27：陀螺仪
     * 0x28～0x2D：加速度计
     */
    platform_read(NULL, LSM6DSR_OUTX_L_G, raw, 12U);

    /* 前 6 字节：陀螺仪 */
    data_raw_angular_rate[0] =
        (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);

    data_raw_angular_rate[1] =
        (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);

    data_raw_angular_rate[2] =
        (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

    /* 后 6 字节：加速度计 */
    data_raw_acceleration[0] =
        (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);

    data_raw_acceleration[1] =
        (int16_t)(((uint16_t)raw[9] << 8) | raw[8]);

    data_raw_acceleration[2] =
        (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);

    acceleration_mg[0] =
        lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[0]);

    acceleration_mg[1] =
        lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[1]);

    acceleration_mg[2] =
        lsm6dsr_from_fs2g_to_mg(data_raw_acceleration[2]);

    angular_rate_mdps[0] =
        lsm6dsr_from_fs2000dps_to_mdps(
            data_raw_angular_rate[0]);

    angular_rate_mdps[1] =
        lsm6dsr_from_fs2000dps_to_mdps(
            data_raw_angular_rate[1]);

    angular_rate_mdps[2] =
        lsm6dsr_from_fs2000dps_to_mdps(
            data_raw_angular_rate[2]);
}

/*
 * 基于最近一次读取的数据执行姿态解算。
 * 该函数耗时相对较长，适合低频调用。
 */
void FusionTasks(void)
{
    const FusionVector accelerometer = {
        .array = {
            acceleration_mg[0] / 1000.0f,
            acceleration_mg[1] / 1000.0f,
            acceleration_mg[2] / 1000.0f
        }
    };

    FusionVector gyroscope = {
        .array = {
            angular_rate_mdps[0] / 1000.0f,
            angular_rate_mdps[1] / 1000.0f,
            angular_rate_mdps[2] / 1000.0f
        }
    };

    gyroscope = FusionCalibrationInertial(
        gyroscope,
        gyroscopeMisalignment,
        gyroscopeSensitivity,
        gyroscopeOffset);

    gyroscope = FusionOffsetUpdate(&offset, gyroscope);

    FusionAhrsUpdateNoMagnetometer(
        &ahrs,
        gyroscope,
        accelerometer,
        samplePeriod);

    euler = FusionQuaternionToEuler(
        FusionAhrsGetQuaternion(&ahrs));
}

static uint8_t spiTransferByte(const uint8_t data)
{
    uint8_t read_data;

    DL_SPI_transmitData8(SPI_IMU660RB_INST, data);

    while (DL_SPI_isRXFIFOEmpty(SPI_IMU660RB_INST)) {
        /* Wait for received byte */
    }

    read_data = DL_SPI_receiveData8(SPI_IMU660RB_INST);

    while (DL_SPI_isBusy(SPI_IMU660RB_INST)) {
        /* Wait for SPI transfer completion */
    }

    return read_data;
}

/*
 * @brief Write generic device registers.
 *
 * @param handle Customizable bus argument; unused in this implementation.
 * @param reg    First register address.
 * @param bufp   Pointer to source data.
 * @param len    Number of bytes to write.
 */
static int32_t platform_write(
    void *handle,
    uint8_t reg,
    const uint8_t *bufp,
    uint16_t len)
{
    /* Current project uses only one IMU SPI device. */
    (void)handle;

    DL_GPIO_clearPins(
        GPIO_IMU660RB_PIN_IMU660RB_CS_PORT,
        GPIO_IMU660RB_PIN_IMU660RB_CS_PIN);

    spiTransferByte(reg);

    while (len > 0U) {
        spiTransferByte(*bufp);
        bufp++;
        len--;
    }

    DL_GPIO_setPins(
        GPIO_IMU660RB_PIN_IMU660RB_CS_PORT,
        GPIO_IMU660RB_PIN_IMU660RB_CS_PIN);

    return 0;
}

/*
 * @brief Read generic device registers.
 *
 * @param handle Customizable bus argument; unused in this implementation.
 * @param reg    First register address.
 * @param bufp   Pointer to destination buffer.
 * @param len    Number of bytes to read.
 */
static int32_t platform_read(
    void *handle,
    uint8_t reg,
    uint8_t *bufp,
    uint16_t len)
{
    /* Current project uses only one IMU SPI device. */
    (void)handle;

    DL_GPIO_clearPins(
        GPIO_IMU660RB_PIN_IMU660RB_CS_PORT,
        GPIO_IMU660RB_PIN_IMU660RB_CS_PIN);

    /* Set SPI read bit */
    reg |= 0x80U;
    spiTransferByte(reg);

    while (len > 0U) {
        *bufp = spiTransferByte(0U);
        bufp++;
        len--;
    }

    DL_GPIO_setPins(
        GPIO_IMU660RB_PIN_IMU660RB_CS_PORT,
        GPIO_IMU660RB_PIN_IMU660RB_CS_PIN);

    return 0;
}

/*
 * @brief Platform-specific millisecond delay.
 */
static void platform_delay(uint32_t ms)
{
    mspm0_delay_ms(ms);
}