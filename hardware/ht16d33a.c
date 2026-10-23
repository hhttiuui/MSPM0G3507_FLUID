/*
 * ht16d33a.c —— HT16D33A 16×16 LED 点阵驱动实现（SPI 3 线）
 *
 * 所有依据都在 ht16d33a.h 顶部（手册页码都标了）。
 *
 * 通信：SPI0 已由 syscfg 初始化（2MHz / Mode0 / 8bit / MSB first），本文件只做
 * "拉低 CSB → 移字节 → 满足 tCSH → 拉高 CSB"这套事务，不碰 SPI 配置。
 *
 * ⚠ 这颗芯片没有可用的回读通路（本板 DIO 只做输出），所以 dbg.led 里看到的是
 *   "我们发出了什么"，有没有真的点亮只能看灯。
 */

#include "ht16d33a.h"
#include "debug.h"
#include "clock.h"

#define CSB_PORT    GPIO_LED_DRV_PORT
#define CSB_PIN     GPIO_LED_DRV_PIN_LED_CSB_PIN

/* SPI 等待上限：一次 8bit 移位只要几十个循环，给个宽松上限防止死等 */
#define SPI_WAIT_LIMIT      (100000u)

/* LED on/off 参考表：开源驱动里标着 "see HT16D33A datasheet page21~22" 的那张，
 * 但它是**别人那块面板**的图案（由面板接线/未用引脚决定），在我们板上不适用。
 * 只留作对照，默认不用。 */
static const uint8_t s_onoffRef[HT16D33A_ONOFF_BYTES] = {
    0xF8, 0x7F, 0xF8, 0x7F, 0xF8, 0x7F, 0xF8, 0x7F,
    0xCE, 0x7F, 0xCE, 0x7F, 0xCE, 0x7F, 0x7E, 0x7E,
    0x7E, 0x7E, 0x7E, 0x7E, 0xFE, 0x73, 0xFE, 0x73,
    0xFE, 0x73, 0xFE, 0x1F, 0xFE, 0x1F, 0xFE, 0x1F,
};

/* ==================== 最底层 ==================== */

static void CsbLow(void)
{
    DL_GPIO_clearPins(CSB_PORT, CSB_PIN);
}

/* 拉高 CSB 前必须满足 tCSH ≥ 4µs（最后 CLK → CSB 上升），否则最后一个字节可能没被锁存 */
static void CsbHigh(void)
{
    DL_Common_delayCycles(HT16D33A_CSB_HOLD_CYCLES);
    DL_GPIO_setPins(CSB_PORT, CSB_PIN);
}

static uint8_t SpiTransferByte(uint8_t data)
{
    uint32_t guard;
    uint8_t  rd;

    DL_SPI_transmitData8(SPI_LED_DRV_INST, data);

    guard = SPI_WAIT_LIMIT;
    while (DL_SPI_isRXFIFOEmpty(SPI_LED_DRV_INST) && (--guard != 0u)) {
        /* Wait for received byte */
    }
    if (guard == 0u) {
        dbg.led.timeout++;      /* 应为 0；>0 说明 SPI 没在转（配置/时钟问题） */
    }

    rd = DL_SPI_receiveData8(SPI_LED_DRV_INST);

    guard = SPI_WAIT_LIMIT;
    while (DL_SPI_isBusy(SPI_LED_DRV_INST) && (--guard != 0u)) {
        /* Wait for SPI transfer completion */
    }
    if (guard == 0u) {
        dbg.led.timeout++;
    }

    dbg.led.xfer++;
    dbg.led.tx = data;

    return rd;
}

/* 一个 CSB 帧：[首字节][数据…] */
static void Frame(uint8_t first, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    dbg.led.bytes = (uint16_t) (len + 1u);

    CsbLow();
    (void) SpiTransferByte(first);
    for (i = 0u; i < len; i++) {
        (void) SpiTransferByte(data[i]);
    }
    CsbHigh();
}

/* ==================== 命令 / 页 / RAM ==================== */

void HT16D33A_WriteCmd(uint8_t cmd, uint8_t data)
{
    dbg.led.cmd  = cmd;
    dbg.led.data = data;

    Frame(cmd, &data, 1u);
}

void HT16D33A_SelectPage(uint8_t page)
{
    dbg.led.page = page;

    /* 0xFD 后跟页号（0x00~0x0F → Page1~Page16）。
     * 之所以每轮写 RAM 前都显式选一次，是因为"当前页"是器件里的状态，
     * 别的命令（图像/影像模式）也可能改它，不显式选就容易写错页。 */
    HT16D33A_WriteCmd(HT16D33A_CMD_SET_RAM_PAGE, (uint8_t) (page & 0x0Fu));
}

/* 一个 CSB 帧写一段显示 RAM：[0x80][起始地址][数据…] */
static void RamChunk(uint8_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t  buf[HT16D33A_BURST_MAX + 1u];
    uint16_t i;

    buf[0] = addr;
    for (i = 0u; i < len; i++) {
        buf[i + 1u] = data[i];
    }

    dbg.led.addr = addr;
    Frame(HT16D33A_CMD_WRITE_LED_DATA, buf, (uint16_t) (len + 1u));
}

void HT16D33A_WriteRam(uint8_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t chunk;

    HT16D33A_SelectPage(HT16D33A_PAGE_DEFAULT);

    while (len > 0u) {
        chunk = (len > HT16D33A_BURST_MAX) ? HT16D33A_BURST_MAX : len;

        RamChunk(addr, data, chunk);

        addr  = (uint8_t) (addr + chunk);
        data += chunk;
        len  -= chunk;
    }
}

void HT16D33A_FillRam(uint8_t addr, uint8_t value, uint16_t len)
{
    uint8_t  buf[HT16D33A_BURST_MAX];
    uint16_t i;
    uint16_t chunk;

    for (i = 0u; i < (uint16_t) HT16D33A_BURST_MAX; i++) {
        buf[i] = value;
    }

    HT16D33A_SelectPage(HT16D33A_PAGE_DEFAULT);

    while (len > 0u) {
        chunk = (len > HT16D33A_BURST_MAX) ? HT16D33A_BURST_MAX : len;

        RamChunk(addr, buf, chunk);

        addr = (uint8_t) (addr + chunk);
        len -= chunk;
    }
}

void HT16D33A_WriteOnOff(uint8_t pattern)
{
    uint8_t  buf[HT16D33A_ONOFF_BYTES + 1u];
    uint16_t i;

    buf[0] = 0x00u;                     /* 起始地址（5 位，0 = 头一个字节）*/

    for (i = 0u; i < (uint16_t) HT16D33A_ONOFF_BYTES; i++) {
        if (pattern == HT16D33A_ONOFF_ALL_ENABLE) {
            buf[i + 1u] = 0xFFu;        /* 全部 LED 受显示数据控制 */
        } else if (pattern == HT16D33A_ONOFF_ALL_MASK) {
            buf[i + 1u] = 0x00u;        /* 全部屏蔽 = 全黑 */
        } else {
            buf[i + 1u] = s_onoffRef[i];
        }
    }

    HT16D33A_SelectPage(HT16D33A_PAGE_DEFAULT);

    dbg.led.addr = 0u;
    Frame(HT16D33A_CMD_WRITE_LED_ONOFF, buf, (uint16_t) (HT16D33A_ONOFF_BYTES + 1u));
}

/* 全局亮度（0x37 的 BC7..0）。0 = 全灭，0xFF = 最亮。
 * 低电检查的"熄屏窗口"用它：立即生效、不动显示 RAM，写完再设回 MAX
 * 就能原样恢复画面（不需要重画一帧）。 */
void HT16D33A_SetBrightness(uint8_t level)
{
    HT16D33A_WriteCmd(HT16D33A_CMD_SET_BRIGHTNESS, level);
}

/* ==================== 初始化 ==================== */

uint8_t HT16D33A_Init(void)
{
    uint32_t t0 = tick_ms;

    dbg.led.init_res = 0u;

    mspm0_delay_ms(10);                                  /* 等上电稳定 */

    HT16D33A_SelectPage(HT16D33A_PAGE_DEFAULT);          /* 0xFD：先选中 Page1 */

    HT16D33A_WriteCmd(HT16D33A_CMD_RESET, 0x00u);        /* 0xCC 软件复位 */
    mspm0_delay_ms(5);

    HT16D33A_WriteCmd(HT16D33A_CMD_DISPLAY_MODE, 0x00u); /* 0x30：MP=0 */

    /* 0x31：BGS + 矩阵规格。16×16 是"矩阵类型 3"，MT1:0 = 3 */
    HT16D33A_WriteCmd(HT16D33A_CMD_CONFIG_MODE, HT16D33A_MODE_VALUE);

    HT16D33A_WriteCmd(HT16D33A_CMD_SET_CURRENT, HT16D33A_CURRENT_24MA); /* 恒流半档 24mA */
    HT16D33A_WriteCmd(HT16D33A_CMD_SET_BRIGHTNESS, HT16D33A_BRIGHTNESS_MAX); /* 全局亮度拉满 */
    HT16D33A_WriteCmd(HT16D33A_CMD_SET_V_BLANK, 0x8Fu);                 /* VBEN=1, VB=15 */
    HT16D33A_WriteCmd(HT16D33A_CMD_SYS_CONTROL, HT16D33A_SYS_OSC_ON);

    /* ⚠ 顺序不能颠倒：先让 OSC 起来，才能写 RAM */
    HT16D33A_FillRam(0x00u, 0x00u, HT16D33A_RAM_BYTES);  /* 清屏 */

    /* ⚠ 这一步不能省：LED on/off RAM 复位默认全 0 = 全部屏蔽，
     *   不写全 1 的话显示数据写了也不亮 */
    HT16D33A_WriteOnOff(HT16D33A_ONOFF_ALL_ENABLE);

    HT16D33A_WriteCmd(HT16D33A_CMD_SYS_CONTROL, HT16D33A_SYS_DISPLAY_ON);

    dbg.led.init_ms = tick_ms - t0;

    return dbg.led.init_res;
}
