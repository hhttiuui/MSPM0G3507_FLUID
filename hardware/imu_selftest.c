/*
 * imu_selftest.c —— LSM6DS3TR-C 读取自检（无屏版本）
 *
 * 本工程已砍掉 OLED 显示（后续显示只用 LED），所以这里**不画任何东西**：
 *   初始化 → 连续读六轴 → 每 publish_ms 发布一次窗口统计 + 翻一次心跳 LED。
 *
 * 所有参数都在 debug.c 的 dbg 里，用 CCS 的 Expressions 展开 dbg 看：
 *   who  识别与初始化结果    spi  最近事务/收发字节/超时次数
 *   raw  六轴原始值 + 采样数  phy  换算后的 mg / mdps / m°C
 *   perf 单次读取耗时 + 窗口 min/max   fault HardFault 现场
 *
 * 怎么判断"陀螺仪通没通"：
 *   dbg.who.init_res == 0   识别 + 寄存器读写都正常
 *   dbg.spi.rx 不恒为 0      MISO 有被器件驱动（恒 0x00 或 0xFF = 没接/没通）
 *   dbg.raw.cnt 在涨         采样循环在跑
 *   转动板子时 dbg.phy.g*_mdps 正负跟着变、停下来回到 0 附近
 */
#include "ti_msp_dl_config.h"
#include "clock.h"
#include "debug.h"
#include "lsm6ds3trc.h"
#include "power.h"
#include "imu_selftest.h"

void ImuSelfTest_Run(void)
{
    uint32_t nextPublish;

    (void) LSM6DS3TRC_Init();

    /* 先读一次并发布窗口，免得刚上电时 dbg.perf.read_min_us 一直停在
     * 0xFFFFFFFF（那个初值的含义是"还没发布过窗口"，看着容易误判成坏了）。 */
    LSM6DS3TRC_ReadAll();
    DBG_PerfPublish();

    nextPublish = tick_ms + dbg.test.publish_ms;

    for (;;) {
        LSM6DS3TRC_ReadAll();

        /* 一键开关机：USER1 长按 2s 后由 TIMA0 中断把 PA22 拉低，
         * 这里只负责"别再干活了"，安静等断电（否则关机后还在跑采样没意义）。 */
        PowerKey_CheckShutdown();

        if ((int32_t)(tick_ms - nextPublish) >= 0) {
            DBG_PerfPublish();

            if (dbg.test.heartbeat != 0u) {
                /* 翻指示灯 LED 做心跳：LED 不闪了说明主循环卡住了 */
                DL_GPIO_togglePins(LED_PORT, LED_PIN_LED_PIN);
            }

            nextPublish = tick_ms + dbg.test.publish_ms;
        }

        if (dbg.test.throttle_ms != 0u) {
            mspm0_delay_ms(dbg.test.throttle_ms);
        }
    }
}
