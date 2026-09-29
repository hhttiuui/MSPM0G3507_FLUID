/*
 * MSPM0G3507 流体模拟 + OLED 显示
 *
 * 主循环在 hardware/fluid_demo.c：读 IMU 加速度计 → 流体模拟一步 → 推屏。
 * 启动链、时钟节拍和三个预留中断处理函数在本文件。
 *
 * 保留的预留资源（当前不参与流体显示，但初始化完整保留）：
 *   UART_0 = UART1 (PA9=RX / PA8=TX) 中断已配好，收到字节直接丢弃
 *   UART_1 = UART3 (PB3=RX)         中断已配好，收到字节直接丢弃
 *   TIMER_0 = TIMA0，1ms 周期        外设中断已使能，处理函数是空壳
 *   注意：这三个中断的 **NVIC 使能是关闭的**（见 main() 里注释），
 *         处理函数保留着是为了随时能打开。
 * OLED（SSD1306，I2C1）：SDA=PA16，SCL=PA15。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "clock_selftest.h"
#include "oled_selftest.h"
#include "fluid_demo.h"
#include "oled_hardware_i2c.h"

/* ==================== 中断与周期调度 ==================== */

void SysTick_Handler(void)
{
    tick_ms++;
}

/* TIMER_0（TIMA0）1ms 周期。当前只把挂起标志读掉，留作后续扩展。 */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            break;

        default:
            break;
    }
}

/* UART_0（UART1，PA9=RX / PA8=TX）接收中断。当前把收到的字节丢弃。 */
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            (void)DL_UART_Main_receiveData(UART_0_INST);
            break;

        default:
            break;
    }
}

/* UART_1（UART3，PB3=RX）接收中断。当前把收到的字节丢弃。 */
void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            (void)DL_UART_Main_receiveData(UART_1_INST);
            break;

        default:
            break;
    }
}

/* ==================== 初始化与主循环 ==================== */

/*
 * 调试阶段两个额外开关（进入后不返回）；不需要时保持 0 即可：
 *   CLOCK_SELFTEST_ENABLE = 1   只跑 CPUCLK 实测自检  (hardware/clock_selftest.c)
 *   OLED_SELFTEST_ENABLE  = 1   只跑 OLED 刷屏/映射验证 (hardware/oled_selftest.c)
 *
 * 三个预留外设的 NVIC 使能目前是关的，想用的时候在这里加：
 *     NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);   // 1ms 周期
 *     NVIC_EnableIRQ(UART_0_INST_INT_IRQN);    // UART1 接收
 *     NVIC_EnableIRQ(UART_1_INST_INT_IRQN);    // UART3 接收
 * （外设侧的中断使能 SYSCFG_DL_init() 里已经做好了，只差 NVIC 这一层）
 */
int main(void)
{
    SYSCFG_DL_init();           /* 外设初始化（含 UART_0 / UART_1 / TIMER_0） */
    SysTick_Init();             /* tick_ms 1ms 节拍，OLED 的超时判断依赖它 */

    OLED_Init();
    OLED_Clear();

#if CLOCK_SELFTEST_ENABLE
    ClockSelfTest_Run();        /* 实测 CPUCLK 是否为 80MHz，不返回 */
#endif

#if OLED_SELFTEST_ENABLE
    OledDrawTest_Run();         /* 刷屏耗时 + 网格映射验证，不返回 */
#endif

    FluidDemo_Run();            /* 流体模拟 + OLED 显示，不返回 */

    return 0;
}
