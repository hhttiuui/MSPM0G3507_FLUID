/*
 * MSPM0G3507 流体模拟 + LED 点阵显示
 *
 * 主循环在 hardware/fluid_demo.c：假重力 → 流体模拟一步 → 映射层 → LED 点阵推屏。
 *   ⚠ 暂时**不接 IMU**，用"假重力"（方向每 2s 换一次）驱动，详见 fluid_demo.c。
 *   ⚠ 想看 LED 点阵的综合演示（全亮→全黑→游走→图案），把 led_show.h 的
 *     LED_SHOW_ENABLE 置 1 即可，那个分支优先级更高。
 * 启动链、时钟节拍（SysTick）、5ms 按键节拍（TIMER_0 中断）和处理函数在本文件。
 *
 * 保留的预留资源（当前不参与流体显示，但初始化完整保留）：
 *   UART_0 = UART1 (PB6=TX / PB7=RX) 中断已配好，收到字节直接丢弃
 *   TIMER_0 = TIMA0，**5ms 周期**，NVIC 已使能 —— 每拍跑 Key_Task() +
 *             PowerKey_Task()（3 路按键扫描 + 一键开关机），见本文件的处理函数。
 *   UART_0 的 NVIC 使能仍是**关闭**的（想用在这里加 NVIC_EnableIRQ）。
 *
 * OLED（SSD1306，I2C1：SCL=PB2 / SDA=PB3）：**整套驱动已删除**
 *         （后续显示只用 LED）。syscfg 里的 I2C_OLED 实例还留着没动，
 *         要彻底清掉得改 .syscfg 再重新生成。
 * LED 点阵：**HT16D33A（16×16 恒流驱动）**，挂 SPI0：
 *          CLK=PA6 / DIO=PA5 / CSB=PA4（CSB 用 GPIO，因为手册要求
 *          最后一个 CLK 上升沿后 CSB 至少再保持低 4µs）。
 *          驱动 software/硬件分层：ht16d33a.c（芯片）→ led_matrix.c（面板映射）
 *          → led_show.c / fluid_demo.c（应用）。
 * IMU：SPI1，SCLK=PA17 / PICO=PA18 / POCI=PA16，CS=PA15（本板是 LSM6DS3TR-C，
 *      WHO_AM_I=0x6A；驱动见 hardware/lsm6ds3trc.c。注意 hardware/IMU660RB/
 *      那套是给 LSM6DSR(0x6B) 的，两者不能混用）。**当前流体还没接它。**
 *
 * 电池采样（`BAT_ADC` = ADC0 通道 0 = PA27）：驱动在 hardware/battery.c
 *         板上分压：VBAT ─100kΩ─┬─100kΩ─ GND，中间抽头接 PA27（比值 2:1）。
 *         ⚠️ ADC 的「通道」和「引脚」是绑死的——换引脚必须同时换通道。
 *         本工程就是从 ADC1/CH4/PB17 改成 ADC0/CH0/PA27，为的是腾出 PB17。
 * 电源保持 + 一键开关机（`GPIO_PWR` / `PIN_PWR_HOLD` = PA22）：
 *         开机自锁靠 SysConfig 的 initialValue = SET 在 GPIO 初始化时完成；
 *         关机靠 USER1 长按 2 秒 → 拉低 PA22。见 hardware/power.h / power.c
 * 按键（`GPIO_USERKEY`，按下 = 低）：USER1 = PA25（内部上拉，一键开关机用）、
 *         USER2 = PA24 / USER3 = PA23（板上外部上拉）。扫描与去抖见
 *         hardware/key.c，由 TIMA0 的 5ms 中断驱动。
 *
 * 封装：**LQFP-48 (PT)**，订货型号 MSPM0G3507SPTR。
 *       PT 与 VQFN-48(RGZ) 的引脚定义完全一致（手册里两者共用同一列脚号），
 *       所以从 RGZ 换成 PT 后 SysConfig 生成的文件逐字节不变，代码零改动。
 *       唯一区别是散热（RθJA 69.2 vs 30.1 °C/W）和焊接难度——
 *       本工程 MCU 功耗仅约 50mW，温升约 3.5°C，用 LQFP 更好焊好查。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "imu_selftest.h"
#include "led_show.h"
#include "fluid_demo.h"
#include "battery.h"
#include "key.h"
#include "power.h"

/* ==================== 中断与周期调度 ==================== */

/*
 * 工程里有两套周期节拍，分工如下：
 *   SysTick  —— 1ms，唯一的「活」节拍：tick_ms 由它递增，全工程的时间判断都依赖它。
 *   TIMER_0  —— TIMA0，5ms，按键节拍：每拍扫一次 3 路按键并跑一次一键开关机。
 *
 * 为什么按键节拍单独用 TIMA0 而不是塞进 SysTick：
 *   按键要的是"稳定的慢节拍"，5ms 一拍刚好（20ms 去抖 = 4 拍，长按 2s = 400 拍）；
 *   而 SysTick 的 1ms 是给 tick_ms 计时用的，两者的周期将来会各自独立调整。
 * 为什么按键状态机放中断里而不是主循环：主循环一旦卡死，PA22 就永远放不掉，
 *   用户只能拔电池 —— 放中断里才保证任何情况下都能关机。
 */
void SysTick_Handler(void)
{
    tick_ms++;
}

/* TIMER_0（TIMA0）5ms 周期：按键扫描 + 一键开关机。
 * ⚠ 这两件事都很轻（3 次 GPIO 读 + 几个计数器比较），适合放中断；
 *   但**不要再往这里加耗时的活**，否则会拖慢整个节拍。 */
void TIMER_0_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
        case DL_TIMERA_IIDX_ZERO:
            Key_Task();         /* 3 路按键：采样 + 去抖 + 更新 dbg.key */
            PowerKey_Task();    /* 一键开关机状态机：只调 Key_IsDown(KEY_USER1) */
            break;

        default:
            break;
    }
}

/* UART_0（UART1，PB7=RX / PB6=TX）接收中断。当前把收到的字节丢弃。 */
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

/* ==================== 初始化与主循环 ==================== */

/*
 * 调试阶段几个额外开关（进入后不返回）；不需要时保持 0 即可：
 *   LED_SHOW_ENABLE       = 1   只跑 LED 点阵综合演示 (hardware/led_show.c)
 *                               **当前 = 0**：让位给流体模拟
 *   IMU_SELFTEST_ENABLE   = 1   只跑 LSM6DS3TR-C 读取自检 (hardware/imu_selftest.c)
 *
 * LED 演示 / IMU 自检在下面用 #if/#elif 互斥，优先级 LED > IMU > 流体 Demo。
 *
 * TIMA0(TIMER_0) 的 NVIC 在 main() 里打开 —— 它现在跑 5ms 的按键节拍
 *   （Key_Task + PowerKey_Task）。UART_0 的 NVIC 仍然是关的，想用在这里加：
 *     NVIC_EnableIRQ(UART_0_INST_INT_IRQN);    // UART1 接收
 * （外设侧的中断使能 SYSCFG_DL_init() 里已经做好了，只差 NVIC 这一层）
 */
int main(void)
{
    /*
     * 下面几步的顺序不能随便调：
     *   ① SYSCFG_DL_init() 会把 PIN_PWR_HOLD(PA22) 置高——**电源自锁就是这一步完成的**。
     *      它必须排在最前：晚一步按键就已经松开了，整机会直接掉电。
     *   ② SysTick_Init() 要给所有靠 tick_ms 计时的模块（LED 演示 / 流体统计）用。
     *   ③ Key_Init() 要排在打开 TIMA0 中断**之前** —— 否则第一拍 Key_Task()
     *      可能拿着还没初始化的按键状态跑。
     */
    SYSCFG_DL_init();           /* 外设初始化（UART_0 / TIMER_0 / I2C_OLED / SPI_IMU660RB /
                                   SPI_LED_DRV / BAT_ADC）+ 电源自锁 PA22 */
    Battery_Init();             /* 使能 BAT_ADC(ADC0) 中断，供 Battery_ReadMilliVolt() 按需采样 */
    SysTick_Init();             /* tick_ms 1ms 节拍，各模块的计时基准 */

    /* 按键取当前电平做初值：一键开关机就是"按着键上电"的，此刻 USER1 已经是低，
     * 初值必须取真实电平，否则去抖器会在 20ms 后把这次开机当成一次"新按下"。 */
    Key_Init();

    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);   /* TIMA0 5ms：按键扫描 + 一键开关机 */

    /* 两个程序互斥，优先级：LED 演示 > 陀螺仪自检 > 流体 Demo。
     * 想跑哪个就把对应的 *_ENABLE 置 1，并把优先级更高的那个关掉。 */
#if LED_SHOW_ENABLE
    LedShow_Run();              /* LED 点阵综合演示（见 hardware/led_show.c），不返回 */
#elif IMU_SELFTEST_ENABLE
    ImuSelfTest_Run();          /* LSM6DS3TR-C 读取自检（无屏，观测量都在 dbg 里），不返回 */
#else
    FluidDemo_Run();            /* 流体模拟 → LED 点阵（暂用假重力），不返回 */
#endif

    return 0;
}
