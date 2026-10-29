/*
 * battery.h —— 电池电压采样（ADC12）+ 运行期低电保护
 *
 * ---------------------------------------------------------------------------
 * 硬件（分压 + 高边开关）
 * ---------------------------------------------------------------------------
 *
 *         ┌──[PMOS]──┬── R1 ──┬── PA27 (ADC0 通道 0)
 *   VBAT ─┘          │        │
 *                    │        R2
 *                    │        │
 *                   GND      GND
 *
 *   实际阻值：R1 = R2 = 10 kΩ（2026-10-15 实测）。比值 2:1，与旧文档写的
 *   100k+100k 相同，所以 BAT_DIV_NUM/DEN 不用改。
 *   ⚠️ 旧版本这里写的是 "PB17 (ADC1_CH4)"，是错的 —— 本工程已改成
 *      ADC0 / 通道 0 / PA27（为的是腾出 PB17）。按旧注释接线会接错引脚。
 *
 *   VBAT_mV = raw × VDDA_mV × (R1+R2)/R2 ÷ 4095
 *
 * ---------------------------------------------------------------------------
 * 比率式测量：实测标定结果（2026-10-22，可调电源 + 实测扫描）
 * ---------------------------------------------------------------------------
 *   参考电压用 VDDA，而 VDDA 由 ME6206A33XG 从电池降压得到。
 *
 *   ① **VDDA 实测 ≈ 3350 mV，不是 3300**。
 *      用可调电源从 4.2 V 扫到 3.4 V，九档读数反推出的 VDDA 全部落在
 *      3329~3389 mV（均值 3350）。所以 BAT_VDDA_MV 已按实测改成 3350。
 *      ME6206A33XG 精度 ±2%（3.234~3.366 V），3350 在规格内。
 *
 *   ② **读数有个"地板" ≈ 3450 mV**（= VDDA + LDO 压差 ≈ 3350 + 100）。
 *      电池继续下降时 LDO 掉出稳压区，VDDA 跟着塌，而 ADC 是比率式测量
 *      ⇒ 读数会**卡住甚至反向**（电池越没电，读数越大）。
 *      实测：3.4 V 时读数 3342（仍略低于电源，方向正常）；
 *            **3.3 V 时开始反向** —— 这就是地板。
 *      ⇒ 所以低电阈值必须明显高于地板：BAT_LOW_MV = 3550（地板之上 100 mV）。
 *        定成 3300/3400 会导致保护静默失效（永远不触发）。
 *
 *   ③ **读数会被 LED 负载压低 200~335 mV**（实测跨度：
 *      4.2 V 时 213 mV、3.8 V 时 335 mV）。LED 点阵满载 15 颗 × 24 mA = 360 mA，
 *      逐相扫描，压降落在"可调电源引线 + 端子 + 一键开关机 PMOS + PCB 走线"上。
 *      ⚠️ 这意味着：**带载读数根本不能用来判断低电** —— 一颗健康的 3.8 V 电池
 *        满载时会读成 3.43 V，直接低于阈值 ⇒ 会误关机。
 *
 * ---------------------------------------------------------------------------
 * 所以运行期低电检查用"安静窗口"采样
 * ---------------------------------------------------------------------------
 *   每 BAT_CHECK_MS 毫秒，把 LED 全局亮度（0x37）设为 0 熄屏 → 等
 *   BAT_SETTLE_MS 让电流和电源恢复 → 精确采样 → 立刻恢复亮度。
 *   熄屏时长 ≈ 1.5 ms（含一次采样 ~0.5 ms），肉眼不可见。
 *
 *   连续 BAT_LOW_CONFIRM 次低于 BAT_LOW_MV 才关机（默认 3 次 × 2 s = 6 s），
 *   防止一次异常读数把设备关掉。
 *
 *   ⚠️ 渲染层必须配合：每帧开头先问 Battery_QuietBegin()，返回 1 就执行上面的
 *      "熄屏 → 延时 → Battery_QuietSample() → 恢复亮度"。见 fluid_demo.c 的
 *      FluidDemo_Push()。漏掉这一步 = 运行期低电保护不工作（不会误关机，但也
 *      不会保护）。
 *
 * ---------------------------------------------------------------------------
 * 采样放在哪里（重要）
 * ---------------------------------------------------------------------------
 *   · ADC 中断里**只置标志**，不做任何耗时操作。
 *   · 不要放进 TIMA0 的 5 ms 中断：一次转换 ≈130 µs，占该节拍 2.6%，会拉歪
 *     按键扫描；而且 Flash 擦写期间中断是关闭的。
 *   · 精确采样放在主循环的"安静窗口"里（阻塞 ~0.5 ms，每 2 s 一次）。
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

/* ==================== 需要按实际硬件修改的常数 ==================== */

/* ADC 满量程：12 bit 单端 = 4095
 * （ADC12_CTL2.RES 的复位值就是 12 bit，SysConfig 没改过） */
#define BAT_ADC_FULL          4095u

/* 分压比 = (R1 + R2) / R2。R1 = R2 = 10k 时为 2 */
#define BAT_DIV_NUM              2u
#define BAT_DIV_DEN              1u

/* ADC 参考电压(mV)。用 VDDA 当参考，**实测反推值 ≈3350**（不是标称 3300）。
 * 依据：4.2→3.4 V 九档扫描，反推 VDDA 全部落在 3329~3389 mV。 */
#define BAT_VDDA_MV           3350u

/* 低电阈值(mV)。设为 0 表示关闭低电保护。
 * ⚠️ 必须明显高于"读数地板"（≈3450 = VDDA + LDO压差）：
 *    3.3 V 时读数开始反向，若阈值贴太近，保护会永远不触发。
 *    3550 = 地板 + 100 mV 余量，对应电池空载约 3.57 V（剩 10% 左右）。 */
#define BAT_LOW_MV            3550u

/* 低电解除的迟滞(mV)：回到 BAT_LOW_MV + 这个值以上才重新计数 */
#define BAT_LOW_HYST_MV        100u

/* 运行期低电检查周期(ms) */
#define BAT_CHECK_MS          2000u

/* 连续多少次低于阈值才关机。3 次 × 2 s = 6 s 确认窗口，防单次误判 */
#define BAT_LOW_CONFIRM          3u

/* 熄屏后等待多久再采样(ms)：让 LED 电流归零、电源从压降中恢复 */
#define BAT_SETTLE_MS            1u

/* 单次读取的采样次数（中值滤波）。
 * 中值天然抗单次野值；平均值会被一个异常值整体拉偏。 */
#define BAT_SAMPLES              3u

/* 转换超时时返回的错误码。
 * ⚠️ 不能用 0 当错误码：raw = 0 是合法结果（电池拔了 / 分压断了）*/
#define BAT_ADC_RAW_ERROR    (0xFFFFu)

/* ==================== API ==================== */

/* 使能 ADC 中断。只需在 main() 里调用一次 */
void     Battery_Init(void);

/* 触发一次转换并返回原始值 0 ~ 4095；超时返回 BAT_ADC_RAW_ERROR。
 * ⚠️ 阻塞约 130 µs，不要在中断里调用 */
uint16_t Battery_ReadRaw(void);

/* 丢弃首次 + 采 BAT_SAMPLES 次取中值，换算成电池电压(mV)。
 * 失败返回 0；成功时结果同时发布到 dbg.bat。
 * ⚠️ 阻塞约 (BAT_SAMPLES+1) × 130 µs ≈ 0.5 ms。
 *    **必须在 LED 熄灭时调用**（带载读数会低 200~335 mV）：
 *      · 开机时 main() 里调一次（那时 LED 还没点亮）
 *      · 运行期由 Battery_QuietSample() 在熄屏窗口里调 */
uint32_t Battery_ReadMilliVolt(void);

/* 最近一次采样的电压是否低于 BAT_LOW_MV（带迟滞，不触发新采样）。
 * 需要"当前值"请读 dbg.bat.mv；需要判断保护状态请读 dbg.bat.low_cnt。 */
uint8_t  Battery_IsLow(void);

/* 运行期低电检查的调度器：主循环每帧调一次（非阻塞，只置一个"该采样了"标志）。
 * 实际采样由渲染层的安静窗口完成，见 Battery_QuietBegin()。 */
void     Battery_Task(void);

/* ---- 安静窗口（渲染层配合）----
 *
 * 典型用法（见 fluid_demo.c 的 FluidDemo_Push）：
 *
 *     if (Battery_QuietBegin() != 0u) {
 *         HT16D33A_SetBrightness(0);               // 熄屏
 *         mspm0_delay_ms(BAT_SETTLE_MS);
 *         Battery_QuietSample();                   // 可能关机（不返回）
 *         HT16D33A_SetBrightness(HT16D33A_BRIGHTNESS_MAX);
 *     }
 *
 * Battery_QuietBegin() 返回 1 表示"本轮该精确采样了"（一次性）。
 */
uint8_t  Battery_QuietBegin(void);

/* 在熄屏窗口里做一次精确采样：更新 dbg.bat，并累计低电次数。
 * 达到 BAT_LOW_CONFIRM 次即触发关机（不返回）。 */
void     Battery_QuietSample(void);

/* ==================== 低电关机 + 动画接口 ====================
 *
 * 默认行为（没注册动作时）：低电确认后**直接关机**。
 *
 * 本工程注册了关机动画，见 app.c 的 App_LowBatteryAction() / App_ShutdownAnim()：
 *
 *     static void App_LowBatteryAction(void)
 *     {
 *         App_ShutdownAnim();           // 电池一格一格空掉 + 闪烁，约 1.1 s
 *         Battery_ShutdownNow();        // 播完再关
 *     }
 *     Battery_SetLowAction(App_LowBatteryAction);   // 在 App_Init() 里注册
 *
 * 要点：
 *   · 动作函数运行在主循环上下文，可以正常调用显示/延时函数；
 *   · 它应当在结束时调用 Battery_ShutdownNow()（不返回）；
 *   · 即使它返回了，Battery_QuietSample() 也会兜底调用一次关机，不会停在低电状态；
 *   · 动作函数里**不要**再调用 Battery_QuietSample()（会递归）。
 *   · ⚠ 进动作函数时全局亮度（0x37）还是 **0** —— 调用方为了精确采样刚把屏
 *     熄掉了。要在里面显示任何东西，第一步必须先把亮度恢复，否则整段动画
 *     都在黑屏上播完（这个坑在 App_ShutdownAnim() 里踩过）。
 */
typedef void (*bat_low_action_t)(void);

/* 注册低电动作（NULL = 直接关机，默认） */
void Battery_SetLowAction(bat_low_action_t fn);

/* 立即关机：释放电源自锁（PA22 拉低），然后停在这里等硬件断电。不返回。 */
void Battery_ShutdownNow(void);

#endif /* BATTERY_H */
