/*
 * battery.h —— 电池电压采样（ADC12）
 *
 * ---------------------------------------------------------------------------
 * 硬件（分压 + 高边开关）
 * ---------------------------------------------------------------------------
 *
 *         ┌──[PMOS]──┬── R1 ──┬── PB17 (ADC1_CH4)
 *   VBAT ─┘          │        │
 *                    │        R2
 *                    │        │
 *                   GND      GND
 *                     ↑
 *              或 VDD 上加 load switch，由 GPIO_USERKEY/GPIO_PWR 的富余脚控制（可选）
 *
 *   VBAT_mV = raw × VDDA_mV × (R1+R2)/R2 ÷ 4095
 *
 * ---------------------------------------------------------------------------
 * 转换流程（照抄官方例程 adc12_single_conversion）
 * ---------------------------------------------------------------------------
 *   SYSCFG_DL_init() 里已经调用 DL_ADC12_enableConversions()（ENC = 1）
 *   每次采样：enableConversions() → startConversion() → 等待 → 读 MEM0
 *
 *   参考电压 = VDDA（宏 BAT_ADC_ADCMEM_0_REF_VOLTAGE_V = 3.3）。
 *   ⚠️ 所以 VDDA 必须是**被稳压过**的：
 *      - 有 LDO/DC-DC 把电池稳到 3.3 V → 直接用本模块即可
 *      - 电池直接供 MCU（VDD = VBAT）→ 用 VDDA 当参考测出来恒为满量程，
 *        必须改成内部 VREF（要占用 VREF+ 引脚，另见 README）
 *   另外 MSPM0G3507 的 VDD 上限是 3.6 V，锂电池满电 4.2 V 不能直接接。
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

/* ==================== 需要按实际硬件修改的常数 ==================== */

/* ADC 满量程：12 bit 单端 = 4095
 * （ADC12_CTL2.RES 的复位值就是 12 bit，SysConfig 没改过） */
#define BAT_ADC_FULL          4095u

/* 分压比 = (R1 + R2) / R2。R1 = R2 = 100k 时为 2 */
#define BAT_DIV_NUM              2u
#define BAT_DIV_DEN              1u

/* ADC 参考电压(mV)。这里用 VDDA，与 SysConfig 生成的 3.3 V 对应。
 * 板子做好后用万用表量一下实际 VDDA 会更准。 */
#define BAT_VDDA_MV           3300u

/* 低电阈值(mV)。设为 0 表示不做低电判断 */
#define BAT_LOW_MV            3300u

/* ==================== API ==================== */

/* 使能 ADC 中断。只需在 main() 里调用一次 */
void     Battery_Init(void);

/* 触发一次转换并返回原始值 0 ~ 4095（内部带超时保护） */
uint16_t Battery_ReadRaw(void);

/* 触发一次转换并换算成电池电压，单位 mV */
uint32_t Battery_ReadMilliVolt(void);

/* 电池电压低于 BAT_LOW_MV 时返回 1 */
uint8_t  Battery_IsLow(void);

#endif /* BATTERY_H */
