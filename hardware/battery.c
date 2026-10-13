/*
 * battery.c —— 电池电压采样（ADC12）
 *
 * 见 battery.h 的说明。转换流程与官方例程 adc12_single_conversion 一致：
 * 中断方式等待 MEM0 结果，只是把例程里的 __WFE() 换成了带超时的自旋，
 * 避免在没有其他唤醒源时死等。
 */
#include "ti_msp_dl_config.h"
#include "battery.h"

/* ADC 转换完成标志，由 BAT_ADC_INST_IRQHandler 置位 */
static volatile uint8_t s_adcDone;

/* 自旋上限。转换耗时 ≈ 采样 125 us + 转换 ≈ 130 us，
 * 正常几百微秒内一定命中；超出只可能是 ADC 没配好，防死等用。 */
#define BAT_ADC_TIMEOUT 200000u

void Battery_Init(void)
{
    s_adcDone = 0u;
    NVIC_ClearPendingIRQ(BAT_ADC_INST_INT_IRQN);
    NVIC_EnableIRQ(BAT_ADC_INST_INT_IRQN);
}

uint16_t Battery_ReadRaw(void)
{
    uint32_t guard = 0u;

    s_adcDone = 0u;

    /* ENC = 1 之后 SC = 1 才会真正启动转换，所以先使能再触发 */
    DL_ADC12_enableConversions(BAT_ADC_INST);
    DL_ADC12_startConversion(BAT_ADC_INST);

    while (s_adcDone == 0u) {
        if (++guard > BAT_ADC_TIMEOUT) {
            break;              /* 超时保护，避免死等 */
        }
    }

    return (uint16_t)DL_ADC12_getMemResult(BAT_ADC_INST, BAT_ADC_ADCMEM_0);
}

uint32_t Battery_ReadMilliVolt(void)
{
    /* 先乘后除，避免整数截断带来的误差。
     * 最大 raw × VDDA × num = 4095 × 3300 × 2 = 27 027 000 < 2^32，不溢出 */
    uint32_t raw = (uint32_t)Battery_ReadRaw();

    return (raw * BAT_VDDA_MV * BAT_DIV_NUM) /
           (BAT_ADC_FULL * BAT_DIV_DEN);
}

uint8_t Battery_IsLow(void)
{
#if (BAT_LOW_MV > 0u)
    return (Battery_ReadMilliVolt() < BAT_LOW_MV) ? 1u : 0u;
#else
    return 0u;
#endif
}

/* ADC1 的转换完成中断。宏由 SysConfig 生成：
 * BAT_ADC_INST_IRQHandler → ADC1_IRQHandler */
void BAT_ADC_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(BAT_ADC_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            s_adcDone = 1u;
            break;

        default:
            break;
    }
}
