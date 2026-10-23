/*
 * battery.c —— 电池电压采样（ADC12）+ 运行期低电保护
 *
 * 结构：
 *   ① Bat_SampleOnce()      阻塞单次转换（~130 µs），带超时
 *   ② Battery_ReadMilliVolt 丢弃首次 + 采 BAT_SAMPLES 次取中值
 *   ③ Battery_Task()        调度器：每 BAT_CHECK_MS 置一次"该采样了"
 *   ④ Battery_QuietSample() 在渲染层的熄屏窗口里做精确采样 + 低电判断
 *   ⑤ Battery_ShutdownNow() 释放电源自锁并停机（低电确认后调用）
 *
 * 原理与实测依据见 battery.h 顶部。
 * ⚠ 中断里只置标志（见文件末尾的 BAT_ADC_INST_IRQHandler）。
 */
#include "ti_msp_dl_config.h"
#include "battery.h"
#include "clock.h"          /* tick_ms */
#include "debug.h"          /* dbg.bat */
#include "power.h"          /* Power_Release() */

/* ADC 转换完成标志，由 BAT_ADC_INST_IRQHandler 置位 */
static volatile uint8_t s_adcDone;

/* 阻塞路径的自旋上限。转换耗时 ≈ 采样 125 us + 转换 ≈ 130 us，
 * 正常几百微秒内一定命中；超出只可能是 ADC 没配好，防死等用。 */
#define BAT_ADC_SPIN_LIMIT  200000u

/* ---- 调度 / 低电状态 ---- */
static uint32_t         s_nextCheck;    /* 下一次安静窗口的时刻 */
static uint8_t          s_quietReq;     /* 1 = 渲染层该做一次熄屏采样 */
static uint8_t          s_lowCnt;       /* 连续低电次数 */
static bat_low_action_t s_lowAction;    /* 低电动作（NULL = 直接关机） */

/* ==================== 底层：单次转换（阻塞）==================== */

/* 返回值：0~4095，或 BAT_ADC_RAW_ERROR（超时） */
static uint16_t Bat_SampleOnce(void)
{
    uint32_t guard = 0u;

    s_adcDone = 0u;

    /* ENC = 1 之后 SC = 1 才会真正启动转换，所以先使能再触发 */
    DL_ADC12_enableConversions(BAT_ADC_INST);
    DL_ADC12_startConversion(BAT_ADC_INST);

    while (s_adcDone == 0u) {
        if (++guard > BAT_ADC_SPIN_LIMIT) {
            return BAT_ADC_RAW_ERROR;
        }
    }

    return (uint16_t) DL_ADC12_getMemResult(BAT_ADC_INST, BAT_ADC_ADCMEM_0);
}

/* 三个数取中值 */
static uint16_t Bat_Median3(uint16_t a, uint16_t b, uint16_t c)
{
    uint16_t t;

    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }

    return b;
}

/* raw → mV。先乘后除，避免整数截断误差。
 * 最大 raw × VDDA × num = 4095 × 3350 × 2 = 27 436 500 < 2^32，不溢出 */
static uint32_t Bat_RawToMv(uint16_t raw)
{
    return ((uint32_t) raw * BAT_VDDA_MV * BAT_DIV_NUM) /
           (BAT_ADC_FULL * BAT_DIV_DEN);
}

/* 把一次成功的中值结果发布到 dbg.bat，并维护自开机以来的极值。
 * 极值的用处：mv_max ≈ LED 全灭的读数，mv_min ≈ 最重载的读数，
 * 两者之差就是负载调制量（实测 200~335 mV）——差值大说明读数不可信。 */
static void Bat_Publish(uint16_t raw)
{
    uint32_t mv;

    if (raw == BAT_ADC_RAW_ERROR) {
        dbg.bat.err++;
        return;
    }

    mv = Bat_RawToMv(raw);

    dbg.bat.raw = raw;
    dbg.bat.mv  = mv;
    dbg.bat.cnt++;

    if ((dbg.bat.cnt == 1u) || (mv < dbg.bat.mv_min)) {
        dbg.bat.mv_min = mv;
    }
    if (mv > dbg.bat.mv_max) {
        dbg.bat.mv_max = mv;
    }
}

/* ==================== 对外接口 ==================== */

void Battery_Init(void)
{
    s_adcDone = 0u;

    s_nextCheck = tick_ms + BAT_CHECK_MS;
    s_quietReq  = 0u;
    s_lowCnt    = 0u;
    s_lowAction = 0;

    NVIC_ClearPendingIRQ(BAT_ADC_INST_INT_IRQN);
    NVIC_EnableIRQ(BAT_ADC_INST_INT_IRQN);
}

uint16_t Battery_ReadRaw(void)
{
    return Bat_SampleOnce();
}

uint32_t Battery_ReadMilliVolt(void)
{
    uint16_t v[BAT_SAMPLES];
    uint16_t raw;
    uint8_t  i;

    /* ① 首次转换丢弃：采样电容是"冷"的、LDO 刚上电也还没稳，读数偏低 */
    (void) Bat_SampleOnce();

    /* ② 采 BAT_SAMPLES 次 */
    for (i = 0u; i < BAT_SAMPLES; i++) {
        v[i] = Bat_SampleOnce();
    }

    /* ③ 取中值。单次超时会被中值自动排除（0xFFFF 是最大值） */
    raw = Bat_Median3(v[0], v[1], v[2]);

    if (raw == BAT_ADC_RAW_ERROR) {
        dbg.bat.err++;
        return 0u;              /* 0 mV 物理上不可能 ⇒ 天然的错误标志 */
    }

    Bat_Publish(raw);

    return dbg.bat.mv;
}

uint8_t Battery_IsLow(void)
{
#if (BAT_LOW_MV > 0u)
    /* 只读最近一次的结果，不触发新采样（避免意外阻塞） */
    return ((dbg.bat.mv != 0u) && (dbg.bat.mv < BAT_LOW_MV)) ? 1u : 0u;
#else
    return 0u;
#endif
}

/*
 * 调度器：主循环每帧调一次，**不阻塞**。
 * 到点就置 s_quietReq，等渲染层在合适的位置做熄屏采样。
 */
void Battery_Task(void)
{
#if (BAT_LOW_MV > 0u)
    if ((int32_t) (tick_ms - s_nextCheck) >= 0) {
        s_nextCheck = tick_ms + BAT_CHECK_MS;
        s_quietReq  = 1u;
    }
#endif
}

uint8_t Battery_QuietBegin(void)
{
    return s_quietReq;
}

void Battery_QuietSample(void)
{
#if (BAT_LOW_MV > 0u)
    uint32_t mv;

    s_quietReq = 0u;

    /* 此刻 LED 已经熄灭，读数才是可信的 */
    mv = Battery_ReadMilliVolt();

    if (mv == 0u) {
        return;                     /* 读不到：不参与判断，也不清零计数 */
    }

    dbg.bat.quiet_cnt++;

    if (mv < BAT_LOW_MV) {
        if (s_lowCnt < 0xFFu) {
            s_lowCnt++;
        }
    } else if (mv >= (BAT_LOW_MV + BAT_LOW_HYST_MV)) {
        s_lowCnt = 0u;              /* 回到迟滞区以上才清零 */
    }

    dbg.bat.low_cnt = s_lowCnt;

    if (s_lowCnt >= BAT_LOW_CONFIRM) {
        /* 低电确认。先给"动画接口"一次机会（默认没注册 = 直接关机） */
        if (s_lowAction != 0) {
            s_lowAction();
        }

        /* 钩子返回了也照样关机，避免停在低电状态继续耗电 */
        Battery_ShutdownNow();
    }
#else
    s_quietReq = 0u;
#endif
}

void Battery_SetLowAction(bat_low_action_t fn)
{
    s_lowAction = fn;
}

void Battery_ShutdownNow(void)
{
    dbg.bat.shutdown = 1u;

    /* 释放电源自锁：PA22 → 低，PMOS 栅极被 51k 拉回 VBAT，整机断电。
     * 之后保持电容还能撑一会儿，所以停在这里不再执行任何逻辑。 */
    Power_Release();

    for (;;) {
        __WFI();
    }
}

/* ADC 的转换完成中断（BAT_ADC = ADC0，宏由 SysConfig 生成）。
 * ⚠ 这里只置标志 —— 换算、滤波、判断全部放在任务/主循环里做。 */
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
