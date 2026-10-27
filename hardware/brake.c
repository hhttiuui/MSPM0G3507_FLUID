/*
 * brake.c —— 刹车高亮：加速度计判定 + 全屏满亮 + 可替换显示
 *
 * 设计说明、标定方法、和低电保护的冲突，全部写在 brake.h 顶部。
 * 这里只放实现，逐段注释讲"为什么这么写"。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "debug.h"
#include "app.h"
#include "brake.h"
#include "led_matrix.h"
#include "ht16d33a.h"

/* ==================== 参数 ==================== */

/* 低通系数。46 Hz 下 τ ≈ 1/(0.25×46) ≈ 87 ms：
 * 足够吃掉踏频（~1.5 Hz）和路面颠簸，又不至于把 0.5 s 的刹车削平。 */
#define BRK_LPF_ALPHA       _IQ(0.25f)

/* 进入 / 退出阈值（mg）。滞回必须留够，否则在边界上会反复开关。 */
#define BRK_THR_DEF_MG      (150)
#define BRK_REL_DEF_MG      (60)
#define BRK_THR_MIN_MG      (30)
#define BRK_THR_MAX_MG      (800)

/* 持续时间：越阈值要挺住多久才认；低于释放阈值要挺住多久才灭；
 * 以及一次刹车**最短**亮多久（点刹只有 0.3 s，不设下限会闪一下就没）。 */
#define BRK_ON_MS           (180u)
#define BRK_OFF_MS          (350u)
#define BRK_MIN_ON_MS       (400u)

/* 刹车画面重推间隔。流体场景一帧 21 ms，这个值必须比它小，
 * 否则会漏掉场景的重绘、出现"闪回液体一帧"。 */
#define BRK_REDRAW_MS       (15u)

/* 刹车时的全局亮度。恒流档（0x36）**不动** —— 那是装机时定的硬件电流，
 * 每帧改它会让电源纹波变大；要更亮就全局把 cc 调高。 */
#define BRK_BOOST_BC        HT16D33A_BRIGHTNESS_MAX

/* ==================== 状态 ==================== */

static uint8_t  s_enabled = 1u;     /* brk 0 可关掉检测 */
static uint8_t  s_forced;           /* brkf 1 强制进入（演示 / 拍摄）*/
static uint8_t  s_active;           /* 1 = 正在刹车高亮 */
static uint8_t  s_axis;             /* 0 = 屏幕 x，1 = 屏幕 y */
static int8_t   s_sign = +1;        /* 行进方向对应哪个符号 */
static uint16_t s_thrMg = BRK_THR_DEF_MG;
static uint16_t s_relMg = BRK_REL_DEF_MG;
static uint8_t  s_disp  = BRAKE_DISP_SOLID;

static int32_t  s_filtMg;           /* 低通后的轴加速度（mg，带符号）*/
static uint32_t s_aboveMs;          /* 连续超阈值的时长 */
static uint32_t s_belowMs;          /* 连续低于释放阈值的时长 */
static uint32_t s_onMs;             /* 本次刹车开始的时刻 */
static uint32_t s_lastDrawMs;       /* 上次推刹车画面的时刻 */

/* 进刹车前的画面快照。灰度模式 256 B，二值模式 32 B。
 * 静态图 / 文字定格 / 动图这些**不重绘**的场景，退出刹车就靠它还原。 */
static uint8_t  s_saved[HT16D33A_RAM_BYTES];
static uint8_t  s_snapValid;
static uint8_t  s_snapScene;

static brake_draw_fn s_hook;

/* ==================== 刹车画面 ==================== */

/* 内置 0：全屏满亮。LedMatrix_FillAll(1) 会把 256 个 RAM 槽全写 0xFF，
 * 其中 16 个槽对应面板上不存在的灯（CA19），写了不亮，无害。 */
static void brk_disp_solid(void)
{
    LedMatrix_FillAll(1u);
}

/* 内置 1：下半屏满亮。留作"接口确实可用"的示例 ——
 * 想换成双闪、刹车条、跑马灯，照抄这个函数注册进去就行。 */
static void brk_disp_bar(void)
{
    uint8_t x, y;

    LedMatrix_Clear();

    for (y = (uint8_t) ((LED_APP_ROWS / 2u) + 1u); y <= (uint8_t) LED_APP_ROWS; y++) {
        for (x = 1u; x <= (uint8_t) LED_APP_COLS; x++) {
            LedMatrix_SetCellLevel(x, y, LED_MATRIX_MAX_LEVEL);
        }
    }
}

/* 按当前配置画一帧并推屏。**只在这里 Flush**，所以注册进来的画法
 * 只负责往帧缓冲里画，不要自己推。 */
static void brk_draw(void)
{
    brake_draw_fn fn;

    if (s_disp == BRAKE_DISP_HOOK) {
        fn = s_hook;
    } else if (s_disp == BRAKE_DISP_BAR) {
        fn = brk_disp_bar;
    } else {
        fn = brk_disp_solid;
    }

    if (fn != 0) {
        fn();
    }
    LedMatrix_Flush();
}

/* Q24 模拟单位 → mg。标定用，两个轴都要算，所以单独抽出来。 */
static int16_t brk_to_mg(_iq v)
{
    int32_t t = (int32_t) (((int64_t) v * 1000) / (int64_t) _IQ(9.8f));

    return (int16_t) ((t > 32767) ? 32767 : ((t < -32768) ? -32768 : t));
}

/* ==================== 状态机 ==================== */

static void brk_snapshot(void)
{
    LedMatrix_Snapshot(s_saved);
    s_snapScene = App_Current();
    s_snapValid = 1u;
}

static void brk_activate(void)
{
    if (s_active != 0u) {
        return;
    }

    brk_snapshot();                 /* 先存下场景的画面，退出时原样还回去 */
    s_active     = 1u;
    s_aboveMs    = 0u;              /* 计时器归零：下一次刹车要重新挺满 180 ms */
    s_belowMs    = 0u;
    s_lastDrawMs = (uint32_t) (tick_ms - (uint32_t) BRK_REDRAW_MS);   /* 立即画一次 */

    dbg.brake.active = 1u;
    dbg.brake.count++;

    HT16D33A_SetBrightness(BRK_BOOST_BC);
    brk_draw();
}

static void brk_deactivate(void)
{
    if (s_active == 0u) {
        return;
    }

    s_active  = 0u;
    s_aboveMs = 0u;
    s_belowMs = 0u;
    dbg.brake.active = 0u;

    /* 亮度还给用户设定的值（不是写死 MAX —— 和低电安静窗口同一个道理） */
    HT16D33A_SetBrightness(App_Brightness());

    /* 还原画面。场景在刹车期间被切走过的话，快照已经由
     * Brake_NotifySceneChanged() 更新成新场景的内容，这里照样安全。 */
    if ((s_snapValid != 0u) && (s_snapScene == App_Current())) {
        LedMatrix_Restore(s_saved);
        LedMatrix_Flush();
    }

    s_snapValid = 0u;
}

/* 刹车态下每帧做两件事：重设亮度 + 重推画面。
 *
 * 为什么要重设亮度：battery.c 每 2 s 熄屏 1 ms 采样，采完恢复成用户设定值。
 * 只设一次的话会被打回正常亮度，刹车看起来"没反应"。
 *
 * 为什么要节流：静态图场景的 tick 是空的，主循环被 __WFI() 压到约 1 kHz，
 * 不节流就是每秒推 1000 次屏（每次 1.2 ms），CPU 直接跑满。 */
static void brk_hold(void)
{
    uint32_t now = tick_ms;

    if ((uint32_t) (now - s_lastDrawMs) < (uint32_t) BRK_REDRAW_MS) {
        return;
    }
    s_lastDrawMs = now;

    HT16D33A_SetBrightness(BRK_BOOST_BC);
    brk_draw();
}

/* ==================== 对外接口 ==================== */

void Brake_Init(void)
{
    s_enabled   = 1u;
    s_forced    = 0u;
    s_active    = 0u;
    s_axis      = 0u;
    s_sign      = +1;
    s_thrMg     = BRK_THR_DEF_MG;
    s_relMg     = BRK_REL_DEF_MG;
    s_disp      = BRAKE_DISP_SOLID;
    s_filtMg    = 0;
    s_aboveMs   = 0u;
    s_belowMs   = 0u;
    s_onMs      = 0u;
    s_lastDrawMs = tick_ms;
    s_snapValid = 0u;
    s_snapScene = 0u;
    s_hook      = 0;

    dbg.brake.active  = 0u;
    dbg.brake.enabled = s_enabled;
    dbg.brake.disp    = s_disp;
    dbg.brake.forced  = 0u;
    dbg.brake.val_mg  = 0;
    dbg.brake.thr_mg  = s_thrMg;
    dbg.brake.count   = 0u;
}

void Brake_Tick(const app_in_t *in)
{
    int32_t  raw, mg, decel;
    uint32_t dt;

    /* ---- 两个轴的原始值都换算成 mg 存进 dbg（**标定就靠这两个数**）----
     * 静止时：重力压在哪个轴上，那个轴就是 ±1000 mg 左右；
     * 另一个接近 0 的才是"行进方向"轴，刹车检测要选它。
     * 这两个值是**未滤波、未选轴**的，就是屏幕坐标下的原始加速度。 */
    dbg.brake.rawx_mg = brk_to_mg(in->ax_raw);
    dbg.brake.rawy_mg = brk_to_mg(in->ay_raw);

    /* ---- 取轴 → 转成"减速度" ----
     * ⚠ 这里必须**有符号**。以前用 |v|，结果是：
     *   ① brksgn 完全不起作用（绝对值把符号吃掉了）；
     *   ② 起步猛蹬（0.2 g 以上）和刹车一样会点亮 —— 方向反了。
     * 现在定义 decel = 沿行进方向的减速度（正 = 正在减速）：
     *   s_sign = +1 表示"行进方向 = 轴的正方向"，刹车时原始值变负，取反得正；
     *   s_sign = -1 表示行进方向是轴的反方向，刹车时原始值变正，直接用。
     */
    raw = (s_axis == 0u) ? in->ax_raw : in->ay_raw;

    /* Q24 模拟单位 → mg（9.8 对应 1000 mg）。
     * 注意这里是**整数域**的换算，不是 _IQmpy —— 用 _IQmpy 会把
     * alpha(≈4e6) 乘上 mg 值，32 位溢出后结果完全错。 */
    mg = (int32_t) (((int64_t) raw * 1000) / (int64_t) _IQ(9.8f));

    /* ---- 一阶低通：y += α(x − y)。用 64 位算，避免 _IQmpy 的饱和 ---- */
    s_filtMg += (int32_t) (((int64_t) BRK_LPF_ALPHA * (int64_t) (mg - s_filtMg))
                           >> IQ_SHIFT);

    /* 减速度：正 = 正在减速（就是"该亮刹车灯"的方向）*/
    decel = (s_sign > 0) ? -s_filtMg : s_filtMg;

    dbg.brake.val_mg = (int16_t) ((decel > 32767) ? 32767
                                  : ((decel < -32768) ? -32768 : decel));

    /* ---- 强制模式：跳过检测，直接维持刹车态（拍摄 / 演示）---- */
    if (s_forced != 0u) {
        if (s_active == 0u) {
            brk_activate();
        }
        brk_hold();
        return;
    }

    /* ---- 检测关闭 / 没有 IMU：立刻退出刹车态 ----
     * 没有 IMU 时 ax_raw/ay_raw 恒为 0（假重力不写这两个字段），
     * 不显式退出的话会永远停在刹车态。 */
    if ((s_enabled == 0u) || (dbg.fluid.imu_use == 0u)) {
        if (s_active != 0u) {
            brk_deactivate();
        }
        s_aboveMs = 0u;
        s_belowMs = 0u;
        return;
    }

    /* ---- 滞回计时（**有符号**：只看减速方向）----
     * decel ≥ thr        → 正在刹车
     * decel ≤ rel(60mg)  → 没在刹车（含"正在加速"，decel 为负）
     * 中间地带两个计时器都不动，这就是滞回。 */
    dt = (in->dt_ms != 0u) ? (uint32_t) in->dt_ms : 1u;

    if (decel >= (int32_t) s_thrMg) {
        s_aboveMs += dt;
        s_belowMs  = 0u;
    } else if (decel <= (int32_t) s_relMg) {
        s_belowMs += dt;
        s_aboveMs  = 0u;
    }

    if (s_active == 0u) {
        if (s_aboveMs >= (uint32_t) BRK_ON_MS) {
            s_onMs = in->now_ms;
            brk_activate();
        }
    } else {
        if ((s_belowMs >= (uint32_t) BRK_OFF_MS) &&
            ((uint32_t) (in->now_ms - s_onMs) >= (uint32_t) BRK_MIN_ON_MS)) {
            brk_deactivate();
        }
    }

    if (s_active != 0u) {
        brk_hold();
    }
}

void Brake_NotifySceneChanged(void)
{
    /* 刹车期间切了场景：新场景的 enter() 已经把画面画好了，
     * 重新快照一次，退出刹车时才不会还原成旧场景的画面。 */
    if (s_active != 0u) {
        brk_snapshot();
        s_lastDrawMs = (uint32_t) (tick_ms - (uint32_t) BRK_REDRAW_MS);
    }
}

uint8_t Brake_IsActive(void)
{
    return s_active;
}

void Brake_SetEnabled(uint8_t on)
{
    s_enabled = (on != 0u) ? 1u : 0u;
    dbg.brake.enabled = s_enabled;

    if (s_enabled == 0u) {
        s_aboveMs = 0u;
        s_belowMs = 0u;
        brk_deactivate();
    }
}

void Brake_SetForced(uint8_t on)
{
    s_forced = (on != 0u) ? 1u : 0u;
    dbg.brake.forced = s_forced;

    if (s_forced == 0u) {
        brk_deactivate();
    }
}

uint8_t Brake_SetThreshold(uint16_t mg)
{
    if ((mg < (uint16_t) BRK_THR_MIN_MG) || (mg > (uint16_t) BRK_THR_MAX_MG)) {
        return 0u;
    }

    s_thrMg = mg;
    /* 释放阈值跟着走（保持 1:0.4 的比例），否则调大阈值后滞回带可能变负 */
    s_relMg = (uint16_t) ((mg * 2u) / 5u);

    dbg.brake.thr_mg = s_thrMg;
    return 1u;
}

void Brake_SetAxis(uint8_t axis)
{
    s_axis = (axis != 0u) ? 1u : 0u;
    s_filtMg = 0;                   /* 换轴后滤波器从头来，避免旧值拖尾 */
}

void Brake_SetSign(int8_t sign)
{
    s_sign = (sign < 0) ? -1 : +1;
    s_filtMg = 0;
}

uint8_t Brake_SetDisplay(uint8_t id)
{
    if ((id != BRAKE_DISP_SOLID) && (id != BRAKE_DISP_BAR) &&
        (id != BRAKE_DISP_HOOK)) {
        return 0u;
    }

    s_disp = id;
    dbg.brake.disp = s_disp;

    if (s_active != 0u) {
        s_lastDrawMs = (uint32_t) (tick_ms - (uint32_t) BRK_REDRAW_MS);
    }
    return 1u;
}

void Brake_SetDisplayHook(brake_draw_fn fn)
{
    s_hook = fn;
}
