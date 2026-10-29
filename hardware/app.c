/*
 * app.c —— 场景框架 + 输入层
 *
 * ==================== 本文件管什么 ====================
 *
 *   ① 输入层：IMU 初始化 / 双通路分解 / 假重力退路 / 按键边沿 → app_in_t
 *   ② 场景表：const 数组，顺序即场景号
 *   ③ 切换：App_Goto() = exit → 清屏 → enter
 *   ④ 主循环：6 步，每步都不能阻塞
 *
 * ==================== 重力来源与坐标映射 ====================
 *
 * FLUID_USE_IMU = 1（默认）：用 LSM6DS3TR-C 的加速度计读数当重力。
 *   加速度计静止时测的是"重力反作用力"，所以**平放时某轴读 +1000mg**。
 *
 *   关键：IMU 的芯片坐标系 ≠ 屏幕坐标系，必须做一次坐标变换。
 *   本板已完成标定（见下面 FLUID_IMU_* 宏处的三点实测推导），结论是
 *   **两轴对调 + 都取负**。若换了板子或改了 IMU 贴装方向，需要重新标定：
 *
 *     把屏幕按"正着放 → 顺时针90° → 顺时针180°"各停一次，记下液体落在哪边，
 *     和"应该落在哪边"（下 → 右 → 上）对比，即可解出这 4 个宏。
 *     快速口诀：
 *       液体流向与倾斜方向相反   → 该轴 SIGN 取反
 *       液体左右流但你是上下倾斜 → 两个 SRC 对调
 *
 *   ⚠ 这个映射对**摇晃**同样成立：加速度计测的是比力，"运动加速度"和重力反作用
 *     走的是同一条换算链，所以左右摇时给出的方向自动就是"惯性滞后"的方向
 *     （向右推板子 → 液体往左堆），不需要为摇晃另加符号。
 *
 *   dbg.fluid.ax10 / ay10 是**送进场景后**的值（模拟单位 ×10），
 *   可以直接确认映射对不对：正着放时应为 (0, +98) 即"向下"。
 *
 * FLUID_USE_IMU = 0：退回"假重力"（每 DEMO_TILT_MS 换一个方向）。
 *   用于 IMU 没焊 / 没通 / 想对比效果时。IMU 初始化失败会自动走这条路。
 *
 * ==================== 数值处理 ====================
 *   原始读数有噪声，直接喂物理会让液体抖。这里做三件事：
 *     ① 死区：|读数| 小于阈值时当 0，避免静止时液体自己慢慢漂
 *     ② 一阶低通：α 越小越平滑但越迟钝
 *     ③ 双通路分解：把读数按频率拆成"慢 = 重力/倾角"和"快 = 摇晃/惯性"，
 *        于是"摇一摇液体跟着晃"和"静止不抖"可以同时成立 —— 详见下面的
 *        《重力 / 摇晃：双通路分解》
 *   都是 Q24 定点运算，不引入软浮点。
 */

#include "ti_msp_dl_config.h"
#include "clock.h"
#include "debug.h"
#include "app.h"
#include "scenes.h"
#include "battery.h"
#include "fluid_demo.h"         /* FluidDemo_IqSelfTest */
#include "led_matrix.h"
#include "ht16d33a.h"
#include "key.h"
#include "lsm6ds3trc.h"
#include "power.h"
#include "vcmd.h"
#include "vofa.h"
#include "glyph_store.h"
#include "flash_store.h"
#include "asset_route.h"
#include "brake.h"              /* 刹车高亮：检测 + 接管显示 */

/* ==================== 配置 ==================== */

/* 1 = 用真实 IMU 的加速度计驱动重力；0 = 用假重力（编译期强制退回）*/
#define APP_USE_IMU         1

/* IMU 初始化失败时是否退回假重力：1 = 退回（程序照常跑），0 = 整屏点亮停住 */
#define APP_IMU_FALLBACK    1

/* 选轴：0 = AX, 1 = AY, 2 = AZ
 * 符号：+1 或 -1
 *
 * 本板实测（IMU 与屏同向竖装）：把"喂进去的值"当 (ax, ay) 看，三种姿态下
 *   屏幕正着放    → 液体落在【左】(-1, 0)，应为【下】(0,+1)
 *   顺时针 90°    → 液体落在【上】(0,-1)，应为【右】(+1,0)
 *   顺时针 180°   → 液体落在【右】(+1,0)，应为【上】(0,-1)
 * 三点解出唯一线性映射：
 *   ax_屏 = -ay_IMU
 *   ay_屏 = -ax_IMU
 * 即【两轴对调 + 都取负】。
 *
 * ⚠ 注意这个映射矩阵行列式 = -1，是"旋转 + 镜像"而不是纯旋转
 *   （纯旋转行列式应为 +1）。也就是说本板 IMU 的 X-Y 平面相对屏幕是**镜像**的，
 *   常见成因：芯片贴在 PCB 背面、或封装轴向与屏幕轴向手性相反。
 *   这不影响使用，但换板/换贴装方向后必须重新用上面三点法标定。 */
#define APP_IMU_AX_SRC      1     /* 屏幕 x 取 AY */
#define APP_IMU_AY_SRC      0     /* 屏幕 y 取 AX */
#define APP_IMU_AX_SIGN     (+1)
#define APP_IMU_AY_SIGN     (-1)

/* ⚠⚠ 上面 AX_SIGN 的符号和 led_matrix.h 的 LED_ORIENT_FLIP_Y 是**绑死的**。
 *
 * 为什么：这套轴向映射是用"把板子摆三个姿势、看液体落在哪边"标出来的，
 * 参照物是**屏幕**。屏幕要是左右镜像了，标出来的映射就把镜像也吸收进去了 ——
 * 于是液体流向是对的，但应用 x 轴相对物理方向是反的（文字暴露了这一点）。
 *
 * 2026-10-24 把 FLIP_Y 改成 1（修文字镜像）的同时，这里 AX_SIGN 从 -1 翻成 +1，
 * 抵消掉 x 轴反向，液体流向保持不变。
 *
 * 以后只改其中一个 → 文字或液体必有一个是反的：
 *   只改 FLIP_Y 不改这里   → 文字正常了，但液体左右流反
 *   只改这里不改 FLIP_Y   → 液体正常了，但文字还是镜像
 *
 * 竖轴同理：如果哪天把 FLIP_X 改成 1，AY_SIGN 也要跟着翻。 */

/* ==================== 重力 / 摇晃：双通路分解 ====================
 *
 * 问题：一个低通没法同时满足两件事 ——
 *   静态平滑（α 小）：静止 / 慢倾斜时液体不抖，但手摇 1~3Hz 的左右晃动会被
 *                     衰减到只剩两三成，液体只是"慢慢歪过去"，不"晃"；
 *   动态跟手（α 大）：摇起来有反应，但静止时噪声直通，液体表面自己抖。
 *
 * 解法：把加速度计读数按频率拆成两条通路。加速度计测的是**比力**，本身就等于
 *       "重力反作用 + 自身运动加速度"，所以两条通路就是同一次读数的分解：
 *
 *                     ┌─ 死区 40mg → 慢通路低通 α_slow ─────────────────┐
 *   raw ─(选轴/符号)──┤                                                  ├→ 相加 → 限幅
 *                     └─ (raw − 慢) 高通 → α_fast 低通 → 死区 90mg ──────┘（× 增益）
 *
 *   · 慢通路 = 原来的行为，**逐位不变**（同一个 α、同一个死区）⇒ 静态手感不会
 *     和以前有任何差别
 *   · 快通路 = 读数里"变化的那部分"。静止时它恒为 0（增益再大也不起作用），
 *     一摇就出来 ⇒ 摇晃强度可以独立调，不用拿静态平滑去换
 *   · 高通天然"回中"：停止摇晃后快通路自己衰减到 0，只剩重力，
 *     不需要任何状态机 / 计时器
 *
 * ⚠ 这里**有两个死区，作用不同，不能合并**：
 *
 *   APP_IMU_DEADBAND_MG (40mg)：**慢通路输入**的门槛，小于它就当 0，避免静止
 *     时液体自己慢慢漂。但它只在"信号本身很小"时才管用 —— 板子有较大静态倾斜时
 *     （比如某轴持续 800mg），骑在上面的 ±5~10mg 噪声是**穿过**这个死区进到后面去的。
 *
 *   APP_SHAKE_DEADBAND (90mg)：**快通路输出后**的门槛。只有它能挡住两种假摇晃：
 *     ① 倾斜态下的噪声（快通路把它看成"变化"）；
 *     ② 信号压在 40mg 死区边界上时，输入会在 0 和 40mg 之间反复跳，快通路会把这
 *        种方波当成摇晃。
 *   ⇒ 所以快通路的输入必须用**未加死区的原始值**（避开②），门槛留给输出端。
 */

/* 死区（mg）：小于这个值当 0。静止时加速度计噪声约 ±10mg，取 40 足够稳。
 * ⚠ 这是**慢通路（重力）**的门槛，只作用于慢通路 */
#define APP_IMU_DEADBAND_MG     40

/* ---- 慢通路：重力 / 倾角 ---- */
/* 一阶低通系数 α（Q24）。越小越平滑、越迟钝。
 * 0.10 ≈ 时间常数 10 帧；跟手但不抖。
 * ⚠ 现在有快通路了，这里**不要再为了摇晃去调大** —— 调大只会让静止的液体跟着
 *   噪声抖，摇晃那一半应该交给 APP_SHAKE_ALPHA / APP_SHAKE_GAIN */
#define APP_IMU_LPF_ALPHA       _IQ(0.10f)

/* ---- 快通路：摇晃 / 惯性 ---- */
/* 快通路的低通系数 α（Q24），只负责吃掉高通残留的噪声。
 * 0.60 ≈ 时间常数 1.7 帧（fps≈30 时截止 ≈2.9Hz）：2Hz 的手摇通过 ~0.82。
 * 调大更跟手也更毛糙，调小更稳但摇不动。 */
#define APP_SHAKE_ALPHA         _IQ(0.60f)

/* 摇晃增益（Q24）。1.0 = 物理正确（摇晃 0.3g 就是 30% 的重力）。
 * 上面那条低通会吃掉一部分幅度，所以默认给 1.5。串口 `shake <n×100>` 可改。 */
#define APP_SHAKE_GAIN          _IQ(1.5f)

/* 摇晃死区（模拟单位）：90mg = 90 × 9.8/1000 = 0.882。
 * 想摇得更灵敏就往下调（60mg 也还能用），想更稳就往上调（120mg）。 */
#define APP_SHAKE_DEADBAND      _IQ(0.882f)

/* 复合之后的限幅（模拟单位，9.8 = 1g）。防"磕桌子"那种单帧尖峰把液体甩到边上。
 * 想保留最猛的冲击感可以调到 _IQ(39.2f)（= 4g，等于不限）。 */
#define APP_ACC_CLAMP           _IQ(24.5f)

/* 假重力方向保持多久（ms）：4 个方向 下→左→上→右 循环 */
#define APP_TILT_MS             2000u

/* 加速度计 1g 对应的 mg 值（换算成模拟单位 9.8 用）*/
#define APP_MG_PER_G            1000

/* 小场景共用的像素缓冲。定义放这里，声明在 app.h（见那里的规矩）。 */
app_scratch_t s_scratch;

/* ==================== 场景表 ====================
 *
 * 顺序即场景号（串口 `mode <n>` 用这个编号）。加场景 = 加一行 + 改 scenes.h。
 * 函数指针 4 字节 × 4 个 × N，全在 Flash，RAM 零开销。 */
static const scene_t s_scenes[] = {
    /* 0 */ { FluidScene_Enter, FluidScene_Tick, FluidScene_Exit, FluidScene_Cmd },
    /* 1 */ { StillScene_Enter, StillScene_Tick, StillScene_Exit, StillScene_Cmd },
    /* 2 */ { TextScene_Enter,  TextScene_Tick,  TextScene_Exit,  TextScene_Cmd  },
    /* 3 */ { FrameScene_Enter, FrameScene_Tick, FrameScene_Exit, 0             },
    /* 4 */ { GeomScene_Enter,  GeomScene_Tick,  GeomScene_Exit,  GeomScene_Cmd },
};

#define SCENE_COUNT     ((uint8_t) (sizeof(s_scenes) / sizeof(s_scenes[0])))

static uint8_t  s_cur;              /* 当前场景号 */
static uint8_t  s_started;          /* 0 = 还没进过任何场景（首次不进 exit）*/
static uint32_t s_lastMs;           /* 上一帧的 tick_ms，用来算 dt */

/* ==================== 全局亮度 / 电量 / 省电 ====================
 *
 * 亮度分两层，**别混**：
 *
 *   s_bcUser  用户设定。长按 USER3 / 串口 `bc` / 开机从 Flash 恢复都写这里。
 *             ← **只有这个进 Flash**。存生效值的话，省电期间存下 128，
 *               下次开机用户设定就永远回不到 255 了（这是个真坑）。
 *   s_bcEff   实际写进 0x37 的值 = 省电模式对 s_bcUser 封顶后的结果。
 *             ← 所有"恢复亮度"的地方（安静窗口、刹车退出）都用
 *               App_Brightness()，它返回这个值，于是省电状态自动被尊重。
 *
 * 省电模式只做一件事：**给全局亮度加一个上限**，不碰任何场景的灰度表。
 * 所以逐点之间的亮度比完全不变，画面结构不受影响，只是整体压暗 ——
 * 这也意味着它**不能**改善"中心/边缘对比度"，那是灰度表的事。
 */
#define APP_BC_LADDER_N      (4u)

/* 亮度档位：暗 → 亮。和长按 USER3 的循环顺序一致 */
static const uint8_t s_bcLadder[APP_BC_LADDER_N] = { 64u, 128u, 192u, 255u };

/* 省电模式的亮度上限（= 档位第二档，满亮的一半）。
 * 为什么是"封顶"而不是"降两档"：降两档的话用户设 192 会变 64、设 255 会
 * 变 128，同一个省电模式在不同档位下省的电不一样，行为不可预期。
 * 封顶就是"最亮只能到 128"，省多少是确定的。
 * 用户主动把亮度调到 64 仍然有效 —— 想更暗还能更暗。 */
#define APP_PSAVE_CAP        (128u)

/* 电量阈值(%)：低于 ON 进入省电，高于 OFF 退出。
 * 两个阈值都落在 3650~3800 这一段（斜率 25%/150 mV），所以 5% 迟滞 = 30 mV。
 * 比 BAT_LOW_HYST_MV(100 mV) 窄，但这里用的是 3 次中值滤波后的**空载**采样，
 * 噪声在 ±10 mV 量级，30 mV 够用。 */
#define APP_PSAVE_PCT_ON     (30u)
#define APP_PSAVE_PCT_OFF    (35u)

/* 空载电压 → 电量(%)。锂电放电曲线的 5 段折线。
 *
 * ⚠ 必须用**空载**读数：带载读数会被压低 200~335 mV（battery.h §③）。
 *   同一颗 3.8 V 的电池带载读成 3.47 V，按这张表直接从 40% 掉到 0% ——
 *   误差不是几个百分点，是整段垮掉。这也是安静窗口非做不可的原因。
 *
 *   4150 → 100%    4000 → 70%    3800 → 40%    3650 → 15%    3500 → 0%
 *
 * 平台区（3.7~3.9 V）电压变化小、电量变化大，所以这一段斜率必须平，
 * 否则会看到"电量从 45% 直接掉到 20%"。整体误差仍有 ±10~15% ——
 * 这是单节锂电靠电压估电量的物理极限，不是实现问题。
 * 低于 3450 mV 就进了读数地板（battery.h §②），测不出来，一律当 0。 */
static uint8_t App_BatPercent(uint32_t mv)
{
    static const uint16_t v[5] = { 3500u, 3650u, 3800u, 4000u, 4150u };
    static const uint8_t  p[5] = {    0u,   15u,   40u,   70u,  100u };
    uint8_t i;

    if (mv >= (uint32_t) v[4]) {
        return p[4];
    }
    if (mv <= (uint32_t) v[0]) {
        return 0u;
    }

    for (i = 0u; i < 4u; i++) {
        if (mv < (uint32_t) v[i + 1u]) {
            uint32_t num = (uint32_t) (mv - v[i]) * (uint32_t) (p[i + 1u] - p[i]);

            return (uint8_t) ((uint32_t) p[i] + num / (uint32_t) (v[i + 1u] - v[i]));
        }
    }
    return p[4];
}

/* 用户设定（进 Flash）。低电安静窗口熄屏采样后恢复的是"生效值"，不是这个。 */
static uint8_t  s_bcUser = HT16D33A_BRIGHTNESS_MAX;

/* 实际写进 0x37 的值（省电封顶之后）*/
static uint8_t  s_bcEff  = HT16D33A_BRIGHTNESS_MAX;

static uint8_t  s_psave;        /* 1 = 省电中（亮度封顶）*/
static uint8_t  s_psaveVeto;    /* 1 = 用户手动调过亮度，别再自动降 */

/* 调试强制（串口 `psave`）：0 = 自动，1 = 强制开，2 = 强制关。
 * 和 veto 的区别：veto 是"用户否决"，电量回到 APP_PSAVE_PCT_OFF 以上会
 * 自动失效；force 不会 —— 专门用来在没有低电电池的情况下验降档效果，
 * 不然命令发下去 2 秒就被下一次安静窗口的自动判定推翻了。 */
static uint8_t  s_psaveForce;

/* 按"用户设定 + 省电状态"算出实际亮度并写芯片。
 * ⚠ 这里**不碰 Flash**（不调 route_mark_dirty）：省电是自动行为，
 *   电量在 30% 附近抖动时如果每次都标脏，会反复擦写扇区。
 *   用户主动改亮度才标脏，那条路径在调用方。 */
static void app_bright_apply(void)
{
    uint8_t v = s_bcUser;

    if ((s_psave != 0u) && (v > APP_PSAVE_CAP)) {
        v = APP_PSAVE_CAP;
    }
    s_bcEff = v;
    HT16D33A_SetBrightness(s_bcEff);
}

/* 按最近一次**空载**读数更新省电状态。只改标志、不写芯片 ——
 * 调用方紧接着会调 app_bright_apply()，避免连写两次 0x37。 */
static void app_psave_update(uint32_t mv)
{
    uint8_t pct;

    if (s_psaveForce != 0u) {
        return;                     /* 被串口 `psave` 强制，自动判定不介入 */
    }
    if (mv == 0u) {
        return;                     /* 读不到就保持原状，别乱切 */
    }
    pct = App_BatPercent(mv);

    if (s_psave == 0u) {
        if (pct >= APP_PSAVE_PCT_OFF) {
            /* 电量回到上限以上，否决权失效 —— 否则用户否决一次就永久
             * 不再进省电了（踩过：veto 只在"省电中"那条分支里清，
             * 而否决之后 psave 恒为 0，那条分支永远走不到）。 */
            s_psaveVeto = 0u;
        } else if ((s_psaveVeto == 0u) && (pct <= APP_PSAVE_PCT_ON)) {
            s_psave = 1u;
        }
    } else if (pct >= APP_PSAVE_PCT_OFF) {
        s_psave     = 0u;
        s_psaveVeto = 0u;
    }
}

/* ==================== 输入层状态 ==================== */

static uint8_t  s_useImu;                                   /* 本机 IMU 是否可用 */
static uint32_t s_tiltStart;                                /* 假重力换向计时 */
static uint8_t  s_tilt;                                     /* 假重力当前方向 */

/* 假重力的 4 个方向（模拟单位）。约定：x 正 = 屏幕右，y 正 = 屏幕下 */
static const _iq s_tiltAcc[4][2] = {
    { _IQ(0.0f),  _IQ(9.8f) },      /* 0 下 */
    { _IQ(-9.8f), _IQ(0.0f) },      /* 1 左 */
    { _IQ(0.0f),  _IQ(-9.8f) },     /* 2 上 */
    { _IQ(9.8f),  _IQ(0.0f) },      /* 3 右 */
};

/* 慢通路状态（重力/倾角的低通输出，Q24，单位：模拟单位）*/
static _iq      s_axSlow;
static _iq      s_aySlow;

/* 快通路状态（摇晃分量，Q24。**不含增益**，增益在合成时乘）*/
static _iq      s_sxFast;
static _iq      s_syFast;

/* 滤波器是否已用第一次读数播种。不播种的话滤波器从 0 开始爬，
 * 开机头几百毫秒会先"自由落体"再稳住。 */
static uint8_t  s_filtSeeded;

/* 摇晃增益（Q24）。串口 `shake <n×100>` 改；1.0 = 物理正确。 */
static _iq      s_shakeGain = APP_SHAKE_GAIN;

/* ==================== 重力：工具函数 ==================== */

/* 死区（mg）：小于阈值的当 0。
 * ⚠ **只给慢通路（重力）用**：快通路的输入必须用未加死区的原始值，否则信号压在
 *   死区边界上时会 0/40mg 反复跳，那种方波会被快通路误当成摇晃。 */
static int32_t App_Deadband(int32_t mg)
{
    if ((mg > -APP_IMU_DEADBAND_MG) && (mg < APP_IMU_DEADBAND_MG)) {
        return 0;
    }

    return mg;
}

/* 从 dbg.phy 里取出指定轴、指定符号的加速度值（mg）。**不做死区** */
static int32_t App_PickAxis(uint8_t src, int8_t sign)
{
    int32_t v;

    switch (src) {
        case 0:  v = dbg.phy.ax_mg; break;
        case 1:  v = dbg.phy.ay_mg; break;
        case 2:  v = dbg.phy.az_mg; break;
        default: v = 0;             break;
    }

    return (sign < 0) ? -v : v;
}

/* mg → 模拟单位（9.8 对应 1000mg）。
 * result = mg * 9.8 / 1000，先乘后除避免整除丢精度。 */
static _iq App_MgToSim(int32_t mg)
{
    int64_t t = (int64_t) mg * (int64_t) _IQ(9.8f);

    t = t / APP_MG_PER_G;

    return (_iq) t;
}

/* 读一次 IMU：选轴 → 符号 → 双通路分解（慢=重力 / 快=摇晃）→ 相加 → 限幅。
 * 结果写进 in 的 ax/ay/ax_slow/ay_slow/shx/shy。 */
static void App_ReadGravity(app_in_t *in)
{
    int32_t rawXmg, rawYmg;
    _iq     slowInX, slowInY;
    _iq     totalX, totalY;

    /* 一次突发读拿六轴（约十几 µs），结果落在 dbg.phy / dbg.raw */
    LSM6DS3TRC_ReadAll();

    /* 选轴 + 符号。**这里不加死区** —— 死区只属于慢通路，见下 */
    rawXmg = App_PickAxis(APP_IMU_AX_SRC, APP_IMU_AX_SIGN);
    rawYmg = App_PickAxis(APP_IMU_AY_SRC, APP_IMU_AY_SIGN);

    /* 慢通路的输入 = 死区之后的值 */
    slowInX = App_MgToSim(App_Deadband(rawXmg));
    slowInY = App_MgToSim(App_Deadband(rawYmg));

    if (s_filtSeeded == 0u) {
        /* 第一次：慢通路直接赋当前值（避免从 0 慢慢爬上去，开机时液体乱冲一下）；
         * 快通路清零 —— 此刻还没有"变化"，别把开机瞬间当成摇晃 */
        s_axSlow     = slowInX;
        s_aySlow     = slowInY;
        s_sxFast     = 0;
        s_syFast     = 0;
        s_filtSeeded = 1u;
    } else {
        /* 慢通路：一阶低通 y[n] = y[n-1] + α·(x[n] − y[n-1]) */
        s_axSlow += _IQmpy(APP_IMU_LPF_ALPHA, slowInX - s_axSlow);
        s_aySlow += _IQmpy(APP_IMU_LPF_ALPHA, slowInY - s_aySlow);

        /* 快通路：先高通（raw − 慢，把"变化量"挑出来），再低通吃噪声。
         * ⚠ 高通用的是**未加死区的 raw**，这样信号压在 40mg 死区边界上时
         *   不会在快通路里造出假摇晃。 */
        s_sxFast += _IQmpy(APP_SHAKE_ALPHA,
                           (App_MgToSim(rawXmg) - s_axSlow) - s_sxFast);
        s_syFast += _IQmpy(APP_SHAKE_ALPHA,
                           (App_MgToSim(rawYmg) - s_aySlow) - s_syFast);
    }

    /* 摇晃死区：平滑完还是小值的当 0（挡"倾斜态下的噪声"）。
     * 这里清的是**状态**而不是只清输出，这样 dbg 里看到的 fast 就是"实际贡献了多少"，
     * 静止时读数为 0 一目了然。 */
    if ((s_sxFast > -APP_SHAKE_DEADBAND) && (s_sxFast < APP_SHAKE_DEADBAND)) {
        s_sxFast = 0;
    }
    if ((s_syFast > -APP_SHAKE_DEADBAND) && (s_syFast < APP_SHAKE_DEADBAND)) {
        s_syFast = 0;
    }

    /* 合成：重力 + 摇晃×增益，然后限幅（防磕碰尖峰）*/
    totalX = s_axSlow + _IQmpy(s_shakeGain, s_sxFast);
    totalY = s_aySlow + _IQmpy(s_shakeGain, s_syFast);

    if (totalX > APP_ACC_CLAMP) {
        totalX = APP_ACC_CLAMP;
    } else if (totalX < -APP_ACC_CLAMP) {
        totalX = -APP_ACC_CLAMP;
    }
    if (totalY > APP_ACC_CLAMP) {
        totalY = APP_ACC_CLAMP;
    } else if (totalY < -APP_ACC_CLAMP) {
        totalY = -APP_ACC_CLAMP;
    }

    /* 未滤波的原始值（**不加死区、不滤波**）：刹车检测要自己做 90 ms 低通，
     * 喂它双通路滤波后的值会把刹车的"持续性"糊掉。见 app.h 的字段说明。
     * AZ 是屏幕法线方向 —— 板子竖装时法线水平，正好是行进方向。 */
    in->ax_raw  = App_MgToSim(rawXmg);
    in->ay_raw  = App_MgToSim(rawYmg);
    in->az_raw  = App_MgToSim(App_PickAxis(2u, +1));

    in->ax      = totalX;
    in->ay      = totalY;
    in->ax_slow = s_axSlow;
    in->ay_slow = s_aySlow;
    in->shx     = _IQmpy(s_shakeGain, s_sxFast);
    in->shy     = _IQmpy(s_shakeGain, s_syFast);
}

/* ==================== 输入：每帧一次 ==================== */

static void App_ReadInput(app_in_t *in)
{
    uint32_t now = tick_ms;
    uint32_t dt  = now - s_lastMs;
    _iq      fx, fy;
    int32_t  mx, my;
    uint8_t  k;

    if (dt > 0xFFFFu) {
        dt = 0xFFFFu;                   /* 卡顿过就截断，别让场景看到离谱的 dt */
    }
    in->dt_ms  = (uint16_t) dt;
    in->now_ms = now;
    s_lastMs   = now;

    if (s_useImu != 0u) {
        App_ReadGravity(in);
    } else {
        /* ---- 假重力：每 APP_TILT_MS 换一个方向（IMU 不可用时的退路）---- */
        if ((uint32_t) (now - s_tiltStart) >= APP_TILT_MS) {
            s_tiltStart += APP_TILT_MS;
            s_tilt = (uint8_t) ((s_tilt + 1u) & 3u);
            dbg.fluid.tilt = s_tilt;
        }

        in->ax      = s_tiltAcc[s_tilt][0];
        in->ay      = s_tiltAcc[s_tilt][1];
        in->ax_slow = in->ax;
        in->ay_slow = in->ay;

        /* 没有 IMU 就没有真实的加速度：raw 清零，刹车检测会自动退出
         * （brake.c 里判 imu_use）。不能让假重力顶上来，否则每 2 s
         * 换一次方向会把刹车灯点得乱七八糟。 */
        in->ax_raw  = 0;
        in->ay_raw  = 0;
        in->az_raw  = 0;
        in->shx     = 0;
        in->shy     = 0;
    }

    /* 按键边沿。两种边沿都要每帧读掉 —— 读后自动清零，少读一帧那个边沿就丢了。
     *
     * ⚠ 切场景用的是**松发**：in->key 里装的是"松开"边沿（Key_TookRelease）。
     *   这样 USER1 长按 2 s 关机时不会顺手把大类也切了 —— 长按期间没有松开事件，
     *   关机动作先发生，松手时板子已经断电。短按则是在松手那一刻切。
     *   "按下"边沿这里照样读掉（丢弃），免得残留给 led_show 之类的其它消费者。 */
    in->key = 0u;
    for (k = 0u; k < (uint8_t) KEY_NUM; k++) {
        (void) Key_TookPress((key_id_t) k);
        if (Key_TookRelease((key_id_t) k) != 0u) {
            in->key = (uint8_t) (in->key | APP_KEY_BIT(k));
        }
    }

    /* 摇晃幅度（×10，取绝对值，截断到 255）—— 供场景做手势判断。
     * 注意这是**瞬时值**，窗口峰值由场景自己累计（fluid 场景要写 dbg.fluid.shake_pk10）。 */
    fx = in->shx;
    fy = in->shy;
    mx = (int32_t) (((int64_t) ((fx < 0) ? -fx : fx) * 10) / IQ_ONE);
    my = (int32_t) (((int64_t) ((fy < 0) ? -fy : fy) * 10) / IQ_ONE);
    if (my > mx) {
        mx = my;
    }
    in->shake10 = (uint8_t) ((mx > 255) ? 255 : mx);
}

/* ==================== 场景切换 ==================== */

/* 上位机直显：存下 240 B 像素并切到"单帧画面"场景。
 * 由 vcmd.c 的二进制包解析调用（0xA5 开头的那套协议）。 */
void App_ShowDirectFrame(const uint8_t *px)
{
    FrameScene_SetPixels(px);
    App_Goto(3u);
}

/* 上位机字形条带：切到文字场景，再用发来的字形把条带覆盖掉。
 * 顺序很重要 —— App_Goto 会触发 TextScene_Enter() → Restart()，
 * 那里可能用内置字库重画一次；所以必须**先切场景、后拼条带**。 */
uint8_t App_ShowGlyphs(const uint8_t *p, uint16_t len, uint8_t count, int8_t spacing)
{
    App_Goto(2u);
    return TextScene_ComposeGlyphs(p, len, count, spacing);
}

/* 上位机条带（整条长图）：切到文字场景后把条带灌进去，由 MCU 自己滚。
 * 顺序同 App_ShowGlyphs：先切场景（Enter 会重画一次），后覆盖条带。 */
uint8_t App_ShowStrip(const uint8_t *bits, uint16_t cols)
{
    App_Goto(2u);
    return TextScene_LoadStrip(bits, cols);
}

/* 把 Flash 里存的自定义滚动消息应用到文字场景。
 * 上电加载后、以及串口收完一整轮数据后调用。 */
uint8_t App_ApplyStoredGlyphs(void)
{
    const glyph_meta_t *m = GlyphStore_Meta();

    if ((GlyphStore_HasData() == 0u) || (GlyphStore_DataLen() == 0u)) {
        return 0u;
    }

    TextScene_SetScroll(m->step, m->ms, m->lev);

    return App_ShowGlyphs(GlyphStore_Data(), GlyphStore_DataLen(),
                          m->count, m->spacing);
}

void App_Goto(uint8_t id)
{
    if (id >= SCENE_COUNT) {
        id = 0u;
    }
    if ((s_started != 0u) && (id == s_cur)) {
        return;                         /* 已经是它，不重入 enter/exit */
    }

    if ((s_started != 0u) && (s_scenes[s_cur].exit != 0)) {
        s_scenes[s_cur].exit();
    }

    /* ★ 切换的语义边界：清屏 + 一次 Flush。
     *   DDRAM 是持久的，不清的话上一个场景没覆盖到的格子会一直亮着，
     *   看起来像"残影 bug"，实际只是没人擦过。 */
    LedMatrix_Clear();
    LedMatrix_Flush();

    s_cur     = id;
    s_started = 1u;

    if (s_scenes[s_cur].enter != 0) {
        s_scenes[s_cur].enter();
    }

    dbg.app.scene = s_cur;
    dbg.app.switches++;

    /* 刹车期间切了场景：新场景的 enter() 已经画好了，
     * 让刹车层重新快照，退出时才不会还原成旧场景的画面。 */
    Brake_NotifySceneChanged();
}

void App_Step(int8_t d)
{
    int16_t n = (int16_t) s_cur + (int16_t) d;

    /* 循环：0 的上一个 = 最后一个 */
    while (n < 0) {
        n = (int16_t) (n + SCENE_COUNT);
    }
    while (n >= (int16_t) SCENE_COUNT) {
        n = (int16_t) (n - SCENE_COUNT);
    }

    App_Goto((uint8_t) n);
}

uint8_t App_Current(void) { return s_cur; }
uint8_t App_Count(void)   { return SCENE_COUNT; }

/* 用户主动改亮度（长按 / 串口 bc / 开机从 Flash 恢复）。
 * 只记"用户设定"，实际写进 0x37 的是封顶后的值 —— 见 app_bright_apply()。 */
void App_SetBrightness(uint8_t bc)
{
    s_bcUser = bc;
    app_bright_apply();
}

/* 显示层用这个：已经算上省电封顶。安静窗口 / 刹车退出恢复亮度都走它 */
uint8_t App_Brightness(void) { return s_bcEff; }

/* 存 Flash 用这个：必须存用户设定，不能存封顶后的值 */
uint8_t App_BrightnessUser(void) { return s_bcUser; }

/* ==================== 两级路由（大类 × 小类）====================
 *
 * 按键绑 (大类, 小类)，再由 route_apply() 解析成"进哪个场景"。
 * 这样加内容不用动按键逻辑，也不会搅乱 s_scenes[] 的顺序（= mode 号）。
 * 见 多资源与两级按键方案.md §4。
 */

/* 状态脏了以后空闲这么久才写 Flash —— 擦一个扇区约 20 ms，
 * 期间取指被阻塞、动画会卡一帧，所以不能每按一次键就写。 */
#define ROUTE_SAVE_DELAY_MS     (2000u)

static uint8_t  s_cat = (uint8_t) ROUTE_CAT_FLUID;
static uint8_t  s_item[ROUTE_CAT_COUNT];
static uint8_t  s_dirty;
static uint32_t s_dirtyMs;

/* 每个大类有几项。
 *
 * ⚠ 阶段 1 是**占位值**：文字/图像各算 1 项（当前只有"最近一次上传或内置"那一份），
 *   几何动画 0 项（还没有场景）→ 按大类时会跳过它。
 *   阶段 2 把 TEXT 改成"读文字预设区有效条数"，
 *   阶段 3 把 IMAGE 改成"数有几个有效动图槽头"。 */
static uint8_t route_item_count(uint8_t cat)
{
    switch ((route_cat_t) cat) {
        case ROUTE_CAT_FLUID:
            return 1u;

        case ROUTE_CAT_TEXT:
            /* 第 0 项 = 原来的行为（TextScene_Enter 自己决定：有存 Flash 的自定义
             * 字形就用它，否则用内置字库的当前文字），第 1 项起是文字预设。
             * 这样"用上位机存一条条带"的老用法不会被新加的预设挤掉。 */
            return (uint8_t) (1u + TextScene_StoreCount());

        case ROUTE_CAT_IMAGE:
            /* 第 0 项 = 扩散环（内置，恒定有），第 1 项起 = Flash 里的图/动图槽。
             * 上位机直显那一帧不再占一项：上传时会自动 App_Goto(3) 切过去，
             * 它本身又掉电就丢，不适合放在"图像库"里。 */
            return (uint8_t) (1u + FrameScene_SlotCount());

        case ROUTE_CAT_GEOM:
            /* 项数 = 动画表 s_anims[] 的长度（scene_still.c）。
             * 加动画只改那张表，这里不用动 —— 不要再写死数字。 */
            return GeomScene_AnimCount();

        default:
            return 0u;
    }
}

uint8_t Route_ItemCount(uint8_t cat) { return route_item_count(cat); }
uint8_t Route_Cat(void)  { return s_cat; }
uint8_t Route_Item(void) { return s_item[s_cat]; }

/* 把当前 (cat, item) 应用到显示。
 * 必须走 App_Goto()：它负责 exit → 清屏 → Flush → enter，跳过会留残影。 */
static void route_apply(void)
{
    switch ((route_cat_t) s_cat) {
        case ROUTE_CAT_FLUID:
            App_Goto(0u);
            break;

        case ROUTE_CAT_TEXT:
            /* 顺序不能反：先进场景（Enter 会用字库/字形重画），再套预设。
             * StoreApply() 内部会 Restart()，先套的话会被 Enter 覆盖掉。
             * item 0 不做任何事 —— 保持 Enter 的行为（存 Flash 的字形 / 内置字库）。 */
            App_Goto(2u);
            if (s_item[ROUTE_CAT_TEXT] >= 1u) {
                (void) TextScene_StoreApply((uint8_t) (s_item[ROUTE_CAT_TEXT] - 1u));
            }
            break;

        case ROUTE_CAT_IMAGE:
            if (s_item[ROUTE_CAT_IMAGE] == 0u) {
                /* 第一项：扩散环。它属于几何场景（场景 4），所以这里要跨场景。
                 * 顺序：先 Select 再 Goto —— Goto 会触发 Enter，Enter 里按 anim 初始化。 */
                GeomScene_Select(2u);
                App_Goto(4u);
            } else {
                App_Goto(3u);
                (void) FrameScene_PlaySlot((uint8_t) (s_item[ROUTE_CAT_IMAGE] - 1u));
            }
            break;

        case ROUTE_CAT_GEOM:
            GeomScene_Select(s_item[ROUTE_CAT_GEOM]);
            App_Goto(4u);
            break;

        default:
            break;
    }

    dbg.app.cat  = s_cat;
    dbg.app.item = s_item[s_cat];
}

static void route_mark_dirty(void)
{
    s_dirty   = 1u;
    s_dirtyMs = tick_ms;
}

/* 把当前状态写进设置记录。失败不算致命：保持脏，下个窗口再试。 */
static void route_save(void)
{
    flash_settings_t st;
    uint8_t i;

    st.cat = s_cat;
    for (i = 0u; i < (uint8_t) ROUTE_CAT_COUNT; i++) {
        st.item[i] = s_item[i];
    }
    /* 存**用户设定**，不是 App_Brightness()（那是省电封顶后的生效值）。
     * 存错的话，省电期间保存一次，下次开机用户设定就永久变成 128 了。 */
    st.bc = App_BrightnessUser();

    if (FlashStore_SettingsSave(&st) != 0u) {
        s_dirty = 0u;
    } else {
        s_dirtyMs = tick_ms;
    }
}

/* 从 cat 出发按 d 方向找下一个非空大类；全都空就返回原值 */
static uint8_t route_step_cat(uint8_t cat, int8_t d)
{
    uint8_t n = (uint8_t) ROUTE_CAT_COUNT;
    uint8_t i;

    for (i = 0u; i < n; i++) {
        cat = (uint8_t) ((d > 0) ? ((cat + 1u) % n) : ((cat + n - 1u) % n));
        if (route_item_count(cat) != 0u) {
            return cat;
        }
    }

    return cat;
}

void Route_Init(void)
{
    flash_settings_t st;
    uint8_t i;

    for (i = 0u; i < (uint8_t) ROUTE_CAT_COUNT; i++) {
        s_item[i] = 0u;
    }
    s_cat   = (uint8_t) ROUTE_CAT_FLUID;
    s_dirty = 0u;

    if (FlashStore_SettingsLoad(&st) != 0u) {
        if (st.cat < (uint8_t) ROUTE_CAT_COUNT) {
            s_cat = st.cat;
        }
        /* 小类要按当前项数取模：项数变少时不能让下标越界 */
        for (i = 0u; i < (uint8_t) ROUTE_CAT_COUNT; i++) {
            uint8_t n = route_item_count(i);

            s_item[i] = (n == 0u) ? 0u : (uint8_t) (st.item[i] % n);
        }

        /* 恢复全局亮度。0 表示"没设置过/用户关过"，那就保持默认最大值 ——
         * 上电就是黑屏太难判断是坏还是关机。 */
        if (st.bc != 0u) {
            App_SetBrightness(st.bc);
        }
    }

    /* 恢复出来的大类可能已经空了（比如几何动画还没做）→ 顺延 */
    if (route_item_count(s_cat) == 0u) {
        s_cat = route_step_cat(s_cat, +1);
    }

    dbg.app.cat  = s_cat;
    dbg.app.item = s_item[s_cat];
}

void Route_Boot(void)
{
    route_apply();
}

void Route_KeyCat(void)
{
    s_cat = route_step_cat(s_cat, +1);
    route_apply();
    route_mark_dirty();
}

void Route_KeyItem(void)
{
    uint8_t n = route_item_count(s_cat);

    if (n <= 1u) {
        return;                             /* 只有一项：按键无效，不是故障 */
    }

    s_item[s_cat] = (uint8_t) ((s_item[s_cat] + 1u) % n);
    route_apply();
    route_mark_dirty();
}

void Route_Tick(void)
{
    if (s_dirty == 0u) {
        return;
    }

    /* ---- 长按关机的安全网 ----
     *
     * PowerKey 在 USER1 按住 POWER_OFF_HOLD_MS(2000 ms) 后释放自锁，随后主循环会
     * 停在 PowerKey_CheckShutdown() 里等硬件断电 —— **之后再也不会跑到这里**。
     * 所以"按一次键切了类、2 s 内立刻长按关机"这种情况，下面那条 2 s 空闲策略
     * 永远等不到，状态就丢了。这里在按住满 1 s 时提前落盘，留 1 s 余量。
     *
     * 短按（约 100 ms）够不到 1 s，所以不会因为每次按键都擦一次扇区。 */
    if ((Key_IsDown(KEY_USER1) != 0u) && ((tick_ms - s_dirtyMs) >= 1000u)) {
        route_save();
        return;
    }

    if ((tick_ms - s_dirtyMs) < ROUTE_SAVE_DELAY_MS) {
        return;
    }

    route_save();
}

/* ==================== 低电关机动作 ====================
 *
 * battery.c 只负责"判断低电"，具体怎么收场交给这里。
 * 现在 = **熄屏 → 立即关机**（不播动画）。
 *
 * 以后要做"先播一段低电动画再关机"，把函数体换成：
 *     LedShow_PlayLowBattery();     // 播完自动返回
 *     Battery_ShutdownNow();
 * 即可，其它地方一行都不用改（接口见 battery.h）。
 *
 * ⚠ 熄屏这一步不能省：电源自锁释放后，整机还要靠保持电容撑一小会儿，
 *   不熄屏的话最后那一帧会冻在屏上。 */
/* 关机动画定义在后面（和开机电量画面共用绘制函数），这里先声明 */
static void App_ShutdownAnim(void);

static void App_LowBatteryAction(void)
{
    /* 低电确认（连续 3 次 < BAT_LOW_MV）后先播一段关机动画再断电。
     * 约 1.1 s，期间按键可跳过 —— 跳过也照样关机。
     * ⚠ 必须在 ShutdownNow() 之前：那之后是 for(;;) __WFI()，没有代码再执行了。 */
    App_ShutdownAnim();

    Battery_ShutdownNow();          /* 释放电源自锁；不返回 */
}

/* ==================== 命令分发 ====================
 *
 * vcmd.c 只负责把文本翻译成"命令号 + 4 个参数"投进 dbg.cmd 邮箱；
 * 真正的动作在这里、在主循环的固定位置执行。这样命令生效的时机可预测，
 * 不会在 SPI 推屏中途改显示状态。
 *
 * 分工：
 *   全局（本文件）：遥测开关、统计清零、亮度/恒流、摇晃增益、场景切换
 *   场景（scene->cmd）：各场景自己的参数和绘制命令
 */
void App_CmdTask(void)
{
    uint16_t id;
    int32_t  arg[4];
    uint8_t  i;

    if (dbg.cmd.pending == 0u) {
        return;
    }

    id = dbg.cmd.id;
    for (i = 0u; i < 4u; i++) {
        arg[i] = dbg.cmd.arg[i];
    }

    dbg.cmd.pending = 0u;
    dbg.cmd.result  = 1u;

    switch (id) {
        /* ---------------- 遥测 ---------------- */
        case DBG_CMD_SET_DIV:
            VOFA_SetDivShift((uint32_t) arg[0]);
            break;

        case DBG_CMD_SET_MASK:
            VOFA_SetMask((uint32_t) arg[0]);
            break;

        case DBG_CMD_RESET_STATS:
            dbg.uart.rx_bytes  = 0u;
            dbg.uart.cmds      = 0u;
            dbg.uart.bad       = 0u;
            dbg.uart.tx_frames = 0u;
            dbg.uart.tx_drops  = 0u;
            dbg.uart.tx_stuck  = 0u;
            break;

        /* ---------------- Flash 自检 ----------------
         * 用一个**专门的测试扇区**（数据区第 3 个扇区，0x10C00），
         * 不碰 0x10000（头部）和 0x10400（字形数据），所以随时可以跑。
         * 擦 → 写 32 字节图案 → 读回比对，结果进 dbg.flash。 */
        case DBG_CMD_FLASH_TEST:
            {
                static const uint8_t pat[32] = {
                    0x00u, 0x11u, 0x22u, 0x33u, 0x44u, 0x55u, 0x66u, 0x77u,
                    0x88u, 0x99u, 0xAAu, 0xBBu, 0xCCu, 0xDDu, 0xEEu, 0xFFu,
                    0x5Au, 0xA5u, 0x0Fu, 0xF0u, 0x3Cu, 0xC3u, 0x01u, 0x80u,
                    0x7Fu, 0xFEu, 0x12u, 0x34u, 0x56u, 0x78u, 0x9Au, 0xBCu
                };
                /* ⚠ 必须是专用测试区：这里会擦 1 个扇区。
                 * 以前写的是 FLASH_STORE_BASE + 3 扇区，基底搬到 0x18000 后
                 * 那个地址正好是文字预设区 —— 一按 ftest 就擦掉预设。 */
                uint32_t addr = FLASH_REG_TEST;

                dbg.flash.runs++;

                dbg.flash.erase_ok  = FlashStore_Erase(addr, 1u);
                dbg.flash.write_ok  = (dbg.flash.erase_ok != 0u)
                                      ? FlashStore_Write(addr, pat, 32u) : 0u;
                dbg.flash.verify_ok = (dbg.flash.write_ok != 0u)
                                      ? FlashStore_Verify(addr, pat, 32u) : 0u;
                dbg.flash.last_ok   = dbg.flash.verify_ok;
            }
            break;

        /* ---------------- 显示硬件（全局） ---------------- */
        case DBG_CMD_LED_SET_BC:
            if ((arg[0] >= 0) && (arg[0] <= 255)) {
                App_SetBrightness((uint8_t) arg[0]);
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        case DBG_CMD_LED_SET_CC:
            if ((arg[0] >= 0) && (arg[0] <= 15)) {
                HT16D33A_WriteCmd(HT16D33A_CMD_SET_CURRENT, (uint8_t) arg[0]);
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        /* ---------------- 输入层参数 ---------------- */
        case DBG_CMD_FLUID_SET_SHAKE:
            /* 参数是增益 ×100（150 = 1.5）。虽然编号还是 FLUID_ 前缀，
             * 但它现在属于输入层 —— 摇晃增益对所有场景都有意义。 */
            if ((arg[0] >= 0) && (arg[0] <= 1000)) {
                s_shakeGain = (_iq) (((int64_t) arg[0] * IQ_ONE) / 100);
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        /* ---------------- 刹车高亮 ---------------- */
        case DBG_CMD_BRAKE_ENABLE:
            Brake_SetEnabled((arg[0] != 0) ? 1u : 0u);
            break;

        case DBG_CMD_BRAKE_FORCE:
            Brake_SetForced((arg[0] != 0) ? 1u : 0u);
            break;

        case DBG_CMD_BRAKE_THR:
            if (Brake_SetThreshold((uint16_t) arg[0]) == 0u) {
                dbg.cmd.result = 2u;        /* 30~800 mg */
            }
            break;

        case DBG_CMD_BRAKE_AXIS:
            if ((arg[0] < 0) || (arg[0] > 2)) {
                dbg.cmd.result = 2u;
            } else {
                Brake_SetAxis((uint8_t) arg[0]);
            }
            break;

        case DBG_CMD_BRAKE_SIGN:
            if ((arg[0] != 1) && (arg[0] != -1)) {
                dbg.cmd.result = 2u;
            } else {
                Brake_SetSign((int8_t) arg[0]);
            }
            break;

        case DBG_CMD_BRAKE_DISP:
            if (Brake_SetDisplay((uint8_t) arg[0]) == 0u) {
                dbg.cmd.result = 2u;        /* 0 全亮 / 1 下半条 */
            }
            break;

        case DBG_CMD_BRAKE_MODE:
            if (Brake_SetMode((uint8_t) arg[0]) == 0u) {
                dbg.cmd.result = 2u;        /* 0 幅值 / 1 变化率 */
            }
            break;

        case DBG_CMD_BRAKE_DTHR:
            if (Brake_SetDThr((uint16_t) arg[0]) == 0u) {
                dbg.cmd.result = 2u;        /* 200~20000 mg/s */
            }
            break;

        case DBG_CMD_BRAKE_HOLD:
            if (Brake_SetHoldMs((uint16_t) arg[0]) == 0u) {
                dbg.cmd.result = 2u;        /* 200~5000 ms */
            }
            break;

        /* ---------------- 场景切换 ---------------- */
        case DBG_CMD_APP_MODE:
            App_Goto((uint8_t) arg[0]);
            break;

        case DBG_CMD_APP_NEXT:
            App_Step(+1);
            break;

        case DBG_CMD_APP_PREV:
            App_Step(-1);
            break;

        /* ---------------- 省电模式（调试用） ----------------
         * psave 0 = 强制关（同时置否决位，防止下一轮采样又自动打开）
         * psave 1 = 强制开（亮度封顶到 APP_PSAVE_CAP）
         * psave 2 = 交回自动判定（按电量 30%/35% 迟滞）
         * 有它就不用等电池真掉到 30% 才能验降档效果。 */
        case DBG_CMD_APP_PSAVE:
            if ((arg[0] >= 0) && (arg[0] <= 2)) {
                if (arg[0] == 0) {
                    s_psaveForce = 2u;      /* 强制关，且自动判定不再介入 */
                    s_psave      = 0u;
                    s_psaveVeto  = 1u;
                } else if (arg[0] == 1) {
                    s_psaveForce = 1u;      /* 强制开 */
                    s_psave      = 1u;
                } else {
                    s_psaveForce = 0u;      /* 交回自动判定 */
                    s_psave      = 0u;
                    s_psaveVeto  = 0u;
                }
                app_bright_apply();
            } else {
                dbg.cmd.result = 2u;
            }
            break;

        /* ---------------- 其余交给当前场景 ---------------- */
        default:
            if ((s_scenes[s_cur].cmd == 0) ||
                (s_scenes[s_cur].cmd(id, arg) == 0u)) {
                dbg.cmd.result = 3u;
            }
            break;
    }
}

/* ==================== USER3 长按 = 全局亮度 ==================== */

/* 长按判定阈值。比 USER1 的关机阈值(2000ms)短，因为亮度是常调的；
 * 800ms 足够和"短按切小项"区分开。 */
#define APP_KEY_LONG_MS     (800u)

/* 亮度档位：暗 → 亮，长按循环。用固定档位而不是连续加减，
 * 是因为没有屏幕显示"当前多少"，档位跳变最容易感知。 */
static uint8_t app_bright_next(void)
{
    uint8_t cur  = s_bcUser;
    uint8_t next = s_bcLadder[0];
    uint8_t i;

    /* ⚠ 这里**不要**为省电模式截断档位表。调用方（app_user3_task）在长按时
     *   已经把 s_psave 清掉了 —— 省电下的第一下长按只负责"解除封顶、恢复
     *   用户设定"，不跳档；之后才走这里的正常 4 档循环。
     *   早先版本在这里按 s_psave 截断成 2 档，配合"长按直接跳档"会算出
     *   255 → 64，用户想变亮却变最暗。 */
    for (i = 0u; i < APP_BC_LADDER_N; i++) {
        if (s_bcLadder[i] > cur) {
            next = s_bcLadder[i];
            break;
        }
    }

    return next;
}

/* USER3：短按（松手）= 切小项；长按 ≥ APP_KEY_LONG_MS = 亮度跳一档。
 * ⚠ 用"按住电平 + 时间"判定，而不是 in.key 的松开边沿 ——
 *   长按必须在**按住期间**就生效，否则用户得先松手才知道成没成。 */
static void app_user3_task(void)
{
    static uint8_t  s_down;
    static uint8_t  s_long;
    static uint32_t s_downMs;

    if (Key_IsDown(KEY_USER3) != 0u) {
        if (s_down == 0u) {
            s_down   = 1u;
            s_long   = 0u;
            s_downMs = tick_ms;
        } else if ((s_long == 0u) && ((tick_ms - s_downMs) >= APP_KEY_LONG_MS)) {
            s_long = 1u;
            if (s_psave != 0u) {
                /* 省电模式下长按 = **用户否决省电**，而且这一下**只负责
                 * "亮回来"，不跳档**。
                 *
                 * 为什么不能顺手调一档：用户设的是 255、屏幕上是 128（封顶），
                 * 此时 app_bright_next() 从 255 往上找不到档位，会循环回 64 ——
                 * 用户想变亮，结果变成最暗，这是实测出来的坑。
                 * 第一下长按恢复用户设定，之后的长按才正常循环。
                 * veto 保持到电量回到 APP_PSAVE_PCT_OFF 以上，否则 2 s 后
                 * 又被自动降档，用户会以为按键坏了。 */
                s_psave     = 0u;
                s_psaveVeto = 1u;
                App_SetBrightness(s_bcUser);    /* 档位不动，只是解除封顶 */
            } else {
                App_SetBrightness(app_bright_next());
                route_mark_dirty();             /* 让设置记录 2 s 后落盘 */
            }
        }
        return;
    }

    if (s_down == 0u) {
        return;                             /* 一直没按过 */
    }

    s_down = 0u;
    if (s_long == 0u) {
        Route_KeyItem();                    /* 短按：切小项 */
    }
    /* 长按的那次松手不再触发任何动作 —— 已经消费在亮度上了 */
}

/* ==================== 低电安静窗口 ==================== */

/* LED 满载 360 mA 会把电池读数压低 200~335 mV（battery.h §③），所以必须
 * 先熄屏再采，否则一颗健康的 3.8 V 电池会被读成 3.43 V，直接误判低电关机。
 *
 * 用 0x37 = 0 熄屏：一条命令立即生效，不清帧缓冲、也不等扫描走完，
 * 采样后写回就能原样恢复画面。熄屏 ≈ BAT_SETTLE_MS + 一次采样 ≈ 1.5 ms，
 * 肉眼不可见。
 *
 * ⚠ 这段以前只写在 fluid_demo.c 里，于是**只有流体场景会刷新电量**：
 *   心跳 / 文字 / 静态图下 dbg.bat.mv 永远停在开机那一次采样，
 *   低电保护和省电模式全都静默失效（battery.h §"渲染层必须配合"警告过）。
 *   现在提到这里，所有场景共享；以后加新场景**不需要**再管这件事。
 *
 * 必须**在场景 tick 之前**调用：熄屏只改 0x37，DDRAM 内容不动，所以紧接着
 * 的 tick 重绘、或者"定格不重绘"，都不会让画面出错，恢复亮度就是原样。 */
static void App_QuietWindow(void)
{
    if (Battery_QuietBegin() == 0u) {
        return;
    }

    HT16D33A_SetBrightness(HT16D33A_BRIGHTNESS_OFF);
    mspm0_delay_ms(BAT_SETTLE_MS);
    Battery_QuietSample();              /* 低电确认时这里直接关机，不返回 */

    app_psave_update(dbg.bat.mv);       /* 只改标志，不写芯片 */
    app_bright_apply();                 /* 恢复亮度（省电模式下是封顶后的值）*/
}

/* ==================== 开机电量画面 ==================== */

/* 布局（应用坐标 15 列 × 16 行，见 led_matrix.h）：
 *
 *        正极帽  x 6~10, y 1
 *        外壳    x 4~12 的 y 2 / y 16 两条横边
 *                x 4 / x 12 的 y 2~16 两条竖边
 *        液柱    x 5~11, y 3~15（13 行），从下往上填
 *
 * 边框亮度给 32 而不是 56：DDRAM 是物理线性的，人眼 ≈ 亮度^0.45，
 * 56 在眼里已经是 sRGB 130 的中灰，会抢液柱的视觉重量。
 *
 * **不跟随重力**：开机时设备朝向还没确定（用户可能正拿着、正在转），
 * 画面跟着转反而乱。固定竖放，进了场景再由各场景自己摆正。
 */
#define APP_SPLASH_FRAME_LV  (32u)    /* 外壳 / 正极帽亮度 */
#define APP_SPLASH_FILL_LV   (255u)   /* 液柱亮度 */
#define APP_SPLASH_EDGE_MS   (200u)   /* ① 边框先出现 */
#define APP_SPLASH_RISE_MS   (25u)    /* ② 液柱每升一行 */
#define APP_SPLASH_HOLD_MS   (600u)   /* ③ 停留 */
#define APP_SPLASH_BLINK_MS  (150u)   /* ④ 低电闪烁的半周期 */
#define APP_SPLASH_FADE_MS   (250u)   /* ⑤ 整体淡出（分 8 帧） */
#define APP_SPLASH_ROWS      (13u)    /* 液柱总行数 */

/* 电量 → 液柱行数。四舍五入；非 0 时至少 1 行 ——
 * 否则 5% 和 0% 长得一样，用户会以为电池已经空了。 */
static uint8_t app_splash_rows(uint8_t pct)
{
    uint8_t h = (uint8_t) (((uint16_t) pct * APP_SPLASH_ROWS + 50u) / 100u);

    if ((h == 0u) && (pct != 0u)) {
        h = 1u;
    }
    return h;
}

/* 画一帧。rows = 液柱行数 0~13；fillLv = 液柱亮度（淡出时逐帧降）*/
static void app_splash_draw(uint8_t rows, uint8_t frameLv, uint8_t fillLv)
{
    uint8_t x, y;

    LedMatrix_Clear();

    for (x = 6u; x <= 10u; x++) {                           /* 正极帽 */
        LedMatrix_SetCellLevel(x, 1u, frameLv);
    }
    for (x = 4u; x <= 12u; x++) {                           /* 上下横边 */
        LedMatrix_SetCellLevel(x, 2u, frameLv);
        LedMatrix_SetCellLevel(x, 16u, frameLv);
    }
    for (y = 2u; y <= 16u; y++) {                           /* 左右竖边 */
        LedMatrix_SetCellLevel(4u, y, frameLv);
        LedMatrix_SetCellLevel(12u, y, frameLv);
    }
    for (y = (uint8_t) (15u - rows + 1u); y <= 15u; y++) {  /* 液柱：从下往上 */
        for (x = 5u; x <= 11u; x++) {
            LedMatrix_SetCellLevel(x, y, fillLv);
        }
    }

    LedMatrix_Flush();
}

/* 等 ms，期间按键可跳过。返回 1 = 用户要求跳过。
 *
 * ⚠ 用 Key_TookPress()（按下**边沿**）而不是 Key_IsDown()（电平）：
 *   本板 USER2(PA24) 没装按键、引脚悬空恒读低，用电平判会一进来就退出
 *   （led_show.c:200 踩过同一个坑）。边沿判据天生免疫卡住的键。
 *   三个键都读一遍，把边沿标志清干净，免得留给后面的场景。 */
static uint8_t app_splash_wait(uint32_t ms)
{
    uint32_t t0 = tick_ms;
    uint8_t  hit = 0u;

    while ((uint32_t) (tick_ms - t0) < ms) {
        PowerKey_CheckShutdown();       /* 长按 USER1 关机在开机画面期间仍然有效 */

        if (Key_TookPress(KEY_USER1) != 0u) { hit = 1u; }
        if (Key_TookPress(KEY_USER2) != 0u) { hit = 1u; }
        if (Key_TookPress(KEY_USER3) != 0u) { hit = 1u; }
        if (hit != 0u) {
            return 1u;
        }
        __WFI();
    }
    return 0u;
}

/* ==================== 关机动画 ====================
 *
 * 低电关机前播一段：**当前电量 → 最后一格消失 → 空电池闪 3 下 → 边框淡出**。
 * 约 1.1 s。两个时机共用同一个函数：
 *   · 开机就低电      → App_BootSplash() 里判定后直接播
 *   · 运行中确认低电  → App_LowBatteryAction()（battery.c 的钩子）
 *
 * 为什么不做"满格掉到空"：低电关机时电量必然只剩 0~1 格（阈值 3550 mV
 * 对应约 5%），画满格再掉光是假的，还会让人以为电量中途回升过。
 * 直接从"你只剩这一格"开始，反而更直白。
 *
 * ⚠ 进来时必须先把 0x37 恢复：运行中这条路是从 Battery_QuietSample() 进来的，
 *   外面刚把亮度写成 0 熄屏采样，不恢复的话整段动画都在黑屏上播完。
 */
#define APP_SHUT_HOLD_MS     (80u)    /* ① 定格当前电量 */
#define APP_SHUT_DROP_MS     (60u)    /* ② 最后一格消失 */
#define APP_SHUT_BLINK_MS    (130u)   /* ③ 空电池闪烁的半周期 */
#define APP_SHUT_FADE_MS     (200u)   /* ④ 边框淡出（分 6 帧） */

static void App_ShutdownAnim(void)
{
    uint8_t i, rows, lv;

    /* ① 恢复亮度，而且**直接用满档**。
     *   两个理由：
     *   · 运行中这条路进来时 0x37 还是 0（安静窗口刚熄屏采样），不恢复就是黑屏；
     *   · 此刻多半已经在省电模式里（eff=128），边框 32 乘上 128/255 只剩
     *     sRGB 71，几乎看不见 —— 而这是设备最后一次点亮，必须让人看清。
     *     满亮 1 秒 ≈ 0.1 mAh，相对于"已经要关机了"完全可以忽略。 */
    HT16D33A_SetBrightness(HT16D33A_BRIGHTNESS_MAX);

    /* ② 定格当前电量，让用户看清"确实只剩这么点" */
    rows = app_splash_rows(App_BatPercent(dbg.bat.mv));
    app_splash_draw(rows, APP_SPLASH_FRAME_LV, APP_SPLASH_FILL_LV);
    if (app_splash_wait(APP_SHUT_HOLD_MS) != 0u) { goto shut_done; }

    /* ③ 最后一格消失 */
    if (rows != 0u) {
        app_splash_draw(0u, APP_SPLASH_FRAME_LV, 0u);
        if (app_splash_wait(APP_SHUT_DROP_MS) != 0u) { goto shut_done; }
    }

    /* ④ 空电池闪 3 下 —— 这一段是动画的主体，读作"没电了" */
    for (i = 0u; i < 3u; i++) {
        app_splash_draw(0u, APP_SPLASH_FRAME_LV, 0u);
        if (app_splash_wait(APP_SHUT_BLINK_MS) != 0u) { goto shut_done; }
        app_splash_draw(0u, 0u, 0u);
        if (app_splash_wait(APP_SHUT_BLINK_MS) != 0u) { goto shut_done; }
    }

    /* ⑤ 边框淡出，别"啪"一下就黑 */
    for (i = 6u; i > 0u; i--) {
        lv = (uint8_t) ((uint32_t) APP_SPLASH_FRAME_LV * i / 6u);
        app_splash_draw(0u, lv, 0u);
        if (app_splash_wait(APP_SHUT_FADE_MS / 6u) != 0u) { goto shut_done; }
    }

shut_done:
    /* 留一张全黑的屏给 ShutdownNow()。它也靠保持电容撑一小会儿，
     * 这期间屏幕上不该还挂着半张电池图。 */
    LedMatrix_Clear();
    LedMatrix_Flush();
}

/* 上电先报一次电量，约 1.4 s，按任一键可跳过。
 *
 * 为什么要做：这块板子没有电压显示，电池剩多少全靠猜；而开机本来就要等
 * IMU 初始化，这一秒多正好用来说一件有用的事。
 *
 * 调用点见 App_Init()：放在 IQ 自检和 IMU 初始化**之后** —— 那两步失败会
 * FillAll 卡死，属于故障指示，不该被电量图盖掉；能走到这里就说明硬件都正常。 */
static void App_BootSplash(void)
{
    uint8_t  pct, rows, i, lv, low;
    uint32_t mv, v;

    /* 重采。main() 里那次紧跟 SYSCFG_DL_init，LDO 输出可能还没稳；
     * 此刻 LED 全黑、电源已经工作了几十毫秒，读数更可信。
     *
     * 连采 BAT_LOW_CONFIRM 次（间隔 40 ms）：**单次读数偏低就判定"没电"直接
     * 关机太危险** —— 一颗好电池被关掉，用户只会以为板子坏了。
     * 3 次全低才认，和运行期那套确认逻辑对齐。 */
    mv  = 0u;
    low = 0u;
    for (i = 0u; i < BAT_LOW_CONFIRM; i++) {
        if (i != 0u) {
            mspm0_delay_ms(40u);
        }
        v = Battery_ReadMilliVolt();
        if (v != 0u) {
            mv = v;
            if (v < BAT_LOW_MV) {
                low++;
            }
        }
    }

    if (mv == 0u) {
        return;                     /* 一直读不到：别显示 0% 的电池，也别关机 */
    }

    /* 开机就低电：不播电量画面，直接播关机动画然后关。
     * 动画第一帧就是真实电量（0~1 格），所以不需要额外定格。
     * ShutdownNow() 不返回 —— 后面的 GlyphStore_Load / Route_Init 都不会执行。 */
    if (low >= BAT_LOW_CONFIRM) {
        App_ShutdownAnim();
        Battery_ShutdownNow();
    }

    pct  = App_BatPercent(mv);
    rows = app_splash_rows(pct);

    /* ① 边框先立住，液柱 0 行 */
    app_splash_draw(0u, APP_SPLASH_FRAME_LV, 0u);
    if (app_splash_wait(APP_SPLASH_EDGE_MS) != 0u) { goto splash_done; }

    /* ② 液柱逐行涨上去 */
    for (i = 1u; i <= rows; i++) {
        app_splash_draw(i, APP_SPLASH_FRAME_LV, APP_SPLASH_FILL_LV);
        if (app_splash_wait(APP_SPLASH_RISE_MS) != 0u) { goto splash_done; }
    }

    /* ③ 停留 */
    if (app_splash_wait(APP_SPLASH_HOLD_MS) != 0u) { goto splash_done; }

    /* ④ 电量 ≤ 20% 时闪两下，提醒该充电了（不影响正常启动）*/
    if (pct <= 20u) {
        for (i = 0u; i < 2u; i++) {
            app_splash_draw(0u, APP_SPLASH_FRAME_LV, 0u);
            if (app_splash_wait(APP_SPLASH_BLINK_MS) != 0u) { goto splash_done; }
            app_splash_draw(rows, APP_SPLASH_FRAME_LV, APP_SPLASH_FILL_LV);
            if (app_splash_wait(APP_SPLASH_BLINK_MS) != 0u) { goto splash_done; }
        }
    }

    /* ⑤ 淡出：边框和液柱一起衰减，8 帧 */
    for (i = 8u; i > 0u; i--) {
        lv = (uint8_t) ((uint32_t) APP_SPLASH_FRAME_LV * i / 8u);
        app_splash_draw(rows, lv,
                        (uint8_t) ((uint32_t) APP_SPLASH_FILL_LV * i / 8u));
        if (app_splash_wait(APP_SPLASH_FADE_MS / 8u) != 0u) { goto splash_done; }
    }

splash_done:
    /* 交出去之前灭屏。Route_Boot() 里的 App_Goto 也会清屏，但那是"切场景"
     * 的语义；这里先灭掉，万一后面哪步卡住也不会留一张电量图当背景。 */
    LedMatrix_Clear();
    LedMatrix_Flush();

    /* 用开机这次采样直接判一次省电。不这么做的话要等 2 s 后的第一次安静
     * 窗口才降档，用户会看到"开机全亮一下、然后突然变暗"。 */
    app_psave_update(dbg.bat.mv);
    app_bright_apply();
}

/* ==================== 每帧 ==================== */

void App_Tick(void)
{
    app_in_t in;

    /* 低电安静窗口：必须**在场景 tick 之前**（理由见函数上方注释）*/
    App_QuietWindow();

    App_ReadInput(&in);

    /* 全局事件先消费：消费掉就从 in.key 清位，场景只拿到剩下的。
     *
     * ⚠ 本板 **USER2 (PA24) 没有装按键，引脚悬空** —— 悬空输入经过去抖器可能
     *   随机被承认成"按下"，拿它做切场景会莫名其妙跳场景，所以**不用它**。
     *   syscfg 里给 PA24 开了内部上拉，至少让读数稳定；以后焊上按键就能用。
     *
     * 按键分配（两级路由，见 asset_route.h / 多资源与两级按键方案.md §4）：
     *   USER1 (PA25) 短按 = **切大类**；长按 2s = 关机（走 TIMA0 中断）
     *   USER3 (PA23) 短按 = **切小类**（当前大类内循环）
     *
     * 两边都是**松发**触发（in.key 装的是松开边沿，见 App_ReadInput）。
     * 长按关机由 PowerKey 用 Key_IsDown()（按住电平）判断，所以：
     *   · 短按 → 松手那一刻切一次；
     *   · 长按 2s → 关机过程中**不会**先切大类（以前压发会先切一次）。
     */
    if ((in.key & APP_KEY_USER1) != 0u) {
        in.key = (uint8_t) (in.key & (uint8_t) ~APP_KEY_USER1);
        Route_KeyCat();
    }

    /* USER3 交给 app_user3_task()：它要区分"短按切小项"和"长按调亮度"，
     * 光有松开边沿不够，得自己看电平和时长。这里把边沿位清掉即可。 */
    in.key = (uint8_t) (in.key & (uint8_t) ~APP_KEY_USER3);
    app_user3_task();

    if (s_scenes[s_cur].tick != 0) {
        s_scenes[s_cur].tick(&in);
    }

    /* ---- 刹车高亮：**必须在场景 tick 之后** ----
     * 场景每帧都会重画 DDRAM，刹车画面放在前面会被直接盖掉。
     * 这条顺序是刹车能不能看见的唯一条件，别调换。 */
    Brake_Tick(&in);
}

/* ==================== 初始化 + 主循环 ==================== */

static void App_Init(void)
{
    /* 串口调试通道**第一个初始化**：后面任何一步失败（数学层自检、IMU、LED 芯片）
     * 都还能靠串口把原因报出来，比"整屏点亮当报错信号"有用得多。 */
    VCMD_Init();                            /* 武装 RX DMA */
    VOFA_Init();                            /* 遥测默认关闭，发 div 1 才开 */

    LedMatrix_Init();                       /* 芯片初始化 + 清屏 */

    /* 注册低电关机动作（熄屏 + 关机）。不注册则默认直接关机、不熄屏。 */
    Battery_SetLowAction(App_LowBatteryAction);

    /* 数学层自检：坏了就把整屏点亮停住（还能长按 USER1 关机）*/
    dbg.fluid.iq_ok = FluidDemo_IqSelfTest();
    if (dbg.fluid.iq_ok == 0u) {
        LedMatrix_FillAll(1u);
        LedMatrix_Flush();
        for (;;) {
            PowerKey_CheckShutdown();
            VCMD_Tick();                    /* 至少还能从串口看到 rx_bytes / last_id */
            VOFA_Tick();
            __WFI();
        }
    }

    /* ---- IMU ----
     * 初始化返回码写进 dbg.fluid.imu_ok，方便对照 lsm6ds3trc.h：
     *   0 = OK，1 = WHO_AM_I 不对（可能装的是 LSM6DSR 0x6B），2 = 寄存器读不回来
     * 失败时（若允许 fallback）退回假重力，程序照常跑，不会卡死。 */
#if APP_USE_IMU
    uint32_t r = LSM6DS3TRC_Init();
    dbg.fluid.imu_ok = (uint8_t) r;

    if (r == LSM6DS3TRC_OK) {
        s_useImu = 1u;
    } else {
#if APP_IMU_FALLBACK
        s_useImu = 0u;                      /* 退回假重力 */
#else
        LedMatrix_FillAll(1u);
        LedMatrix_Flush();
        for (;;) {
            PowerKey_CheckShutdown();
            __WFI();
        }
#endif
    }
#else
    dbg.fluid.imu_ok = 0xFFu;               /* 0xFF = 编译期就没启用 IMU */
#endif

    dbg.fluid.imu_use = s_useImu;

    /* 滤波器复位：第一次读 IMU 时会用当时的读数播种（见 App_ReadGravity），
     * 这样开机瞬间就朝真实重力方向，不会先"自由落体"一下 */
    s_axSlow     = 0;
    s_aySlow     = 0;
    s_sxFast     = 0;
    s_syFast     = 0;
    s_filtSeeded = 0u;

    /* 刹车高亮：状态清零 + 同步 dbg。放在 IMU 之后 ——
     * 它要看 dbg.fluid.imu_use 决定是否参与检测。 */
    Brake_Init();

    dbg.fluid.tilt = s_tilt;
    s_tiltStart = tick_ms;
    s_lastMs    = tick_ms;

    dbg.app.scene_cnt = SCENE_COUNT;

    /* ---- 开机电量画面（约 1.4 s，按任一键跳过）----
     * 放在这里而不是更早，有两个原因：
     *   ① 上面的 IQ 自检 / IMU 初始化失败会 FillAll 卡死，那是故障指示，
     *      不该被电量图盖掉 —— 能走到这一行就说明硬件都正常；
     *   ② 电源已经工作了几十毫秒，重采样比 main() 里那次更可信。
     * 也不跟随重力：开机时朝向还没确定，固定竖放。 */
    App_BootSplash();

    /* ⚠ 上面阻塞了 1.4 s，dt 基准必须重新对时。
     *   否则主循环第一帧会拿到一个 1400 ms 的 dt（App_ReadInput 只截断到
     *   65535，拦不住），假重力也会因为 s_tiltStart 落后而提前换向。
     *   现在的场景不用 dt_ms，但 brake.c 用了 —— 以后打开 BRAKE_ENABLE 就会踩。 */
    s_lastMs    = tick_ms;
    s_tiltStart = tick_ms;

    /* 从 Flash 恢复自定义滚动消息（只读进 RAM，不做任何绘制）。
     * 校验失败 / 没存过 → 什么都不做，文字场景用内置字库。 */
    (void) GlyphStore_Load();

    /* 两级路由：先读设置记录（只读 Flash，不绘制），再按它进场景。
     * 没存过 / 校验失败 → 默认 (流体, 0)，行为和以前完全一样。 */
    Route_Init();
    Route_Boot();
}

void App_Run(void)
{
    uint32_t loopStart;

    App_Init();

    for (;;) {
        loopStart = tick_ms;

        /* 一键开关机：USER1 长按 2s 后由 TIMA0 中断拉低 PA22（释放电源自锁），
         * 这里只需要停下来别再干活，安静等硬件断电 */
        PowerKey_CheckShutdown();

        /* 低电检查调度：每 BAT_CHECK_MS 置一次"该熄屏采样了"。
         * 真正的采样在推屏的安静窗口里做（见 fluid_demo.c 的 FluidDemo_Push）。 */
        Battery_Task();

        /* ---- 串口：收命令（不阻塞，没数据时只读两个寄存器）---- */
        VCMD_Tick();

        /* ---- 执行命令：固定在这里生效，不会在推屏中途改显示状态 ---- */
        App_CmdTask();

        /* ---- 输入 + 全局事件 + 当前场景一步 ---- */
        App_Tick();

        /* ---- 路由状态落盘：脏了且空闲 2 s 才真写 Flash（擦扇区会卡一帧）---- */
        Route_Tick();

        /* ---- 串口：发遥测（DMA 忙就丢帧，绝不等待）---- */
        VOFA_Tick();

        /* ---- 空闲节能 ----
         *
         * 流体场景一轮 ≈20ms（物理占大头），永远走不到这里。
         * 但**静态图场景的 tick 是空的** —— 不睡的话主循环会以几十 kHz 空转：
         *   · 白烧电（这块板子是电池供电）
         *   · 遥测 div 1 时每秒几万帧全撞在 DMA 忙上，tx_drops 爆表
         *
         * tick_ms 是 1ms 的 SysTick 累加的，所以"这一圈不到 1ms"就意味着
         * tick_ms 没变 —— 睡到下一个 SysTick 再继续，循环自然被压到 ≈1kHz。
         * 按键/关机在 TIMA0 中断里，不受影响；串口 RX 是 DMA 轮询，最坏延迟 1ms。 */
        if (tick_ms == loopStart) {
            __WFI();
        }
    }
}
