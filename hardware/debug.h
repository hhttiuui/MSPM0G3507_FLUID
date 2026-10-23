/*
 * debug.h —— 观测量的唯一存放点：一个 volatile 结构体 dbg
 *
 * 分工（与 MSPM0G3507_DRV8316 工程同一套做法）：
 *   本文件   观测量和观测接口的唯一定义源
 *   debug.c  观测逻辑的实现（窗口统计 / HardFault 现场 / 时间打点换算）
 *   驱动层   lsm6ds3trc.c 只往 dbg 里写**瞬时值**（寄存器回读、原始值、单次耗时）
 *   上层     imu_selftest.c 只负责"调用 + 定期发布窗口"
 *
 * 好处：加新观测量只要在下面 dbg_t 里加一行成员，既不用改 debug.c，
 *       也不用往 CCS 的 Expressions 列表里手动加 —— 展开 dbg 就能看到。
 *
 * 上电后按这个顺序看就能判断陀螺仪到底通没通：
 *   dbg.who.id       读到的 WHO_AM_I 实测值（LSM6DS3TR-C 应为 0x6A）
 *   dbg.who.init_res 0 = 初始化通过；1 = ID 不对；2 = 寄存器写了读不回来
 *   dbg.spi.xfer     在涨          数据线上确实在收发
 *   dbg.spi.rx       不恒为 0x00/0xFF   MISO 有被器件驱动
 *   dbg.raw.cnt      在涨          读取循环在跑
 *   dbg.raw.*        转动板子时变化   六轴数据是真的
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>
#include <stdbool.h>

/* 下面的 DBG_Mark()/DBG_Elapsed() 要用 TIMER_0(TIMA0) 和
 * DL_Timer_getTimerCount()，所以这里需要生成代码的头文件。 */
#include "ti_msp_dl_config.h"

/* ---- 器件识别 ---- */
typedef struct {
    uint8_t  id;        /* 最近一次读到的 WHO_AM_I(0x0F) 实测值 */
    uint8_t  want;      /* 期望值（LSM6DS3TR-C = 0x6A） */
    uint8_t  match;     /* 1 = id == want */
    uint8_t  init_res;  /* 初始化返回码：0 = 成功，非 0 见 lsm6ds3trc.h 的 LSM6DS3TRC_ERR_* */
    uint32_t init_ms;   /* 初始化耗时 (ms) */
} dbg_who_t;

/* ---- SPI 总线（判断"到底有没有在通信"看这里） ---- */
typedef struct {
    uint8_t  last_reg;  /* 最近访问的寄存器地址（不含 R/W 位） */
    uint8_t  rw;        /* 1 = 读，0 = 写 */
    uint8_t  cmd;       /* 最近一次事务的首字节（读时已含 0x80） */
    uint8_t  tx;        /* 最近一次发出的数据字节 */
    uint8_t  rx;        /* 最近一次收到的字节（MISO 原始值） */
    uint32_t xfer;      /* 累计收发字节数（单调） */
    uint32_t timeout;   /* 等 RXFIFO / 空闲 的超时次数，应为 0（单调）>0 说明 SPI 卡了 */
} dbg_spi_t;

/* ---- 六轴原始值（最近一次，LSB） ---- */
typedef struct {
    int16_t  gx, gy, gz;    /* 陀螺仪原始值（OUTX_L_G..OUTZ_H_G） */
    int16_t  ax, ay, az;    /* 加速度计原始值（OUTX_L_XL..OUTZ_H_XL） */
    uint32_t cnt;           /* 累计读取次数（单调） */
} dbg_raw_t;

/* ---- 换算后的物理量（整数定点，不是浮点） ----
 * 本工程只用到三轴数据，不在这里做姿态解算；量程假设：
 *   陀螺 ±2000 dps → 70   mdps/LSB
 *   加速度 ±2 g    → 61   µg/LSB
 * ⚠ 改了 lsm6ds3trc.h 里的 LSM6DS3TRC_FS_G / LSM6DS3TRC_FS_XL，
 *   这里的实际含义就跟着变（换算系数在 lsm6ds3trc.c 里同步改）。
 *
 * 为什么不用 float：M0+ 没有 FPU，一次浮点乘就是一次库调用；而传感器 1 LSB
 * 本来就是 61 µg / 70 mdps —— 显示成 mg / mdps 已经"刚好不丢信息"。 */
typedef struct {
    int32_t gx_mdps, gy_mdps, gz_mdps;  /* 角速度 (毫度/秒) */
    int32_t ax_mg,   ay_mg,   az_mg;    /* 加速度 (毫克) */
    int32_t temp_mdegc;                 /* 温度 (毫摄氏度) */
} dbg_phy_t;

/* ---- 时间统计（时间基准 = TIMER_0(TIMA0)）----
 * TIMA0：BUSCLK 80MHz ÷ 8 = 10MHz、16 位、周期 5ms ⇒ **1 拍 = 100ns**，
 * 量程 6.55ms。一次 14 字节突发读约 14~20us（140~200 拍），远小于量程。
 * ⚠ 分频是 8 而不是 2 的原因：5ms 在 40MHz 下要 200000 拍，超出 16 位计数器
 *   范围（SysConfig 会直接报 out of range），所以只能把定时器时钟降到 10MHz。
 *   代价是"1 拍"的物理长度从 25ns 变成 100ns —— DBG_TicksToUs() 的除数
 *   必须同步改，否则所有耗时都会显示成真实值的 1/4。
 *
 * ⚠ TIMA0 是**递减**计数器（DL_TIMER_TIMER_MODE_PERIODIC = DOWN），
 *   所以"耗时 = 早的打点 − 晚的打点"，跨回绕靠 16 位取模自动正确。
 *
 * ⚠ read_min_us / read_max_us 是**窗口量**：debug.c 里每调用一次
 *   DBG_PerfPublish() 就把上一个窗口的统计发布到这里并清零重算。
 *   所以读到的永远是完整窗口的结果，不会被上电初期的一次性异常永久污染。
 *   win 是已发布的窗口数（单调）；win == 0 表示"还没发布过第一个窗口"，
 *   此时 read_min_us 显示 0xFFFFFFFF 是正常的。 */
typedef struct {
    uint32_t read_us;       /* 最近一次六轴读取耗时 (us)（瞬时值） */
    uint32_t read_min_us;   /* 上一个窗口内最小（窗口量） */
    uint32_t read_max_us;   /* 上一个窗口内最大（窗口量） */
    uint32_t win;           /* 已发布窗口数（单调） */
} dbg_perf_t;

/* ---- 姿态解算结果（占位）----
 * 现在只读三轴原始数据，不做解算，所以这里全是 0。
 * 后续做"简单解算"（例如陀螺积分角 + 加速度计重力方向求倾角）时，
 * 把结果写进这里，单位统一 0.01°，同样是整数。 */
typedef struct {
    int32_t roll_x100;    /* 横滚角 ×100 */
    int32_t pitch_x100;   /* 俯仰角 ×100 */
    int32_t yaw_x100;     /* 偏航角 ×100（无磁力计时会漂，仅作参考） */
} dbg_att_t;

/* ---- CPU 卡死定位 ----
 * 现象：主循环不再前进（dbg.spi.xfer 不涨）。
 *   cnt != 0 → 进过 HardFault
 *   pc       → 出错那条指令的地址（在 CCS 的 Disassembly 里跳过去即可）
 *   lr       → 从哪调过来的 */
typedef struct {
    uint32_t cnt;
    uint32_t pc;
    uint32_t lr;
} dbg_fault_t;

/* ---- 调试期运行时开关（在 CCS 的 Expressions 里直接改，不用重新烧录） ----
 * 本工程已砍掉 OLED 显示（显示只用 LED），所以观测量全部靠展开 dbg 来看。
 * LED 点阵那几个"试模式"的开关已经删掉了 —— 现在只保留一个综合演示，
 * 它的节奏是编译期常量（见 hardware/led_show.h），运行时不需再调。 */
typedef struct {
    uint32_t throttle_ms;  /* 两次采样之间的延时 (ms)，0 = 不延时（跑满） */
    uint32_t publish_ms;   /* 窗口发布周期 (ms)：到点发布 min/max，并翻一次心跳 LED */
    uint32_t heartbeat;    /* 1 = 每次发布时翻转指示灯 LED（PB8）做心跳，0 = 不闪 */
} dbg_test_t;

/* ---- 按键（3 路，实现见 hardware/key.c）----
 * raw / down 都是位图：bit0 = USER1(PA25)、bit1 = USER2(PA24)、bit2 = USER3(PA23)。
 * 排查顺序：
 *   raw 一直不动      → 先怀疑 syscfg 里的上拉/下拉和接线
 *   raw 会动但 down 不动 → 去抖参数（KEY_DEBOUNCE_TICKS）问题 */
typedef struct {
    uint8_t  raw;           /* 最近一拍采到的原始电平位图（1 = 按下，低有效） */
    uint8_t  down;          /* 去抖后的稳定状态位图（1 = 按下） */
    uint32_t press_cnt[3];  /* 各键累计"按下"次数（单调），顺序同 key_id_t */
} dbg_key_t;

/* ---- LED 点阵（HT16D33A，实现见 hardware/ht16d33a.c / led_matrix.c / led_show.c）----
 * ⚠ 这颗芯片**只写不能读**（单根 DIO，没有回读通路），所以 dbg.led 里看到的只是
 *   "我们发出了什么"，有没有真的显示只能看灯。排查顺序：
 *     xfer 不涨        → SPI 没在转（syscfg/时钟问题）
 *     timeout != 0     → SPI 等待超时
 *     bytes 不是 34    → CSB 帧拼错了（一帧应该是 [0x80][地址][32 字节数据]）
 *   点不亮时优先怀疑：① CSB 帧格式 ② RAM 地址映射（led_matrix.c，已实测验证）
 *   ③ 0x84 LED_ON_OFF 掩码（初始化里写全 1，少了就一个灯都不亮） */
typedef struct {
    uint8_t  cmd;       /* 最近一条命令码 */
    uint8_t  data;      /* 最近一条命令的参数字节 */
    uint8_t  tx;        /* 最近一次发出的数据字节 */
    uint8_t  addr;      /* 最近一次 RAM / 使能 RAM 写的起始地址 */
    uint8_t  page;      /* 当前选中的 RAM 页（0x00 = Page1） */
    uint16_t bytes;     /* 最近一次 CSB 事务的字节数 */
    uint32_t xfer;      /* 累计发出字节数（单调，判活用） */
    uint32_t timeout;   /* SPI 等待超时次数，应为 0 */
    uint8_t  init_res;  /* 初始化返回码 */
    uint16_t step;      /* 综合演示游走段的格号 0..239 */
    uint8_t  show_stage;/* 综合演示当前段：0 没在播 / 1 全亮 / 2 全黑 / 3 游走 / 4 图案
                         * （取值见 led_show.h 的 LED_SHOW_STAGE_*）*/
    uint32_t init_ms;   /* 初始化耗时 (ms) */
} dbg_led_t;

/* ---- 流体演示（实现见 hardware/fluid_demo.c）----
 * 显示和参数读取都只剩 LED 点阵，界面上的值全搬到这里：
 * 看它跑了多少帧、多快，以及假重力当前指向哪边。 */
typedef struct {
    int32_t  ax10;        /* 送进流体的 x 加速度 ×10（模拟单位，整数好读） */
    int32_t  ay10;        /* 同上，y（约定 y 正 = 屏幕下方） */
    /* ---- 重力 / 摇晃 双通路分解的观测值（×10，模拟单位；原理与调参见
     *      fluid_demo.c 顶部的《重力 / 摇晃：双通路分解》）----
     * ax10/ay10 是**两条通路相加后**的总量（也就是真正喂给流体的值），
     * 下面 4 个是拆开看的两条通路：
     *   slow = 重力 / 倾角分量（慢通路）
     *   fast = 摇晃 / 惯性分量（快通路，**已含 FLUID_SHAKE_GAIN**）
     * 判读：静止时 slow ≈ ±98（1g 向下）、fast ≈ 0；
     *       左右摇时 fast 大幅摆动（几十~几百）、slow 基本不动。
     * ⚠ 只在"真实 IMU"模式下更新；假重力模式不走分解，这 4 个一直是 0。 */
    int32_t  ax_slow10;
    int32_t  ay_slow10;
    int32_t  ax_fast10;
    int32_t  ay_fast10;
    uint32_t shake_pk10;  /* 上一个窗口内 |fast| 的峰值 ×10（静止 ≈0，摇起来几百）。
                           * 这是**窗口量**：发布后清零重算 */
    uint32_t sim_ms_x10;  /* 上一窗口的单步物理耗时 ×10（单位 0.1ms） */
    uint32_t fps;         /* 上一窗口的帧率 */
    uint32_t frames;      /* 累计物理步数 */
    uint8_t  tilt;        /* 假重力方向：0 下 / 1 左 / 2 上 / 3 右。
                           * ⚠ 只在"假重力"模式下才更新；用真实 IMU 时它停在初值 0 */
    uint8_t  iq_ok;       /* 数学层自检：1 = 通过；0 = 整屏点亮并停在那儿 */
    uint8_t  imu_ok;      /* LSM6DS3TR-C 初始化返回码：0 = OK（见 lsm6ds3trc.h 的 _OK/_ERR_ID/_ERR_READBACK）*/
    uint8_t  imu_use;     /* 1 = 当前用真实 IMU 驱动重力；0 = 退回假重力（编译期关掉或 IMU 初始化失败）*/

    /* ---- 渲染观测（参数见 fluid_demo.c 的 FLUID_DENS_FULL_PCT / FLUID_LEVEL_*）----
     * 现在默认是**灰度渲染**：level = density / DENS_FULL × 255。
     *   dens_max       ：窗口内最大密度 ×1000（液体内核，通常几千；1 粒子满权重 = 1000）
     *   dens_lit       ：亮度 ≥ FLUID_LEVEL_VISIBLE_MIN 的格子数（肉眼算"亮"的）
     *   dens_lit_core  ：亮度过半的格子数 ≈ 液体内核，可以和二进制模式的
     *                    "亮了多少格"直接对比
     *   dens_full_x1000：当前满亮度密度 ×1000（= 静止密度 × PCT%）。
     *                    "液体看起来太瘦" → 调小 FLUID_DENS_FULL_PCT（更亮更满）
     *                    "内核一片白没层次" → 调大 PCT
     *   rest_dens      ：静止密度（Q24 原始整数），上面那个满量程就是由它推出来的。
     *                    **必须是正数**；负数说明 density_update() 的累加又溢出了。
     *                    （2026-10 已把该累加器从 int32 改成 int64，N 再大也不会翻负）*/
    uint32_t dens_max;
    uint32_t dens_lit;
    uint32_t dens_lit_core;
    uint32_t dens_full_x1000;
    int32_t  rest_dens;

    /* ---- 硬件开方（MATHACL SQRT）自检结果（见 fluid_iqmath.h）----
     * iq_sqrt_ok: 1 = 硬件开方自检通过，_IQsqrt 走硬件（快）
     *             0 = 自检没通过，已自动退回软件开方（仿真照跑，只是慢）
     *             0xFF = IQ_SQRT_MODE = 0，本来就是软件实现
     * iq_sqrt_sf / iq_sqrt_it：自检点（_IQ(4.0)）用到的 sfactor 与 NUMITER，
     * 只是留档，便于以后对照手册核对。 */
    uint8_t  iq_sqrt_ok;
    uint8_t  iq_sqrt_sf;        /* 自检点的 sfactor（0~63）*/
    uint8_t  iq_sqrt_it;        /* NUMITER，固定 31 */
    int32_t  iq_sqrt_err;       /* sqrt(_IQ(4.0)) 实测误差（Q24 LSB），期望 ≈ -1024
                                   有值且量级这么小 ⇒ 硬件开方确实在跑、精度够用 */

    /* ---- 硬件乘法（MATHACL MPY32）自检结果 ----
     * iq_mpy_ok: 1 = 硬件乘法可用；0 = 自检不过已退回软件；0xFF = 编译期就是软件
     * iq_mpy_err: 硬件 vs 精确 64 位软件参考的最大绝对误差（Q24 LSB）。
     *             正确时应为 0~1（截断 vs 四舍五入的差），若上千说明硬件算错了。 */
    uint8_t  iq_mpy_ok;
    int32_t  iq_mpy_err;
} dbg_fluid_t;

/* ---- 一键开关机（实现见 hardware/power.c）----
 * state: 0 = 等开机键松开（开机那次按键还按着，绝不能当关机）
 *        1 = 已就绪
 *        2 = 长按计时中
 *        3 = 已释放自锁（PA22 已拉低，等松手断电）
 * hold_ms: 长按已持续多久，到 POWER_OFF_HOLD_MS 就关机 */
typedef struct {
    uint8_t  state;
    uint32_t hold_ms;
} dbg_pwr_t;

/* ---- 电池采样（实现见 hardware/battery.c）----
 * 这个模块现在只做"测量"，不做任何判断/关机动作。
 * 判读顺序：
 *   cnt 不涨            → Battery_Task() 没被调用，或 ADC 没在转换
 *   err 非 0            → 转换超时（ADC 配置 / 中断问题）
 *   mv 与万用表差很多   → 分压比(BAT_DIV_*) 或 VDDA(BAT_VDDA_MV) 不准
 *   mv_max − mv_min 偏大 → 读数被 LED 负载调制。文档实测同一颗电池
 *                          LED 全灭读 3333、全亮读 3601（差 268 mV），
 *                          此时读数只能当参考，不能用来定低电阈值
 *   mv = 0 且 err 不涨  → raw = 0，分压断了 / 电池没接 */ 
typedef struct {
    uint16_t raw;       /* 最近一次中值滤波后的原始值 0~4095 */
    uint32_t mv;        /* 换算后的 VBAT (mV)；0 表示读取失败 */
    uint32_t mv_min;    /* 自开机以来最小（≈ LED 全亮时的"带载"读数） */
    uint32_t mv_max;    /* 自开机以来最大（≈ LED 全灭时的"空载"读数） */
    uint32_t cnt;       /* 成功采样次数（单调） */
    uint32_t err;       /* 转换超时次数，应为 0 */

    /* ---- 运行期低电保护（见 battery.h）----
     * quiet_cnt 停滞     → 渲染层没配合安静窗口（Battery_QuietBegin 没被消费）
     * low_cnt 在涨      → 连续低电次数，到 BAT_LOW_CONFIRM(3) 就关机
     * shutdown = 1      → 已因低电关机 */
    uint32_t quiet_cnt; /* 熄屏精确采样次数 */
    uint8_t  low_cnt;   /* 连续低电次数（0 ~ BAT_LOW_CONFIRM）*/
    uint8_t  shutdown;  /* 1 = 已触发低电关机 */
} dbg_bat_t;

typedef struct {
    dbg_who_t   who;
    dbg_spi_t   spi;
    dbg_raw_t   raw;
    dbg_phy_t   phy;
    dbg_perf_t  perf;
    dbg_att_t   att;
    dbg_key_t   key;
    dbg_pwr_t   pwr;
    dbg_led_t   led;
    dbg_fluid_t fluid;
    dbg_bat_t   bat;
    dbg_fault_t fault;
    dbg_test_t  test;
} dbg_t;

extern volatile dbg_t dbg;

/* ==================== 打点接口（实现见 debug.h 内联 / debug.c） ====================
 *
 * 典型用法（见 imu_selftest.c）：
 *     uint16_t t0 = DBG_Mark();
 *     ... 一段要测的代码 ...
 *     dbg.perf.read_us = DBG_TicksToUs(DBG_Elapsed(t0));
 *     DBG_PerfAcc(dbg.perf.read_us);
 */

/* 读一次 TIMA0 计数（10MHz ⇒ 1 拍 = 100ns）。
 * 写成 static inline 是为了和直接读寄存器一样快，没有函数调用开销。 */
static inline uint16_t DBG_Mark(void)
{
    return (uint16_t) DL_Timer_getTimerCount(TIMER_0_INST);
}

/* 两个打点之间的差（t_before 早于 t_after，递减计数器）。
 * 16 位取模 ⇒ 只要间隔 < 65536 拍就是精确值，跨回绕不用特判。 */
static inline uint16_t DBG_Delta(uint16_t t_before, uint16_t t_after)
{
    return (uint16_t)(t_before - t_after);
}

/* 从 t0 到"现在"的差 */
static inline uint16_t DBG_Elapsed(uint16_t t0)
{
    return DBG_Delta(t0, DBG_Mark());
}

/* 拍 → 微秒（100ns/拍 ⇒ 10 拍 = 1us）。整数除法，向下取整到 1us。 */
static inline uint32_t DBG_TicksToUs(uint16_t ticks)
{
    return (uint32_t) ticks / 10u;
}

/* 累加一次耗时到当前窗口（min/max 语义只有 debug.c 里那一处） */
void DBG_PerfAcc(uint32_t us);

/* 发布当前窗口的统计到 dbg.perf 并清零重算（由上层每个刷新周期调一次） */
void DBG_PerfPublish(void);

/* 故障现场捕获（实现在 debug.c）。只由 HardFault_Handler 通过汇编跳转进来；
 * 普通代码不要调用 —— 它不会返回。 */
void FaultCapture(uint32_t *sp);

#endif /* DEBUG_H */
