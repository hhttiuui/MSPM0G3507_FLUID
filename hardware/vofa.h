#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

/**
 * @file vofa.h
 * @brief 串口上行遥测：FireWater 文本协议 + UART1 TX DMA
 *
 * 移植自 MSPM0G3507_DRV8316 工程（那边已验证过）。
 *
 * ==================== 硬件 ====================
 *   UART1  TX = PB6，**115200** 8N1
 *   DMA_CH1  触发源 = DMA_UART1_TX_TRIG，b2f / 字节宽 / 源自增
 *   TX FIFO 阈值 = LEVEL_EMPTY（FIFO 空了触发一次 DMA 搬一个字节）
 *
 * ==================== 为什么必须用 DMA ====================
 *   4 通道一帧约 28 字节，115200 下要 2.4 ms；8 通道约 60 字节要 5.2 ms。
 *   一帧物理才 21.5 ms，阻塞发送会直接吃掉 11~24% 的帧时间，
 *   还会污染 `dbg.fluid.sim_ms_x10` 的测量。所以主循环只负责拼帧，
 *   搬运交给 DMA，忙就丢帧，绝不等待。
 *
 * ==================== 为什么是 FireWater 而不是 JustFloat ====================
 *   本工程全程定点、不引入软浮点；FireWater 直接发整数文本，
 *   在 VOFA+ 里协议选 FireWater、通道数 = 使能的通道个数即可。
 *
 * ==================== 上报格式 ====================
 *   sim, fps, ax10, ay10, lit, bat_mv, shake, dens_max\n
 *   只发掩码里打开的通道，逗号分隔，行尾 '\n'。
 *
 * ⚠ 默认**关闭**（div = 0）。要看得先发 `div 1` 打开，
 *   否则一开串口就被遥测刷屏，命令都看不清。
 */

/* ---- 通道定义：mask 的 bit 号就是下标，顺序即上报顺序 ---- */
#define VOFA_CH_SIM        0u   /* 单步物理耗时 ×10（203 = 20.3 ms）*/
#define VOFA_CH_FPS        1u   /* 帧率 */
#define VOFA_CH_AX10       2u   /* 送进流体的 x 加速度 ×10 */
#define VOFA_CH_AY10       3u   /* 送进流体的 y 加速度 ×10（正 = 屏幕下方）*/
#define VOFA_CH_LIT        4u   /* 亮格数（亮度达到"看得见"的格子）*/
#define VOFA_CH_BAT_MV     5u   /* 电池电压 mV（0 = 读取失败）*/
#define VOFA_CH_SHAKE      6u   /* 窗口内摇晃峰值 ×10 */
#define VOFA_CH_DENS_MAX   7u   /* 窗口内最大密度 ×1000 */
#define VOFA_CH_SCENE      8u   /* 当前场景号（0 流体 / 1 静态图 / 2 文字）*/
#define VOFA_CH_ROT        9u   /* 文字场景当前旋转方向 0~3 */
#define VOFA_CH_BRAKE      10u  /* 刹车高亮是否激活 0/1 */
#define VOFA_CH_RAWX       11u  /* 屏幕 x 原始加速度 mg（标定用）*/
#define VOFA_CH_RAWY       12u  /* 屏幕 y 原始加速度 mg（标定用）*/
#define VOFA_CH_DECEL      13u  /* 沿行进方向的减速度 mg（正 = 正在减速）*/
#define VOFA_CH_RAWZ       14u  /* 屏幕法线原始加速度 mg（默认检测轴）*/
#define VOFA_CH_BASE       15u  /* 当前基线 mg（应≈静止时的 rawz）*/

#define VOFA_CH_ALL        0xFFFFu  /* 16 路全开，mask 取值 0~65535 */
#define VOFA_CH_DEFAULT    0x000Fu  /* 默认只开前 4 路：sim/fps/ax/ay */

/* ---- 分频：0 = 关闭；n = 每 2^n 帧发一帧 ----
 * 46 fps 下：1 → 23 Hz   2 → 11.5 Hz   3 → 5.8 Hz   4 → 2.9 Hz */
#define VOFA_DIV_OFF       0u
#define VOFA_DIV_2         1u

/* 上电初始化。必须在主循环开始前调用一次。默认关闭上报。 */
void VOFA_Init(void);

/* 主循环每圈调一次。不做阻塞等待，DMA 忙时直接丢帧并计数。 */
void VOFA_Tick(void);

/* 设置分频：0 = 关闭，1~7 = 每 2^n 帧一帧。 */
void VOFA_SetDivShift(uint32_t n);

/* 设置通道掩码：bit0~8 对应 VOFA_CH_*（共 9 路，取值 0~511）。
 * ⚠ 改了掩码以后 VOFA+ 里的通道数要跟着改（它按逗号个数解析）。 */
void VOFA_SetMask(uint32_t mask);

#endif /* VOFA_H */
