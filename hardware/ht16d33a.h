/*
 * ht16d33a.h —— HT16D33A 16×16 LED 点阵驱动（SPI 3 线：CLK / DIO / CSB）
 *
 * 依据：Holtek HT16D33A/HT16D33B 规格书 V1.51（2025-02-25，86 页）。
 * 下面每条都注明手册出处，改代码前先看这里，别凭印象改。
 *
 * ==================== 器件与接线 ====================
 * HT16D33A = **SPI 3 线**版本；HT16D33B 才是 I2C 版本（AD/SCL/SDA）。
 * ⚠ 接口是**由型号固定的**，没有"选接口"的引脚或命令 —— 这条排除了最大的不确定性。
 *
 * 28SSOP 封装（本板）引脚：**7 = CSB，8 = CLK，9 = DIO**（手册 p6 引脚图）。
 * 本工程挂在 SPI0（syscfg 里叫 SPI_LED_DRV）：CLK=PA6、DIO=PA5（SPI PICO）、
 * CSB=PA4（普通 GPIO，低有效）。2MHz / Mode0 / 8bit / MSB first。
 *
 * ==================== 时序（手册 p11~12「SPI 3 线串行总线」） ====================
 *   tCLK ≥ 250ns  → f ≤ 4MHz      （我们用 2MHz ✓）
 *   tCW  ≥ 100ns
 *   tDS/tHS ≥ 50ns
 *   tCSW ≥ 100ns（CSB 高电平宽度）
 *   tCSL ≥ 50ns （CSB 低 → 第一个 CLK）
 *   **tCSH ≥ 4µs（最后一个 CLK → CSB 拉高）**  ← 板子注释说的那条，已实现
 *   DIO 是双向的（CMOS 输出），所以理论上可以回读（见文件末尾"回读"说明）。
 *
 * ==================== 命令（手册 p36~38 命令表） ====================
 * 所有命令都是「[命令码][数据...]」在**同一个 CSB 帧**里，MSB first。
 *   0xFD  RAM 页地址设置   ：后跟 0x00~0x0F，选 Page1~Page16
 *   0x80  写显示 RAM       ：[0x80][A7..A0][D7..D0]…  地址自增
 *   0x81  读显示 RAM       ：[0x81][A7..A0] → 读 [D7..D0]
 *   0x84  写 LED on/off RAM：[0x84][0 0 0 A4..A0][D7..D0]…
 *   0x85  读 LED on/off RAM
 *   0x30  显示模式         ：bit0 = MP（图像/影像模式）
 *   0x31  模式设置         ：bit7 = BGS（0=灰度模式，1=二进制模式）
 *                           bit1:0 = MT（矩阵规格：0=类型1(9×10+9×10)，1=类型2(12×12)，2/3=类型3(16×16)）
 *   0x35  系统 on/off      ：bit1 = FON（振荡器），bit0 = DON（显示）
 *   0x36  恒流            ：CC3..0（0..15 → 3..48mA），默认 0x0A
 *   0x37  全局亮度        ：BC7..0，默认 0xFF
 *   0x38  开路/短路检测
 *   0x39  V_BLANK 电压    ：默认 0x0F（VBEN + VB3..0）
 *   0x70  读内部信息       → 回读 1 字节
 *   0xCC  软件复位
 *
 * ==================== 显示 RAM 的结构（手册 p19~26、p33~35、p40~41） ====================
 * 芯片里有 **16 个 RAM 页**（Page1~Page16，就是一帧一帧，用于动画/影像模式），
 * **每页各自含三块 RAM**：LED Control（LED on/off）+ Display Data（DDRAM）+ Fade data。
 * 用 0xFD 选页；**不选就用上次选中的页（上电默认 Page1）**。
 *
 * **二进制模式**（BGS=1）：DDRAM 是 32×8 bit = **32 字节**，"RAM 地址 ↔ CA 引脚输出
 * 一一对应，字里每个 bit ↔ COM 输出一一对应"。16×16 的换算：
 *      addr = 2×(CA-1) + (COM-1)/8        CA = 1..16
 *      bit  = (COM-1) % 8                 COM = 1..16
 * **灰度模式**（BGS=0）：1 个点 1 字节（可调亮度），16×16 共 256 字节。
 *
 * **LED on/off RAM** 也是 32×8 bit，是"LED 屏蔽/使能"数据：
 * **写 1 = 该 LED 受显示数据控制（亮），写 0 = 屏蔽**。
 * ⚠ 它的复位默认是**全 0**（命令表里 0x84 的数据默认 "00h"）⇒
 *   **不写这个寄存器，一个灯都不会亮**。所以初始化里必须写全 1。
 *
 * ==================== 回读（可选，需要硬件支持） ====================
 * DIO 是双向的：发完 0x81 / 0x85 / 0x70 之后器件会驱动 DIO 把数据吐回来。
 * 本板只引了 CLK/DIO/CSB 三根线，而 SPI0 的 POCI(PA10) 没有接到芯片，
 * 所以要么把 PA5 临时切成输入来读，要么不读（只用眼睛看灯）。
 * 若以后想加"数值验证"，0x70（读内部信息）是最省事的入口。
 */

#ifndef __HT16D33A_H
#define __HT16D33A_H

#include <stdint.h>
#include "ti_msp_dl_config.h"

/* ==================== 命令码（手册 p36~38） ==================== */
#define HT16D33A_CMD_SET_RAM_PAGE       (0xFDu)  /* 后跟 0x00~0x0F 选页 */
#define HT16D33A_CMD_WRITE_LED_DATA     (0x80u)  /* [0x80][地址][数据]… */
#define HT16D33A_CMD_READ_LED_DATA      (0x81u)
#define HT16D33A_CMD_WRITE_LED_ONOFF    (0x84u)  /* [0x84][0 0 0 A4..A0][数据]… */
#define HT16D33A_CMD_READ_LED_ONOFF     (0x85u)
#define HT16D33A_CMD_DISPLAY_MODE       (0x30u)
#define HT16D33A_CMD_CONFIG_MODE        (0x31u)
#define HT16D33A_CMD_SYS_CONTROL        (0x35u)
#define HT16D33A_CMD_SET_CURRENT        (0x36u)
#define HT16D33A_CMD_SET_BRIGHTNESS     (0x37u)
#define HT16D33A_CMD_SET_V_BLANK        (0x39u)
#define HT16D33A_CMD_READ_INFO          (0x70u)  /* 读内部信息 */
#define HT16D33A_CMD_RESET              (0xCCu)

/* 0x31 模式设置的取值 */
#define HT16D33A_MODE_GRAYSCALE         (0x00u)  /* BGS=0：灰度模式，1 点 1 字节 */
#define HT16D33A_MODE_BINARY            (0x80u)  /* BGS=1：二进制模式，1 点 1 bit */
#define HT16D33A_MT_TYPE1               (0x00u)  /* 类型 1：9×10 + 9×10 */
#define HT16D33A_MT_TYPE2               (0x01u)  /* 类型 2：12×12 */
#define HT16D33A_MT_TYPE3               (0x03u)  /* 类型 3：16×16（本板） */

/* 0x35 系统控制 */
#define HT16D33A_SYS_STANDBY            (0x00u)
#define HT16D33A_SYS_OSC_ON             (0x02u)  /* FON = 1 */
#define HT16D33A_SYS_DISPLAY_ON         (0x03u)  /* FON = 1, DON = 1 */

/* 0x36 恒流档位 CC3..CC0（数据 = 0..15 → 约 3..48mA，手册默认 0x0A = 33mA）
 * ⚠ 亮度其实有两级：**全局亮度 0x37（BC7..0）** + **这里的恒流**。两个都要拉满才最亮。
 * ⚠ 48mA 是**每颗 LED** 的灌电流上限：一相里 15 颗同时亮时，公共端的瞬时电流是
 *   它们的总和（几百 mA 级）。实测 48mA 偏亮/偏耗电，所以当前用**半档 24mA**。 */
#define HT16D33A_CURRENT_48MA           (0x0Fu)  /* 15 ≈ 48mA，最亮 */
#define HT16D33A_CURRENT_24MA           (0x07u)  /* 7  ≈ 24mA，48mA 的一半（当前）*/
#define HT16D33A_CURRENT_33MA           (0x0Au)  /* 10 ≈ 33mA，手册默认 */
#define HT16D33A_CURRENT_6MA            (0x01u)  /* 1  ≈ 6mA，省电档 */

/* 0x37 全局亮度 BC7..0 的取值范围。
 * 0 = 全灭（低电检查的"熄屏窗口"就用它：一条命令立即生效，
 * 不用清 RAM、也不用等扫描走完，采样完再写回 MAX 即可恢复画面）。 */
#define HT16D33A_BRIGHTNESS_MAX         (0xFFu)
#define HT16D33A_BRIGHTNESS_OFF         (0x00u)

/* ==================== 显示模式选择 ====================
 * 手册 p19~23（二进制）与 p24~26（灰度）各有一套 RAM 映射表。
 *
 * 【二进制 BGS=1】32 字节 = 256 bit，1 bit/点，只有亮/灭：
 *     addr = 2×(CA-1) + (COM-1)/8      bit = (COM-1)%8
 *   好处：每帧 SPI 只 32 字节（2MHz 下约 0.17ms）。
 *
 * 【灰度 BGS=0】256 字节 = 256×8bit，**1 字节/点**，可调 0~255 亮度：
 *     addr = 16×(CA-1) + (COM-1)       该字节的值 = 这一点的亮度
 *   依据：手册 p26「矩阵类型3：16×16」灰度模式映射表，逐行写着
 *       00h=(C1-1) 01h=(C1-2) … 0Fh=(C1-16)
 *       10h=(C2-1) 11h=(C2-2) … 1Fh=(C2-16)  20h=(C3-1) …
 *   代价：每帧 SPI 涨到 256 字节（2MHz 下约 1.19ms，分 8 个 burst）。
 *
 * ⚠ 两种模式下 (CA,COM) ↔ 物理灯的对应关系是**同一套**，所以 led_matrix.c 里
 *   那两次旋转的推导（阶梯走线 + 通道轮转）在灰度下原样沿用，只有打包方式变了。
 *
 * 亮度是乘法关系（手册 p50）：
 *     亮度 = 最大恒流率 ICA（0x36 的 CC3..0）× 数字调光占空比（0x37 的 BC7..0 / 255）
 *   二进制模式下这两个是**唯一**的亮度旋钮；灰度模式下再多一层逐点字节。 */
#define HT16D33A_USE_BINARY_MODE        (0)

#if HT16D33A_USE_BINARY_MODE
    #define HT16D33A_MODE_VALUE         (HT16D33A_MODE_BINARY | HT16D33A_MT_TYPE3)   /* 0x83 */
    #define HT16D33A_RAM_BYTES          (32u)   /* 32×8bit */
#else
    #define HT16D33A_MODE_VALUE         (HT16D33A_MODE_GRAYSCALE | HT16D33A_MT_TYPE3) /* 0x03 */
    #define HT16D33A_RAM_BYTES          (256u)  /* 1 点 1 字节 */
#endif

/* LED on/off（使能）RAM：固定 32 字节，1 bit/LED */
#define HT16D33A_ONOFF_BYTES            (32u)

/* 一次 CSB 事务最多写多少字节（分块，别让单帧太长） */
#define HT16D33A_BURST_MAX              (32u)

/* 选页用（Page1 = 0x00） */
#define HT16D33A_PAGE_DEFAULT           (0x00u)

/* 事务收尾延时：手册要求 tCSH ≥ 4µs。DL_Common_delayCycles 按 4 周期循环计，
 * CPU 80MHz ⇒ 800 周期 = 10µs，留了一倍余量。 */
#define HT16D33A_CSB_HOLD_CYCLES        (800u)

/* ==================== 接口 ==================== */

/* LED on/off（使能）图案：0 = 全 1（全部 LED 受控，**默认用这个**）
 *                      1 = 全 0（全屏蔽，用来验证"确实没写就全黑"）
 *                      2 = 参考表（开源驱动里那张，是**别人面板**的图案，仅作对照） */
#define HT16D33A_ONOFF_ALL_ENABLE       (0u)
#define HT16D33A_ONOFF_ALL_MASK         (1u)
#define HT16D33A_ONOFF_REF_TABLE        (2u)

/* 初始化：选页 → 复位 → 显示模式 → 16×16 模式 → 电流/亮度/V_BLANK → OSC 开
 *         → 清屏 → 写 LED 使能（全 1，**不写就全黑**）→ 显示开。
 * 返回 0 表示流程走完。⚠ 这颗芯片**没有可靠的 ID 回读通路**（本板 DIO 只做输出），
 * 所以返回值只能说明"我们把命令都发出去了"，不代表器件真的照做了。 */
uint8_t HT16D33A_Init(void);

/* 发一条"命令 + 1 字节参数" */
void HT16D33A_WriteCmd(uint8_t cmd, uint8_t data);

/* 选 RAM 页（0x00~0x0F → Page1~Page16）。所有 RAM 读写都在"当前选中的页"上进行 */
void HT16D33A_SelectPage(uint8_t page);

/* 写当前页的显示 RAM（二进制模式 32 字节 / 灰度模式 256 字节） */
void HT16D33A_WriteRam(uint8_t addr, const uint8_t *data, uint16_t len);

/* 用同一个值填一段显示 RAM */
void HT16D33A_FillRam(uint8_t addr, uint8_t value, uint16_t len);

/* 写 LED on/off（使能）RAM —— 见上面的图案定义 */
void HT16D33A_WriteOnOff(uint8_t pattern);

/* 设置全局亮度（0x37 的 BC7..0）：0 = 全灭，0xFF = 最亮。
 * 用于低电检查的熄屏窗口 —— 立即生效，不影响显示 RAM 里的内容。 */
void HT16D33A_SetBrightness(uint8_t level);

#endif /* __HT16D33A_H */
