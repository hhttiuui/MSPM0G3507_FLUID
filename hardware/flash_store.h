/*
 * flash_store.h —— 主 Flash 数据区的最底层读写（数据区固定 0x10000 起）
 *
 * ==================== TI 例程给的硬约束 ====================
 * 参考：SDK 的 flashctl_multiple_size_write / flashctl_blank_verify，
 *       以及 dl_flashctl.h 的 @param 说明。
 *
 *  1) **所有 program API 的地址必须是 8 字节（64-bit flash word）对齐**：
 *       "@param address ... must be flash word (64-bit) aligned"
 *     ⇒ 不能按任意字节写，只能按 8 字节整字写。本模块负责把调用方的
 *       数据补齐到 8 字节（补 0xFF）再逐字写。
 *
 *  2) 8-bit 版 API 的 CMDBYTEN 固定 0x0101（只开 byte0），
 *     用它无法写到字内任意位置 ⇒ 统一用 64-bit 版（CMDBYTEN=0xFF）。
 *
 *  3) ECC 由硬件生成，**每个 word 只能编程一次**（位只能 1→0）。
 *     ⇒ 必须先擦后写，且同一个字只写一次。
 *
 *  4) 擦除粒度 = 1 扇区 = 1024 B（DL_FLASHCTL_SECTOR_SIZE）。
 *
 *  5) `...FromRAM` 版本在 RAM 中执行并**阻塞等待完成**；执行期间 Flash
 *     不可读（取指会卡住）。本工程 TIMA0 每 5 ms 中断一次，向量表在 Flash，
 *     所以**每条命令前后都要关/开中断**。
 *
 *  6) 每条命令执行完，控制器会自动把内存重新设为受保护状态
 *     ⇒ 每次擦/写前都要重新 DL_FlashCTL_unprotectSector()。
 *
 * ==================== 地址规划 ====================
 *   0x00000000 – 0x00017FFF  固件（实测 56.6 KB，余 39 KB）
 *   0x00018000 – 0x0001FFFF  数据区（32 KB，本模块只管读写，用途见下面的分区）
 *
 * ⚠ 数据区起点从 0x10000 挪到 0x18000 的原因（见 多资源与两级按键方案.md §1）：
 *   固件已经涨到 56.6 KB，离 0x10000 只剩 7.4 KB，而链接器**不会**报错，
 *   长过去就是运行时踩数据。挪到 0x18000 后固件预算 96 KB，风险解除。
 *   代价：迁移前存在 0x10000 的自定义字形读不到了，需要重传一次。
 */
#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>

/* 数据区起点。固件一旦长过这里，链接器不会报错（地址是写死的），
 * 所以发布前务必看一次 map：FLASH 用量必须远小于 0x18000。 */
#define FLASH_STORE_BASE        (0x00018000u)

/* 扇区大小 = 擦除粒度（DL_FLASHCTL_SECTOR_SIZE） */
#define FLASH_STORE_SECTOR      (1024u)

/* ==================== 数据区分区 ====================
 *
 *   0x18000  自定义字形   2 KB  （扇区0 头部 + 扇区1 数据）
 *   0x18800  设置记录     1 KB  （只用 12 B）
 *   0x18C00  文字预设     2 KB  （51 条 × 40 B）
 *   0x19400  动图槽 0    16 KB  （头 1 KB + 数据 15 KB，4bit 128 帧）
 *   0x1D400  动图槽 1     8 KB  （头 1 KB + 数据 7 KB， 4bit  59 帧）
 *   0x1F400  余量         3 KB
 *
 * 每个区都按 1 KB 扇区对齐；分区变了要同步改 多资源与两级按键方案.md §2。
 */
#define FLASH_REG_GLYPH         (FLASH_STORE_BASE + 0x0000u)   /* 2 KB */
#define FLASH_REG_SETTINGS      (FLASH_STORE_BASE + 0x0800u)   /* 1 KB */
#define FLASH_REG_TEXT          (FLASH_STORE_BASE + 0x0C00u)   /* 2 KB */
#define FLASH_REG_ANIM0         (FLASH_STORE_BASE + 0x1400u)   /* 16 KB */
#define FLASH_REG_ANIM1         (FLASH_STORE_BASE + 0x5400u)   /*  8 KB */
#define FLASH_REG_TEST          (FLASH_STORE_BASE + 0x7400u)   /*  3 KB：ftest 专用测试区 */
#define FLASH_REG_LIMIT         (0x00020000u)                  /* flash 末尾 */

/* 编译期检查：分区不能越过 flash 末尾 */
#if ((FLASH_REG_TEST + 0x0C00u) > FLASH_REG_LIMIT)
#error "数据区分区越过 flash 末尾，检查 FLASH_REG_* 宏"
#endif

/* ⚠ ftest 会**擦除** FLASH_REG_TEST 指向的扇区。
 *   这里必须是一个没有任何真实数据的专用区 —— 曾经它写的是
 *   `FLASH_STORE_BASE + 3 扇区`，在旧的 0x10000 基底下刚好是闲置区；
 *   基底搬到 0x18000 之后那个地址正好是"文字预设区"，一按 ftest 就把
 *   预设擦光。所以改成显式的 FLASH_REG_TEST。 */
#if (FLASH_REG_TEST != (FLASH_REG_ANIM1 + 0x2000u))
#error "ftest 测试区必须紧接在最后一个真实分区之后"
#endif

/* 编译期检查：相邻分区不能重叠（改宏时最容易犯的错） */
#if ((FLASH_REG_GLYPH + 2u * FLASH_STORE_SECTOR) != FLASH_REG_SETTINGS)
#error "字形区必须正好 2 个扇区，且紧邻设置记录区"
#endif
#if ((FLASH_REG_SETTINGS + FLASH_STORE_SECTOR) != FLASH_REG_TEXT)
#error "设置记录区必须正好 1 个扇区，且紧邻文字预设区"
#endif
#if ((FLASH_REG_TEXT + 0x800u) != FLASH_REG_ANIM0)
#error "文字预设区必须正好 2 个扇区，且紧邻动图槽 0"
#endif
#if ((FLASH_REG_ANIM0 + 0x4000u) != FLASH_REG_ANIM1)
#error "动图槽 0 必须正好 16 KB，且紧邻动图槽 1"
#endif

/* 擦除从 addr 开始的 n 个扇区（addr 必须扇区对齐）。
 * 返回 1 = 成功，0 = 失败（某一扇区擦除命令返回 FAILED）。 */
uint8_t FlashStore_Erase(uint32_t addr, uint32_t nSectors);

/* 把 data 写到 addr（addr 必须 **8 字节对齐**；len 任意）。
 * 末尾不足 8 字节的部分补 0xFF 一起写入（Flash 擦除态）。
 * 返回 1 = 成功，0 = 失败。 */
uint8_t FlashStore_Write(uint32_t addr, const uint8_t *data, uint32_t len);

/* 读回逐字节比对（Flash 是内存映射的，直接读地址即可）。
 * 返回 1 = 一致，0 = 不一致。 */
uint8_t FlashStore_Verify(uint32_t addr, const uint8_t *data, uint32_t len);

/* CRC16-CCITT（多项式 0x1021，初值 0xFFFF）。
 * 传 0xFFFF 开始新的一轮；分段调用时把上次返回值传进来。 */
uint16_t FlashStore_Crc16(uint16_t crc, const uint8_t *p, uint32_t len);

/* ==================== 设置记录（1 个扇区）====================
 *
 * 存"上次在那个大类的哪一项"，上电由 Route_Init() 读回。
 *
 * ⚠ 写入 = 擦 1 个扇区，擦除期间取指被阻塞、本模块关中断，
 *   流体动画会卡住约 20 ms。所以**只在空闲时写一次**，不要每按一次键就写。
 *   （调用方 Route_Tick() 已按"脏了以后空闲 2 s"节流。）
 *
 * ⚠ 读的时候不要强转指针成结构体：Cortex-M0+ 不支持非对齐访问，
 *   本模块内部用 memcpy 逐字节解析，调用方只拿结构体。
 */
#define FLASH_SETTINGS_MAGIC    (0x5EC7u)
#define FLASH_SETTINGS_VERSION  (1u)

typedef struct {
    uint8_t cat;            /* 上次大类（route_cat_t）*/
    uint8_t item[4];        /* 每个大类各自上次的小类 */
    uint8_t bc;             /* 亮度（当前只记录，不自动恢复）*/
} flash_settings_t;

/* 读设置记录。返回 1 = 有效（*out 已填），0 = 没存过 / 校验失败（*out 不动）。 */
uint8_t FlashStore_SettingsLoad(flash_settings_t *out);

/* 写设置记录（擦扇区 → 写 → 读回校验）。返回 1 = 成功。 */
uint8_t FlashStore_SettingsSave(const flash_settings_t *in);

#endif /* FLASH_STORE_H */
