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
 *   0x00000000 – 0x0000FFFF  固件（当前约 54 KB）
 *   0x00010000 – 0x0001FFFF  数据区（64 KB，本模块只管读写，不划分用途）
 */
#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>

/* 数据区起点。固件一旦长过这里，链接器不会报错（地址是写死的），
 * 所以发布前务必看一次 map：FLASH 用量必须远小于 0x10000。 */
#define FLASH_STORE_BASE        (0x00010000u)

/* 扇区大小 = 擦除粒度（DL_FLASHCTL_SECTOR_SIZE） */
#define FLASH_STORE_SECTOR      (1024u)

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

#endif /* FLASH_STORE_H */
