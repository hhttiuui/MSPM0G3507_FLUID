/*
 * flash_store.c —— 主 Flash 数据区读写
 *
 * 约束与依据见 flash_store.h 顶部（全部来自 TI 例程与 dl_flashctl.h）。
 */
#include <string.h>
#include "ti_msp_dl_config.h"
#include "flash_store.h"

/* 写一个 8 字节 flash word。
 * addr 必须 8 字节对齐；word 指向 8 字节源数据。
 *
 * ⚠ 源指针必须是 32 位对齐的：programMemory64 直接读 data[0]/data[1]，
 *   所以这里先 memcpy 到局部 uint32_t[2]（栈上是天然对齐的）。
 *
 * ⚠ 关中断的理由：命令在 RAM 中执行并轮询 STATCMD，期间 Flash 不可读。
 *   如果这时 TIMA0 中断进来，取向量/取指都会卡住。
 */
static uint8_t flash_write_word(uint32_t addr, const uint8_t *word)
{
    uint32_t            tmp[2];
    uint32_t            primask;
    DL_FLASHCTL_COMMAND_STATUS st;

    memcpy(tmp, word, 8u);

    primask = __get_PRIMASK();
    __disable_irq();

    /* 每条命令执行完控制器会自动重新加保护，所以每次都要先解锁 */
    DL_FlashCTL_unprotectSector(FLASHCTL, addr, DL_FLASHCTL_REGION_SELECT_MAIN);
    st = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(FLASHCTL, addr, tmp);

    if (primask == 0u) {
        __enable_irq();
    }

    return (st == DL_FLASHCTL_COMMAND_STATUS_PASSED) ? 1u : 0u;
}

uint8_t FlashStore_Erase(uint32_t addr, uint32_t nSectors)
{
    uint32_t i;

    for (i = 0u; i < nSectors; i++) {
        uint32_t                a = addr + (i * FLASH_STORE_SECTOR);
        uint32_t                primask;
        DL_FLASHCTL_COMMAND_STATUS st;

        primask = __get_PRIMASK();
        __disable_irq();

        DL_FlashCTL_unprotectSector(FLASHCTL, a, DL_FLASHCTL_REGION_SELECT_MAIN);
        st = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL, a,
                                            DL_FLASHCTL_COMMAND_SIZE_SECTOR);

        if (primask == 0u) {
            __enable_irq();
        }

        if (st != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
            return 0u;
        }
    }

    return 1u;
}

uint8_t FlashStore_Write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    uint8_t  word[8];
    uint32_t i;
    uint32_t n;

    for (i = 0u; i < len; i += 8u) {
        /* 组一个整字：不足 8 字节的尾部补 0xFF（Flash 擦除态） */
        for (n = 0u; n < 8u; n++) {
            word[n] = ((i + n) < len) ? data[i + n] : 0xFFu;
        }

        if (flash_write_word(addr + i, word) == 0u) {
            return 0u;
        }
    }

    return 1u;
}

uint8_t FlashStore_Verify(uint32_t addr, const uint8_t *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *) addr;
    uint32_t i;

    for (i = 0u; i < len; i++) {
        if (p[i] != data[i]) {
            return 0u;
        }
    }

    return 1u;
}

uint16_t FlashStore_Crc16(uint16_t crc, const uint8_t *p, uint32_t len)
{
    uint32_t i;
    uint8_t  b;

    for (i = 0u; i < len; i++) {
        crc ^= (uint16_t) ((uint16_t) p[i] << 8);

        for (b = 0u; b < 8u; b++) {
            if ((crc & 0x8000u) != 0u) {
                crc = (uint16_t) ((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t) (crc << 1);
            }
        }
    }

    return crc;
}
