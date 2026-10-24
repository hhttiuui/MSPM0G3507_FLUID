/*
 * glyph_store.h —— 自定义滚动字形的持久化（存 Flash，掉电保留）
 *
 * 数据来源：上位机逐字发来的 1 位字形（见 vcmd.c 的 ver=2/3/4 包）。
 * 上电时 GlyphStore_Load() 校验通过就把字形交给文字场景去拼接、滚动。
 *
 * ==================== 存储布局 ====================
 *   0x00010000  扇区 0：头部 16 B（其余 0xFF）
 *   0x00010400  扇区 1：字形数据（最多 1024 B）
 *
 *   字形数据格式（和 TextScene_ComposeGlyphs 的输入一致）：
 *     count × { w(1), h(1), bitmap[ceil(w/8) × h] }   行优先，MSB = 最左
 *
 * ==================== 掉电保护 ====================
 *   写入顺序：擦两个扇区 → 写数据 → 读回校验 → **最后写头部**。
 *   头部是"提交"动作：中途断电时头部仍是 0xFF，magic 不匹配 ⇒ 视为无数据，
 *   上电自动回退到内置字库，不会出现"半截字形"。
 */
#ifndef GLYPH_STORE_H
#define GLYPH_STORE_H

#include <stdint.h>
#include "flash_store.h"

#define GLYPH_STORE_HDR_ADDR    (FLASH_STORE_BASE)
#define GLYPH_STORE_DATA_ADDR   (FLASH_STORE_BASE + FLASH_STORE_SECTOR)
#define GLYPH_STORE_MAX_DATA    (FLASH_STORE_SECTOR)      /* 1 KB 足够 1 位 19 字 */

typedef struct {
    uint8_t  valid;         /* 1 = 头部校验通过 */
    uint8_t  count;         /* 字形个数 */
    uint8_t  spacing;       /* 字间距（像素，有符号：负 = 收紧，和上位机一致） */
    uint8_t  step;          /* 每帧滚动像素 */
    uint8_t  lev;           /* 亮度 */
    uint8_t  loop;          /* 1 = 循环滚动 */
    uint16_t ms;            /* 帧间隔 */
    uint16_t len;           /* 数据字节数 */
} glyph_meta_t;

/* 开始接收一轮（清空 RAM 缓冲，记下参数）。返回 1 = 成功。 */
uint8_t GlyphStore_Begin(uint8_t count, int8_t spacing, uint8_t step,
                         uint16_t ms, uint8_t lev, uint8_t loop);

/* 追加一个字形（[w][h][bitmap...]）。返回 1 = 成功，0 = 缓冲满/越界。 */
uint8_t GlyphStore_Append(const uint8_t *p, uint16_t len);

/* 落盘：擦 → 写 → 校验 → 写头部。返回 1 = 成功。 */
uint8_t GlyphStore_Commit(void);

/* 上电调用：读头部 → 校验 magic/CRC → 把数据搬到 RAM。返回 1 = 有效。 */
uint8_t GlyphStore_Load(void);

/* 当前是否有一份有效数据（已加载或刚提交） */
uint8_t GlyphStore_HasData(void);

/* 接收结束时校验：传入上位机算的 CRC，和内部滚动 CRC 比对 */
uint8_t GlyphStore_CrcOk(uint16_t crc);

/* 取当前（已加载或刚收完的）数据指针与长度 */
const uint8_t *GlyphStore_Data(void);
uint16_t       GlyphStore_DataLen(void);
const glyph_meta_t *GlyphStore_Meta(void);

#endif /* GLYPH_STORE_H */
