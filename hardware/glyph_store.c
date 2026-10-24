/*
 * glyph_store.c —— 自定义滚动字形的 Flash 持久化
 *
 * 布局与掉电保护见 glyph_store.h 顶部。
 * 底层读写全部走 flash_store（8 字节整字 + 关中断）。
 */
#include <string.h>
#include "ti_msp_dl_config.h"
#include "glyph_store.h"

#define GLYPH_MAGIC0        (0x47u)     /* 'G' */
#define GLYPH_MAGIC1        (0x4Cu)     /* 'L' */
#define GLYPH_VER           (1u)
#define GLYPH_HDR_LEN       (16u)

#define GLYPH_FLAG_LOOP     (0x01u)

/* RAM 工作缓冲：接收时攒在这里，校验通过再一次性落盘。
 * 1 KB —— 1 位字形下 19 个 12×12 字才 646 B，够用。
 * ⚠ 8 位字形（每字 16×16 = 256 B）放不下，那需要流式落盘，见 README 的后续计划。 */
static uint8_t      s_data[GLYPH_STORE_MAX_DATA];
static uint16_t     s_len;
static uint16_t     s_crc;          /* 接收过程中的滚动 CRC */
static glyph_meta_t s_meta;

/* ==================== 头部编解码 ==================== */

static void hdr_encode(uint8_t *h, uint16_t crc)
{
    h[0]  = GLYPH_MAGIC0;
    h[1]  = GLYPH_MAGIC1;
    h[2]  = GLYPH_VER;
    h[3]  = (uint8_t) ((s_meta.loop != 0u) ? GLYPH_FLAG_LOOP : 0u);
    h[4]  = s_meta.count;
    h[5]  = (uint8_t) s_meta.spacing;      /* int8 原样存（补码） */
    h[6]  = s_meta.step;
    h[7]  = s_meta.lev;
    h[8]  = (uint8_t) (s_meta.ms & 0xFFu);
    h[9]  = (uint8_t) (s_meta.ms >> 8);
    h[10] = (uint8_t) (s_meta.len & 0xFFu);
    h[11] = (uint8_t) (s_meta.len >> 8);
    h[12] = (uint8_t) (crc & 0xFFu);
    h[13] = (uint8_t) (crc >> 8);
    h[14] = 0u;
    h[15] = 0u;
}

static uint8_t hdr_decode(const uint8_t *h)
{
    if ((h[0] != GLYPH_MAGIC0) || (h[1] != GLYPH_MAGIC1) ||
        (h[2] != GLYPH_VER)) {
        return 0u;
    }

    s_meta.loop    = ((h[3] & GLYPH_FLAG_LOOP) != 0u) ? 1u : 0u;
    s_meta.count   = h[4];
    s_meta.spacing = (int8_t) h[5];
    s_meta.step    = h[6];
    s_meta.lev     = h[7];
    s_meta.ms      = (uint16_t) ((uint16_t) h[8] | ((uint16_t) h[9] << 8));
    s_meta.len     = (uint16_t) ((uint16_t) h[10] | ((uint16_t) h[11] << 8));
    s_crc          = (uint16_t) ((uint16_t) h[12] | ((uint16_t) h[13] << 8));

    if ((s_meta.count == 0u) || (s_meta.len == 0u) ||
        (s_meta.len > GLYPH_STORE_MAX_DATA)) {
        return 0u;
    }

    return 1u;
}

/* ==================== 接收 ==================== */

uint8_t GlyphStore_Begin(uint8_t count, int8_t spacing, uint8_t step,
                         uint16_t ms, uint8_t lev, uint8_t loop)
{
    if (count == 0u) {
        return 0u;
    }

    s_len         = 0u;
    s_crc         = 0xFFFFu;
    s_meta.valid  = 0u;
    s_meta.count  = count;
    s_meta.spacing = spacing;
    s_meta.step   = step;
    s_meta.ms     = ms;
    s_meta.lev    = lev;
    s_meta.loop   = (loop != 0u) ? 1u : 0u;

    return 1u;
}

uint8_t GlyphStore_Append(const uint8_t *p, uint16_t len)
{
    if ((p == 0) || (len == 0u)) {
        return 0u;
    }
    if ((uint32_t) s_len + (uint32_t) len > (uint32_t) GLYPH_STORE_MAX_DATA) {
        return 0u;                       /* 放不下：整轮作废，不写 Flash */
    }

    memcpy(&s_data[s_len], p, len);
    s_crc = FlashStore_Crc16(s_crc, p, (uint32_t) len);
    s_len = (uint16_t) (s_len + len);

    return 1u;
}

uint8_t GlyphStore_Commit(void)
{
    uint8_t  hdr[GLYPH_HDR_LEN];
    uint8_t  ok;

    if ((s_len == 0u) || (s_meta.count == 0u)) {
        return 0u;
    }

    s_meta.len = s_len;
    hdr_encode(hdr, s_crc);

    /* ① 擦头部 + 数据两个扇区 */
    if (FlashStore_Erase(GLYPH_STORE_HDR_ADDR, 2u) == 0u) {
        return 0u;
    }

    /* ② 写数据 */
    if (FlashStore_Write(GLYPH_STORE_DATA_ADDR, s_data, (uint32_t) s_len) == 0u) {
        return 0u;
    }

    /* ③ 读回校验 */
    if (FlashStore_Verify(GLYPH_STORE_DATA_ADDR, s_data, (uint32_t) s_len) == 0u) {
        return 0u;
    }

    /* ④ 最后写头部（提交动作） */
    ok = FlashStore_Write(GLYPH_STORE_HDR_ADDR, hdr, (uint32_t) GLYPH_HDR_LEN);
    if (ok == 0u) {
        return 0u;
    }

    s_meta.valid = 1u;
    return 1u;
}

/* ==================== 上电加载 ==================== */

uint8_t GlyphStore_Load(void)
{
    const uint8_t *h = (const uint8_t *) GLYPH_STORE_HDR_ADDR;
    uint8_t  hdr[GLYPH_HDR_LEN];
    uint16_t crc;

    /* 直接读 Flash（内存映射）。先拷到 RAM 再解析：Cortex-M0+ 不支持
     * 非对齐访问，按字节解析最稳。 */
    memcpy(hdr, h, GLYPH_HDR_LEN);

    s_meta.valid = 0u;
    if (hdr_decode(hdr) == 0u) {
        return 0u;
    }

    memcpy(s_data, (const void *) GLYPH_STORE_DATA_ADDR, s_meta.len);

    crc = FlashStore_Crc16(0xFFFFu, s_data, (uint32_t) s_meta.len);
    if (crc != s_crc) {
        return 0u;                       /* 数据损坏：当作没有 */
    }

    s_len        = s_meta.len;
    s_meta.valid = 1u;

    return 1u;
}

const uint8_t *GlyphStore_Data(void)
{
    return s_data;
}

uint8_t GlyphStore_HasData(void)
{
    return (s_meta.valid != 0u) ? 1u : 0u;
}

uint8_t GlyphStore_CrcOk(uint16_t crc)
{
    return (crc == s_crc) ? 1u : 0u;
}

uint16_t GlyphStore_DataLen(void)
{
    return s_len;
}

const glyph_meta_t *GlyphStore_Meta(void)
{
    return &s_meta;
}
