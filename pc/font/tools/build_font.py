#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_font.py —— 从文泉驿点阵宋 BDF 生成「行对齐 1bpp」点阵字库

为什么要有这个脚本：
  单片机显示中文需要点阵字库，而 TTF 缩到 12px 基本糊成一团。
  文泉驿点阵宋是**手工调的 12×12 点阵**，笔画清晰，是这个尺寸下最好的选择。

输出（默认写在本目录的上一级）：
  wqy12_gb2312.bin   母库：ASCII + GB2312 全部汉字（PC 端裁子集用）
  wqy12_common.bin   常用库：ASCII + 574 个高频字 + 常用标点
  wqy12_common.h     常用库的 C 数组 + 二分查找函数（可直接贴进工程）
  wqy12_index.txt    码位 → 字符 → 步进 的对照表（人看的）

用法：
  python build_font.py                     # 用默认参数重建全部
  python build_font.py --bdf xxx.bdf       # 换字号（9pt=11×11 / 10pt=12×12 / 12pt=16×16）

数据格式（小端，全部按字节对齐，MCU 侧不需要任何移位跨字节）：

  文件头 16 字节
    0..3    "WQF1"
    4       单元宽 W        = 12
    5       单元高 H        = 14   （12 行字身 + 2 行下伸区）
    6       每行字节数 BPR  = ceil(W/8) = 2
    7       标志           = 0
    8..9    字形数 count   （u16）
    10      基线行         （ASCII 底部落在这一行，中文占 0..基线行）
    11      每字形字节数   = BPR × H
    12..15  保留

  码位表 count × 4 字节（u32，升序，可二分）
  步进表 count × 1 字节（每字形占几个像素宽）
  位图区 count × (BPR × H) 字节

  三个表顺序排列，第 i 个字形的位图偏移 = i * BPR * H（定长，不用存偏移）。
    逐行从上到下；每行 BPR 字节，最高位 = 最左像素。
    取像素：bit = bm[y*BPR + (x>>3)] & (0x80 >> (x&7))
"""
import argparse
import os
import re
import struct

# ---------------- BDF 解析 ----------------

def parse_bdf(path):
    """返回 (glyphs, ascent)。glyphs: {codepoint: dict(bw,bh,bx,by,dw,bmp)}"""
    glyphs = {}
    ascent = 12
    cur = None
    in_bmp = False

    with open(path, encoding='latin-1') as f:
        for line in f:
            line = line.rstrip('\n').rstrip('\r')

            if line.startswith('FONT_ASCENT'):
                ascent = int(line.split()[1])
                continue
            if line.startswith('STARTCHAR'):
                cur = {'bmp': []}
                in_bmp = False
                continue
            if cur is None:
                continue

            if line.startswith('ENCODING'):
                cur['enc'] = int(line.split()[1])
            elif line.startswith('BBX'):
                p = line.split()
                cur['bw'], cur['bh'], cur['bx'], cur['by'] = map(int, p[1:5])
            elif line.startswith('DWIDTH'):
                cur['dw'] = int(line.split()[1])
            elif line.startswith('BITMAP'):
                in_bmp = True
            elif line.startswith('ENDCHAR'):
                in_bmp = False
                if cur.get('enc', -1) >= 0 and cur['bmp']:
                    glyphs[cur['enc']] = cur
                cur = None
            elif in_bmp and re.fullmatch(r'[0-9A-Fa-f]+', line.strip()):
                cur['bmp'].append(line.strip())

    return glyphs, ascent


# ---------------- 字符集 ----------------

def charset_ascii():
    return set(range(32, 127))


def charset_gb2312():
    """GB2312 全集：用 Python 内置 codec 枚举，不依赖任何映射表。"""
    out = set()
    for hi in range(0xA1, 0xF8):
        for lo in range(0xA1, 0xFF):
            try:
                ch = bytes([hi, lo]).decode('gb2312')
            except UnicodeDecodeError:
                continue
            out.add(ord(ch))
    return out


def charset_from_map(path):
    """u8g2 的 .map 格式：十进制范围 32-128，十六进制单值 $4E00。"""
    out = set()
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.split('#')[0].strip().rstrip(',').strip()
            if not line:
                continue
            for tok in line.split(','):
                tok = tok.strip()
                if not tok:
                    continue
                if tok.startswith('$'):
                    out.add(int(tok[1:], 16))
                elif '-' in tok:
                    a, b = tok.split('-')
                    out.update(range(int(a), int(b) + 1))
                else:
                    out.add(int(tok))
    return out


# 中文排版最常用的一批标点（GB2312 里都有，但常用库要显式带上）
PUNCT = '，。、；：？！“”‘’（）《》【】…—·℃°％＃＠'


# ---------------- 栅格化 ----------------

def render_cell(g, cell_w, cell_h, bpr, base_row):
    """把一个 BDF 字形放进 cell_w × cell_h 的格子，返回 bpr*cell_h 字节。

    纵向：字形的底边贴在基线上，所以顶部行 = base_row - (by + bh)。
    横向：按 xoff 左对齐，超出格宽的部分裁掉（GB2312 里不会发生）。
    """
    out = bytearray(bpr * cell_h)
    top = base_row - (g['by'] + g['bh'])
    nbits = len(g['bmp'][0]) * 4 if g['bmp'] else 0   # 每行 hex 字符数 × 4 bit

    for i, hexrow in enumerate(g['bmp']):
        y = top + i
        if y < 0 or y >= cell_h:
            continue
        val = int(hexrow, 16) if hexrow else 0

        for x in range(g['bw']):
            cx = x + g['bx']
            if cx < 0 or cx >= cell_w:
                continue
            if (val >> (nbits - 1 - x)) & 1:
                out[y * bpr + (cx >> 3)] |= 0x80 >> (cx & 7)

    return bytes(out)


# ---------------- 打包 ----------------

MAGIC = b'WQF1'


def pack(glyphs_bdf, codes, cell_w, cell_h, base_row):
    bpr = (cell_w + 7) // 8
    codes = sorted(c for c in codes if c in glyphs_bdf)

    blobs = []
    advs = []
    for cp in codes:
        g = glyphs_bdf[cp]
        blobs.append(render_cell(g, cell_w, cell_h, bpr, base_row))
        advs.append(min(255, max(1, g.get('dw', cell_w))))

    hdr = bytearray(16)
    hdr[0:4] = MAGIC
    hdr[4] = cell_w
    hdr[5] = cell_h
    hdr[6] = bpr
    hdr[7] = 0
    struct.pack_into('<H', hdr, 8, len(codes))
    hdr[10] = base_row
    hdr[11] = bpr * cell_h

    cp_tab = b''.join(struct.pack('<I', cp) for cp in codes)
    adv_tab = bytes(advs)
    bm = b''.join(blobs)

    return bytes(hdr + cp_tab + adv_tab + bm), codes, advs, bpr


def emit_header(path, blob, codes, advs, cell_w, cell_h, bpr, name, note):
    """生成可直接编译的 C 头文件：数组 + 二分查找 + 取像素宏。"""
    base_row = blob[10]
    count = len(codes)
    # ⚠ 必须和 pack() 的布局一致：头 16 B + 码位表 count×4 + 步进表 count×1。
    #   这里原来是 count*8（早期"码位 u32 + 偏移 u16 + 步进 u8 + 填充"那版的
    #   残留），改格式后忘了同步 —— 结果整个位图数组错位 1797 字节，
    #   每个字形都指向别处，屏幕上是纯乱码。见 emit_header 末尾的自检。
    bm_off = 16 + count * 4 + count

    lines = []
    lines.append('/* %s' % note)
    lines.append(' *')
    lines.append(' * 由 build_font.py 自动生成，不要手改。')
    lines.append(' * 字库：文泉驿点阵宋（GPL v2 + 字体嵌入例外）')
    lines.append(' * 单元 %d×%d，基线行 %d，%d 个字形，共 %d 字节 Flash。'
                 % (cell_w, cell_h, base_row, count, len(blob)))
    lines.append(' *')
    lines.append(' * 用法：')
    lines.append(' *   const wqf_glyph_t *g = WQF_Find(0x4E2D);   // "中"')
    lines.append(' *   if (g) { x = 0;')
    lines.append(' *     for (px = 0; px < g->adv; px++)')
    lines.append(' *       if (WQF_PIXEL(g->bm, px, row)) LedMatrix_SetCellLevel(++x, row+1, 255);')
    lines.append(' *   }')
    lines.append(' */')
    lines.append('#ifndef %s_H' % name.upper())
    lines.append('#define %s_H' % name.upper())
    lines.append('')
    lines.append('#include <stdint.h>')
    lines.append('')
    lines.append('#define WQF_W        %d' % cell_w)
    lines.append('#define WQF_H        %d' % cell_h)
    lines.append('#define WQF_BPR      %d' % bpr)
    lines.append('#define WQF_BASE     %d' % base_row)
    lines.append('#define WQF_COUNT    %d' % count)
    lines.append('')
    lines.append('/* 取位图里 (x, y) 那个像素，返回 0 或非 0 */')
    lines.append('#define WQF_PIXEL(bm, x, y)  \\')
    lines.append('    ((bm)[(y) * WQF_BPR + ((x) >> 3)] & (0x80u >> ((x) & 7u)))')
    lines.append('')
    lines.append('typedef struct {')
    lines.append('    uint32_t cp;        /* 码位 */')
    lines.append('    uint8_t  adv;       /* 步进宽度（像素）*/')
    lines.append('    const uint8_t *bm;  /* 位图，WQF_BPR * WQF_H 字节 */')
    lines.append('} wqf_glyph_t;')
    lines.append('')

    # 码位表 + 步进表（分开存，省得每项都要 4 字节对齐）
    lines.append('static const uint32_t s_wqf_cp[WQF_COUNT] = {')
    for i in range(0, count, 10):
        lines.append('    ' + ', '.join('0x%04X' % c for c in codes[i:i+10]) + ',')
    lines.append('};')
    lines.append('')
    lines.append('static const uint8_t s_wqf_adv[WQF_COUNT] = {')
    for i in range(0, count, 24):
        lines.append('    ' + ', '.join('%3d' % a for a in advs[i:i+24]) + ',')
    lines.append('};')
    lines.append('')
    lines.append('/* 位图区：每字形 WQF_BPR*WQF_H = %d 字节 */' % (bpr * cell_h))
    lines.append('static const uint8_t s_wqf_bm[] = {')
    for i in range(0, len(blob) - bm_off, 16):
        chunk = blob[bm_off + i: bm_off + i + 16]
        lines.append('    ' + ','.join('0x%02X' % b for b in chunk) + ',')
    lines.append('};')
    lines.append('')
    lines.append('/* 二分查找。找不到返回 0，调用方要判空。 */')
    lines.append('static const wqf_glyph_t *WQF_Find(uint32_t cp)')
    lines.append('{')
    lines.append('    static wqf_glyph_t g;')
    lines.append('    uint16_t lo = 0u, hi = WQF_COUNT;')
    lines.append('')
    lines.append('    while (lo < hi) {')
    lines.append('        uint16_t mid = (uint16_t) ((lo + hi) >> 1);')
    lines.append('')
    lines.append('        if (s_wqf_cp[mid] == cp) {')
    lines.append('            g.cp  = cp;')
    lines.append('            g.adv = s_wqf_adv[mid];')
    lines.append('            g.bm  = &s_wqf_bm[mid * (WQF_BPR * WQF_H)];')
    lines.append('            return &g;')
    lines.append('        }')
    lines.append('        if (s_wqf_cp[mid] < cp) { lo = (uint16_t) (mid + 1u); }')
    lines.append('        else                    { hi = mid; }')
    lines.append('    }')
    lines.append('    return 0;')
    lines.append('}')
    lines.append('')
    lines.append('#endif /* %s_H */' % name.upper())
    lines.append('')

    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines))

    # ---- 回读自检 ----
    # 生成的 .h 里位图区必须和 blob 的位图区逐字节一致。上面那个 bm_off
    # 错位 bug 就是靠这个检查抓出来的：光"能编译"完全不够。
    verify_header(path, blob, bm_off, count, bpr * cell_h)


def verify_header(path, blob, bm_off, count, bpg):
    """回读 .h，确认位图数组和 blob 的位图区逐字节一致。"""
    import re as _re

    txt = open(path, encoding='utf-8').read()
    i = txt.index('s_wqf_bm[] = {')
    j = txt.index('\n};', i)
    got = bytes(int(t, 16) for t in _re.findall(r'0x([0-9A-Fa-f]{2})', txt[i:j]))
    want = blob[bm_off:bm_off + count * bpg]

    if got != want:
        raise SystemExit(
            '生成的字库头文件自检失败：位图数组 %d 字节，应为 %d 字节。\n'
            '  → 大概率是 emit_header 里的 bm_off 和 pack() 的布局不一致。'
            % (len(got), len(want)))

    # 顺手抽查第一个和最后一个字形
    for k in (0, count - 1):
        if got[k * bpg:(k + 1) * bpg] != want[k * bpg:(k + 1) * bpg]:
            raise SystemExit('字形 %d 的位图不一致' % k)

    print('  自检通过：位图 %d 字节逐字节一致' % len(got))


# ---------------- 主流程 ----------------

def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.dirname(here)

    ap = argparse.ArgumentParser()
    ap.add_argument('--bdf', default=os.path.join(here, '..', '_src', 'wenquanyi_10pt.bdf'),
                    help='BDF 源文件（默认 10pt = 12×12）')
    ap.add_argument('--map', default=os.path.join(here, '..', '_src', 'chinese2.map'),
                    help='常用字 .map（u8g2 格式）')
    ap.add_argument('--out', default=out_dir, help='输出目录')
    args = ap.parse_args()

    bdf_path = os.path.normpath(args.bdf)
    if not os.path.exists(bdf_path):
        raise SystemExit('找不到 BDF：%s\n先从 '
                         'https://github.com/larryli/u8g2_wqy/tree/master/bdf 下载'
                         % bdf_path)

    print('解析 %s ...' % os.path.basename(bdf_path))
    glyphs, ascent = parse_bdf(bdf_path)
    print('  %d 个字形，FONT_ASCENT = %d' % (len(glyphs), ascent))

    # 单元尺寸：宽度取 CJK 字宽；高度取「最高顶到最低底」的实际跨度，
    # 不猜常数 —— 换字号（9pt/10pt/12pt）时自动适配。
    cjk = [v for k, v in glyphs.items() if 0x4E00 <= k <= 0x9FFF]
    cell_w = max(v['bw'] for v in cjk) if cjk else 12
    base_row = ascent - 1             # 基线所在行（ASCII 底部落在这一行）

    used = charset_ascii() | charset_gb2312()
    span = [v for k, v in glyphs.items() if k in used]
    cell_h = max(base_row - v['by'] for v in span)

    print('  单元 %d×%d，基线行 %d' % (cell_w, cell_h, base_row))

    # ---- 母库：ASCII + GB2312 ----
    gb = charset_gb2312()
    master = charset_ascii() | gb
    blob, codes, advs, bpr = pack(glyphs, master, cell_w, cell_h, base_row)
    p = os.path.join(args.out, 'wqy12_gb2312.bin')
    with open(p, 'wb') as f:
        f.write(blob)
    print('母库  %-22s %6d 字形  %7d 字节' % (os.path.basename(p), len(codes), len(blob)))

    with open(os.path.join(args.out, 'wqy12_index.txt'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('# 码位\t字符\t步进\n')
        for cp, a in zip(codes, advs):
            f.write('U+%04X\t%s\t%d\n' % (cp, chr(cp) if cp >= 32 else ' ', a))

    # ---- 常用库：ASCII + 574 高频字 + 标点 ----
    common = charset_ascii() | {ord(c) for c in PUNCT}
    map_path = os.path.normpath(args.map)
    if os.path.exists(map_path):
        common |= charset_from_map(map_path)
    else:
        print('  （没找到 %s，常用库退化为 ASCII + 标点）' % os.path.basename(map_path))

    blob, codes, advs, bpr = pack(glyphs, common, cell_w, cell_h, base_row)
    with open(os.path.join(args.out, 'wqy12_common.bin'), 'wb') as f:
        f.write(blob)
    print('常用  wqy12_common.bin        %6d 字形  %7d 字节' % (len(codes), len(blob)))

    emit_header(os.path.join(args.out, 'wqy12_common.h'), blob, codes, advs,
                cell_w, cell_h, bpr, 'WQF12_COMMON',
                '12×12 常用中文点阵字库（ASCII + 574 高频字 + 常用标点）')

    print('完成。')

    # ---- 自检：把「中」「A」画出来看一眼 ----
    print('\n自检（. = 灭, # = 亮）：')
    for ch in '中A国1':
        g = glyphs.get(ord(ch))
        if not g:
            continue
        cell = render_cell(g, cell_w, cell_h, bpr, base_row)
        print('  %s  (adv=%d)' % (ch, g.get('dw', cell_w)))
        for y in range(cell_h):
            row = ''.join('#' if cell[y*bpr + (x >> 3)] & (0x80 >> (x & 7)) else '.'
                          for x in range(cell_w))
            print('    ' + row)


if __name__ == '__main__':
    main()
