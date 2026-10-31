#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
subset_font.py —— 从母库裁出「只含你要显示的字」的最小字库

母库 wqy12_gb2312.bin 有 7539 个字形、243 KB，塞不进 128 KB 的 Flash。
但实际要显示的往往只有几十个字，裁完通常 2~5 KB。

用法：
  # 按文字裁（最常用）
  python subset_font.py --text "室内温度湿度" --out my_font.h

  # 按码位范围裁（比如只要全部数字和字母）
  python subset_font.py --range U+0030-U+0039,U+0041-U+005A --out abc.h

  # 顺带把 ASCII 全带上（省得漏字）
  python subset_font.py --text "温度25℃" --ascii --out my_font.h

  # 只统计不输出
  python subset_font.py --text "室内温度" --dry-run

输出的 .h 和 wqy12_common.h 结构完全一样，可以直接替换。
找不到的字会列出来（母库只有 GB2312 + ASCII，生僻字没有）。
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_font import emit_header          # noqa: E402


def load_bin(path):
    """读 .bin，返回 (cell_w, cell_h, bpr, base_row, codes, advs, bitmaps)。"""
    with open(path, 'rb') as f:
        blob = f.read()

    if blob[0:4] != b'WQF1':
        raise SystemExit('不是 WQF1 格式：%s' % path)

    cell_w = blob[4]
    cell_h = blob[5]
    bpr    = blob[6]
    count  = struct.unpack_from('<H', blob, 8)[0]
    base_row = blob[10]
    bpg    = blob[11]                      # 每字形字节数

    cp_off  = 16
    adv_off = cp_off + count * 4
    bm_off  = adv_off + count

    codes = [struct.unpack_from('<I', blob, cp_off + i * 4)[0] for i in range(count)]
    advs  = list(blob[adv_off:adv_off + count])
    bms   = [blob[bm_off + i * bpg: bm_off + (i + 1) * bpg] for i in range(count)]

    return cell_w, cell_h, bpr, base_row, codes, advs, bms


def parse_ranges(spec):
    """'U+4E00-U+4E10,U+0030' → 码位集合"""
    out = set()
    for part in spec.split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            a, b = part.split('-')
            out.update(range(int(a.replace('U+', ''), 16), int(b.replace('U+', ''), 16) + 1))
        else:
            out.add(int(part.replace('U+', ''), 16))
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)

    ap = argparse.ArgumentParser()
    ap.add_argument('--src', default=os.path.join(root, 'wqy12_gb2312.bin'),
                    help='母库 .bin')
    ap.add_argument('--text', default='', help='要包含的文字')
    ap.add_argument('--range', default='', help='码位范围，如 U+0030-U+0039')
    ap.add_argument('--ascii', action='store_true', help='额外带上全部 ASCII')
    ap.add_argument('--out', default='', help='输出的 .h')
    ap.add_argument('--name', default='', help='头文件的宏前缀（默认由文件名推）')
    ap.add_argument('--dry-run', action='store_true', help='只统计，不写文件')
    args = ap.parse_args()

    src = os.path.normpath(args.src)
    if not os.path.exists(src):
        raise SystemExit('找不到母库：%s\n先跑 build_font.py 生成。' % src)

    cell_w, cell_h, bpr, base_row, codes, advs, bms = load_bin(src)
    index = {cp: i for i, cp in enumerate(codes)}

    want = set()
    if args.text:
        want.update(ord(c) for c in args.text if c.strip())
    if args.range:
        want |= parse_ranges(args.range)
    if args.ascii:
        want |= set(range(32, 127))

    if not want:
        raise SystemExit('没指定任何字：用 --text 或 --range')

    # 缺字要报出来，不能静默丢掉 —— 否则上板才发现少字
    missing = sorted(cp for cp in want if cp not in index)
    have = sorted(cp for cp in want if cp in index)

    print('母库 %s：%d 字形，%d×%d' % (os.path.basename(src), len(codes), cell_w, cell_h))
    print('请求 %d 个码位 → 命中 %d 个' % (len(want), len(have)))
    if missing:
        print('⚠ 母库没有这 %d 个：%s' % (
            len(missing), ' '.join(chr(c) for c in missing[:40])))
        print('  （GB2312 之外的字需要换 12pt 的完整 BDF 重新生成母库）')

    if not have:
        raise SystemExit('一个都没命中')

    bpg = bpr * cell_h
    out_codes = have
    out_advs = [advs[index[cp]] for cp in have]
    out_bms = [bms[index[cp]] for cp in have]

    blob = bytearray(16)
    blob[0:4] = b'WQF1'
    blob[4] = cell_w
    blob[5] = cell_h
    blob[6] = bpr
    struct.pack_into('<H', blob, 8, len(out_codes))
    blob[10] = base_row
    blob[11] = bpg
    blob += b''.join(struct.pack('<I', cp) for cp in out_codes)
    blob += bytes(out_advs)
    blob += b''.join(out_bms)

    print('结果：%d 字形，位图 %d 字节，头+表 %d 字节，合计 %d 字节（%.1f KB）'
          % (len(out_codes), len(out_codes) * bpg, 16 + 5 * len(out_codes),
             len(blob), len(blob) / 1024))

    if args.dry_run:
        return

    out = args.out
    if not out:
        out = os.path.join(root, 'my_font.h')
    name = args.name or os.path.splitext(os.path.basename(out))[0].replace('-', '_').upper()

    emit_header(out, bytes(blob), out_codes, out_advs, cell_w, cell_h, bpr, name,
                '自定义中文点阵字库（%d 字，由 subset_font.py 生成）' % len(out_codes))
    print('已写出 %s' % out)


if __name__ == '__main__':
    main()
