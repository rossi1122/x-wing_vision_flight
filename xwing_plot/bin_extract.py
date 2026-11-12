#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bin_extract.py — 從飛控 .BIN 只抽出「轉彎分析」需要的資料, 存成很小的 .json.gz (方便傳檔)
═══════════════════════════════════════════════════════════════════
飛控 BIN 很大 (幾百 MB). 轉彎圖 (通過轉彎點 → 對正 + 拉平) 只需要:
  · MSG (Reached/Passed waypoint)、CMD (航點)、MODE   → 全部保留
  · POS / ATT / GPS / CTUN / NTUN                     → 只留每次經過航點前 10 s ~ 後 50 s
不需要 pymavlink, 只用 Python 內建模組 (Windows 筆電直接跑).

用法 (在 xwing_plot 資料夾):
  python bin_extract.py C:\\路徑\\00000005.BIN
  python bin_extract.py 00000005.BIN --before 10 --after 50
輸出: 跟 BIN 同一個資料夾的 <BIN 名>_turns.json.gz   ← 傳這個檔案
"""
import argparse
import gzip
import json
import os
import re
import struct
import sys
import time

FORMAT = dict(b='b', B='B', h='h', H='H', i='i', I='I', q='q', Q='Q', f='f', d='d',
              n='4s', N='16s', Z='64s', c='h', C='H', e='i', E='I', L='i', M='B', a='64s')
KEEP_ALL = {'MSG', 'CMD', 'MODE'}
KEEP_WIN = {'POS', 'ATT', 'GPS', 'CTUN', 'NTUN'}


def scan(data, wanted, window=None):
    """回傳 {名稱: [dict, ...]}; window=[(t0, t1), ...] (秒) 只用在 KEEP_WIN."""
    formats = {128: (89, 'FMT', '', [], None)}
    out = {k: [] for k in wanted}
    i, n = 0, len(data)
    while i + 3 <= n:
        if data[i] != 0xA3 or data[i + 1] != 0x95:
            i += 1
            continue
        typ = data[i + 2]
        f = formats.get(typ)
        if f is None:
            i += 1
            continue
        length, name, fmt, cols, st = f
        if length < 3 or i + length > n:
            i += 1 if length < 3 else n
            continue
        if name == 'FMT':
            payload = data[i + 3:i + length]
            if len(payload) == 86:
                t, size, rn, rf, rc = struct.unpack('<BB4s16s64s', payload)
                rn = rn.rstrip(b'\0').decode('ascii', 'ignore')
                rf = rf.rstrip(b'\0').decode('ascii', 'ignore')
                rc = rc.rstrip(b'\0').decode('ascii', 'ignore').split(',')
                try:
                    stc = struct.Struct('<' + ''.join(FORMAT[c] for c in rf))
                except KeyError:
                    stc = None
                formats[t] = (size, rn, rf, rc, stc)
            i += length
            continue
        if name in wanted and st is not None and st.size == length - 3:
            vals = st.unpack_from(data, i + 3)
            row = {}
            for v, c, k in zip(vals, fmt, cols):
                if c == 'L':
                    v = v * 1e-7
                elif c in 'cCeE':
                    v = v * 0.01
                elif c in 'nNZa':
                    v = v.rstrip(b'\0').decode('ascii', 'ignore')
                row[k] = v
            keep = True
            if window is not None and name in KEEP_WIN:
                tt = row.get('TimeUS', 0) / 1e6
                keep = any(a <= tt <= b for a, b in window)
            if keep:
                out[name].append(row)
        i += length
    return out


def main():
    ap = argparse.ArgumentParser(description='BIN → 轉彎分析用的小檔案')
    ap.add_argument('bin', help='飛控 .BIN')
    ap.add_argument('--before', type=float, default=10.0, help='每次經過航點前保留幾秒')
    ap.add_argument('--after', type=float, default=50.0, help='每次經過航點後保留幾秒')
    a = ap.parse_args()
    if not os.path.isfile(a.bin):
        sys.exit(f'找不到 {a.bin}')
    t_start = time.time()
    print(f'[bin] 讀取 {a.bin} ({os.path.getsize(a.bin) / 1e6:.0f} MB) ...')
    with open(a.bin, 'rb') as f:
        data = f.read()
    # 第 1 遍: 航點 / 模式訊息 (決定要留哪些時間段)
    first = scan(data, KEEP_ALL)
    wps = []
    for m in first['MSG']:
        if re.search(r'(Reached|Passed) waypoint #\d+', m.get('Message', '')):
            wps.append(m['TimeUS'] / 1e6)
    if not wps:
        sys.exit('BIN 裡沒有 Reached/Passed waypoint 訊息 (這份不是 AUTO 飛行?)')
    window = [(t - a.before, t + a.after) for t in wps]
    print(f'[bin] 經過航點 {len(wps)} 次, 抽出每次前 {a.before:g} s ~ 後 {a.after:g} s ...')
    # 第 2 遍: 時間段內的位置 / 姿態 / GPS / 空速
    second = scan(data, KEEP_WIN, window)
    out = dict(first, **second)
    out['_source'] = os.path.basename(a.bin)
    dst = os.path.splitext(a.bin)[0] + '_turns.json.gz'
    with gzip.open(dst, 'wt', encoding='utf-8') as f:
        json.dump(out, f, separators=(',', ':'))
    for k in sorted(KEEP_ALL | KEEP_WIN):
        print(f'   {k:5s} {len(out[k]):7d} 筆')
    print(f'\n[bin] 完成 ({time.time() - t_start:.0f} s) → {dst}  ({os.path.getsize(dst) / 1e6:.2f} MB)  ← 傳這個檔案')


if __name__ == '__main__':
    main()
