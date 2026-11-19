#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
car_gps_extract.py — 從車子的 .BIN 只抽出 GPS (+ MSG / MODE), 存成很小的 .json.gz (方便傳檔)
═══════════════════════════════════════════════════════════════════
車子 BIN 常常幾十 MB, 但比對真車 / 虛擬目標只需要 GPS (5 Hz, 幾千筆).
不需要 pymavlink, 只用 Python 內建模組 (Windows 筆電直接跑).

用法 (在 xwing_plot 資料夾):
  python car_gps_extract.py C:\\路徑\\00000012.BIN
  python car_gps_extract.py car_bin\\*.BIN            # 一次處理多個
輸出: 跟 BIN 同一個資料夾的 <BIN 名>_gps.json.gz   ← 傳這個檔案 (通常 < 1 MB)
"""
import glob
import gzip
import json
import os
import sys
import time

from bin_extract import scan


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    paths = []
    for a in sys.argv[1:]:
        paths += glob.glob(a) or [a]
    for p in paths:
        if not os.path.isfile(p):
            print(f'找不到 {p}')
            continue
        t0 = time.time()
        print(f'[car] 讀取 {p} ({os.path.getsize(p) / 1e6:.0f} MB) ...')
        with open(p, 'rb') as f:
            data = f.read()
        out = scan(data, {'GPS', 'MSG', 'MODE'})
        out['_source'] = os.path.basename(p)
        dst = os.path.splitext(p)[0] + '_gps.json.gz'
        with gzip.open(dst, 'wt', encoding='utf-8') as f:
            json.dump(out, f, separators=(',', ':'))
        print(f'   GPS {len(out["GPS"])} 筆, MSG {len(out["MSG"])}, MODE {len(out["MODE"])}')
        print(f'[car] 完成 ({time.time() - t0:.0f} s) → {dst}  ({os.path.getsize(dst) / 1e6:.2f} MB)  ← 傳這個檔案')


if __name__ == '__main__':
    main()
