#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
split_csv.py — 把很大的 CSV 切成幾份 (每份都有表頭), 方便傳檔
═══════════════════════════════════════════════════════════════════
只用 Python 內建模組. 依「檔案大小」平均切, 不會切在一列的中間.

用法 (在 xwing_plot 資料夾):
  python split_csv.py xwing_vision_XXXX.csv              # 切 3 份
  python split_csv.py xwing_vision_XXXX.csv --parts 4    # 切 4 份
  python split_csv.py xwing_vision_XXXX.csv --zip        # 每份再各自壓成 zip (更小)
找檔案: 先找你給的路徑, 找不到再去 x-wing_virtual\\flight_logs 找.
輸出: 跟原檔同一個資料夾的 split\\<原檔名>_part1of3.csv ...
"""
import argparse
import os
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def main():
    ap = argparse.ArgumentParser(description='大 CSV 切成幾份')
    ap.add_argument('csv')
    ap.add_argument('--parts', type=int, default=3, help='切幾份 (預設 3)')
    ap.add_argument('--zip', action='store_true', help='每份各自壓成 zip')
    a = ap.parse_args()
    src = a.csv
    if not os.path.isfile(src):
        src = os.path.join(ROOT, 'flight_logs', os.path.basename(a.csv))
    if not os.path.isfile(src):
        sys.exit(f'找不到 {a.csv}')
    if a.parts < 2:
        sys.exit('--parts 至少 2')

    total = os.path.getsize(src)
    target = total / a.parts
    stem = os.path.splitext(os.path.basename(src))[0]
    outdir = os.path.join(os.path.dirname(os.path.abspath(src)), 'split')
    os.makedirs(outdir, exist_ok=True)
    print(f'[split] {src}  ({total / 1e6:.1f} MB) → {a.parts} 份')

    outs = []
    with open(src, 'rb') as f:
        header = f.readline()
        k, fh, written = 0, None, 0
        for line in f:
            if fh is None or (written >= target * k and k < a.parts):
                if fh:
                    fh.close()
                k += 1
                p = os.path.join(outdir, f'{stem}_part{k}of{a.parts}.csv')
                fh = open(p, 'wb')
                fh.write(header)
                outs.append(p)
            fh.write(line)
            written += len(line)
        if fh:
            fh.close()

    for p in outs:
        if a.zip:
            zp = os.path.splitext(p)[0] + '.zip'
            with zipfile.ZipFile(zp, 'w', zipfile.ZIP_DEFLATED) as z:
                z.write(p, os.path.basename(p))
            os.remove(p)
            p = zp
        print(f'   {os.path.basename(p)}  {os.path.getsize(p) / 1e6:.1f} MB')
    print(f'\n[split] 完成 → {outdir}   ← 傳這幾個檔案')


if __name__ == '__main__':
    main()
