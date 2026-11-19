#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cut_log.py — 把很大的 xwing_vision / xwing_virtual log 只切出要分析的趟次 (方便傳檔)
═══════════════════════════════════════════════════════════════════
整次飛行只啟動一次程式時, CSV 會很大 (50 Hz × 每秒一列 × 一百多欄).
這支只留下指定趟次 GUIDED 前後幾秒的列, 欄位全部保留, 輸出成 zip.

用法 (在 xwing_plot 資料夾裡):
  python cut_log.py                                   # flight_logs 最新那份, 每一趟都切
  python cut_log.py xwing_vision_XXXX_flight1.csv     # 指定 log
  python cut_log.py xwing_vision_XXXX_flight1.csv --run 3 4
  python cut_log.py ... --pad 10                       # GUIDED 前後多留幾秒 (預設 5)

輸出: flight_logs/cut/<log 名>_cut.zip
      (裡面有 <log 名>_run3.csv, _run4.csv ... 加上 events.txt), 傳這個 zip 就好.
畫圖程式可以直接讀切出來的 csv.
"""
import argparse
import csv
import glob
import os
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def main():
    ap = argparse.ArgumentParser(description='切出指定趟次, 讓 log 變小')
    ap.add_argument('log', nargs='?', help='xwing_vision_*.csv / xwing_virtual_*.csv (不給 = flight_logs 最新那份)')
    ap.add_argument('--run', type=int, nargs='*', default=None, help='要切的趟次 (預設全部)')
    ap.add_argument('--pad', type=float, default=5.0, help='GUIDED 前後多留幾秒')
    ap.add_argument('--log-dir', default=os.path.join(ROOT, 'flight_logs'))
    a = ap.parse_args()

    log = a.log
    if not log:
        from plot_common import list_logs
        cs = list_logs(a.log_dir)
        if not cs:
            sys.exit(f'{a.log_dir} 裡沒有 log')
        log = max(cs, key=os.path.getmtime)
    elif not os.path.isfile(log):
        cand = os.path.join(a.log_dir, os.path.basename(log))
        if not os.path.isfile(cand):
            sys.exit(f'找不到 {log}')
        log = cand
    print(f'[cut] log: {log}  ({os.path.getsize(log) / 1e6:.1f} MB)')

    # 第一遍: 找每一趟 GUIDED 的時間範圍
    span = {}
    with open(log, newline='', encoding='utf-8-sig') as f:
        rd = csv.DictReader(f)
        head = rd.fieldnames
        for row in rd:
            try:
                t, g, r = float(row['t']), row.get('guided', '0'), int(float(row.get('run') or 0))
            except (TypeError, ValueError):
                continue
            if g in ('1', '1.0', 'True') and r > 0:
                lo, hi = span.get(r, (t, t))
                span[r] = (min(lo, t), max(hi, t))
    if not span:
        sys.exit('這份 log 沒有任何 GUIDED 段')
    runs = a.run or sorted(span)
    runs = [r for r in runs if r in span]
    if not runs:
        sys.exit(f'找不到指定的趟次; 這份 log 有: {sorted(span)}')

    stem = os.path.splitext(os.path.basename(log))[0]
    srcdir = os.path.dirname(os.path.abspath(log))
    outdir = os.path.join(srcdir, 'cut')          # 子資料夾: 畫圖 --all 不會重複畫到
    os.makedirs(outdir, exist_ok=True)
    outs = {}
    writers = {}
    for r in runs:
        p = os.path.join(outdir, f'{stem}_run{r}.csv')
        fh = open(p, 'w', newline='', encoding='utf-8')
        w = csv.DictWriter(fh, fieldnames=head)
        w.writeheader()
        outs[r] = (p, fh)
        writers[r] = w
    with open(log, newline='', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            try:
                t = float(row['t'])
            except (TypeError, ValueError):
                continue
            for r in runs:
                lo, hi = span[r]
                if lo - a.pad <= t <= hi + a.pad:
                    writers[r].writerow(row)
    for r in runs:
        outs[r][1].close()

    zp = os.path.join(outdir, f'{stem}_cut.zip')
    with zipfile.ZipFile(zp, 'w', zipfile.ZIP_DEFLATED) as z:
        for r in runs:
            z.write(outs[r][0], os.path.basename(outs[r][0]))
            print(f'[cut] run {r}: {span[r][1] - span[r][0]:6.1f} s GUIDED → '
                  f'{os.path.basename(outs[r][0])} ({os.path.getsize(outs[r][0]) / 1e6:.1f} MB)')
        ev = os.path.join(srcdir, f'{stem}_events.txt')
        if os.path.isfile(ev):
            z.write(ev, os.path.basename(ev))
        # 雲台 UART 傳輸診斷 (10/30 以後的飛行程式才有): 整份都放, 壓縮後通常只有幾 MB
        for p in sorted(glob.glob(os.path.join(srcdir, f'{stem}_uart_*'))
                        + glob.glob(os.path.join(srcdir, f'{stem}_dmesg_*.txt'))):
            z.write(p, os.path.basename(p))
            print(f'[cut] + {os.path.basename(p)} ({os.path.getsize(p) / 1e6:.1f} MB)')
    print(f'\n[cut] 完成 → {zp}  ({os.path.getsize(zp) / 1e6:.2f} MB)  ← 傳這個檔案')


if __name__ == '__main__':
    main()
