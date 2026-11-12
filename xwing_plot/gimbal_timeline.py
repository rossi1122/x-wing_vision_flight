#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gimbal_timeline.py — 整份 log 裡雲台 UART 什麼時候有資料、那時飛機在地上還是天上
═══════════════════════════════════════════════════════════════════
只用 Python 內建模組, 讀完整的 xwing_vision_*.csv (不是 cut 出來的).

用法 (在 xwing_plot 資料夾):
  python gimbal_timeline.py                         # flight_logs 最新那份
  python gimbal_timeline.py xwing_vision_XXXX.csv
輸出: 螢幕 + <log 名>_gimbal_timeline.txt (跟 log 同資料夾)

判斷「天上」: AGL > 20 m 且 空速 > 12 m/s (只是估計; 準確的起降看飛控 BIN 的 armed/disarmed).
"""
import argparse
import csv
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def fnum(x):
    try:
        v = float(x)
        return v if v == v else None
    except (TypeError, ValueError):
        return None


def where(agl, v):
    if agl is None or v is None:
        return '?'
    return '天上' if agl > 20 and v > 12 else '地上'


def main():
    ap = argparse.ArgumentParser(description='雲台 UART 有資料的時段')
    ap.add_argument('log', nargs='?')
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
        log = os.path.join(a.log_dir, os.path.basename(log))
    if not os.path.isfile(log):
        sys.exit(f'找不到 {a.log}')

    segs = []          # 雲台 link 段: [t0, t1, row0, row1, frames0, frames1, modes]
    air = []           # 天上 / 地上 段: [狀態, t0, t1]
    runs = {}          # 趟次 → (t0, t1)
    cur = None
    last = None
    with open(log, newline='', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            t = fnum(row.get('t'))
            if t is None:
                continue
            last = (t, row)
            w = where(fnum(row.get('agl')), fnum(row.get('V')))
            if w != '?':
                if air and air[-1][0] == w:
                    air[-1][2] = t
                else:
                    air.append([w, t, t])
            r = int(fnum(row.get('run')) or 0)
            if row.get('guided') in ('1', '1.0', 'True') and r > 0:
                lo, hi = runs.get(r, (t, t))
                runs[r] = (min(lo, t), max(hi, t))
            link = fnum(row.get('vis_link')) == 1
            fr = fnum(row.get('vis_frames'))
            md = row.get('vis_gimbal_mode', '')
            if link:
                if cur is None:
                    cur = [t, t, row, row, fr, fr, set()]
                    segs.append(cur)
                cur[1], cur[3], cur[5] = t, row, fr
                if md not in ('', 'nan'):
                    cur[6].add(md)
            else:
                cur = None
    if last is None:
        sys.exit('log 是空的')
    # 很短的天上/地上跳動 (< 3 s) 合併掉
    merged = []
    for s in air:
        if merged and (s[2] - s[1] < 3 or merged[-1][0] == s[0]):
            merged[-1][2] = s[2]
        else:
            merged.append(s)

    out = [f'log: {log}', f'程式時長 {last[0]:.1f} s', '']
    out.append('【雲台 UART 有資料的時段】 (vis_link = 1)')
    if not segs:
        out.append('  整份 log 都沒有收到雲台資料')
    for s in segs:
        t0, t1, r0, r1, f0, f1, md = s
        fps = ((f1 - f0) / (t1 - t0)) if (f0 is not None and f1 is not None and t1 > t0) else None
        out.append(f'  {t0:8.1f} ~ {t1:8.1f} s ({t1 - t0:7.1f} s)'
                   + (f'  {fps:4.1f} 幀/s' if fps else '')
                   + f'  雲台模式 {",".join(sorted(md)) or "—"}')
        for tag, rr in (('開始', r0), ('結束', r1)):
            agl, v = fnum(rr.get('agl')), fnum(rr.get('V'))
            out.append(f'      {tag}: {where(agl, v)}  AGL {agl if agl is None else round(agl)} m  '
                       f'空速 {v if v is None else round(v, 1)} m/s  飛控 {rr.get("fc_mode", "")}  '
                       f'電壓 {rr.get("batt_v", "")}')
    out.append('')
    out.append('【飛機 天上 / 地上】 (AGL > 20 m 且 空速 > 12 m/s 算天上)')
    for w, t0, t1 in merged:
        out.append(f'  {t0:8.1f} ~ {t1:8.1f} s  {w}')
    out.append('')
    out.append('【每趟 GUIDED 期間 雲台有沒有資料】')
    for r in sorted(runs):
        lo, hi = runs[r]
        cov = sum(max(0.0, min(hi, s[1]) - max(lo, s[0])) for s in segs)
        out.append(f'  run {r}: {lo:8.1f} ~ {hi:8.1f} s  雲台有資料 {100 * cov / max(hi - lo, 1e-6):5.1f} %')
    txt = '\n'.join(out)
    print(txt)
    dst = os.path.splitext(log)[0] + '_gimbal_timeline.txt'
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(txt + '\n')
    print(f'\n→ {dst}')


if __name__ == '__main__':
    main()
