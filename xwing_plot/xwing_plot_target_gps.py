#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_plot_target_gps.py — 虛擬目標 vs 真車 GPS 比較 (改寫自學長 plot_target_gps_comparison.py)
═══════════════════════════════════════════════════════════════════
用 GPS 時間 (UTC) 把車子 BIN 的 GPS 跟 xwing_vision / xwing_virtual log 的虛擬目標 (tgt_n/tgt_e) 對齊,
畫出「同一時刻兩者差多遠」. 看虛擬軌跡 (起點 / 方位 / 10 m/s / 撥 GUIDED 的時機) 跟
真車對不對得上.

與學長版的差異:
  · 讀 xwing_vision_*.csv (也收 B 的 xwing_virtual_*.csv); 只需要 numpy + matplotlib (不用 pandas)
  · 一份 log 有多趟 (run 1, 2 …) 時逐趟分開比較 (--run 指定單趟)
  · 預設路徑改成相對路徑, 不用改程式裡的 Windows 路徑
  · 也被 xwing_plot_report.py 呼叫 (--car-bin-dir), 一個指令出全部圖

用法:
  python3 xwing_plot_target_gps.py ../flight_logs/xwing_vision_XXXX_flight2_A.csv   (BIN 放 car_bin/)
  python3 xwing_plot_target_gps.py <log.csv> --bin-file car_bin/00000031.BIN --run 2

BIN 檔: 車子飛控 SD 卡 LOGS/ 裡的 .BIN. 程式自己掃 GPS 時間, 挑出完整涵蓋
PRE_DIVE~DIVE 的那一個 (只能有一個, 所以資料夾裡放當天的就好).
"""
import argparse
import csv
import glob
import os
import struct
import sys

import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

GPS_UTC_OFFSET_S = 18
EARTH_RADIUS_M = 6378137.0
GPS_EPOCH_UNIX = 315964800
FORMAT = dict(b='b', B='B', h='h', H='H', i='i', I='I', q='q', Q='Q',
              f='f', d='d', n='4s', N='16s', Z='64s', c='h', C='H',
              e='i', E='I', L='i', M='B', a='64s')


# ─────────────────────────── BIN ───────────────────────────
def read_bin_gps(path, leap_seconds=GPS_UTC_OFFSET_S, instance=None):
    """從 DataFlash .BIN 解出 GPS (用檔案內的 FMT 定義). 回傳 dict of numpy arrays,
    依 gps_unix 排序. 只取 3D fix 以上; 多顆 GPS 只取一顆 (預設 instance 0)."""
    with open(path, 'rb') as f:
        data = f.read()
    formats = {128: (89, 'FMT', '', [])}
    rec = []
    i = 0
    n = len(data)
    while i + 3 <= n:
        if data[i] != 0xA3 or data[i + 1] != 0x95 or data[i + 2] not in formats:
            i += 1
            continue
        length, name, fmt, cols = formats[data[i + 2]]
        if length < 3:
            i += 1
            continue
        if i + length > n:
            break
        payload = data[i + 3:i + length]
        i += length
        if name == 'FMT':
            if len(payload) != 86:
                continue
            typ, size, rn, rf, rc = struct.unpack('<BB4s16s64s', payload)
            formats[typ] = (size, rn.rstrip(b'\0').decode('ascii', 'ignore'),
                            rf.rstrip(b'\0').decode('ascii', 'ignore'),
                            rc.rstrip(b'\0').decode('ascii', 'ignore').split(','))
        elif name == 'GPS':
            try:
                vals = struct.unpack('<' + ''.join(FORMAT[c] for c in fmt), payload)
            except (KeyError, struct.error):
                continue
            vals = [v * (1e-7 if c == 'L' else .01 if c in 'cCeE' else 1)
                    for v, c in zip(vals, fmt)]
            rec.append(dict(zip(cols, vals)))
    need = ('GWk', 'GMS', 'Status', 'Lat', 'Lng', 'Spd')
    if not rec or not all(k in rec[0] for k in need):
        raise ValueError('沒有可用的 GPS week/time/position 訊息')
    g = {k: np.array([r.get(k, np.nan) for r in rec], float) for k in need}
    inst = np.array([r.get('I', 0) for r in rec], float)
    ok = (g['GWk'] > 0) & (g['Status'] >= 3) & (np.abs(g['Lat']) <= 90) & (np.abs(g['Lng']) <= 180)
    if ok.any():
        avail = np.unique(inst[ok])
        use = instance if instance is not None else (0 if 0 in avail else avail[0])
        ok &= inst == use
    if not ok.any():
        raise ValueError('沒有有效的 3D GPS 定位')
    g = {k: v[ok] for k, v in g.items()}
    g['gps_unix'] = GPS_EPOCH_UNIX + g['GWk'] * 604800 + g['GMS'] / 1000.0 - leap_seconds
    order = np.argsort(g['gps_unix'], kind='stable')
    g = {k: v[order] for k, v in g.items()}
    keep = np.r_[True, np.diff(g['gps_unix']) > 0]
    return {k: v[keep] for k, v in g.items()}


def find_bins(bin_dir=None, bin_files=None, recursive=False):
    paths = list(bin_files or [])
    if bin_dir:
        pat = os.path.join(bin_dir, '**', '*') if recursive else os.path.join(bin_dir, '*')
        paths += [p for p in glob.glob(pat, recursive=recursive)
                  if os.path.isfile(p) and p.lower().endswith('.bin')]
    return sorted(set(paths))


def load_bins(paths, leap_seconds=GPS_UTC_OFFSET_S, instance=None, quiet=False):
    bins = []
    for p in paths:
        try:
            g = read_bin_gps(p, leap_seconds, instance)
            bins.append((p, g))
            if not quiet:
                print(f"[BIN] {os.path.basename(p)}: {taiwan_time(g['gps_unix'][0])} ~ "
                      f"{taiwan_time(g['gps_unix'][-1])}  ({len(g['gps_unix'])} 筆 GPS)")
        except Exception as exc:  # noqa: BLE001
            print(f"[跳過 BIN] {os.path.basename(p)}: {exc}")
    return bins


def taiwan_time(t, ms=True):
    import datetime as _dt
    s = _dt.datetime.fromtimestamp(float(t), _dt.timezone(_dt.timedelta(hours=8)))
    return s.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3] if ms else s.strftime('%H:%M:%S')


def interpolate(x, y, times, max_gap):
    """線性內插; 不外插, 也不跨過超過 max_gap 秒的 GPS 斷點."""
    times = np.asarray(times, float)
    out = np.interp(times, x, y)
    right = np.clip(np.searchsorted(x, times, side='left'), 0, len(x) - 1)
    left = np.clip(right - 1, 0, len(x) - 1)
    exact = np.abs(x[right] - times) < 1e-6
    valid = (times >= x[0]) & (times <= x[-1]) & (exact | ((x[right] - x[left]) <= max_gap))
    out[~valid] = np.nan
    return out


# ─────────────────────────── log ───────────────────────────
def _num(rows, k):
    out = np.full(len(rows), np.nan)
    for i, r in enumerate(rows):
        v = r.get(k, '')
        try:
            out[i] = float(v)
        except (TypeError, ValueError):
            pass
    return out


def load_log_basic(path):
    with open(path, newline='', encoding='utf-8-sig') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('log 是空的')
    d = {k: _num(rows, k) for k in ('t', 'gps_unix', 'run', 'guided', 'lat', 'lon',
                                     'ac_n', 'ac_e', 'tgt_n', 'tgt_e')}
    d['mode'] = np.array([(r.get('mode') or '').strip().upper() for r in rows])
    if np.all(np.isnan(d['run'])):
        d['run'] = np.where(d['guided'] > 0, 1.0, 0.0)
    return d


def runs_in(d):
    g = (d['guided'] > 0) & (d['run'] > 0)
    return sorted(int(r) for r in np.unique(d['run'][g]))


# ─────────────────────────── 共用 (也給 xwing_plot_report 用) ───────────────────────────
def pick_bin(bins, seg0, seg1):
    """挑出唯一一個完整涵蓋 [seg0, seg1] (gps_unix) 的 BIN. 回傳 (path, gps dict)."""
    cands = []
    for p, g in bins:
        x = g['gps_unix']
        overlap = max(0.0, min(x[-1], seg1) - max(x[0], seg0))
        full = x[0] <= seg0 and x[-1] >= seg1
        if overlap > 0:
            cands.append((full, overlap, p, g))
    cands.sort(key=lambda c: (c[0], c[1]), reverse=True)
    if not cands or not cands[0][0]:
        raise ValueError(f'找不到完整涵蓋 PRE_DIVE~DIVE ({taiwan_time(seg0)} ~ '
                         f'{taiwan_time(seg1, ms=False)}) 的 BIN')
    if len(cands) > 1 and cands[1][0]:
        raise ValueError('多個 BIN 都涵蓋任務時間, 無法判定是哪一個: '
                         + ', '.join(os.path.basename(c[2]) for c in cands if c[0]))
    return cands[0][2], cands[0][3]


def log_origin(lat, lon, ac_n, ac_e):
    """log 自己的 NED 原點 (由 lat/lon 與 ac_n/ac_e 反推, 不用飛控 home)."""
    rad = np.pi / 180
    ok = np.isfinite(lat) & np.isfinite(lon) & np.isfinite(ac_n) & np.isfinite(ac_e)
    if ok.sum() < 2:
        raise ValueError('log 沒有經緯度')
    lat0 = float(np.median(lat[ok] - ac_n[ok] / EARTH_RADIUS_M / rad))
    lon0 = float(np.median(lon[ok] - ac_e[ok] / (EARTH_RADIUS_M * np.cos(lat0 * rad)) / rad))
    res = np.hypot((lat[ok] - lat0) * rad * EARTH_RADIUS_M - ac_n[ok],
                   (lon[ok] - lon0) * rad * EARTH_RADIUS_M * np.cos(lat0 * rad) - ac_e[ok])
    if np.quantile(res, .95) > .5:
        raise ValueError('log 的經緯度與 ac_n/ac_e 不是固定原點座標, 停止比較')
    return lat0, lon0


def car_ned(g, lat0, lon0):
    rad = np.pi / 180
    return ((g['Lat'] - lat0) * rad * EARTH_RADIUS_M,
            (g['Lng'] - lon0) * rad * EARTH_RADIUS_M * np.cos(lat0 * rad))


# ─────────────────────────── 比較 ───────────────────────────
def compare(d, run, bins, out_prefix, max_gap=2.0, after=10.0, leap_seconds=GPS_UTC_OFFSET_S,
            title=None, pdf=None):
    """d: load_log_basic 的結果; 只看 run 這一趟 (GUIDED 中的列).
    回傳 (summary_lines, stats dict). 失敗丟 ValueError (訊息就是原因)."""
    sel = (d['run'] == run) & (d['guided'] > 0)
    if sel.sum() < 2:
        raise ValueError(f'第 {run} 趟沒有 GUIDED 資料')
    gu = d['gps_unix']
    if np.isfinite(gu[sel]).sum() < 2 or np.nanmax(gu[sel]) < 1e9:
        raise ValueError('log 沒有 gps_unix (飛控 GPS 時間), 無法跟車子對時')
    mode = d['mode']
    dive = sel & (mode == 'DIVE') & np.isfinite(gu)
    pre = sel & (mode == 'PRE_DIVE') & np.isfinite(gu)
    if dive.sum() < 2:
        raise ValueError(f'第 {run} 趟沒有 DIVE (沒推頭), 不比較')
    if np.hypot(np.nanmax(d['tgt_n'][sel]) - np.nanmin(d['tgt_n'][sel]),
                np.nanmax(d['tgt_e'][sel]) - np.nanmin(d['tgt_e'][sel])) < .1:
        raise ValueError('虛擬目標沒有移動 (靜止版), 不比較')
    seg0 = gu[pre].min() if pre.any() else gu[dive].min()
    seg1 = gu[dive].max()
    bin_path, g = pick_bin(bins, seg0, seg1)

    lat0, lon0 = log_origin(d['lat'], d['lon'], d['ac_n'], d['ac_e'])
    car_n, car_e = car_ned(g, lat0, lon0)

    # 這一趟的時間窗: 目標開始動 ~ PULLOUT/DONE 後 after 秒 (仍在 GUIDED 內)
    idx = np.where(sel & np.isfinite(gu))[0]
    t_g = gu[idx]
    tn, te = d['tgt_n'][idx], d['tgt_e'][idx]
    m = mode[idx]
    moved = np.hypot(tn - tn[0], te - te[0]) > .05
    t_move = t_g[moved].min() if moved.any() else t_g[0]
    t_dive0 = gu[dive].min()
    t_dive1 = gu[dive].max()
    pull = sel & (mode == 'PULLOUT') & np.isfinite(gu)
    t_end = (gu[pull].max() if pull.any() else t_dive1) + after
    w = (t_g >= t_move - .02) & (t_g <= t_end) & (t_g >= g['gps_unix'][0]) & (t_g <= g['gps_unix'][-1])
    if w.sum() < 2:
        raise ValueError('可比較的時間太短')
    tt, tn, te, m = t_g[w], tn[w], te[w], m[w]
    rn = interpolate(g['gps_unix'], car_n, tt, max_gap)
    re_ = interpolate(g['gps_unix'], car_e, tt, max_gap)
    rs = interpolate(g['gps_unix'], g['Spd'], tt, max_gap)
    err_n, err_e = rn - tn, re_ - te
    dist = np.hypot(err_n, err_e)
    # 沿目標前進方向的前後差 (+ = 車子在虛擬目標前面), 往南 / 往北開都適用
    _un, _ue = tn[-1] - tn[0], te[-1] - te[0]
    _ul = np.hypot(_un, _ue)
    err_trk = (err_n * _un + err_e * _ue) / _ul if _ul > 0 else np.full_like(err_n, np.nan)
    hdg_deg = float(np.degrees(np.arctan2(_ue, _un)) % 360) if _ul > 0 else float('nan')
    if np.isfinite(dist).sum() < 2:
        raise ValueError('可比較的 GPS 點不足 (BIN 在這段時間有斷訊?)')
    rel = tt - t_dive0

    # 虛擬目標速度 (DIVE 段中位數, 由位置反推)
    dt = np.diff(tt)
    spd = np.hypot(np.diff(tn), np.diff(te)) / np.where(dt > 0, dt, np.nan)
    vs = np.nanmedian(spd[m[1:] == 'DIVE']) if (m[1:] == 'DIVE').any() else np.nanmedian(spd)
    v_virtual = round(float(vs), 1)

    fig = plt.figure(figsize=(12, 8), layout='constrained')
    gs = fig.add_gridspec(2, 2)
    a, c, e = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
    a.plot(te, tn, label='Virtual target', color='tab:blue', lw=2)
    a.plot(re_, rn, label='Real car GPS', color='tab:orange', lw=1.6)
    dm = np.where(m == 'DIVE')[0]
    if len(dm):
        ends = dm[[0, -1]]
        a.scatter(te[ends], tn[ends], color='tab:blue', s=45, zorder=3)
        a.scatter(re_[ends], rn[ends], color='tab:orange', marker='x', s=50, zorder=3)
        for k, lab in zip(ends, ('DIVE start', 'DIVE end')):
            a.annotate(lab, (te[k], tn[k]), textcoords='offset points', xytext=(8, 0), fontsize=8)
    a.scatter([0], [0], marker='*', s=150, color='k', zorder=4, label='Virtual start (origin)')
    a.set(xlabel='East (m)', ylabel='North (m)', title='(a) Ground track')
    a.axis('equal')
    c.plot(rel, dist, color='tab:purple', label='Horizontal separation')
    c.set(xlabel='Time from DIVE start (s)', ylabel='Distance (m)',
          title='(b) Same-time position difference (car − virtual)')
    e.plot(rel, rs, color='tab:orange', label='Real car GPS speed')
    e.axhline(v_virtual, color='tab:blue', ls='--', label=f'Virtual speed {v_virtual:g} m/s')
    e.set(xlabel='Time from DIVE start (s)', ylabel='Speed (m/s)', title='(c) Ground speed')
    for ax in (a, c, e):
        ax.grid(alpha=.25)
        ax.legend(loc='best', fontsize=8)
    for ax in (c, e):
        if pre.any():
            ax.axvspan(gu[pre].min() - t_dive0, 0, color='gold', alpha=.16)
        ax.axvspan(0, t_dive1 - t_dive0, color='tab:green', alpha=.12)
        ax.axvline(0, color='gray', ls=':', lw=1)
    fig.get_layout_engine().set(rect=(0, 0, .86, 1))
    fig.legend(handles=[Patch(facecolor='gold', alpha=.16, label='PRE_DIVE'),
                        Patch(facecolor='tab:green', alpha=.12, label='DIVE')],
               loc='upper left', bbox_to_anchor=(.865, .84), frameon=False, fontsize=11)
    fig.suptitle(f"Fig 14  Virtual target vs real car GPS — {title or ''}  run {run}  vs  {os.path.basename(bin_path)}\n"
                 f"GPS UTC alignment | DIVE {taiwan_time(t_dive0)} ~ "
                 f"{taiwan_time(t_dive1).split(' ')[-1]} (Taiwan time)", fontsize=11)
    fig.savefig(out_prefix + '.png', dpi=150)
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)

    with open(out_prefix + '_aligned.csv', 'w', newline='', encoding='utf-8-sig') as f:
        wr = csv.writer(f)
        wr.writerow(['gps_unix', 't_from_dive_s', 'mode', 'tgt_n', 'tgt_e', 'car_n', 'car_e',
                     'err_n', 'err_e', 'distance_error', 'car_speed'])
        for row in zip(tt, rel, m, tn, te, rn, re_, err_n, err_e, dist, rs):
            wr.writerow([f'{row[0]:.3f}', f'{row[1]:.3f}', row[2]] +
                        ['' if not np.isfinite(v) else f'{v:.2f}' for v in row[3:]])

    stats = {}
    lines = [f'BIN: {os.path.basename(bin_path)}',
             f'BIN GPS: {taiwan_time(g["gps_unix"][0])} ~ {taiwan_time(g["gps_unix"][-1])}',
             f'log 原點 (反推): lat={lat0:.7f}, lon={lon0:.7f}',
             f'對時: GPS UTC (閏秒 {leap_seconds} s), 線性內插, 沒有任何時間/位置平移;'
             f' GPS 斷點 > {max_gap} s 不內插',
             f'虛擬目標速度 {v_virtual:g} m/s']
    for ph in ('PRE_DIVE', 'DIVE', 'PULLOUT'):
        k = (m == ph) & np.isfinite(dist)
        if k.any():
            stats[ph] = dict(mean=float(dist[k].mean()), max=float(dist[k].max()),
                             rmse=float(np.sqrt(np.mean(dist[k] ** 2))),
                             v=float(np.nanmean(rs[k])),
                             along=float(np.nanmean(err_n[k])), cross=float(np.nanmean(err_e[k])),
                             track=float(np.nanmean(err_trk[k])))
            s = stats[ph]
            lines.append(f'{ph:<8s}: 平均差 {s["mean"]:6.1f} m  最大 {s["max"]:6.1f} m  '
                         f'RMSE {s["rmse"]:6.1f} m  (北向差 {s["along"]:+.1f}, 東向差 {s["cross"]:+.1f}) '
                         f' 車速平均 {s["v"]:.1f} m/s')
    if 'DIVE' in stats:
        s = stats['DIVE']
        # 沿目標前進方向: + = 車子在前面 (超前), − = 車子在後面 (落後)
        hint = ('車子落後虛擬目標' if s['track'] < -3 else
                '車子超前虛擬目標' if s['track'] > 3 else '前後大致對齊')
        lines.append(f'DIVE 段: {hint} (沿目標方位 {hdg_deg:.0f}° 的前後差 {s["track"]:+.1f} m, '
                     f'+ = 車子在前面)')
    with open(out_prefix + '_summary.txt', 'w', encoding='utf-8-sig') as f:
        f.write('\n'.join(lines) + '\n')
    return lines, stats


def main():
    ap = argparse.ArgumentParser(description='虛擬目標 vs 真車 GPS (BIN) 比較')
    ap.add_argument('log', help='xwing_vision_*.csv (主 log)')
    ap.add_argument('--bin-dir', default=None, help='車子 BIN 資料夾 (預設: xwing_plot/car_bin/)')
    ap.add_argument('--bin-file', action='append', default=[], help='直接指定 BIN (可多次)')
    ap.add_argument('--recursive', action='store_true', help='包含 BIN 子資料夾')
    ap.add_argument('--run', type=int, default=None, help='只比較第幾趟 (預設全部)')
    ap.add_argument('-o', '--outdir', default=None, help='輸出資料夾 (預設 xwing_plot/plot_results/<log 名>/)')
    ap.add_argument('--after', type=float, default=10, help='PULLOUT 後多畫幾秒')
    ap.add_argument('--max-gap', type=float, default=2, help='GPS 內插允許的最大斷點 (s)')
    ap.add_argument('--leap-seconds', type=int, default=GPS_UTC_OFFSET_S)
    ap.add_argument('--gps-instance', type=int, default=None, help='車子有兩顆 GPS 時指定 0/1')
    a = ap.parse_args()

    if not os.path.isfile(a.log):
        sys.exit(f'找不到 log: {a.log}')
    bin_dir = a.bin_dir
    if bin_dir is None and not a.bin_file:
        bin_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'car_bin')
    paths = find_bins(bin_dir, a.bin_file, a.recursive)
    if not paths:
        sys.exit(f'沒有 BIN 檔 (找的地方: {bin_dir or a.bin_file})')
    bins = load_bins(paths, a.leap_seconds, a.gps_instance)
    if not bins:
        sys.exit('沒有可用的 BIN')
    stem = os.path.splitext(os.path.basename(a.log))[0]
    outdir = a.outdir or os.path.join(os.path.dirname(os.path.abspath(__file__)), 'plot_results', stem)
    d = load_log_basic(a.log)
    runs = [a.run] if a.run else runs_in(d)
    if not runs:
        sys.exit('log 裡沒有任何 GUIDED 段')
    okn = 0
    for r in runs:
        od = os.path.join(outdir, f'run{r}')
        os.makedirs(od, exist_ok=True)
        try:
            lines, _ = compare(d, r, bins, os.path.join(od, 'fig14_target_gps'), a.max_gap,
                               a.after, a.leap_seconds, title=stem)
            okn += 1
            print(f'\n[OK] 第 {r} 趟 → {od}')
            print('\n'.join('     ' + x for x in lines))
        except ValueError as exc:
            print(f'[跳過] 第 {r} 趟: {exc}')
    print(f'\n完成 {okn}/{len(runs)} 趟')


if __name__ == '__main__':
    main()
