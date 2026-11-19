#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_plot_report.py — xwing_vision 實飛 log → 報告圖 + 自動檢查 (飛完直接給老師看)
═══════════════════════════════════════════════════════════════════
參考學長 plot_flight_report / plot_moving_report 的圖表配置, 改成讀 xwing_vision_*.csv (B 的 xwing_virtual_*.csv 也能畫)
(沒有 YOLO / _vision.csv / real_* 欄位, 改畫虛擬目標 PID、方向舵、雲台 UART 紀錄).
只需要 numpy + matplotlib.

用法 (在 xwing_plot/ 資料夾裡執行; 車子 BIN 放 xwing_plot/car_bin/ 會自動比對):
  python3 xwing_plot_report.py --all                          # ../flight_logs/ 裡每一份都畫
  python3 xwing_plot_report.py --latest                       # flight_logs/ 裡最新的一份
  python3 xwing_plot_report.py flight_logs/xwing_vision_20261112_xxxxxx_flight2.csv
  python3 xwing_plot_report.py <log.csv> --car-bin-dir <資料夾>   # BIN 放在別的地方時
  python3 xwing_plot_report.py <log1.csv> <log2.csv>          # 一次畫好幾份
  python3 xwing_plot_report.py <log.csv> --run 2              # 只畫第 2 趟
  python3 xwing_plot_report.py <log.csv> --full               # 不裁切, 畫整份 (地面測試用)

輸出: xwing_plot/plot_results/<log 檔名>/run<N>/
  Fig 1~9 照學長 realflight_plot 的版面 (論文用):
  fig1_image_error.png            影像平面誤差 (% of half-frame), DIVE
  fig2_attitude.png               pitch / 航向 (相對固定方位) / roll, 命令 vs 實際
  fig3_altitude_profile.png       (a) AGL vs 時間 (b) 慣性座標剖面 (c) DIVE 中與目標距離
  fig4_glide_slope.png            航跡角 vs 理想下滑角
  fig4b_gimbal_detection_rate.png 各階段雲台偵測率 (有雲台才有)
  fig5_pid_replay_gimbal.png      同一套 PID: 虛擬目標 vs 雲台資料 (DIVE 有雲台資料才有)
  fig6_airspeed.png               空速 + 各階段統計表
  fig7_control_loop_error.png     DIVE 控制迴路誤差 (影像 → 姿態追蹤)
  fig8_gimbal_attitude.png        雲台 pitch / roll / 模式 (有雲台才有)
  fig9_dive_trajectory.png        DIVE 飛機與目標軌跡 (目標方向 = Y 軸, 每秒機頭箭頭)
  X-wing 才有的診斷圖:
  fig10_rudder.png                方向舵 raw / cmd / sent, 限幅, 偏航角速度
  fig11_pid_terms.png             DIVE 段 PID 各項
  fig12_predicted_miss.png        預測落點誤差
  fig13_system.png                電壓、控制迴路週期
  fig14_target_gps.png            (car_bin 有 BIN) 虛擬目標 vs 真車 GPS
  fig15_gimbal_vs_car.png         (DIVE 有雲台資料) 雲台 vs 虛擬 vs 真車 拆解, 偵測與延遲
  report.pdf                      上面全部合成一份
  summary.txt                     時間線 + 數字 + 自動檢查 (⚠ = 建議看的參數)
"""
import argparse
import csv
import glob
import math
import os
import re
import sys

import numpy as np
import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Patch

HERE = os.path.dirname(os.path.abspath(__file__))       # xwing_plot/
ROOT = os.path.dirname(HERE)                             # 專案根目錄 (config / dive_guidance / flight_logs)
sys.path[:0] = [HERE, ROOT]
plt.rcParams['axes.unicode_minus'] = False

try:
    from xwing_vision_config import XwingVisionConfig
    CFG = XwingVisionConfig()
except Exception:  # noqa: BLE001
    try:                                     # B 的專案 (hsu_realflight_true)
        from xwing_virtual_config import XwingVirtualConfig
        CFG = XwingVirtualConfig()
    except Exception:  # noqa: BLE001  (單獨拷到筆電也能跑, 用下面的預設值)
        CFG = None
from plot_common import list_logs, latest_log

DEFAULTS = dict(CRUISE_AGL_M=240.0, DIVE_ANGLE_DEG=30.0, PULLOUT_AGL_M=140.0,
                PULLOUT_VMAX_MPS=60.0, ENTRY_MIN_AGL_M=200.0, ABORT_MIN_AGL_M=100.0,
                V_CRUISE_MPS=30.0, PREDIVE_HANDOFF_PITCH_TOL_DEG=3.0,
                ROLL_PROT_FULL_DEG=25.0, ROLL_PROT_ZERO_DEG=35.0, RUDDER_LIMIT_DEG=30.0,
                IMG_W=1920, IMG_H=1080, FX_1X=1066.7, FY_1X=1059.5,
                CX_1X=959.4932, CY_1X=545.0292, GIMBAL_LOCK_PITCH_DEG=-30.0,
                GIMBAL_LOCK_TOL_DEG=2.0, PID_LAT_KP=2.0, PID_LAT_KI=0.2, PID_LAT_KD=0.0,
                PID_LON_KP=0.8, PID_LON_KI=0.1, PID_LON_KD=0.2, K_LAT_TO_RUDDER=0.5)


def C(name):
    if CFG is not None and hasattr(CFG, name):
        return getattr(CFG, name)
    return DEFAULTS[name]


MODE_COLOR = {'SEARCH': '#4A90E2', 'PRE_DIVE': '#F5A623', 'DIVE': '#E4572E',     # 跟學長圖同色
              'PULLOUT': '#8E44AD', 'DONE': '#95A5A6', 'ABORT': '#D0021B'}
C_CMD, C_ACT, C_REF = '#D0021B', '#1F4E79', '#555555'
C_TGT = '#D0021B'


# ─────────────────────────── log 讀取 ───────────────────────────
class Log:
    """CSV → 欄位字串陣列; num() 轉 float (空白/文字 = NaN)."""

    def __init__(self, cols):
        self.cols = cols
        self._num = {}
        self.n = len(next(iter(cols.values()))) if cols else 0

    @classmethod
    def load(cls, path):
        with open(path, newline='', encoding='utf-8-sig') as f:
            rd = csv.reader(f)
            head = next(rd)
            rows = [r for r in rd if r]
        cols = {}
        for j, k in enumerate(head):
            cols[k.strip()] = np.array([r[j].strip() if j < len(r) else '' for r in rows], dtype=object)
        return cls(cols)

    def has(self, k):
        return k in self.cols and np.any(np.isfinite(self.num(k)))

    def num(self, k):
        if k not in self._num:
            out = np.full(self.n, np.nan)
            if k in self.cols:
                for i, v in enumerate(self.cols[k]):
                    if v in ('', 'None', 'nan'):
                        continue
                    if v in ('True', 'true'):
                        out[i] = 1.0
                    elif v in ('False', 'false'):
                        out[i] = 0.0
                    else:
                        try:
                            out[i] = float(v)
                        except ValueError:
                            pass
            self._num[k] = out
        return self._num[k]

    def s(self, k):
        if k not in self.cols:
            return np.array([''] * self.n, dtype=object)
        return self.cols[k]

    def sub(self, mask):
        L = Log({k: v[mask] for k, v in self.cols.items()})
        L._num = {k: v[mask] for k, v in self._num.items()}
        return L


def read_events(csv_path):
    p = csv_path[:-4] + '_events.txt' if csv_path.endswith('.csv') else None
    if p and not os.path.isfile(p):
        # cut_log.py 切出來的 <log>_runN.csv → 用原本 <log>_events.txt
        p2 = re.sub(r'_run\d+_events\.txt$', '_events.txt', p)
        p = p2 if os.path.isfile(p2) else None
    if not p or not os.path.isfile(p):
        return []
    # Orin (Linux) 寫的是 UTF-8; 在 Windows 跑 test_plot_report.py 時飛行程式用系統編碼 (cp950) 寫
    with open(p, 'rb') as f:
        raw = f.read()
    text = None
    for enc in ('utf-8', 'cp950'):
        try:
            text = raw.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    if text is None:
        text = raw.decode('utf-8', errors='replace')
    out = []
    for ln in text.splitlines():
        m = re.match(r'\s*([\d.]+)s\s+(.*)', ln)
        if m:
            out.append((float(m.group(1)), m.group(2)))
    return out


# ─────────────────────────── 小工具 ───────────────────────────
def wrap180(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def rmse(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.sqrt(np.mean(x ** 2))) if len(x) else float('nan')


def nanmean(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(x.mean()) if len(x) else float('nan')


def nanmax(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(x.max()) if len(x) else float('nan')


def nanmin(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(x.min()) if len(x) else float('nan')


def f1(x, nd=1, unit=''):
    return '—' if x is None or not np.isfinite(x) else f'{x:.{nd}f}{unit}'


def mode_segments(t, mode):
    segs = []
    if len(t) == 0:
        return segs
    s = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or mode[i] != mode[s]:
            e = t[i] if i < len(t) else t[-1]
            segs.append((mode[s], t[s], e))
            s = i
    return segs


def shade_modes(ax, segs):
    for m, a, b in segs:
        c = MODE_COLOR.get(m)
        if c:
            ax.axvspan(a, b, color=c, alpha=.10, lw=0)


def mode_legend(fig, segs, y=.995):
    seen = []
    for m, _, _ in segs:
        if m in MODE_COLOR and m not in seen:
            seen.append(m)
    if seen:
        fig.legend(handles=[Patch(facecolor=MODE_COLOR[m], alpha=.25, label=m) for m in seen],
                   loc='upper right', ncol=len(seen), fontsize=8, frameon=False,
                   bbox_to_anchor=(.99, y))


def stat_box(ax, text, loc='upper left'):
    xy = {'upper left': (.01, .97, 'left', 'top'), 'upper right': (.99, .97, 'right', 'top'),
          'lower left': (.01, .03, 'left', 'bottom'), 'lower right': (.99, .03, 'right', 'bottom')}[loc]
    ax.text(xy[0], xy[1], text, transform=ax.transAxes, ha=xy[2], va=xy[3], fontsize=8,
            family='monospace', bbox=dict(boxstyle='round', fc='white', ec='#BBBBBB', alpha=.85))


# ─────────────────────────── 單趟資料整理 ───────────────────────────
class Run:
    def __init__(self, L, run, full=False, pre=3.0, post=3.0):
        t = L.num('t')
        g = L.num('guided') > 0
        r = L.num('run') if L.has('run') else np.where(g, 1.0, 0.0)
        self.run = run
        if full:
            sel = np.ones(L.n, bool)
            self.t_on = float(t[g][0]) if g.any() else float(t[0])
        else:
            mine = (r == run)
            gm = mine & g
            if not gm.any():
                raise ValueError(f'第 {run} 趟沒有 GUIDED 資料')
            self.t_on = float(t[gm][0])
            t_off = float(t[gm][-1])
            t_last = float(t[mine][-1])
            sel = (t >= self.t_on - pre) & (t <= min(t_off + post, t_last))
        self.L = L.sub(sel)
        self.t = self.L.num('t') - self.t_on          # 0 = 撥 GUIDED
        self.mode = np.array([m if gg > 0 else '' for m, gg in
                              zip(self.L.s('mode'), self.L.num('guided'))], dtype=object)
        self.mode_raw = self.L.s('mode')
        self.segs = [s for s in mode_segments(self.t, self.mode) if s[0]]
        # 各模式第一次出現的時間 (GUIDED 中)
        self.first = {}
        for m, a, _ in self.segs:
            self.first.setdefault(m, a)

    def m(self, *modes):
        return np.isin(self.mode, modes)

    def window(self, pad0=2.0, pad1=3.0):
        """PRE_DIVE 前 pad0 秒 ~ PULLOUT/ABORT 後 pad1 秒; 沒推頭就整段."""
        a = self.first.get('PRE_DIVE', self.first.get('DIVE'))
        if a is None:
            return self.t[0], self.t[-1]
        b = None
        for k in ('DONE', 'ABORT'):
            if k in self.first:
                b = self.first[k] if b is None else min(b, self.first[k])
        if b is None:
            b = self.t[-1]
        return a - pad0, min(b + pad1, self.t[-1])


# ─────────────────────────── 圖 ───────────────────────────
def _save(fig, path, pdf):
    fig.savefig(path, dpi=140)
    if pdf is not None:
        pdf.savefig(fig)
    plt.close(fig)


def _osc_rate(t, e, dead=0.3):
    """|e| > dead 的過零次數 / 秒 (大略代表擺盪頻率 ×2)."""
    ok = np.isfinite(e) & (np.abs(e) > dead)
    if ok.sum() < 5 or (t[-1] - t[0]) <= 0:
        return 0.0
    sg = np.sign(e[ok])
    return float(np.sum(sg[1:] != sg[:-1]) / (t[-1] - t[0]) / 2.0)


def fig5_pid(R, path, title, pdf):
    L, t = R.L, R.t
    a0, a1 = R.window(1.0, 1.0)
    w = (t >= a0) & (t <= a1)
    dn = np.where(R.m('DIVE'), 1.0, np.nan)
    fig, ax = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True, layout='constrained')
    for a, pre, out, lab, gains in (
            (ax[0], 'pid_lat', 'pid_dpsi_deg', 'Lateral → heading / rudder',
             (C('PID_LAT_KP'), C('PID_LAT_KI'), C('PID_LAT_KD'))),
            (ax[1], 'pid_lon', 'pid_dtheta_deg', 'Longitudinal → pitch',
             (C('PID_LON_KP'), C('PID_LON_KI'), C('PID_LON_KD')))):
        for k, col in (('p', '#1F77B4'), ('i', '#FF7F0E'), ('d', '#2CA02C')):
            a.plot(t[w], (L.num(f'{pre}_{k}') * dn)[w], color=col, lw=1, label=k.upper())
        a.plot(t[w], (L.num(out) * dn)[w], color='k', lw=1.5, label=f'output {out}')
        a.axhline(0, color='k', lw=.5)
        a.set_ylabel(lab + '\n(deg)')
        stat_box(a, f'KP {gains[0]:g}  KI {gains[1]:g}  KD {gains[2]:g}', 'upper left')
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right', ncol=4)
        a.set_xlim(a0, a1)
    ax[1].set_xlabel('Time from GUIDED (s)')
    fig.suptitle(f'Fig 11  DIVE PID terms — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig6_rudder(R, path, title, pdf, st):
    L, t = R.L, R.t
    a0, a1 = R.window(1.0, 2.0)
    w = (t >= a0) & (t <= a1)
    fig, ax = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True, layout='constrained')
    raw, cmd, sent = L.num('rudder_raw_deg'), L.num('rudder_cmd_deg'), L.num('rudder_sent_deg')
    lim = L.num('rudder_limit_deg')
    ax[0].plot(t[w], raw[w], color='#BBBBBB', label='raw (before limit)')
    ax[0].plot(t[w], cmd[w], color=C_CMD, lw=1, label='cmd')
    ax[0].plot(t[w], sent[w], color=C_ACT, lw=1.4, label='sent to FC')
    if np.isfinite(lim[w]).any():
        ax[0].plot(t[w], lim[w], color='k', ls='--', lw=.8, label='± limit')
        ax[0].plot(t[w], -lim[w], color='k', ls='--', lw=.8)
    ax[0].set_ylabel('Rudder (deg)')
    for k, col, lab in (('lim_by_airspeed', '#FF7F0E', 'limited by airspeed'),
                        ('lim_by_roll', '#D62728', 'limited by roll')):
        f = L.num(k)
        on = w & (f < 0.99)          # 欄位是縮放係數: 1.0 = 沒被限制, < 1 = 被砍
        if on.any():
            ax[0].fill_between(t, -1e3, 1e3, where=on, color=col, alpha=.12, label=lab)
    yl = nanmax(np.abs(np.r_[raw[w], sent[w], lim[w]]))
    if np.isfinite(yl):
        ax[0].set_ylim(-yl * 1.15 - 1, yl * 1.15 + 1)
    r = L.num('r_dps')
    ax[1].plot(t[w], r[w], color=C_ACT, label='yaw rate r')
    ax[1].set_ylabel('Yaw rate (deg/s)')
    ax[2].plot(t[w], L.num('roll_deg')[w], color=C_ACT, label='roll')
    ax[2].plot(t[w], (L.num('pid_u_err_deg') * np.where(R.m('DIVE'), 1.0, np.nan))[w], color=C_CMD, lw=1, label='lateral image err (deg)')
    ax[2].set_ylabel('deg')
    ax[2].set_xlabel('Time from GUIDED (s)')
    d = R.m('DIVE')
    if d.any() and np.isfinite(sent[d]).any():
        L_ = np.where(np.isfinite(lim), lim, C('RUDDER_LIMIT_DEG'))
        sat = np.abs(sent[d]) >= L_[d] - 0.5
        st['rud_sat_pct'] = 100.0 * sat.mean()
        st['rud_max'] = nanmax(np.abs(sent[d]))
        st['rud_lim_roll_pct'] = 100.0 * np.nanmean(L.num('lim_by_roll')[d] < 0.99)
        stat_box(ax[0], f'DIVE max |sent| {st["rud_max"]:5.1f}°\n'
                        f'at limit       {st["rud_sat_pct"]:5.1f} %\n'
                        f'roll-limited   {st["rud_lim_roll_pct"]:5.1f} %', 'lower left')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right', ncol=3)
        a.set_xlim(a0, a1)
    fig.suptitle(f'Fig 10  Rudder loop (DIVE) — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig8_miss(R, path, title, pdf, st):
    L, t = R.L, R.t
    a0, a1 = R.window(1.0, 1.0)
    w = (t >= a0) & (t <= a1)
    # 只畫 DIVE: 平飛 / 推頭時直線外推的落點在幾公里外, 沒有意義
    dn = np.where(R.m('DIVE'), 1.0, np.nan)
    fig, ax = plt.subplots(2, 1, figsize=(12, 7), sharex=True, layout='constrained')
    mt, ml, mc = (L.num('miss_pred_m') * dn, L.num('miss_pred_lon_m') * dn,
                  L.num('miss_pred_lat_m') * dn)
    ax[0].plot(t[w], mt[w], color='k', label='|miss|')
    ax[0].plot(t[w], ml[w], color='#1F77B4', label='along-track (+ = long / overshoot)')
    ax[0].plot(t[w], mc[w], color='#FF7F0E', label='cross-track (+ = right)')
    ax[0].axhline(0, color='k', lw=.5)
    ax[0].set_ylabel('Predicted miss (m)')
    d = R.m('DIVE') & np.isfinite(mt)
    if d.any():
        i = np.where(d)[0][-1]
        st['miss_end'] = (float(mt[i]), float(ml[i]), float(mc[i]))
        stat_box(ax[0], f'end of DIVE: |miss| {mt[i]:6.1f} m\n'
                        f'  along {ml[i]:+6.1f} m  cross {mc[i]:+6.1f} m', 'upper left')
    ax[1].plot(t[w], (L.num('impact_t_fall_s') * dn)[w], color=C_ACT, label='time to impact (straight-line)')
    ax[1].set_ylabel('s')
    ax[1].set_xlabel('Time from GUIDED (s)')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right')
        a.set_xlim(a0, a1)
    fig.suptitle(f'Fig 12  Predicted miss in DIVE (straight-line extrapolation) — {title}',
                 fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


def fig10_system(R, path, title, pdf, st):
    L, t = R.L, R.t
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True, layout='constrained')
    bv = L.num('batt_v')
    bv_ok = np.where(bv > 1.0, bv, np.nan)
    # MAVROS 的電池 topic 會輪流收到 BATT / BATT2 → 兩顆電壓差很多時分開畫
    packs = [('battery V', bv_ok)]
    if np.isfinite(bv_ok).sum() > 10 and nanmax(bv_ok) > 2.0 * nanmin(bv_ok):
        thr = math.sqrt(nanmax(bv_ok) * nanmin(bv_ok))
        packs = [('battery (low-V pack)', np.where(bv_ok < thr, bv_ok, np.nan)),
                 ('battery (high-V pack)', np.where(bv_ok >= thr, bv_ok, np.nan))]
    for (lab, v), col in zip(packs, (C_ACT, '#D62728')):
        k = np.isfinite(v)
        ax[0].plot(t[k], v[k], '.-', ms=2, lw=.8, color=col, label=lab)
    ax[0].set_ylabel('Battery (V)')
    st['batt_packs'] = [(lab, nanmin(v)) for lab, v in packs]
    dt = L.num('dt') * 1000
    ax[1].plot(t, dt, color=C_ACT, lw=.8, label='loop period')
    ax[1].axhline(20, color=C_REF, ls='--', lw=.8, label='50 Hz = 20 ms')
    ax[1].plot(t, L.num('ctrl_ms'), color=C_CMD, lw=.8, label='control compute')
    ax[1].set_ylabel('ms')
    ax[2].plot(t, L.num('nz_g'), color=C_ACT, label='nz (g)')
    ax[2].set_ylabel('nz (g)')
    ax[2].set_xlabel('Time from GUIDED (s)')
    st['batt_min'] = nanmin(bv_ok)
    st['batt_bad_pct'] = 100.0 * np.mean(np.isfinite(bv) & (bv <= 1.0)) if L.has('batt_v') else 0.0
    st['dt_med'] = float(np.nanmedian(dt)) if np.isfinite(dt).any() else float('nan')
    st['dt_slow_pct'] = 100.0 * np.nanmean(dt > 40) if np.isfinite(dt).any() else float('nan')
    st['dt_max'] = nanmax(dt)
    stat_box(ax[1], f'median {f1(st["dt_med"])} ms  max {f1(st["dt_max"])} ms\n'
                    f'> 40 ms: {f1(st["dt_slow_pct"], 2)} %', 'upper left')
    for a in ax:
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=8, loc='upper right')
    fig.suptitle(f'Fig 13  System health — {title}', fontsize=11, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)


# ─────────────────────────── 論文圖 Fig 1~9 (照學長 realflight_plot 的版面) ───────────────────────────
C_BLK = '#17202A'
C_TRK = '#C0392B'
C_VIR = '#1F77B4'      # 虛擬目標 (實際控制用)
C_GIM = '#1E8449'      # 雲台 (補償後)
C_CAR = '#8E6C3A'      # 真車 GPS


def _dive_span(R):
    """(DIVE 開始, DIVE 結束) 秒, 以撥 GUIDED 為 0; 沒有 DIVE 回傳 None."""
    if 'DIVE' not in R.first:
        return None
    return R.first['DIVE'], float(np.nanmax(R.t[R.m('DIVE')]))


def _guided_end(R):
    g = R.mode != ''
    return float(np.nanmax(R.t[g])) if g.any() else float(R.t[-1])


def _bands(ax, R, shift=0.0, x0=None, x1=None, labels=False):
    """模式底色 + 切換虛線; labels=True 時在上方寫模式名 (學長 Fig 3a 的樣子)."""
    span = (x1 - x0) if (x0 is not None and x1 is not None) else None
    for m, a, b in R.segs:
        c = MODE_COLOR.get(m)
        if not c:
            continue
        a, b = a - shift, b - shift
        if (x1 is not None and a >= x1) or (x0 is not None and b <= x0):
            continue
        ax.axvspan(a, b, color=c, alpha=.16, lw=0)
        if x0 is None or a > x0 + 1e-6:
            ax.axvline(a, color=c, ls='--', lw=1.0)
        if labels:
            lo = a if x0 is None else max(a, x0)
            hi = b if x1 is None else min(b, x1)
            if span is None or (hi - lo) > 0.05 * span:
                ax.text((lo + hi) / 2, .93, m, transform=ax.get_xaxis_transform(), ha='center',
                        va='top', color=c, fontsize=9, fontweight='bold')


def _band_legend(fig, R, extra=()):
    seen = []
    for m, _, _ in R.segs:
        if m in MODE_COLOR and m not in seen:
            seen.append(m)
    h = [Patch(facecolor=MODE_COLOR[m], alpha=.35, label=m) for m in seen] + list(extra)
    if h:
        fig.legend(handles=h, loc='upper center', ncol=len(h), fontsize=9, frameon=False,
                   bbox_to_anchor=(.5, .965))


def _title(fig, n, text, title):
    fig.suptitle(f'Fig {n}  {text} — {title}', fontsize=12, fontweight='bold')


def _course(L):
    vn, ve = L.num('vn'), L.num('ve')
    c = np.degrees(np.arctan2(ve, vn))
    yaw = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
    return np.where(np.isfinite(c) & (np.hypot(vn, ve) > 2.0), c, yaw)


def _nice_lim(x, floor):
    m = nanmax(np.abs(x))
    if not np.isfinite(m):
        return floor
    m = max(floor, 1.1 * m)
    step = 25 if m < 300 else 100
    return math.ceil(m / step) * step


def compute_stats(R, st):
    """summary.txt 自動檢查用的數字 (跟畫哪張圖無關)."""
    L, t = R.L, R.t
    gm = np.where(R.mode != '', 1.0, np.nan)
    pc, th = L.num('pitch_cmd') * gm, L.num('theta_deg')
    d = R.m('DIVE')
    if d.any():
        st['pitch_rmse_dive'] = rmse((pc - th)[d])
        st['pitch_maxerr_dive'] = nanmax(np.abs(pc - th)[d])
        st['roll_max_dive'] = nanmax(np.abs(L.num('roll_deg')[d]))
        pu, pv = L.num('pid_u_err_deg'), L.num('pid_v_err_deg')
        half = d & (t >= (R.first['DIVE'] + nanmax(t[d])) / 2)
        for k, e in (('u', pu), ('v', pv)):
            st[f'{k}_rmse'] = rmse(e[d])
            st[f'{k}_mean_2nd'] = nanmean(e[half])
            fin = e[d][np.isfinite(e[d])]
            st[f'{k}_final'] = float(fin[-1]) if len(fin) else float('nan')
            st[f'{k}_osc_hz'] = _osc_rate(t[d], e[d])
    s = R.m('SEARCH')
    if s.any() and 'PRE_DIVE' in R.first:
        hd = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
        he = wrap180(L.num('heading_cmd') * gm - hd)
        tp = R.first['PRE_DIVE']
        st['hdg_err_search_end'] = nanmean(he[s & (t >= tp - 2) & (t < tp)])
    if L.has('vis_link') or L.has('vis_frames'):
        link = L.num('vis_link')
        g = R.mode != ''
        st['vis_link_pct'] = 100.0 * np.nanmean(link[g] > 0) if g.any() else float('nan')
        for m in ('SEARCH', 'PRE_DIVE', 'DIVE'):
            k = R.m(m) & (link > 0)
            if k.any():
                st[f'vis_det_{m}'] = 100.0 * np.nanmean(L.num('vis_valid')[k] > 0)
        lk = L.num('gimbal_locked')
        if 'PRE_DIVE' in R.first and np.isfinite(lk).any():
            k = (t >= R.first['PRE_DIVE']) & (lk > 0)
            st['gimbal_lock_s'] = float(t[k][0] - R.first['PRE_DIVE']) if k.any() else float('nan')


def figP1_image_error(R, path, title, pdf, gimb):
    """Fig 1  影像平面誤差 (% of half-frame), t = 0 在 DIVE 進入."""
    sp = _dive_span(R)
    if sp is None:
        return False
    L = R.L
    d = R.m('DIVE')
    tt = R.t - sp[0]
    W2, H2 = C('IMG_W') / 2.0, C('IMG_H') / 2.0
    usp = np.where(np.isfinite(L.num('virt_u_sp')), L.num('virt_u_sp'), C('IMG_W') / 2.0)
    vsp = np.where(np.isfinite(L.num('virt_v_sp')), L.num('virt_v_sp'), C('IMG_H') / 2.0)
    eu = (L.num('virt_bbox_u') - usp) / W2 * 100
    ev = (L.num('virt_bbox_v') - vsp) / H2 * 100
    off = d & ((L.num('virt_detected') <= 0) | (np.abs(eu) > 100) | (np.abs(ev) > 100))
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), layout='constrained')
    for a, e, ttl, sym, half, sp_ in ((ax[0], eu, '(a) Lateral image error', 'lateral error $e_u$', W2, usp),
                                      (ax[1], ev, '(b) Longitudinal image error',
                                       'longitudinal error $e_v$', H2, vsp)):
        ge = None
        if gimb is not None:
            gx = gimb[0] if a is ax[0] else gimb[1]
            ge = np.where(gimb[2] & d, (gx - sp_) / half * 100, np.nan)
        lim = _nice_lim(np.r_[e[d], ge[d] if ge is not None else []], 120)
        a.fill_between(tt, -lim, lim, where=off, color='#BBBBBB', alpha=.35, lw=0,
                       label='virtual target off-frame')
        a.axhline(100, color='#999999', lw=1, label='frame edge (±100 %)')
        a.axhline(-100, color='#999999', lw=1)
        a.axhline(0, color='#1E8449', ls='--', lw=1.2, label='image center (0 %)')
        a.plot(tt[d], e[d], color=C_BLK, lw=1.7, label=sym + ' (virtual)')
        box = f'DIVE RMSE = {rmse(e[d]):7.2f} %'
        if ge is not None and np.isfinite(ge).any():
            a.plot(tt, ge, '.', ms=3, color='#E67E22', label='gimbal (compensated)')
            box += f'\ngimbal RMS = {rmse(ge):7.2f} %'
        stat_box(a, box, 'upper right')
        a.set_xlim(0, sp[1] - sp[0])
        a.set_ylim(-lim, lim)
        a.set_title(ttl, loc='left')
        a.set(xlabel='t (s)   [ t = 0 at DIVE entry ]', ylabel='error (% of half-frame)')
        a.grid(alpha=.25, ls=':')
        a.legend(loc='upper left', ncol=3, fontsize=8)
    _title(fig, 1, 'Image-plane error convergence', title)
    _save(fig, path, pdf)
    return True


def figP2_attitude(R, path, title, pdf):
    """Fig 2  Pitch / 航向 (相對固定參考) / Roll, t = 0 在撥 GUIDED (SEARCH 進入)."""
    L, t = R.L, R.t
    sp = _dive_span(R)
    x0, x1 = 0.0, (sp[1] if sp else _guided_end(R))
    w = (t >= x0) & (t <= x1)
    d = R.m('DIVE')
    fig, ax = plt.subplots(3, 1, figsize=(10.5, 10), layout='constrained')
    th, pc = L.num('theta_deg'), L.num('pitch_cmd')
    act = R.m('PRE_DIVE', 'DIVE', 'PULLOUT')
    ax[0].plot(t[w], th[w], color=C_BLK, lw=1.7, label='actual')
    ax[0].plot(t[w], np.where(act, pc, np.nan)[w], '--', color=C_TRK, lw=1.6, label='command')
    if d.any():
        stat_box(ax[0], f'DIVE tracking RMSE = {rmse((pc - th)[d]):5.2f} deg', 'upper right')
    ax[0].set(ylabel='pitch (deg)')
    ax[0].set_title(r'(a) Pitch $\theta$', loc='left')

    # (b) 航向相對固定參考 = DIVE 進入時 飛機→虛擬目標 的方位 (沒有 DIVE 就用 PRE_DIVE / GUIDED 進入)
    an, ae, tn, te = L.num('ac_n'), L.num('ac_e'), L.num('tgt_n'), L.num('tgt_e')
    ref_at = 'DIVE entry' if 'DIVE' in R.first else ('PRE_DIVE entry' if 'PRE_DIVE' in R.first
                                                     else 'GUIDED entry')
    tr = R.first.get('DIVE', R.first.get('PRE_DIVE', 0.0))
    i = int(np.argmin(np.abs(t - tr)))
    ref = math.degrees(math.atan2(te[i] - ae[i], tn[i] - an[i])) % 360.0
    hd = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
    ax[1].plot(t[w], wrap180(hd - ref)[w], color=C_BLK, lw=1.7, label='actual heading − fixed reference')
    sc = R.m('SEARCH', 'PRE_DIVE')
    ax[1].plot(t[w], np.where(sc, wrap180(L.num('heading_cmd') - ref), np.nan)[w], '--', color=C_TRK,
               lw=1.5, label='SEARCH / PRE_DIVE: heading command − reference')
    eu = L.num('pid_u_err_deg')
    ax[1].plot(t[w], np.where(d, -eu, np.nan)[w], '--', color='#7B241C', lw=1.5,
               label='DIVE: −pid_u_err_deg')
    ax[1].axhline(0, color='#888888', lw=.8)
    box = f'Reference = {ref:.3f} deg ({ref_at})'
    if d.any():
        box += f'\nDIVE image-error RMS = {rmse(eu[d]):.2f} deg'
    stat_box(ax[1], box, 'lower right')
    ax[1].set(ylabel='relative angle (deg)')
    ax[1].set_title('(b) Heading relative to fixed aircraft-to-target bearing / image error', loc='left')

    roll = L.num('roll_deg')
    if L.has('fc_nav_roll_deg'):
        rc, rlab = L.num('fc_nav_roll_deg'), 'FC roll target'
    else:
        rc, rlab = np.where(R.m('PRE_DIVE', 'DIVE'), 0.0, np.nan), 'command (roll = 0)'
    ax[2].plot(t[w], roll[w], color=C_BLK, lw=1.7, label='actual')
    ax[2].plot(t[w], rc[w], '--', color=C_TRK, lw=1.5, label=rlab)
    for y in (C('ROLL_PROT_FULL_DEG'), -C('ROLL_PROT_FULL_DEG')):
        ax[2].axhline(y, color='#BBBBBB', lw=.8, ls=':')
    if d.any():
        stat_box(ax[2], f'DIVE tracking RMSE = {rmse((rc - roll)[d]):5.2f} deg', 'upper right')
    ax[2].set(ylabel='roll (deg)')
    ax[2].set_title(r'(c) Roll $\phi$  (dotted = roll protection ±%g°)' % C('ROLL_PROT_FULL_DEG'),
                    loc='left')
    for a in ax:
        _bands(a, R, x0=x0, x1=x1)
        a.set_xlim(x0, x1)
        a.set_xlabel('t (s)   [ t = 0 at GUIDED (SEARCH) entry ]')
        a.grid(alpha=.25, ls=':')
        a.legend(loc='upper left', fontsize=8, ncol=3)
    _title(fig, 2, 'Attitude and lateral image error', title)
    _band_legend(fig, R)
    fig.get_layout_engine().set(rect=(0, 0, 1, .955))
    _save(fig, path, pdf)


def figP3_altitude(R, path, title, pdf, car):
    """Fig 3  (a) AGL vs 時間; (b) 沿 DIVE 進入時視線的慣性座標剖面; (c) DIVE 中與目標的水平距離."""
    L, t = R.L, R.t
    sp = _dive_span(R)
    agl = L.num('agl')
    x1a = sp[1] if sp else _guided_end(R)
    if sp is None:
        fig, a0 = plt.subplots(1, 1, figsize=(10, 4), layout='constrained')
        ax = [a0]
    else:
        fig, ax = plt.subplots(3, 1, figsize=(10, 12.5), layout='constrained',
                               gridspec_kw=dict(height_ratios=[1, 1.35, .8]))
    w = (t >= 0) & (t <= x1a)
    ax[0].plot(t[w], agl[w], color=C_BLK, lw=1.8)
    _bands(ax[0], R, x0=0, x1=x1a, labels=True)
    ax[0].set_xlim(0, x1a)
    ax[0].set(xlabel='t (s)   [ t = 0 at GUIDED (SEARCH) entry ]', ylabel='AGL (m)')
    ax[0].set_title('(a) Altitude profile vs. time', loc='left')
    ax[0].grid(alpha=.25, ls=':')
    if sp is None:
        _title(fig, 3, 'Altitude profile (no DIVE in this run)', title)
        _save(fig, path, pdf)
        return
    d = R.m('DIVE')
    an, ae, tn, te = L.num('ac_n'), L.num('ac_e'), L.num('tgt_n'), L.num('tgt_e')
    i_d = int(np.where(d)[0][0])
    i_e = int(np.where(d)[0][-1])
    los = np.array([tn[i_d] - an[i_d], te[i_d] - ae[i_d]])
    los = los / max(np.hypot(*los), 1e-6)
    On, Oe = tn[i_d], te[i_d]
    s_ac = (an - On) * los[0] + (ae - Oe) * los[1]
    s_tg = (tn - On) * los[0] + (te - Oe) * los[1]
    b = ax[1]
    w3 = (t >= 0) & (t <= sp[1] + 4.0)
    for m in ('SEARCH', 'PRE_DIVE', 'DIVE', 'PULLOUT', 'DONE', 'ABORT'):
        k = w3 & (R.mode == m)
        if k.any():
            b.plot(s_ac[k], agl[k], color=MODE_COLOR[m], lw=2.4, label=m)
    b.axhline(0, color='#8B7355', lw=1.5)
    kd = np.where(d)[0]
    sub = kd[::max(1, int(round(0.25 / max(np.nanmedian(np.diff(t[kd])), 1e-3))))]
    b.plot(s_tg[kd], np.zeros(len(kd)), color='#E06C75', lw=3, alpha=.8, label='target ground track')
    b.plot(s_tg[sub], np.zeros(len(sub)), 'o', color='#D0021B', ms=4)
    b.plot(s_tg[i_d], 0, 'o', mfc='white', mec='#D0021B', mew=2, ms=10, label='target @ dive entry', zorder=5)
    b.plot(s_tg[i_e], 0, 'o', color='#D0021B', mec='#7B0000', ms=11, label='target @ DIVE end', zorder=5)
    ang = C('DIVE_ANGLE_DEG')
    if 'PRE_DIVE' in R.first:
        i_p = int(np.argmin(np.abs(t - R.first['PRE_DIVE'])))
        b.plot(s_tg[i_p], 0, 'D', mfc='white', mec='#D0021B', mew=2, ms=10, label='target @ PRE_DIVE entry',
               zorder=5)
        b.plot(s_ac[i_p], agl[i_p], 'D', mfc='white', mec=MODE_COLOR['PRE_DIVE'], mew=2, ms=9, zorder=5)
        b.plot([s_ac[i_p], s_tg[i_p]], [agl[i_p], 0], ':', color=MODE_COLOR['PRE_DIVE'], lw=1.3)
        b.axvline(s_ac[i_p], color=MODE_COLOR['PRE_DIVE'], ls=':', lw=1.5)
        xs = np.array([s_ac[i_p], s_ac[i_p] + agl[i_p] / math.tan(math.radians(ang))])
        b.plot(xs, [agl[i_p], 0], '-.', color='#52A37A', lw=1.5,
               label=f'ideal {ang:g}° (config) from PRE_DIVE entry')
        b.annotate(f'{L.num("ground_m")[i_p]:.0f} m to target\n(PRE_DIVE entry)',
                   (s_ac[i_p], 0), xycoords=('data', 'axes fraction'), xytext=(0, -22),
                   textcoords='offset points', ha='center', va='top', color=MODE_COLOR['PRE_DIVE'],
                   fontsize=9, fontweight='bold')
    b.plot(s_ac[i_d], agl[i_d], 'o', mfc='white', mec=C_BLK, mew=2, ms=10, zorder=5)
    b.plot(s_ac[i_e], agl[i_e], 's', mfc='white', mec='#1E8449', mew=2, ms=9, zorder=5)
    th_e = L.num('theta_deg')[i_e]
    if np.isfinite(th_e) and th_e < -1:
        xh = s_ac[i_e] + agl[i_e] / math.tan(math.radians(-th_e))
        b.plot([s_ac[i_e], xh], [agl[i_e], 0], '--', color='#1E8449', lw=2,
               label=f'DIVE-end pitch {th_e:.1f}° extended')
        b.plot(xh, 0, 'x', color='#1E8449', ms=12, mew=3, zorder=6)
    car_d = None
    if car is not None:
        cn, ce = car
        s_car = (cn - On) * los[0] + (ce - Oe) * los[1]
        k = d & np.isfinite(s_car)
        if k.any():
            b.plot(s_car[k], np.zeros(k.sum()) + 1.5, color=C_CAR, lw=2.5, label='real car GPS (DIVE)')
            b.plot(s_car[np.where(k)[0][-1]], 1.5, 's', color=C_CAR, ms=9, label='real car @ DIVE end')
            car_d = np.hypot(cn - an, ce - ae)
    v_t = (s_tg[i_e] - s_tg[i_d]) / max(t[i_e] - t[i_d], 1e-3)
    b.set_title(f'(b) Altitude profile in inertial frame — target moves '
                f'{"right" if v_t >= 0 else "left"} (ideal {ang:g}° glide slope)', loc='left')
    b.set(xlabel='downrange along initial LOS (m)   [ 0 = target at dive entry ]', ylabel='AGL (m)')
    b.set_ylim(bottom=-6)
    b.xaxis.labelpad = 30 if 'PRE_DIVE' in R.first else 4
    b.grid(alpha=.25, ls=':')
    b.legend(loc='lower left', ncol=2, fontsize=7.5)
    c = ax[2]
    tt = t - sp[0]
    gd = L.num('ground_m')
    c.plot(tt[d], gd[d], color=C_BLK, lw=1.6, label='aircraft → virtual target')
    box = f'virtual: {gd[i_d]:.1f} m  ->  {gd[i_e]:.1f} m'
    if car_d is not None:
        c.plot(tt[d], car_d[d], color=C_CAR, lw=1.6, label='aircraft → real car (GPS)')
        kk = np.where(d & np.isfinite(car_d))[0]
        if len(kk):
            box += f'\nreal car: {car_d[kk[0]]:.1f} m  ->  {car_d[kk[-1]]:.1f} m'
    stat_box(c, box, 'upper right')
    c.set_xlim(0, sp[1] - sp[0])
    c.set(xlabel='t (s)   [ t = 0 at DIVE entry ]', ylabel='Horizontal distance (m)')
    c.set_title('(c) Aircraft-to-target horizontal distance during DIVE', loc='left')
    c.grid(alpha=.25, ls=':')
    c.legend(loc='lower left', fontsize=8)
    _title(fig, 3, 'Altitude profile', title)
    _save(fig, path, pdf)


def figP4_glide(R, path, title, pdf):
    """Fig 4  俯衝航跡角 vs 理想下滑角, t = 0 在 DIVE 進入."""
    sp = _dive_span(R)
    if sp is None:
        return False
    L = R.L
    d = R.m('DIVE')
    tt = R.t - sp[0]
    ang = C('DIVE_ANGLE_DEG')
    gam = L.num('gamma_deg')
    err = L.num('glide_slope_err_deg') if L.has('glide_slope_err_deg') else gam + ang
    fig, ax = plt.subplots(2, 1, figsize=(10, 7.2), layout='constrained')
    ax[0].axhline(-ang, color='#2E8B57', ls='--', lw=1.5, label=f'ideal glide slope (−{ang:g}°)')
    ax[0].plot(tt[d], gam[d], color=C_BLK, lw=1.8, label=r'flight-path angle $\gamma$')
    gd = gam[d][np.isfinite(gam[d])]
    if len(gd):
        stat_box(ax[0], f'final γ = {gd[-1]:6.2f} deg\nmin   γ = {gd.min():6.2f} deg', 'upper right')
    ax[0].set_title('(a) Flight-path angle vs. ideal glide slope (DIVE phase)', loc='left')
    ax[0].set(ylabel=r'$\gamma$ (deg)')
    ax[0].legend(loc='lower left', ncol=2, fontsize=8)
    ax[1].plot(tt[d], err[d], color=C_TRK, lw=1.8, label=r'glide-slope error  $\gamma - \gamma_{cmd}$')
    ax[1].fill_between(tt[d], 0, err[d], color=C_TRK, alpha=.15)
    ax[1].axhline(0, color='#2E8B57', ls='--', lw=1.2)
    stat_box(ax[1], f'DIVE RMSE = {rmse(err[d]):5.2f} deg', 'upper right')
    ax[1].set_title('(b) Glide-slope error', loc='left')
    ax[1].set(ylabel='error (deg)')
    ax[1].legend(loc='upper left', fontsize=8)
    for a in ax:
        a.set_xlim(0, sp[1] - sp[0])
        a.set_xlabel('t (s)   [ t = 0 at DIVE entry ]')
        a.grid(alpha=.25, ls=':')
    _title(fig, 4, 'Glide-slope tracking (DIVE phase)', title)
    _save(fig, path, pdf)
    return True


def figP4b_detection(R, path, title, pdf):
    """Fig 4b  各階段雲台偵測率 (以收到的新辨識結果計)."""
    L = R.L
    if not (L.has('vis_link') and L.has('vis_valid')):
        return False
    link = L.num('vis_link') > 0
    new = (L.num('vis_new') > 0) if L.has('vis_new') else np.ones(L.n, bool)
    valid = L.num('vis_valid') > 0
    phases = [m for m in ('SEARCH', 'PRE_DIVE', 'DIVE') if R.m(m).any()]
    if not phases:
        return False
    fig, ax = plt.subplots(figsize=(10, 6), layout='constrained')
    for i, m in enumerate(phases):
        k = R.m(m) & link & new
        n = int(k.sum())
        nd = int((k & valid).sum())
        pct = 100.0 * nd / n if n else 0.0
        ax.bar(i, pct, width=.55, color=MODE_COLOR[m])
        ax.text(i, pct + 2, f'{pct:.1f}%\n{nd} / {n} frames', ha='center', va='bottom', fontsize=12)
    ax.set_xticks(range(len(phases)), phases, fontsize=12)
    ax.set_ylim(0, 115)
    ax.set_yticks(range(0, 101, 20))
    ax.set_ylabel('Target detection rate (%)', fontsize=12)
    ax.set_xlabel('Mission phase\n\nDetected target frames / gimbal results received in each phase')
    ax.spines[['top', 'right']].set_visible(False)
    ax.grid(axis='y', alpha=.3, ls=':')
    ax.set_title('(b) Gimbal (partner Orin) target detection rate', loc='left', fontweight='bold',
                 fontsize=13)
    fig.suptitle(f'Fig 4b — {title}', fontsize=10, x=.99, ha='right')
    _save(fig, path, pdf)
    return True


def figP5_replay(R, path, title, pdf, gimb):
    """Fig 5  同一套 PID (50 Hz): 虛擬目標 vs 雲台補償後的輸入與重算命令."""
    sp = _dive_span(R)
    if sp is None or gimb is None or not (gimb[2] & R.m('DIVE')).any():
        return False
    L = R.L
    d = R.m('DIVE')
    tt = R.t - sp[0]
    fxv, fyv, cxv, cyv = _virt_intrinsics()
    usp = np.where(np.isfinite(L.num('virt_u_sp')), L.num('virt_u_sp'), cxv)
    vsp = np.where(np.isfinite(L.num('virt_v_sp')), L.num('virt_v_sp'), cyv)
    vu, vv = L.num('virt_bbox_u'), L.num('virt_bbox_v')
    gu, gv, gok = gimb[0], gimb[1], gimb[2]
    rep_v = replay_pid(R, vu, vv, L.num('virt_detected') > 0)
    rep_g = replay_pid(R, gu, gv, gok)
    n = 5 if rep_g is not None else 2
    fig, ax = plt.subplots(n, 1, figsize=(10, 2.6 * n + 1), layout='constrained')
    dn = np.where(d, 1.0, np.nan)
    for a, x, xs, f, lab in ((ax[0], vu, usp, fxv, '(a) Horizontal input error before PID filtering'),
                             (ax[1], vv, vsp, fyv, '(b) Vertical input error before PID filtering')):
        gx = gu if a is ax[0] else gv
        a.plot(tt, np.degrees(np.arctan((x - xs) / f)) * dn, color=C_VIR, lw=2, label='Virtual target')
        a.plot(tt, np.where(gok, np.degrees(np.arctan((gx - xs) / f)), np.nan) * dn, '--', color=C_GIM,
               lw=1.6, marker='.', ms=3, label='Gimbal data with compensation')
        a.set_title(lab, loc='center', fontweight='bold')
        a.set_ylabel('Error (deg)')
    if rep_g is not None:
        K, lim = C('K_LAT_TO_RUDDER'), C('RUDDER_LIMIT_DEG')
        for a, kv, lab, yl in ((ax[2], 2, '(c) Recomputed heading commands', 'Heading (deg)'),
                               (ax[3], 0, '(d) Recomputed pitch commands', 'Pitch (deg)')):
            yv, yg = rep_v[kv], rep_g[kv]
            if kv == 2:
                base = yv[np.isfinite(yv)][0] if np.isfinite(yv).any() else 0.0
                yv, yg = base + wrap180(yv - base), base + wrap180(yg - base)
            a.plot(tt, yv, color=C_VIR, lw=2, label='Virtual target')
            a.plot(tt, yg, '--', color=C_GIM, lw=1.8, label='Gimbal data with compensation')
            a.set_title(lab, loc='center', fontweight='bold')
            a.set_ylabel(yl)
        ax[4].plot(tt, np.clip(K * rep_v[1], -lim, lim), color=C_VIR, lw=2, label='Virtual target')
        ax[4].plot(tt, np.clip(K * rep_g[1], -lim, lim), '--', color=C_GIM, lw=1.8,
                   label='Gimbal data with compensation')
        ax[4].set_title(f'(e) Recomputed rudder commands (K_LAT_TO_RUDDER = {K:g}, ±{lim:g}°)',
                        loc='center', fontweight='bold')
        ax[4].set_ylabel('Rudder (deg)')
    for a in ax:
        a.set_xlim(0, sp[1] - sp[0])
        a.set_xlabel('Time since DIVE entry (s)')
        a.grid(alpha=.25, ls=':')
        a.legend(loc='upper right', fontsize=8)
    fig.suptitle(f'Fig 5  Inputs then same PID at 50 Hz — {title}\n'
                 'Gimbal: pitch/roll earth-referenced, yaw = aircraft nose (offline, open-loop replay)',
                 fontsize=12, fontweight='bold')
    _save(fig, path, pdf)
    return True


def figP6_airspeed(R, path, title, pdf):
    """Fig 6  空速 (撥 GUIDED → DIVE 結束), 各階段平均虛線 + 表格."""
    L, t = R.L, R.t
    sp = _dive_span(R)
    x1 = sp[1] if sp else _guided_end(R)
    w = (t >= 0) & (t <= x1)
    V = L.num('V')
    fig, ax = plt.subplots(figsize=(10, 5), layout='constrained')
    ax.plot(t[w], V[w], color=C_BLK, lw=1.8, drawstyle='steps-post', label='airspeed $V$')
    rows = []
    for m in ('SEARCH', 'PRE_DIVE', 'DIVE'):
        k = w & (R.mode == m)
        if not k.any():
            continue
        a, b = float(t[k][0]), float(t[k][-1])
        ax.plot([a, b], [nanmean(V[k])] * 2, '--', color=MODE_COLOR[m], lw=1.5)
        rows.append(f'{m:<9s}{b - a:5.1f}{V[k][0]:7.1f}{V[k][-1]:6.1f}{nanmean(V[k]):7.1f}'
                    f'{nanmin(V[k]):7.1f}{nanmax(V[k]):7.1f}')
    kk = np.where(w & np.isfinite(V))[0]
    if len(kk):
        c = MODE_COLOR.get(R.mode[kk[-1]], C_TRK)
        ax.plot(t[kk[-1]], V[kk[-1]], 'o', mfc='white', mec=c, mew=2, ms=8, zorder=5)
        ax.annotate(f'{V[kk[-1]]:.1f} m/s', (t[kk[-1]], V[kk[-1]]), xytext=(-8, 10),
                    textcoords='offset points', ha='right', color=c, fontweight='bold')
    if np.isfinite(nanmax(V[w])) and nanmax(V[w]) > 0.8 * C('PULLOUT_VMAX_MPS'):
        ax.axhline(C('PULLOUT_VMAX_MPS'), color=C_TRK, ls=':', lw=1.2,
                   label=f'Vmax {C("PULLOUT_VMAX_MPS"):g} m/s (early pullout)')
    if rows:
        stat_box(ax, 'mode       dur  entry  exit   mean    min    max\n' + '\n'.join(rows)
                 + '\n' + ' ' * 38 + '(s, m/s)', 'lower right')
    _bands(ax, R, x0=0, x1=x1, labels=True)
    ax.set_xlim(0, x1)
    ax.set(xlabel='t (s)   [ t = 0 at GUIDED entry;  end = DIVE exit ]', ylabel='airspeed (m/s)')
    ax.set_title('Airspeed by guidance phase  (dashed = phase mean)', loc='left')
    ax.grid(alpha=.25, ls=':')
    ax.legend(loc='upper left', fontsize=9)
    _title(fig, 6, 'Airspeed (GUIDED entry → DIVE exit)', title)
    _save(fig, path, pdf)


def figP7_loop_error(R, path, title, pdf, gimb):
    """Fig 7  DIVE 控制迴路誤差: 影像誤差 → 姿態追蹤誤差."""
    sp = _dive_span(R)
    if sp is None:
        return False
    L = R.L
    d = R.m('DIVE')
    tt = R.t - sp[0]
    agl = L.num('agl')
    kd = np.where(d)[0]
    xl = f't (s)   [ t = 0 at DIVE entry,  AGL {agl[kd[0]]:.0f} to {agl[kd[-1]]:.0f} m ]'
    eu, ev = L.num('pid_u_err_deg'), L.num('pid_v_err_deg')
    pe = L.num('pitch_cmd') - L.num('theta_deg')
    ce = wrap180(L.num('heading_cmd') - _course(L))
    ge = [None, None]
    if gimb is not None and (gimb[2] & d).any():
        fxv, fyv, cxv, cyv = _virt_intrinsics()
        usp = np.where(np.isfinite(L.num('virt_u_sp')), L.num('virt_u_sp'), cxv)
        vsp = np.where(np.isfinite(L.num('virt_v_sp')), L.num('virt_v_sp'), cyv)
        ge = [np.where(gimb[2] & d, np.degrees(np.arctan((gimb[0] - usp) / fxv)), np.nan),
              np.where(gimb[2] & d, np.degrees(np.arctan((gimb[1] - vsp) / fyv)), np.nan)]
    fig, ax = plt.subplots(4, 1, figsize=(9, 12), layout='constrained')
    spec = ((eu, '#2E86C1', r'$e_u$ lateral image error (right +)', '(a)  Lateral image error',
             r'$e_u$ (deg)', ge[0]),
            (ev, '#17A589', r'$e_v$ longitudinal image error (down +)', '(b)  Longitudinal image error',
             r'$e_v$ (deg)', ge[1]),
            (pe, C_TRK, r'pitch tracking error $\theta_{cmd} - \theta$', '(c)  Longitudinal tracking error',
             'pitch error (deg)', None),
            (ce, '#8E44AD', r'course tracking error $\psi_{cmd} - \chi$', '(d)  Lateral tracking error',
             'heading error (deg)', None))
    for a, (y, col, lab, ttl, yl, g) in zip(ax, spec):
        a.plot(tt[d], y[d], color=col, lw=1.8, label=f'{lab}   RMS = {rmse(y[d]):.1f} deg')
        if g is not None:
            a.plot(tt, g, '.', ms=3, color='#E67E22', label=f'gimbal (compensated)   RMS = {rmse(g):.1f} deg')
        a.axhline(0, color='#AAAAAA', lw=1)
        a.set_xlim(0, sp[1] - sp[0])
        a.set(xlabel=xl, ylabel=yl)
        a.set_title(ttl, loc='left', fontsize=10)
        a.grid(alpha=.25, ls=':')
        a.legend(loc='upper right', fontsize=8)
    _title(fig, 7, 'DIVE control-loop error (3-layer)', title)
    _save(fig, path, pdf)
    return True


def figP8_gimbal(R, path, title, pdf):
    """Fig 8  雲台姿態 (pitch / roll / yaw) + 模式 (+ 雷射距離), t = 0 在撥 GUIDED."""
    L, t = R.L, R.t
    if not (L.has('vis_gimbal_pitch') or L.has('vis_link')):
        return False
    sp = _dive_span(R)
    x1 = (sp[1] + 2.0) if sp else _guided_end(R)
    w = (t >= 0) & (t <= x1)
    las = L.num('vis_laser_m') if L.has('vis_laser_m') else None
    has_las = las is not None and np.isfinite(las[w]).any()   # V0 系列 / v8 才有雷射距離
    gy = L.num('vis_gimbal_yaw') if L.has('vis_gimbal_yaw') else None
    has_yaw = gy is not None and np.isfinite(gy[w]).any()     # 新格式 V0 系列才有 yaw
    # 面板: pitch / roll / (yaw) / mode / (laser)
    keys = ['pitch', 'roll'] + (['yaw'] if has_yaw else []) + ['mode'] + (['laser'] if has_las else [])
    hr = {'pitch': 1.2, 'roll': 1, 'yaw': 1, 'mode': .7, 'laser': 1}
    fig, axs = plt.subplots(len(keys), 1, figsize=(10, 2.2 + 2.3 * len(keys)), layout='constrained',
                            gridspec_kw=dict(height_ratios=[hr[k] for k in keys]))
    ax = dict(zip(keys, axs))
    gp = L.num('vis_gimbal_pitch')
    lock, tol = C('GIMBAL_LOCK_PITCH_DEG'), C('GIMBAL_LOCK_TOL_DEG')
    ax['pitch'].plot(t[w], gp[w], color=C_BLK, lw=1.8, label='gimbal pitch (earth-ref)')
    ax['pitch'].axhspan(lock - tol, lock + tol, color='#7D3C98', alpha=.10)
    ax['pitch'].axhline(lock, color='#7D3C98', ls='--', lw=1.2, label=f'lock {lock:g}° ± {tol:g}°')
    ax['roll'].plot(t[w], L.num('vis_gimbal_roll')[w], color=C_BLK, lw=1.8, label='gimbal roll')
    if has_yaw:
        # 對方確認 (2026/11/10): yaw 相對地面, 雲台 yaw 一直置中 = 相機永遠朝機頭方向.
        #   → 分析裡相機方位一律用飛機航向 (本圖只畫回報值供檢查, 不用來算投影)
        ax['yaw'].plot(t[w], gy[w], color=C_BLK, lw=1.8, label='gimbal yaw (reported)')
        ax['yaw'].axhline(0, color='#7D3C98', ls='--', lw=1.2, label='centered = aircraft nose')
    ax['mode'].plot(t[w], L.num('vis_gimbal_mode')[w], color=C_BLK, lw=1.8, drawstyle='steps-post',
                    label='reported')
    ax['mode'].plot(t[w], L.num('gimbal_mode_req')[w], '--', color=C_TRK, lw=1.5,
                    drawstyle='steps-post', label='requested')
    ax['mode'].set_yticks([0, 1, 2], ['0 manual', '1 track', '2 lock'])
    req = L.num('gimbal_mode_req')
    kr = np.where(w & (req == 2))[0]
    if len(kr):
        tr = t[kr[0]]
        lk = np.where(w & (t >= tr) & (L.num('gimbal_locked') > 0))[0]
        for a in axs:
            a.axvline(tr, color='#7D3C98', ls='-.', lw=1.5)
            if len(lk):
                a.axvline(t[lk[0]], color='#1E8449', ls='-.', lw=1.5)
        txt = f'LOCK requested\nt = {tr:.2f} s'
        if len(lk):
            txt += f'\nlocked +{t[lk[0]] - tr:.2f} s'
        ax['pitch'].text(tr, .95, ' ' + txt, transform=ax['pitch'].get_xaxis_transform(), va='top',
                         fontsize=9, color='#7D3C98', bbox=dict(boxstyle='round', fc='white', ec='#7D3C98'))
    if has_las:
        ax['laser'].plot(t[w], las[w], '.', color=C_BLK, ms=2.5, label='gimbal laser range')
        ax['laser'].plot(t[w], L.num('slant_m')[w], '--', color=C_TRK, lw=1.5,
                         label='slant range to virtual target')
    names = {'pitch': ('Gimbal pitch', 'Pitch (deg)'), 'roll': ('Gimbal roll', 'Roll (deg)'),
             'yaw': ('Gimbal yaw (earth-ref, held centered on aircraft nose)', 'Yaw (deg)'),
             'mode': ('Gimbal mode', 'Mode'), 'laser': ('Laser range', 'Range (m)')}
    panels = [(ax[k], f'({chr(97 + i)}) {names[k][0]}', names[k][1]) for i, k in enumerate(keys)]
    for a, ttl, yl in panels:
        _bands(a, R, x0=0, x1=x1)
        a.set_xlim(0, x1)
        a.set(xlabel='Time (s) [t = 0 at GUIDED (SEARCH) entry]', ylabel=yl)
        a.set_title(ttl, loc='left', fontweight='bold')
        a.grid(alpha=.25, ls=':')
        a.legend(loc='lower left', fontsize=8)
    _title(fig, 8, 'Gimbal attitude', title)
    _band_legend(fig, R)
    fig.get_layout_engine().set(rect=(0, 0, 1, .95))
    _save(fig, path, pdf)
    return True


def figP9_traj(R, path, title, pdf, car):
    """Fig 9  DIVE 飛機與目標軌跡: 目標行進方向 = Y 軸, 原點 = DIVE 進入時的虛擬目標."""
    sp = _dive_span(R)
    if sp is None:
        return False
    L, t = R.L, R.t
    d = R.m('DIVE')
    kd = np.where(d)[0]
    i0, i1 = kd[0], kd[-1]
    an, ae, tn, te = L.num('ac_n'), L.num('ac_e'), L.num('tgt_n'), L.num('tgt_e')
    fw = np.array([tn[i1] - tn[i0], te[i1] - te[i0]])
    if np.hypot(*fw) < 1.0:                          # 靜止目標: 用 DIVE 進入時的視線
        fw = np.array([tn[i0] - an[i0], te[i0] - ae[i0]])
    fw = fw / max(np.hypot(*fw), 1e-6)
    rt = np.array([-fw[1], fw[0]])
    On, Oe = tn[i0], te[i0]
    X = lambda n, e: (n - On) * rt[0] + (e - Oe) * rt[1]     # noqa: E731
    Y = lambda n, e: (n - On) * fw[0] + (e - Oe) * fw[1]     # noqa: E731
    psi_f = math.degrees(math.atan2(fw[1], fw[0]))
    fig, ax = plt.subplots(figsize=(7, 11), layout='constrained')
    ax.axvline(0, color='#AAB7B8', lw=1)
    ax.axhline(0, color='#AAB7B8', lw=.6)
    ax.plot(X(an[kd], ae[kd]), Y(an[kd], ae[kd]), '--', color='k', lw=2.2, label='Aircraft trajectory')
    ax.plot(X(tn[kd], te[kd]), Y(tn[kd], te[kd]), '--', color=C_TRK, lw=2.2,
            label='Vehicle trajectory (virtual target)')
    if car is not None:
        cn, ce = car
        k = kd[np.isfinite(cn[kd])]
        if len(k):
            ax.plot(X(cn[k], ce[k]), Y(cn[k], ce[k]), '-', color=C_CAR, lw=2, label='Real car GPS')
            ax.plot(X(cn[k[0]], ce[k[0]]), Y(cn[k[0]], ce[k[0]]), '^', color=C_CAR, ms=10)
            ax.plot(X(cn[k[-1]], ce[k[-1]]), Y(cn[k[-1]], ce[k[-1]]), 'D', color=C_CAR, ms=9)
    ax.plot(X(an[i0], ae[i0]), Y(an[i0], ae[i0]), 'o', color='k', ms=11)
    ax.plot(X(an[i1], ae[i1]), Y(an[i1], ae[i1]), 's', color='k', ms=10)
    ax.plot(0, 0, '*', color=C_TRK, ms=16)
    ax.plot(X(tn[i1], te[i1]), Y(tn[i1], te[i1]), 'D', color=C_TRK, ms=11)
    for txt, n, e, dx in (('Vehicle entry', On, Oe, 10), ('Vehicle end', tn[i1], te[i1], 10),
                          ('Aircraft entry', an[i0], ae[i0], 10), ('Aircraft end', an[i1], ae[i1], 10)):
        ax.annotate(txt, (X(n, e), Y(n, e)), xytext=(dx, -4), textcoords='offset points', fontsize=11,
                    color='#4D5656')
    yaw = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
    arrow = 14.0
    for s_ in np.arange(0, sp[1] - sp[0] + 1e-6, 1.0):
        i = int(np.argmin(np.abs(t - (sp[0] + s_))))
        if not d[i] or not np.isfinite(yaw[i]):
            continue
        rel = math.radians(yaw[i] - psi_f)
        x, y = X(an[i], ae[i]), Y(an[i], ae[i])
        ax.annotate('', (x + arrow * math.sin(rel), y + arrow * math.cos(rel)), (x, y),
                    arrowprops=dict(arrowstyle='-|>', color='#2E6DA4', lw=1.6))
        ax.text(x + 22, y, f'{s_:.0f} s', color='#2E6DA4', fontsize=11, va='center')
    ax.plot([], [], color='#2E6DA4', marker='>', label='Aircraft heading every 1 s')
    ax.set_aspect('equal', adjustable='datalim')
    ax.set(xlabel='Cross-track position (m)', ylabel='Along vehicle travel direction (m)')
    ax.grid(alpha=.25, ls=':')
    ax.legend(loc='upper center', bbox_to_anchor=(.5, 1.13), fontsize=10)
    fig.suptitle(f'Fig 9  DIVE aircraft and vehicle trajectories\nVehicle path aligned with x = 0\n{title}',
                 fontsize=12)
    _save(fig, path, pdf)
    return True


# ─────────────────────────── 雲台資料 → 控制命令 (離線重放) ───────────────────────────
# 雲台模型: pitch / roll 相對地面 (自穩), yaw 跟機頭.
#   (2026/11/10 對方確認: V0 回報的 yaw 相對地面, 但雲台 yaw 一直置中 → 相機永遠朝機頭方向)
# 把雲台相機看到的目標方向, 轉到 DIVE 用的虛擬相機 (機身固定, 光軸 = 機頭, roll=0,
# 與 dive_guidance.project_body_camera 同一條旋轉鏈), 再丟進同一套 DivePID.
_R_G2C = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)


def _ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def _rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])


def _rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[1, 0, 0], [0, c, s], [0, -s, c]])


def _virt_intrinsics():
    if CFG is not None and hasattr(CFG, 'virtual_intrinsics'):
        return CFG.virtual_intrinsics()
    return C('FX_1X'), C('FY_1X'), C('CX_1X'), C('CY_1X')


def _interp_at(t_full, y_full, tq, angle=False):
    ok = np.isfinite(t_full) & np.isfinite(y_full)
    if ok.sum() < 2:
        return np.full(len(tq), np.nan)
    y = y_full[ok]
    if angle:
        y = np.degrees(np.unwrap(np.radians(y)))
    return np.interp(tq, t_full[ok], y)


def gimbal_to_virtual(R, args):
    """每一列: 雲台偵測 → 虛擬相機像素 (u, v). 回傳 (u, v, valid, image_age_ms)."""
    L = R.L
    n = L.n
    t = L.num('t')
    fxv, fyv, cxv, cyv = _virt_intrinsics()
    ox, oy = L.num('vis_off_x'), L.num('vis_off_y')
    fx, fy = L.num('vis_fx'), L.num('vis_fy')
    gp = args.gimbal_pitch_sign * L.num('vis_gimbal_pitch') + args.gimbal_pitch_offset
    gr = args.gimbal_roll_sign * L.num('vis_gimbal_roll')
    gr = np.where(np.isfinite(gr), gr, 0.0)
    valid = (L.num('vis_valid') > 0) & (L.num('vis_link') > 0) & np.isfinite(ox) & \
        np.isfinite(oy) & (fx > 0) & (fy > 0) & np.isfinite(gp)
    # 取像時刻: 控制這一拍時影像已多舊 (v7); 沒有就用 收到多久 + 辨識多久
    age = L.num('vis_cap_age_ms')
    alt = L.num('vis_rx_age_ms') + np.where(np.isfinite(L.num('vis_det_age_ms')),
                                            L.num('vis_det_age_ms'), 0.0)
    age = np.where(np.isfinite(age), age, np.where(np.isfinite(alt), alt, 0.0))
    tc = t - age / 1000.0
    hdg_col = 'yaw_deg' if L.has('yaw_deg') else 'chi_deg'
    th_c = _interp_at(t, L.num('theta_deg'), tc)
    hd_c = _interp_at(t, L.num(hdg_col), tc, angle=True)
    u = np.full(n, np.nan)
    v = np.full(n, np.nan)
    for i in np.where(valid)[0]:
        ray = np.array([ox[i] / fx[i], oy[i] / fy[i], 1.0])
        cam = _R_G2C @ _ry(gp[i]) @ _rx(gr[i]) @ _rz(hd_c[i])
        ref = _R_G2C @ _ry(th_c[i]) @ _rz(hd_c[i])
        c = ref @ cam.T @ ray
        if np.all(np.isfinite(c)) and c[2] > 1e-6:
            u[i] = cxv + fxv * c[0] / c[2]
            v[i] = cyv + fyv * c[1] / c[2]
    return u, v, np.isfinite(u), age


def car_ned_rows(R, bins):
    """真車 GPS 內插到 log 每一列 (log 的 NED 座標). 回傳 ((car_n, car_e), '') 或 (None, 原因)."""
    import xwing_plot_target_gps as TG
    L = R.L
    gu = L.num('gps_unix')
    d = R.m('PRE_DIVE', 'DIVE')
    if not d.any() or np.isfinite(gu[d]).sum() < 2:
        return None, 'log 沒有 gps_unix 或沒有推頭'
    try:
        _, g = TG.pick_bin(bins, np.nanmin(gu[d]), np.nanmax(gu[d]))
        lat0, lon0 = TG.log_origin(L.num('lat'), L.num('lon'), L.num('ac_n'), L.num('ac_e'))
    except ValueError as exc:
        return None, str(exc)
    cn, ce = TG.car_ned(g, lat0, lon0)
    return (TG.interpolate(g['gps_unix'], cn, gu, 2.0), TG.interpolate(g['gps_unix'], ce, gu, 2.0)), ''


def car_in_virtual(R, bins):
    """用真車 GPS 算「車子理論上在虛擬相機的哪裡」(= 沒有量測誤差的雲台). 失敗回傳 None, 原因."""
    L = R.L
    car, why = car_ned_rows(R, bins)
    if car is None:
        return None, why
    rn, re_ = car
    fxv, fyv, cxv, cyv = _virt_intrinsics()
    an, ae, agl = L.num('ac_n'), L.num('ac_e'), L.num('agl')
    th = L.num('theta_deg')
    hd = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
    u = np.full(L.n, np.nan)
    v = np.full(L.n, np.nan)
    for i in np.where(np.isfinite(rn) & np.isfinite(an) & np.isfinite(agl) & np.isfinite(th))[0]:
        w = np.array([rn[i] - an[i], re_[i] - ae[i], agl[i]])
        c = _R_G2C @ (_ry(th[i]) @ _rz(hd[i]) @ w)
        if c[2] > 0.5:
            u[i] = cxv + fxv * c[0] / c[2]
            v[i] = cyv + fyv * c[1] / c[2]
    return (u, v), ''


def replay_pid(R, mu, mv, mdet):
    """在 DIVE 段用 (mu, mv, mdet) 重跑一次 DivePID; 回傳 (pitch_cmd, dpsi, heading_cmd)."""
    try:
        from dive_guidance import DivePID
        try:
            from xwing_vision_common import build_gcfg
        except ImportError:
            from xwing_virtual_common import build_gcfg
    except Exception:  # noqa: BLE001
        return None
    if CFG is None:
        return None
    gcfg = build_gcfg(CFG, moving=True, predive_dist=CFG.PREDIVE_DIST_M)
    L = R.L
    t = L.num('t')
    idx = np.where(R.m('DIVE'))[0]
    out_p = np.full(L.n, np.nan)
    out_d = np.full(L.n, np.nan)
    out_h = np.full(L.n, np.nan)
    if len(idx) < 3:
        return out_p, out_d, out_h
    pc, hc = L.num('pitch_cmd'), L.num('heading_cmd')
    i0 = idx[0]
    j = i0 - 1 if i0 > 0 and np.isfinite(pc[i0 - 1]) else i0
    pid = DivePID(gcfg)
    pid.reset(pitch0=float(pc[j]), heading0=float(hc[j]))
    usp, vsp = L.num('virt_u_sp'), L.num('virt_v_sp')
    vn, ve, th = L.num('vn'), L.num('ve'), L.num('theta_deg')
    yaw = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
    prev = i0
    for k in idx[1:]:
        dt = max(1e-3, min(t[k] - t[prev], 0.5))
        prev = k
        if np.isfinite(vn[k]) and np.isfinite(ve[k]) and vn[k] ** 2 + ve[k] ** 2 > 4.0:
            hnow = math.degrees(math.atan2(ve[k], vn[k]))
        else:
            hnow = float(yaw[k]) if np.isfinite(yaw[k]) else 0.0
        det = bool(mdet[k])
        p, h = pid.update(float(mu[k]) if det else None, float(mv[k]) if det else None,
                          float(usp[k]) if np.isfinite(usp[k]) else gcfg.CX,
                          float(vsp[k]) if np.isfinite(vsp[k]) else gcfg.CY,
                          det, float(th[k]), hnow, dt)
        out_p[k] = p
        out_d[k] = pid.info['dpsi']
        out_h[k] = h
    return out_p, out_d, out_h


def fig12_gimbal_replay(R, path, title, pdf, st, args, bins):
    """雲台資料當控制輸入: 誤差、重放命令與實際命令比較. 沒有 DIVE 或沒有雲台資料就不畫."""
    L, t = R.L, R.t
    d = R.m('DIVE')
    if not d.any() or not (L.has('vis_valid') and L.has('vis_off_x')):
        return False
    gu, gv, gok, age = gimbal_to_virtual(R, args)
    if not (gok & d).any():
        st['gr_note'] = 'DIVE 段沒有任何雲台偵測'
        return False
    fxv, fyv, cxv, cyv = _virt_intrinsics()
    deg_u = lambda u: np.degrees(np.arctan((u - cxv) / fxv))   # noqa: E731
    deg_v = lambda v: np.degrees(np.arctan((v - cyv) / fyv))   # noqa: E731
    vu, vv = L.num('virt_bbox_u'), L.num('virt_bbox_v')
    car, why = car_in_virtual(R, bins) if bins else (None, '沒有給車子 BIN')
    st['gr_car_note'] = why
    dn = np.where(d, 1.0, np.nan)

    # 重放: (1) 虛擬量測 → 確認重放 = 實際; (2) 雲台量測
    rep_v = replay_pid(R, vu, vv, L.num('virt_detected') > 0)
    rep_g = replay_pid(R, gu, gv, gok)
    K = C('K_LAT_TO_RUDDER')
    lim = C('RUDDER_LIMIT_DEG')
    pc_log = L.num('pitch_cmd')
    rud_log = L.num('rudder_raw_deg')
    if not np.isfinite(rud_log[d]).any():
        rud_log = K * L.num('pid_dpsi_deg')

    a0, a1 = R.window(1.0, 1.0)
    w = (t >= a0) & (t <= a1)
    fig, ax = plt.subplots(5, 1, figsize=(12, 13), sharex=True, layout='constrained')
    for k_, (fv, fg, col, lab) in enumerate(((deg_u(vu), deg_u(gu), 0, 'Lateral (+right)'),
                                             (deg_v(vv), deg_v(gv), 1, 'Vertical (+down)'))):
        a = ax[k_]
        a.plot(t[w], (fv * dn)[w], color=C_ACT, lw=1.4, label='virtual target (used for control)')
        a.plot(t[w], (np.where(gok, fg, np.nan) * dn)[w], '.', ms=3, color='#FF7F0E',
               label='gimbal → virtual camera')
        if car is not None:
            fc = deg_u(car[0]) if k_ == 0 else deg_v(car[1])
            a.plot(t[w], (fc * dn)[w], color='#2CA02C', lw=1, ls='--', label='real car GPS (geometry)')
        a.axhline(0, color='k', lw=.5)
        a.set_ylabel(f'{lab}\noffset (deg)')
    pr = rep_g[0] if rep_g is not None else None
    ax[2].plot(t[w], (pc_log * dn)[w], color=C_ACT, lw=1.4, label='pitch cmd actual (virtual)')
    if rep_v is not None:
        ax[2].plot(t[w], rep_v[0][w], color='#7F7F7F', lw=.9, ls=':', label='replay with virtual (check)')
        ax[2].plot(t[w], pr[w], color='#FF7F0E', lw=1.4, label='replay with gimbal data')
    ax[2].plot(t[w], (L.num('theta_deg') * dn)[w], color='#999999', lw=.8, label='pitch actual')
    ax[2].set_ylabel('Pitch cmd (deg)')
    ax[3].plot(t[w], (np.clip(rud_log, -lim, lim) * dn)[w], color=C_ACT, lw=1.4,
               label='rudder actual (virtual)')
    if rep_g is not None:
        ax[3].plot(t[w], np.clip(K * rep_v[1], -lim, lim)[w], color='#7F7F7F', lw=.9, ls=':',
                   label='replay with virtual (check)')
        ax[3].plot(t[w], np.clip(K * rep_g[1], -lim, lim)[w], color='#FF7F0E', lw=1.4,
                   label='replay with gimbal data')
    ax[3].set_ylabel(f'Rudder (deg)\nK_LAT={K:g}, ±{lim:g}')
    ax[4].plot(t[w], (gok * 1.0 * dn)[w], color='#2CA02C', drawstyle='steps-post',
               label='gimbal detection (1/0)')
    ax[4].set_ylim(-.1, 1.15)
    ax[4].set_ylabel('Detected')
    a4 = ax[4].twinx()
    a4.plot(t[w], (np.where(L.num('vis_link') > 0, age, np.nan) * dn)[w], color='#9467BD', lw=.9,
            label='image age at control tick (ms)')
    a4.set_ylabel('Image age (ms)', color='#9467BD')
    ax[4].set_xlabel('Time from GUIDED (s)')

    # ── 數字 ──
    st['gr_det_pct'] = 100.0 * gok[d].mean()
    st['gr_age_med'] = float(np.nanmedian(age[d & gok])) if (d & gok).any() else float('nan')
    eu_g, ev_g = deg_u(gu), deg_v(gv)
    eu_v, ev_v = deg_u(vu), deg_v(vv)
    m = d & gok
    st['gr_du_vs_virtual'] = (nanmean((eu_g - eu_v)[m]), rmse((eu_g - eu_v)[m]))
    st['gr_dv_vs_virtual'] = (nanmean((ev_g - ev_v)[m]), rmse((ev_g - ev_v)[m]))
    if car is not None:
        eu_c, ev_c = deg_u(car[0]), deg_v(car[1])
        st['gr_du_vs_car'] = (nanmean((eu_g - eu_c)[m]), rmse((eu_g - eu_c)[m]))
        st['gr_dv_vs_car'] = (nanmean((ev_g - ev_c)[m]), rmse((ev_g - ev_c)[m]))
        st['gr_car_vs_virtual'] = (rmse((eu_c - eu_v)[d]), rmse((ev_c - ev_v)[d]))
    if rep_g is not None:
        st['gr_check_pitch'] = rmse((rep_v[0] - pc_log)[d])
        st['gr_check_rud'] = rmse((K * rep_v[1] - rud_log)[d])
        st['gr_pitch_rmse'] = rmse((pr - pc_log)[d])
        st['gr_pitch_mean'] = nanmean((pr - pc_log)[d])
        st['gr_rud_rmse'] = rmse((np.clip(K * rep_g[1], -lim, lim) - np.clip(rud_log, -lim, lim))[d])
    box = (f'gimbal detection in DIVE {st["gr_det_pct"]:5.1f} %   image age median '
           f'{f1(st["gr_age_med"], 0)} ms\n'
           f'gimbal − virtual: u {st["gr_du_vs_virtual"][0]:+.2f}° (RMSE {st["gr_du_vs_virtual"][1]:.2f}°)'
           f'  v {st["gr_dv_vs_virtual"][0]:+.2f}° (RMSE {st["gr_dv_vs_virtual"][1]:.2f}°)')
    if car is not None:
        box += (f'\ngimbal − car GPS: u {st["gr_du_vs_car"][0]:+.2f}° (RMSE {st["gr_du_vs_car"][1]:.2f}°)'
                f'  v {st["gr_dv_vs_car"][0]:+.2f}° (RMSE {st["gr_dv_vs_car"][1]:.2f}°)')
    stat_box(ax[0], box, 'lower left')
    if rep_g is not None:
        stat_box(ax[2], f'gimbal replay − actual: RMSE {st["gr_pitch_rmse"]:.2f}°  '
                        f'mean {st["gr_pitch_mean"]:+.2f}°\n'
                        f'replay check (virtual): RMSE {st["gr_check_pitch"]:.3f}°', 'lower left')
        stat_box(ax[3], f'gimbal replay − actual: RMSE {st["gr_rud_rmse"]:.2f}°\n'
                        f'replay check (virtual): RMSE {st["gr_check_rud"]:.3f}°', 'lower left')
    for a in list(ax):
        shade_modes(a, R.segs)
        a.grid(alpha=.25)
        a.legend(fontsize=7, loc='upper right', ncol=3)
        a.set_xlim(a0, a1)
    a4.legend(fontsize=7, loc='lower right')
    fig.suptitle(f'Fig 15  Gimbal vs virtual target vs real car (offline replay, gimbal pitch/roll earth-ref,'
                 f' yaw = nose) — {title}', fontsize=10, x=.02, ha='left')
    mode_legend(fig, R.segs)
    _save(fig, path, pdf)
    return True


def analyse_gimbal_replay(st):
    """回傳 (數字, 檢查) 兩個 list, 給 summary 的獨立段落."""
    nums, chk = [], []
    if 'gr_det_pct' not in st:
        if st.get('gr_note'):
            nums.append(st['gr_note'])
        return nums, chk
    nums.append(f'DIVE 段雲台偵測率 {st["gr_det_pct"]:.1f} %, 影像延遲中位數 {f1(st["gr_age_med"], 0)} ms')
    du, dv = st['gr_du_vs_virtual'], st['gr_dv_vs_virtual']
    nums.append(f'雲台 − 虛擬目標 (角度): 橫向 平均 {du[0]:+.2f}° RMSE {du[1]:.2f}°, '
                f'縱向 平均 {dv[0]:+.2f}° RMSE {dv[1]:.2f}°')
    if 'gr_du_vs_car' in st:
        cu, cv = st['gr_du_vs_car'], st['gr_dv_vs_car']
        nums.append(f'雲台 − 真車 GPS 幾何 (= 雲台本身的量測誤差): 橫向 平均 {cu[0]:+.2f}° RMSE {cu[1]:.2f}°, '
                    f'縱向 平均 {cv[0]:+.2f}° RMSE {cv[1]:.2f}°')
        pu, pv = st['gr_car_vs_virtual']
        nums.append(f'真車 − 虛擬目標 (= 車子沒在虛擬位置造成的差): 橫向 RMSE {pu:.2f}°, 縱向 RMSE {pv:.2f}°')
    else:
        nums.append(f'沒有真車比較 ({st.get("gr_car_note", "")}) → 上面的差 = 雲台誤差 + 車子位置差, 分不開')
    if 'gr_pitch_rmse' in st:
        nums.append(f'重放命令 (雲台) − 實際命令: pitch RMSE {st["gr_pitch_rmse"]:.2f}° '
                    f'(平均 {st["gr_pitch_mean"]:+.2f}°), 方向舵 RMSE {st["gr_rud_rmse"]:.2f}°')
        nums.append(f'重放檢查 (用虛擬量測重放 vs 實際): pitch {st["gr_check_pitch"]:.3f}°, '
                    f'方向舵 {st["gr_check_rud"]:.3f}°')
        if st['gr_check_pitch'] > 0.5 or st['gr_check_rud'] > 1.0:
            chk.append('⚠ 用虛擬量測重放跟實際命令對不起來 → 重放不可信 (config 跟飛的時候不同?), 下面的比較僅供參考')
        else:
            chk.append('✓ 重放檢查通過: 用虛擬量測重放 = 實際命令, 雲台重放的比較可信')
    else:
        chk.append('· 找不到 dive_guidance / xwing_vision_common, 沒做命令重放 (只比誤差)')
    det = st['gr_det_pct']
    chk.append(('✓ ' if det >= 90 else '⚠ ') + f'DIVE 雲台偵測率 {det:.0f} %'
               + ('' if det >= 90 else ' → 掉幀多, 控制會常常沿用舊量測; 要用來控制得先提高偵測率'))
    age = st['gr_age_med']
    if np.isfinite(age):
        chk.append(('✓ ' if age <= 150 else '⚠ ') + f'影像延遲中位數 {age:.0f} ms'
                   + ('' if age <= 150 else ' → 延遲大, 直接接 PID 容易振盪: 要降增益或做延遲補償'))
    if 'gr_du_vs_car' in st:
        cu, cv = st['gr_du_vs_car'], st['gr_dv_vs_car']
        if abs(cv[0]) > 1.0 or abs(cu[0]) > 1.0:
            chk.append(f'⚠ 雲台跟真車幾何有固定偏差 (橫向 {cu[0]:+.1f}°, 縱向 {cv[0]:+.1f}°) → '
                       f'雲台 pitch 零點 / 安裝角 / 座標假設要校正 (縱向偏差可重跑試 '
                       f'--gimbal-pitch-offset {cv[0]:+.1f})')
        else:
            chk.append(f'✓ 雲台跟真車幾何沒有明顯固定偏差 (橫向 {cu[0]:+.1f}°, 縱向 {cv[0]:+.1f}°)')
        if max(cu[1], cv[1]) > 2.0:
            chk.append(f'⚠ 雲台量測跳動大 (RMSE 橫向 {cu[1]:.1f}° / 縱向 {cv[1]:.1f}°) → 要先濾波才適合當控制輸入')
    # 結論: 有真車時看「雲台本身的量測品質」(偏差 / 跳動 / 偵測率 / 延遲);
    # 命令差主要來自車子沒在虛擬位置 + 重放是開環 (飛機沒照雲台命令飛), 只當參考.
    quality = det >= 90 and np.isfinite(age) and age <= 150
    if 'gr_du_vs_car' in st:
        cu, cv = st['gr_du_vs_car'], st['gr_dv_vs_car']
        quality = quality and max(abs(cu[0]), abs(cv[0])) <= 1.0 and max(cu[1], cv[1]) <= 2.0
        chk.append(('✓ 初步判斷: 雲台量測品質足以當控制輸入' if quality else
                    '⚠ 初步判斷: 雲台量測品質還不夠直接當控制輸入 (看上面 ⚠ 的項目)')
                   + ' — 依據: 偵測率、延遲、雲台 vs 真車的偏差與跳動')
    else:
        chk.append(('· 偵測率 / 延遲 OK' if quality else '⚠ 偵測率或延遲不夠')
                   + '; 沒有真車 GPS, 無法確認雲台量得準不準 → 車子 BIN 放進 xwing_plot/car_bin/ 再跑一次')
    if 'gr_pitch_rmse' in st:
        chk.append('· 命令差 (pitch RMSE {:.1f}°, 方向舵 {:.1f}°) 是開環重放: 飛機沒有照雲台命令飛, '
                   '誤差不會被修掉, 積分會一直累積 → 只看「一開始往哪個方向修、修多少」, '
                   '不要拿 RMSE 當成控制效果'.format(st['gr_pitch_rmse'], st['gr_rud_rmse']))
    return nums, chk


# ─────────────────────────── 自動檢查 ───────────────────────────
def analyse(R, st, events, gps_lines):
    """回傳 (時間線, 數字, 檢查) 三段文字. 檢查用 ✓ / ⚠ / ✗ 開頭."""
    L, t = R.L, R.t
    agl, V = L.num('agl'), L.num('V')
    tl = []
    for m, a, b in R.segs:
        i0 = np.argmin(np.abs(t - a))
        tl.append(f'  {a:7.2f} ~ {b:7.2f} s  {m:<8s} ({b - a:5.2f} s)  進入時 AGL {f1(agl[i0])} m'
                  f'  V {f1(V[i0])} m/s  離目標 {f1(L.num("ground_m")[i0], 0)} m')
    ev_lines = [f'  [{tt - (R.t_on + 0):7.2f}] {s}' for tt, s in events]
    nums, chk = [], []

    def ok(s):
        chk.append('✓ ' + s)

    def warn(s):
        chk.append('⚠ ' + s)

    def bad(s):
        chk.append('✗ ' + s)

    # 1. 撥 GUIDED 時
    g0 = R.mode != ''
    if g0.any():
        i = np.where(g0)[0][0]
        nums.append(f'撥 GUIDED: AGL {f1(agl[i])} m, 空速 {f1(V[i])} m/s, 離虛擬起點 '
                    f'{f1(np.hypot(L.num("ac_n")[i], L.num("ac_e")[i]), 0)} m')
        if np.isfinite(agl[i]) and agl[i] < 20:
            chk.append(f'· 撥 GUIDED 時 AGL {agl[i]:.0f} m = 地面測試, 不檢查飛行項目')
        elif abs(agl[i] - C('CRUISE_AGL_M')) > 20:
            warn(f'撥 GUIDED 時 AGL {agl[i]:.0f} m, 跟巡航 {C("CRUISE_AGL_M"):g} m 差超過 20 m '
                 f'→ 推頭點的幾何會跟設計不同 (看 Mission Planner 航點高度)')
        else:
            ok(f'撥 GUIDED 高度 {agl[i]:.0f} m (設計 {C("CRUISE_AGL_M"):g} m)')
    if 'ABORT' in R.first:
        why = [s for _, s in events if 'ABORT' in s]
        bad('這趟有 ABORT: ' + (why[0] if why else '看 events.txt'))
    if 'PRE_DIVE' not in R.first:
        refuse = [x for _, x in events if '不啟動' in x]
        if refuse:
            chk.append('· ' + refuse[0])
        warn('這趟沒有進 PRE_DIVE (沒推頭) → 看 events.txt 是「拒絕啟動」還是提早切出 GUIDED')
        return tl, ev_lines, nums, chk

    # 2. SEARCH 對準
    he = st.get('hdg_err_search_end', float('nan'))
    if np.isfinite(he):
        nums.append(f'推頭前 2 s 航向誤差 {he:+.1f}°')
        (ok if abs(he) < 5 else warn)(
            f'推頭前航向誤差 {he:+.1f}°' + ('' if abs(he) < 5 else
                                       ' → 還沒對準就推頭: 撥 GUIDED 再早一點 / 遠一點, 或 GUIDED_P 轉向太慢'))

    # 3. PRE_DIVE 推頭時間
    if 'DIVE' in R.first:
        tp = R.first['DIVE'] - R.first['PRE_DIVE']
        nums.append(f'PRE_DIVE 推頭用時 {tp:.2f} s')
        if tp > 4.0:
            warn(f'推頭到 −{C("DIVE_ANGLE_DEG"):g}° 花 {tp:.1f} s (>4 s) → 飛控 pitch 迴路偏慢: '
                 f'看飛控 pitch 角速度上限 / pitch 增益 (PTCH_RATE_MAX、PTCH2SRV_*) 或 '
                 f'PREDIVE_HANDOFF_PITCH_TOL_DEG={C("PREDIVE_HANDOFF_PITCH_TOL_DEG"):g}')
        else:
            ok(f'推頭用時 {tp:.1f} s')

    d = R.m('DIVE')
    if d.any():
        gam = L.num('gamma_deg')
        late = d & (t >= R.first['DIVE'] + 1.0)
        gm = nanmean(gam[late]) if late.any() else nanmean(gam[d])
        nums.append(f'DIVE 時長 {nanmax(t[d]) - R.first["DIVE"]:.2f} s, 平均航跡角 γ {gm:.1f}° '
                    f'(設計 −{C("DIVE_ANGLE_DEG"):g}°)')
        if np.isfinite(gm) and abs(gm + C('DIVE_ANGLE_DEG')) > 3:
            warn(f'實際俯衝角 γ {gm:.1f}° 跟設計 −{C("DIVE_ANGLE_DEG"):g}° 差 '
                 f'{abs(gm + C("DIVE_ANGLE_DEG")):.1f}° → 看 pitch 追蹤 (Fig 2, Fig 7) / ALPHA_DIVE_DEG')
        elif np.isfinite(gm):
            ok(f'俯衝角 γ {gm:.1f}°')
        pr = st.get('pitch_rmse_dive', float('nan'))
        if np.isfinite(pr):
            nums.append(f'DIVE pitch 追蹤 RMSE {pr:.2f}°, 最大差 {f1(st.get("pitch_maxerr_dive"), 1)}°')
            if pr > 3:
                warn(f'pitch 追蹤 RMSE {pr:.1f}° 偏大 → 飛控 pitch 迴路跟不上命令 '
                     f'(先調飛控, 不要先加 PID_LON 增益)')
            else:
                ok(f'pitch 追蹤 RMSE {pr:.1f}°')
        vmax = nanmax(V[d])
        nums.append(f'DIVE 最大空速 {vmax:.1f} m/s (上限 {C("PULLOUT_VMAX_MPS"):g})')
        pull_by_v = any('PULLOUT' in s and ('超速' in s or 'Vmax' in s or 'VMAX' in s) for _, s in events)
        if vmax >= C('PULLOUT_VMAX_MPS') - 0.5 or pull_by_v:
            warn(f'DIVE 空速到 {vmax:.1f} m/s, 因超速提前拉平 (沒到 {C("PULLOUT_AGL_M"):g} m) '
                 f'→ 俯衝角太陡 / 怠速油門太大 (THR_MIN) / 撥 GUIDED 高度太高')
        elif vmax >= C('PULLOUT_VMAX_MPS') - 5:
            warn(f'DIVE 空速 {vmax:.1f} m/s 接近上限 {C("PULLOUT_VMAX_MPS"):g}')
        else:
            ok(f'DIVE 最大空速 {vmax:.1f} m/s')

        # 影像誤差
        for ax_, nm, kp, ki in (('u', '橫向', 'PID_LAT_KP', 'PID_LAT_KI'),
                                ('v', '縱向', 'PID_LON_KP', 'PID_LON_KI')):
            r_, mn, osc, fin = (st.get(f'{ax_}_rmse'), st.get(f'{ax_}_mean_2nd'),
                                st.get(f'{ax_}_osc_hz'), st.get(f'{ax_}_final'))
            if r_ is None or not np.isfinite(r_):
                continue
            nums.append(f'{nm}影像誤差 DIVE RMSE {r_:.2f}°, 後半段平均 {mn:+.2f}°, 結束 {fin:+.2f}°, '
                        f'擺盪 {osc:.2f} Hz')
            msg = []
            if osc > 0.8 and r_ > 1.0:
                msg.append(f'誤差來回擺盪 ({osc:.1f} Hz) → {kp}={C(kp):g} 可能太大')
            if abs(mn) > 1.5:
                fb_ = L.num('lateral_fallback')[R.m('DIVE')]
                if ax_ == 'u' and np.isfinite(fb_).any() and np.nanmax(fb_) > 0.5:
                    msg.append(f'後半段一直偏 {mn:+.1f}° 沒收斂 → 這趟退回 legacy bank (見下方 ✗), 先修那個')
                else:
                    msg.append(f'後半段一直偏 {mn:+.1f}° 沒收斂 → {ki}={C(ki):g} 可能不夠 '
                               + ('或方向舵不夠力 (Fig 10)' if ax_ == 'u' else '或 pitch 追不上 (Fig 2, Fig 7)'))
            if r_ > 5:
                msg.append(f'RMSE {r_:.1f}° 很大 → 先看推頭時是否已經對準 (Fig 1 起點)')
            (warn if msg else ok)(f'{nm}影像誤差 RMSE {r_:.1f}°' + ('：' + '；'.join(msg) if msg else ''))

        # 橫向退回 legacy bank (啟動時找不到 GUIDED_P / 43002 連續失敗) → DIVE 沒用方向舵
        fb = L.num('lateral_fallback')[R.m('DIVE')]
        legacy = bool(np.isfinite(fb).any() and np.nanmax(fb) > 0.5)
        if legacy:
            bad('lateral_fallback=1: 橫向退回 legacy bank, DIVE 用滾轉轉彎、方向舵完全沒用 '
                '→ 終端機找「找不到 GUIDED_P」/「43002 連續失敗」; Mission Planner 查 GUIDED_P '
                '(沒有 = 韌體不同, 燒跟 A 機一樣的韌體; 有 = MAVROS 參數還沒載完就啟動程式)')

        # 方向舵
        sp = st.get('rud_sat_pct')
        if sp is not None and np.isfinite(sp) and not legacy:
            nums.append(f'方向舵 DIVE 最大 {st["rud_max"]:.1f}°, 打到限幅 {sp:.0f} % 時間, '
                        f'被 roll 保護限制 {st["rud_lim_roll_pct"]:.0f} %')
            if sp > 20:
                warn(f'方向舵 {sp:.0f} % 時間打滿 → 側偏太大 (推頭前沒對準) 或 K_LAT_TO_RUDDER×PID_LAT 太大; '
                     f'若橫向誤差也收斂不了 = 方向舵權限不夠')
            else:
                ok(f'方向舵打滿 {sp:.0f} % 時間')
            if st['rud_lim_roll_pct'] > 10:
                warn(f'{st["rud_lim_roll_pct"]:.0f} % 時間被 roll 保護砍方向舵 → 方向舵造成大 roll '
                     f'(ROLL_PROT_FULL/ZERO={C("ROLL_PROT_FULL_DEG"):g}/{C("ROLL_PROT_ZERO_DEG"):g}°)')
        rm = st.get('roll_max_dive')
        if rm is not None and np.isfinite(rm):
            nums.append(f'DIVE 最大 |roll| {rm:.1f}°')
            if rm > C('ROLL_PROT_FULL_DEG'):
                warn(f'DIVE roll 到 {rm:.0f}° (> {C("ROLL_PROT_FULL_DEG"):g}°) → 方向舵-滾轉耦合大, '
                     f'看 Fig 10 是否方向舵一打就滾')

        me = st.get('miss_end')
        if me:
            nums.append(f'DIVE 結束時預測落點誤差 {me[0]:.1f} m (沿航跡 {me[1]:+.1f}, 側向 {me[2]:+.1f})')
    score = [s for _, s in events if '成績' in s]
    if score:
        nums.append('程式成績: ' + score[-1].replace('★', '').strip())

    # 拉平
    for m in ('PULLOUT', 'ABORT'):
        if m in R.first:
            k = (t >= R.first[m]) & (R.mode != '')      # 只算 GUIDED 中 (切 FBWA 後是飛手在飛)
            amin = nanmin(agl[k])
            i0 = np.argmin(np.abs(t - R.first[m]))
            nums.append(f'{m} 開始 AGL {f1(agl[i0])} m / V {f1(V[i0])} m/s, 之後最低 AGL {f1(amin)} m'
                        f' (GUIDED 中)')
            if m == 'PULLOUT':
                under = C('PULLOUT_AGL_M') - amin
                if amin < C('ABORT_MIN_AGL_M'):
                    bad(f'拉平後最低 {amin:.0f} m, 低於 ABORT 高度 {C("ABORT_MIN_AGL_M"):g} m '
                        f'→ 拉平太晚: PULLOUT_AGL_M 提高 或 降低俯衝速度')
                elif under > 15:
                    warn(f'拉平掉了 {under:.0f} m 才停 (最低 {amin:.0f} m) → 考慮 PULLOUT_AGL_M 提高 '
                         f'{under:.0f} m 或確認 TECS / pitch 上仰速度')
                else:
                    ok(f'拉平最低 {amin:.0f} m (設定 {C("PULLOUT_AGL_M"):g} m)')

    # 空速計 vs GPS 速度 (DIVE 中風的影響相對小)
    vg = L.num('v_gps') if 'v_gps' in L._num else None
    if vg is not None:
        k = R.m('DIVE') & np.isfinite(vg) & np.isfinite(V) & (vg > 5)
        if k.sum() > 10:
            ratio = nanmean(V[k] / vg[k])
            nums.append(f'DIVE 空速計 / GPS 速度 = {ratio:.2f} (空速 {nanmean(V[k]):.1f}, GPS {nanmean(vg[k]):.1f} m/s)')
            ga = L.num('gamma_air_deg') if 'gamma_air_deg' in L._num else None
            n90 = int(np.sum(ga[k] <= -89.5)) if ga is not None else 0
            if ratio < 0.75 or ratio > 1.3:
                bad(f'空速計讀數跟 GPS 速度差很多 (比例 {ratio:.2f}) → 空速計 / ARSPD_RATIO 不準: '
                    f'60 m/s 超速拉平、飛控 TECS 與舵面增益排程都會錯' +
                    (f'; log 的 γ 有 {n90} 拍被夾在 −90° (圖已改用 GPS 速度算 γ)' if n90 else ''))
            else:
                ok(f'空速計 / GPS 速度 = {ratio:.2f}')

    # 風 / 側滑 (機頭 vs 地速方向): 機頭對準目標, 但實際航跡會偏到下風
    yawc = L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg')
    vn_, ve_, th_ = L.num('vn'), L.num('ve'), L.num('theta_deg')
    if np.isfinite(vn_).any() and np.isfinite(yawc).any():
        course = np.degrees(np.arctan2(ve_, vn_))
        crab = wrap180(yawc - course)
        for m in ('SEARCH', 'DIVE'):
            k = R.m(m) & np.isfinite(crab) & np.isfinite(V)
            if k.sum() < 10:
                continue
            an = V[k] * np.cos(np.radians(th_[k])) * np.cos(np.radians(yawc[k]))
            ae = V[k] * np.cos(np.radians(th_[k])) * np.sin(np.radians(yawc[k]))
            wn, we = nanmean(vn_[k] - an), nanmean(ve_[k] - ae)
            ws = math.hypot(wn, we)
            cm = nanmean(crab[k])
            nums.append(f'{m}: 機頭 − 地速方向 (偏航修正角) {cm:+.1f}°, 估計風 {ws:.1f} m/s '
                        f'從 {math.degrees(math.atan2(-we, -wn)) % 360:.0f}° 吹來 (用空速估, 僅供參考)')
            if m == 'DIVE' and abs(cm) > 4:
                warn(f'DIVE 中機頭跟實際飛行方向差 {cm:+.1f}° (側風) → 虛擬相機跟機頭, PID 把機頭對準目標, '
                     f'實際航跡會偏到下風 (側向落點偏差的主因之一)')

    # 系統
    if np.isfinite(st.get('dt_med', float('nan'))):
        nums.append(f'控制迴路週期 中位數 {st["dt_med"]:.1f} ms, 最大 {st["dt_max"]:.0f} ms, '
                    f'>40 ms 佔 {st["dt_slow_pct"]:.2f} %')
        if st['dt_slow_pct'] > 2 or st['dt_max'] > 200:
            warn('控制迴路有卡頓 (>40 ms 或單次 >200 ms) → Orin 負載 / 其他程式')
        else:
            ok('控制迴路 50 Hz 正常')
    if st.get('batt_packs'):
        nums.append('電壓最低: ' + ', '.join(f'{lab} {f1(v, 2)} V' for lab, v in st['batt_packs']))
    elif np.isfinite(st.get('batt_min', float('nan'))):
        nums.append(f'電壓最低 {st["batt_min"]:.2f} V')
    if st.get('batt_bad_pct', 0) > 5:
        warn(f'batt_v 有 {st["batt_bad_pct"]:.0f} % 讀到 ≈0 V → 第二顆電池監測沒接 / BATT 參數')
    if 'vis_link_pct' in st:
        nums.append(f'雲台 UART 連線 {f1(st["vis_link_pct"])} % (GUIDED 期間); 偵測率 '
                    + ', '.join(f'{m} {f1(st.get("vis_det_" + m))} %' for m in ('SEARCH', 'PRE_DIVE', 'DIVE')))
        if st['vis_link_pct'] < 95:
            warn(f'雲台 UART 連線只有 {st["vis_link_pct"]:.0f} % → 線材 / 對方程式')
        gl = st.get('gimbal_lock_s')
        if gl is not None:
            nums.append(f'雲台鎖定 −30° 到位: PRE_DIVE 後 {f1(gl, 2)} s')
            if not np.isfinite(gl):
                warn('PRE_DIVE 之後雲台一直沒有回報鎖定到位')
    if L.has('vis_laser_m'):                       # v8 雷射測距 (GUIDED 期間)
        las = L.num('vis_laser_m')[R.mode != '']
        if np.isfinite(las).any():
            nums.append(f'雲台雷射距離: 有值 {100 * np.isfinite(las).mean():.0f} % (GUIDED 期間), '
                        f'範圍 {np.nanmin(las):.0f} ~ {np.nanmax(las):.0f} m (Fig 8d)')
    for ln in gps_lines or []:
        nums.append('真車 GPS: ' + ln)
    return tl, ev_lines, nums, chk


# ─────────────────────────── 主程式 ───────────────────────────
def pick_latest(log_dir):
    p = latest_log(log_dir)
    if p is None:
        sys.exit(f'[plot] {log_dir} 裡沒有 xwing_vision_*.csv')
    return p


def runs_of(L):
    g = L.num('guided') > 0
    if not L.has('run'):
        return [1] if g.any() else []
    r = L.num('run')
    return sorted(int(x) for x in np.unique(r[g & (r > 0)]))


def gps_gamma(L):
    """航跡角改用 GPS 速度算. 飛行程式記的 gamma_deg = asin(−vd / 空速),
    空速計不準時會失真 (空速比下降速度還小 → 夾在 −90°). 原值留在 gamma_air_deg."""
    vn, ve, vd = L.num('vn'), L.num('ve'), L.num('vd')
    if not np.isfinite(vd).any():
        return
    L._num['gamma_air_deg'] = L.num('gamma_deg')
    L._num['gamma_deg'] = np.degrees(np.arctan2(-vd, np.hypot(vn, ve)))
    L._num['v_gps'] = np.sqrt(vn ** 2 + ve ** 2 + vd ** 2)


def make_report(log_path, run, outdir, args, bins):
    L = Log.load(log_path)
    gps_gamma(L)
    title = os.path.splitext(os.path.basename(log_path))[0]
    R = Run(L, run, full=args.full)
    events = [(tt, s) for tt, s in read_events(log_path)
              if R.L.n and R.L.num('t')[0] - 0.5 <= tt <= R.L.num('t')[-1] + 0.5]
    os.makedirs(outdir, exist_ok=True)
    p = lambda n: os.path.join(outdir, n)
    st = {}
    sub = f'{title}  run {run}' if not args.full else f'{title}  (full log)'
    for old in glob.glob(os.path.join(outdir, 'fig*')):        # 舊版圖檔 (編號改過) 先清掉
        os.remove(old)
    compute_stats(R, st)
    gimb = None
    if R.L.has('vis_valid') and R.L.has('vis_off_x'):
        g_ = gimbal_to_virtual(R, args)
        gimb = g_ if g_[2].any() else None
    car = car_ned_rows(R, bins)[0] if bins else None
    with PdfPages(p('report.pdf')) as pdf:
        figP1_image_error(R, p('fig1_image_error.png'), sub, pdf, gimb)
        figP2_attitude(R, p('fig2_attitude.png'), sub, pdf)
        figP3_altitude(R, p('fig3_altitude_profile.png'), sub, pdf, car)
        figP4_glide(R, p('fig4_glide_slope.png'), sub, pdf)
        figP4b_detection(R, p('fig4b_gimbal_detection_rate.png'), sub, pdf)
        figP5_replay(R, p('fig5_pid_replay_gimbal.png'), sub, pdf, gimb)
        figP6_airspeed(R, p('fig6_airspeed.png'), sub, pdf)
        figP7_loop_error(R, p('fig7_control_loop_error.png'), sub, pdf, gimb)
        figP8_gimbal(R, p('fig8_gimbal_attitude.png'), sub, pdf)
        figP9_traj(R, p('fig9_dive_trajectory.png'), sub, pdf, car)
        # ── 以下是 X-wing 才有的診斷圖 ──
        if R.m('DIVE').any():
            fig6_rudder(R, p('fig10_rudder.png'), sub, pdf, st)
            fig5_pid(R, p('fig11_pid_terms.png'), sub, pdf)
            fig8_miss(R, p('fig12_predicted_miss.png'), sub, pdf, st)
        fig10_system(R, p('fig13_system.png'), sub, pdf, st)
        gps_lines = []
        if bins:
            import xwing_plot_target_gps as TG
            try:
                gps_lines, _ = TG.compare(TG.load_log_basic(log_path), run, bins,
                                          p('fig14_target_gps'), title=title, pdf=pdf)
            except ValueError as exc:
                gps_lines = [f'沒畫 ({exc})']
        if not fig12_gimbal_replay(R, p('fig15_gimbal_vs_car.png'), sub, pdf, st, args, bins):
            if os.path.isfile(p('fig15_gimbal_vs_car.png')):
                os.remove(p('fig15_gimbal_vs_car.png'))
        # ── xwing_vision: 目標偵測 (vg_*) 對照圖 Fig V1~V5 ──
        import xwing_plot_vision as XV
        vis_res = XV.vision_figures(R, p, sub, pdf, car, sys.modules[__name__])
    tl, evl, nums, chk = analyse(R, st, events, gps_lines)
    g_nums, g_chk = analyse_gimbal_replay(st)
    head = [f'log : {log_path}', f'趟次: run {run}' + ('  (--full 整份)' if args.full else ''),
            f'時間 0 = 撥 GUIDED (log t = {R.t_on:.2f} s)',
            f'參數: 巡航 {C("CRUISE_AGL_M"):g} m / 俯衝 {C("DIVE_ANGLE_DEG"):g}° / 拉平 '
            f'{C("PULLOUT_AGL_M"):g} m / Vmax {C("PULLOUT_VMAX_MPS"):g} m/s; '
            f'PID_LAT {C("PID_LAT_KP"):g}/{C("PID_LAT_KI"):g}/{C("PID_LAT_KD"):g}, '
            f'PID_LON {C("PID_LON_KP"):g}/{C("PID_LON_KI"):g}/{C("PID_LON_KD"):g}, '
            f'K_LAT_TO_RUDDER {C("K_LAT_TO_RUDDER"):g}'
            + ('' if CFG is not None else '  (找不到 xwing_vision_config.py, 用內建預設值)')]
    text = ('\n'.join(head) + '\n\n【模式時間線】\n' + '\n'.join(tl)
            + '\n\n【數字】\n' + '\n'.join('  ' + x for x in nums)
            + '\n\n【自動檢查】 (✓ 正常  ⚠ 建議看  ✗ 有問題  · 說明; 只是根據 log 的提示, 最後仍以圖為準)\n'
            + '\n'.join('  ' + x for x in chk)
            + ('\n\n【雲台資料當控制命令 (離線重放, Fig 5 / Fig 15)】\n'
               + '\n'.join('  ' + x for x in g_nums + g_chk) if (g_nums or g_chk) else '')
            + ('\n\n【目標偵測對照 (Fig V1~V5; xwing_vision)】\n'
               + '\n'.join('  ' + x for x in vis_res[0] + vis_res[1]) if vis_res else '')
            + '\n\n【events.txt (這趟)】\n' + '\n'.join(evl) + '\n')
    with open(p('summary.txt'), 'w', encoding='utf-8') as f:
        f.write(text)
    return text, [m for m, _, _ in R.segs]


def plot_one(log, args, bins):
    """畫一份 log 的每一趟; 回傳 [(趟, 輸出資料夾 或 None, 說明)]."""
    print(f'[plot] log: {log}')
    L = Log.load(log)
    stem = os.path.splitext(os.path.basename(log))[0]
    if args.full:
        runs = [0]
    else:
        runs = [args.run] if args.run else runs_of(L)
        if not runs:
            print('[plot] 這份 log 沒有任何 GUIDED 段, 跳過 (地面測試請加 --full)')
            return [(None, None, '沒有 GUIDED (地面測試請加 --full)')]
    res = []
    for r in runs:
        od = os.path.join(args.outdir, stem, 'full' if args.full else f'run{r}')
        try:
            text, modes = make_report(log, r, od, args, bins)
        except ValueError as exc:
            print(f'[plot] 第 {r} 趟跳過: {exc}')
            res.append((r, None, str(exc)))
            continue
        print('\n' + '=' * 78)
        print(text)
        print(f'[plot] 圖 → {od}')
        res.append((r, od, ' → '.join(modes) or '(不在 GUIDED)'))
    return res


def main():
    ap = argparse.ArgumentParser(description='xwing_vision 實飛 log → 報告圖 + 自動檢查')
    ap.add_argument('log', nargs='*', help='xwing_vision_*.csv (可以一次給好幾份; 不給就要用 --latest / --all)')
    ap.add_argument('--latest', action='store_true', help='用 flight_logs/ 裡最新的 log')
    ap.add_argument('--all', action='store_true', help='flight_logs/ 裡每一份 log 都畫')
    ap.add_argument('--log-dir', default=os.path.join(ROOT, 'flight_logs'),
                    help='log 資料夾 (預設: 專案根目錄的 flight_logs/)')
    ap.add_argument('--run', type=int, default=None, help='只畫第幾趟 (預設每一趟都畫)')
    ap.add_argument('--full', action='store_true', help='不分趟, 畫整份 log (地面測試)')
    ap.add_argument('-o', '--outdir', default=os.path.join(HERE, 'plot_results'),
                    help='輸出根目錄 (預設 xwing_plot/plot_results/)')
    ap.add_argument('--car-bin-dir', default=None,
                    help='車子 BIN 資料夾 → 加畫 Fig 14 (不給時: xwing_plot/car_bin/ 有 BIN 就自動用)')
    ap.add_argument('--no-car', action='store_true', help='就算 car_bin/ 有 BIN 也不比對真車')
    ap.add_argument('--car-bin', action='append', default=[], help='直接指定車子 BIN (可多次)')
    ap.add_argument('--gimbal-pitch-offset', type=float, default=0.0,
                    help='Fig 5/15: 雲台 pitch 零點修正 (deg, 加在回報值上)')
    ap.add_argument('--gimbal-pitch-sign', type=float, default=1.0,
                    help='Fig 5/15: 雲台 pitch 符號 (回報往下為負 = 1)')
    ap.add_argument('--gimbal-roll-sign', type=float, default=1.0, help='Fig 5/15: 雲台 roll 符號')
    args = ap.parse_args()

    logs = []
    if args.all:
        logs = list_logs(args.log_dir)
        if not logs:
            sys.exit(f'[plot] {args.log_dir} 裡沒有 xwing_vision_*.csv')
    elif args.latest:
        logs = [pick_latest(args.log_dir)]
    elif not args.log:
        ap.error('請給 log 檔, 或加 --latest / --all')
    for lg in args.log:
        if not os.path.isfile(lg):
            # 只給檔名 / 從專案根目錄寫的 flight_logs/xxx.csv → 都去 log 資料夾找
            cands = [os.path.join(args.log_dir, lg), os.path.join(args.log_dir, os.path.basename(lg)),
                     os.path.join(ROOT, lg)]
            found = [c for c in cands if os.path.isfile(c)]
            if not found:
                sys.exit(f'[plot] 找不到 {lg}  (找過: {args.log_dir})')
            lg = found[0]
        logs.append(lg)

    bins = []
    auto_dir = os.path.join(HERE, 'car_bin')
    if (not args.no_car and not args.car_bin_dir and not args.car_bin and os.path.isdir(auto_dir)
            and glob.glob(os.path.join(auto_dir, '*.[Bb][Ii][Nn]'))):
        args.car_bin_dir = auto_dir
        print(f'[plot] 自動使用車子 BIN 資料夾: {auto_dir}  (不要比對真車就加 --no-car)')
    if args.no_car:
        args.car_bin_dir, args.car_bin = None, []
    if args.car_bin_dir or args.car_bin:
        import xwing_plot_target_gps as TG
        bins = TG.load_bins(TG.find_bins(args.car_bin_dir, args.car_bin))
        if not bins:
            print('[plot] ⚠ 沒有可用的車子 BIN, 不畫 Fig 14')

    done = [(lg, plot_one(lg, args, bins)) for lg in logs]
    if len(logs) > 1:
        print('\n' + '=' * 78 + f'\n[plot] 總表 ({len(logs)} 份 log)')
        for lg, res in done:
            print(f'  {os.path.basename(lg)}')
            for r, od, msg in res:
                tag = '-' if r is None else f'run{r}'
                print(f'      {tag:<6s} {msg}' + (f'\n             → {od}' if od else ''))


if __name__ == '__main__':
    main()
