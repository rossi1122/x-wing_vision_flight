# -*- coding: utf-8 -*-
"""
xwing_plot_vision.py — 目標偵測 (vg_* 欄位) 的對照圖 Fig V1~V5 + summary 段落
═══════════════════════════════════════════════════════════════════
xwing_plot_report.py 畫每一趟時自動呼叫 (log 裡有 vg_* 欄位才畫, 也就是 xwing_vision 的 log).
--virtual-control (偵測只記錄) 與正式模式 (目標來自偵測) 都適用.

  Fig V1  看車俯角 vs 推頭: 偵測到的俯角、觸發角、實際推頭時刻、
          「角度判斷 / 距離判斷 會在什麼時候推頭」; 下半: 偵測水平距 vs 虛擬 vs 真車
  Fig V2  地圖: 飛機航跡、虛擬目標、偵測到的目標位置、真車 GPS
  Fig V3  偵測位置誤差 (對真車; 沒有車子 BIN 就對虛擬目標), 拆成沿路 / 橫向
  Fig V4  航向: 實際航向命令 vs「朝偵測到的目標」的航向 (視覺控制會怎麼飛)
  Fig V5  偵測與資料鏈: 看不看得到 (各模式比例)、延遲、雲台 yaw − 飛控航向

有真車 GPS (car_bin/ 有車子 BIN) 時, 另外估「雲台俯角 / 方位要補多少」
(= GIMBAL_PITCH_OFFSET_DEG / GIMBAL_YAW_OFFSET_DEG 的建議修正量).

圖的時間軸: 0 s = 撥 GUIDED. 圖用英文標籤 (跟 xwing_plot_report 一樣, 不需要中文字型).
"""
import math

import numpy as np
import matplotlib.pyplot as plt

C_VIRT = '#2E86C1'     # 虛擬目標
C_DET = '#E67E22'      # 偵測
C_CAR = '#27AE60'      # 真車
C_AC = '#1F4E79'       # 飛機
C_TRIG = '#C0392B'     # 觸發角 / 推頭


def _wrap(a):
    return (np.asarray(a, float) + 180.0) % 360.0 - 180.0


def _first(t, mask):
    i = np.where(mask)[0]
    return float(t[i[0]]) if len(i) else None


def _med(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.median(x)) if len(x) else None


def _pct(x, q):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return float(np.percentile(x, q)) if len(x) else None


def _f(x, fmt='{:.1f}', unit=''):
    return (fmt.format(x) + unit) if x is not None and np.isfinite(x) else '—'


def _xlim(R, pad=5.0):
    """畫圖時間範圍: 撥 GUIDED 前 ~ DONE / ABORT 後 pad 秒 (DONE 之後的長段不看)."""
    t = R.t
    end = None
    for k in ('DONE', 'ABORT'):
        if k in R.first:
            end = R.first[k] if end is None else min(end, R.first[k])
    hi = float(t[-1]) if end is None else min(float(t[-1]), end + pad)
    return float(t[0]), hi


def trigger_info(R, RP):
    """推頭依據 / 觸發角 / 距離門檻."""
    L = R.L
    trig = ''
    if 'vg_trigger' in L.cols:
        vals = [v for v in L.s('vg_trigger') if v]
        trig = vals[-1] if vals else ''
    ang = L.num('vg_trigger_angle_deg')
    ang = float(np.nanmedian(ang)) if np.isfinite(ang).any() else float(RP.C('DIVE_ANGLE_DEG'))
    cfg = RP.CFG
    dist = float(getattr(cfg, 'PREDIVE_DIST_M', 0.0) or 0.0) if cfg is not None else 0.0
    if dist <= 0:
        dist = RP.C('CRUISE_AGL_M') / math.tan(math.radians(RP.C('DIVE_ANGLE_DEG')))
    control = ''
    if 'control' in L.cols:
        vals = [v for v in L.s('control') if v]
        control = vals[-1] if vals else ''
    return control, trig, ang, dist


def _base(R):
    L = R.L
    d = {
        't': R.t,
        'valid': L.num('vg_valid') > 0,
        'new': L.num('vg_new') > 0,
        'dep': L.num('vg_depress_deg'),
        'vg_ground': L.num('vg_ground_m'),
        'vg_n': L.num('vg_tgt_n'), 'vg_e': L.num('vg_tgt_e'),
        'vg_brg': L.num('vg_bearing_deg'),
        'an': L.num('ac_n'), 'ae': L.num('ac_e'), 'agl': L.num('agl'),
        'tn': L.num('tgt_n'), 'te': L.num('tgt_e'),
        'v_ground': L.num('ground_m'),
        'hdg_cmd': L.num('heading_cmd'),
        'yaw': L.num('yaw_deg') if L.has('yaw_deg') else L.num('chi_deg'),
    }
    # 偵測位置只取「這拍看得到」的 (vg_tgt 看不到時保持最後一次, 不算新的量測)
    ok = d['valid'] & np.isfinite(d['vg_n'])
    d['det_n'] = np.where(ok, d['vg_n'], np.nan)
    d['det_e'] = np.where(ok, d['vg_e'], np.nan)
    d['dep_ok'] = np.where(d['valid'], d['dep'], np.nan)
    d['vg_ground_ok'] = np.where(d['valid'], d['vg_ground'], np.nan)
    return d


def _target_heading(d):
    """目標前進方位 (虛擬目標軌跡的方向); 拆沿路 / 橫向誤差用."""
    tn, te = d['tn'], d['te']
    ok = np.isfinite(tn) & np.isfinite(te)
    if ok.sum() >= 2:
        i = np.where(ok)[0]
        dn, de = tn[i[-1]] - tn[i[0]], te[i[-1]] - te[i[0]]
        if math.hypot(dn, de) > 5.0:
            return math.degrees(math.atan2(de, dn))
    return None


def fig_v1_trigger(R, path, sub, pdf, RP, d, car, info, st):
    control, trig, ang, dist = info
    t = d['t']
    fig, ax = plt.subplots(2, 1, figsize=(12, 8.2), sharex=True)
    x0, x1 = _xlim(R)
    pd0 = R.first.get('PRE_DIVE')
    # 「會在什麼時候推頭」: GUIDED 中、實際推頭那一拍 (含) 以前第一拍達到條件
    #   (正式模式下觸發的那一拍 mode 已經是 PRE_DIVE, 所以要包含它)
    srch = (R.mode != '') & ((t <= pd0 + 0.05) if pd0 is not None else True)
    t_ang = _first(t, srch & (d['dep_ok'] >= ang))
    t_dst = _first(t, srch & (d['vg_ground_ok'] <= dist))
    st.update(t_pd=pd0, t_ang=t_ang, t_dst=t_dst)

    a = ax[0]
    RP._bands(a, R, x0=x0, x1=x1)
    a.plot(t, d['dep_ok'], '.', ms=2.5, color=C_DET, label='Look-down angle (detection)')
    vlos = np.degrees(np.arctan2(d['agl'], d['v_ground']))
    a.plot(t, vlos, '-', lw=1.2, color=C_VIRT, label='LOS to virtual target')
    if car is not None:
        cn, ce = car
        cg = np.hypot(cn - d['an'], ce - d['ae'])
        a.plot(t, np.degrees(np.arctan2(d['agl'], cg)), '-', lw=1.2, color=C_CAR,
               label='LOS to real car (GPS)')
    a.axhline(ang, color=C_TRIG, ls='--', lw=1.2, label=f'Trigger angle {ang:.1f} deg')
    for k, (tt, lab, c, ls) in enumerate(((pd0, 'Actual PRE_DIVE', 'k', '-'),
                                          (t_ang, 'Angle trigger', C_TRIG, ':'),
                                          (t_dst, 'Distance trigger', C_DET, ':'))):
        if tt is not None:
            a.axvline(tt, color=c, ls=ls, lw=1.4)
            a.text(tt, .03 + .3 * k, ' ' + lab, rotation=90, transform=a.get_xaxis_transform(),
                   fontsize=8, color=c, va='bottom', ha='right')
    a.set_ylabel('Look-down angle (deg)')
    a.set_ylim(0, max(60.0, ang + 15.0))
    a.legend(fontsize=8, loc='upper left', ncol=2)
    a.grid(alpha=.3)

    a = ax[1]
    RP._bands(a, R, x0=x0, x1=x1)
    a.plot(t, d['vg_ground_ok'], '.', ms=2.5, color=C_DET, label='Ground range (detection)')
    a.plot(t, d['v_ground'], '-', lw=1.2, color=C_VIRT, label='Ground range to virtual target')
    if car is not None:
        a.plot(t, np.hypot(car[0] - d['an'], car[1] - d['ae']), '-', lw=1.2, color=C_CAR,
               label='Ground range to real car (GPS)')
    a.axhline(dist, color=C_TRIG, ls='--', lw=1.0, label=f'Distance threshold {dist:.0f} m')
    if pd0 is not None:
        a.axvline(pd0, color='k', lw=1.2)
    a.set_ylabel('Ground range (m)')
    a.set_xlabel('Time since GUIDED (s)')
    a.set_xlim(x0, x1)
    hi = np.nanmax(np.concatenate([d['v_ground'][np.isfinite(d['v_ground'])], [dist * 1.6]]))
    a.set_ylim(0, min(hi * 1.05, 2000))
    a.legend(fontsize=8, loc='upper right')
    a.grid(alpha=.3)
    txt = (f'control: {control or "?"}   trigger: {trig or "?"}\n'
           f'actual PRE_DIVE  t = {_f(pd0, "{:.2f}")} s\n'
           f'angle  would fire t = {_f(t_ang, "{:.2f}")} s\n'
           f'dist.  would fire t = {_f(t_dst, "{:.2f}")} s')
    RP.stat_box(ax[1], txt, loc='lower left')
    RP._band_legend(fig, R)
    fig.suptitle(f'Fig V1  Look-down angle & dive trigger — {sub}', fontsize=11, x=.02, ha='left')
    fig.tight_layout(rect=(0, 0, 1, .95))
    RP._save(fig, path, pdf)


def fig_v2_map(R, path, sub, pdf, RP, d, car):
    fig, ax = plt.subplots(figsize=(10, 9))
    x0, x1 = _xlim(R, pad=2.0)
    w = (d['t'] >= x0) & (d['t'] <= x1)
    # 東 = x, 北 = y
    for m, c in RP.MODE_COLOR.items():
        k = R.m(m) & w
        if k.any():
            ax.plot(d['ae'][k], d['an'][k], '.', ms=2, color=c, label=f'Aircraft {m}')
    k = (R.mode == '') & w
    if k.any():
        ax.plot(d['ae'][k], d['an'][k], '.', ms=1.5, color='#BBBBBB', label='Aircraft (not GUIDED)')
    ax.plot(d['te'][w], d['tn'][w], '-', lw=2, color=C_VIRT, label='Virtual target')
    ax.plot(d['det_e'][w], d['det_n'][w], 'o', ms=2.5, color=C_DET, alpha=.6,
            label='Detected target')
    if car is not None:
        ax.plot(car[1][w], car[0][w], '-', lw=2, color=C_CAR, label='Real car (GPS)')
    pd0 = R.first.get('PRE_DIVE')
    if pd0 is not None:
        i = int(np.argmin(np.abs(d['t'] - pd0)))
        ax.plot(d['ae'][i], d['an'][i], '*', ms=15, color='k', label='Aircraft at PRE_DIVE')
        ax.plot(d['te'][i], d['tn'][i], 's', ms=9, mfc='none', mec=C_VIRT, mew=2,
                label='Virtual target at PRE_DIVE')
        if car is not None and np.isfinite(car[0][i]):
            ax.plot(car[1][i], car[0][i], 's', ms=9, mfc='none', mec=C_CAR, mew=2,
                    label='Real car at PRE_DIVE')
    ax.plot(0, 0, 'x', ms=10, color='k')
    ax.annotate(' WP4 (virtual start)', (0, 0), fontsize=8)
    ax.set_ylabel('North from WP4 (m)')
    # 航線幾乎是直線: 東西向不等比例放大, 才看得出橫向差
    ee = np.concatenate([x[w][np.isfinite(x[w])] for x in
                         [d['ae'], d['te'], d['det_e']] + ([car[1]] if car is not None else [])]
                        + [np.array([0.0])])
    nn = np.concatenate([x[w][np.isfinite(x[w])] for x in
                         [d['an'], d['tn'], d['det_n']] + ([car[0]] if car is not None else [])]
                        + [np.array([0.0])])
    span_n = max(float(np.ptp(nn)), 50.0)
    span_e = max(float(np.ptp(ee)), 40.0)
    if span_e < span_n / 2.5:
        mid = float(np.mean([ee.min(), ee.max()]))
        ax.set_xlim(mid - span_e * .8, mid + span_e * .8)
        ax.set_xlabel('East from WP4 (m)   ★ east axis exaggerated (not to scale)')
    else:
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('East from WP4 (m)')
    ax.grid(alpha=.3)
    ax.legend(fontsize=8, loc='best')
    fig.suptitle(f'Fig V2  Map: aircraft / virtual target / detected target / car — {sub}',
                 fontsize=11, x=.02, ha='left')
    fig.tight_layout(rect=(0, 0, 1, .96))
    RP._save(fig, path, pdf)


def _errors(d, ref_n, ref_e, hdg):
    en, ee = d['det_n'] - ref_n, d['det_e'] - ref_e
    tot = np.hypot(en, ee)
    if hdg is None:
        return tot, None, None
    h = math.radians(hdg)
    along = en * math.cos(h) + ee * math.sin(h)
    cross = -en * math.sin(h) + ee * math.cos(h)
    return tot, along, cross


def fig_v3_error(R, path, sub, pdf, RP, d, car, st):
    t = d['t']
    hdg = _target_heading(d)
    refs = []
    if car is not None:
        refs.append(('real car (GPS)', car[0], car[1], C_CAR))
    refs.append(('virtual target', d['tn'], d['te'], C_VIRT))
    fig, ax = plt.subplots(len(refs), 1, figsize=(12, 3.6 * len(refs) + .6), sharex=True,
                           squeeze=False)
    x0, x1 = _xlim(R)
    for k, (name, rn, re_, c) in enumerate(refs):
        a = ax[k][0]
        RP._bands(a, R, x0=x0, x1=x1)
        tot, along, cross = _errors(d, rn, re_, hdg)
        a.plot(t, tot, '.', ms=2.5, color=c, label='|detected − ref|')
        if along is not None:
            a.plot(t, along, '.', ms=2, color='#7D3C98', alpha=.7,
                   label='along road (+ = detected ahead)')
            a.plot(t, cross, '.', ms=2, color='#B7950B', alpha=.7,
                   label='cross road (+ = detected right)')
        a.axhline(0, color='k', lw=.6)
        a.set_ylabel(f'Detected − {name}\n(m)')
        a.grid(alpha=.3)
        a.legend(fontsize=8, loc='upper right', ncol=3)
        srch = R.m('SEARCH')
        key = 'car' if name.startswith('real') else 'virt'
        st[f'err_{key}_med'] = _med(tot[srch])
        st[f'err_{key}_p95'] = _pct(tot[srch], 95)
        st[f'err_{key}_along'] = _med(along[srch]) if along is not None else None
        st[f'err_{key}_cross'] = _med(cross[srch]) if cross is not None else None
        RP.stat_box(a, f'SEARCH: median {_f(st[f"err_{key}_med"], "{:.0f}", " m")}, '
                       f'95% {_f(st[f"err_{key}_p95"], "{:.0f}", " m")}\n'
                       f'along {_f(st[f"err_{key}_along"], "{:+.0f}", " m")}, '
                       f'cross {_f(st[f"err_{key}_cross"], "{:+.0f}", " m")}', loc='upper left')
        lim = _pct(np.abs(np.concatenate([tot[np.isfinite(tot)], [10.0]])), 98)
        a.set_ylim(-(lim or 50) * 1.2, (lim or 50) * 1.2)
    ax[-1][0].set_xlabel('Time since GUIDED (s)')
    ax[-1][0].set_xlim(x0, x1)
    RP._band_legend(fig, R)
    fig.suptitle(f'Fig V3  Detected target position error — {sub}', fontsize=11, x=.02, ha='left')
    fig.tight_layout(rect=(0, 0, 1, .94))
    RP._save(fig, path, pdf)


def fig_v4_heading(R, path, sub, pdf, RP, d, st):
    t = d['t']
    brg_det = np.degrees(np.arctan2(d['det_e'] - d['ae'], d['det_n'] - d['an'])) % 360
    brg_virt = np.degrees(np.arctan2(d['te'] - d['ae'], d['tn'] - d['an'])) % 360
    fig, ax = plt.subplots(2, 1, figsize=(12, 7.5), sharex=True)
    x0, x1 = _xlim(R)
    a = ax[0]
    RP._bands(a, R, x0=x0, x1=x1)
    ref = np.nanmedian(brg_virt) if np.isfinite(brg_virt).any() else 0.0

    def un(x):     # 以虛擬方位為中心展開, 避免 0/360 跳
        return ref + _wrap(np.asarray(x, float) - ref)
    a.plot(t, un(d['hdg_cmd']), '-', lw=1.4, color='k', label='Heading command (actual)')
    a.plot(t, un(brg_virt), '-', lw=1.0, color=C_VIRT, label='Bearing to virtual target')
    a.plot(t, un(brg_det), '.', ms=2.5, color=C_DET, label='Bearing to detected target')
    a.plot(t, un(d['yaw']), '-', lw=1.0, color=C_AC, alpha=.6, label='Aircraft heading')
    a.set_ylabel('deg')
    a.grid(alpha=.3)
    a.legend(fontsize=8, loc='upper right', ncol=2)
    a = ax[1]
    RP._bands(a, R, x0=x0, x1=x1)
    diff = _wrap(brg_det - brg_virt)
    a.plot(t, diff, '.', ms=2.5, color=C_DET, label='Bearing: detected − virtual')
    a.axhline(0, color='k', lw=.6)
    a.set_ylabel('deg')
    a.set_xlabel('Time since GUIDED (s)')
    a.set_xlim(x0, x1)
    a.grid(alpha=.3)
    a.legend(fontsize=8, loc='upper right')
    srch = R.m('SEARCH')
    st['brg_diff_med'] = _med(diff[srch])
    st['brg_diff_p95'] = _pct(np.abs(diff[srch]), 95)
    RP.stat_box(a, f'SEARCH: median {_f(st["brg_diff_med"], "{:+.1f}", " deg")}, '
                   f'|95%| {_f(st["brg_diff_p95"], "{:.1f}", " deg")}', loc='upper left')
    lim = st['brg_diff_p95'] or 10.0
    a.set_ylim(-max(lim * 1.4, 5), max(lim * 1.4, 5))
    RP._band_legend(fig, R)
    fig.suptitle(f'Fig V4  Steering: virtual vs detected target — {sub}', fontsize=11,
                 x=.02, ha='left')
    fig.tight_layout(rect=(0, 0, 1, .94))
    RP._save(fig, path, pdf)


def fig_v5_health(R, path, sub, pdf, RP, d, st):
    L = R.L
    t = d['t']
    fig, ax = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True)
    x0, x1 = _xlim(R)
    a = ax[0]
    RP._bands(a, R, x0=x0, x1=x1)
    link = L.num('vis_link')
    a.step(t, np.where(np.isfinite(link), link, 0) + 0.04, where='post', color='#7F8C8D',
           lw=1, label='UART link')
    a.step(t, d['valid'].astype(float), where='post', color=C_DET, lw=1.2, label='Target seen')
    rates = []
    for m in ('SEARCH', 'PRE_DIVE', 'DIVE'):
        k = R.m(m)
        if k.any():
            r_ = float(np.mean(d['valid'][k])) * 100.0
            st[f'seen_{m}'] = r_
            rates.append(f'{m} {r_:.0f}%')
    a.set_yticks([0, 1])
    a.set_yticklabels(['no', 'yes'])
    a.set_ylim(-.2, 1.3)
    a.legend(fontsize=8, loc='upper right')
    RP.stat_box(a, 'seen: ' + '  '.join(rates), loc='upper left')
    a.grid(alpha=.3)
    a = ax[1]
    RP._bands(a, R, x0=x0, x1=x1)
    e2e = L.num('vis_e2e_ms')
    cap = L.num('vis_cap_age_ms')
    a.plot(t, e2e, '.', ms=2, color='#8E44AD', label='Capture → received (ms)')
    a.plot(t, cap, '.', ms=1.5, color='#BB8FCE', alpha=.6, label='Image age at control tick (ms)')
    a.set_ylabel('ms')
    a.grid(alpha=.3)
    a.legend(fontsize=8, loc='upper right')
    st['e2e_med'] = _med(e2e)
    RP.stat_box(a, f'capture→received median {_f(st["e2e_med"], "{:.0f}", " ms")}', loc='upper left')
    a = ax[2]
    RP._bands(a, R, x0=x0, x1=x1)
    yd = L.num('vg_yaw_diff_deg')
    a.plot(t, yd, '.', ms=2, color='#16A085', label='Gimbal yaw − FC heading (deg)')
    a.axhline(0, color='k', lw=.6)
    a.set_ylabel('deg')
    a.set_xlabel('Time since GUIDED (s)')
    a.set_xlim(x0, x1)
    a.grid(alpha=.3)
    a.legend(fontsize=8, loc='upper right')
    st['yaw_diff_med'] = _med(yd)
    st['yaw_diff_std'] = float(np.nanstd(yd)) if np.isfinite(yd).sum() > 2 else None
    RP.stat_box(a, f'median {_f(st["yaw_diff_med"], "{:+.1f}", " deg")}, '
                   f'std {_f(st["yaw_diff_std"], "{:.1f}", " deg")}', loc='upper left')
    if np.isfinite(yd).any():
        lim = max(5.0, (_pct(np.abs(yd), 98) or 5.0) * 1.3)
        a.set_ylim(-lim, lim)
    RP._band_legend(fig, R)
    fig.suptitle(f'Fig V5  Detection & data link — {sub}', fontsize=11, x=.02, ha='left')
    fig.tight_layout(rect=(0, 0, 1, .94))
    RP._save(fig, path, pdf)


def calib_from_car(R, d, car, st):
    """有真車時: 偵測的俯角 / 方位 vs 真車 → 建議的雲台校正量."""
    if car is None:
        return
    cn, ce = car
    rn, re_ = cn - d['an'], ce - d['ae']
    g = np.hypot(rn, re_)
    true_dep = np.degrees(np.arctan2(d['agl'], g))
    true_brg = np.degrees(np.arctan2(re_, rn))
    k = R.m('SEARCH', 'PRE_DIVE') & d['valid'] & np.isfinite(true_dep) & (g > 80) & (g < 900)
    if k.sum() < 20:
        return
    dd = true_dep[k] - d['dep'][k]                  # + = 真的比較往下 (偵測少算)
    db = _wrap(true_brg[k] - d['vg_brg'][k])        # + = 真車在偵測方位的右邊
    st['calib_n'] = int(k.sum())
    st['calib_dep_err'] = _med(dd)
    st['calib_dep_iqr'] = (_pct(dd, 75) or 0) - (_pct(dd, 25) or 0)
    st['calib_brg_err'] = _med(db)
    st['calib_brg_iqr'] = (_pct(db, 75) or 0) - (_pct(db, 25) or 0)


def vision_figures(R, p, sub, pdf, car, RP):
    """畫 Fig V1~V5, 回傳 summary 用的 (數字行, 檢查行). 沒有 vg_* 欄位回傳 None."""
    L = R.L
    if 'vg_valid' not in L.cols:
        return None
    st = {}
    info = trigger_info(R, RP)
    d = _base(R)
    fig_v1_trigger(R, p('figV1_lookdown_trigger.png'), sub, pdf, RP, d, car, info, st)
    fig_v2_map(R, p('figV2_map.png'), sub, pdf, RP, d, car)
    fig_v3_error(R, p('figV3_position_error.png'), sub, pdf, RP, d, car, st)
    fig_v4_heading(R, p('figV4_heading.png'), sub, pdf, RP, d, st)
    fig_v5_health(R, p('figV5_detection_link.png'), sub, pdf, RP, d, st)
    calib_from_car(R, d, car, st)
    return summarise(R, d, car, info, st)


def summarise(R, d, car, info, st):
    control, trig, ang, dist = info
    t = d['t']
    nums, chk = [], []
    ctrl_txt = {'vision': '目標偵測 (正式模式)',
                'virtual': '虛擬軌跡 (--virtual-control, 偵測只記錄)'}.get(control, control or '?')
    nums.append(f'控制來源: {ctrl_txt}')
    nums.append(f'推頭依據 (偵測): {trig or "?"}  觸發角 {ang:.1f}°  距離門檻 {dist:.0f} m'
                + ('  (--virtual-control 時實際推頭是看虛擬距離)' if control == 'virtual' else ''))
    t_det = _first(t, (R.mode != '') & d['valid'])
    if t_det is not None:
        i = int(np.argmin(np.abs(t - t_det)))
        nums.append(f'撥 GUIDED 後第一次看到目標: t = {t_det:.2f} s, 偵測水平距 '
                    f'{_f(d["vg_ground"][i], "{:.0f}", " m")}, 俯角 {_f(d["dep"][i], "{:.1f}", "°")}')
    else:
        nums.append('撥 GUIDED 後一直沒看到目標')
    seen = [f'{m} {st[f"seen_{m}"]:.0f}%' for m in ('SEARCH', 'PRE_DIVE', 'DIVE')
            if f'seen_{m}' in st]
    if seen:
        nums.append('看得到目標的比例: ' + '  '.join(seen))
    pd0, ta, tdst = st.get('t_pd'), st.get('t_ang'), st.get('t_dst')
    if pd0 is not None:
        i = int(np.argmin(np.abs(t - pd0)))
        line = (f'實際推頭 (PRE_DIVE) t = {pd0:.2f} s: 偵測俯角 {_f(d["dep_ok"][i], "{:.1f}", "°")}, '
                f'偵測水平距 {_f(d["vg_ground_ok"][i], "{:.0f}", " m")}, '
                f'虛擬水平距 {_f(d["v_ground"][i], "{:.0f}", " m")}')
        if car is not None:
            g = math.hypot(car[0][i] - d['an'][i], car[1][i] - d['ae'][i])
            if np.isfinite(g):
                line += (f', 真車水平距 {g:.0f} m (真實視線 '
                         f'{math.degrees(math.atan2(d["agl"][i], g)):.1f}°)')
        nums.append(line)
    else:
        nums.append('這趟沒有推頭')
    def would(tt):
        if tt is None:
            return ('到實際推頭時都還沒達到條件 (會比實際晚推頭, 或不推頭)' if pd0 is not None
                    else '整趟都沒有達到條件')
        return f't = {tt:.2f} s' + (f' (比實際 {tt - pd0:+.2f} s)' if pd0 is not None else '')
    nums.append(f'角度判斷會推頭: {would(ta)}')
    nums.append(f'距離判斷會推頭: {would(tdst)}')
    if car is not None and st.get('err_car_med') is not None:
        nums.append(f'偵測位置 vs 真車 (SEARCH): 中位數 {st["err_car_med"]:.0f} m, 95% '
                    f'{_f(st.get("err_car_p95"), "{:.0f}", " m")}; 沿路 '
                    f'{_f(st.get("err_car_along"), "{:+.0f}", " m")} (+ = 偵測在車前面), 橫向 '
                    f'{_f(st.get("err_car_cross"), "{:+.0f}", " m")} (+ = 偵測在右邊)')
    elif car is None:
        nums.append('沒有車子 BIN → 偵測位置只能跟虛擬目標比 (放進 xwing_plot/car_bin/ 才會跟真車比)')
    if st.get('err_virt_med') is not None:
        nums.append(f'偵測位置 vs 虛擬目標 (SEARCH): 中位數 {st["err_virt_med"]:.0f} m; 沿路 '
                    f'{_f(st.get("err_virt_along"), "{:+.0f}", " m")}, 橫向 '
                    f'{_f(st.get("err_virt_cross"), "{:+.0f}", " m")}  (= 真車與虛擬軌跡差多少 + 偵測誤差)')
    if st.get('brg_diff_med') is not None:
        nums.append(f'SEARCH 航向: 朝偵測目標 − 朝虛擬目標 中位數 {st["brg_diff_med"]:+.1f}°, '
                    f'|95%| {_f(st.get("brg_diff_p95"), "{:.1f}", "°")}  (視覺控制時航向會差這麼多)')
    nums.append(f'影像延遲 (取像→收到) 中位數 {_f(st.get("e2e_med"), "{:.0f}", " ms")}; '
                f'雲台 yaw − 飛控航向 中位數 {_f(st.get("yaw_diff_med"), "{:+.1f}", "°")}, '
                f'標準差 {_f(st.get("yaw_diff_std"), "{:.1f}", "°")}')
    if st.get('calib_n'):
        nums.append(f'雲台校正估計 (對真車, {st["calib_n"]} 筆): 俯角少算 '
                    f'{st["calib_dep_err"]:+.2f}° (四分位距 {st["calib_dep_iqr"]:.2f}°), '
                    f'方位偏 {st["calib_brg_err"]:+.2f}° (四分位距 {st["calib_brg_iqr"]:.2f}°)')

    # ── 自動檢查 ──
    for m in ('PRE_DIVE', 'DIVE'):
        r_ = st.get(f'seen_{m}')
        if r_ is not None and r_ < 80:
            chk.append(f'⚠ {m} 只有 {r_:.0f}% 看得到目標 — 正式模式連續 1.5 s 看不到就 ABORT')
    if st.get('seen_SEARCH') is not None and st['seen_SEARCH'] < 30:
        chk.append(f'⚠ SEARCH 只有 {st["seen_SEARCH"]:.0f}% 看得到目標 — 正式模式可能一直不推頭')
    if ta is not None and pd0 is not None and abs(ta - pd0) > 1.5:
        chk.append(f'· 角度判斷跟實際推頭差 {ta - pd0:+.1f} s'
                   + (' (--virtual-control: 實際推頭看虛擬目標, 真車不在虛擬位置就會差)'
                      if control == 'virtual' else ''))
    if car is not None and st.get('err_car_med') is not None:
        if st['err_car_med'] > 30:
            chk.append(f'⚠ 偵測位置離真車中位數 {st["err_car_med"]:.0f} m — 先看沿路 (俯角) 還是橫向 (方位) 偏')
        else:
            chk.append(f'✓ 偵測位置離真車中位數 {st["err_car_med"]:.0f} m')
    if st.get('calib_n'):
        dd, db = st['calib_dep_err'], st['calib_brg_err']
        if abs(dd) > 0.5:
            chk.append(f'⚠ 俯角偏 {dd:+.2f}° → GIMBAL_PITCH_OFFSET_DEG 建議加 {-dd:+.2f} '
                       f'(角度判斷的推頭時機會差 {abs(dd):.1f}°; 地面高低也會影響這個估計)')
        if abs(db) > 1.0:
            chk.append(f'⚠ 方位偏 {db:+.2f}° → GIMBAL_YAW_OFFSET_DEG 建議加 {db:+.2f}')
        if abs(dd) <= 0.5 and abs(db) <= 1.0:
            chk.append('✓ 雲台俯角 / 方位跟真車一致 (偏差 < 0.5° / 1°)')
    ys = st.get('yaw_diff_std')
    if ys is not None:
        chk.append(('✓' if ys < 2.0 else '⚠') + f' 雲台 yaw − 飛控航向 標準差 {ys:.1f}°'
                   + (' (穩定)' if ys < 2.0 else ' (不穩, 維持 GIMBAL_YAW_MODE=none)'))
    e2e = st.get('e2e_med')
    if e2e is not None and e2e > 150:
        chk.append(f'⚠ 影像延遲中位數 {e2e:.0f} ms (偏大)')
    return nums, chk

