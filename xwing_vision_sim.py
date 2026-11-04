#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_vision_sim.py — 全視覺任務閉迴路模擬 (不需要 ROS / 飛機 / 雲台)
═══════════════════════════════════════════════════════════════════
用【實飛同一套】程式: 狀態機 (dive_guidance, VISION_GUIDANCE) + VisionTracker
(vision_guidance) + 參數 (xwing_vision_config / _common), 接上:
  · 點質量飛機模型 (= hsu_realflight_true/xwing_virtual_sim.py)
  · 真車 + 模擬對方雲台相機 (vision_sim_camera.py), 30 Hz、80 ms 延遲、像素雜訊

看的是: 什麼時候鎖定、推頭時「視覺水平距」和「真實水平距」差多少、
俯衝角、對【真車】的 miss; 也可以跟虛擬目標控制 (xwing_virtual 做法) 比.

車子情境 (--car):
  virtual   車剛好照虛擬目標開 (切 GUIDED 時在 WP4, 立刻 10 m/s)
  slow      11/04 實測: 切 GUIDED 前 5 s 從 WP4 北方 45 m 出發, 0.45 m/s² 慢慢加到 9 m/s
  stopped   車停在 WP4 北方 41 m 沒開 (11/04 第一架 run 3)
  none      沒有車 / 辨識完全看不到 (測逾時中止)

用法:
  python3 xwing_vision_sim.py                       # 全視覺, 車 = slow
  python3 xwing_vision_sim.py --car virtual
  python3 xwing_vision_sim.py --compare             # 同一台車: 全視覺 vs 虛擬目標控制
  python3 xwing_vision_sim.py --dropout 0.3         # 30% 漏偵測
  python3 xwing_vision_sim.py --pitch-bias 1.0      # 雲台角度有 1° 沒校正的偏差
  python3 xwing_vision_sim.py --blackout 14:16      # t=14~16 s 資料鏈中斷
  python3 xwing_vision_sim.py --all                 # 跑全部情境, 印總表
"""

import argparse
import contextlib
import io
import math

from xwing_vision_config import XwingVisionConfig
from xwing_vision_common import (build_gcfg, build_target, target_heading_deg,
                                 control_camera)
from dive_guidance import MissionStateMachine
from virtual_source import VirtualSource, extrapolate_impact, miss_distance
from attitude_time_history import AngleHistory
from vision_guidance import VisionTracker
from vision_sim_camera import SimCar, SimGimbalCamera
from xwing_uart_vision import gimbal_state_label

G = 9.81

CAR_CASES = {
    #           起點 (沿馬路, 正 = 已經過 WP4)  出發時刻 (相對切 GUIDED)  加速度  最高速  有車
    'virtual': dict(s0=0.0, t_start=0.0, accel=None, vmax=10.0, present=True),
    'slow':    dict(s0=-45.0, t_start=-5.0, accel=0.45, vmax=9.0, present=True),
    'stopped': dict(s0=-41.0, t_start=0.0, accel=None, vmax=0.0, present=True),
    'none':    dict(s0=0.0, t_start=0.0, accel=None, vmax=10.0, present=False),
}


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def simulate(cfg=None, vision=True, car_case='slow', guided_dist=613.0,
             dropout=0.0, detect_range=900.0, pitch_bias=0.0, yaw_bias=0.0,
             blackout=None, ld=8.0, q_max=20.0, n_max=2.5, dt=0.02,
             t_after_done=6.0, seed=1, verbose=False):
    cfg = cfg or XwingVisionConfig()
    start = cfg.MOVING_START_GPS
    heading = target_heading_deg(cfg, start)
    gcfg = build_gcfg(cfg, True, cfg.PREDIVE_DIST_M, vision=vision)
    sm = MissionStateMachine(gcfg, verbose=verbose)
    tracker = VisionTracker(cfg, control_camera(cfg, gcfg))
    target = build_target(cfg, True, start, start, heading, cfg.TARGET_SPEED_MPS, t0=0.0)
    with contextlib.redirect_stdout(io.StringIO()):
        virt = VirtualSource(cfg, gcfg, start[0], start[1], target)

    hr = math.radians(heading)
    cc = CAR_CASES[car_case]
    car = SimCar(cc['s0'] * math.cos(hr), cc['s0'] * math.sin(hr), heading,
                 t_start=cc['t_start'], accel=cc['accel'], vmax=cc['vmax'],
                 present=cc['present'])
    cam = SimGimbalCamera(detect_range_m=detect_range, dropout=dropout,
                          pitch_bias_deg=pitch_bias, yaw_bias_deg=yaw_bias, seed=seed)
    poses = AngleHistory(size=2000)

    n, e = -guided_dist * math.cos(hr), -guided_dist * math.sin(hr)
    h = cfg.CRUISE_AGL_M
    V = cfg.V_CRUISE_MPS
    chi = heading
    gam = 0.0
    theta = cfg.ALPHA_NOM_DEG
    vc = cfg.V_CRUISE_MPS
    k_drag = (G / (ld * vc * vc)) if ld and ld > 0 else 0.0
    thr_cruise = k_drag * vc * vc
    tmax = 0.6 * G
    h_ref = h
    v_ref = cfg.SEARCH_SPEED_MPS

    # 切 GUIDED 前先讓雲台追蹤 3 s (STANDBY 已送 '1')
    t = -3.0
    armed = False
    traj = []
    events = []
    mode_prev = None
    t_done = None
    res = {'lock': None, 'predive': None, 'dive': None, 'pullout': None,
           'miss_car': None, 'miss_virtual': None, 'abort': '', 'max_V': V}

    while t < 120.0:
        if not armed and t >= 0.0:
            sm.arm()
            armed = True
        gstate = gimbal_state_label(True, sm.mission_armed, sm.mode)
        cam.request_mode(cfg.GIMBAL_MODE_KEYS.get(gstate))
        cam.enabled = not (blackout and blackout[0] <= t < blackout[1])
        ac = {'n': n, 'e': e, 'agl': h, 'yaw': chi}
        cam.step(t, dt, ac, car)
        poses.append(t, (0.0, theta, chi, chi))

        dive_mode = (sm.mode == sm.DIVE)
        vf = virt.get_fields({'n': n, 'e': e, 'agl': h, 'theta_deg': theta,
                              'chi_deg': chi, 'gimbal_locked': dive_mode, 't': t})
        vg_inp, _row = tracker.update(cam.latest(), t, t, poses, h, n, e)
        cg, sg = math.cos(math.radians(gam)), math.sin(math.radians(gam))
        vn = V * cg * math.cos(math.radians(chi))
        ve = V * cg * math.sin(math.radians(chi))
        vd = -V * sg
        inp = {'t': t, 'guided': True, 'agl_m': h, 'airspeed_mps': V,
               'gs_mps': V * cg, 'vn': vn, 've': ve, 'vd': vd,
               'yaw_true_deg': chi, 'pitch_deg': theta, 'roll_deg': 0.0,
               'ac_n': n, 'ac_e': e, **vf}
        if vision:
            inp.update(vg_inp)
        out = sm.update(inp) if armed else {'mode': 'SEARCH', 'pitch_cmd_deg': theta,
                                            'heading_cmd_deg': chi, 'vis_locked': 0}
        mode = out['mode']
        car_n, car_e, car_vn, car_ve = car.now(t)
        true_ground = math.hypot(car_n - n, car_e - e)

        if out.get('vis_locked') and res['lock'] is None:
            res['lock'] = {'t': t, 'true': true_ground,
                           'vis': vg_inp.get('vis_ground_med_m')}
        if mode != mode_prev:
            events.append((t, mode, out.get('abort_reason', '')))
            if mode == 'PRE_DIVE':
                res['predive'] = {'t': t, 'true': true_ground,
                                  'vis': vg_inp.get('vis_ground_med_m'),
                                  'virt': vf['ground_m']}
            elif mode == 'DIVE':
                res['dive'] = {'t': t, 'true': true_ground}
            elif mode in ('ABORT', 'DONE') and out.get('abort_reason'):
                res['abort'] = out['abort_reason']
            if mode in ('PRE_DIVE', 'PULLOUT'):
                h_ref = cfg.TECS_HOLD_ALT_M
                v_ref = cfg.TECS_SPEED_MPS
            mode_prev = mode
        if sm.pullout_snapshot is not None and res['pullout'] is None:
            sp = sm.pullout_snapshot
            res['pullout'] = {'t': t, 'agl': sp['agl']}
            i_n, i_e, tf = extrapolate_impact(sp['ac_n'], sp['ac_e'], sp['agl'],
                                              sp['vn'], sp['ve'], sp['vd'])
            if i_n is not None:
                res['miss_car'] = miss_distance(i_n, i_e, tf, car_n, car_e,
                                                car_vn, car_ve, sp['vn'], sp['ve'])
                res['miss_virtual'] = miss_distance(i_n, i_e, tf, sp['tgt_n'],
                                                    sp['tgt_e'], sp['tgt_vn'] or 0.0,
                                                    sp['tgt_ve'] or 0.0,
                                                    sp['vn'], sp['ve'])
        traj.append({'t': t, 'mode': mode, 'n': n, 'e': e, 'agl': h, 'V': V,
                     'gamma': gam, 'theta': theta, 'chi': chi,
                     'car_n': car_n, 'car_e': car_e, 'true_ground': true_ground,
                     'vis_ground': vg_inp.get('vis_ground_med_m'),
                     'gimbal_pitch': cam.pitch, 'locked': out.get('vis_locked', 0)})

        # ══ 飛機模型 (= xwing_virtual_sim) ══
        if mode in ('SEARCH', 'DONE'):
            gam_cmd = max(-15.0, min(10.0, 0.3 * (h_ref - h)))
            theta_cmd = gam_cmd + cfg.ALPHA_NOM_DEG
            thrust = max(0.0, min(tmax, thr_cruise + 0.4 * (v_ref - V)
                                  + G * math.sin(math.radians(gam))))
        else:
            theta_cmd = out['pitch_cmd_deg']
            if mode in ('DIVE', 'PULLOUT'):
                thrust = 0.0
            elif mode == 'ABORT':
                thrust = tmax
            else:
                thrust = max(0.0, thr_cruise * (1.0 + (cfg.V_CRUISE_MPS - V) * 0.1))
        dth = (theta_cmd - theta) / 0.25 * dt
        theta += max(-q_max * dt, min(q_max * dt, dth))
        alpha = cfg.ALPHA_DIVE_DEG if mode in ('PRE_DIVE', 'DIVE') else cfg.ALPHA_NOM_DEG
        gdot = (theta - alpha - gam) / 0.5
        gdot_max = math.degrees((n_max - 1.0) * G / max(V, 5.0))
        gam += max(-gdot_max, min(gdot_max, gdot)) * dt
        V += (thrust - k_drag * V * V - G * math.sin(math.radians(gam))) * dt
        V = max(cfg.V_STALL_MPS, V)
        res['max_V'] = max(res['max_V'], V)

        if armed and mode in ('SEARCH', 'DONE', 'ABORT'):
            rate_max = math.degrees(G * math.tan(math.radians(22.0)) / max(V, 5.0))
            err = 0.0 if mode == 'DONE' else _wrap(out['heading_cmd_deg'] - chi)
            chi += max(-rate_max * dt, min(rate_max * dt, err / 2.0 * dt))
        elif mode == 'DIVE':
            chi += _wrap(out['heading_cmd_deg'] - chi) / 2.0 * dt
        chi %= 360.0

        cg, sg = math.cos(math.radians(gam)), math.sin(math.radians(gam))
        n += V * cg * math.cos(math.radians(chi)) * dt
        e += V * cg * math.sin(math.radians(chi)) * dt
        h += V * sg * dt

        if mode == 'DONE' and t_done is None:
            t_done = t
        if t_done is not None and t - t_done >= t_after_done:
            break
        if h <= 0.0:
            break
        t += dt

    g = [r['gamma'] for r in traj if r['mode'] == 'DIVE']
    res.update(traj=traj, events=events, car_case=car_case, vision=vision,
               dive_gamma=(sum(g) / len(g)) if g else None,
               min_agl=min((r['agl'] for r in traj if r['mode'] in ('PULLOUT', 'DONE')
                            and res['pullout']), default=None))
    return res


def _f(x, fmt='{:.0f}'):
    return fmt.format(x) if x is not None else '—'


def print_report(r):
    print("=" * 78)
    print(f" 控制: {'全視覺' if r['vision'] else '虛擬目標 (xwing_virtual)'}   "
          f"車子情境: {r['car_case']}")
    print("=" * 78)
    for t, m, why in r['events']:
        print(f"  t={t:6.2f}s  → {m}" + (f"  ({why})" if why else ""))
    if r['lock']:
        lk = r['lock']
        print(f" 鎖定: t={lk['t']:.2f}s  真實水平距 {lk['true']:.0f} m  視覺 {_f(lk['vis'])} m")
    if r['predive']:
        p = r['predive']
        print(f" 推頭: t={p['t']:.2f}s  真實水平距 {p['true']:.0f} m  視覺 {_f(p['vis'])} m  "
              f"虛擬目標 {p['virt']:.0f} m")
    if r['dive_gamma'] is not None:
        print(f" DIVE 平均航跡角 {r['dive_gamma']:.1f}°  最高空速 {r['max_V']:.1f} m/s  "
              f"拉平後最低 {_f(r['min_agl'])} m")
    if r['miss_car']:
        m = r['miss_car']
        print(f" ★ 對真車 miss = {m['miss_total']:.1f} m "
              f"(沿航跡 {m['miss_lon']:+.1f}, 側向 {m['miss_lat']:+.1f})   "
              f"對虛擬目標 {r['miss_virtual']['miss_total']:.1f} m")
    elif r['abort']:
        print(f" 沒有俯衝完成: {r['abort']}")
    print("=" * 78)


def summary_row(name, r):
    p = r['predive'] or {}
    m = r['miss_car']['miss_total'] if r['miss_car'] else None
    return (f"{name:<34}{_f(r['lock']['t'] if r['lock'] else None, '{:.1f}'):>7}"
            f"{_f(p.get('true')):>8}{_f(p.get('vis')):>8}"
            f"{_f(r['dive_gamma'], '{:.1f}'):>8}{_f(m, '{:.1f}'):>9}  {r['abort']}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--car', choices=sorted(CAR_CASES), default='slow')
    ap.add_argument('--virtual-control', action='store_true',
                    help='用虛擬目標控制 (xwing_virtual 做法) 跑同一台車')
    ap.add_argument('--compare', action='store_true', help='全視覺 vs 虛擬目標控制')
    ap.add_argument('--all', action='store_true', help='跑全部情境, 印總表')
    ap.add_argument('--guided-dist', type=float, default=613.0)
    ap.add_argument('--dropout', type=float, default=0.0, help='漏偵測機率 0~1')
    ap.add_argument('--detect-range', type=float, default=900.0, help='辨識距離上限 m')
    ap.add_argument('--pitch-bias', type=float, default=0.0,
                    help='雲台回報 pitch 與光軸的偏差 (度, 未校正)')
    ap.add_argument('--yaw-bias', type=float, default=0.0, help='光軸偏機頭右方 (度)')
    ap.add_argument('--blackout', type=str, default=None, metavar='T0:T1',
                    help='這段時間資料鏈中斷 (秒, 相對切 GUIDED)')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--verbose', action='store_true')
    args = ap.parse_args()
    bo = tuple(float(x) for x in args.blackout.split(':')) if args.blackout else None
    kw = dict(guided_dist=args.guided_dist, dropout=args.dropout,
              detect_range=args.detect_range, pitch_bias=args.pitch_bias,
              yaw_bias=args.yaw_bias, blackout=bo, seed=args.seed,
              verbose=args.verbose)

    if args.all:
        print(f"{'情境':<34}{'鎖定s':>7}{'推頭真':>8}{'推頭視':>8}{'DIVEγ':>8}"
              f"{'真車miss':>9}  中止原因")
        cases = [('全視覺 / 車照虛擬目標開', dict(car_case='virtual')),
                 ('全視覺 / 車慢 (11/04)', dict(car_case='slow')),
                 ('全視覺 / 車沒開', dict(car_case='stopped')),
                 ('全視覺 / 沒有車', dict(car_case='none')),
                 ('全視覺 / 車慢 + 30% 漏偵測', dict(car_case='slow', dropout=0.3)),
                 ('全視覺 / 車慢 + pitch 偏 1°', dict(car_case='slow', pitch_bias=1.0)),
                 ('全視覺 / 車慢 + yaw 偏 3°', dict(car_case='slow', yaw_bias=3.0)),
                 ('全視覺 / 車慢 + DIVE 中斷線 2 s', dict(car_case='slow', blackout=(5.5, 7.5))),
                 ('虛擬控制 / 車照虛擬目標開', dict(car_case='virtual', vision=False)),
                 ('虛擬控制 / 車慢 (11/04)', dict(car_case='slow', vision=False)),
                 ('虛擬控制 / 車沒開', dict(car_case='stopped', vision=False))]
        for name, c in cases:
            k = dict(kw, **c)
            print(summary_row(name, simulate(**k)))
        return
    if args.compare:
        for v in (True, False):
            print_report(simulate(vision=v, car_case=args.car, **kw))
        return
    print_report(simulate(vision=not args.virtual_control, car_case=args.car, **kw))


if __name__ == '__main__':
    main()
