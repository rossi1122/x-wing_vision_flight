#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_vision_guidance.py — 全視覺版測試 (不需要 ROS / 飛控 / 雲台)
═══════════════════════════════════════════════════════════════════
1. 幾何: 方位、水平距、DIVE 補償的正負號 (手算期望值, 不靠模擬器)
2. 狀態機: 鎖定、推頭條件、跟丟逾時、DIVE 交棒要看得到車;
           VISION_GUIDANCE=False 時行為與原本相同
3. 閉迴路模擬 (xwing_vision_sim): 各種車子情境的結果合理
4. 實飛主程式 xwing_vision_flight.main(): 假 rospy + 假時鐘 + 模擬雲台,
   走完「撥 GUIDED → 鎖定 → PRE_DIVE → DIVE → PULLOUT → DONE」,
   以及資料鏈中斷時拒絕啟動、沒有車時逾時結束

用法: python3 test_vision_guidance.py      (最後印 PASS / FAIL)
"""

import csv
import glob
import math
import os
import sys
import types

REPO = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, REPO)

FAILS = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


# ══════════════════════════════════════════════════════════════════
# 1. 幾何
# ══════════════════════════════════════════════════════════════════
def test_geometry():
    print("[1] 幾何")
    from xwing_vision_config import XwingVisionConfig
    from xwing_vision_common import build_gcfg, control_camera
    from vision_guidance import measure

    cfg = XwingVisionConfig()
    gcfg = build_gcfg(cfg, True, cfg.PREDIVE_DIST_M)
    ctrl = control_camera(cfg, gcfg)

    def pkt(u, v, pitch, roll=0.0):
        return {'target_cx': u, 'target_cy': v, 'fx': 1000.0, 'fy': 1000.0,
                'cx': 960.0, 'cy': 540.0, 'pitch': pitch, 'roll': roll}

    # 往南飛 (yaw 180), 雲台 −30°, 車在畫面正中央 → 方位 180, 水平距 240/tan30
    m = measure(pkt(960, 540, -30.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(abs(m['ground_m'] - 415.69) < 0.5, f"正中央: 水平距 {m['ground_m']:.1f} ≈ 415.7")
    check(abs(m['bearing_deg'] - 180.0) < 0.01, f"正中央: 方位 {m['bearing_deg']:.2f} = 180")
    check(abs(m['depress_deg'] - 30.0) < 0.01, "正中央: 俯角 30°")
    # 對地 pitch: 飛機機頭抬 5° 不影響結果
    m5 = measure(pkt(960, 540, -30.0), (0.0, 5.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(abs(m5['ground_m'] - m['ground_m']) < 1e-6, "機頭 pitch 不影響距離 (雲台回報對地角)")
    # 車在畫面右邊 → 方位往右 (往南飛時右邊 = 西, 方位 > 180)
    mr = measure(pkt(960 + 100, 540, -30.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(mr['bearing_off_deg'] > 0 and mr['bearing_deg'] > 180.0,
          f"車在右邊: 偏機頭 {mr['bearing_off_deg']:+.2f}° (右正)")
    # 車在畫面下面 → 比較近
    md = measure(pkt(960, 540 + 100, -30.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(md['ground_m'] < m['ground_m'], f"車在下面: 水平距 {md['ground_m']:.0f} < 416")
    # 雲台偏差校正: 光軸實際往下多 1° (回報 −29 實際 −30) → offset −1 補回
    cfg_b = XwingVisionConfig(GIMBAL_PITCH_OFFSET_DEG=-1.0)
    mb = measure(pkt(960, 540, -29.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg_b, ctrl)
    check(abs(mb['ground_m'] - 415.69) < 0.5, "GIMBAL_PITCH_OFFSET_DEG 正確補償")
    # DIVE 補償: 機頭 −27°, 相機對地 −30°, 車剛好在機頭正前方
    #   機頭方向的俯角 27° → 在相機裡位於中心上方 3°; 補償後應在控制相機正中央
    v_img = 540 - 1000 * math.tan(math.radians(3.0))
    mc = measure(pkt(960, v_img, -30.0), (0.0, -27.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(abs(mc['comp_u'] - ctrl[2]) < 0.5 and abs(mc['comp_v'] - ctrl[3]) < 0.5,
          f"DIVE 補償: 車在機頭正前方 → 控制像素 ({mc['comp_u']:.1f},{mc['comp_v']:.1f}) "
          f"≈ 中心 ({ctrl[2]:.1f},{ctrl[3]:.1f})")
    # 射線朝上 → 無效
    check(measure(pkt(960, 0, 20.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl) is None,
          "射線朝上 → 無效")


# ══════════════════════════════════════════════════════════════════
# 2. 狀態機
# ══════════════════════════════════════════════════════════════════
def _sm(vision=True, **ov):
    from xwing_vision_config import XwingVisionConfig
    from xwing_vision_common import build_gcfg
    from dive_guidance import MissionStateMachine
    cfg = XwingVisionConfig(**ov)
    sm = MissionStateMachine(build_gcfg(cfg, True, cfg.PREDIVE_DIST_M, vision=vision),
                             verbose=False)
    sm.arm()
    return sm


def _inp(t, **kw):
    d = {'t': t, 'guided': True, 'agl_m': 240.0, 'airspeed_mps': 30.0,
         'gs_mps': 30.0, 'vn': -30.0, 've': 0.0, 'vd': 0.0,
         'yaw_true_deg': 180.0, 'pitch_deg': 0.0, 'roll_deg': 0.0,
         'ac_n': 600.0, 'ac_e': 0.0, 'tgt_n': 0.0, 'tgt_e': 0.0, 'tgt_valid': True,
         'ground_m': 600.0, 'predive_dist_m': 600.0, 'bbox_detected': True,
         'vis_age_s': None, 'vis_hits': 0, 'vis_bearing_deg': None,
         'vis_bearing_off_deg': None, 'vis_ground_med_m': None,
         'real_control_detected': False}
    d.update(kw)
    return d


def test_state_machine():
    print("[2] 狀態機")
    see = dict(vis_age_s=0.05, vis_bearing_deg=185.0, vis_bearing_off_deg=5.0)
    # 鎖定要視窗內 10 筆
    sm = _sm()
    sm.update(_inp(0.0, vis_hits=9, vis_ground_med_m=500.0, **see))
    check(not sm._vis_locked, "視窗內 9 筆還不鎖定")
    out = sm.update(_inp(0.02, vis_hits=10, vis_ground_med_m=500.0, **see))
    check(sm._vis_locked and out['vis_locked'] == 1, "視窗內 10 筆 → 鎖定")
    check(abs(out['heading_cmd_deg'] - 185.0) < 1e-6, "鎖定後航向 = 視覺方位")
    # 距離 ≤ 415.69 但偏機頭 > 10° → 不推頭
    out = sm.update(_inp(0.04, vis_hits=11, vis_ground_med_m=410.0,
                         vis_age_s=0.05, vis_bearing_deg=200.0, vis_bearing_off_deg=20.0))
    check(out['mode'] == 'SEARCH', "偏機頭 20° → 先不推頭")
    out = sm.update(_inp(0.06, vis_hits=12, vis_ground_med_m=410.0, **see))
    check(out['mode'] == 'PRE_DIVE' and out['pitch_cmd_deg'] == -30.0,
          "距離 410 m、偏 5° → PRE_DIVE, pitch −30°")
    # 虛擬目標距離不影響 (虛擬 600 m 還沒到也推頭了) — 上面已驗
    # PRE_DIVE: pitch 到位但看不到車 → 不交棒
    out = sm.update(_inp(1.0, pitch_deg=-29.0, vis_hits=13, vis_ground_med_m=380.0,
                         real_control_detected=False, **see))
    check(out['mode'] == 'PRE_DIVE', "pitch 到位但補償像素無效 → 不進 DIVE")
    out = sm.update(_inp(1.02, pitch_deg=-29.0, vis_hits=14, vis_ground_med_m=380.0,
                         real_control_detected=True, real_control_u=960.0,
                         real_control_v=545.0, real_control_cap_t=1.0, **see))
    check(out['mode'] == 'DIVE', "pitch 到位且看得到車 → DIVE")
    # DIVE 跟丟 1 s → ABORT
    out = sm.update(_inp(1.5, pitch_deg=-30.0, agl_m=220.0, vis_age_s=0.6))
    check(out['mode'] == 'DIVE', "DIVE 跟丟 0.6 s → 還在 DIVE (保持命令)")
    out = sm.update(_inp(2.0, pitch_deg=-30.0, agl_m=215.0, vis_age_s=1.1))
    check(out['mode'] == 'ABORT' and out['abort_reason'] == 'VISION_LOST_DIVE',
          "DIVE 跟丟 1.1 s → ABORT")

    # 沒鎖定逾時 → DONE (不拉起)
    sm = _sm()
    sm.update(_inp(0.0))
    out = sm.update(_inp(25.1))
    check(out['mode'] == 'DONE' and out['abort_reason'] == 'VISION_NOT_FOUND',
          "25 s 沒看到車 → DONE (VISION_NOT_FOUND)")
    # 鎖定後跟丟 5 s → DONE
    sm = _sm()
    sm.update(_inp(0.0, vis_hits=10, vis_ground_med_m=550.0, **see))
    out = sm.update(_inp(3.0, vis_hits=0, vis_age_s=3.0, vis_bearing_deg=185.0))
    check(out['mode'] == 'SEARCH' and abs(out['heading_cmd_deg'] - 185.0) < 1e-6,
          "SEARCH 跟丟 3 s → 保持最後方位")
    out = sm.update(_inp(5.2, vis_hits=0, vis_age_s=5.2, vis_bearing_deg=185.0))
    check(out['mode'] == 'DONE' and out['abort_reason'] == 'VISION_LOST_SEARCH',
          "SEARCH 跟丟 5.2 s → DONE")
    # 太晚鎖定 (< 300 m) → DONE
    sm = _sm()
    out = sm.update(_inp(0.0, vis_hits=10, vis_ground_med_m=280.0, **see))
    check(out['mode'] == 'DONE' and out['abort_reason'].startswith('VISION_TOO_CLOSE'),
          "鎖定時只剩 280 m → DONE (避免太陡)")
    # PRE_DIVE 跟丟 1.5 s → ABORT
    sm = _sm()
    sm.update(_inp(0.0, vis_hits=10, vis_ground_med_m=400.0, **see))
    out = sm.update(_inp(1.6, vis_age_s=1.6, vis_bearing_deg=185.0))
    check(out['mode'] == 'ABORT' and out['abort_reason'] == 'VISION_LOST_PREDIVE',
          "PRE_DIVE 跟丟 1.6 s → ABORT")
    # 虛擬模式不受視覺欄位影響 (原行為)
    sm = _sm(vision=False)
    out = sm.update(_inp(0.0, ground_m=400.0, predive_dist_m=400.0))
    check(out['mode'] == 'PRE_DIVE', "VISION_GUIDANCE=False: 照虛擬距離推頭 (原行為)")


# ══════════════════════════════════════════════════════════════════
# 3. 閉迴路模擬
# ══════════════════════════════════════════════════════════════════
def test_sim():
    print("[3] 閉迴路模擬")
    from xwing_vision_sim import simulate
    # 車照虛擬目標開: 幾何與虛擬控制相同 → miss 應該差不多 (純追擊無前置, 兩者都打在車後)
    r = simulate(vision=True, car_case='virtual')
    rvv = simulate(vision=False, car_case='virtual')
    check(r['miss_car'] is not None
          and abs(r['miss_car']['miss_total'] - rvv['miss_car']['miss_total']) < 10.0,
          f"車照虛擬目標開: 全視覺 miss {r['miss_car']['miss_total']:.1f} m ≈ "
          f"虛擬控制 {rvv['miss_car']['miss_total']:.1f} m (同一套追擊幾何)")
    check(r['predive'] and abs(r['predive']['vis'] - r['predive']['true']) < 15.0,
          "推頭時視覺水平距與真實相差 < 15 m")
    # 車慢 (11/04): 全視覺在「真實」415 m 推頭; 虛擬控制被假想車騙, 推頭時真車只剩 ~340 m
    rv = simulate(vision=True, car_case='slow')
    rb = simulate(vision=False, car_case='slow')
    check(abs(rv['predive']['true'] - 415.7) < 15.0,
          f"車慢: 全視覺推頭時真實水平距 {rv['predive']['true']:.0f} m ≈ 416")
    check(abs(rb['predive']['true'] - 415.7) > 50.0,
          f"車慢: 虛擬控制推頭時真實水平距 {rb['predive']['true']:.0f} m (偏離 > 50 m)")
    # 車停著沒開: 全視覺打得到, 虛擬控制打在假想車的位置
    rs = simulate(vision=True, car_case='stopped')
    rsb = simulate(vision=False, car_case='stopped')
    check(rs['miss_car']['miss_total'] < 20.0 < rsb['miss_car']['miss_total'],
          f"車沒開: 全視覺 miss {rs['miss_car']['miss_total']:.1f} m, "
          f"虛擬控制 {rsb['miss_car']['miss_total']:.1f} m")
    rbo = simulate(vision=True, car_case='slow', blackout=(5.5, 7.5))
    check(rbo['abort'] == 'VISION_LOST_DIVE' and rbo['miss_car'] is None,
          "DIVE 中資料鏈斷 2 s → VISION_LOST_DIVE 拉起中止")
    rn = simulate(vision=True, car_case='none')
    check(rn['abort'] == 'VISION_NOT_FOUND' and rn['predive'] is None,
          "沒有車 → 不俯衝, VISION_NOT_FOUND")
    rd = simulate(vision=True, car_case='slow', dropout=0.3)
    check(rd['miss_car'] is not None and rd['lock']['t'] < 1.0
          and abs(rd['predive']['true'] - 415.7) < 20.0,
          "30% 漏偵測: 1 s 內鎖定, 仍在 ~416 m 推頭並完成俯衝")


# ══════════════════════════════════════════════════════════════════
# 4. 實飛主程式 (假 ROS)
# ══════════════════════════════════════════════════════════════════
def run_flight(script, car_present=True, link_ok_from=0.0, case='vtest', stop_at=60.0):
    """跑一次 xwing_vision_flight.main(). script = [(t_rel, guided), ...]."""
    import importlib
    import time as _time
    clk = {'t': 1000.0}
    real_time, real_mono = _time.time, _time.monotonic
    _time.time = lambda: clk['t']
    _time.monotonic = lambda: clk['t']
    T0 = clk['t']
    DT = 0.02
    stop = {'v': False}

    rospy = types.ModuleType('rospy')

    class Rate:
        def __init__(self, hz):
            pass

        def sleep(self):
            step()
    rospy.Rate = Rate
    rospy.init_node = lambda *a, **k: None
    rospy.is_shutdown = lambda: stop['v']
    rospy.sleep = lambda s: step()
    sys.modules['rospy'] = rospy
    for mod, names in {
            'mavros_msgs': [], 'mavros_msgs.msg': ['State', 'RCOut', 'AttitudeTarget',
                                                   'NavControllerOutput', 'VFR_HUD',
                                                   'HomePosition'],
            'mavros_msgs.srv': ['ParamGet'], 'sensor_msgs': [],
            'sensor_msgs.msg': ['NavSatFix', 'Imu', 'BatteryState', 'TimeReference'],
            'std_msgs': [], 'std_msgs.msg': ['Float64'], 'geometry_msgs': [],
            'geometry_msgs.msg': ['TwistStamped']}.items():
        m = types.ModuleType(mod)
        for n in names:
            setattr(m, n, type(n, (), {}))
        sys.modules[mod] = m

    import xwing_vision_flight as F
    importlib.reload(F)
    import xwing_uart_vision
    from attitude_time_history import AngleHistory
    from vision_sim_camera import SimCar, SimGimbalCamera

    cfg = F.XwingVisionConfig()
    S0 = cfg.MOVING_START_GPS
    R = 6378137.0

    def ne2gps(n, e):
        return (S0[0] + math.degrees(n / R),
                S0[1] + math.degrees(e / (R * math.cos(math.radians(S0[0])))))

    AC = {'n': 613.0, 'e': 0.0, 'h': 240.0, 'V': 30.0, 'pitch': 0.0, 'chi': 180.0}
    BR = {'out': None}
    t_guided = next((tr for tr, g in script if g), 0.0)
    car = SimCar(0.0, 0.0, 180.0, t_start=t_guided, vmax=10.0, present=car_present)
    cam = SimGimbalCamera(seed=3)
    poses = AngleHistory(size=3000)

    def guided_now():
        g = False
        for tr, gv in script:
            if clk['t'] - T0 >= tr:
                g = gv
        return g

    def step():
        clk['t'] += DT
        trel = clk['t'] - T0
        out = BR['out']
        g = guided_now()
        active = g and out and out.get('cmd_active')
        if active and out['mode'] not in ('DONE', 'SEARCH'):
            tgt = out['pitch_cmd_deg']
        else:
            tgt = max(-5, min(8, 0.5 * (240 - AC['h'])))
        AC['pitch'] += max(-15 * DT, min(15 * DT, tgt - AC['pitch']))
        if active and out['mode'] in ('SEARCH', 'DIVE'):
            err = (out['heading_cmd_deg'] - AC['chi'] + 180) % 360 - 180
            AC['chi'] += max(-10 * DT, min(10 * DT, err / 2.0 * DT * 50))
        if g and out and out['mode'] == 'DIVE':
            AC['V'] = min(60, AC['V'] + 3 * DT)
        elif not (g and out and out['mode'] == 'PRE_DIVE'):
            AC['V'] += (30 - AC['V']) * 0.02
        v, c = AC['V'], math.radians(AC['chi'])
        AC['h'] += v * math.sin(math.radians(AC['pitch'])) * DT
        hv = v * math.cos(math.radians(AC['pitch']))
        AC['n'] += hv * math.cos(c) * DT
        AC['e'] += hv * math.sin(c) * DT
        poses.append(clk['t'], (0.0, AC['pitch'], AC['chi'] % 360, AC['chi'] % 360))
        cam.enabled = trel >= link_ok_from
        cam.step(trel, DT, {'n': AC['n'], 'e': AC['e'], 'agl': AC['h'],
                            'yaw': AC['chi'] % 360}, car)
        if trel > stop_at:
            stop['v'] = True

    class FakeIO:
        def __init__(self):
            self.pose_history = poses

        def snapshot(self):
            lat, lon = ne2gps(AC['n'], AC['e'])
            g = guided_now()
            c = math.radians(AC['chi'])
            hv = AC['V'] * math.cos(math.radians(AC['pitch']))
            return {'guided': g, 'lat': lat, 'lon': lon, 'rel_alt': AC['h'],
                    'compass_hdg': AC['chi'] % 360, 'pitch_deg': AC['pitch'],
                    'roll_deg': 0.0, 'airspeed': AC['V'],
                    'vn': hv * math.cos(c), 've': hv * math.sin(c),
                    'vd': -AC['V'] * math.sin(math.radians(AC['pitch'])),
                    'yaw_rate_dps': 0.0, 'gps_unix': None, 'p_dps': 0, 'q_dps': 0,
                    'r_dps': 0, 'nz_g': 1, 'fc_pitch_tgt_deg': None,
                    'fc_nav_pitch_deg': None, 'fc_nav_roll_deg': None,
                    'fc_nav_bearing_deg': None, 'fc_mode': 'GUIDED' if g else 'AUTO',
                    'batt_v': 16.0, 'rcout': [1500] * 8}

    class FakeBridge:
        def __init__(self, **k):
            pass

        def start(self):
            pass

        def stop(self):
            pass

        def submit(self, out, *a, **k):
            BR['out'] = out

        def get_debug(self):
            return {}

        def send_tecs_targets_async(self, alt_m=None, speed_mps=None, tag='', on_done=None):
            if on_done:
                on_done(True, True)

    class FakeLink:
        """用 SimGimbalCamera 冒充 UartVisionLink (同一組方法)."""
        def __init__(self, port, baud=115200, **kw):
            self.port, self.baud = port, baud
            self.connected = True
            self.stats = {'frames': 0, 'crc': 0, 'bytes': 0, 'tx_bytes': 0,
                          'serial_err': 0}
            self._req = None

        def start(self):
            pass

        def stop(self):
            pass

        def latest(self):
            # 時鐘: 模擬相機用相對時間, 封包時間換回假時鐘的 monotonic
            p = cam.latest()
            if p is None:
                return None
            p = dict(p)
            p['rx_mono'] += T0
            p['cap_mono'] += T0
            return p

        def request_mode(self, key):
            self._req = key
            cam.request_mode(key)

        def mode_status(self):
            p = cam.latest()
            return self._req, (self._req is not None and p is not None
                               and p['mode'] == int(self._req))

        def is_locked(self, pkt):
            return (pkt is not None and pkt['mode'] == 2
                    and abs(pkt['pitch'] + 30.0) <= 2.0)

        def mode_timing(self):
            return {'key': self._req, 't_req': None, 't_confirm': None,
                    't_settled': None, 'pitch_settled': None}

    F.MavrosIO = FakeIO
    F.ArduPilotAttitudeBridge = FakeBridge
    F.check_airspeed_params = lambda cfg: None
    F.keyboard_thread = lambda flags: None
    xwing_uart_vision.UartVisionLink = FakeLink

    sys.argv = ['x', '--case', case]
    try:
        F.main()
    finally:
        _time.time, _time.monotonic = real_time, real_mono
    ev = sorted(glob.glob(os.path.join(REPO, 'flight_logs', f'*{case}_events.txt')))[-1]
    text = open(ev, encoding='utf-8').read()
    c = ev.replace('_events.txt', '.csv')
    rows = list(csv.DictReader(open(c, encoding='utf-8')))
    for p in (ev, c):
        os.remove(p)
    return text, rows


def test_flight():
    print("[4] 實飛主程式 (假 ROS + 模擬雲台)")
    text, rows = run_flight([(2.0, True)], case='vtest_full')
    for key in ('第 1 趟任務開始', '視覺鎖定目標', 'MODE: SEARCH → PRE_DIVE',
                'MODE: PRE_DIVE → DIVE', 'MODE: DIVE → PULLOUT', 'MODE: PULLOUT → DONE',
                '對視覺估計的目標位置', "雲台狀態 → PRE_DIVE: 模式鍵 '2'"):
        check(key in text, f"完整一趟: events 有「{key}」")
    pre = next((r for r in rows if r['mode'] == 'PRE_DIVE'), None)
    check(pre is not None and pre['vg_ground_med_m'] != ''
          and float(pre['vg_ground_med_m']) <= 416.0,
          "CSV: 推頭那拍的視覺水平距 ≤ 415.69 m")
    check(all(r['control'] == 'vision' for r in rows), "CSV: control = vision")

    text, rows = run_flight([(2.0, True)], car_present=False, case='vtest_nocar',
                            stop_at=40.0)
    check('VISION_NOT_FOUND' in text and 'PRE_DIVE' not in text.split('第 1 趟任務開始')[1],
          "沒有車: 25 s 後 VISION_NOT_FOUND, 不推頭")

    text, rows = run_flight([(2.0, True)], link_ok_from=999.0, case='vtest_nolink',
                            stop_at=6.0)
    check('不啟動任務: 雲台資料鏈中斷' in text, "資料鏈沒資料: 撥 GUIDED 拒絕啟動")


def main():
    test_geometry()
    test_state_machine()
    test_sim()
    test_flight()
    print()
    if FAILS:
        print(f"FAIL ({len(FAILS)} 項)")
        for f in FAILS:
            print("  - " + f)
        sys.exit(1)
    print("PASS")


if __name__ == '__main__':
    main()
