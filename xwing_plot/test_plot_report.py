"""test_plot_report.py — 不用飛機, 先試畫圖流程 (xwing_plot_report.py)
用假 rospy + 簡單飛機模型跑【真的】xwing_vision_flight.main(), 產生一份完整飛完一趟
(SEARCH → PRE_DIVE → DIVE → PULLOUT → DONE) 的 log, 再做一個假的車子 BIN
(比虛擬目標晚 4 s 出發), 最後自動跑 xwing_plot_report.py --car-bin-dir.
⚠ 飛機模型很粗, 圖裡的數字不代表實飛結果, 只用來確認畫圖流程能跑.

用法: python3 test_plot_report.py            (一趟)
      python3 test_plot_report.py --two-runs (同一份 log 切兩次 GUIDED)
      python3 test_plot_report.py --vision   (加假雲台: pitch/roll 相對地面, 看真車 → 測 Fig 12 / Fig V1~V5)
      python3 test_plot_report.py --vision --vision-control  (目標來自偵測 = 正式模式; 預設 --virtual-control)
      python3 test_plot_report.py --vision --pitch-bias 1    (假雲台俯角偏 1° → summary 應建議 −1)
輸出: xwing_plot/plot_test/ (log + car_bin/)  →  xwing_plot/plot_results/xwing_vision_<時間>_plottest/run1/"""
import sys, types, math, os, glob, shutil, struct, random, subprocess
HERE = os.path.dirname(os.path.abspath(__file__))       # xwing_plot/
REPO = os.path.dirname(HERE)                             # 專案根目錄 (飛行程式在這)
OUT = os.path.join(HERE, 'plot_test')
TWO = '--two-runs' in sys.argv
VIS = '--vision' in sys.argv
VCTRL = '--vision-control' in sys.argv       # 目標來自偵測 (要配 --vision)
# --pitch-bias X: 假雲台回報的 pitch 比光軸實際多 X 度 (測 summary 的校正建議; 正確答案 = −X)
PBIAS = float(sys.argv[sys.argv.index('--pitch-bias') + 1]) if '--pitch-bias' in sys.argv else 0.0
if VCTRL and not VIS:
    sys.exit('--vision-control 要配 --vision (需要假雲台)')
sys.path.insert(0, REPO)
os.chdir(REPO)
import time as _time
CLK = {'t': 1000.0}
_time.time = lambda: CLK['t']
_time.monotonic = lambda: CLK['t']
DT = 0.02
GPS0 = 1793320000.0          # 2026-10-30 附近 (UTC)
rospy = types.ModuleType('rospy')
STOP = {'v': False}
class Rate:
    def __init__(self, hz): pass
    def sleep(self): step()
rospy.Rate = Rate
rospy.init_node = lambda *a, **k: None
rospy.is_shutdown = lambda: STOP['v']
rospy.sleep = lambda s: None
sys.modules['rospy'] = rospy
for mod, names in {'mavros_msgs': [], 'mavros_msgs.msg': ['State','RCOut','AttitudeTarget','NavControllerOutput','VFR_HUD','HomePosition'],
                   'mavros_msgs.srv': ['ParamGet'], 'sensor_msgs': [], 'sensor_msgs.msg': ['NavSatFix','Imu','BatteryState','TimeReference'],
                   'std_msgs': [], 'std_msgs.msg': ['Float64'], 'geometry_msgs': [], 'geometry_msgs.msg': ['TwistStamped']}.items():
    m = types.ModuleType(mod)
    for n in names: setattr(m, n, type(n, (), {}))
    sys.modules[mod] = m
import xwing_vision_flight as F
from attitude_time_history import AngleHistory
cfg = F.XwingVisionConfig()
POSES = AngleHistory(size=4000)          # 取像時刻的飛機姿態 (VisionSource 用)
S0 = cfg.MOVING_START_GPS
R = 6378137.0
def ne2gps(n, e):
    return (S0[0] + math.degrees(n / R),
            S0[1] + math.degrees(e / (R * math.cos(math.radians(S0[0])))))
from xwing_vision_common import target_heading_deg
HDG = target_heading_deg(cfg, S0)       # 目標前進方位 (跟著 config, 往南 180 / 往北 0 都能跑)
def ax_ne(s, x):
    """沿目標方位 s 公尺 (+ = 前進方向), 右側 x 公尺 → (n, e)."""
    h = math.radians(HDG)
    return s * math.cos(h) - x * math.sin(h), s * math.sin(h) + x * math.cos(h)
random.seed(1)
_n0, _e0 = ax_ne(-650.0, -25.0)          # 目標起點後方 650 m, 航線左側 25 m
AC = {'n': _n0, 'e': _e0, 'h': 238.0, 'V': 30.0, 'pitch': 0.0, 'chi': (HDG + 3.0) % 360,
      'roll': 0.0, 'r': 0.0}
BR = {'out': None}
SCRIPT = [(2.0, True)] + ([(40.0, False), (60.0, True)] if TWO else [])
T0 = CLK['t']
def guided_now():
    g = False
    for tr, gv in SCRIPT:
        if CLK['t'] - T0 >= tr: g = gv
    return g
RUD = {'raw': 0.0, 'sent': 0.0}
def step():
    CLK['t'] += DT
    out = BR['out']; g = guided_now()
    if g and out and out.get('cmd_active'):
        tgt = out['pitch_cmd_deg']
    else:
        tgt = max(-5, min(8, 0.5 * (240 - AC['h'])))
    AC['pitch'] += max(-18 * DT, min(18 * DT, (tgt - AC['pitch']) * 4 * DT))
    mode = out['mode'] if (g and out) else 'X'
    if mode == 'DIVE': AC['V'] = min(62, AC['V'] + 2.6 * DT)
    elif mode == 'PRE_DIVE': AC['V'] += 1.0 * DT
    else: AC['V'] += (30 - AC['V']) * 0.02
    # 航向: SEARCH 跟 heading_cmd (FC 協調轉彎), DIVE 用方向舵
    if g and out and out.get('heading_cmd_deg') is not None and mode in ('SEARCH', 'PRE_DIVE'):
        err = (out['heading_cmd_deg'] - AC['chi'] + 180) % 360 - 180
        AC['r'] = max(-8, min(8, 0.8 * err))
    elif mode == 'DIVE':
        AC['r'] = 0.35 * RUD['sent'] + random.gauss(0, 0.3)
    else:
        AC['r'] *= 0.9
    AC['chi'] = (AC['chi'] + AC['r'] * DT) % 360
    AC['roll'] += (2.0 * AC['r'] - AC['roll']) * 0.05
    v = AC['V']
    AC['h'] += v * math.sin(math.radians(AC['pitch'])) * DT
    AC['n'] += v * math.cos(math.radians(AC['pitch'])) * math.cos(math.radians(AC['chi'])) * DT
    AC['e'] += v * math.cos(math.radians(AC['pitch'])) * math.sin(math.radians(AC['chi'])) * DT
    POSES.append(CLK['t'], (AC['roll'], AC['pitch'], AC['chi'] % 360, AC['chi'] % 360))
    if CLK['t'] - T0 > (100 if TWO else 45): STOP['v'] = True
    if TWO and abs(CLK['t'] - T0 - 59.0) < DT / 2:
        _n, _e = ax_ne(-640.0, 20.0)
        AC.update({'n': _n, 'e': _e, 'h': 241.0, 'V': 30.0, 'pitch': 0.0, 'chi': (HDG - 2.0) % 360})

class FakeIO:
    def __init__(self): self.pose_history = POSES
    def snapshot(self):
        lat, lon = ne2gps(AC['n'], AC['e'])
        g = guided_now()
        c = math.cos(math.radians(AC['pitch']))
        vn = AC['V'] * c * math.cos(math.radians(AC['chi']))
        ve = AC['V'] * c * math.sin(math.radians(AC['chi']))
        vd = -AC['V'] * math.sin(math.radians(AC['pitch']))
        return {'guided': g, 'lat': lat, 'lon': lon, 'rel_alt': AC['h'],
                'compass_hdg': AC['chi'], 'pitch_deg': AC['pitch'], 'roll_deg': AC['roll'],
                'airspeed': AC['V'], 'vn': vn, 've': ve, 'vd': vd,
                'yaw_rate_dps': AC['r'], 'gps_unix': GPS0 + CLK['t'] - T0, 'p_dps': 0, 'q_dps': 0,
                'r_dps': AC['r'], 'nz_g': 1, 'fc_pitch_tgt_deg': None,
                'fc_nav_pitch_deg': None, 'fc_nav_roll_deg': None,
                'fc_nav_bearing_deg': None, 'fc_mode': 'GUIDED' if g else 'AUTO',
                'batt_v': 16.4 - 0.004 * (CLK['t'] - T0), 'rcout': [1500] * 8}
class FakeBridge:
    def __init__(self, **k): pass
    def start(self): pass
    def stop(self): pass
    def submit(self, out, *a, **k):
        BR['out'] = out
        if out.get('mode') == 'DIVE' and out.get('pid_dpsi_deg') not in (None, ''):
            raw = cfg.K_LAT_TO_RUDDER * float(out['pid_dpsi_deg'])
            lim = cfg.RUDDER_LIMIT_DEG
            RUD['raw'] = raw
            cmd = max(-lim, min(lim, raw))
            RUD['sent'] += max(-cfg.RUDDER_SLEW_DPS * DT, min(cfg.RUDDER_SLEW_DPS * DT, cmd - RUD['sent']))
        else:
            RUD['raw'] = 0.0; RUD['sent'] = 0.0
    def get_debug(self):
        o = BR['out'] or {}
        if o.get('mode') == 'DIVE':
            return {'brg_mask': 128, 'rudder_raw_deg': round(RUD['raw'], 2),
                    'rudder_cmd_deg': round(max(-30, min(30, RUD['raw'])), 2),
                    'rudder_sent_deg': round(RUD['sent'], 2), 'rudder_limit_deg': 30.0,
                    'lim_by_airspeed': 1.0, 'lim_by_roll': (1.0 if abs(AC['roll']) <= 25 else 0.5),
                    'brg_thr': 0.05}
        return {'brg_mask': 132, 'brg_thr': 0.55}
    def send_tecs_targets_async(self, alt_m=None, speed_mps=None, tag='', on_done=None):
        if on_done: on_done(True, True)
F.MavrosIO = FakeIO
F.ArduPilotAttitudeBridge = FakeBridge
F.check_airspeed_params = lambda cfg: None
F.keyboard_thread = lambda flags: None


# ── 車子 (BIN 與假雲台共用): 比虛擬目標晚 4 s 出發, 7 s 加速到 10 m/s, 起點北方 10 m ──
CAR_LEAD = -4.0
def car_ne(t_rel):
    tm = t_rel - 2.0 + CAR_LEAD          # 相對第 1 次撥 GUIDED
    dist = 0 if tm < 0 else (5 / 7.0 * tm * tm if tm < 7 else 35 + 10 * (tm - 7))
    return ax_ne(dist - 10.0, -1.5)      # 起點後方 10 m 出發, 沿目標方位開


# ── 假雲台 (對方 Orin NX): pitch/roll 相對地面, yaw 跟機頭; '1' 追蹤車子, '2' 鎖 -30° ──
class FakeVision:
    FX = FY = 1400.0
    W, H = 1920, 1080
    def __init__(self, port, baud, **k):
        self.port, self.baud = port, baud
        self.connected = True
        self.stats = {'frames': 0, 'clock': 0, 'bytes': 0, 'header': 0, 'crc': 0, 'payload': 0,
                      'discarded': 0, 'reopen': 0, 'mode_tx': 0, 'serial_err': 0, 'open_fail': 0}
        self.req = None
        self.mode = 0
        self.gp = -20.0
        self.t_req = None
    def start(self): pass
    def stop(self): pass
    def request_mode(self, key):
        if key is not None and key != self.req:
            self.req, self.t_req = key, CLK['t']
    def mode_status(self):
        return self.req, (self.req is not None and self.mode == int(self.req))
    def mode_timing(self):
        return {'key': self.req, 't_req': self.t_req, 't_confirm': None, 't_settled': None,
                'pitch_settled': None}
    def is_locked(self, pkt):
        return pkt is not None and pkt['mode'] == 2 and abs(pkt['pitch'] + 30) <= 2
    def latest(self):
        now = CLK['t']
        if self.req is not None and self.t_req is not None and now - self.t_req > 0.2:
            self.mode = int(self.req)
        cn, ce = car_ne(now - T0)
        dn, de, h = cn - AC['n'], ce - AC['e'], AC['h']
        if self.mode == 2:
            want = -30.0
        else:
            want = -math.degrees(math.atan2(h, math.hypot(dn, de)))
        self.gp += max(-60 * DT, min(60 * DT, want - self.gp))       # 雲台 60°/s
        rid = int(now * 30)                                         # 30 Hz 辨識
        cam = F_R_G2C @ ry(self.gp) @ rz(AC['chi'])
        c = cam @ [dn, de, h]
        ox = oy = float('nan')
        ok = c[2] > 1
        if ok:
            ox = self.FX * c[0] / c[2] + random.gauss(0, 3)
            oy = self.FY * c[1] / c[2] + random.gauss(0, 3)
            ok = abs(ox) < self.W / 2 and abs(oy) < self.H / 2 and random.random() < 0.95
        self.stats['frames'] = rid
        laser = math.sqrt(dn * dn + de * de + h * h) + random.gauss(0, 1) if ok else float('nan')
        return {'rx_mono': now - 0.01, 'result_id': rid, 'ver': 0, 'valid': bool(ok),
                'target_cx': self.W / 2 + ox, 'target_cy': self.H / 2 + oy,
                'offset_x': ox if ok else 0.0, 'offset_y': oy if ok else 0.0,
                'fx': self.FX, 'fy': self.FY, 'cx': self.W / 2, 'cy': self.H / 2,
                'image_width': self.W, 'image_height': self.H,
                'pitch': self.gp + PBIAS + random.gauss(0, 0.1), 'roll': random.gauss(0, 0.2),
                'mode': self.mode, 'detection_age_ms': 30.0, 'angle_age_ms': 5.0,
                'e2e_ms': 80.0, 'e2e_err_ms': 2.0, 'cap_mono': now - 0.08,
                'laser_distance_m': laser,                          # 雷射測到真車的斜距
                # 雲台 yaw: 對地角度, 置中朝機頭 (2026/11/10 確認) → ≈ 機頭方位 + 小抖動
                'yaw': (AC['chi'] + random.gauss(0, 0.3)) % 360.0}

import numpy as _np
F_R_G2C = _np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
def ry(d):
    c, s_ = math.cos(math.radians(d)), math.sin(math.radians(d))
    return _np.array([[c, 0, -s_], [0, 1, 0], [s_, 0, c]])
def rz(d):
    c, s_ = math.cos(math.radians(d)), math.sin(math.radians(d))
    return _np.array([[c, s_, 0], [-s_, c, 0], [0, 0, 1]])
if VIS:
    import xwing_uart_vision
    xwing_uart_vision.UartVisionLink = FakeVision
case = ('plottest2' if TWO else 'plottest') + ('_gimbal' if VIS else '')
case += '_visionctrl' if VCTRL else ''
sys.argv = (['x', '--moving', '--case', case]
            + ([] if VCTRL else ['--virtual-control'])        # 預設 = 偵測只記錄 (飛測計畫)
            + ([] if VIS else ['--no-vision']))
F.main()
f = sorted(glob.glob(os.path.join(REPO, 'flight_logs', f'*{case}_events.txt')))[-1]
c = f.replace('_events.txt', '.csv')
os.makedirs(OUT, exist_ok=True)
for p in (f, c):
    shutil.move(p, os.path.join(OUT, os.path.basename(p)))
print('log ->', os.path.join(OUT, os.path.basename(c)))

# ── 假車子 BIN: 從起點後方 10 m 出發, 沿目標方位 ~10 m/s, GPS 5 Hz ──
def fmt_msg(typ, name, fmt, cols):
    length = 3 + struct.calcsize('<' + ''.join({'Q':'Q','B':'B','I':'I','H':'H','L':'i','f':'f'}[x] for x in fmt))
    return b'\xa3\x95\x80' + struct.pack('<BB4s16s64s', typ, length, name.encode(), fmt.encode(), cols.encode())
buf = fmt_msg(130, 'GPS', 'QBBIHLLf', 'TimeUS,I,Status,GMS,GWk,Lat,Lng,Spd')
for k in range(int(130 * 5)):
    tt = k * 0.2
    unix = GPS0 - 20 + tt
    gps_s = unix - 315964800 + 18
    wk = int(gps_s // 604800); ms = int(round((gps_s - wk * 604800) * 1000))
    tm = tt - 20 - 2.0 + CAR_LEAD   # 相對 GUIDED 時刻
    spd = 0 if tm < 0 else min(10.0, 10.0 * tm / 7.0)
    n, e = car_ne(tt - 20)
    n += random.gauss(0, 0.6); e += random.gauss(0, 0.6)
    la, lo = ne2gps(n, e)
    buf += b'\xa3\x95' + struct.pack('<BQBBIHiif', 130, int(tt * 1e6), 0, 3, ms, wk,
                                       int(round(la * 1e7)), int(round(lo * 1e7)), spd + random.gauss(0, 0.15))
os.makedirs(os.path.join(OUT, 'car_bin'), exist_ok=True)
open(os.path.join(OUT, 'car_bin', '00000031.BIN'), 'wb').write(buf)
print('bin ->', os.path.join(OUT, 'car_bin', '00000031.BIN'))

print('\n[test] 開始畫圖 ...\n')
r = subprocess.call([sys.executable, os.path.join(HERE, 'xwing_plot_report.py'),
                     os.path.join(OUT, os.path.basename(c)),
                     '--car-bin-dir', os.path.join(OUT, 'car_bin')])
print('\n[test] ' + ('完成 — 看 plot_results/ 裡的 report.pdf' if r == 0 else '畫圖失敗'))
sys.exit(r)
