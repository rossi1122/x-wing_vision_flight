#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_vision_flight.py — X-wing 目標偵測版實飛主程式
═══════════════════════════════════════════════════════════════════
由 hsu_realflight_true/xwing_virtual_flight.py (2026/11/04 實飛版) 改出來.
飛法、參數、狀態機與判斷邏輯、重複撥 GUIDED、啟動檢查、雲台 UART 全部相同,
★ 唯一的差別: 狀態機的「目標」輸入來源
      原本: 虛擬軌跡 (virtual_source.VirtualSource)
      現在: 對方雲台的目標偵測 (vision_guidance.VisionSource)

    MAVROS 姿態 (含取像時刻的姿態歷史) ─┐
    對方雲台 UART (目標像素 + 對地 pitch) ─┴→ VisionSource
        → tgt_n/tgt_e、predive_dist_m、bbox_u/v ... (與虛擬目標同名欄位)
        → MissionStateMachine (dive_guidance, 原封不動) → AttitudeBridge → 飛控

任務 (同 xwing_virtual):
  AUTO 240 m / 30 m/s → WP3 撥 GUIDED = 任務開始
  → SEARCH: 飛控 43002 朝目標位置飛 (還沒偵測到車前朝虛擬目標帶路)
  → 到目標水平距 ≤ 415.69 m → PRE_DIVE 直接命令 pitch −30°
  → pitch 到 −27° 內 (且看得到目標) → DIVE: 目標在畫面的位置 PID 修 pitch / 方向舵
  → AGL ≤ 140 m (或空速 ≥ 60 m/s) → PULLOUT → DONE: TECS 140 m / 30 m/s
  PRE_DIVE / DIVE 看不到目標 > 1.5 s → ABORT (原 ABORT_TARGET_LOST_S)

用法:
  python3 xwing_vision_flight.py                       # 目標來自偵測 (預設)
  python3 xwing_vision_flight.py --virtual-control     # 目標來自虛擬軌跡 (= xwing_virtual, 偵測只記錄)
  python3 xwing_vision_flight.py --vision-port /dev/ttyUSB0
  python3 xwing_vision_flight.py --bench               # 掛機

雲台模式鍵 (同 xwing_virtual): SEARCH 送 '1' 追蹤, PRE_DIVE / DIVE 送 '2' 鎖定 −30° (對地).
"""

import argparse
import csv
import math
import os
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from xwing_vision_config import XwingVisionConfig
from xwing_vision_common import (build_gcfg, build_target, apply_bridge_cfg,
                                 target_heading_deg, road_length_m, control_camera)
from dive_guidance import MissionStateMachine
from attitude_bridge import ArduPilotAttitudeBridge, AttitudeBridgeConfig
from attitude_time_history import AngleHistory
from virtual_source import VirtualSource, gps_to_ned, extrapolate_impact, miss_distance
from vision_guidance import VisionSource, VG_CSV_FIELDS, predive_trigger_angle_deg
from xwing_uart_vision import VISION_CSV_FIELDS, vision_row, gimbal_state_label

try:
    import rospy
    from mavros_msgs.msg import (State, RCOut, AttitudeTarget,
                                 NavControllerOutput)
    from mavros_msgs.srv import ParamGet
    from sensor_msgs.msg import NavSatFix, Imu, BatteryState, TimeReference
    from std_msgs.msg import Float64
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import VFR_HUD, HomePosition
    ROS_OK = True
except ImportError:
    ROS_OK = False


LOOP_HZ = 50.0

# NED 原點 = 虛擬目標起點 (固定 GPS 點), 與飛控 home 無關.
NED_ORIGIN = None

# ── 主 log 欄位 (50Hz 控制迴路, 逐拍) — 與 xwing_virtual 相同, 再加全視覺欄位 ──
CSV_FIELDS = [
    't', 'gps_unix', 'run', 'mode', 'guided', 'control',
    'ac_n', 'ac_e', 'agl', 'V', 'chi_deg', 'gamma_deg', 'theta_deg',
    'roll_deg', 'yaw_deg', 'vn', 've', 'vd', 'dt',
    # 虛擬目標 (帶路 / 對照)
    'tgt_n', 'tgt_e', 'slant_m', 'ground_m', 'predive_dist_m',
    'virt_bbox_u', 'virt_bbox_v', 'virt_detected',
    # PID setpoint + 命令
    'virt_u_sp', 'virt_v_sp', 'pitch_cmd', 'heading_cmd', 'throttle',
    'pid_u_err_deg', 'pid_v_err_deg',
    'pid_lat_p', 'pid_lat_i', 'pid_lat_d',
    'pid_lon_p', 'pid_lon_i', 'pid_lon_d',
    'pid_dpsi_deg', 'pid_dtheta_deg', 'pid_compute_ms',
    'brg_mask', 'rudder_raw_deg', 'rudder_cmd_deg', 'rudder_sent_deg',
    'rudder_limit_deg', 'lim_by_airspeed', 'lim_by_roll',
    'hdg_cmd_ok_cnt', 'lateral_fallback', 'brg_stale', 'brg_stale_events',
    'brg_entry_roll0', 'brg_hdg_want',
    'img_err_u_pct', 'img_err_v_pct', 'glide_slope_err_deg',
    'miss_pred_m', 'miss_pred_lon_m', 'miss_pred_lat_m', 'impact_t_fall_s',
    'p_dps', 'q_dps', 'r_dps', 'nz_g',
    'fc_pitch_tgt_deg', 'fc_nav_pitch_deg', 'fc_nav_roll_deg',
    'fc_nav_bearing_deg', 'fc_mode', 'batt_v',
    'rcout1', 'rcout2', 'rcout3', 'rcout4',
    'lat', 'lon', 'tgt_lat', 'tgt_lon',
    'abort_reason', 'ctrl_ms',
]
CSV_FIELDS += VG_CSV_FIELDS          # 偵測: 方位 / 水平距 / 畫面位置 / 目標位置估計
CSV_FIELDS += VISION_CSV_FIELDS      # 對方封包原始值 (與 xwing_virtual 相同)


# ══════════════════════════════════════════════════════════════════
#  MAVROS 訂閱 (= xwing_virtual_flight.MavrosIO, 加回取像時刻用的姿態歷史)
# ══════════════════════════════════════════════════════════════════
class MavrosIO:
    GPS_TIME_STALE_S = 5.0

    def __init__(self):
        self.lock = threading.Lock()
        self.guided = False
        self.connected = False
        self.lat = self.lon = None
        self.rel_alt = None
        self.compass_hdg = None
        self.vn = self.ve = self.vd = None
        self.yaw_rate_dps = 0.0
        self.airspeed = None
        self.pitch_deg = self.roll_deg = None
        self.home_lat = self.home_lon = None
        self.last_pos_t = 0.0
        self.p_dps = self.q_dps = self.r_dps = None
        self.nz_g = None
        self.fc_mode = ''
        self.fc_pitch_tgt_deg = None
        self.fc_nav_pitch_deg = None
        self.fc_nav_roll_deg = None
        self.fc_nav_bearing_deg = None
        self.batt_v = None
        self.rcout = [None] * 8
        self.gps_unix_ref = None
        self.gps_rx_t = None
        # (roll, pitch, yaw, course) 依 IMU 收到時刻 (monotonic) — 視覺取像時刻查姿態
        self.pose_history = AngleHistory()

        rospy.Subscriber("/mavros/state", State, self._cb_state)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self._cb_gps)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self._cb_alt)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self._cb_hdg)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self._cb_vel)
        rospy.Subscriber("/mavros/imu/data", Imu, self._cb_imu)
        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self._cb_vfr)
        rospy.Subscriber("/mavros/home_position/home", HomePosition, self._cb_home)
        rospy.Subscriber("/mavros/rc/out", RCOut, self._cb_rcout)
        rospy.Subscriber("/mavros/battery", BatteryState, self._cb_batt)
        rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget,
                         self._cb_fc_target)
        rospy.Subscriber("/mavros/nav_controller_output", NavControllerOutput,
                         self._cb_nav_out)
        rospy.Subscriber("/mavros/time_reference", TimeReference, self._cb_timeref)

    def _cb_state(self, m):
        with self.lock:
            self.connected = m.connected
            self.guided = (m.mode == 'GUIDED')
            self.fc_mode = m.mode

    def _cb_rcout(self, m):
        with self.lock:
            ch = list(m.channels[:8])
            self.rcout = ch + [None] * (8 - len(ch))

    def _cb_batt(self, m):
        with self.lock:
            self.batt_v = m.voltage

    def _cb_timeref(self, m):
        with self.lock:
            self.gps_unix_ref = m.time_ref.to_sec()
            self.gps_rx_t = time.time()

    def _cb_nav_out(self, m):
        with self.lock:
            self.fc_nav_roll_deg = float(m.nav_roll)
            self.fc_nav_pitch_deg = float(m.nav_pitch)
            self.fc_nav_bearing_deg = float(m.nav_bearing)

    def _cb_fc_target(self, m):
        with self.lock:
            q = m.orientation
            sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            self.fc_pitch_tgt_deg = -math.degrees(math.asin(sinp))

    def _cb_gps(self, m):
        with self.lock:
            self.lat, self.lon = m.latitude, m.longitude
            self.last_pos_t = time.time()

    def _cb_alt(self, m):
        with self.lock:
            self.rel_alt = m.data

    def _cb_hdg(self, m):
        with self.lock:
            self.compass_hdg = m.data

    def _cb_vel(self, m):
        with self.lock:
            self.vn = m.twist.linear.y
            self.ve = m.twist.linear.x
            self.vd = -m.twist.linear.z

    def _cb_imu(self, m):
        with self.lock:
            self.yaw_rate_dps = -math.degrees(m.angular_velocity.z)
            self.p_dps = math.degrees(m.angular_velocity.x)
            self.q_dps = -math.degrees(m.angular_velocity.y)
            self.r_dps = -math.degrees(m.angular_velocity.z)
            self.nz_g = m.linear_acceleration.z / 9.81
            q = m.orientation
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            # ★ NED pitch (nose-up 正): 掛機實測 MAVROS ENU/FLU 此式解出
            #   nose-up 為負 → 取負轉 NED. 增量式 PID 直接用, 符號錯會發散.
            self.pitch_deg = -math.degrees(math.asin(sinp))
            self.roll_deg = math.degrees(math.atan2(sinr, cosr))
            # 姿態歷史 (同 run_vision_flight): ENU yaw → 真方位, course 用地速方向
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = (90.0 - math.degrees(math.atan2(siny, cosy))) % 360.0
            vn, ve = self.vn, self.ve
            course = (math.degrees(math.atan2(ve, vn))
                      if vn is not None and ve is not None and vn * vn + ve * ve > 4
                      else (self.compass_hdg if self.compass_hdg is not None else yaw))
            self.pose_history.append(time.monotonic(),
                                     (self.roll_deg, self.pitch_deg, yaw, course))

    def _cb_vfr(self, m):
        with self.lock:
            self.airspeed = m.airspeed

    def _cb_home(self, m):
        with self.lock:
            if self.home_lat is None:
                print(f"[IO] home = ({m.geo.latitude:.7f}, {m.geo.longitude:.7f})")
            self.home_lat = m.geo.latitude
            self.home_lon = m.geo.longitude

    def snapshot(self):
        with self.lock:
            d = {k: v for k, v in self.__dict__.items()
                 if k not in ('lock', 'pose_history')}
        if d.get('gps_unix_ref') is not None and d.get('gps_rx_t') is not None:
            age = time.time() - d['gps_rx_t']
            d['gps_unix'] = (d['gps_unix_ref'] + age
                             if age <= self.GPS_TIME_STALE_S else None)
        else:
            d['gps_unix'] = None
        return d


def keyboard_thread(flags):
    while not flags['quit']:
        try:
            line = input().strip().lower()
        except EOFError:
            break
        if line == 'd':
            flags['arm'] = True
        elif line == 'x':
            flags['abort'] = True
        elif line == 'q':
            flags['quit'] = True


def check_airspeed_params(cfg):
    """啟動時檢查飛控空速參數 (= xwing_virtual_flight). 只印警告, 不擋起飛."""
    try:
        rospy.wait_for_service("/mavros/param/get", timeout=3.0)
        get = rospy.ServiceProxy("/mavros/param/get", ParamGet)
    except Exception as e:
        print(f"[xwing] 空速參數檢查跳過 ({e})")
        return

    def _read(*names):
        for n in names:
            try:
                r = get(n)
                if r.success:
                    v = r.value.real if r.value.real != 0 else float(r.value.integer)
                    return n, v
            except Exception:
                pass
        return None, None

    n_min, v_min = _read("AIRSPEED_MIN", "ARSPD_FBW_MIN")
    n_max, v_max = _read("AIRSPEED_MAX", "ARSPD_FBW_MAX")
    n_cru, v_cru = _read("AIRSPEED_CRUISE", "TRIM_ARSPD_CM")
    if n_cru == "TRIM_ARSPD_CM" and v_cru is not None:
        v_cru = v_cru / 100.0
    if v_min is not None:
        ok = v_min <= cfg.SEARCH_SPEED_MPS
        print(f"[xwing] {n_min}={v_min:.0f} m/s"
              + (" OK" if ok else f" ★★★ > SEARCH {cfg.SEARCH_SPEED_MPS:.0f} m/s, "
                 f"CHANGE_SPEED 會被拒!"))
    if v_max is not None:
        ok = v_max >= cfg.PULLOUT_VMAX_MPS
        print(f"[xwing] {n_max}={v_max:.0f} m/s"
              + (" OK" if ok else f" ★★★ < 速度上限 {cfg.PULLOUT_VMAX_MPS:.0f} m/s, "
                 f"俯衝時飛控會抬頭壓速度, 跟俯衝命令打架!"))
    if v_cru is not None:
        ok = abs(v_cru - cfg.V_CRUISE_MPS) < 1.0
        print(f"[xwing] {n_cru}→巡航 {v_cru:.0f} m/s"
              + (" OK" if ok else f" ★ 與程式巡航 {cfg.V_CRUISE_MPS:.0f} m/s 不同, "
                 f"撥 GUIDED 時會加減速"))


def _parse_gps(s, name):
    try:
        a, b = s.split(',')
        return float(a), float(b)
    except Exception:
        print(f"[FATAL] --{name} 格式應為 lat,lon (例 23.0012,120.0056)")
        sys.exit(1)


def main():
    if not ROS_OK:
        print("rospy / mavros 不可用 — 本程式只能在機載電腦 (ROS 環境) 執行.")
        print("沒有 ROS 時可先跑模擬: python3 xwing_vision_sim.py")
        sys.exit(1)

    ap = argparse.ArgumentParser()
    ap.add_argument('--moving', dest='moving', action='store_true', default=None,
                    help='虛擬帶路目標移動 (預設由 MOVING_TARGET 決定)')
    ap.add_argument('--static', dest='moving', action='store_false',
                    help='虛擬帶路目標停在起點')
    ap.add_argument('--start', type=str, default=None,
                    help='虛擬目標起點 "lat,lon" (未給用 MOVING_START_GPS)')
    ap.add_argument('--speed', type=float, default=None,
                    help='虛擬目標速度 m/s (未給用 TARGET_SPEED_MPS)')
    ap.add_argument('--target', type=str, default=None,
                    help='靜止虛擬目標 "lat,lon" (未給用 TARGET_GPS)')
    ap.add_argument('--predive-dist', type=float, default=None,
                    help='推頭距離門檻 m (覆寫 PREDIVE_DIST_M; 只影響 distance 模式)')
    ap.add_argument('--trigger', choices=('angle', 'distance'), default=None,
                    help='推頭依據: angle = 看車俯角 ≥ 觸發角 (預設, PREDIVE_TRIGGER); '
                         'distance = 偵測水平距 ≤ 門檻 (原本)')
    ap.add_argument('--trigger-angle', type=float, default=None, metavar='DEG',
                    help='angle 模式的觸發角 (度, 預設 = 俯衝角 30)')
    ap.add_argument('--tgt-heading', type=float, default=None, metavar='DEG',
                    help='虛擬目標方位 (度), 覆寫 config')
    ap.add_argument('--virtual-control', action='store_true',
                    help='目標來自虛擬軌跡 (= xwing_virtual), 偵測只記錄')
    ap.add_argument('--bench', action='store_true',
                    help='掛機: 忽略 AGL 進入條件與啟動檢查 (仍需 GUIDED)')
    ap.add_argument('--bench-pitch', type=float, default=None, metavar='DEG',
                    help='掛機: DIVE 縱向增量的基準姿態覆寫 (例 -20)')
    ap.add_argument('--bench-dive', action='store_true',
                    help='掛機: 跳過 pitch 容差直接交棒 DIVE')
    ap.add_argument('--ground-test', action='store_true',
                    help='地面測試: SEARCH 保持機頭不朝目標')
    ap.add_argument('--case', type=str, default=None,
                    help='自訂 case 標籤加進檔名')
    ap.add_argument('--no-vision', action='store_true',
                    help='不開雲台 UART (只能配 --virtual-control)')
    ap.add_argument('--vision-port', type=str, default=None,
                    help='視覺 UART 串口 (覆寫 VISION_UART_PORT, 例 /dev/ttyUSB0)')
    ap.add_argument('--no-gimbal-cmd', action='store_true',
                    help='只收雲台資料, 不送模式鍵')
    args = ap.parse_args()

    cfg = XwingVisionConfig()
    fx, fy, cx, cy = cfg.virtual_intrinsics()
    use_vision = not args.virtual_control
    if args.trigger is not None:
        cfg.PREDIVE_TRIGGER = args.trigger
    if args.trigger_angle is not None:
        cfg.PREDIVE_TRIGGER_ANGLE_DEG = float(args.trigger_angle)
    if cfg.PREDIVE_TRIGGER not in ('angle', 'distance'):
        print(f"[FATAL] PREDIVE_TRIGGER 只能是 'angle' / 'distance' "
              f"(現在是 {cfg.PREDIVE_TRIGGER!r})")
        sys.exit(1)
    _trig_angle = predive_trigger_angle_deg(cfg)
    if not (0.0 < _trig_angle < 90.0):
        print(f"[FATAL] 觸發角 {_trig_angle} 不合理 (要在 0~90°)")
        sys.exit(1)
    if cfg.GIMBAL_YAW_MODE not in ('none', 'body', 'earth'):
        print(f"[FATAL] GIMBAL_YAW_MODE 只能是 'none' / 'body' / 'earth' "
              f"(現在是 {cfg.GIMBAL_YAW_MODE!r})")
        sys.exit(1)
    print(f"[xwing] 雲台 yaw 用法: GIMBAL_YAW_MODE={cfg.GIMBAL_YAW_MODE!r}"
          + (" (不用 yaw, 相機方位 = 飛機 yaw)" if cfg.GIMBAL_YAW_MODE == 'none' else ""))
    if use_vision and args.no_vision:
        print("[FATAL] 目標來自偵測就一定要開雲台 UART; 沒接雲台請加 --virtual-control")
        sys.exit(1)

    moving = cfg.MOVING_TARGET if args.moving is None else args.moving
    cfg.MOVING_TARGET = moving

    if moving:
        start_gps = (_parse_gps(args.start, 'start') if args.start
                     else cfg.MOVING_START_GPS)
        speed = args.speed if args.speed is not None else cfg.TARGET_SPEED_MPS
        if args.tgt_heading is not None:
            heading = float(args.tgt_heading) % 360.0
        else:
            heading = target_heading_deg(cfg, start_gps)
        print(f"[xwing] 虛擬目標 ({'帶路' if use_vision else '控制'}): 起點 {start_gps} "
              f"方位 {heading:.1f}° 速度 {speed} m/s "
              f"(起點→航向點 {road_length_m(cfg, start_gps):.0f} m)")
    else:
        start_gps = (_parse_gps(args.target, 'target') if args.target
                     else cfg.TARGET_GPS)
        speed, heading = 0.0, 0.0
        print(f"[xwing] 靜止虛擬目標: {start_gps}")

    if args.predive_dist is not None:
        predive_dist = float(args.predive_dist)
    elif moving:
        predive_dist = cfg.PREDIVE_DIST_M
    else:
        predive_dist = cfg.STATIC_PREDIVE_DIST_M
    print(f"[xwing] 目標來源: {'★ 目標偵測' if use_vision else '虛擬軌跡 (偵測只記錄)'}")
    print(f"[xwing] 巡航 {cfg.CRUISE_AGL_M:.0f} m / {cfg.V_CRUISE_MPS:.0f} m/s, "
          f"俯衝 {cfg.DIVE_ANGLE_DEG:.0f}° → {cfg.PULLOUT_AGL_M:.0f} m 拉平, "
          f"速度上限 {cfg.PULLOUT_VMAX_MPS:.0f} m/s")
    if use_vision and cfg.PREDIVE_TRIGGER == 'angle':
        trigger_desc = f"看車俯角 ≥ {_trig_angle:.1f}° (不用高度)"
    elif use_vision:
        trigger_desc = f"偵測水平距 ≤ {predive_dist:.0f} m"
    else:
        trigger_desc = f"到虛擬目標水平距 ≤ {predive_dist:.0f} m"
    print(f"[xwing] 推頭門檻: {trigger_desc}")

    rospy.init_node('xwing_vision_flight', anonymous=True)
    io = MavrosIO()

    gcfg = build_gcfg(cfg, moving, predive_dist,
                      bench=args.bench, bench_pitch=args.bench_pitch,
                      bench_dive=args.bench_dive, ground_test=args.ground_test)
    sm = MissionStateMachine(gcfg, verbose=True)
    vsrc = VisionSource(cfg, control_camera(cfg, gcfg), predive_dist_m=gcfg.PREDIVE_DIST_M)

    bcfg = apply_bridge_cfg(AttitudeBridgeConfig(), cfg)
    bridge = ArduPilotAttitudeBridge(dive_angle_deg=cfg.DIVE_ANGLE_DEG, cfg=bcfg)

    # ── 雲台視覺 UART (對方 Orin NX → USB-UART) ──
    vision = None
    gimbal_cmd = cfg.GIMBAL_MODE_CMD_ENABLED and not args.no_gimbal_cmd
    if cfg.VISION_UART_ENABLED and not args.no_vision:
        from xwing_uart_vision import UartVisionLink
        vport = args.vision_port or cfg.VISION_UART_PORT
        try:
            vision = UartVisionLink(vport, cfg.VISION_UART_BAUD,
                                    mode_resend_s=cfg.GIMBAL_MODE_RESEND_S,
                                    mode_max_tries=cfg.GIMBAL_MODE_MAX_TRIES,
                                    lock_key=cfg.GIMBAL_LOCK_KEY,
                                    lock_pitch_deg=cfg.GIMBAL_LOCK_PITCH_DEG,
                                    lock_tol_deg=cfg.GIMBAL_LOCK_TOL_DEG)
            vision.start()
            print(f"[xwing] 雲台視覺 UART: {vport} @ {cfg.VISION_UART_BAUD}; "
                  f"模式鍵 {'ON' if gimbal_cmd else 'OFF'}")
        except ImportError as e:
            if use_vision:
                print(f"[FATAL] 雲台視覺 UART 無法啟用: {e} — 沒有目標來源 "
                      f"(pip3 install pyserial, 或加 --virtual-control)")
                sys.exit(1)
            print(f"[xwing] ★ 雲台視覺 UART 無法啟用: {e} — 繼續飛 (虛擬目標控制)")
            vision = None
    if vision is None:
        gimbal_cmd = False

    print("[xwing] 等待 GPS fix ...")
    t_wait = time.time()
    while not rospy.is_shutdown():
        s = io.snapshot()
        if s['lat'] is not None:
            break
        if time.time() - t_wait > 30.0:
            print("[xwing] ★ 30s 沒等到 GPS, 檢查 MAVROS")
            t_wait = time.time()
        rospy.sleep(0.5)

    global NED_ORIGIN
    NED_ORIGIN = (float(start_gps[0]), float(start_gps[1]))
    o_lat, o_lon = NED_ORIGIN
    print(f"[xwing] NED 原點固定在虛擬目標起點 ({o_lat:.7f}, {o_lon:.7f})")
    target = build_target(cfg, moving, NED_ORIGIN, start_gps, heading, speed)
    virt = VirtualSource(cfg, gcfg, o_lat, o_lon, target, predive_point_ne=None)

    # ── log ──
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), cfg.OUT_DIR)
    os.makedirs(log_dir, exist_ok=True)
    if args.case:
        tag = ''.join(c if (c.isalnum() or c in '-_') else '_' for c in args.case)
    else:
        tag = 'vision' if use_vision else 'virtual'
    stem = f'xwing_vision_{stamp}_{tag}'
    csv_path = os.path.join(log_dir, f'{stem}.csv')
    ev_path = os.path.join(log_dir, f'{stem}_events.txt')

    fcsv = open(csv_path, 'w', newline='', buffering=1)
    writer = csv.DictWriter(fcsv, fieldnames=CSV_FIELDS, extrasaction='ignore')
    writer.writeheader()
    fcsv.flush(); os.fsync(fcsv.fileno())
    fev = open(ev_path, 'a', buffering=1)
    t0 = time.time()
    _ev_lock = threading.Lock()

    def event(msg):
        line = f"{time.time() - t0:8.2f}s  {msg}"
        print(line)
        with _ev_lock:
            try:
                fev.write(line + '\n')
                fev.flush(); os.fsync(fev.fileno())
            except (OSError, ValueError):
                pass

    event(f"xwing_vision: 目標來源={'目標偵測' if use_vision else '虛擬軌跡'} "
          f"巡航 {cfg.CRUISE_AGL_M:.0f}m/{cfg.V_CRUISE_MPS:.0f}m/s "
          f"俯衝 {cfg.DIVE_ANGLE_DEG:.0f}° 拉平 {cfg.PULLOUT_AGL_M:.0f}m "
          f"Vmax {cfg.PULLOUT_VMAX_MPS:.0f}m/s 推頭: {trigger_desc}")
    event(f"DIVE 方向舵: 方向舵 PID (版本 A); 空速排程 RUDDER_VSCHED={cfg.RUDDER_VSCHED}")
    event(f"虛擬目標 ({'帶路 + 對照' if use_vision else '控制'}) "
          f"{'移動' if moving else '靜止'} 起點 {start_gps} 方位 {heading:.1f}° "
          f"速度 {speed} m/s")
    if use_vision:
        event(f"目標偵測: 封包 {cfg.VISION_PKT_MAX_AGE_S:.1f}s 沒更新 = 看不到; "
              f"雲台偏差 pitch {cfg.GIMBAL_PITCH_OFFSET_DEG:+.1f}° "
              f"yaw {cfg.GIMBAL_YAW_OFFSET_DEG:+.1f}°")
    event(f"雲台視覺 UART: "
          + (f"{vision.port} @ {vision.baud}; 模式鍵 "
             f"{'ON ' + str(cfg.GIMBAL_MODE_KEYS) if gimbal_cmd else 'OFF'}"
             if vision is not None else "未啟用"))
    if vision is not None and hasattr(vision, 'enable_diag'):
        vision.log = event
        vision.rx_lost_s = cfg.VISION_STALE_S
        vision.enable_diag(os.path.join(log_dir, stem))

    _sync_run = {'on': True}

    def _fsync_loop():
        while _sync_run['on']:
            try:
                fcsv.flush(); os.fsync(fcsv.fileno())
            except (OSError, ValueError):
                pass
            time.sleep(1.0 / cfg.FSYNC_HZ)
    threading.Thread(target=_fsync_loop, daemon=True).start()
    print(f"[xwing] 主 log → {csv_path}")

    flags = {'arm': False, 'abort': False, 'quit': False}
    threading.Thread(target=keyboard_thread, args=(flags,), daemon=True).start()

    bridge.start()
    check_airspeed_params(cfg)
    print("[xwing] ★ AUTO 對準 → WP3 撥 GUIDED = 任務開始; 看到車且到推頭距離自動俯衝")
    print("[xwing] 掛機備用鍵盤: d=arm x=abort q=quit")

    rate = rospy.Rate(LOOP_HZ)
    half_w, half_h = cfg.IMG_W / 2.0, cfg.IMG_H / 2.0
    guided_prev = False
    guided_off_t = None
    run_idx = 0
    mode_prev = sm.mode
    last_print = 0.0
    err_count = 0
    vis_last_rid = None
    gimbal_state_prev = None
    vis_down = False
    gim_logged = (None, None, None)
    vis_check_t = 0.0
    scored = {'done': False}
    _miss_warned = False
    pid_last_t = None

    arm_min_dist = predive_dist + cfg.ARM_MIN_MARGIN_M

    def arm_check(s):
        """撥 GUIDED 時能不能開始任務 (= xwing_virtual). None = 可以; 否則 = 拒絕原因."""
        if args.bench or args.ground_test:
            return None
        agl = s['rel_alt']
        if agl is not None and agl < cfg.ARM_GROUND_AGL_M:
            return None
        if agl is None or agl < cfg.ENTRY_MIN_AGL_M:
            return (f"高度 {agl if agl is not None else float('nan'):.0f} m "
                    f"(< {cfg.ENTRY_MIN_AGL_M:.0f} m), 到不了推頭條件")
        an, ae = gps_to_ned(s['lat'], s['lon'], o_lat, o_lon)
        d0 = math.hypot(an, ae)
        if d0 < arm_min_dist:
            return (f"離目標起點只有 {d0:.0f} m (< {arm_min_dist:.0f} m), "
                    f"一撥就會推頭")
        if d0 > cfg.ARM_MAX_DIST_M:
            return (f"離目標起點 {d0:.0f} m (> {cfg.ARM_MAX_DIST_M:.0f} m), "
                    f"不在 WP3 附近 / --start 打錯")
        if s['compass_hdg'] is None:
            return "沒有羅盤航向, 無法確認目標在機頭前方"
        brg = math.degrees(math.atan2(-ae, -an)) % 360.0
        off = abs((brg - s['compass_hdg'] + 180.0) % 360.0 - 180.0)
        if off > cfg.ARM_MAX_BEARING_OFF_DEG:
            return (f"目標起點不在機頭前方 (方位差 {off:.0f}° > "
                    f"{cfg.ARM_MAX_BEARING_OFF_DEG:.0f}°), 已經飛過或方向不對")
        return None

    def do_arm(s, t):
        an, ae = gps_to_ned(s['lat'], s['lon'], o_lat, o_lon)
        d0 = math.hypot(an, ae)
        event(f"切 GUIDED 時離目標起點 {d0:.0f} m")
        if moving:
            virt.target.t0 = t
            tn, te, _, _ = virt.target_now(t)
            event(f"種點(移動): 虛擬目標起點 NED=({tn:.1f},{te:.1f}) "
                  f"方位 {virt.target.heading_deg:.1f}° 速度 {virt.target.speed} m/s")
        else:
            tn, te, _, _ = virt.target_now(t)
            event(f"種點(靜止): 虛擬目標 NED=({tn:.1f},{te:.1f})")
        if use_vision:
            _lv = vsrc.last_valid
            event("目標偵測: " + (f"撥 GUIDED 時已偵測到目標 (水平距 {_lv['ground_m']:.0f} m)"
                                  if _lv is not None else "還沒偵測到目標 — 先朝虛擬目標帶路"))
        _cruise_alt = s.get('rel_alt')
        if _cruise_alt is None or _cruise_alt < 5.0:
            _cruise_alt = cfg.CRUISE_AGL_M
        _search_spd = cfg.SEARCH_SPEED_MPS

        def _arm_tecs_done(ok_a, ok_s, _alt=_cruise_alt, _spd=_search_spd):
            event(f"TECS 目標預送完成 ALT={_alt:.0f}m "
                  f"SPD={_spd:.0f}m/s ok=({ok_a},{ok_s})")
            if not ok_s:
                event(f"★ SEARCH 空速目標 {_spd:.0f}m/s 被飛控拒絕 — "
                      f"檢查 AIRSPEED_MIN ≤ {_spd:.0f} ≤ AIRSPEED_MAX")

        bridge.send_tecs_targets_async(alt_m=_cruise_alt, speed_mps=_search_spd,
                                       tag='/巡航', on_done=_arm_tecs_done)
        event(f"TECS 目標預送 (背景) ALT={_cruise_alt:.0f}m SPD={_search_spd:.0f}m/s")
        sm.arm()

    try:
        while not rospy.is_shutdown() and not flags['quit']:
            try:
                _tick_mono = time.monotonic()
                t = time.time() - t0
                s = io.snapshot()

                if s['guided'] and not guided_prev:
                    if sm.mode == sm.SEARCH and not sm.mission_armed:
                        _why = arm_check(s) if s['lat'] is not None else "沒有 GPS"
                        if _why is None:
                            run_idx += 1
                            event(f"★ GUIDED 上升沿 — 種點 + ARM (第 {run_idx} 趟任務開始)")
                            do_arm(s, t)
                        else:
                            event(f"★★ GUIDED 上升沿 — 不啟動任務: {_why}. "
                                  f"程式只保持當下航向直飛; 切回 AUTO, 下一圈到 WP3 再撥")
                    else:
                        event(f"GUIDED 上升沿 — 離開不到 {cfg.REARM_OFF_S:.1f}s, "
                              f"沿用第 {run_idx} 趟 (mode={sm.mode})")
                elif guided_prev and not s['guided']:
                    if sm.mission_armed or sm.mode != sm.SEARCH:
                        event(f"★ 離開 GUIDED (fc_mode={s['fc_mode']}) — "
                              f"第 {run_idx} 趟 mode={sm.mode} "
                              f"({cfg.REARM_OFF_S:.0f}s 內沒回來就重置)")
                guided_prev = s['guided']
                if s['guided']:
                    guided_off_t = None
                elif guided_off_t is None:
                    guided_off_t = t

                if (guided_off_t is not None
                        and t - guided_off_t >= cfg.REARM_OFF_S
                        and (sm.mission_armed or sm.mode != sm.SEARCH)):
                    sm = MissionStateMachine(gcfg, verbose=True)
                    vsrc.reset()
                    mode_prev = sm.mode
                    scored['done'] = False
                    _miss_warned = False
                    event(f"任務重置 — 下次撥 GUIDED = 第 {run_idx + 1} 趟 "
                          f"(重新偵測目標; 虛擬目標回到起點)")

                if flags['arm']:
                    flags['arm'] = False
                    if sm.mission_armed:
                        print("[xwing] 任務已 ARM, 忽略 'd'")
                    elif s['guided'] and s['lat'] is not None:
                        _why = arm_check(s)
                        if _why is None:
                            run_idx += 1
                            event(f"★ 鍵盤 'd' — 種點 + ARM (第 {run_idx} 趟)")
                            do_arm(s, t)
                        else:
                            event(f"★★ 鍵盤 'd' — 不啟動任務: {_why}")
                if flags['abort']:
                    flags['abort'] = False
                    sm.operator_abort(t)

                if s['lat'] is None:
                    rate.sleep()
                    continue

                ac_n, ac_e = gps_to_ned(s['lat'], s['lon'], o_lat, o_lon)
                agl = s['rel_alt']
                hdg = s['compass_hdg'] if s['compass_hdg'] is not None else 0.0
                pitch = s['pitch_deg'] if s['pitch_deg'] is not None else 0.0
                V = s['airspeed'] if s['airspeed'] is not None else 0.0

                dive_mode = (sm.mode == sm.DIVE)
                vfields = virt.get_fields({
                    'n': ac_n, 'e': ac_e, 'agl': agl,
                    'theta_deg': pitch, 'chi_deg': hdg,
                    'gimbal_locked': dive_mode,
                    't': t,
                })

                # ── 目標偵測: 最新封包 → 與虛擬目標同名的欄位 ──
                _mono = time.monotonic()
                _vpkt = vision.latest() if vision is not None else None
                _vg_fields, _vg_row = vsrc.get_fields(_vpkt, _mono, io.pose_history,
                                                      agl, ac_n, ac_e, origin=NED_ORIGIN)

                inp = {
                    't': t, 'guided': s['guided'],
                    'agl_m': agl, 'airspeed_mps': s['airspeed'],
                    'gs_mps': (math.hypot(s['vn'], s['ve'])
                               if s['vn'] is not None else None),
                    'vn': s['vn'], 've': s['ve'], 'vd': s['vd'],
                    'yaw_true_deg': hdg, 'pitch_deg': pitch, 'roll_deg': s['roll_deg'],
                    'ac_n': ac_n, 'ac_e': ac_e,
                    **vfields,
                }
                if use_vision:
                    # ★ 目標輸入換成偵測. 從沒偵測到過 → 位置先用虛擬目標帶路,
                    #   但距離 / 畫面 / 有效旗標一律來自偵測 (看不到 = 不會推頭)
                    _g = dict(_vg_fields)
                    if _g['tgt_n'] is None:
                        _g['tgt_n'], _g['tgt_e'] = vfields['tgt_n'], vfields['tgt_e']
                    inp.update(_g)
                out = sm.update(inp)

                if out['mode'] != mode_prev:
                    _vd = _vg_fields.get('ground_m')
                    _vdep = _vg_row.get('vg_depress_deg')
                    banner = (f"★★ MODE: {mode_prev} → {out['mode']} "
                              f"(t={t:.1f}s AGL="
                              f"{f'{agl:.1f}' if agl is not None else '?'} "
                              f"V={V:.1f} 看車俯角="
                              f"{f'{_vdep:.1f}°' if _vdep not in (None, '') else '—'} "
                              f"偵測水平距="
                              f"{f'{_vd:.0f}m' if _vd is not None else '—'} "
                              f"虛擬水平距={vfields['ground_m']:.0f}m "
                              f"pitch_cmd={out['pitch_cmd_deg']:+.1f} "
                              f"hdg_cmd={out['heading_cmd_deg']:.1f}"
                              + (f" 原因={out.get('abort_reason')}"
                                 if out.get('abort_reason') else "") + ")")
                    print("\n" + "=" * 64 + "\n" + banner + "\n" + "=" * 64)
                    event(banner)
                    mode_prev = out['mode']
                    if getattr(sm, 'tecs_resend', False):
                        sm.tecs_resend = False
                        _tag = f"/{out['mode']}"

                        def _hold_done(ok_a, ok_s, _tag=_tag):
                            event(f"TECS 高度目標 → {cfg.TECS_HOLD_ALT_M:.0f}m "
                                  f"完成{_tag} ok=({ok_a},{ok_s})")

                        bridge.send_tecs_targets_async(
                            alt_m=cfg.TECS_HOLD_ALT_M, tag=_tag, on_done=_hold_done)
                        event(f"TECS 高度目標 → {cfg.TECS_HOLD_ALT_M:.0f}m "
                              f"(背景發送{_tag})")

                bridge.submit(out, hdg, s['yaw_rate_dps'], s['airspeed'],
                              roll_deg=s['roll_deg'], guided=s['guided'],
                              hdg_valid=s['compass_hdg'] is not None)
                _ctrl_ms = (time.monotonic() - _tick_mono) * 1000.0

                # ── 雲台: 任務狀態 → 模式鍵; 最新封包 → log ──
                _gstate = gimbal_state_label(s['guided'], sm.mission_armed,
                                             out['mode'])
                if vision is not None and _gstate != gimbal_state_prev:
                    gimbal_state_prev = _gstate
                    if gimbal_cmd:
                        _key = cfg.GIMBAL_MODE_KEYS.get(_gstate)
                        vision.request_mode(_key)
                        event(f"雲台狀態 → {_gstate}: 模式鍵 "
                              + (repr(_key) if _key is not None else "不要求"))
                _vrow, vis_last_rid = vision_row(
                    vision, _vpkt, vis_last_rid, _mono, fx, fy, cx, cy,
                    _gstate, cfg.VISION_STALE_S)
                if vision is not None and gimbal_cmd:
                    _tm = vision.mode_timing()
                    if _tm['t_req'] is not None:
                        if _tm['t_req'] != gim_logged[0]:
                            gim_logged = (_tm['t_req'], None, None)
                        if _tm['t_confirm'] is not None and gim_logged[1] is None:
                            gim_logged = (gim_logged[0], _tm['t_confirm'], gim_logged[2])
                            event(f"雲台回報模式 '{_tm['key']}' 已確認 "
                                  f"(要求後 {_tm['t_confirm'] - _tm['t_req']:.2f}s)")
                        if _tm['t_settled'] is not None and gim_logged[2] is None:
                            gim_logged = (gim_logged[0], gim_logged[1], _tm['t_settled'])
                            event(f"★ 雲台鎖定到位 pitch={_tm['pitch_settled']:.1f}° "
                                  f"(要求後 {_tm['t_settled'] - _tm['t_req']:.2f}s, "
                                  f"任務狀態 {_gstate})")
                if vision is not None and t - vis_check_t >= 1.0:
                    vis_check_t = t
                    _age = (_mono - _vpkt['rx_mono']) if _vpkt else None
                    _down = _age is None or _age > cfg.VISION_STALE_S
                    if _down and not vis_down:
                        event(f"★★ 雲台視覺資料鏈中斷: "
                              + (f"{_age:.1f}s 沒收到有效封包" if _age is not None
                                 else "尚未收到任何有效封包")
                              + (f" (串口 {vision.port} 已開啟, 收 "
                                 f"{vision.stats['bytes']} bytes" if vision.connected
                                 else f" (★ 串口 {vision.port} 開不了 — 檢查 USB-UART"
                                 f" 有沒有插 / 串口名稱")
                              + f", CRC 錯 {vision.stats['crc']}, 串口錯 "
                              f"{vision.stats['serial_err']})"
                              + (" ← 目標偵測中斷 = 看不到目標" if use_vision
                                 else " ← 控制不受影響"))
                    elif not _down and vis_down:
                        event("雲台視覺資料鏈恢復")
                    vis_down = _down

                # ── 主 log ──
                dt_loop = (t - pid_last_t) if pid_last_t is not None else 0.0
                pid_last_t = t
                gam = (math.degrees(math.asin(max(-1, min(1,
                       -s['vd'] / max(V, 1.0))))) if s['vd'] is not None else 0.0)
                vu = vfields['bbox_u']
                vv = vfields['bbox_v']
                _brg = bridge.get_debug()

                # 逐拍外插 miss: 對狀態機用的目標 (偵測 = 目標位置估計, 靜止假設)
                _mp = _mlon = _mlat = _tf = ''
                _ref = (inp['tgt_n'], inp['tgt_e'],
                        inp.get('tgt_vn') or 0.0, inp.get('tgt_ve') or 0.0)
                try:
                    if (agl is not None and agl > 0.5
                            and (s['vd'] or 0.0) > 0.1
                            and inp['tgt_n'] is not None):
                        _in, _ie, _tfall = extrapolate_impact(
                            ac_n, ac_e, agl,
                            s['vn'] or 0.0, s['ve'] or 0.0, s['vd'] or 0.0)
                        if _in is not None:
                            _md = miss_distance(_in, _ie, _tfall, *_ref,
                                                s['vn'] or 0.0, s['ve'] or 0.0)
                            _mp = round(_md['miss_total'], 2)
                            _mlon = round(_md['miss_lon'], 2)
                            _mlat = round(_md['miss_lat'], 2)
                            _tf = round(_tfall, 2)
                except Exception as _e:
                    if not _miss_warned:
                        _miss_warned = True
                        event(f"★ 逐幀 miss 計算失敗: {type(_e).__name__}: {_e}")

                _tlat = o_lat + math.degrees(vfields['tgt_n'] / 6378137.0)
                _tlon = o_lon + math.degrees(
                    vfields['tgt_e'] / (6378137.0 * math.cos(math.radians(o_lat))))

                def _r(x, nd=2):
                    return round(x, nd) if x is not None else ''

                row = {
                    't': round(t, 3),
                    'gps_unix': (f"{s['gps_unix']:.3f}" if s['gps_unix'] is not None else ''),
                    'run': run_idx,
                    'mode': out['mode'], 'guided': int(s['guided']),
                    'control': 'vision' if use_vision else 'virtual',
                    'ac_n': round(ac_n, 2), 'ac_e': round(ac_e, 2),
                    'agl': _r(agl), 'V': round(V, 2),
                    'chi_deg': round(hdg, 2), 'gamma_deg': round(gam, 2),
                    'theta_deg': round(pitch, 2), 'roll_deg': _r(s['roll_deg']),
                    'yaw_deg': round(hdg, 1),
                    'vn': _r(s['vn']), 've': _r(s['ve']), 'vd': _r(s['vd']),
                    'dt': round(dt_loop, 4),
                    'tgt_n': round(vfields['tgt_n'], 2),
                    'tgt_e': round(vfields['tgt_e'], 2),
                    'slant_m': _r(vfields['slant_m'], 1),
                    'ground_m': _r(vfields['ground_m'], 1),
                    'predive_dist_m': round(vfields['predive_dist_m'], 1),
                    'virt_bbox_u': _r(vu, 1), 'virt_bbox_v': _r(vv, 1),
                    'virt_detected': int(vfields['bbox_detected']),
                    'virt_u_sp': out.get('pid_u_sp', ''),
                    'virt_v_sp': out.get('pid_v_sp', ''),
                    'pitch_cmd': out['pitch_cmd_deg'],
                    'heading_cmd': out['heading_cmd_deg'],
                    'throttle': _brg.get('brg_thr', ''),
                    **{k: _brg.get(k, '') for k in (
                        'brg_mask', 'rudder_raw_deg', 'rudder_cmd_deg',
                        'rudder_sent_deg', 'rudder_limit_deg',
                        'lim_by_airspeed', 'lim_by_roll',
                        'hdg_cmd_ok_cnt', 'lateral_fallback',
                        'brg_stale', 'brg_stale_events',
                        'brg_entry_roll0', 'brg_hdg_want')},
                    **{k: out.get(k, '') for k in (
                        'pid_u_err_deg', 'pid_v_err_deg',
                        'pid_lat_p', 'pid_lat_i', 'pid_lat_d',
                        'pid_lon_p', 'pid_lon_i', 'pid_lon_d',
                        'pid_dpsi_deg', 'pid_dtheta_deg', 'pid_compute_ms')},
                    'miss_pred_m': _mp, 'miss_pred_lon_m': _mlon,
                    'miss_pred_lat_m': _mlat, 'impact_t_fall_s': _tf,
                    'img_err_u_pct': (round((vu - cx) / half_w * 100, 2)
                                      if vu is not None else ''),
                    'img_err_v_pct': (round((vv - cy) / half_h * 100, 2)
                                      if vv is not None else ''),
                    'glide_slope_err_deg': (round(gam + cfg.DIVE_ANGLE_DEG, 2)
                                            if out['mode'] == 'DIVE' else ''),
                    'p_dps': _r(s['p_dps'], 1), 'q_dps': _r(s['q_dps'], 1),
                    'r_dps': _r(s['r_dps'], 1), 'nz_g': _r(s['nz_g']),
                    'fc_pitch_tgt_deg': _r(s['fc_pitch_tgt_deg']),
                    'fc_nav_pitch_deg': _r(s['fc_nav_pitch_deg']),
                    'fc_nav_roll_deg': _r(s['fc_nav_roll_deg']),
                    'fc_nav_bearing_deg': _r(s['fc_nav_bearing_deg'], 1),
                    'fc_mode': s['fc_mode'], 'batt_v': _r(s['batt_v']),
                    **{f'rcout{i+1}': (s['rcout'][i] if s['rcout'][i] is not None else '')
                       for i in range(4)},
                    'lat': s['lat'], 'lon': s['lon'],
                    'tgt_lat': round(_tlat, 7), 'tgt_lon': round(_tlon, 7),
                    'abort_reason': out.get('abort_reason', ''),
                    'ctrl_ms': round(_ctrl_ms, 2),
                }
                row.update(_vg_row)
                row.update(_vrow)
                writer.writerow(row)

                if t - last_print >= 1.0:
                    last_print = t
                    _np = s['fc_nav_pitch_deg']
                    _nr = s['fc_nav_roll_deg']
                    _fc = (f" | FC nav_pitch={_np:+6.1f} nav_roll={_nr:+6.1f}"
                           if _np is not None else " | FC nav_* 無資料")
                    _vs = ''
                    if vision is not None:
                        _vd = _vg_fields.get('ground_m')
                        _vdep = _vg_row.get('vg_depress_deg')
                        _vs = (f" | VIS link={_vrow['vis_link']} "
                               f"det={int(_vg_fields['tgt_valid'])} "
                               f"dep={f'{_vdep:.1f}°' if _vdep not in (None, '') else '-'} "
                               f"dist={f'{_vd:.0f}m' if _vd is not None else '-'} "
                               f"gmode={_vrow['vis_gimbal_mode'] if _vrow['vis_gimbal_mode'] != '' else '-'}"
                               f"(req {_vrow['gimbal_mode_req'] or '-'}) "
                               f"gpitch={_vrow['vis_gimbal_pitch'] if _vrow['vis_gimbal_pitch'] != '' else '-'}")
                    print(f"[{out['mode']:8s}] t={t:6.1f} AGL={agl or 0:6.1f} "
                          f"V={V:5.1f} 虛擬={vfields['ground_m']:6.0f}m "
                          f"pitch={out['pitch_cmd_deg']:+6.1f} "
                          f"hdg={out['heading_cmd_deg']:6.1f} g={int(s['guided'])}"
                          + _fc + _vs)

                if sm.pullout_snapshot is not None and not scored['done']:
                    scored['done'] = True
                    sp = sm.pullout_snapshot
                    i_n, i_e, t_f = extrapolate_impact(
                        sp['ac_n'], sp['ac_e'], sp['agl'],
                        sp['vn'] or 0, sp['ve'] or 0, sp['vd'] or 0.1)
                    if i_n is not None:
                        md = miss_distance(i_n, i_e, t_f, sp['tgt_n'], sp['tgt_e'],
                                           sp['tgt_vn'], sp['tgt_ve'],
                                           sp['vn'] or 0, sp['ve'] or 0)
                        event(f"★★★ 成績: miss = {md['miss_total']:.2f} m "
                              f"(lon {md['miss_lon']:+.2f}, lat {md['miss_lat']:+.2f}) — 對"
                              + ("偵測到的目標位置 (靜止假設); 對真車請用車子 GPS 事後算"
                                 if use_vision else "虛擬目標"))

            except Exception as e:
                err_count += 1
                if err_count <= 20:
                    event(f"★ 主迴圈例外 #{err_count}: {type(e).__name__}: {e}")
            rate.sleep()
    finally:
        try:
            bridge.stop()
        except Exception:
            pass
        if vision is not None:
            try:
                vision.stop()
                event(f"雲台視覺 UART 統計: {vision.stats}")
            except Exception:
                pass
        event(f"程式結束 mode={sm.mode} errors={err_count} "
              f"時長 {time.time() - t0:.1f}s")
        _sync_run['on'] = False
        flags['quit'] = True
        fcsv.flush(); os.fsync(fcsv.fileno()); fcsv.close()
        fev.close()
        print(f"[xwing] log 已存 {csv_path}")


if __name__ == '__main__':
    main()
