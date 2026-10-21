#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_vision_flight.py — vision_dive 實飛主程式 (真實相機 DIVE 控制版)
═══════════════════════════════════════════════════════════════════
這一版: SEARCH/PRE_DIVE 使用虛擬目標幾何；predive 距離可切換 ray projection。
DIVE 使用真實 YOLO 的相機補償像素控制。漏偵測時保持命令並凍結 PID 積分。
Ray projection 全階段測距記錄，原始與平滑後水平距可與虛擬目標比較。

三條執行緒:
  1. 視覺線程 (~30Hz): FrameGrabber → YOLO → 雲台追蹤 (真實 bbox)
                       → 寫標註影片 + raw 影片 + 視覺 log (每幀釘飛機狀態)
  2. 控制線程 (50Hz):  MAVROS + 真實相機補償影像 → MissionStateMachine (影像 PID)
                       → AttitudeBridge → SET_ATTITUDE_TARGET + 寫主 log
  3. 主線程:           連線 / GUIDED 上升沿種點 / 收尾

目標 GPS 在命令列或 config 給定 (見 --target / --start).
撥 GUIDED = 任務開始; 飛機到目標水平距 ≤ 門檻 → 進 PRE_DIVE; 其餘同前.
  門檻: 靜止版 config.STATIC_PREDIVE_DIST_M, 移動版 config.PREDIVE_DIST_M.
  ★ 2026/09/30: 靜止版不再用固定 PRE_DIVE 點, 改成跟移動版一樣看距離.

用法:
  python3 run_vision_flight.py --static --target 23.001234,120.005678
  python3 run_vision_flight.py --no-video      # 不錄影
  python3 run_vision_flight.py --bench         # 掛機: 忽略 AGL 進入條件
"""

import argparse
import csv
import math
import os
import sys
import threading
import traceback
import time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import Config
from dive_guidance import GuidanceConfig, MissionStateMachine
from gimbal_pid_compensation import REAL_PID_FIELDS
from attitude_time_history import AngleHistory
from gimbal_pid_compensation import (GimbalCompPIDLogger, capture_geometry,
                                     GEOMETRY_FIELDS, COMP_PID_FIELDS,
                                     RAY_LOG_FIELDS, control_inputs)
from attitude_bridge import ArduPilotAttitudeBridge, AttitudeBridgeConfig
from vision import VisionFrontend, FrameGrabber
from video_writer import VideoWriter
from virtual_source import VirtualSource
from virtual_source import gps_to_ned, extrapolate_impact, miss_distance

try:
    import rospy
    from mavros_msgs.msg import (State, RCOut, AttitudeTarget,
                             NavControllerOutput)
    from sensor_msgs.msg import NavSatFix, Imu, BatteryState, TimeReference
    from std_msgs.msg import Float64
    from geometry_msgs.msg import TwistStamped
    from mavros_msgs.msg import VFR_HUD, HomePosition
    ROS_OK = True
except ImportError:
    ROS_OK = False


LOOP_HZ = 50.0

# ★ 2026/09/15: NED 原點 = 固定 GPS 點, 不再用飛控 home.
#   靜止版 = TARGET_GPS, 移動版 = MOVING_START_GPS (車起點).
#   原因: 飛控 home 在地面待機 / 解鎖時會移動. 舊版目標用「程式啟動時的
#   home」換算, 飛機用「當下 home」換算, 兩邊基準不同 → 09/14 四趟偏了
#   6~15 m (兩趟因此卡在 SEARCH, 另兩趟俯衝瞄準點與 miss 成績也偏了).
#   原點固定後, 飛機與目標用同一個點換算, 相減時基準抵消,
#   結果等同直接拿兩個 GPS 相減. main() 啟動時設定一次, 之後不變.
NED_ORIGIN = None

# GUIDED 任務 CSV 管理：保留在本檔，不增加執行用模組。
class MissionCSV:
    def __init__(self, directory, tag, main_fields, vision_fields, recording_stem=""):
        self.directory = directory
        self.tag = tag
        self.recording_stem = recording_stem
        self.fields = (list(main_fields) + ['mission_id', 'mission_t', 'recording_stem'],
                       list(vision_fields) + ['mission_id', 'mission_t', 'cap_mission_t', 'recording_stem'])
        self.lock = threading.RLock()
        self.generation = 0
        self.mission_id = 0
        self.active = False
        self.start_t = None
        self.start_mono = None
        self.files = []
        self.writers = []
        self.paths = []
        self.saved_pairs = []

    def begin(self, t, mono):
        with self.lock:
            if self.active:
                return tuple(self.paths)
            number = self.mission_id + 1
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            stem = f'flight_{stamp}_{self.tag}_mission{number:03d}'
            paths = [os.path.join(self.directory, stem + suffix)
                     for suffix in ('.csv', '_vision.csv')]
            opened = []
            try:
                writers = []
                for path, fields in zip(paths, self.fields):
                    f = open(path, 'x', newline='', buffering=1)
                    opened.append(f)
                    writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
                    writer.writeheader()
                    f.flush()
                    os.fsync(f.fileno())
                    writers.append(writer)
            except Exception:
                for f in opened:
                    f.close()
                    os.unlink(f.name)
                raise
            self.generation += 1
            self.mission_id = number
            self.start_t, self.start_mono = t, mono
            self.files, self.writers, self.paths = opened, writers, paths
            self.saved_pairs.append(tuple(paths))
            self.active = True
            return tuple(paths)

    def end(self):
        with self.lock:
            if not self.active:
                return
            self.active = False
            self.generation += 1
            files, self.files = self.files, []
            self.writers = []
            errors = []
            for f in files:
                try:
                    f.flush()
                    os.fsync(f.fileno())
                except OSError as exc:
                    errors.append(exc)
                finally:
                    f.close()
            if errors:
                raise errors[0]

    def snapshot(self):
        with self.lock:
            return self.generation, self.active, self.start_mono

    def write(self, kind, row, generation=None, cap_mono=None):
        with self.lock:
            if not self.active or (generation is not None and generation != self.generation):
                return False
            if cap_mono is not None and cap_mono < self.start_mono:
                return False
            record = dict(row)
            record['recording_stem'] = self.recording_stem
            record['mission_id'] = self.mission_id
            record['mission_t'] = round(float(row['t']) - self.start_t, 3)
            if kind == 1:
                cap_t = row.get('cap_t')
                record['cap_mission_t'] = (round(float(cap_t) - self.start_t, 3)
                                           if cap_t not in (None, '') else '')
            self.writers[kind].writerow(record)
            return True

    def sync(self):
        with self.lock:
            for f in self.files:
                f.flush()
                os.fsync(f.fileno())


# ── 主 log 欄位 (50Hz 控制迴路, 逐拍): 論文畫圖 + 後處理對齊 ──
# virt_* = 虛擬目標算的 (實際餵 PID); real_* 在視覺 log, 不在這.
CSV_FIELDS = [
    't', 'gps_unix', 'frame_idx', 'mode', 'guided',
    # 飛機狀態 (NED)
    'ac_n', 'ac_e', 'agl', 'V', 'chi_deg', 'gamma_deg', 'theta_deg',
    'roll_deg', 'yaw_deg', 'vn', 've', 'vd', 'dt',
    # 虛擬目標 (控制路徑真相來源)
    'tgt_n', 'tgt_e', 'slant_m', 'ground_m', 'predive_dist_m',
    # ★ 2026/09/15: 主 log 的 virt_bbox_u/v、virt_u_sp/v_sp、img_err_*_pct
    #   都是【虛擬 (控制) 像素】= config.VIRTUAL_ZOOM_LEVEL 的座標系,
    #   不是相機畫面像素. 相機像素版本在視覺 log.
    'virt_bbox_u', 'virt_bbox_v', 'virt_detected',
    # PID setpoint + 命令 (比較基準)
    'virt_u_sp', 'virt_v_sp', 'pitch_cmd', 'heading_cmd', 'throttle',
    # 影像 PID 各項 (DIVE 段)
    'pid_u_err_deg', 'pid_v_err_deg',
    'pid_lat_p', 'pid_lat_i', 'pid_lat_d',
    'pid_lon_p', 'pid_lon_i', 'pid_lon_d',
    'pid_dpsi_deg', 'pid_dtheta_deg', 'pid_compute_ms',
    # 橫向 v2 (方向舵迴路) 取證
    'brg_mask', 'rudder_raw_deg', 'rudder_cmd_deg', 'rudder_sent_deg',
    'rudder_limit_deg', 'lim_by_airspeed', 'lim_by_roll',
    'hdg_cmd_ok_cnt', 'lateral_fallback', 'brg_stale', 'brg_stale_events',
    'brg_entry_roll0', 'brg_hdg_want',     # ★ 2026/09/30 撥 GUIDED 進入段
    # 影像誤差百分比 (論文圖)
    'img_err_u_pct', 'img_err_v_pct', 'glide_slope_err_deg',
    # ★ 2026/09/11: 逐幀外插彈著點 → miss distance.
    #   原本只在 PULLOUT 那一刻算一次並印進 events.txt, 畫不出收斂曲線,
    #   四個 case 也難比較. 現在每一拍都算, 直接畫 miss vs 剩餘高度.
    #   只有在下降 (vd > 0.1) 時才有值, 平飛/爬升時為空.
    'miss_pred_m', 'miss_pred_lon_m', 'miss_pred_lat_m', 'impact_t_fall_s',
    # 飛控取證
    'p_dps', 'q_dps', 'r_dps', 'nz_g',
    'fc_pitch_tgt_deg', 'fc_nav_pitch_deg', 'fc_nav_roll_deg',
    'fc_nav_bearing_deg', 'fc_mode', 'batt_v',
    'rcout1', 'rcout2', 'rcout3', 'rcout4',
    'lat', 'lon',
    # ★ 2026/09/28: 影像 → 控制器 延遲 (ms). 目前控制吃虛擬目標,
    #   這些數字代表「若改用真實影像, 控制器拿到的資料有多舊」.
    #   vis_age_ms    : 送 bridge 當下, 最新視覺結果是多久前拍的
    #   vis_new       : 這拍拿到新的一幀 =1, 重用舊幀 =0
    #   ctrl_ms       : 控制迴圈這一拍 (讀狀態 → bridge.submit) 花多久
    #   img_to_cmd_ms : bridge 最近一次送出命令時, 該命令的影像已多舊
    #   (vis_age_ms / img_to_cmd_ms 已含 config.CAPTURE_LATENCY_MS)
    'vis_age_ms', 'vis_new', 'ctrl_ms', 'img_to_cmd_ms',
]

# ── 視覺 log 欄位 (每處理幀一列, 跟 raw 影片 1:1; 後處理重放 PID 用) ──
CSV_FIELDS += REAL_PID_FIELDS + COMP_PID_FIELDS + GEOMETRY_FIELDS + RAY_LOG_FIELDS
CSV_FIELDS += ['predive_distance_source', 'predive_used_dist_m', 'dive_command_source']

VISION_FIELDS = [
    # ★ raw_frame_idx = FrameGrabber 的相機幀序 = raw 影片的幀號.
    #   raw 錄影已與 YOLO 解耦, 兩者不再 1:1, 後處理請用這個對齊.
    'frame_idx', 'raw_frame_idx', 't', 'gps_unix', 'mode', 'dive_mode',
    # 真實 YOLO / DKF (機上即時; 交叉驗證 raw 影片離線重跑)
    'real_detected', 'real_bbox_u', 'real_bbox_v', 'real_bbox_w', 'real_bbox_h',
    'real_conf', 'dkf_u', 'dkf_v', 'dkf_status',
    # 飛機狀態快照 (釘在這一幀; 後處理重放 PID 直接取)
    'ac_n', 'ac_e', 'agl', 'theta_deg', 'chi_deg', 'vn', 've', 'vd', 'frame_dt',
    # 虛擬目標這幀的 setpoint (後處理用同一 setpoint 比較)
    # ★ 2026/09/15: virt_u_sp/v_sp = 已換算成【相機像素】(ZOOM_LEVEL),
    #   後處理直接跟 raw 影片的 YOLO 框比, 不用改.
    #   原始控制座標 (VIRTUAL_ZOOM_LEVEL 像素) 另存 *_ctrl.
    'virt_u_sp', 'virt_v_sp',
    # 雲台
    'gimbal_yaw', 'gimbal_pitch', 'gimbal_mode',
    # 延遲補償取證 (VisionState 早就有, 之前漏記)
    'inf_ms', 'delay_ms',
    # ★ 2026/09/15: 任務模式 = 標註影片左上角顯示的字
    #   NOT_IN_MISSION (沒切/脫離 GUIDED) / SEARCH / PRE_DIVE / DIVE /
    #   PULLOUT / DONE / ABORT
    'mission_mode',
    # ★ 2026/09/15: setpoint 的控制座標原值 (虛擬像素, 見上)
    'virt_u_sp_ctrl', 'virt_v_sp_ctrl',
    # ★ 2026/09/28: 影像延遲
    #   cap_t           : 相機擷取這一幀的時刻 (跟 t 同一時間軸, 秒)
    #   cap_to_ready_ms : 擷取 → YOLO + DKF 算完 (不含 CAPTURE_LATENCY_MS)
    'cap_t', 'cap_to_ready_ms',
]
VISION_FIELDS += GEOMETRY_FIELDS + RAY_LOG_FIELDS


#  MAVROS 訂閱 (照搬 run_flight.MavrosIO 慣例)
# ══════════════════════════════════════════════════════════════════
class MavrosIO:
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
        self.roll_enu = self.pitch_enu = self.yaw_enu = None
        self.home_lat = self.home_lon = None
        self.last_pos_t = 0.0
        self.p_dps = self.q_dps = self.r_dps = None
        self.nz_g = None
        self.fc_mode = ''
        self.fc_pitch_tgt_deg = None
        # ★ 2026/09/11: ArduPlane 不發 ATTITUDE_TARGET (那是 Copter 的),
        #   所以 fc_pitch_tgt_deg 永遠是空的. 飛控真正在用的姿態目標要看
        #   NAV_CONTROLLER_OUTPUT 的 nav_pitch / nav_roll — 那就是 ArduPlane
        #   內部的 nav_pitch_cd / nav_roll_cd, 可以直接回答
        #   「我送的姿態命令飛控到底吃了沒有」.
        self.fc_nav_pitch_deg = None
        self.fc_nav_roll_deg = None
        self.fc_nav_bearing_deg = None
        self.batt_v = None
        self.rcout = [None] * 8
        self.gps_unix = None      # GPS 絕對時間 (unix epoch 秒; 跟車 BIN 對齊)
        # ★ 2026/09/15: time_reference 來自 SYSTEM_TIME, ArduPilot 預設約 1 Hz.
        #   舊版直接把收到的值當「現在」→ 50Hz log 裡是 1 秒一階的階梯,
        #   對齊車軌跡最壞差將近 1 s (車速 10 m/s ≈ 10 m).
        #   現在記下「收到時的 GPS 時間 + 本機收到時刻」, snapshot() 時外插.
        self.gps_unix_ref = None  # 最後一筆 time_ref (unix 秒)
        self.gps_rx_t = None      # 收到它的本機時刻 (time.time())

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
        rospy.Subscriber("/mavros/setpoint_raw/target_attitude", AttitudeTarget, self._cb_fc_target)
        rospy.Subscriber("/mavros/nav_controller_output", NavControllerOutput,
                         self._cb_nav_out)
        # GPS 絕對時間 (跟車 BIN 對齊用): MAVROS 把 GPS 時間發在 time_reference
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
        # time_reference.time_ref = GPS 絕對時間 (ROS Time). 轉 unix 秒.
        # ★ 只記參考點, 真正的 gps_unix 在 snapshot() 依經過時間外插.
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
            # ENU 四元數 → euler (給射線投影 ego 用) + pitch/roll NED
            sinr = 2.0 * (q.w * q.x + q.y * q.z)
            cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
            self.roll_enu = math.atan2(sinr, cosr)
            sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            self.pitch_enu = math.asin(sinp)
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw_enu = math.atan2(siny, cosy)
            # ★ NED pitch (nose-up 正): 掛機實測 MAVROS ENU/FLU 此式解出
            #   nose-up 為負 → 取負轉 NED. 增量式 PID 直接用, 符號錯會發散.
            self.pitch_deg = -math.degrees(math.asin(sinp))
            self.roll_deg = math.degrees(math.atan2(sinr, cosr))
            yaw = (90.0 - math.degrees(self.yaw_enu)) % 360.0
            vn, ve = self.vn, self.ve
            course = (math.degrees(math.atan2(ve, vn))
                      if vn is not None and ve is not None and vn*vn+ve*ve > 4
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

    # time_reference 超過這麼久沒更新 → gps_unix 視為無效 (留空, 不亂外插)
    GPS_TIME_STALE_S = 5.0

    def snapshot(self):
        with self.lock:
            d = dict(self.__dict__)
        # ★ 2026/09/15: gps_unix = 最後一筆 GPS 時間 + 收到後經過的本機時間.
        #   (time.time() 在飛行中不會被 NTP 大幅跳動; 短外插誤差 << 1 ms)
        if d.get('gps_unix_ref') is not None and d.get('gps_rx_t') is not None:
            age = time.time() - d['gps_rx_t']
            d['gps_unix'] = (d['gps_unix_ref'] + age
                             if age <= self.GPS_TIME_STALE_S else None)
        else:
            d['gps_unix'] = None
        return d


# ══════════════════════════════════════════════════════════════════
#  共享視覺狀態 (視覺線程寫, 控制線程讀 frame_idx / 雲台角)
# ══════════════════════════════════════════════════════════════════
class SharedVision:
    def __init__(self):
        self.lock = threading.Lock()
        self.generation = 0
        self.frame_idx = -1
        self.st = None                    # 最新 VisionState (真實 YOLO)
        self.gimbal_att = (0.0, 0.0, 0.0)
        self.geometry = {}
        self.projection_cfg = None
        self.gimbal_mode = 'follow'
        # ★ 2026/09/28: 延遲量測用 (time.monotonic())
        self.raw_idx = -1
        self.ready_mono = None
        self.cap_mono = None              # 這個結果的影像擷取時刻

    def put(self, frame_idx, st, gimbal_att, gimbal_mode,
            raw_idx=-1, cap_mono=None, geometry=None, generation=None):
        with self.lock:
            if generation is not None and generation != self.generation:
                return
            self.frame_idx = frame_idx
            self.st = st
            self.gimbal_att = gimbal_att
            self.gimbal_mode = gimbal_mode
            self.raw_idx = raw_idx
            self.cap_mono = cap_mono
            self.ready_mono = time.monotonic()
            self.geometry = dict(geometry or {})

    def reset_mission(self, generation):
        with self.lock:
            self.generation = generation
            self.frame_idx = self.raw_idx = -1
            self.st = None
            self.geometry = {}
            self.cap_mono = self.ready_mono = None
            self.gimbal_mode = 'follow'

    def get(self):
        with self.lock:
            return self.frame_idx, self.st, self.gimbal_att, self.gimbal_mode

    def get_packet(self):
        """Read result, frame IDs and timing atomically."""
        with self.lock:
            return (self.frame_idx, self.st, self.gimbal_att, self.gimbal_mode,
                    self.raw_idx, self.cap_mono, self.ready_mono, dict(self.geometry))

    def get_timing(self):
        """回傳 (raw_idx, cap_mono) — 最新視覺結果的相機幀序與擷取時刻."""
        with self.lock:
            return self.raw_idx, self.cap_mono


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


# ══════════════════════════════════════════════════════════════════
#  視覺線程 — 只驅動雲台 + 錄影 + 視覺 log; 不進飛機 PID
# ══════════════════════════════════════════════════════════════════
# ══════════════════════════════════════════════════════════════════
#  合成 bbox 注入 (掛機驗影像 PID → 舵面)
# ══════════════════════════════════════════════════════════════════
#  為什麼需要: 掛機時飛機不動, 虛擬目標在 AGL≈0 的幾何下:
#    · 目標只會【左右】移動 (視線永遠水平)
#    · 上下位置完全由機頭姿態決定 → 縱向 PID 驗不到
#  這裡直接覆寫 vfields 的 bbox, 讓目標自己在影像平面掃, 繞過幾何限制.
#  覆寫點在 virt.get_fields() 之後、送進狀態機之前, 所以下游 (狀態機 /
#  PID / bridge / mask / 錄影 / CSV) 全部走【正常路徑】.
#
#  幅度換算 (增益 PID_LAT_KP=4.5, K_LAT_TO_RUDDER=0.5):
#      50px → δr 3.0°   100px → 6.0°   200px → 12.1°   250px+ → 飽和
#  想看比例關係用 ≤200; 想看飽和行為才用 300.
INJECT_PATTERNS = ('lat', 'lon', 'step', 'corners')


def inject_bbox(name, t, cx, sp_v, amp, period):
    """回傳 (u, v). t 從注入開始算."""
    w = 2.0 * math.pi / max(period, 1e-3)
    if name == 'lat':                       # 只有 u 動 → 驗方向舵
        return cx + amp * math.sin(w * t), sp_v
    if name == 'lon':                       # 只有 v 動 → 驗升降舵
        return cx, sp_v + amp * math.sin(w * t)
    if name == 'step':                      # 對角階梯 → 看響應
        k = (0.0, 1.0, 0.0, -1.0)[int(t / (period / 2.0)) % 4]
        return cx + amp * k, sp_v + amp * k
    if name == 'corners':                   # 四角 → 驗四象限符號
        du, dv = ((+1, +1), (-1, +1), (-1, -1), (+1, -1))[
            int(t / (period / 2.0)) % 4]
        return cx + amp * du, sp_v + amp * dv
    return None, None


# ══════════════════════════════════════════════════════════════════
#  標註影片: 左上角任務模式 (2026/09/15)
# ══════════════════════════════════════════════════════════════════
_MODE_COLOR = {                       # BGR
    'NOT IN MISSION': (170, 170, 170),  # 灰
    'SEARCH':         (255, 255, 0),    # 青
    'PRE_DIVE':       (0, 255, 255),    # 黃
    'DIVE':           (0, 0, 255),      # 紅
    'PULLOUT':        (0, 165, 255),    # 橘
    'DONE':           (0, 255, 0),      # 綠
    'ABORT':          (255, 0, 255),    # 洋紅
}


def mission_label(guided, mission_mode, abort_reason=''):
    """沒切 / 脫離 GUIDED → NOT IN MISSION; 否則 = 狀態機模式."""
    if not guided:
        return 'NOT IN MISSION'
    m = mission_mode or 'SEARCH'
    if m == 'ABORT' and abort_reason:
        return f'ABORT: {abort_reason}'
    return m


def draw_mission_mode(frame, label, zoom=None):
    """左上角黑底色字. 字級依畫面高度縮放 (1080p ≈ 1.6).
    zoom 不是 None 時, 在模式框正下方再畫一行 'ZOOM {zoom}x' (白字)."""
    import cv2
    h = frame.shape[0]
    scale = max(0.6, h / 675.0)
    thick = max(2, int(round(scale * 2)))
    font = cv2.FONT_HERSHEY_SIMPLEX
    pad = int(10 * scale)

    # ── 第一行: 任務模式 ──
    (tw, th), base = cv2.getTextSize(label, font, scale, thick)
    x0, y0 = pad, pad
    y1 = y0 + th + base + 2 * pad
    cv2.rectangle(frame, (x0, y0), (x0 + tw + 2 * pad, y1), (0, 0, 0), -1)
    color = _MODE_COLOR.get(label.split(':')[0], (255, 255, 255))
    cv2.putText(frame, label, (x0 + pad, y0 + pad + th), font, scale, color,
                thick, cv2.LINE_AA)

    # ── 第二行: 相機倍率 ──
    if zoom is not None:
        ztxt = f'ZOOM {float(zoom):g}x'
        (zw, zh), zbase = cv2.getTextSize(ztxt, font, scale, thick)
        zy0 = y1 + pad // 2
        cv2.rectangle(frame, (x0, zy0),
                      (x0 + zw + 2 * pad, zy0 + zh + zbase + 2 * pad),
                      (0, 0, 0), -1)
        cv2.putText(frame, ztxt, (x0 + pad, zy0 + pad + zh), font, scale,
                    (255, 255, 255), thick, cv2.LINE_AA)
    return frame


def vision_loop(cfg, io, vf, grabber, gimbal, shared, video_annot, video_raw,
                mission_csv, flags, dive_flag, virt_sp_ref, t0, hb,
                err_log_path):
    """
    ~CAM_FPS Hz. 每處理一幀:
      YOLO → 雲台追蹤 (真實 bbox) → 標註影片
      → 視覺 log 釘飛機狀態 + 這幀虛擬 setpoint

    ★ 2026/09/09 兩項修正 (2026/09/08 實飛: 本執行緒在 754.7s 靜默死亡,
      主 log 卻跑到 1075.6s, raw 影片跟著缺了最後 5 分 21 秒):
      1. raw 錄影已搬到 FrameGrabber 取像執行緒, 本函式不再負責
      2. 逐幀 try/except: 單幀出錯只跳過該幀並記錄, 不再讓整條執行緒死
    hb: 心跳 dict, 主執行緒用它判斷本執行緒是否還活著.
    """
    frame_idx = -1
    last_raw_idx = -1
    last_t = time.time()
    consec_err = 0
    hb['alive'] = True
    vision_generation = -1
    while not flags['quit'] and not rospy.is_shutdown():
        # ★ 2026/09/15: 帶 since_id → 幀序沒變就回 None, 不再每 2ms 白複製
        #   一整張 1080p (約 6 MB) 才發現是重複幀.
        frame, raw_idx, cap_mono = grabber.read_with_id(
            since_id=last_raw_idx, with_t=True)
        if frame is None:
            if raw_idx >= 0 and raw_idx == last_raw_idx:
                hb['dup'] = hb.get('dup', 0) + 1     # 等新幀 (相機比迴圈慢)
                time.sleep(0.002)
            else:
                time.sleep(0.005)                    # 相機還沒出第一幀
            continue
        # ★★ 2026/09/11: 同一張相機影像不重複處理.
        #   FrameGrabber.read() 永遠回傳「最新幀」, 不會等新的.
        #   相機實測只有 15fps, 而本迴圈跑 23.7fps → 每張影像平均被
        #   處理 1.58 次. 後果不只是浪費 GPU:
        #     · DKF 會用同一筆量測連續 update → 共變異數被灌到過度自信
        #     · 標註影片與視覺 log 充滿重複幀, 統計出來的 fps 是假的
        #     · frame_dt 量到的是迴圈週期, 不是真正的量測間隔
        if raw_idx == last_raw_idx:
            hb['dup'] = hb.get('dup', 0) + 1
            time.sleep(0.002)
            continue
        last_raw_idx = raw_idx
        now = time.time()
        dt = now - last_t
        last_t = now
        frame_idx += 1
        try:
            generation, active, start_mono = mission_csv.snapshot()
            if generation != vision_generation:
                # No main-thread mutation of DKF/tracker while YOLO is running.
                vf.reset_mission()
                with mission_csv.lock:
                    if generation != mission_csv.generation:
                        continue
                    gimbal.reset_mission()
                vision_generation = generation
                dt = 1.0 / cfg.CAM_FPS
            # Pre-entry captured images may remain in the grabber; discard them.
            if active and cap_mono is not None and cap_mono < start_mono:
                continue
            _vision_step(cfg, io, vf, grabber, gimbal, shared, video_annot,
                         mission_csv, generation, dive_flag, virt_sp_ref, t0,
                         frame, raw_idx, frame_idx, now, dt,
                         cap_mono=cap_mono)
            consec_err = 0
            hb['t'] = now
            hb['frame'] = frame_idx
            hb['errors'] = hb.get('errors', 0)
        except Exception:
            consec_err += 1
            hb['errors'] = hb.get('errors', 0) + 1
            tb = traceback.format_exc()
            hb['last_err'] = tb.strip().splitlines()[-1]
            msg = (f"\n[Vision] ★ 第 {frame_idx} 幀處理例外 "
                   f"(連續第 {consec_err} 次, 累計 {hb['errors']}):\n{tb}")
            print(msg)
            try:
                with open(err_log_path, 'a') as ef:
                    ef.write(f"=== t={now - t0:.3f}s frame_idx={frame_idx} "
                             f"raw_idx={raw_idx} consec={consec_err} ===\n{tb}\n")
            except Exception:
                pass
            # 連續錯太多次 = 不是偶發, 停止視覺但【不】影響控制與 raw 錄影
            if consec_err >= 50:
                hb['alive'] = False
                hb['fatal'] = f"連續 {consec_err} 幀例外, 視覺已停止"
                print(f"[Vision] ★★★ {hb['fatal']} (raw 錄影與控制不受影響)")
                return
            time.sleep(0.01)
    hb['alive'] = False
    hb['fatal'] = None       # 正常退出


def _vision_step(cfg, io, vf, grabber, gimbal, shared, video_annot,
                 mission_csv, generation, dive_flag, virt_sp_ref, t0,
                 frame, raw_idx, frame_idx, now, dt, cap_mono=None):
    """單幀處理 (原 vision_loop 迴圈本體). 任何例外由呼叫端捕捉.
    cap_mono: 這一幀的相機擷取時刻 (time.monotonic()), 量延遲用."""
    ray_reference_mono = time.monotonic()
    s = io.snapshot()
    guided = s['guided']
    dive_mode = dive_flag.is_set()
    gy, gp, gr = gimbal.get_attitude()

    ac = {
        'roll_enu': s['roll_enu'] or 0.0,
        'pitch_enu': s['pitch_enu'] or 0.0,
        'yaw_enu': s['yaw_enu'] or 0.0,
        'body_pitch_deg': s['pitch_deg'] or 0.0,
        'body_yaw_true_deg': s['compass_hdg'] or 0.0,
        'agl': s['rel_alt'],
        'ray_reference_mono': ray_reference_mono,
    }
    if s['lat'] is not None:
        ac['x_ned'], ac['y_ned'] = gps_to_ned(
            s['lat'], s['lon'], NED_ORIGIN[0], NED_ORIGIN[1])
    else:
        ac['x_ned'] = ac['y_ned'] = 0.0
    gimbal_dict = {'yaw': gy, 'pitch': gp}

    # YOLO + DKF + ray 測距; 補償後真實 bbox 交給 DIVE PID。
    st, annot = vf.step(frame, ac, gimbal_dict, dive_mode=dive_mode, dt=dt,
                        cap_mono=cap_mono)

    # Reject commands from an inference that crossed a mission boundary.
    with mission_csv.lock:
        if generation != mission_csv.generation:
            return
        if not guided and not vf.locked:
            gimbal.standby_tick()
        elif dive_mode:
            gimbal.enter_fpv()
        else:
            has_t = st.detected or st.dkf_used
            u = st.u_used if st.u_used is not None else st.bbox_u
            v = st.v_used if st.v_used is not None else st.bbox_v
            gimbal.track_tick(has_t, u, v, cfg.IMG_W, cfg.IMG_H, vf.fx, vf.fy)

    # ★ 2026/09/15: 任務模式 (主迴圈每拍寫進 virt_sp_ref)
    m_label = mission_label(guided, virt_sp_ref.get('mission_mode'),
                            virt_sp_ref.get('abort_reason', ''))

    # 標註影片 (節流 25fps, 給人看)
    if video_annot is not None:
        draw_mission_mode(annot, m_label, zoom=cfg.ZOOM_LEVEL)
        video_annot.write(annot)

    # ★ 2026/09/28: 視覺結果完成時刻 (YOLO + DKF + 雲台 + 標註都做完)
    ready_mono = time.monotonic()
    if cap_mono is not None:
        _cap_to_ready = round((ready_mono - cap_mono) * 1000.0, 2)
        # 擷取時刻換到 t 的時間軸: now(time.time) 對應 ready 前的某一刻,
        # 用 monotonic 差值回推, 誤差 < 1 ms
        _cap_t = round((time.time() - t0) - (ready_mono - cap_mono), 3)
    else:
        _cap_to_ready = _cap_t = ''

    # ── 視覺 log (釘這一幀飛機狀態 + 虛擬 setpoint) ──
    vsp = dict(virt_sp_ref)   # 控制線程最新虛擬 setpoint (控制像素)
    _cu, _cv = cfg.virtual_to_camera_px(vsp.get('u', ''), vsp.get('v', ''))
    _cam_sp = ((round(_cu, 1) if isinstance(_cu, float) else _cu),
               (round(_cv, 1) if isinstance(_cv, float) else _cv))
    vrow = {
        'frame_idx': frame_idx, 'raw_frame_idx': raw_idx,
        't': round(now - t0, 3),
        'gps_unix': (f"{s['gps_unix']:.3f}" if s['gps_unix'] is not None else ''),
        'mode': 'DIVE' if dive_mode else ('GUIDED' if guided else 'STANDBY'),
        'dive_mode': int(dive_mode),
        'real_detected': int(st.detected),
        'real_bbox_u': round(st.bbox_u, 1) if st.bbox_u is not None else '',
        'real_bbox_v': round(st.bbox_v, 1) if st.bbox_v is not None else '',
        'real_bbox_w': round(st.bbox_w, 1) if st.bbox_w is not None else '',
        'real_bbox_h': round(st.bbox_h, 1) if st.bbox_h is not None else '',
        'real_conf': round(st.conf, 3) if st.conf is not None else '',
        'dkf_u': round(st.u_used, 1) if st.u_used is not None else '',
        'dkf_v': round(st.v_used, 1) if st.v_used is not None else '',
        'dkf_status': st.dkf_status or '',
        'ac_n': round(ac['x_ned'], 2), 'ac_e': round(ac['y_ned'], 2),
        'agl': round(s['rel_alt'], 2) if s['rel_alt'] is not None else '',
        'theta_deg': round(s['pitch_deg'], 2) if s['pitch_deg'] is not None else '',
        'chi_deg': round(s['compass_hdg'], 2) if s['compass_hdg'] is not None else '',
        'vn': round(s['vn'], 2) if s['vn'] is not None else '',
        've': round(s['ve'], 2) if s['ve'] is not None else '',
        'vd': round(s['vd'], 2) if s['vd'] is not None else '',
        'frame_dt': round(dt, 4),
        # ★ 2026/09/15: 控制座標 (虛擬 1x) → 相機像素 (2x), 視角不變.
        'virt_u_sp': _cam_sp[0], 'virt_v_sp': _cam_sp[1],
        'virt_u_sp_ctrl': vsp.get('u', ''), 'virt_v_sp_ctrl': vsp.get('v', ''),
        'mission_mode': m_label.replace(' ', '_'),
        'gimbal_yaw': round(gy, 1), 'gimbal_pitch': round(gp, 1),
        'gimbal_mode': gimbal.mode,
        'inf_ms': round(st.inf_ms, 2) if getattr(st, 'inf_ms', None) is not None else '',
        'delay_ms': round(st.delay_ms, 2) if getattr(st, 'delay_ms', None) is not None else '',
        'cap_t': _cap_t,
        'cap_to_ready_ms': _cap_to_ready,
    }
    try:
        geometry = capture_geometry(cfg, shared.projection_cfg, st, cap_mono,
                                    io.pose_history, gimbal)
    except Exception as exc:
        geometry = {'gcomp_valid': 0, 'gcomp_reason': 'geometry_error: ' + str(exc)}
    vrow.update(geometry)
    _, _ray_vrow = control_inputs(cfg, st, geometry, cap_mono,
                                 time.monotonic(), now - t0, None)
    vrow.update({key: _ray_vrow[key] for key in RAY_LOG_FIELDS})
    mission_csv.write(1, vrow, generation=generation, cap_mono=cap_mono)

    shared.put(frame_idx, st, (gy, gp, gr), gimbal.mode,
               raw_idx=raw_idx, cap_mono=cap_mono, geometry=geometry,
               generation=generation)


# ══════════════════════════════════════════════════════════════════
#  主程式 (控制線程)
# ══════════════════════════════════════════════════════════════════
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
        sys.exit(1)

    ap = argparse.ArgumentParser()
    ap.add_argument('--moving', dest='moving', action='store_true', default=None,
                    help='移動目標 (預設由 config.MOVING_TARGET 決定)')
    ap.add_argument('--static', dest='moving', action='store_false',
                    help='靜止目標')
    # 移動版: 目標起點 + 車速 (方位由 config 兩座標算; 不用飛機航向)
    ap.add_argument('--start', type=str, default=None,
                    help='移動目標起點 GPS "lat,lon" (未給用 config.MOVING_START_GPS)')
    ap.add_argument('--speed', type=float, default=None,
                    help='移動目標車速 m/s (未給用 config.TARGET_SPEED_MPS)')
    # 靜止版: 目標 GPS (進俯衝用「到目標水平距」, 門檻 STATIC_PREDIVE_DIST_M)
    ap.add_argument('--target', type=str, default=None,
                    help='靜止目標 GPS "lat,lon"')
    ap.add_argument('--predive-dist', type=float, default=None,
                    help='進俯衝距離門檻 m (覆寫 config; 靜止版覆寫 '
                         'STATIC_PREDIVE_DIST_M, 移動版覆寫 PREDIVE_DIST_M)')
    # ★ 2026/09/30: 靜止版已改用距離門檻, --predive 保留只為了舊指令不報錯
    ap.add_argument('--predive', type=str, default=None,
                    help='(已停用) 舊版固定 PRE_DIVE 點, 給了會被忽略')
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--no-annot', action='store_true',
                    help='不錄標註影片 (省一路 NVENC + 一半磁碟寫入). '
                         '框的座標都在視覺 CSV 裡, 事後用 make_annot_video.py '
                         '重建即可 — 內容完全一樣.')
    ap.add_argument('--bench', action='store_true',
                    help='掛機: 忽略 AGL 進入條件 (仍需 GUIDED). '
                         '會停在 PRE_DIVE 持續命令俯衝姿態, 方便觀察舵面.')
    ap.add_argument('--tgt-heading', type=float, default=None, metavar='DEG',
                    help='移動目標的行進方位 (度), 覆寫 config 的 '
                         'MOVING_HEADING_DEG / MOVING_HEADING_GPS. '
                         '掛機驗橫向時設成【垂直於視線】的方位, 車子才會橫掃'
                         '過畫面; 用 config 的馬路方位會讓車子直接跑遠.')
    ap.add_argument('--inject-bbox', choices=INJECT_PATTERNS, default=None,
                    help='掛機: 用合成 bbox 覆寫虛擬目標的影像位置, 驗影像 '
                         'PID → 舵面. lat=橫向掃(方向舵) lon=縱向掃(升降舵) '
                         'step=對角階梯 corners=四角. 只在 --bench 有效.')
    ap.add_argument('--inject-amp', type=float, default=150.0, metavar='PX',
                    help='合成 bbox 幅度 (px, 預設 150). 50→δr 3° '
                         '100→6° 200→12° 250以上飽和.')
    ap.add_argument('--inject-period', type=float, default=8.0, metavar='SEC',
                    help='合成 bbox 週期 (秒, 預設 8)')
    ap.add_argument('--bench-pitch', type=float, default=None, metavar='DEG',
                    help='掛機: DIVE 縱向增量的基準姿態覆寫 (例 --bench-pitch -20). '
                         'DIVE 的 pitch 是 pitch_now+dtheta, 桌上 pitch_now≈+1 → '
                         '一進 DIVE 舵面就回平. 設成 -20 才看得到俯衝姿態. '
                         '只在 --bench 有效, 實飛不要加.')
    ap.add_argument('--bench-dive', action='store_true',
                    help='掛機時允許跳過 pitch 容差直接交棒 DIVE. '
                         '★ 注意: DIVE 的 pitch 命令是相對實際姿態的增量式, '
                         '桌上實際 pitch≈0 → 進 DIVE 後升降舵就不動了. '
                         '只有要驗狀態機轉換時才用, 看舵面請用 --bench.')
    ap.add_argument('--ground-test', action='store_true',
                    help='地面測試: SEARCH 保持機頭不朝目標 (切 GUIDED 不打 yaw)')
    ap.add_argument('--dive-hold-heading', action='store_true',
                    help='版本 B: DIVE 不修 yaw, 凍結進 DIVE 那刻的航向 (縱向仍修)')
    ap.add_argument('--case', type=str, default=None,
                    help='自訂 case 標籤加進檔名 (未給則自動用 移動/靜止+版本A/B)')
    args = ap.parse_args()

    cfg = Config()
    # ★★ 2026/09/15: 控制主線 (虛擬目標 / PID / 注入 / 主 log) 用【虛擬內參】;
    #   真實影像 (VisionFrontend / 雲台追蹤 / 錄影) 在各自模組內用相機內參.
    #   本函式裡的 fx, fy, cx, cy 一律指控制座標系.
    fx, fy, cx, cy = cfg.virtual_intrinsics()
    cam_fx, cam_fy, cam_cx, cam_cy = cfg.effective_intrinsics()

    moving = cfg.MOVING_TARGET if args.moving is None else args.moving
    # ★ 2026/09/12: 把實際模式寫回 cfg, 否則 --static 時 vision.py 的
    #   地理閘門還是會用移動目標的膨脹半徑 (兩邊模式不一致).
    cfg.MOVING_TARGET = moving

    if moving:
        start_gps = (_parse_gps(args.start, 'start') if args.start
                     else cfg.MOVING_START_GPS)
        speed = args.speed if args.speed is not None else cfg.TARGET_SPEED_MPS
        if start_gps is None:
            print("[FATAL] 移動版需目標起點 GPS (--start 或 config.MOVING_START_GPS)")
            sys.exit(1)
        # 方位優先序: --tgt-heading > config.MOVING_HEADING_DEG > 兩點算
        if args.tgt_heading is not None:
            heading = float(args.tgt_heading) % 360.0
            print(f"[flight] 目標方位由 --tgt-heading 指定: {heading:.1f}°")
        elif cfg.MOVING_HEADING_DEG is not None:
            heading = cfg.MOVING_HEADING_DEG
        else:
            hn, he = gps_to_ned(cfg.MOVING_HEADING_GPS[0], cfg.MOVING_HEADING_GPS[1],
                                start_gps[0], start_gps[1])
            heading = math.degrees(math.atan2(he, hn)) % 360.0
        print(f"[flight] 移動目標: 起點 {start_gps} 方位 {heading:.1f}° 車速 {speed} m/s")
    else:
        target_gps = (_parse_gps(args.target, 'target') if args.target
                      else getattr(cfg, 'TARGET_GPS', None))
        if target_gps is None:
            print("[FATAL] 靜止版需目標 GPS (--target 或 config.TARGET_GPS)")
            sys.exit(1)
        if args.predive:
            print("[flight] ★ --predive 已停用 (靜止版改用到目標水平距), 忽略")

    # ★ 2026/09/30: 靜止/移動都用「飛機到目標水平距 ≤ 門檻」進 PRE_DIVE,
    #   只差門檻各自分開設.
    if args.predive_dist is not None:
        predive_dist = float(args.predive_dist)
    elif moving:
        predive_dist = cfg.PREDIVE_DIST_M
    else:
        predive_dist = cfg.STATIC_PREDIVE_DIST_M
    print(f"[flight] 進俯衝門檻: 到目標水平距 ≤ {predive_dist:.0f} m "
          f"({'移動' if moving else '靜止'}版; "
          f"來源={'ray projection' if cfg.PREDIVE_USE_RAY_PROJECTION else '虛擬目標'})")

    rospy.init_node('vision_dive_flight', anonymous=True)
    io = MavrosIO()

    gcfg = GuidanceConfig(
        DIVE_ANGLE_DEG=cfg.DIVE_ANGLE_DEG,
        ALPHA_NOM_DEG=cfg.ALPHA_NOM_DEG,
        ALPHA_DIVE_DEG=cfg.ALPHA_DIVE_DEG,
        PREDIVE_TRIGGER_MODE='target_dist',     # 靜止/移動都看到目標距離
        PREDIVE_DIST_M=predive_dist,
        DIVE_USE_REAL_CAMERA=True,
        PREDIVE_HANDOFF_PITCH_TOL_DEG=cfg.PREDIVE_HANDOFF_PITCH_TOL_DEG,
        PULLOUT_AGL_M=cfg.PULLOUT_AGL_M,
        PULLOUT_VMAX_MPS=cfg.PULLOUT_VMAX_MPS,
        TECS_HOLD_ALT_M=cfg.TECS_HOLD_ALT_M,
        TECS_SPEED_MPS=cfg.TECS_SPEED_MPS,
        ABORT_MIN_AGL_M=(0.0 if args.bench else cfg.ABORT_MIN_AGL_M),
        ABORT_TARGET_LOST_S=cfg.ABORT_TARGET_LOST_S,
        ENTRY_MIN_AGL_M=(0.0 if args.bench else 60.0),
        BENCH_GEOMETRY_AGL_M=(120.0 if args.bench else 0.0),
        BENCH_HANDOFF_ANY_PITCH=args.bench_dive,
        BENCH_DIVE_PITCH_BASE_DEG=(args.bench_pitch if args.bench else None),
        FX=fx, FY=fy, CX=cx, CY=cy, IMG_W=cfg.IMG_W, IMG_H=cfg.IMG_H,
        PID_LAT_KP=cfg.PID_LAT_KP, PID_LAT_KI=cfg.PID_LAT_KI, PID_LAT_KD=cfg.PID_LAT_KD,
        PID_LON_KP=cfg.PID_LON_KP, PID_LON_KI=cfg.PID_LON_KI, PID_LON_KD=cfg.PID_LON_KD,
        PID_SP_LEAD_S=cfg.PID_SP_LEAD_S,
        PID_MEAS_TAU_S=cfg.PID_MEAS_TAU_S, PID_D_TAU_S=cfg.PID_D_TAU_S,
        PID_OUT_TAU_S=cfg.PID_OUT_TAU_S,
        # 移動: 前置瞄準; 靜止: 不前置.
        # ★ 2026/09/12: --inject-bbox 時強制關閉前置瞄準.
        #   原因: lead_setpoint_px 依賴 t_go, 而掛機時飛機不動 →
        #     vd≈0  → 不走 AGL/vd
        #     接近率≈0 → t_go = 斜距/max(接近率,3) 被夾在 30s 上限
        #   → 前置點偏目標 60m → setpoint 偏畫面中心 850px 以上
        #   → 誤差 = bbox − setpoint 恆為大值 → PID 直接飽和 → 舵面貼死不動
        #   (2026/09/12 實測: u_sp 掉到 112, dpsi 全程飽和 30°)
        #   注入的目的是看「誤差變化 → 舵面修正」, 前置瞄準會把誤差直接
        #   推到飽和, 兩件事互相汙染. 前置瞄準的正確性只能實飛驗
        #   (它需要真實接近率), 掛機驗不了.
        # ★ 2026/09/15: 再加 config.MOVING_TARGET_LEAD 總開關 (預設 False = 關).
        DIVE_TARGET_LEAD=(moving and getattr(cfg, 'MOVING_TARGET_LEAD', False)
                          and not (args.bench and args.inject_bbox)),
        DIVE_AIM_V_OFFSET_PX=cfg.virtual_aim_v_offset_px,   # 控制像素
        GIMBAL_DIVE_TILT_BODY_DEG=0.0,
        GIMBAL_FPV_SETTLE_S=0.3,
        SEARCH_HOLD_HEADING=args.ground_test,   # 地面測試不朝目標打 yaw
        DIVE_HOLD_HEADING=args.dive_hold_heading,  # 版本 B: DIVE 不修 yaw
    )
    sm = MissionStateMachine(gcfg, verbose=True)
    real_pid_logger = GimbalCompPIDLogger(cfg, gcfg)

    bcfg = AttitudeBridgeConfig()
    bcfg.TECS_HOLD_ALT_M = cfg.TECS_HOLD_ALT_M
    bcfg.TECS_SPEED_MPS = cfg.TECS_SPEED_MPS
    bcfg.V_REF_MPS = cfg.V_CRUISE_MPS
    bcfg.THR_TRIM = cfg.THR_TRIM
    bcfg.THR_MIN = cfg.THR_MIN
    # ★ 2026/09/12: 方向舵迴路參數改由 config.py 統一管理 (原本寫在
    #   AttitudeBridgeConfig 裡, 場地上要改得翻兩個檔案).
    #   用 getattr 取, 舊 config 沒有這些欄位就沿用 bridge 自己的預設值.
    for _k in ('RUDDER_LIMIT_DEG', 'K_PSI', 'K_R', 'K_LAT_TO_RUDDER',
               'RUDDER_SLEW_DPS', 'RUDDER_VSCHED',
               'ROLL_PROT_FULL_DEG', 'ROLL_PROT_ZERO_DEG',
               'GUIDED_ENTRY_ROLL0_S'):
        if hasattr(cfg, _k):
            setattr(bcfg, _k, getattr(cfg, _k))
    bridge = ArduPilotAttitudeBridge(dive_angle_deg=cfg.DIVE_ANGLE_DEG, cfg=bcfg)

    from gimbal import GimbalManager
    gimbal = GimbalManager(cfg)
    if not gimbal.connect():
        print("[FATAL] 雲台連線失敗")
        sys.exit(1)

    grabber = FrameGrabber(cfg)
    if not grabber.start():
        print("[FATAL] 取像失敗")
        gimbal.close()
        sys.exit(1)

    vf = VisionFrontend(cfg)

    print("[flight] 等待 GPS fix ...")
    t_wait = time.time()
    while not rospy.is_shutdown():
        s = io.snapshot()
        if s['lat'] is not None:
            break
        if time.time() - t_wait > 30.0:
            print("[flight] ★ 30s 沒等到 GPS, 檢查 MAVROS")
            t_wait = time.time()
        rospy.sleep(0.5)

    s = io.snapshot()
    global NED_ORIGIN
    NED_ORIGIN = (float(start_gps[0]), float(start_gps[1])) if moving else \
                 (float(target_gps[0]), float(target_gps[1]))
    o_lat, o_lon = NED_ORIGIN
    print(f"[flight] NED 原點固定在{'車起點' if moving else '目標'} GPS "
          f"({o_lat:.7f}, {o_lon:.7f}) — 與飛控 home 無關")
    from virtual_source import StaticTarget, MovingTarget
    if moving:
        sn, se = gps_to_ned(start_gps[0], start_gps[1], o_lat, o_lon)   # = (0, 0)
        # t0 佔位 (實際在 GUIDED 上升沿重設); 進俯衝用「到目標距離」故 predive_point=None
        # ★ 2026/09/15: 佔位 0.0 → inf. 舊值讓虛擬車從【程式啟動】就開始跑,
        #   任務前 log 的 tgt_n/e 與 console 的 pd_dist 全是錯的.
        #   inf → max(0, t−inf)=0 → 撥 GUIDED 前停在起點.
        target = MovingTarget(sn, se, heading, speed, t0=float('inf'))
        virt = VirtualSource(cfg, gcfg, o_lat, o_lon, target,
                             predive_point_ne=None)
    else:
        tn, te = gps_to_ned(target_gps[0], target_gps[1], o_lat, o_lon)  # = (0, 0)
        target = StaticTarget(tn, te)
        # predive_point=None → 進俯衝用「到目標水平距」(同移動版)
        virt = VirtualSource(cfg, gcfg, o_lat, o_lon, target,
                             predive_point_ne=None)

    # ── log + 影片 ──
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), cfg.OUT_DIR)
    os.makedirs(log_dir, exist_ok=True)
    # ★ case 標籤加進檔名, 不用另外記哪趟是哪個 case.
    #   自訂 --case 優先; 否則自動組: 移動/靜止 + 版本A/B (yaw修/凍結)
    if args.case:
        tag = ''.join(c if (c.isalnum() or c in '-_') else '_' for c in args.case)
    else:
        tag = ('moving' if moving else 'static') + \
              ('_B_yawhold' if args.dive_hold_heading else '_A_yawpid')
    stem = f'flight_{stamp}_{tag}'
    ev_path = os.path.join(log_dir, f'{stem}_events.txt')
    vid_annot = os.path.join(log_dir, f'{stem}.mp4')
    vid_raw = os.path.join(log_dir, f'{stem}_raw.mp4')
    mission_csv = MissionCSV(log_dir, tag, CSV_FIELDS, VISION_FIELDS, recording_stem=stem)

    fev = open(ev_path, 'a', buffering=1)
    t0 = time.time()

    # ★ 2026/09/15: TECS 目標改背景發送, 結果由背景執行緒回呼 event()
    #   → 兩條執行緒會同時寫 events.txt, 加鎖避免行交錯.
    _ev_lock = threading.Lock()

    def event(msg):
        line = f"{time.time() - t0:8.2f}s  {msg}"
        print(line)
        with _ev_lock:
            try:
                fev.write(line + '\n')
                fev.flush(); os.fsync(fev.fileno())
            except (OSError, ValueError):
                pass          # 收尾時檔案已關, 背景回呼晚到 → 只印不寫

    # ★ 2026/09/15: 把「這趟用什麼變焦 / 前置瞄準」寫進 events,
    #   log 自己說明自己, 後處理時不會拿錯內參或搞混有沒有前置.
    event(f"相機 ZOOM_LEVEL={cfg.ZOOM_LEVEL:g}x  相機內參 fx={cam_fx:.1f} "
          f"fy={cam_fy:.1f} cx={cam_cx:.1f} cy={cam_cy:.1f}  "
          f"攻角補償 v+={cfg.dive_aim_v_offset_px:.1f}px (相機像素)")
    event(f"虛擬 VIRTUAL_ZOOM_LEVEL={getattr(cfg, 'VIRTUAL_ZOOM_LEVEL', cfg.ZOOM_LEVEL):g}x  "
          f"控制內參 fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}  "
          f"攻角補償 v+={cfg.virtual_aim_v_offset_px:.1f}px (控制像素)  "
          f"| 主 log 像素=控制座標, 視覺 log virt_u/v_sp=相機座標")
    event(f"目標={'移動' if moving else '靜止'}  前置瞄準 DIVE_TARGET_LEAD="
          f"{'ON' if gcfg.DIVE_TARGET_LEAD else 'OFF'}")

    video_annot = video_raw = None
    if not args.no_video:
        # ★ 2026/09/11: --no-annot 省掉一路 1080p NVENC.
        #   標註影片 = raw 影片 + 框, 而框的座標全在視覺 CSV 裡,
        #   事後 make_annot_video.py 可以完全重建 → 飛行中沒有必要編碼.
        #   Jetson 出現 over-current 降頻時, 這是最划算的減負.
        if args.no_annot:
            print("[flight] --no-annot: 不錄標註影片 "
                  "(事後 make_annot_video.py 重建)")
        else:
            # 節流關閉: 視覺迴圈本來就低於 VIDEO_FPS, 節流只會白丟幀
            video_annot = VideoWriter(vid_annot, cfg.VIDEO_FPS,
                                      cfg.IMG_W, cfg.IMG_H, throttle=False)
            if not video_annot.ok:
                video_annot = None
        # ★ raw: 不節流, queue 放大 (8 幀只有 0.3s 緩衝, 編碼器一頓就掉幀)
        video_raw = VideoWriter(vid_raw, cfg.CAM_FPS, cfg.IMG_W, cfg.IMG_H,
                                throttle=False, queue_len=120)
        if not video_raw.ok:
            print("[flight] ★ raw 影片啟動失敗 — 後處理需要它!")
            video_raw = None
        else:
            # ★★ raw 錄影掛到取像執行緒 (與 YOLO 解耦).
            #    YOLO/DKF/雲台/CSV 任何一步掛掉都不會再讓 raw 影片跟著停.
            grabber.recorder = video_raw
            print("[flight] raw 錄影已掛在取像執行緒 (與 YOLO 解耦)")

    # 背景 fsync (兩個 csv)
    _sync_run = {'on': True}

    def _fsync_loop():
        while _sync_run['on']:
            try:
                mission_csv.sync()
            except OSError as exc:
                event(f"CSV 同步失敗: {exc}")
            time.sleep(1.0 / cfg.FSYNC_HZ)
    threading.Thread(target=_fsync_loop, daemon=True).start()
    print("[flight] CSV 在每次進 GUIDED 建立新一組，撥離即關檔")
    print(f"[flight] 影片 → {vid_annot} + {vid_raw} (raw)")

    flags = {'arm': False, 'abort': False, 'quit': False}
    threading.Thread(target=keyboard_thread, args=(flags,), daemon=True).start()

    shared = SharedVision()
    shared.projection_cfg = gcfg
    dive_flag = threading.Event()
    virt_sp_ref = {'u': cx, 'v': cy,   # 控制線程每拍更新, 視覺線程讀去記 log
                   'mission_mode': 'SEARCH', 'abort_reason': ''}   # ★ 影片左上角

    # ★ 視覺心跳 (主執行緒看門狗用) + 例外 log
    vision_hb = {'t': time.time(), 'frame': -1, 'alive': False,
                 'errors': 0, 'fatal': None, 'last_err': None, 'dup': 0}
    err_log_path = os.path.join(log_dir, f'{stem}_error.log')
    vision_thread = threading.Thread(
        target=vision_loop,
        args=(cfg, io, vf, grabber, gimbal, shared, video_annot, video_raw,
              mission_csv, flags, dive_flag, virt_sp_ref, t0, vision_hb,
              err_log_path),
        daemon=True)
    vision_thread.start()

    bridge.start()
    print("[flight] ★ AUTO 對準 → 撥 GUIDED = 任務開始; 到目標距離門檻自動俯衝")
    print("[flight] 掛機備用鍵盤: d=arm x=abort q=quit")

    rate = rospy.Rate(LOOP_HZ)
    half_w, half_h = cfg.IMG_W / 2.0, cfg.IMG_H / 2.0
    guided_prev = False
    mode_prev = sm.mode
    last_print = 0.0
    # 合成 bbox 注入狀態 (只在 --bench 生效; 實飛 args.inject_bbox 為 None)
    _inject = args.inject_bbox if args.bench else None
    _inject_t0 = None
    _inject_sp_v = cy + cfg.virtual_aim_v_offset_px   # 注入在控制座標系
    if _inject and not args.bench:
        print("[flight] ⚠ --inject-bbox 只在 --bench 有效, 已忽略")
    if _inject and moving and getattr(cfg, 'MOVING_TARGET_LEAD', False):
        print("[flight] ⚠ --inject-bbox + 移動目標: 前置瞄準 (DIVE_TARGET_LEAD) "
              "已自動關閉")
        print("[flight]   掛機時 t_go 會被夾在 30s 上限 → setpoint 偏 850px → "
              "誤差直接飽和")
        print("[flight]   前置瞄準的正確性請用 test_logic_sim.py --check 驗 "
              "(那裡飛機真的會飛)")
    _vid_check_t = 0.0
    _vid_warned = set()
    err_count = 0
    scored = {'done': False}
    _miss_warned = False
    pid_last_t = None
    _vis_prev_raw = -1                  # ★ 2026/09/28: 判斷 vis_new 用
    _cap_lat_ms = float(getattr(cfg, 'CAPTURE_LATENCY_MS', 0.0) or 0.0)

    pending_arm = False

    def do_arm(s, t):
        # Moving target timing is initialized at the GUIDED edge.
        if moving:
            # Movement clock was set at GUIDED entry, even if GPS was delayed.
            tn, te, _, _ = virt.target_now(t)
            event(f"種點(移動): 目標當下 NED=({tn:.1f},{te:.1f}) "
                  f"方位 {virt.target.heading_deg:.1f}° 車速 {virt.target.speed} m/s "
                  f"t0={virt.target.t0:.1f}s")
        else:
            tn, te, _, _ = virt.target_now(t)
            event(f"種點(靜止): 目標 NED=({tn:.1f},{te:.1f}) "
                  f"進俯衝門檻 到目標水平距 ≤ {gcfg.PREDIVE_DIST_M:.0f}m")
        # ★ 2026/09/11: 撥 GUIDED 時送【當下高度】, 不是 TECS_HOLD_ALT_M.
        #   SEARCH 已改成 pitch/高度全交 TECS, 這裡若送 70m, 飛機一撥 GUIDED
        #   就會從巡航 120m 往下掉, 到 PRE_DIVE 點高度全錯.
        #   70m 等進 PRE_DIVE 才送 (sm.tecs_resend 觸發; PULLOUT 再保險重送一次).
        _cruise_alt = s.get('rel_alt')
        if _cruise_alt is None or _cruise_alt < 5.0:
            _cruise_alt = cfg.CRUISE_AGL_M          # 掛機/沒高度時用設定值
        _search_spd = getattr(cfg, 'SEARCH_SPEED_MPS', cfg.TECS_SPEED_MPS)

        # ★ 2026/09/15: 改背景發送 (見 bridge.send_tecs_targets_async).
        #   結果晚一點由背景執行緒回報, 不再卡住 50Hz 控制迴圈.
        def _arm_tecs_done(ok_a, ok_s, _alt=_cruise_alt, _spd=_search_spd):
            event(f"TECS 目標預送完成 ALT={_alt:.0f}m "
                  f"SPD={_spd:.0f}m/s ok=({ok_a},{ok_s})")
            if not ok_s:
                event(f"★ SEARCH 空速目標 {_spd:.0f}m/s 被飛控拒絕 — "
                      f"檢查 AIRSPEED_MIN (舊韌體 ARSPD_FBW_MIN) "
                      f"是否 ≤ {_spd:.0f}")

        bridge.send_tecs_targets_async(alt_m=_cruise_alt,
                                       speed_mps=_search_spd,
                                       tag='/巡航', on_done=_arm_tecs_done)
        event(f"TECS 目標預送 (背景) ALT={_cruise_alt:.0f}m "
              f"SPD={_search_spd:.0f}m/s")
        sm.arm()

    try:
        while not rospy.is_shutdown() and not flags['quit']:
            try:
                _tick_mono = time.monotonic()     # ★ ctrl_ms 起點
                t = time.time() - t0
                s = io.snapshot()

                if s['guided'] != guided_prev:
                    # Release old commands even when GPS is temporarily unavailable.
                    bridge.reset_mission()
                    dive_flag.clear()
                    sm.reset_mission()
                    real_pid_logger = GimbalCompPIDLogger(cfg, gcfg)
                    mode_prev = sm.mode
                    scored['done'] = False
                    _miss_warned = False
                    _inject_t0 = None
                    pid_last_t = None
                    _vis_prev_raw = -1
                    virt_sp_ref.update(u=cx, v=cy, mission_mode=sm.SEARCH,
                                       abort_reason='')
                    flags['arm'] = flags['abort'] = False
                    if s['guided']:
                        paths = mission_csv.begin(t, _tick_mono)
                        virt.reset_mission(t)
                        pending_arm = True
                        event(f"★ 任務 {mission_csv.mission_id:03d} 開始 — GUIDED 上升沿")
                        event(f"主 log → {paths[0]} | 視覺 log → {paths[1]}")
                    else:
                        pending_arm = False
                        virt.reset_mission()
                        mission_csv.end()
                        event(f"★ 任務 {mission_csv.mission_id:03d} 結束 — 已關閉 CSV")
                    shared.reset_mission(mission_csv.generation)
                    guided_prev = s['guided']

                # Wait for GPS without requiring a second switch edge.
                if pending_arm and s['guided'] and s['lat'] is not None:
                    do_arm(s, t)
                    pending_arm = False

                if flags['arm']:
                    flags['arm'] = False
                    # ★ 2026/09/15: 已 arm 就不再重來 (否則移動目標 t0 被重設,
                    #   虛擬車跳回起點, TECS 高度也被改回當下高度).
                    if sm.mission_armed:
                        print("[flight] 任務已 ARM, 忽略 'd'")
                    elif s['guided'] and s['lat'] is not None:
                        event("★ 鍵盤 'd' — 種點 + ARM")
                        do_arm(s, t)
                        pending_arm = False
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

                # 虛擬目標保留為距離基準、進場幾何與移動 setpoint 來源。
                vac = {
                    'n': ac_n, 'e': ac_e, 'agl': agl,
                    'theta_deg': pitch, 'chi_deg': hdg,
                    'gimbal_locked': dive_mode,   # DIVE 用 body 投影, 否則置中
                    't': t,                       # 移動目標算當下位置用
                }
                vfields = virt.get_fields(vac)

                # ── 合成 bbox 注入 (只在 --bench + --inject-bbox) ──
                if _inject is not None:
                    if _inject_t0 is None:
                        _inject_t0 = t
                        print(f"[flight] ★ 合成 bbox 注入啟動: {_inject} "
                              f"幅度 ±{args.inject_amp:.0f}px "
                              f"週期 {args.inject_period:.0f}s "
                              f"(瞄準點 u={cx:.0f} v={_inject_sp_v:.1f})")
                    _iu, _iv = inject_bbox(_inject, t - _inject_t0, cx,
                                           _inject_sp_v, args.inject_amp,
                                           args.inject_period)
                    if _iu is not None:
                        vfields['bbox_u'] = _iu
                        vfields['bbox_v'] = _iv
                        vfields['bbox_detected'] = 1
                        vfields['bbox_center_offset_norm'] = (
                            math.hypot(_iu - cx, _iv - cy)
                            / math.hypot(cfg.IMG_W / 2.0, cfg.IMG_H / 2.0))

                (frame_idx, vst, gatt, gmode, _vis_raw, _vis_cap,
                 _vis_ready, _vis_geometry) = shared.get_packet()
                _vis_new = int(_vis_raw >= 0 and _vis_raw != _vis_prev_raw)
                _vis_prev_raw = _vis_raw

                # 組狀態機 inp: 距離切換與真實補償 bbox 使用同一份相機封包。
                inp = {
                    't': t, 'guided': s['guided'],
                    'agl_m': agl, 'airspeed_mps': s['airspeed'],
                    'gs_mps': (math.hypot(s['vn'], s['ve'])
                               if s['vn'] is not None else None),
                    'vn': s['vn'], 've': s['ve'], 'vd': s['vd'],
                    'yaw_true_deg': hdg, 'pitch_deg': pitch, 'roll_deg': s['roll_deg'],
                    'ac_n': ac_n, 'ac_e': ac_e,
                    **vfields,   # tgt_n/tgt_e/tgt_valid/slant_m/ground_m/
                                 # predive_dist_m/bbox_*/bbox_center_offset_norm
                }
                _camera_inputs, _ray_row = control_inputs(
                    cfg, vst, _vis_geometry, _vis_cap, time.monotonic(), t,
                    vfields['ground_m'])
                inp.update(_camera_inputs)
                if inp['real_control_detected']:
                    inp['real_control_offset_norm'] = math.hypot(
                        inp['real_control_u'] - cx, inp['real_control_v'] - cy
                    ) / math.hypot(cfg.IMG_W / 2., cfg.IMG_H / 2.)
                _virtual_seq_before = sm.pid.update_seq
                out = sm.update(inp)
                # Raw PID 為 observer; real_comp_* 直接記錄 sm.pid 實際控制值。
                _real_pid_row = dict.fromkeys(REAL_PID_FIELDS + COMP_PID_FIELDS + GEOMETRY_FIELDS, '')
                try:
                    _real_pid_row = real_pid_logger.tick(
                        sm, out, inp, _virtual_seq_before, vst, frame_idx,
                        _vis_raw, _vis_cap, _vis_ready, time.monotonic(),
                        time.time(), t0, _vis_geometry)
                except Exception as exc:
                    # CSV observer 故障不影響獨立的真實相機控制路徑。
                    if real_pid_logger.enabled:
                        event(f"真實 PID 記錄停用 (相機補償控制繼續): {exc}")
                    real_pid_logger.enabled = False
                    _real_pid_row['real_pid_enabled'] = 0

                # ── 視覺線程要記的虛擬 setpoint ───────────────────
                # ★★ 2026/09/12 修正 (影響論文核心比較):
                #   舊版把 vfields['bbox_u/v'] 寫進來 —— 那是虛擬目標投影在
                #   畫面上的【量測值】, 不是 PID 的 setpoint.
                #   真正的 setpoint 在 dive_guidance 裡另外算:
                #     sp_u/v = lead_setpoint_px(...)      移動目標的前置偏移
                #     sp_v  += DIVE_AIM_V_OFFSET_PX       攻角補償 (1x: 24.0 px; 舊 2x: 48.1 px)
                #     再過 τ=0.6s 低通
                #   後處理是拿 virt_u_sp/virt_v_sp 當參考去重放 real PID 的,
                #   記錯就等於【把誤差當成目標】, real-vs-virtual 的逐幀比較
                #   全部失效 —— 而那正是這一版流程要驗的核心結論.
                #   靜止版也躲不掉: 那 24 px (1x) 的攻角補償是常數偏移.
                #   必須放在 sm.update() 之後才拿得到.
                virt_sp_ref['u'] = out.get('pid_u_sp', cx)
                virt_sp_ref['v'] = out.get('pid_v_sp', cy)
                virt_sp_ref['mission_mode'] = out['mode']
                virt_sp_ref['abort_reason'] = out.get('abort_reason') or ''

                if out['mode'] == 'DIVE':
                    dive_flag.set()
                else:
                    dive_flag.clear()

                if out['mode'] != mode_prev:
                    banner = (f"★★ MODE: {mode_prev} → {out['mode']} "
                              f"(t={t:.1f}s AGL={agl if agl is not None else '?'} "
                              f"V={V:.1f} pitch_cmd={out['pitch_cmd_deg']:+.1f} "
                              f"hdg_cmd={out['heading_cmd_deg']:.1f}"
                              + (f" abort={out.get('abort_reason')}"
                                 if out.get('abort_reason') else "") + ")")
                    print("\n" + "=" * 64 + "\n" + banner + "\n" + "=" * 64)
                    event(banner)
                    mode_prev = out['mode']
                    if getattr(sm, 'tecs_resend', False):
                        # 進 PRE_DIVE (提早讓 TECS 收斂) 與進 PULLOUT (保險重送)
                        # 都會設這個旗標 → TECS 高度目標切到拉平高度.
                        # ★★ 2026/09/15: 改背景發送. 原本同步呼叫會在
                        #   PRE_DIVE / PULLOUT 當拍卡住控制迴圈, 超過 0.4s
                        #   bridge 看門狗就停送 (拉平命令送不出去, log 斷掉).
                        sm.tecs_resend = False
                        _tag = f"/{out['mode']}"

                        def _hold_done(ok_a, ok_s, _tag=_tag):
                            event(f"TECS 高度目標 → {cfg.TECS_HOLD_ALT_M:.0f}m "
                                  f"完成{_tag} ok=({ok_a},{ok_s})")

                        bridge.send_tecs_targets_async(
                            alt_m=cfg.TECS_HOLD_ALT_M, tag=_tag,
                            on_done=_hold_done)
                        event(f"TECS 高度目標 → {cfg.TECS_HOLD_ALT_M:.0f}m "
                              f"(背景發送{_tag})")

                _sub_mono = time.monotonic()
                # ★ 2026/09/30: 傳 guided → bridge 自己抓上升沿做
                #   「roll=0 維持 GUIDED_ENTRY_ROLL0_S + 43002 立即開送」.
                #   hdg_valid: 羅盤沒值時 hdg 是 0.0 充數, 不能拿來鎖機頭.
                bridge.submit(out, hdg, s['yaw_rate_dps'], s['airspeed'],
                              roll_deg=s['roll_deg'], src_cap_mono=_vis_cap,
                              guided=s['guided'],
                              hdg_valid=s['compass_hdg'] is not None)
                _ctrl_ms = (time.monotonic() - _tick_mono) * 1000.0
                _vis_age = ((_sub_mono - _vis_cap) * 1000.0 + _cap_lat_ms
                            if _vis_cap is not None else None)
                _i2c = getattr(bridge, 'last_img_to_cmd_ms', None)

                # ── 主 log ──
                dt_loop = (t - pid_last_t) if pid_last_t is not None else 0.0
                pid_last_t = t
                gam = (math.degrees(math.asin(max(-1, min(1,
                       -s['vd'] / max(V, 1.0))))) if s['vd'] is not None else 0.0)
                vu = vfields['bbox_u']
                vv = vfields['bbox_v']
                _brg = bridge.get_debug()

                # ── 逐幀 miss distance (沿當下速度向量外插到地面) ──
                _mp = _mlon = _mlat = _tf = ''
                try:
                    # ★ 2026/09/11 修正: 原本寫 s['ac_n'] / s['ac_e'], 但
                    #   MavrosIO 沒有這兩個屬性 (NED 是下面 gps_to_ned 算的
                    #   local variable) → KeyError 被 except 吃掉 →
                    #   miss_pred_* 永遠是空值而且不報錯.
                    if (agl is not None and agl > 0.5
                            and (s['vd'] or 0.0) > 0.1
                            and vfields.get('tgt_valid')):
                        _in, _ie, _tfall = extrapolate_impact(
                            ac_n, ac_e, agl,
                            s['vn'] or 0.0, s['ve'] or 0.0, s['vd'] or 0.0)
                        if _in is not None:
                            _md = miss_distance(
                                _in, _ie, _tfall,
                                vfields['tgt_n'], vfields['tgt_e'],
                                vfields.get('tgt_vn') or 0.0,
                                vfields.get('tgt_ve') or 0.0,
                                s['vn'] or 0.0, s['ve'] or 0.0)
                            _mp = round(_md['miss_total'], 2)
                            _mlon = round(_md['miss_lon'], 2)
                            _mlat = round(_md['miss_lat'], 2)
                            _tf = round(_tfall, 2)
                except Exception as _e:
                    # ★ 不再靜默吞掉: 只印第一次, 避免洗版
                    if not _miss_warned:
                        _miss_warned = True
                        _w = f"★ 逐幀 miss 計算失敗: {type(_e).__name__}: {_e}"
                        print("\n" + _w)
                        event(_w)
                row = {
                    't': round(t, 3),
                    'gps_unix': (f"{s['gps_unix']:.3f}" if s['gps_unix'] is not None else ''),
                    'frame_idx': frame_idx,
                    'mode': out['mode'], 'guided': int(s['guided']),
                    'ac_n': round(ac_n, 2), 'ac_e': round(ac_e, 2),
                    'agl': round(agl, 2) if agl is not None else '',
                    'V': round(V, 2),
                    'chi_deg': round(hdg, 2), 'gamma_deg': round(gam, 2),
                    'theta_deg': round(pitch, 2),
                    'roll_deg': round(s['roll_deg'], 2) if s['roll_deg'] is not None else '',
                    'yaw_deg': round(hdg, 1),
                    'vn': round(s['vn'], 2) if s['vn'] is not None else '',
                    've': round(s['ve'], 2) if s['ve'] is not None else '',
                    'vd': round(s['vd'], 2) if s['vd'] is not None else '',
                    'dt': round(dt_loop, 4),
                    'tgt_n': round(vfields['tgt_n'], 2),
                    'tgt_e': round(vfields['tgt_e'], 2),
                    'slant_m': round(vfields['slant_m'], 1) if vfields['slant_m'] is not None else '',
                    'ground_m': round(vfields['ground_m'], 1) if vfields['ground_m'] is not None else '',
                    'predive_dist_m': round(vfields['predive_dist_m'], 1),
                    'virt_bbox_u': round(vu, 1) if vu is not None else '',
                    'virt_bbox_v': round(vv, 1) if vv is not None else '',
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
                    'miss_pred_m': _mp,
                    'miss_pred_lon_m': _mlon,
                    'miss_pred_lat_m': _mlat,
                    'impact_t_fall_s': _tf,
                    'img_err_u_pct': (round((vu - cx) / half_w * 100, 2)
                                      if vu is not None else ''),
                    'img_err_v_pct': (round((vv - cy) / half_h * 100, 2)
                                      if vv is not None else ''),
                    'glide_slope_err_deg': (round(gam + cfg.DIVE_ANGLE_DEG, 2)
                                            if out['mode'] == 'DIVE' else ''),
                    'p_dps': round(s['p_dps'], 1) if s['p_dps'] is not None else '',
                    'q_dps': round(s['q_dps'], 1) if s['q_dps'] is not None else '',
                    'r_dps': round(s['r_dps'], 1) if s['r_dps'] is not None else '',
                    'nz_g': round(s['nz_g'], 2) if s['nz_g'] is not None else '',
                    'fc_pitch_tgt_deg': round(s['fc_pitch_tgt_deg'], 2) if s['fc_pitch_tgt_deg'] is not None else '',
                    'fc_nav_pitch_deg': round(s['fc_nav_pitch_deg'], 2) if s['fc_nav_pitch_deg'] is not None else '',
                    'fc_nav_roll_deg': round(s['fc_nav_roll_deg'], 2) if s['fc_nav_roll_deg'] is not None else '',
                    'fc_nav_bearing_deg': round(s['fc_nav_bearing_deg'], 1) if s['fc_nav_bearing_deg'] is not None else '',
                    'fc_mode': s['fc_mode'],
                    'batt_v': round(s['batt_v'], 2) if s['batt_v'] is not None else '',
                    **{f'rcout{i+1}': (s['rcout'][i] if s['rcout'][i] is not None else '')
                       for i in range(4)},
                    'lat': s['lat'], 'lon': s['lon'],
                    'vis_age_ms': round(_vis_age, 2) if _vis_age is not None else '',
                    'vis_new': _vis_new,
                    'ctrl_ms': round(_ctrl_ms, 2),
                    'img_to_cmd_ms': (round(_i2c + _cap_lat_ms, 2)
                                      if _i2c is not None else ''),
                }
                row.update(_real_pid_row)
                row.update(_ray_row)
                row['dive_command_source'] = ('real_camera_compensated'
                    if out['mode'] == 'DIVE' else '')
                mission_csv.write(0, row)

                # ══════════════════════════════════════════════════
                #  健康看門狗 (每 2s) — 2026/09/08 實飛的教訓:
                #  視覺執行緒在 754.7s 靜默死亡, 主 log 卻跑到 1075.6s,
                #  飛完才發現影片和視覺資料缺了最後 5 分 21 秒.
                #  現在任何一條資料鏈斷掉都會【當場】在 console 跳出來.
                # ══════════════════════════════════════════════════
                if t - _vid_check_t >= 2.0:
                    _vid_check_t = t
                    # (a) 視覺執行緒: thread 死了, 或活著但卡住 (心跳過期)
                    if 'vision' not in _vid_warned:
                        _hb_age = time.time() - vision_hb.get('t', 0)
                        _dead = not vision_thread.is_alive()
                        _fatal = vision_hb.get('fatal')
                        if _dead or _fatal or _hb_age > 2.0:
                            _vid_warned.add('vision')
                            _why = (_fatal or ("執行緒已結束" if _dead
                                    else f"心跳停了 {_hb_age:.1f}s (可能卡在 YOLO/IO)"))
                            _w = (f"★★★ 視覺資料鏈中斷 @ t={t:.1f}s "
                                  f"(frame_idx={vision_hb.get('frame')}): {_why}"
                                  + (f" | 最後錯誤: {vision_hb.get('last_err')}"
                                     if vision_hb.get('last_err') else "")
                                  + "  ← 控制與 raw 錄影不受影響")
                            print("\n" + "!" * 64 + "\n" + _w + "\n" + "!" * 64)
                            event(_w)
                    # (b) 相機取像執行緒 (raw 錄影的命脈)
                    if 'camera' not in _vid_warned:
                        _gs = grabber.stats()
                        if _gs['age'] is not None and _gs['age'] > 3.0:
                            _vid_warned.add('camera')
                            _w = (f"★★★ 相機停止供幀 @ t={t:.1f}s "
                                  f"(已 {_gs['age']:.1f}s, 連續失敗 "
                                  f"{_gs['read_fail']}, 重開 {_gs['reopens']} 次)")
                            print("\n" + _w)
                            event(_w)
                    # (c) 兩支影片的寫入器
                    for _nm, _vw in (('annot', video_annot), ('raw', video_raw)):
                        if _vw is None or _nm in _vid_warned:
                            continue
                        _hok, _hmsg = _vw.health()
                        if not _hok:
                            _vid_warned.add(_nm)
                            _w = f"★★★ 錄影 [{_nm}] 已停止: {_hmsg}"
                            print("\n" + _w)
                            event(_w)

                if t - last_print >= 1.0:
                    last_print = t
                    _np = s['fc_nav_pitch_deg']
                    _nr = s['fc_nav_roll_deg']
                    _fc = (f" | FC nav_pitch={_np:+6.1f} nav_roll={_nr:+6.1f}"
                           if _np is not None else " | FC nav_* 無資料")
                    print(f"[{out['mode']:8s}] t={t:6.1f} AGL={agl or 0:6.1f} "
                          f"V={V:5.1f} pd_dist={vfields['predive_dist_m']:6.0f} "
                          f"det={int(vst.detected) if vst else 0} "
                          f"pitch={out['pitch_cmd_deg']:+6.1f} "
                          f"hdg={out['heading_cmd_deg']:6.1f} g={int(s['guided'])}"
                          + _fc)

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
                              f"(lon {md['miss_lon']:+.2f}, lat {md['miss_lat']:+.2f})")

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
        event(f"程式結束 mode={sm.mode} errors={err_count}")
        _sync_run['on'] = False
        flags['quit'] = True
        mission_csv.end()
        shared.reset_mission(mission_csv.generation)
        vision_thread.join(timeout=2.0)
        try:
            grabber.stop()
        except Exception:
            pass
        if video_annot is not None:
            video_annot.release()
        if video_raw is not None:
            video_raw.release()
        gimbal.close()
        mission_csv.end()

        # ══════════════════════════════════════════════════════
        #  ★ 錄影 / 資料鏈總結 — 飛完【立刻】知道資料完不完整,
        #    不用等回家對 CSV 才發現少了 5 分鐘.
        # ══════════════════════════════════════════════════════
        try:
            _dur = time.time() - t0
            _gs = grabber.stats()
            _vok = (vision_hb.get('fatal') is None
                    and vision_hb.get('errors', 0) == 0
                    and 'vision' not in _vid_warned)
            L = ["", "=" * 58, "            錄影 / 資料鏈 總結", "=" * 58,
                 f"  任務時長          : {_dur:.1f} s "
                 f"({int(_dur // 60)}:{_dur % 60:04.1f})",
                 f"  相機擷取幀數      : {_gs['captured']}"
                 + (f"  (重開 {_gs['reopens']} 次)" if _gs['reopens'] else ""),
                 f"  視覺處理幀數      : {vision_hb.get('frame', -1) + 1}"
                 f"   平均 {(vision_hb.get('frame', 0) + 1) / max(_dur, 1e-6):.1f} fps",
                 f"  等待新幀次數      : {vision_hb.get('dup', 0)}"
                 f"   (相機比視覺迴圈慢時會出現, 正常)"]
            if video_raw is not None and video_raw.written > 1 and \
                    video_raw.t_first and video_raw.t_last:
                _cs = video_raw.t_last - video_raw.t_first
                if _cs > 1.0:
                    _cfps = video_raw.written / _cs
                    L.append(f"  相機實際 fps      : {_cfps:.1f}"
                             f"   (config CAM_FPS={cfg.CAM_FPS})")
                    if _cfps < 0.8 * cfg.CAM_FPS:
                        L.append(f"                      ★ 遠低於設定值 — "
                                 f"raw 影片時基會差 "
                                 f"{(cfg.CAM_FPS / _cfps - 1) * 100:.0f}%")
            for _nm, _vw in (('raw  ', video_raw), ('annot', video_annot)):
                if _vw is None:
                    L.append(f"  {_nm} 影片          : 未啟動")
                else:
                    _bk = _vw.enqueued - _vw.written - _vw.dropped
                    L.append(f"  {_nm} 影片          : 入列 {_vw.enqueued}, "
                             f"寫入 {_vw.written}, 丟棄 {_vw.dropped}, "
                             f"殘留 {_bk}"
                             + (f", 第 {_vw.died_at_t:.1f}s 停止"
                                if _vw.died_at_t is not None else "")
                             + ("  ★ 有錯誤 (見上)" if _vw.error else "  OK"))
                    if _nm == 'raw  ' and _vw.first_frame_id is not None:
                        L.append(f"                      MP4 第 0 幀 = 相機第 "
                                 f"{_vw.first_frame_id} 幀 "
                                 f"(後處理 raw_frame_idx = MP4幀號 + "
                                 f"{_vw.first_frame_id})")
                    if _vw.enqueued and _vw.written < 0.95 * _vw.enqueued:
                        L.append(f"                      ★ 只寫出 "
                                 f"{100.0 * _vw.written / _vw.enqueued:.0f}% "
                                 f"— 編碼器跟不上或中途死亡")
            if _vok:
                L.append("  視覺執行緒        : OK")
            else:
                L.append(f"  視覺執行緒        : ★★★ 異常 — "
                         f"最後 frame_idx={vision_hb.get('frame')}, "
                         f"例外 {vision_hb.get('errors', 0)} 次")
                if vision_hb.get('fatal'):
                    L.append(f"                      {vision_hb['fatal']}")
                if vision_hb.get('last_err'):
                    L.append(f"                      最後錯誤: "
                             f"{vision_hb['last_err']}")
                L.append(f"                      traceback → {err_log_path}")
            L += ["=" * 58, ""]
            _sum = "\n".join(L)
            print(_sum)
            for _ln in L:
                if _ln.strip():
                    event(_ln.strip())
        except Exception as _e:
            print(f"[flight] 總結產生失敗: {_e}")

        fev.close()
        for main_path, vision_path in mission_csv.saved_pairs:
            print(f"[flight] log 已存 {main_path} + {vision_path}")
            print(f"[flight] 對應影片/events 共用 stem={stem}; CSV 的 recording_stem 欄位可對照")


if __name__ == '__main__':
    main()
