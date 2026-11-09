# -*- coding: utf-8 -*-
"""
vision_guidance.py — 對方雲台 UART 封包 → 狀態機的「目標」輸入
═══════════════════════════════════════════════════════════════════
做法: 原專案 (xwing_virtual) 的狀態機 / 判斷邏輯【完全不改】,
只把目標輸入的來源從「虛擬軌跡」換成「目標偵測」.
VisionSource.get_fields() 輸出的欄位與 virtual_source.VirtualSource.get_fields()
同名同義, 直接餵進 dive_guidance.MissionStateMachine:

    tgt_n, tgt_e      目標地面位置 (NED)      → SEARCH / PRE_DIVE 航向 = 飛機指向它
    tgt_valid         這拍看得到目標           → PRE_DIVE / DIVE 目標遺失中止 (原邏輯)
    ground_m, slant_m, predive_dist_m          → 推頭: predive_dist_m ≤ 415.69 m (原邏輯)
    bbox_detected, bbox_u, bbox_v,             → DIVE PID / 偏離中止 (原邏輯)
    bbox_center_offset_norm

對方送來 (xwing_uart_vision.UartVisionLink.latest() 的 dict):
    target_cx/cy (無目標 = NaN), fx/fy/cx/cy, pitch/roll (對地, 取像當下),
    mode, result_id, cap_mono (v7/v8 校時後換到本機 monotonic 的取像時刻)

相機模型 (2026/11 與對方確認):
    · pitch = 對地角度 (往下為負), 是【拍照那一刻】的值, 飛機姿態不影響
    · roll  = 相機 roll, 雲台維持 0°
    · 沒有 yaw 軸: 相機永遠朝機頭 → 相機方位 = 飛機 yaw (+ 安裝偏差)
  與 gimbal_pid_compensation.camera_matrix 的 'earth_pitch_roll_body_yaw'
  同一條旋轉鏈 (雲台 yaw 固定 0).

  ★ 2026/11/09 共用接收端 V6: 封包多了雲台 yaw 欄位. 怎麼用由 GIMBAL_YAW_MODE 決定:
    'none'  (預設) 不用 yaw, 相機方位 = 飛機 yaw  (= 上面確認的「沒有 yaw 軸」)
    'body'  yaw = 相機相對機頭的角度 (右為正) → 相機方位 = 飛機 yaw + yaw
    'earth' yaw = 相機的真方位 (對地)         → 相機方位 = yaw
  yaw 是 NaN (舊格式 / 對方沒給) 時一律退回 'none'.
"""

import math

import numpy as np

from dive_guidance import R_G2C, rot_y_np, rot_z_np, wrap_deg
from gimbal_pid_compensation import reference_matrix, rx


def _finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def camera_from_ned(pitch_deg, roll_deg, yaw_deg):
    """NED → 相機 (CV: X 右 Y 下 Z 前). pitch/roll 為對地角, yaw 為相機真方位."""
    return (R_G2C @ rot_y_np(math.radians(pitch_deg)) @ rx(roll_deg)
            @ rot_z_np(math.radians(yaw_deg)))


def camera_yaw(pkt, ac_yaw, cfg):
    """相機真方位 (度). 依 GIMBAL_YAW_MODE 決定要不要用封包的雲台 yaw."""
    mode = getattr(cfg, 'GIMBAL_YAW_MODE', 'none')
    gy = pkt.get('yaw')
    off = cfg.GIMBAL_YAW_OFFSET_DEG
    if mode == 'none' or not _finite(gy):
        return ac_yaw + off
    gy = getattr(cfg, 'GIMBAL_YAW_SIGN', 1.0) * gy
    if mode == 'body':
        return ac_yaw + gy + off
    if mode == 'earth':
        return gy + off
    raise ValueError(f"GIMBAL_YAW_MODE 只能是 'none' / 'body' / 'earth', 收到 {mode!r}")


def measure(pkt, pose_angles, agl, cfg, ctrl):
    """一筆有目標的封包 → 幾何量 dict; 射線沒朝下就回 None.
    pose_angles = (roll, pitch, yaw, course) 取像時刻的飛機姿態 (度).
    ctrl = (FX, FY, CX, CY, tilt_deg) 控制 (虛擬) 相機."""
    fx, fy, cx, cy = pkt['fx'], pkt['fy'], pkt['cx'], pkt['cy']
    ray_cam = np.array([(pkt['target_cx'] - cx) / fx,
                        (pkt['target_cy'] - cy) / fy, 1.0])
    _roll, ac_pitch, ac_yaw, course = pose_angles
    p = cfg.GIMBAL_PITCH_SIGN * pkt['pitch'] + cfg.GIMBAL_PITCH_OFFSET_DEG
    r = cfg.GIMBAL_ROLL_SIGN * pkt['roll'] + cfg.GIMBAL_ROLL_OFFSET_DEG
    cam_yaw = camera_yaw(pkt, ac_yaw, cfg)
    ray = camera_from_ned(p, r, cam_yaw).T @ ray_cam
    n, e, d = ray / np.linalg.norm(ray)
    horiz = math.hypot(n, e)
    if d <= 1e-3 or agl is None or agl <= 0.5:
        return None
    bearing = math.degrees(math.atan2(e, n)) % 360.0
    out = {
        'bearing_deg': bearing,
        'bearing_off_deg': wrap_deg(bearing - ac_yaw),
        'depress_deg': math.degrees(math.atan2(d, horiz)),
        'ground_m': agl * horiz / d,
        'slant_m': agl / d,
        'rel_n': agl * n / d, 'rel_e': agl * e / d,
        'comp_u': None, 'comp_v': None,
    }
    # DIVE: 換到「機身 pitch + 地速方向」的控制相機 (同 gimbal_pid_compensation.transform_pixel),
    #   等於原專案虛擬目標在 DIVE 段的 body-fixed 投影.
    FX, FY, CX, CY, tilt = ctrl
    c = reference_matrix(ac_pitch, course, tilt) @ ray
    if np.isfinite(c).all() and c[2] > 1e-6:
        out['comp_u'] = CX + FX * c[0] / c[2]
        out['comp_v'] = CY + FY * c[1] / c[2]
    return out


VG_CSV_FIELDS = [
    'vg_new',            # 1 = 這拍拿到新的辨識結果
    'vg_valid',          # 1 = 這拍看得到目標 (= 餵給狀態機的 tgt_valid)
    'vg_reason',         # 看不到的原因 (no_target / no_pose / ray_up / stale / no_data)
    'vg_time_src',       # 取像時刻來源: capture (v7/v8 校時) / rx (沒校時, 用收到時刻)
    'vg_pose_method', 'vg_pose_gap_ms',
    'vg_bearing_deg', 'vg_bearing_off_deg', 'vg_depress_deg',
    'vg_ground_m', 'vg_slant_m',
    'vg_tgt_n', 'vg_tgt_e', 'vg_tgt_lat', 'vg_tgt_lon',
    'gcomp_u', 'gcomp_v',
]


class VisionSource:
    """虛擬目標 (virtual_source.VirtualSource) 的替代品: 目標來自對方雲台偵測.
    每個控制拍呼叫 get_fields(); 只在 result_id 改變時處理新的一筆."""

    def __init__(self, cfg, ctrl):
        self.cfg = cfg
        self.ctrl = ctrl
        self.reset()

    def reset(self):
        """新的一趟任務: 忘掉上一趟的目標."""
        self._last_rid = None
        self._cur = None               # 最新一筆辨識結果的量測 (看不到 = None)
        self._reason = 'no_data'
        self._last = None              # 最後一次看到目標的量測 (航向用, 跟丟時保持)

    @property
    def last_valid(self):
        return self._last

    def _process(self, pkt, pose_history, agl):
        """一筆新結果 → 幾何量 dict, 或看不到的原因字串."""
        cfg = self.cfg
        if pkt.get('cap_mono') is not None:
            tcap, src = pkt['cap_mono'], 'capture'
        else:
            tcap, src = pkt['rx_mono'], 'rx'
        if not pkt.get('valid'):
            return 'no_target'
        pose = pose_history.at(tcap, cfg.POSE_MAX_GAP_S)
        if pose is None:
            return 'no_pose'
        m = measure(pkt, pose['angles'], agl, cfg, self.ctrl)
        if m is None:
            return 'ray_up'
        m.update(tcap=tcap, time_src=src, pose_method=pose['method'],
                 pose_gap_ms=pose['distance_s'] * 1000.0)
        return m

    def get_fields(self, pkt, now_mono, pose_history, agl, ac_n, ac_e, origin=None):
        """回傳 (狀態機欄位 dict, log 欄位 dict).
        狀態機欄位同 VirtualSource.get_fields; 從沒看過目標時 tgt_n/tgt_e = None
        (由主程式決定帶路方向)."""
        cfg = self.cfg
        new = False
        if pkt is None:
            self._cur, self._reason = None, 'no_data'
        elif now_mono - pkt['rx_mono'] > cfg.VISION_PKT_MAX_AGE_S:
            self._cur, self._reason = None, 'stale'      # 資料鏈沒有新封包
        elif pkt['result_id'] != self._last_rid:
            new = True
            self._last_rid = pkt['result_id']
            m = self._process(pkt, pose_history, agl)
            if isinstance(m, dict):
                # 目標地面位置 (NED): 收到這筆時的飛機位置 + 射線落地點
                m['tgt_n'], m['tgt_e'] = ac_n + m['rel_n'], ac_e + m['rel_e']
                self._cur = self._last = m
                self._reason = ''
            else:
                self._cur, self._reason = None, m

        cur, last = self._cur, self._last
        FX, FY, CX, CY, _tilt = self.ctrl
        det = cur is not None and cur['comp_u'] is not None
        fields = {
            'tgt_n': last['tgt_n'] if last else None,
            'tgt_e': last['tgt_e'] if last else None,
            'tgt_valid': cur is not None,
            'tgt_vn': 0.0, 'tgt_ve': 0.0,          # 車速未知 (前置瞄準本來就關)
            'slant_m': cur['slant_m'] if cur else None,
            'ground_m': cur['ground_m'] if cur else None,
            'predive_dist_m': cur['ground_m'] if cur else None,
            'bbox_detected': det,
            'bbox_u': cur['comp_u'] if det else None,
            'bbox_v': cur['comp_v'] if det else None,
            'bbox_w': None, 'bbox_h': None,
            'bbox_center_offset_norm': (math.hypot(cur['comp_u'] - CX, cur['comp_v'] - CY)
                                        / math.hypot(CX, CY) if det else None),
        }

        def r(x, nd=2):
            return round(x, nd) if _finite(x) else ''

        row = dict.fromkeys(VG_CSV_FIELDS, '')
        row.update(vg_new=int(new), vg_valid=int(cur is not None), vg_reason=self._reason)
        if cur is not None:
            row.update(vg_time_src=cur['time_src'], vg_pose_method=cur['pose_method'],
                       vg_pose_gap_ms=r(cur['pose_gap_ms'], 1),
                       vg_bearing_deg=r(cur['bearing_deg']),
                       vg_bearing_off_deg=r(cur['bearing_off_deg']),
                       vg_depress_deg=r(cur['depress_deg']),
                       vg_ground_m=r(cur['ground_m'], 1),
                       vg_slant_m=r(cur['slant_m'], 1),
                       gcomp_u=r(cur['comp_u'], 1), gcomp_v=r(cur['comp_v'], 1))
        if last is not None:
            tn, te = last['tgt_n'], last['tgt_e']
            row.update(vg_tgt_n=r(tn, 1), vg_tgt_e=r(te, 1))
            if origin is not None:
                o_lat, o_lon = origin
                row['vg_tgt_lat'] = round(o_lat + math.degrees(tn / 6378137.0), 7)
                row['vg_tgt_lon'] = round(o_lon + math.degrees(
                    te / (6378137.0 * math.cos(math.radians(o_lat)))), 7)
        return fields, row
