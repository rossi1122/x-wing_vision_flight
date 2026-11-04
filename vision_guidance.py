# -*- coding: utf-8 -*-
"""
vision_guidance.py — 對方雲台 UART 封包 → 全視覺導引需要的量
═══════════════════════════════════════════════════════════════════
對方送來 (xwing_uart_vision.UartVisionLink.latest() 的 dict):
    target_cx/cy (無目標 = NaN), fx/fy/cx/cy, pitch/roll (對地, 取像當下),
    mode, result_id, cap_mono (v7/v8 校時後換到本機 monotonic 的取像時刻)

相機模型 (2026/11 與對方確認):
    · pitch = 對地角度 (往下為負), 是【拍照那一刻】的值, 飛機姿態不影響
    · roll  = 相機 roll, 雲台維持 0°
    · 沒有 yaw 軸: 相機永遠朝機頭 → 相機方位 = 飛機 yaw (+ 安裝偏差)
  與 gimbal_pid_compensation.camera_matrix 的 'earth_pitch_roll_body_yaw'
  同一條旋轉鏈 (雲台 yaw 固定 0), 所以 DIVE 補償公式與原專案完全相同.

每筆新的辨識結果算出:
    方位      bearing_deg      目標在地面上的真方位 (SEARCH / PRE_DIVE 航向命令)
    偏機頭    bearing_off_deg  目標偏離機頭幾度 (右正)
    水平距    ground_m         高度 × 水平分量 / 垂直分量 (平地假設)
    補償像素  comp_u/comp_v    換到「機身 pitch + 地速方向」虛擬相機的像素 (DIVE PID 用)
"""

import math
from collections import deque

import numpy as np

from dive_guidance import R_G2C, rot_y_np, rot_z_np, wrap_deg
from gimbal_pid_compensation import reference_matrix, rx


def _finite(x):
    return x is not None and isinstance(x, (int, float)) and math.isfinite(x)


def camera_from_ned(pitch_deg, roll_deg, yaw_deg):
    """NED → 相機 (CV: X 右 Y 下 Z 前). pitch/roll 為對地角, yaw 為相機真方位."""
    return (R_G2C @ rot_y_np(math.radians(pitch_deg)) @ rx(roll_deg)
            @ rot_z_np(math.radians(yaw_deg)))


def measure(pkt, pose_angles, agl, cfg, ctrl):
    """一筆有目標的封包 → 幾何量 dict; 射線沒朝下就回 None.
    pose_angles = (roll, pitch, yaw, course) 取像時刻的飛機姿態 (度).
    ctrl = (FX, FY, CX, CY, tilt_deg) 虛擬 (控制) 相機."""
    fx, fy, cx, cy = pkt['fx'], pkt['fy'], pkt['cx'], pkt['cy']
    ray_cam = np.array([(pkt['target_cx'] - cx) / fx,
                        (pkt['target_cy'] - cy) / fy, 1.0])
    _roll, ac_pitch, ac_yaw, course = pose_angles
    p = cfg.GIMBAL_PITCH_SIGN * pkt['pitch'] + cfg.GIMBAL_PITCH_OFFSET_DEG
    r = cfg.GIMBAL_ROLL_SIGN * pkt['roll'] + cfg.GIMBAL_ROLL_OFFSET_DEG
    cam_yaw = ac_yaw + cfg.GIMBAL_YAW_OFFSET_DEG
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
    # DIVE 補償: 同 gimbal_pid_compensation.transform_pixel
    FX, FY, CX, CY, tilt = ctrl
    c = reference_matrix(ac_pitch, course, tilt) @ ray
    if np.isfinite(c).all() and c[2] > 1e-6:
        out['comp_u'] = CX + FX * c[0] / c[2]
        out['comp_v'] = CY + FY * c[1] / c[2]
    return out


VG_CSV_FIELDS = [
    'vg_new',            # 1 = 這拍拿到新的辨識結果
    'vg_valid',          # 最新這筆有沒有有效目標 (有目標 + 姿態對得上 + 射線朝下)
    'vg_reason',         # 無效原因 (no_target / no_pose / ray_up / stale)
    'vg_hits',           # 最近 VISION_ACQUIRE_WINDOW 筆新結果裡有幾筆有效
    'vg_age_ms',         # 距最後一筆有效偵測的取像時刻 (ms)
    'vg_time_src',       # 取像時刻來源: capture (v7/v8 校時) / rx (沒校時, 用收到時刻)
    'vg_pose_method', 'vg_pose_gap_ms',
    'vg_bearing_deg', 'vg_bearing_off_deg', 'vg_depress_deg',
    'vg_ground_m', 'vg_ground_med_m', 'vg_slant_m',
    'vg_tgt_n', 'vg_tgt_e', 'vg_tgt_lat', 'vg_tgt_lon',
    'gcomp_u', 'gcomp_v', 'real_detected',
]


class VisionTracker:
    """每個控制拍呼叫 update(); 只在 result_id 改變時處理新的一筆."""

    def __init__(self, cfg, ctrl):
        self.cfg = cfg
        self.ctrl = ctrl
        self.reset()

    def reset(self):
        """新的一趟任務: 清掉命中視窗、最後有效值、距離中位數視窗."""
        self._last_rid = None
        # 最近 VISION_ACQUIRE_WINDOW 筆新結果有沒有看到車 (漏一兩筆不會從頭算)
        self._recent = deque(maxlen=max(1, int(self.cfg.VISION_ACQUIRE_WINDOW)))
        self._last = None              # 最後一筆有效量測 (dict, 含 tcap)
        self._last_reason = ''
        self._ground = deque(maxlen=max(1, int(self.cfg.VISION_GROUND_MEDIAN_N)))

    @property
    def last_valid(self):
        return self._last

    @property
    def hits(self):
        """最近 VISION_ACQUIRE_WINDOW 筆新結果裡有幾筆有效."""
        return sum(self._recent)

    def _process(self, pkt, pose_history, agl):
        """新的一筆 → 幾何量 dict, 或無效原因字串."""
        cfg = self.cfg
        if pkt.get('cap_mono') is not None:
            tcap, src = pkt['cap_mono'], 'capture'
        else:
            tcap, src = pkt['rx_mono'], 'rx'
        if not pkt.get('valid'):
            return tcap, src, None, 'no_target'
        pose = pose_history.at(tcap, cfg.POSE_MAX_GAP_S)
        if pose is None:
            return tcap, src, None, 'no_pose'
        m = measure(pkt, pose['angles'], agl, cfg, self.ctrl)
        if m is None:
            return tcap, src, pose, 'ray_up'
        m.update(tcap=tcap, time_src=src, pose_method=pose['method'],
                 pose_gap_ms=pose['distance_s'] * 1000.0)
        return tcap, src, pose, m

    def update(self, pkt, now_mono, t, pose_history, agl, ac_n, ac_e,
               origin=None):
        """回傳 (inp 欄位 dict, log 欄位 dict)."""
        cfg = self.cfg
        new = False
        if pkt is not None and now_mono - pkt['rx_mono'] <= cfg.VISION_STALE_S:
            if pkt['result_id'] != self._last_rid:
                new = True
                self._last_rid = pkt['result_id']
                _tcap, _src, _pose, m = self._process(pkt, pose_history, agl)
                if isinstance(m, dict):
                    # 目標地面位置 (NED): 收到這筆時的飛機位置 + 射線落地點
                    m['tgt_n'], m['tgt_e'] = ac_n + m['rel_n'], ac_e + m['rel_e']
                    if (self._last is None
                            or m['tcap'] - self._last['tcap'] > cfg.VISION_MEAS_MAX_AGE_S):
                        self._ground.clear()     # 中斷過: 不跟舊距離取中位數
                    self._ground.append(m['ground_m'])
                    self._recent.append(1)
                    self._last = m
                    self._last_reason = ''
                else:
                    self._recent.append(0)
                    self._last_reason = m
        elif pkt is not None:
            self._last_reason = 'stale'

        last = self._last
        age = (now_mono - last['tcap']) if last is not None else None
        fresh = age is not None and age <= cfg.VISION_MEAS_MAX_AGE_S
        med = float(np.median(self._ground)) if (fresh and self._ground) else None
        comp_ok = fresh and last['comp_u'] is not None
        FX, FY, CX, CY, _tilt = self.ctrl
        half_diag = math.hypot(CX, CY)
        inp = {
            'vis_age_s': age,
            'vis_hits': self.hits,
            'vis_bearing_deg': last['bearing_deg'] if last else None,
            'vis_bearing_off_deg': last['bearing_off_deg'] if last else None,
            'vis_ground_med_m': med,
            # DIVE 段 (dive_guidance DIVE_USE_REAL_CAMERA 路徑)
            'real_control_detected': bool(comp_ok),
            'real_control_u': last['comp_u'] if comp_ok else None,
            'real_control_v': last['comp_v'] if comp_ok else None,
            'real_control_cap_t': (t - (now_mono - last['tcap'])) if last else None,
            'real_control_offset_norm': (math.hypot(last['comp_u'] - CX,
                                                    last['comp_v'] - CY) / half_diag
                                         if comp_ok else None),
        }

        def r(x, nd=2):
            return round(x, nd) if _finite(x) else ''

        row = dict.fromkeys(VG_CSV_FIELDS, '')
        row.update(vg_new=int(new), vg_hits=self.hits,
                   vg_valid=int(new and self._last_reason == '') if new else '',
                   vg_reason=self._last_reason,
                   vg_age_ms=r(age * 1000.0 if age is not None else None, 1),
                   real_detected=int(comp_ok))
        if last is not None:
            tn, te = last['tgt_n'], last['tgt_e']
            row.update(vg_time_src=last['time_src'],
                       vg_pose_method=last['pose_method'],
                       vg_pose_gap_ms=r(last['pose_gap_ms'], 1),
                       vg_bearing_deg=r(last['bearing_deg']),
                       vg_bearing_off_deg=r(last['bearing_off_deg']),
                       vg_depress_deg=r(last['depress_deg']),
                       vg_ground_m=r(last['ground_m'], 1),
                       vg_ground_med_m=r(med, 1),
                       vg_slant_m=r(last['slant_m'], 1),
                       vg_tgt_n=r(tn, 1), vg_tgt_e=r(te, 1),
                       gcomp_u=r(last['comp_u'], 1), gcomp_v=r(last['comp_v'], 1))
            if origin is not None:
                o_lat, o_lon = origin
                row['vg_tgt_lat'] = round(o_lat + math.degrees(tn / 6378137.0), 7)
                row['vg_tgt_lon'] = round(o_lon + math.degrees(
                    te / (6378137.0 * math.cos(math.radians(o_lat)))), 7)
        return inp, row
