# -*- coding: utf-8 -*-
"""
vision_sim_camera.py — 桌上測試用: 模擬「真車」與「對方雲台相機 + 辨識」
═══════════════════════════════════════════════════════════════════
產生與 xwing_uart_vision.UartVisionLink.latest() 相同格式的封包, 讓
xwing_vision_sim.py (閉迴路模擬) 和 test_vision_guidance.py 不接硬體就能
跑完整條全視覺流程.

雲台行為 (照對方 2026/11 說明):
  '1' 追蹤: pitch 自動轉, 讓車在畫面「上下」置中; 沒看到車 → 慢慢回到 −20°
  '2' 鎖定: pitch 轉到 −30° (對地) 並停住
  沒有 yaw 軸 (相機朝機頭), roll 維持 0°, 回報的 pitch 是拍照那一刻的對地角
辨識: 車在畫面內、距離 ≤ detect_range_m 就偵測得到 (可設隨機漏偵測與像素雜訊)
"""

import math
import random

import numpy as np

from vision_guidance import camera_from_ned

NAN = float('nan')


class SimCar:
    """真車: 沿 heading_deg 直線開. 位置 = 起點 + 依加速度 / 最高速積分."""

    def __init__(self, n0, e0, heading_deg, t_start=0.0, accel=None, vmax=10.0,
                 present=True):
        self.n0, self.e0 = n0, e0
        self.h = math.radians(heading_deg)
        self.t_start = t_start
        self.accel = accel            # None = 一出發就 vmax
        self.vmax = vmax
        self.present = present        # False = 根本沒有車 (辨識永遠看不到)

    def dist_speed(self, t):
        dt = max(0.0, t - self.t_start)
        if self.vmax <= 0:
            return 0.0, 0.0
        if not self.accel:
            return self.vmax * dt, self.vmax
        t_acc = self.vmax / self.accel
        if dt <= t_acc:
            return 0.5 * self.accel * dt * dt, self.accel * dt
        return 0.5 * self.accel * t_acc * t_acc + self.vmax * (dt - t_acc), self.vmax

    def now(self, t):
        s, v = self.dist_speed(t)
        return (self.n0 + s * math.cos(self.h), self.e0 + s * math.sin(self.h),
                v * math.cos(self.h), v * math.sin(self.h))


class SimGimbalCamera:
    def __init__(self, fx=1000.0, fy=1000.0, width=1920, height=1080, fps=30.0,
                 latency_s=0.08, detect_range_m=900.0, dropout=0.0, pix_noise=1.5,
                 track_rate_dps=60.0, lock_rate_dps=20.0, track_tau_s=0.25,
                 idle_pitch_deg=-20.0, lock_pitch_deg=-30.0,
                 pitch_bias_deg=0.0, yaw_bias_deg=0.0, seed=1):
        self.fx, self.fy = fx, fy
        self.w, self.h = width, height
        self.cx, self.cy = width / 2.0, height / 2.0
        self.period = 1.0 / fps
        self.latency = latency_s
        self.detect_range = detect_range_m
        self.dropout = dropout
        self.pix_noise = pix_noise
        self.track_rate = track_rate_dps
        self.lock_rate = lock_rate_dps
        self.track_tau = track_tau_s
        self.idle_pitch = idle_pitch_deg
        self.lock_pitch = lock_pitch_deg
        # 真實光軸 = 回報值 + pitch_bias (對應 config GIMBAL_PITCH_OFFSET_DEG)
        self.pitch_bias = pitch_bias_deg
        self.yaw_bias = yaw_bias_deg          # 光軸偏機頭右方 (對應 GIMBAL_YAW_OFFSET_DEG)
        self.rng = random.Random(seed)
        self.mode = 1
        self.pitch = idle_pitch_deg           # 真實光軸對地 pitch
        self._next_cap = float('-inf')        # 第一次 step 就取像
        self._rid = 0
        self._queue = []                      # (deliver_t, pkt)
        self._latest = None
        self.enabled = True                   # False = 資料鏈中斷 (不再送)

    # ── 與 UartVisionLink 相同的介面 ──
    def request_mode(self, key):
        if key is not None and str(key) in ('0', '1', '2'):
            self.mode = int(key)

    def latest(self):
        return dict(self._latest) if self._latest is not None else None

    # ------------------------------------------------------------------
    def _project(self, ac, car_n, car_e, pitch):
        """車在畫面上的像素 (u, v) 與斜距; 在相機後方回 None."""
        rel = np.array([car_n - ac['n'], car_e - ac['e'], ac['agl']])
        cam = camera_from_ned(pitch, 0.0, ac['yaw'] + self.yaw_bias) @ rel
        if cam[2] <= 1.0:
            return None
        return (self.cx + self.fx * cam[0] / cam[2],
                self.cy + self.fy * cam[1] / cam[2], float(np.linalg.norm(rel)))

    def _visible(self, pix):
        if pix is None:
            return False
        u, v, slant = pix
        return (0 <= u < self.w and 0 <= v < self.h and slant <= self.detect_range)

    def step(self, t, dt, ac, car):
        """ac: n, e, agl, yaw (機頭真方位). car: SimCar."""
        car_n, car_e, _, _ = car.now(t)
        # ── 雲台 pitch ──
        if self.mode == 2:
            target = self.lock_pitch
            rate = self.lock_rate
        else:
            pix = self._project(ac, car_n, car_e, self.pitch) if car.present else None
            if self.mode == 1 and self._visible(pix):
                rel_n, rel_e = car_n - ac['n'], car_e - ac['e']
                horiz = math.hypot(rel_n, rel_e)
                beta = math.atan2(rel_e, rel_n) - math.radians(ac['yaw'] + self.yaw_bias)
                f = max(1.0, horiz * math.cos(beta))
                want = -math.degrees(math.atan2(ac['agl'], f))
                target = self.pitch + (want - self.pitch) * min(1.0, dt / self.track_tau)
                rate = self.track_rate
            elif self.mode == 1:
                target, rate = self.idle_pitch, self.lock_rate
            else:
                target, rate = self.pitch, 0.0
        d = max(-rate * dt, min(rate * dt, target - self.pitch))
        self.pitch += d

        # ── 取像 (fps) → latency 後送到 ──
        if t >= self._next_cap:
            self._next_cap = t + self.period
            pix = self._project(ac, car_n, car_e, self.pitch) if car.present else None
            seen = self._visible(pix) and self.rng.random() >= self.dropout
            self._rid += 1
            if seen:
                u = pix[0] + self.rng.gauss(0.0, self.pix_noise)
                v = pix[1] + self.rng.gauss(0.0, self.pix_noise)
            else:
                u = v = NAN
            pkt = {
                'target_cx': u, 'target_cy': v,
                'offset_x': u - self.cx, 'offset_y': v - self.cy,
                'fx': self.fx, 'fy': self.fy, 'cx': self.cx, 'cy': self.cy,
                'image_width': float(self.w), 'image_height': float(self.h),
                'pitch': self.pitch - self.pitch_bias, 'roll': 0.0,
                'mode': self.mode, 'result_id': self._rid,
                'detection_age_ms': self.latency * 1000.0,
                'angle_age_ms': self.latency * 1000.0,
                'laser_distance_m': NAN, 'yaw': 0.0, 'ver': 10,
                'cap_mono': t, 'e2e_ms': self.latency * 1000.0, 'e2e_err_ms': 1.0,
                'valid': seen,
            }
            self._queue.append((t + self.latency, pkt))
        while self._queue and self._queue[0][0] <= t:
            deliver, pkt = self._queue.pop(0)
            if self.enabled:
                pkt['rx_mono'] = deliver
                self._latest = pkt
