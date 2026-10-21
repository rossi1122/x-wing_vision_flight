"""合併版模組；沿用 GUIDED 多趟初始化版本，控制與估測公式保持原樣。"""

import math
import numpy as np
from collections import deque
import threading
import time


# ========================================================================
# 原模組: vision_driver_car_dive_with_dkf_v3.py
# ========================================================================
# ════════════════════════════════════════════════════════════════════
#  旋轉矩陣工具 (與 gimbal_controller_pinhole.py 同款)
# ════════════════════════════════════════════════════════════════════

def _Rx(phi_deg):
    """繞 X 軸旋轉 (roll)"""
    phi = math.radians(phi_deg)
    c, s = math.cos(phi), math.sin(phi)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def _Ry(theta_deg):
    """繞 Y 軸旋轉 (pitch)"""
    theta = math.radians(theta_deg)
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])

def _Rz(psi_deg):
    """繞 Z 軸旋轉 (yaw)"""
    psi = math.radians(psi_deg)
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])


# 預設 Gimbal → Camera (CV convention: X 右, Y 下, Z 前) 旋轉矩陣
# 與 RayProjectionDistanceEstimator 一致
DEFAULT_R_G2C = np.array([
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 0],
], dtype=float)


# ════════════════════════════════════════════════════════════════════
#  (c) NED → Pixel 反投影工具
# ════════════════════════════════════════════════════════════════════

def reproject_target_ned_to_pixel(
    target_x_ned, target_y_ned,
    ac_x_ned, ac_y_ned, ac_altitude_agl,
    ac_roll_deg, ac_pitch_deg, ac_yaw_true_deg,
    gimbal_pitch_deg, gimbal_yaw_deg,
    fx, fy, cx, cy,
    R_g2c=None,
    gimbal_pitch_offset_deg=0.0,
    gimbal_yaw_offset_deg=0.0,   # SIYI pan 對齊真北的啟動偏移(度); 須與 estimate_distance 同值
    target_altitude_msl=0.0,
):
    """
    把 NED 座標下的目標位置反投影成相機 pixel 座標.

    這是 RayProjectionDistanceEstimator.estimate_distance() 的反向運算:
        forward:  pixel → ray_cam → ray_ned → ground intersect → slant_range
        inverse:  target_ned → vec_ned → vec_cam → pixel

    Args:
        target_x_ned, target_y_ned: 目標在 NED 的位置 (m), 通常從 controller.target_kf.x[0:2] 取
        ac_x_ned, ac_y_ned, ac_altitude_agl: 飛機 NED 位置 + AGL 高度 (m)
        ac_roll/pitch_deg, ac_yaw_true_deg: 飛機姿態 (deg, 真航向 0=北 90=東)
        gimbal_pitch/yaw_deg: 雲台角度 (deg, gimbal_pitch 負=向下看)
        fx, fy, cx, cy: 相機內參 (pixel)
        R_g2c: Gimbal → Camera CV 旋轉, None 用預設
        gimbal_pitch_offset_deg: 同 RayProjectionDistanceEstimator 的常數補償
        target_altitude_msl: 若目標不在地面 (e.g. 山坡上), 給 MSL 高度 (m)

    Returns:
        (u, v, valid):
            u, v: 預測 pixel 座標 (float, 不限定在影像內)
            valid: True 表示目標在相機前方 (Pc_z > 0); False 表示在後方/側面無法成像
    """
    if R_g2c is None:
        R_g2c = DEFAULT_R_G2C

    # 飛機在 NED 的 z 座標 (NED 中 down 為正): -altitude_agl - target_altitude_msl
    # 若目標不在地面, target 的 NED z = -target_altitude_msl
    # vec_ned = target_ned - aircraft_ned
    dx = target_x_ned - ac_x_ned          # north
    dy = target_y_ned - ac_y_ned          # east
    dz = ac_altitude_agl - target_altitude_msl  # down: target 比飛機低多少
    vec_ned = np.array([dx, dy, dz], dtype=float)

    # SIYI A8 三軸 world-stabilized: 相機世界指向只由雲台世界角決定,
    # 與飛機 roll/pitch 無關(雲台自行穩定補償) → 不乘 R_w2b。
    # 注意: vec_ned 的「平移(視差)」仍用飛機 NED 位置, 只有「朝向」拿掉飛機姿態。
    # 此慣例必須與 RayProjectionDistanceEstimator.estimate_distance() 一致, 正反向才閉合。
    gimbal_pitch_eff = gimbal_pitch_deg - gimbal_pitch_offset_deg
    gimbal_yaw_world = gimbal_yaw_deg + gimbal_yaw_offset_deg
    R_ned2gimbal = _Ry(gimbal_pitch_eff) @ _Rz(gimbal_yaw_world)
    R_ned2cam = R_g2c @ R_ned2gimbal

    # 目標在 camera 座標
    Pc = R_ned2cam @ vec_ned

    if Pc[2] <= 0.1:
        # 目標在相機後方 (或太近), 無法成像
        return None, None, False

    # Pinhole projection (CV convention)
    u = fx * Pc[0] / Pc[2] + cx
    v = fy * Pc[1] / Pc[2] + cy

    return float(u), float(v), True


# ════════════════════════════════════════════════════════════════════
#  (a) Ego-Aware DKF
# ════════════════════════════════════════════════════════════════════

class EgoAwareDKF:
    """
    Ego-aware 延遲卡爾曼濾波器 (V3).

    State: [u, v, u_dot, v_dot]  (pixel 座標 + 像素速度)
    重點: u_dot, v_dot 現在只代表「目標自身運動造成的殘差像素速度」,
          飛機/雲台運動造成的 pixel 漂移在 predict() 用 IBVS Jacobian 補償掉.

    新功能:
      1. predict(omega_body_radps, v_world_mps, R_w2b, R_b2g, depth_m)
         在 CV propagation 之外, 加上 ego motion 預測位移.
      2. update_fallback(z, r_std_pixel)
         給 NED 反投影量測用, R 由外部決定.

    保留:
      - update(z, delay_steps, bbox_size)
      - dynamic R / Mahalanobis gate / coasting mode (部分弱化, 因為 ego comp 已治本)
    """

    def __init__(self,
                 dt=0.01,
                 image_delay=0.08,
                 fx=1500.0, fy=1500.0, cx=960.0, cy=505.0,
                 enable_ego_comp=True,
                 max_history_steps=200,
                 max_update_delay_steps=30):
        """
        Args:
            ...
            max_history_steps: ✨ V3.3 history buffer 大小 (步數).
                必須 ≥ 預期最大延遲步數. 預設 200 步 = 2 秒, 對 100Hz dt=0.01 足夠.
            max_update_delay_steps: ✨ V3.3 update 補償深度上限 (步數).
                超過此值的延遲, update 只回去到此上限的位置做 KF update.
                預設 30 步 = 300ms. 太大 → forward propagation 放大 velocity correction 誤差.
                預設值是經驗值, dive scenario 大延遲應靠 NED fallback 補, 不靠 update 補.
        """
        self.dt = dt
        self.dt_nominal = dt      # ★ 初始 dt, delay replay 用 (固定步長)
        self.image_delay = image_delay
        self.default_delay_steps = max(1, int(image_delay / dt))
        self.max_update_delay_steps = max_update_delay_steps

        # 相機內參 (用於 IBVS Jacobian)
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.enable_ego_comp = enable_ego_comp

        # 狀態
        self.x = np.zeros(4)
        self.P = np.eye(4) * 100.0

        # CV propagation matrix (作用在 [u_dot, v_dot] 殘差速度上)
        self.F_base = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

        self.H = np.array([[1.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0]])

        # ✨ Q 大幅下調: 殘差速度 (target ground motion) 的不確定性
        #    地面車 ~10 m/s, 中遠距 ~10 px/s pixel 速度,
        #    accel 變化 ~5 px/s² 量級 → accel_noise=20 比 V2 的 150 合理得多
        accel_noise = 20.0  # px/s²
        self._accel_noise = accel_noise
        dt2 = dt * dt
        self.Q = (accel_noise ** 2) * np.array([
            [dt2*dt2/4, 0,         dt2*dt/2, 0        ],
            [0,         dt2*dt2/4, 0,        dt2*dt/2 ],
            [dt2*dt/2,  0,         dt2,      0        ],
            [0,         dt2*dt/2,  0,        dt2      ],
        ])

        # 量測噪聲 (動態 R, 同 V2)
        # ✨ V3.2: R_min_std 0.5 → 5.0 — 防止 DIVE 末段 bbox 大時 R 過小, gate 過嚴拒所有 YOLO
        self.R_base_std = 8.0
        self.R_bbox_ref = 32.0
        self.R_min_std = 5.0
        self.R_max_std = 25.0

        # ✨ Ego 補償的不確定性: 角速度量測誤差 + 深度估測誤差 → pixel 預測 noise
        # 經驗值: 角速度 1 deg/s 誤差 → fx * sin(1°) ≈ 26 px/s 預測誤差
        self.ego_uncertainty_std = 5.0  # px/s, 額外加到 Q 的速度項

        # ✨ V3.2: gate 50 → 200, 跟 fallback 對齊. 50 太緊, DIVE 末段 LoS 變化快會誤拒
        self.gate_threshold = 200.0
        self.gate_threshold_fallback = 200.0  # ✨ NED 反投影誤差大, gate 放寬

        max_history = max(self.default_delay_steps + 30, max_history_steps)
        self._history = deque(maxlen=max_history)
        self._max_history_size = max_history

        self.initialized = False
        self._predict_count = 0
        self._update_count = 0
        self._update_fallback_count = 0
        self._reject_count = 0

        # Coasting mode (削弱版: ego comp 已大幅減少 ego-induced drift, coast 不需要那麼激進)
        self._consecutive_miss = 0
        # ✨ 抗漂移調整 (依 log 實測: coast 期間殘差速度被外插造成框飄):
        #   coast 提早啟動 + 衰減加快, 讓「沒偵測時」殘差速度快速歸零,
        #   停止的目標其框不再滑走。只影響無偵測(coast)行為, 不動有偵測時的追蹤。
        self.coast_start = 5             # was 25; 多數 coast<25幀, 原值幾乎不啟動
        self.coast_q_inflation = 1.5
        self.coast_q_rate = 0.03
        self.coast_vel_decay = 0.88      # was 0.999; 殘差速度約 5 幀內衰減掉
        self.coast_vel_decay_fast = 0.70 # was 0.99; 長 coast 更快歸零
        # 殘差速度上限 (px/s): 防雜訊/reset 後虛假高速 (log 曾見 565) 把框甩飛
        self.max_residual_vel_pps = 120.0

        # 上次 ego predict 的內部狀態 (給 P 推進用)
        self._last_ego_dpx = np.zeros(2)
        self._last_ego_uncertainty_var = 0.0

        # ★ 暴露給上層 CSV 的 per-update 量測殘差 (每次 update() 後刷新)
        #   update() 時計算 innovation y_innov 和 Mahalanobis² 都只是內部變數,
        #   留下來才能畫 innovation vs time / DKF uncertainty 等分析圖.
        self._last_innovation = np.zeros(2)       # [u_innov, v_innov] (px)
        self._last_innovation_norm = 0.0          # ||y_innov|| (px)
        self._last_mahalanobis2 = 0.0             # mahal²
        self._last_update_was_accepted = False    # update() 是否通過 gate

        # ✨ 上一步飛機 + 雲台狀態 (給 projective predict 算 ego shift)
        self._prev_ac_state = None
        self._prev_gimbal_state = None

        # ✨ V3.6: dt 相關的 ego comp 閾值 (原本寫死, 針對 100Hz 調).
        # 改成可調屬性, 預設值 = 原本寫死的值 → 不影響任何既有呼叫端.
        # 低頻 (e.g. 相機 30Hz) 呼叫端應依 dt 放大這兩個值, 否則:
        #   - ego_gimbal_jump_deg 太小 → 雲台一追就被誤判跳變, 整段不做 ego comp
        #   - ego_dpx_max_px 太小 → 單步合理 ego 位移被 clip, ego comp 被低估
        # 建議縮放: 以 100Hz(dt=0.01) 為基準, 乘上 (dt / 0.01).
        self.ego_gimbal_jump_deg = 1.5     # 單步雲台角度跳變上限 (deg)
        self.ego_dpx_max_px = 30.0         # 單步 ego pixel 位移上限 (px)

        print(f"[DKF V3] 初始化:")
        print(f"  predict 頻率: {1/dt:.0f} Hz")
        print(f"  預設延遲: {image_delay*1000:.0f} ms ({self.default_delay_steps} 步)")
        print(f"  History buffer: {max_history} 步 = {max_history*dt*1000:.0f} ms (最大可補償延遲)")
        print(f"  Update 補償深度上限: {max_update_delay_steps} 步 = {max_update_delay_steps*dt*1000:.0f} ms "
              f"(超過此值的延遲靠 NED fallback)")
        print(f"  Ego 補償: {'ON' if enable_ego_comp else 'OFF'}")
        print(f"  相機內參: fx={fx:.0f} fy={fy:.0f} cx={cx:.0f} cy={cy:.0f}")
        print(f"  過程噪聲 (殘差): {accel_noise:.0f} px/s²  "
              f"(V2 對應值: 150)")
        print(f"  Coasting: start={self.coast_start}步 "
              f"Q_inflate={self.coast_q_inflation}x")

    # ─────────────────────────────────────────────────────────────
    #  Predict (三個版本: pure CV / IBVS Jacobian / Projective)
    # ─────────────────────────────────────────────────────────────

    @staticmethod
    def _build_F(dt):
        return np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

    def _build_Q(self, dt):
        """用實際 dt 重建過程噪聲. accel_noise 從 __init__ 存的值."""
        dt2 = dt * dt
        return (self._accel_noise ** 2) * np.array([
            [dt2*dt2/4, 0,         dt2*dt/2, 0        ],
            [0,         dt2*dt2/4, 0,        dt2*dt/2 ],
            [dt2*dt/2,  0,         dt2,      0        ],
            [0,         dt2*dt/2,  0,        dt2      ],
        ])

    def predict(self, dt=None,
                ac_state=None,         # dict: x_ned, y_ned, alt_agl, roll_deg, pitch_deg, yaw_true_deg
                gimbal_state=None,     # dict: pitch_deg, yaw_deg
                depth_m=None):
        """
        高頻預測 — 推薦用法 (projective ego comp).

        把當前 pixel 反投影到 NED 假設目標位置, 用「新姿態」正投影回 pixel,
        差值就是 ego motion 造成的 pixel shift. 自動包含飛機 + 雲台所有運動.

        三種呼叫方式:
          1. predict()
             退化為純 CV model (V2 行為), 不做 ego comp
          2. predict(ac_state, gimbal_state)
             不需要深度: 用上次的深度, 沒有就退化成純 CV
          3. predict(ac_state, gimbal_state, depth_m=slant_range)  ← 推薦
             完整 ego comp, 旋轉+平移都包

        ac_state dict 必須含:
            x_ned, y_ned (m), alt_agl (m),
            roll_deg, pitch_deg, yaw_true_deg
        gimbal_state dict 必須含:
            pitch_deg, yaw_deg
        depth_m: 當前估測的目標斜距 (m), 從 RayProjection 的 slant_range
        """
        if not self.initialized:
            return

        self._consecutive_miss += 1

        # ── 動態 dt ──
        if dt is not None:
            dt = float(np.clip(dt, 0.02, 0.15))
            self.dt = dt
        _F = self._build_F(self.dt)
        _Q = self._build_Q(self.dt)

        # ── Coasting mode ──
        if self._consecutive_miss > self.coast_start:
            coast_steps = self._consecutive_miss - self.coast_start
            q_factor = min(1.0 + coast_steps * self.coast_q_rate, self.coast_q_inflation)
            Q_eff = _Q * q_factor
            decay = self.coast_vel_decay if coast_steps < 30 else self.coast_vel_decay_fast
            self.x[2] *= decay
            self.x[3] *= decay
        else:
            Q_eff = _Q

        # ── (a) Projective ego compensation ──
        ego_dpx = np.zeros(2)
        ego_var = 0.0

        if (self.enable_ego_comp
                and ac_state is not None
                and gimbal_state is not None
                and self._prev_ac_state is not None
                and self._prev_gimbal_state is not None
                and depth_m is not None and depth_m > 1.0):
            # ✨ V3.1 防護: 偵測雲台角度跳變 (e.g. DIVE 觸發時 gimbal lock)
            # 100Hz 下正常雲台速率 < 60°/s → 單步 < 0.6°
            # 超過閾值就視為 lock/unlock 跳變, 跳過此拍 ego comp
            d_gimbal_pitch = abs(gimbal_state['pitch_deg'] - self._prev_gimbal_state['pitch_deg'])
            d_gimbal_yaw   = abs(gimbal_state['yaw_deg']   - self._prev_gimbal_state['yaw_deg'])
            _jump_thr = self.ego_gimbal_jump_deg
            gimbal_jumped = (d_gimbal_pitch > _jump_thr) or (d_gimbal_yaw > _jump_thr)

            if not gimbal_jumped:
                # Step 1: 反投影 (pixel + depth → NED), 用「上一步的姿態」
                P_ned = self._backproject_pixel_to_ned(
                    u_pixel=self.x[0], v_pixel=self.x[1],
                    depth=depth_m,
                    ac_state=self._prev_ac_state,
                    gimbal_state=self._prev_gimbal_state,
                )
                # Step 2: 用「當前姿態」把這個 NED 點正投影回 pixel
                u_new, v_new, valid = reproject_target_ned_to_pixel(
                    target_x_ned=P_ned[0],
                    target_y_ned=P_ned[1],
                    ac_x_ned=ac_state['x_ned'],
                    ac_y_ned=ac_state['y_ned'],
                    ac_altitude_agl=ac_state['alt_agl'],
                    ac_roll_deg=ac_state['roll_deg'],
                    ac_pitch_deg=ac_state['pitch_deg'],
                    ac_yaw_true_deg=ac_state['yaw_true_deg'],
                    gimbal_pitch_deg=gimbal_state['pitch_deg'],
                    gimbal_yaw_deg=gimbal_state['yaw_deg'],
                    fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                    R_g2c=DEFAULT_R_G2C,
                    target_altitude_msl=P_ned[2],   # 保留高度資訊
                )
                if valid:
                    raw_dpx_x = u_new - self.x[0]
                    raw_dpx_y = v_new - self.x[1]
                    raw_dpx_norm = math.sqrt(raw_dpx_x**2 + raw_dpx_y**2)

                    # ✨ V3.1 防護: 單步 ego shift 異常大 → 視為 depth 估測或姿態量測異常
                    # 100Hz 下正常 ego shift < 20 px (超快機動 30 px). 超過上限就 clip.
                    # V3.6: 上限改為可調 (低頻呼叫端會放大此值, 見 __init__ 註解)
                    EGO_DPX_MAX_PIXEL = self.ego_dpx_max_px
                    if raw_dpx_norm > EGO_DPX_MAX_PIXEL:
                        # 等比例 clip 到上限, 避免完全丟掉訊號
                        scale = EGO_DPX_MAX_PIXEL / raw_dpx_norm
                        ego_dpx[0] = raw_dpx_x * scale
                        ego_dpx[1] = raw_dpx_y * scale
                        # 不確定性放大 (因為我們不信任這個 ego comp)
                        ego_var = (raw_dpx_norm * 0.5) ** 2 + 100.0
                    else:
                        ego_dpx[0] = raw_dpx_x
                        ego_dpx[1] = raw_dpx_y
                        # 不確定性: ego_dpx 大時加大 (補償誤差跟移動量成比例)
                        ego_var = (0.05 * raw_dpx_norm) ** 2 + 1.0
            else:
                # Gimbal 跳變 → 跳過這拍 ego comp, 並把跳變記下
                # 下一拍會用新 prev_state, 自動恢復正常
                if not hasattr(self, '_jump_count'):
                    self._jump_count = 0
                self._jump_count += 1
                if self._jump_count % 10 == 1:
                    print(f"[DKF V3.1] ⚠ Gimbal 跳變偵測 "
                          f"(d_pitch={d_gimbal_pitch:.2f}° d_yaw={d_gimbal_yaw:.2f}°), "
                          f"跳過此拍 ego comp [count={self._jump_count}]")

        # ── 套用 ──
        # 注意: 套用順序很重要
        # x[0:2] (pixel pos) 先做 CV (用 x[2:4] 的殘差速度), 再加 ego 位移
        self.x = _F @ self.x
        self.x[0] += ego_dpx[0]
        self.x[1] += ego_dpx[1]
        # ✨ 殘差速度上限: 防止虛假高速把框甩飛 (見 __init__ max_residual_vel_pps)
        _vmag = math.hypot(self.x[2], self.x[3])
        if _vmag > self.max_residual_vel_pps:
            _sc = self.max_residual_vel_pps / _vmag
            self.x[2] *= _sc
            self.x[3] *= _sc

        # P propagation (用動態 F/Q)
        self.P = _F @ self.P @ _F.T + Q_eff
        self.P[0, 0] += ego_var
        self.P[1, 1] += ego_var

        # 記錄
        self._last_ego_dpx = ego_dpx
        self._last_ego_uncertainty_var = ego_var
        if ac_state is not None:
            self._prev_ac_state = dict(ac_state)
        if gimbal_state is not None:
            self._prev_gimbal_state = dict(gimbal_state)

        self._predict_count += 1
        # ✨ V3.3: history 多存 ego_dpx, update 往前推時可重放
        self._history.append((self.x.copy(), self.P.copy(), ego_dpx.copy()))

    def _backproject_pixel_to_ned(self, u_pixel, v_pixel, depth,
                                   ac_state, gimbal_state):
        """
        把 (pixel + depth) 反投影到 NED 座標.

        步驟:
          Pc = depth * normalize([(u-cx)/fx, (v-cy)/fy, 1])
          P_world = ac_pos_ned + R_cam2ned @ Pc
        """
        # 像素 → 相機座標方向向量
        x1 = (u_pixel - self.cx) / self.fx
        x2 = (v_pixel - self.cy) / self.fy
        Pc_dir = np.array([x1, x2, 1.0])
        Pc = depth * Pc_dir / np.linalg.norm(Pc_dir)

        # SIYI A8 world-stabilized: 相機朝向只由雲台世界角決定, 不乘 R_w2b。
        # 必須與 reproject_target_ned_to_pixel 同慣例, FULL_PROJECTIVE 正反向才閉合。
        R_ned2gimbal = (_Ry(gimbal_state['pitch_deg'])
                        @ _Rz(gimbal_state['yaw_deg']))
        R_ned2cam = DEFAULT_R_G2C @ R_ned2gimbal
        R_cam2ned = R_ned2cam.T   # 旋轉矩陣的逆 = 轉置

        # 在相機座標的點 → NED 座標 (相對於飛機)
        rel_ned = R_cam2ned @ Pc

        # 加上飛機 NED 位置 → 絕對 NED
        # 飛機在 NED 的 z = -alt_agl (NED z 朝下為正, 飛機在地面上)
        ac_ned = np.array([ac_state['x_ned'],
                            ac_state['y_ned'],
                            -ac_state['alt_agl']])
        return ac_ned + rel_ned

    #  Update (YOLO 量測, 含延遲補償)
    # ─────────────────────────────────────────────────────────────

    def update(self, z_image, delay_steps=None, bbox_size=None):
        """
        標準 YOLO 量測更新 (與 V2 同介面, 但延遲前推時也用 ego comp).

        注意: 因為歷史的 ego motion 已經在 _history 裡的 x 反映了,
              這裡的延遲前推用 F_base 即可 (歷史 x 已含 ego 修正).
        """
        z = np.asarray(z_image, dtype=float)

        if not self.initialized:
            self.x[0:2] = z
            self.x[2:4] = 0.0
            self.P = np.diag([20.0**2, 20.0**2, 50.0**2, 50.0**2])
            self.initialized = True
            self._history.append((self.x.copy(), self.P.copy(), np.zeros(2)))
            return

        # 動態 R
        if bbox_size is not None and bbox_size > 5:
            r_std = self.R_base_std * (self.R_bbox_ref / bbox_size)
            r_std = np.clip(r_std, self.R_min_std, self.R_max_std)
        else:
            r_std = self.R_base_std
        R = np.diag([r_std**2, r_std**2])

        if delay_steps is None:
            delay_steps = self.default_delay_steps
        delay_steps = min(delay_steps, len(self._history) - 1)
        delay_steps = max(delay_steps, 0)
        # ✨ V3.3: 明確限制 update 補償深度 (原本靠 buffer 小被動限制)
        # 超過 max_update_delay_steps 的延遲, 實際補償只到上限
        # 因為長距離 forward propagation 會放大 KF velocity correction 的誤差
        # 真正大延遲應該讓 NED fallback 接手
        delay_steps = min(delay_steps, self.max_update_delay_steps)

        # 取延遲時刻狀態
        if delay_steps > 0 and len(self._history) > delay_steps:
            x_d, P_d, _ = self._history[-(delay_steps + 1)]
            x_d, P_d = x_d.copy(), P_d.copy()
        else:
            x_d, P_d = self.x.copy(), self.P.copy()
            delay_steps = 0

        # Innovation + gate
        y_innov = z - self.H @ x_d
        S = self.H @ P_d @ self.H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        mahal2 = float(y_innov.T @ S_inv @ y_innov)
        # ★ 寫進實例變數讓上層讀 (給 CSV / 分析圖用)
        self._last_innovation = y_innov.copy()
        self._last_innovation_norm = float(np.linalg.norm(y_innov))
        self._last_mahalanobis2 = mahal2
        self._last_update_was_accepted = (mahal2 <= self.gate_threshold)

        if mahal2 > self.gate_threshold:
            self._reject_count += 1
            if self._reject_count % 10 == 1:
                print(f"[DKF V3] ⚠️ YOLO 量測被拒 (mahal²={mahal2:.1f}) "
                      f"z={z.astype(int)} vs pred={(self.H @ x_d).astype(int)}")
            return

        # KF update at t_{k-D}
        K = P_d @ self.H.T @ S_inv
        x_corrected = x_d + K @ y_innov
        P_corrected = (np.eye(4) - K @ self.H) @ P_d

        # ✨ V3.3: 前推 D 步 — 只用 F_base (純 CV)
        # NOT replay ego_dpx, 因為歷史 ego_dpx 是用舊 x 算的, KF update 後會 mismatch
        # 嚴重延遲時 (>300ms) 這個前推會丟掉部分 ego motion, 應靠 NED fallback 補
        for _ in range(delay_steps):
            x_corrected = self.F_base @ x_corrected
            P_corrected = self.F_base @ P_corrected @ self.F_base.T + self.Q

        # 覆寫
        self.x = x_corrected
        self.P = P_corrected
        self._update_count += 1
        self._consecutive_miss = 0
        if self._history:
            # 保留歷史 ego_dpx (不變動), 只更新 x, P
            _, _, last_ego = self._history[-1]
            self._history[-1] = (self.x.copy(), self.P.copy(), last_ego)

    # ─────────────────────────────────────────────────────────────
    #  (c) Update with NED reprojection fallback
    # ─────────────────────────────────────────────────────────────

    def update_fallback(self, z_pixel, r_std_pixel, gate_override=None):
        """
        NED 反投影 fallback 量測更新.

        與 update() 的差別:
          - R 由外部決定 (依 target_kf 位置 std + slant_range 推算)
          - 沒有 delay 補償 (反投影是「現在」的預測, 不是延遲量測)
          - gate 放寬 (預設用 gate_threshold_fallback)
          - 不重置 _consecutive_miss (因為這不是真實視覺量測)

        Args:
            z_pixel: array(2,), 反投影的 pixel 座標
            r_std_pixel: float, 量測不確定性 (pixel std)
            gate_override: 自訂 mahalanobis gate threshold, None 用預設
        """
        z = np.asarray(z_pixel, dtype=float)

        if not self.initialized:
            # 用 fallback 初始化 (緊急情況, 比沒有好)
            self.x[0:2] = z
            self.x[2:4] = 0.0
            self.P = np.diag([r_std_pixel**2, r_std_pixel**2,
                              100.0**2, 100.0**2])
            self.initialized = True
            self._history.append((self.x.copy(), self.P.copy(), np.zeros(2)))
            return

        R = np.diag([r_std_pixel**2, r_std_pixel**2])
        gate = gate_override if gate_override is not None else self.gate_threshold_fallback

        y_innov = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        mahal2 = float(y_innov.T @ S_inv @ y_innov)
        # ★ 寫進實例變數 (同 update())
        self._last_innovation = y_innov.copy()
        self._last_innovation_norm = float(np.linalg.norm(y_innov))
        self._last_mahalanobis2 = mahal2
        self._last_update_was_accepted = (mahal2 <= gate)
        if mahal2 > gate:
            self._reject_count += 1
            if self._reject_count % 20 == 1:
                print(f"[DKF V3] ⚠️ NED fallback 量測被拒 (mahal²={mahal2:.1f}, gate={gate:.0f})")
            return

        K = self.P @ self.H.T @ S_inv
        self.x = self.x + K @ y_innov
        self.P = (np.eye(4) - K @ self.H) @ self.P

        self._update_fallback_count += 1
        # 注意: 不重置 _consecutive_miss, 因為 fallback 不算「真的看到了目標」
        if self._history:
            _, _, last_ego = self._history[-1]
            self._history[-1] = (self.x.copy(), self.P.copy(), last_ego)

    # ─────────────────────────────────────────────────────────────
    #  Getters
    # ─────────────────────────────────────────────────────────────

    def get_position(self):
        return self.x[0:2].copy()

    def get_velocity(self):
        """像素殘差速度 (px/s, 不含 ego motion)"""
        return self.x[2:4].copy()

    def get_position_std(self):
        return np.sqrt(np.diag(self.P)[:2])

    def get_stats(self):
        return {
            'predict_count': self._predict_count,
            'update_count': self._update_count,
            'update_fallback_count': self._update_fallback_count,
            'reject_count': self._reject_count,
            'consecutive_miss': self._consecutive_miss,
            'last_ego_dpx': self._last_ego_dpx.copy(),
        }

    def reset(self, initial_position=None):
        if initial_position is not None:
            self.x[0:2] = initial_position
            self.initialized = True
        else:
            self.x = np.zeros(4)
            self.initialized = False
        self.x[2:4] = 0.0
        self.P = np.eye(4) * 100.0
        self._history.clear()
        self._predict_count = 0
        self._update_count = 0
        self._update_fallback_count = 0
        self._reject_count = 0
        self._consecutive_miss = 0
        self._prev_ac_state = None
        self._prev_gimbal_state = None


# ════════════════════════════════════════════════════════════════════
#  自我測試 (跑一個合成情境驗證 ego comp 有效)
# ════════════════════════════════════════════════════════════════════

# ========================================================================
# 原模組: dkf_tracker_gimbal.py
# ========================================================================
#!/usr/bin/env python3
# -*- coding: utf-8 -*-



class DKFTrackerGimbal:
    """
    EgoAwareDKF 的精簡 wrapper (純 CV mode).

    State: [u, v, u_dot, v_dot] (pixel)
    每幀 predict() + (有偵測就 update / 沒偵測就 get_position).

    對長 miss 後重新偵測有保護: 連續 miss 超過閾值, 下次 YOLO 偵測時強制
    reset DKF 而不是 update, 避免 Mahalanobis gate 把新偵測誤判為 outlier.
    """

    def __init__(self,
                 fps=30,
                 screen_width=1920,
                 screen_height=1080,
                 reset_after_miss_frames=None,
                 use_yolo_when_available=True,
                 fx=None, fy=None, cx=None, cy=None,
                 gate_threshold=1000.0,
                 coast_mode='cv',
                 outlier_max_jump_px=200.0,
                 outlier_high_conf=0.4,
                 outlier_hard_jump_px=600.0,
                 outlier_bbox_ratio=None,
                 outlier_max_age_frames=None,
                 hold_timeout_frames=None,
                 min_updates_to_coast=5,
                 enable_full_ego_comp=False,
                 verbose=True):
        """
        Args:
            ...
            outlier_high_conf: 信心高於此值的偵測:
                - 連續追蹤場景: 永遠接受 (不管跳多遠)
                - 初始化 / 長 miss 後重新出現: 必須通過此門檻才接受
                預設 0.4 - 平衡「擋 false positive」vs「讓真實低信心目標進得來」.
                你的 YOLO 模型對真實目標 conf 普遍偏低時可以調更低.
            hold_timeout_frames: DKF hold 超過此幀數沒收到新 YOLO confirm → 主動清空,
                允許重新初始化. 預設 = outlier_max_age_frames (3 秒).
                避免系統永遠卡在 false positive 初始位置.
        """
        """
        Args:
            fps: 主迴圈預期執行頻率, DKF 固定 dt = 1/fps.
                 實際 FPS 抖動 ±30% 內可接受.
            screen_width, screen_height: 影像尺寸 (pixel).
            reset_after_miss_frames: 連續 miss 多少幀後, 下次 YOLO 偵測強制 reset DKF.
                                     None = 自動取 fps 數 (= 1 秒).
                                     太大 → DKF 飄太遠後 gate 拒絕新偵測.
                                     太小 → 失去 DKF 平滑效果.
            use_yolo_when_available: 偵測到時:
                                     True  = 用 YOLO raw 座標 (推薦, 避免濾波延遲)
                                     False = 用 DKF 平滑後座標
            fx, fy, cx, cy: 相機內參. 純 CV mode 不會用到, 傳 None 給 placeholder.
                            升級到 ego comp mode 時才需要真實標定值.
            verbose: 是否印 DKF 初始化訊息.
        """
        self.fps = float(fps)
        self.dt = 1.0 / self.fps
        self.screen_w = int(screen_width)
        self.screen_h = int(screen_height)
        self.use_yolo_when_available = bool(use_yolo_when_available)

        if reset_after_miss_frames is None:
            reset_after_miss_frames = int(fps)
        self.reset_after_miss_frames = int(reset_after_miss_frames)

        # 純 CV mode: 內參只用作 placeholder, 不會被 predict() 用到
        _fx = float(fx) if fx is not None else 1500.0
        _fy = float(fy) if fy is not None else 1500.0
        _cx = float(cx) if cx is not None else self.screen_w / 2.0
        _cy = float(cy) if cy is not None else self.screen_h / 2.0

        # ★ 新增: 給「雲台 ego comp」用的內參副本
        # 公式: 純旋轉相機, 同一個世界點在前後兩幀的 pixel 位置差:
        #   du = -fx * Δyaw_rad     (雲台往右轉 yaw+ → 像素往左移)
        #   dv = -fy * Δpitch_rad   (雲台往上轉 pitch+ → 像素往下移)
        self._ego_fx = _fx
        self._ego_fy = _fy
        # 上一步雲台姿態 (給 ego comp 算 Δyaw, Δpitch 用)
        self._prev_gimbal_yaw_deg = None
        self._prev_gimbal_pitch_deg = None
        # Ego comp 統計 (給 debug 看實際補多少 pixel)
        self._total_ego_shift_u = 0.0
        self._total_ego_shift_v = 0.0
        self._ego_comp_applied_frames = 0
        # 上一次 update 用的 delay_steps (給 get_state / CSV 看實際延遲補償了幾步)
        self._last_delay_steps = 0

        # ★ V3.6: full projective ego comp 開關
        # False (預設) = 原本行為: DKF enable_ego_comp=False, 只做雲台粗近似.
        # True          = DKF enable_ego_comp=True, step() 收到完整飛機姿態 +
        #                 雲台(估測器慣例) + depth 時, 走 EgoAwareDKF 的
        #                 projective predict (反投影→正投影扣 ego 漂移,
        #                 飛機本體 roll/pitch/yaw 全包). 資料不齊的幀自動退回
        #                 雲台粗近似 / 純 CV, 不會壞.
        self.enable_full_ego_comp = bool(enable_full_ego_comp)
        # full ego comp 統計
        self._full_ego_frames = 0          # 實際走 projective 的幀數
        self._full_ego_skipped_frames = 0  # 想走但資料不齊 → 退回粗近似的幀數

        _dkf_ego_flag = self.enable_full_ego_comp

        # 純 CV mode 也不用 image_delay (projective ego comp 不啟用)
        if verbose:
            self.dkf = EgoAwareDKF(
                dt=self.dt,
                image_delay=0.0,
                fx=_fx, fy=_fy, cx=_cx, cy=_cy,
                enable_ego_comp=_dkf_ego_flag,
            )
        else:
            import contextlib, io
            with contextlib.redirect_stdout(io.StringIO()):
                self.dkf = EgoAwareDKF(
                    dt=self.dt,
                    image_delay=0.0,
                    fx=_fx, fy=_fy, cx=_cx, cy=_cy,
                    enable_ego_comp=_dkf_ego_flag,
                )

        # ★ V3.6: 依相機 dt 放大 projective 的兩個閾值.
        # EgoAwareDKF 預設值針對 100Hz(dt=0.01) 調. 相機 ~30Hz 一步是 3 倍多時間,
        # 不放大的話: 雲台一追就被當「跳變」整段不補, 合理 ego 位移也被 clip.
        # 以 dt/0.01 為比例, 並設下限避免 fps 很高時反而過鬆.
        if self.enable_full_ego_comp:
            _scale = max(1.0, self.dt / 0.01)
            self.dkf.ego_gimbal_jump_deg = 1.5 * _scale
            self.dkf.ego_dpx_max_px = 30.0 * _scale
            if verbose:
                print(f"[DKFTracker] full projective ego comp = ON "
                      f"(dt={self.dt*1000:.1f}ms, 閾值縮放×{_scale:.1f}: "
                      f"雲台跳變<{self.dkf.ego_gimbal_jump_deg:.1f}°/步, "
                      f"ego位移≤{self.dkf.ego_dpx_max_px:.0f}px/步)")

        # ★ 重要: 純 CV mode 下放寬 Mahalanobis gate
        # v3 預設 gate=200 對「自身會動的飛機 + 主動雲台」太緊,
        # 會把 ego motion 造成的 pixel 跳動誤判為 outlier 拒絕 YOLO update,
        # DKF 因此永遠跟不上目標. 用更大的 gate 接受這些「看起來像 outlier 但
        # 其實是 ego motion」的量測.
        self.dkf.gate_threshold = float(gate_threshold)
        if verbose:
            print(f"[DKFTracker] gate_threshold 調整: 200.0 → {gate_threshold} "
                  f"(純 CV mode 飛機/雲台場景必要)")

        # 統計
        self._reset_count = 0
        self._total_dkf_coast_frames = 0
        self._total_yolo_frames = 0

        # 最後一次 YOLO 偵測到的 bbox 尺寸 (給 overlay 在補位時畫一樣大的框用)
        self.last_bbox_w = 0.0
        self.last_bbox_h = 0.0

        # Coast mode 設定
        if coast_mode not in ('hold', 'cv'):
            print(f"[DKFTracker] 警告: 未知的 coast_mode='{coast_mode}', 改用 'hold'")
            coast_mode = 'hold'
        self.coast_mode = coast_mode
        # 最後一次 update 後 DKF 位置 (給 hold mode 凍結用)
        self._last_update_pos = None

        # Outlier filter 設定
        self.outlier_max_jump_px = float(outlier_max_jump_px)
        self.outlier_high_conf = float(outlier_high_conf)
        # ★ 框大小一致性 gating: None=關閉. 設 1.8 → 單幀 bbox 面積比 >1.8x 或
        #   <1/1.8 且非高信心 → 拒絕 (跳到大小不同的鄰車). 俯衝漸進放大不會觸發.
        self.outlier_bbox_ratio = (float(outlier_bbox_ratio)
                                   if outlier_bbox_ratio is not None else None)
        self._last_good_bbox_area = None   # 上次接受的 bbox 面積 (px²)
        if outlier_max_age_frames is None:
            # 預設 90 幀 (3 秒 @ 30fps)
            outlier_max_age_frames = max(int(self.fps * 3.0), 30)
        self.outlier_max_age_frames = int(outlier_max_age_frames)
        # Hold timeout: DKF hold 太久沒新 confirm 就主動清空
        if hold_timeout_frames is None:
            hold_timeout_frames = self.outlier_max_age_frames
        self.hold_timeout_frames = int(hold_timeout_frames)
        # ★ 「物理上不可能的跳躍」硬上限: 連續追蹤中, 跳超過此距離
        #   一律視為 outlier, 不管 conf 多高 (high_conf 豁免在此失效).
        #   理由: 目標一幀內位移數百~上千 px 在物理上不可能, 必是 false
        #   positive / ID 跳到別物體 / YOLO 抽風. 預設 600px (≈ 1080p 畫面
        #   1/3 寬), 比 outlier_max_jump_px(200) 寬鬆, 只擋「離譜」等級.
        #   設 None 可關閉此硬門檻 (退回原行為).
        self.outlier_hard_jump_px = (float(outlier_hard_jump_px)
                                     if outlier_hard_jump_px is not None
                                     else None)
        # Outlier filter 狀態
        self._last_good_yolo_pos = None
        self._frames_since_good_yolo = 0
        self._total_outlier_rejected = 0

        # ★ 成熟度門檻: 連續成功 YOLO update (含 RESET_INIT) 數要 ≥ 此值,
        #   DKF 才會在 miss 時對外輸出 coast 預測.
        #   未成熟前 miss 一律回傳 (None, None, False, 'IMMATURE')
        #   ─ 觀測太少時 DKF 速度估計不可信, 不應該讓雲台跟著預測軌跡走.
        #   ─ reset 後計數歸零, 要重新累積.
        #   調整指引:
        #     min_updates_to_coast=1 → 等同舊行為 (一個 sample 就 coast)
        #     min_updates_to_coast=3 → 至少看到 3 幀才信任 (對 7 FPS ≈ 0.4 秒)
        #     min_updates_to_coast=5 → 較保守 (≈ 0.65 秒)
        #     min_updates_to_coast=10→ 保守 (≈ 1.3 秒, 短遮擋 < 1 秒就會被擋)
        self.min_updates_to_coast = int(min_updates_to_coast)
        self._consecutive_yolo_updates = 0

        # ★ Per-frame step info (給 CSV / 分析圖用; 每次 step() 開頭重置)
        #   把 step() 內原本只是區域變數的「這一幀做了什麼」全部寫進來,
        #   讓 get_state() 可以一次拿到, 不用上層拼湊.
        self._last_step_info = self._fresh_step_info()

        if verbose:
            print(f"[DKFTracker] coast_mode = '{coast_mode}' "
                  f"({'YOLO miss 時凍結在最後位置' if coast_mode=='hold' else 'YOLO miss 時 CV 推進'})")
            print(f"[DKFTracker] outlier filter: jump>{outlier_max_jump_px}px "
                  f"AND conf<{outlier_high_conf} → reject; "
                  f"硬上限 jump>{self.outlier_hard_jump_px}px → reject(不豁免)")

    @staticmethod
    def _fresh_step_info():
        """每幀重置成這個空白範本, 然後 step() 過程中填上實際值."""
        return {
            # Ego compensation per-frame
            'ego_comp_mode': 'NONE',          # NONE/GIMBAL_ROTATION/FULL_PROJECTIVE/FALLBACK_CV
            'ego_comp_applied': 0,            # 0/1 — 此幀有沒有實際套用 ego shift
            'ego_du_px': 0.0,                 # 此幀套用的 u 方向 shift (px)
            'ego_dv_px': 0.0,                 # 此幀套用的 v 方向 shift (px)
            'full_ego_used': 0,               # 0/1 — 此幀有沒有走 full projective
            'full_ego_skipped': 0,            # 0/1 — 想用 full 但跳過了
            'full_ego_skip_reason': 'NONE',   # 跳過原因
            # Outlier per-frame
            'outlier_rejected_this_frame': 0, # 0/1
            'outlier_reason': 'NONE',         # NONE/LOW_CONF_JUMP/HARD_JUMP/LOW_CONF_RESET/...
            'yolo_jump_px': 0.0,              # 此幀 YOLO 距上次 good YOLO 的距離
            # Innovation (從底層 DKF 拿)
            'innovation_u': 0.0,
            'innovation_v': 0.0,
            'innovation_norm': 0.0,
            'mahalanobis2': 0.0,
            # Status
            'status': 'WAITING',
        }

    # ----------------------------------------------------------------
    #  主介面
    # ----------------------------------------------------------------
    def step(self, detected, u_yolo=None, v_yolo=None, bbox_w=None, bbox_h=None,
             dt=None,
             confidence=None, gimbal_yaw_deg=None, gimbal_pitch_deg=None,
             measurement_delay_ms=0.0,
             ac_roll_ned_deg=None, ac_pitch_ned_deg=None, ac_yaw_true_deg=None,
             ac_x_ned=None, ac_y_ned=None, ac_alt_agl=None,
             gimbal_pitch_est_deg=None, gimbal_yaw_est_deg=None,
             target_depth_m=None):
        """
        每幀呼叫一次. 順序: predict → (gimbal ego comp) → outlier → (update or coast).

        Args:
            detected: YOLO 是否偵測到此幀的目標 (bool).
            u_yolo, v_yolo: YOLO bbox 中心 pixel (detected=True 時必傳).
            bbox_w: bbox 寬度 (pixel), 給 DKF 動態 R 用. None 用預設 R.
            bbox_h: bbox 高度 (pixel), 給 overlay 畫框尺寸用. None 不更新.
            confidence: YOLO 偵測信心 (0~1). 給 outlier filter 用.
            gimbal_yaw_deg: 雲台當前 yaw (度, SIYI 原始). 給「雲台粗近似」ego
                comp 用 (full ego comp 關閉, 或下面 full 參數不齊時的後援).
            gimbal_pitch_deg: 雲台當前 pitch (度, SIYI 原始). 同上.
            measurement_delay_ms: 此次量測延遲 (ms). 建議傳 inference_ms.

            ───── 以下為 V3.6 full projective ego comp 參數 (全部選填) ─────
            只有 enable_full_ego_comp=True 且「飛機姿態三軸 + AGL +
            雲台估測慣例角度 + depth」全部有值, 這一幀才會走 projective
            predict (飛機本體 roll/pitch/yaw 漂移也扣掉). 缺任何一項 →
            這一幀自動退回雲台粗近似 / 純 CV, 不會壞.

            ac_roll_ned_deg, ac_pitch_ned_deg, ac_yaw_true_deg:
                飛機姿態, NED 慣例, deg. yaw 為真航向 (0=北, 90=東).
                由 MAVROS ENU 經 convert_mavros_enu_to_ned_attitude() 轉出,
                必須跟距離估測器用同一套轉換, 投影鏈才一致.
            ac_x_ned, ac_y_ned: 飛機 NED 北/東座標 (m). 沒有可給 0
                (ego comp 只看前後兩幀差, 常數偏移會抵銷).
            ac_alt_agl: 飛機 AGL 高度 (m).
            gimbal_pitch_est_deg, gimbal_yaw_est_deg:
                雲台角度, 估測器慣例 (經 convert_siyi_gimbal_to_estimator()
                轉出, 與距離估測器同一套). 注意: 這跟 gimbal_*_deg 的
                SIYI 原始值不同, 不可混用.
            target_depth_m: 目標斜距 (m), 通常傳上一幀的 slant_range
                (depth 變化遠慢於單幀, 一幀延遲可接受, 與主控制器
                depth_ema 同作法).

        Returns:
            (u_used, v_used, dkf_used, status):
                status: 'YOLO' / 'DKF_COAST' / 'RESET_INIT' / 'WAITING' /
                        'OUTLIER' / 'IMMATURE'
                IMMATURE = DKF 已初始化但連續 YOLO update 數還不夠
                         (< min_updates_to_coast). 此時 miss 不對外輸出 coast,
                         u_used/v_used=None, dkf_used=False, 上層應視為無目標.
        """
        # ★ 重置 per-frame 紀錄
        self._last_step_info = self._fresh_step_info()
        info = self._last_step_info   # 本次 step 的紀錄 (跟外層共享, 直接改)

        # ★ v1.5: 動態 dt — 用實際幀間隔取代固定 1/fps
        if dt is not None:
            import numpy as _np
            self.dt = float(_np.clip(dt, 0.02, 0.15))

        # ── 判斷這一幀能不能走 full projective ego comp ──
        _full_ego_ready = (
            self.enable_full_ego_comp
            and ac_roll_ned_deg is not None
            and ac_pitch_ned_deg is not None
            and ac_yaw_true_deg is not None
            and ac_alt_agl is not None
            and gimbal_pitch_est_deg is not None
            and gimbal_yaw_est_deg is not None
            and target_depth_m is not None
            and target_depth_m > 1.0
        )
        # ★ 如果 enable_full_ego_comp=True 但本幀條件不齊, 找出哪個沒帶
        if self.enable_full_ego_comp and not _full_ego_ready:
            if ac_roll_ned_deg is None or ac_pitch_ned_deg is None or ac_yaw_true_deg is None:
                info['full_ego_skip_reason'] = 'NO_UAV_ATTITUDE'
            elif ac_alt_agl is None:
                info['full_ego_skip_reason'] = 'NO_AGL'
            elif gimbal_pitch_est_deg is None or gimbal_yaw_est_deg is None:
                info['full_ego_skip_reason'] = 'NO_GIMBAL_ATTITUDE'
            elif target_depth_m is None:
                info['full_ego_skip_reason'] = 'NO_DEPTH'
            elif target_depth_m <= 1.0:
                info['full_ego_skip_reason'] = 'INVALID_DEPTH'
        # 算 delay_steps (給 DKF 內部 update 用)
        if measurement_delay_ms > 0:
            delay_steps = int(round(measurement_delay_ms / 1000.0 / self.dt))
            # Clip 到合理範圍 (避免極端 outlier 把 KF 拉壞)
            delay_steps = max(0, min(delay_steps, 10))
        else:
            delay_steps = 0
        self._last_delay_steps = delay_steps

        # 1. Predict
        if _full_ego_ready:
            # ── V3.6: full projective ego comp ──
            # EgoAwareDKF 內部: 用「上一步姿態」反投影 pixel→NED, 再用
            # 「這一步姿態」正投影回 pixel, 差值就是飛機+雲台所有運動造成的
            # ego 漂移. 飛機本體 roll/pitch/yaw 全包, 不只雲台.
            ac_state = {
                'x_ned': float(ac_x_ned) if ac_x_ned is not None else 0.0,
                'y_ned': float(ac_y_ned) if ac_y_ned is not None else 0.0,
                'alt_agl': float(ac_alt_agl),
                'roll_deg': float(ac_roll_ned_deg),
                'pitch_deg': float(ac_pitch_ned_deg),
                'yaw_true_deg': float(ac_yaw_true_deg),
            }
            gimbal_state = {
                'pitch_deg': float(gimbal_pitch_est_deg),
                'yaw_deg': float(gimbal_yaw_est_deg),
            }
            self.dkf.predict(dt=self.dt,
                             ac_state=ac_state,
                             gimbal_state=gimbal_state,
                             depth_m=float(target_depth_m))
            self._full_ego_frames += 1

            # projective predict 已搬了 self.dkf.x, 但 tracker 自己維護的
            # _last_update_pos / _last_good_yolo_pos (hold mode 凍結點 +
            # outlier spatial 參考) 沒被搬. 用 DKF 這拍實際套用的 ego_dpx
            # 同步它們, 行為跟粗近似路徑一致.
            ego_dpx = getattr(self.dkf, '_last_ego_dpx', None)
            if ego_dpx is not None and self.dkf.initialized:
                _du, _dv = float(ego_dpx[0]), float(ego_dpx[1])
                if (_du != 0.0 or _dv != 0.0):
                    if self._last_update_pos is not None:
                        self._last_update_pos = (
                            self._last_update_pos[0] + _du,
                            self._last_update_pos[1] + _dv)
                    if self._last_good_yolo_pos is not None:
                        self._last_good_yolo_pos = (
                            self._last_good_yolo_pos[0] + _du,
                            self._last_good_yolo_pos[1] + _dv)
                    self._total_ego_shift_u += _du
                    self._total_ego_shift_v += _dv
                    self._ego_comp_applied_frames += 1
                    # ★ per-frame
                    info['ego_du_px'] = _du
                    info['ego_dv_px'] = _dv
                    info['ego_comp_applied'] = 1
            # ★ 不管 ego_dpx 是不是 0, 走進這支 = full projective 路徑
            info['ego_comp_mode'] = 'FULL_PROJECTIVE'
            info['full_ego_used'] = 1
        else:
            # ── 後援: 純 CV predict + 雲台粗近似 ego comp ──
            if self.enable_full_ego_comp and (
                    ac_roll_ned_deg is not None or target_depth_m is not None):
                # 想走 full 但資料不齊 (e.g. MAVROS 掉線 / 還沒估到 depth)
                self._full_ego_skipped_frames += 1
                info['full_ego_skipped'] = 1
            # 每幀都 predict (純 CV: x = F @ x)
            self.dkf.predict(dt=self.dt)

            # 2. ★ 雲台 Ego compensation (純旋轉近似)
            #    Predict 完後, 根據「上一步 → 這一步」的雲台姿態變化, 把 dkf state
            #    位置加上 pixel shift. 讓 DKF 認知「同一個世界點在新姿態下的 pixel」.
            #    SIYI A8 mini 約定: yaw 正 = 雲台往左, pitch 正 = 雲台往上
            #      du = +fx * Δyaw_rad
            #      dv = +fy * Δpitch_rad
            if (self.dkf.initialized
                    and gimbal_yaw_deg is not None and gimbal_pitch_deg is not None
                    and self._prev_gimbal_yaw_deg is not None):
                # ★★ 2026/09/12: 角度差必須 wrap 到 ±180.
                #   DIVE 時這裡塞的是【機身航向】(0~360 會繞圈), 不是雲台角
                #   (±150 不繞圈). 飛機每次穿越正北 359°→0°, 直接相減會得到
                #   -359° 而不是 +1°, DKF 會以為瞬間轉了一大圈.
                _dy = (gimbal_yaw_deg - self._prev_gimbal_yaw_deg + 180.0) \
                    % 360.0 - 180.0
                dy_rad = math.radians(_dy)
                dp_rad = math.radians(gimbal_pitch_deg - self._prev_gimbal_pitch_deg)
                du = self._ego_fx * dy_rad
                dv = self._ego_fy * dp_rad
                # 加到 DKF state 上 (predict 之後, 屬於 prior)
                self.dkf.x[0] += du
                self.dkf.x[1] += dv
                # 同步把 last_update_pos 跟 last_good_yolo_pos 也 shift
                # (這樣 hold mode 凍結時跟著畫面 ego shift 跑, outlier filter
                #  spatial 參考點也更新)
                if self._last_update_pos is not None:
                    self._last_update_pos = (
                        self._last_update_pos[0] + du,
                        self._last_update_pos[1] + dv,
                    )
                if self._last_good_yolo_pos is not None:
                    self._last_good_yolo_pos = (
                        self._last_good_yolo_pos[0] + du,
                        self._last_good_yolo_pos[1] + dv,
                    )
                # 統計
                self._total_ego_shift_u += du
                self._total_ego_shift_v += dv
                self._ego_comp_applied_frames += 1
                # ★ per-frame
                info['ego_du_px'] = du
                info['ego_dv_px'] = dv
                info['ego_comp_applied'] = 1
                info['ego_comp_mode'] = ('FALLBACK_CV' if self.enable_full_ego_comp
                                         else 'GIMBAL_ROTATION')
            else:
                # 沒套用任何 ego comp (沒雲台資料 / 第一幀)
                info['ego_comp_mode'] = ('FALLBACK_CV' if self.enable_full_ego_comp
                                         else 'NONE')

        # 更新「上一步雲台姿態」(粗近似路徑用; 無條件更新, 這樣 full ego 中途
        # 缺資料退回粗近似時 prev 值是新鮮的)
        if gimbal_yaw_deg is not None and gimbal_pitch_deg is not None:
            self._prev_gimbal_yaw_deg = float(gimbal_yaw_deg)
            self._prev_gimbal_pitch_deg = float(gimbal_pitch_deg)

        # 3. Outlier filter (在 update 之前)
        is_outlier = False
        if detected and u_yolo is not None and v_yolo is not None:
            high_conf = (confidence is not None
                         and confidence >= self.outlier_high_conf)
            has_recent_good = (
                self._last_good_yolo_pos is not None
                and self._frames_since_good_yolo <= self.outlier_max_age_frames
            )
            # 算 jump (給 CSV 用)
            if self._last_good_yolo_pos is not None:
                _dx = u_yolo - self._last_good_yolo_pos[0]
                _dy = v_yolo - self._last_good_yolo_pos[1]
                info['yolo_jump_px'] = float((_dx*_dx + _dy*_dy) ** 0.5)
            # 判斷接下來會走 RESET 還是 update
            will_reset = (
                not self.dkf.initialized
                or self.dkf._consecutive_miss >= self.reset_after_miss_frames
            )

            if will_reset:
                # ★ RESET 或初始化 = 重新開始追蹤新軌跡, 必須 high_conf 才信任.
                # 不管 spatial check, 防止 false positive 把系統 RESET 到錯誤位置
                # 後形成「同位置低信心 false positive 連坐通過」的死循環.
                if not high_conf:
                    is_outlier = True
                    self._total_outlier_rejected += 1
                    info['outlier_reason'] = 'LOW_CONF_RESET'
            elif has_recent_good:
                # 連續 update 場景: 跳躍 + high_conf 任一通過即可
                dx = u_yolo - self._last_good_yolo_pos[0]
                dy = v_yolo - self._last_good_yolo_pos[1]
                jump = (dx*dx + dy*dy) ** 0.5
                if jump > self.outlier_max_jump_px and not high_conf:
                    is_outlier = True
                    self._total_outlier_rejected += 1
                    info['outlier_reason'] = 'LOW_CONF_JUMP'
                # ★ 硬上限: 跳到「物理上不可能」的程度時, high_conf 不再豁免.
                #   解掉原本「跳 1200px 但 conf 0.75 → 當成真目標放行」的 bug.
                elif (self.outlier_hard_jump_px is not None
                      and jump > self.outlier_hard_jump_px):
                    is_outlier = True
                    self._total_outlier_rejected += 1
                    info['outlier_reason'] = 'HARD_JUMP'
                # ★ 框大小一致性: 單幀 bbox 面積比突變 (跳到大小不同的鄰車).
                #   俯衝放大是漸進的 (單幀面積比通常 <1.2), 跳到鄰車才會突變.
                #   非 high_conf 才擋, 避免誤殺真目標被部分遮擋時的尺寸抖動.
                elif (self.outlier_bbox_ratio is not None
                      and bbox_w is not None and bbox_h is not None
                      and self._last_good_bbox_area is not None
                      and self._last_good_bbox_area > 1.0
                      and not high_conf):
                    area = float(bbox_w) * float(bbox_h)
                    r = area / self._last_good_bbox_area
                    if r > self.outlier_bbox_ratio or r < 1.0 / self.outlier_bbox_ratio:
                        is_outlier = True
                        self._total_outlier_rejected += 1
                        info['outlier_reason'] = 'BBOX_SIZE_JUMP'
                        info['bbox_area_ratio'] = float(r)
            else:
                # 安全網 (理論上 will_reset 已 cover)
                if not high_conf:
                    is_outlier = True
                    self._total_outlier_rejected += 1
                    info['outlier_reason'] = 'LOW_CONF_NO_RECENT'
            info['outlier_rejected_this_frame'] = 1 if is_outlier else 0

        # 4. 接受的偵測 → update or reset
        if detected and not is_outlier and u_yolo is not None and v_yolo is not None:
            self._total_yolo_frames += 1
            z = np.array([float(u_yolo), float(v_yolo)])

            # 記住最後 bbox 尺寸 (給 overlay 在補位時畫框用)
            if bbox_w is not None:
                self.last_bbox_w = float(bbox_w)
            if bbox_h is not None:
                self.last_bbox_h = float(bbox_h)
            # ★ 記住最後良好 bbox 面積 (給下一幀 bbox 大小 gating 比對)
            if bbox_w is not None and bbox_h is not None:
                self._last_good_bbox_area = float(bbox_w) * float(bbox_h)

            # 處理長 miss 後重新偵測: 避免 KF gate 把新偵測當 outlier
            if (self.dkf.initialized
                    and self.dkf._consecutive_miss >= self.reset_after_miss_frames):
                self.dkf.reset(initial_position=z)
                self.dkf._history.append((self.dkf.x.copy(), self.dkf.P.copy(), np.zeros(2)))
                self._reset_count += 1
                status = 'RESET_INIT'
            else:
                # 標準 KF update with 延遲補償
                # delay_steps > 0: 內部回退 N 步做 update 再前推到當前
                self.dkf.update(
                    z_image=z,
                    delay_steps=delay_steps,
                    bbox_size=float(bbox_w) if bbox_w is not None else None,
                )
                status = 'YOLO'
                # ★ 把底層 update() 算的 innovation 拉出來 (給 CSV)
                info['innovation_u'] = float(getattr(self.dkf, '_last_innovation',
                                                     np.zeros(2))[0])
                info['innovation_v'] = float(getattr(self.dkf, '_last_innovation',
                                                     np.zeros(2))[1])
                info['innovation_norm'] = float(getattr(self.dkf,
                                                        '_last_innovation_norm', 0.0))
                info['mahalanobis2'] = float(getattr(self.dkf,
                                                     '_last_mahalanobis2', 0.0))

            # 強制 DKF 位置 = YOLO bbox 位置 (繞過 KF gain<1 的位置落後)
            # ★ 只在無延遲補償的純 CV 場景套用. 有延遲補償時, KF update 結果
            # 已經是「N 步前 YOLO + propagation 到現在」, 強制覆蓋會抵銷補償.
            if delay_steps == 0:
                self.dkf.x[0] = float(u_yolo)
                self.dkf.x[1] = float(v_yolo)

            # 記住 update / reset 後的位置 (給 hold mode 凍結用)
            self._last_update_pos = (float(u_yolo), float(v_yolo))
            # 更新「上次良好偵測」(給 outlier filter 用)
            self._last_good_yolo_pos = (float(u_yolo), float(v_yolo))
            self._frames_since_good_yolo = 0

            # ★ 成熟度計數: RESET_INIT 重新累積, 否則 +1
            if status == 'RESET_INIT':
                self._consecutive_yolo_updates = 1
            else:
                self._consecutive_yolo_updates += 1

            if self.use_yolo_when_available:
                info['status'] = status
                return float(u_yolo), float(v_yolo), False, status
            else:
                pos = self.dkf.get_position()
                info['status'] = status
                return float(pos[0]), float(pos[1]), False, status

        # 5. YOLO miss 或 outlier → DKF hold/coast
        else:
            # 計算 outlier filter 的「上次良好偵測距現在多久」
            self._frames_since_good_yolo += 1

            # ★ Hold timeout auto-reset: 太久沒新 YOLO confirm 就「忘掉」舊位置
            # 避免系統永遠卡在 false positive 初始位置
            if (self.dkf.initialized
                    and self._frames_since_good_yolo > self.hold_timeout_frames):
                self.dkf.initialized = False
                self._last_good_yolo_pos = None
                self._last_update_pos = None
                self.dkf.x = np.zeros(4)
                self.dkf.P = np.eye(4) * 100.0
                self.dkf._consecutive_miss = 0
                self._reset_count += 1
                self._consecutive_yolo_updates = 0  # ★ 全清, 下次要重新累積

            if not self.dkf.initialized:
                info['status'] = 'WAITING'
                return None, None, False, 'WAITING'

            # ★ 成熟度檢查: 累積 update 數不夠不對外輸出 coast 預測.
            #   理由: 觀測量太少時 DKF 內部速度估計沒收斂, 預測會把
            #   雲台帶到錯誤方向. 不如直接返回「沒目標」讓上層決定.
            if self._consecutive_yolo_updates < self.min_updates_to_coast:
                info['status'] = 'IMMATURE'
                return None, None, False, 'IMMATURE'

            self._total_dkf_coast_frames += 1

            # Hold mode: 凍結在最後 update 位置, 速度歸零
            if self.coast_mode == 'hold' and self._last_update_pos is not None:
                self.dkf.x[0] = self._last_update_pos[0]
                self.dkf.x[1] = self._last_update_pos[1]
                self.dkf.x[2] = 0.0
                self.dkf.x[3] = 0.0

            pos = self.dkf.get_position()
            u = float(np.clip(pos[0], 0.0, self.screen_w - 1.0))
            v = float(np.clip(pos[1], 0.0, self.screen_h - 1.0))
            status = 'OUTLIER' if is_outlier else 'DKF_COAST'
            info['status'] = status
            return u, v, True, status

    # ----------------------------------------------------------------
    #  Getter
    # ----------------------------------------------------------------
    def get_state(self):
        """
        回傳 DKF 內部狀態 (給 CSV / overlay 用).

        Returns:
            dict:
                u, v: 估計 pixel 座標 (None 表示還沒初始化)
                vu, vv: 估計 pixel 速度 (px/s)
                pos_std_u, pos_std_v: 位置標準差 (pixel)
                consecutive_miss: 連續沒收到 update 的幀數
                latency_ms: consecutive_miss * dt * 1000 (DKF 距上次 update 的時間)
                initialized: bool
                predict_count, update_count, reject_count: 累計次數
                reset_count: 強制 reset 次數
                dkf_coast_frames: 累計靠 DKF 補位的幀數
                yolo_frames: 累計 YOLO 偵測到的幀數
        """
        stats = self.dkf.get_stats()

        if not self.dkf.initialized:
            out = {
                'u': None, 'v': None,
                'vu': 0.0, 'vv': 0.0,
                'pos_std_u': 0.0, 'pos_std_v': 0.0,
                'pos_std_norm': 0.0,
                'consecutive_miss': 0,
                'latency_ms': 0.0,
                'initialized': False,
                'predict_count': stats['predict_count'],
                'update_count': stats['update_count'],
                'reject_count': stats['reject_count'],
                'reset_count': self._reset_count,
                'dkf_coast_frames': self._total_dkf_coast_frames,
                'yolo_frames': self._total_yolo_frames,
                'last_bbox_w': self.last_bbox_w,
                'last_bbox_h': self.last_bbox_h,
                'outlier_rejected': self._total_outlier_rejected,
                'ego_comp_frames': self._ego_comp_applied_frames,
                'delay_steps': self._last_delay_steps,
                'total_ego_shift_u': self._total_ego_shift_u,
                'total_ego_shift_v': self._total_ego_shift_v,
                'full_ego_frames': self._full_ego_frames,
                'full_ego_skipped_frames': self._full_ego_skipped_frames,
                'frames_since_good_yolo': self._frames_since_good_yolo,
            }
            out.update(self._last_step_info)
            return out

        std = self.dkf.get_position_std()
        miss = stats['consecutive_miss']
        out = {
            'u': float(self.dkf.x[0]),
            'v': float(self.dkf.x[1]),
            'vu': float(self.dkf.x[2]),
            'vv': float(self.dkf.x[3]),
            'pos_std_u': float(std[0]),
            'pos_std_v': float(std[1]),
            'pos_std_norm': float((std[0]**2 + std[1]**2) ** 0.5),
            'consecutive_miss': int(miss),
            'latency_ms': miss * self.dt * 1000.0,
            'initialized': True,
            'predict_count': stats['predict_count'],
            'update_count': stats['update_count'],
            'reject_count': stats['reject_count'],
            'reset_count': self._reset_count,
            'dkf_coast_frames': self._total_dkf_coast_frames,
            'yolo_frames': self._total_yolo_frames,
            'last_bbox_w': self.last_bbox_w,
            'last_bbox_h': self.last_bbox_h,
            'outlier_rejected': self._total_outlier_rejected,
            'ego_comp_frames': self._ego_comp_applied_frames,
            'delay_steps': self._last_delay_steps,
            'total_ego_shift_u': self._total_ego_shift_u,
            'total_ego_shift_v': self._total_ego_shift_v,
            'full_ego_frames': self._full_ego_frames,
            'full_ego_skipped_frames': self._full_ego_skipped_frames,
            'frames_since_good_yolo': self._frames_since_good_yolo,
            'consecutive_yolo_updates': self._consecutive_yolo_updates,
            'min_updates_to_coast': self.min_updates_to_coast,
        }
        # ★ 把 per-frame step_info 一起 merge 進來
        out.update(self._last_step_info)
        return out

    def reset(self):
        """完全重置 (用於任務切換 / target 切換)"""
        self.dkf.reset()
        self._reset_count = 0
        self._total_dkf_coast_frames = 0
        self._total_yolo_frames = 0
        self._consecutive_yolo_updates = 0  # ★ 成熟度計數歸零
        self._last_good_yolo_pos = None
        self._frames_since_good_yolo = 0


# ════════════════════════════════════════════════════════════════════
#  Self-test
# ════════════════════════════════════════════════════════════════════

# ========================================================================
# 原模組: ray_projection_distance.py
# ========================================================================
#!/usr/bin/env python3
# -*- coding: utf-8 -*-



# ==================== 旋轉矩陣工具 ====================
# 與 gimbal_controller_pinhole.py / vision_driver_car_dive_with_dkf_v3.py
# 逐字相同 — 不要改, 改了反投影 (reproject_target_ned_to_pixel) 就對不上.





# ==================== MAVROS ENU → 估測器 NED 姿態轉換 ====================

def convert_mavros_enu_to_ned_attitude(roll_enu_rad, pitch_enu_rad, yaw_enu_rad):
    """
    把 MAVROS /mavros/imu/data 的 euler (ENU 世界 + FLU 機體, 弧度)
    轉成本估測器幾何鏈要的 (NED 世界 + FRD 機體 + 真航向, 度).

    依據 ROS REP-103 / MAVROS 慣例, 常用且工程上實證可用的對應:
        roll_ned_deg  =  deg(roll_enu)            (正 = 右翼下沉)
        pitch_ned_deg = -deg(pitch_enu)           (正 = 抬頭)
        yaw_true_deg  =  (90 - deg(yaw_enu))      (0=北, 90=東, 順時針為正)

    其中 yaw 轉換是「ENU 的 0=東、逆時針正」→「真航向 0=北、順時針正」.

    ⚠️ 這三個正負號是實飛最常見的 bug 來源. 上面是標準對應, 但不同
       飛控/IMU 安裝方向仍可能差一個符號. 首飛前務必地面驗證 (機頭朝
       已知方向看 yaw_true 對不對; 機身前傾看 pitch_ned 是否變正).

    Args:
        roll_enu_rad, pitch_enu_rad, yaw_enu_rad: euler_from_quaternion
            ([q.x,q.y,q.z,q.w]) 直接吐出來的三個值 (弧度).
    Returns:
        (roll_deg, pitch_deg, yaw_true_deg)
        yaw_true_deg 已 wrap 到 [0, 360).
    """
    roll_deg = math.degrees(roll_enu_rad)
    pitch_deg = -math.degrees(pitch_enu_rad)
    yaw_true_deg = (90.0 - math.degrees(yaw_enu_rad)) % 360.0
    return roll_deg, pitch_deg, yaw_true_deg


# ==================== SIYI 雲台 → 估測器慣例轉換 ====================

def convert_siyi_gimbal_to_estimator(siyi_yaw_deg, siyi_pitch_deg,
                                     yaw_sign=-1.0, pitch_sign=1.0,
                                     yaw_offset_deg=0.0, pitch_offset_deg=0.0):
    """
    SIYI A8 getAttitude() 回傳 (yaw, pitch, roll) 度. 依你 test 檔註解:
        SIYI yaw:   正 = 雲台往左轉      (估測器要: 正 = 往右)  → yaw_sign  = -1
        SIYI pitch: 正 = 往上, 負 = 往下  (估測器要: 負 = 往下看) → pitch_sign = +1

    把每個正負號/offset 都做成參數, 因為這正是要地面校正的地方.
    若校正發現方向相反, 改對應 sign 即可 (不用動幾何).

    Args:
        siyi_yaw_deg, siyi_pitch_deg: SIYI getAttitude() 的 yaw, pitch.
        yaw_sign, pitch_sign: ±1, 校正方向.
        yaw_offset_deg, pitch_offset_deg: 機械零位常數偏差.
    Returns:
        (gimbal_pitch_deg, gimbal_yaw_deg) — 餵給 estimate_distance().
    """
    gimbal_yaw_deg = yaw_sign * siyi_yaw_deg + yaw_offset_deg
    gimbal_pitch_deg = pitch_sign * siyi_pitch_deg + pitch_offset_deg
    return gimbal_pitch_deg, gimbal_yaw_deg


# ==================== 射線投影距離估測器 (實飛標定內參版) ====================

class RayProjectionDistanceEstimator:
    """
    射線投影距離估測器 — 實飛標定內參版.

    與模擬版 V7.0 的唯一原理差別: 內參直接吃標定值, 不從 FOV 反推.
    幾何鏈、平滑、信心度算法完全相同.

    原理:
      1. 像素 (u,v) → 相機座標歸一化射線 Ic
      2. 旋轉鏈 Camera → Gimbal → Body → World(NED)
      3. 假設目標在地面, 用 AGL 高度反算 slant = altitude / Iw[Z]
      不需要目標尺寸, 只要相機標定 + 飛機姿態 + 雲台角度 + 高度.
    """

    def __init__(self,
                 fx, fy, cx, cy,
                 ema_alpha=0.3,
                 outlier_factor=2.0,
                 gimbal_pitch_offset_deg=0.0,
                 gimbal_yaw_offset_deg=0.0,
                 history_len=15,
                 min_iw_down=0.01,
                 conf_ref_iw_down=0.30,
                 verbose=True):
        """
        Args:
            fx, fy: 相機焦距 (pixel), 用你標定的 CAMERA_FX / CAMERA_FY.
            cx, cy: 主點 (pixel). 沒標定就傳 影像寬/2, 高/2.
            ema_alpha: EMA 平滑係數 (越小越平滑, 但 dive 時會 lag —
                       下游若用 last_raw_slant 可避開 lag, 見屬性說明).
            outlier_factor: 離群倍率, raw 偏離 median 超過此倍就 clip.
            gimbal_pitch_offset_deg: ✨ 雲台報告角度 vs 實際光軸的常數偏差(度).
                正值 = 實際光軸比報告值「更朝下」.
                === 地面校正法 ===
                飛機水平靜止 + 目標放正前方已知水平距離 D, 量到的 AGL=H,
                雲台對準目標讓它在畫面正中央. 此時理論俯角 = atan2(H, D).
                記下程式算出的 ground_range, 微調此 offset 直到 ground_range≈D.
            history_len: 平滑用的歷史窗格 (median + outlier 用).
            min_iw_down: Iw[2] 小於此值視為射線沒朝下, 無法估測.
            conf_ref_iw_down: 信心度 = clip(Iw[2]/此值, 0, 1).
                Iw[2]=sin(俯角), 0.30≈俯角17°時信心滿.
        """
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.gimbal_pitch_offset_deg = float(gimbal_pitch_offset_deg)
        # SIYI A8 三軸 world-stabilized: yaw 相對上電朝向, 用此 offset 對齊真北
        # (只影響目標方位/NED 座標, 不影響斜距與地面距離). 啟動時設
        #  gimbal_yaw_offset_deg = aircraft_yaw_true_at_start - gimbal_yaw_at_start.
        self.gimbal_yaw_offset_deg = float(gimbal_yaw_offset_deg)
        self.ema_alpha = float(ema_alpha)
        self.outlier_factor = float(outlier_factor)
        self.min_iw_down = float(min_iw_down)
        self.conf_ref_iw_down = float(conf_ref_iw_down)

        # Gimbal → Camera (CV: X右 Y下 Z前). 與兩個既有檔逐字相同.
        self.R_g2c = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [1, 0, 0],
        ], dtype=float)

        self.distance_history = deque(maxlen=int(history_len))
        self.ema_distance = None

        # Debug / 下游用 (與模擬版同名, drop-in 相容):
        self.last_ray_world = None      # 射線在 NED 的單位方向 (np.array(3,))
        self.last_raw_slant = None      # 未平滑斜距 (m) — dive 時餵 KF 用這個
        self.last_ground_range = None   # 未平滑地面距 (m)
        self.last_target_ned = None     # 目標推估 NED 絕對座標 [N, E, D] (m)
        self.last_confidence = 0.0

        if verbose:
            print(f"[RayProjectionDistanceEstimator RealFlight V1.0] 初始化:")
            print(f"  相機內參(標定): fx={self.fx:.2f} fy={self.fy:.2f} "
                  f"cx={self.cx:.1f} cy={self.cy:.1f}")
            print(f"  gimbal_pitch_offset={self.gimbal_pitch_offset_deg:+.2f}° "
                  f"(0=未校正, 首飛前務必地面校正)")
            print(f"  EMA α={self.ema_alpha} outlier×{self.outlier_factor} "
                  f"history={history_len}")

    # --------------------------------------------------------------
    def estimate_distance(self, pixel_u, pixel_v,
                          aircraft_roll, aircraft_pitch, aircraft_yaw_true,
                          gimbal_pitch, gimbal_yaw,
                          altitude_agl,
                          ac_x_ned=0.0, ac_y_ned=0.0):
        """
        射線投影估測到地面目標的距離.

        Args:
            pixel_u, pixel_v: 目標像素座標 (YOLO bbox 中心 或 DKF 預測點).
            aircraft_roll: 飛機滾轉 (度, 正=右翼下沉) — 已轉好的 NED 值.
            aircraft_pitch: 飛機俯仰 (度, 正=抬頭) — 已轉好的 NED 值.
            aircraft_yaw_true: 真航向 (度, 0=北 90=東) — 已轉好的值.
            gimbal_pitch: 雲台俯仰 (度, 負=向下看) — 已轉好的估測器慣例值.
            gimbal_yaw: 雲台偏航 (度, 正=向右看) — 已轉好的估測器慣例值.
            altitude_agl: AGL 高度 (m). 必須 > 0.
            ac_x_ned, ac_y_ned: 飛機 NED 北/東座標 (m), 用來算 target 絕對
                NED. 沒有就傳 0 (此時 last_target_ned 是「相對飛機」座標).

        Returns:
            (slant_range, ground_range, confidence)
            無法估測時回 (last_estimate_or_None, None, 0.0).
            另外設好 last_ray_world / last_raw_slant / last_ground_range /
            last_target_ned / last_confidence 供下游 (DKF NED fallback) 取用.
        """
        if altitude_agl is None or altitude_agl <= 0.5:
            self.last_confidence = 0.0
            return self._get_current_estimate(), None, 0.0

        # ---- Step 1: 像素 → 相機座標歸一化射線 ----
        x1 = (pixel_u - self.cx) / self.fx
        x2 = (pixel_v - self.cy) / self.fy
        Pc = np.array([x1, x2, 1.0])
        Ic = Pc / np.linalg.norm(Pc)

        # ---- Step 2: 旋轉鏈 (SIYI A8 三軸 world-stabilized) ----
        # SIYI A8 是三軸穩定雲台: 回報 pitch/roll 是「相對水平面」的世界角,
        # yaw 是「相對上電朝向」。相機在世界(NED)的指向只由雲台世界角決定,
        # 與飛機 roll/pitch 無關(雲台已自行穩定補償) → 不可再乘 R_w2b。
        # (此版淨結果等價 test_Visual_Main.py 的 R_g2c @ R_enu2gimbal。)
        # 註: aircraft_roll/pitch 不再進入幾何; aircraft_yaw_true 僅供啟動時
        #     計算 gimbal_yaw_offset_deg 用 (見 __init__)。
        gimbal_pitch_eff = gimbal_pitch - self.gimbal_pitch_offset_deg
        gimbal_yaw_world = gimbal_yaw + self.gimbal_yaw_offset_deg
        R_ned2gimbal = _Ry(gimbal_pitch_eff) @ _Rz(gimbal_yaw_world)
        R_ned2cam = self.R_g2c @ R_ned2gimbal
        R_cam2ned = R_ned2cam.T   # 正交矩陣的逆 = 轉置

        Iw = R_cam2ned @ Ic
        self.last_ray_world = Iw.copy()

        # ---- Step 3: 與地面交叉 ----
        # NED: 飛機在 z=-altitude_agl, 地面 z=0.
        # 射線 P = [acx, acy, -alt] + t*Iw, 解 -alt + t*Iw[2]=0 → t=alt/Iw[2]
        if Iw[2] <= self.min_iw_down:
            # 射線幾乎不朝下 (水平或朝上) → 無法估測
            self.last_confidence = 0.0
            self.last_target_ned = None
            return self._smooth(None), None, 0.0

        slant_range = altitude_agl / Iw[2]
        self.last_raw_slant = slant_range

        horizontal = math.sqrt(Iw[0] ** 2 + Iw[1] ** 2)
        ground_range = slant_range * horizontal
        self.last_ground_range = ground_range

        # 目標絕對 NED 座標 (供 vision_driver_v3 的 reproject / DKF fallback)
        # 飛機 NED = [ac_x, ac_y, -alt]; target = ac_ned + slant*Iw
        ac_ned = np.array([float(ac_x_ned), float(ac_y_ned),
                           -float(altitude_agl)])
        self.last_target_ned = ac_ned + slant_range * Iw

        # ---- Step 4: 信心度 ----
        # Iw[2]=sin(俯角): 射線越陡 → 高度反算越穩 → 信心越高
        confidence = float(np.clip(Iw[2] / self.conf_ref_iw_down, 0.0, 1.0))
        self.last_confidence = confidence

        # ---- Step 5: 平滑 ----
        smoothed_slant = self._smooth(slant_range)
        if smoothed_slant is not None and smoothed_slant > altitude_agl:
            smoothed_ground = math.sqrt(smoothed_slant ** 2 - altitude_agl ** 2)
        else:
            smoothed_ground = ground_range

        return smoothed_slant, smoothed_ground, confidence

    # --------------------------------------------------------------
    def _smooth(self, raw_distance):
        """EMA + median outlier rejection (與模擬版同算法)"""
        if raw_distance is None or raw_distance <= 0:
            return self._get_current_estimate()

        if len(self.distance_history) >= 5:
            median = float(np.median(list(self.distance_history)))
            if median > 0:
                ratio = raw_distance / median
                if ratio > self.outlier_factor or ratio < (1.0 / self.outlier_factor):
                    raw_distance = float(np.clip(
                        raw_distance,
                        median / self.outlier_factor,
                        median * self.outlier_factor))

        self.distance_history.append(raw_distance)
        median_dist = float(np.median(list(self.distance_history)))

        if self.ema_distance is None:
            self.ema_distance = median_dist
        else:
            self.ema_distance = (self.ema_alpha * median_dist +
                                 (1.0 - self.ema_alpha) * self.ema_distance)
        return self.ema_distance

    def _get_current_estimate(self):
        if self.ema_distance is not None:
            return self.ema_distance
        if len(self.distance_history) > 0:
            return float(np.median(list(self.distance_history)))
        return None

    def reset(self):
        self.distance_history.clear()
        self.ema_distance = None
        self.last_ray_world = None
        self.last_raw_slant = None
        self.last_ground_range = None
        self.last_target_ned = None
        self.last_confidence = 0.0


# ==================== Self-test ====================

# ========================================================================
# 原模組: vision.py
# ========================================================================
# -*- coding: utf-8 -*-





# ══════════════════════════════════════════════════════════════════
#  取像: 背景 thread 永遠拿最新幀 (吃掉 V4L2 排隊延遲)
# ══════════════════════════════════════════════════════════════════
class FrameGrabber:
    """背景取像. ★2026/09/09: raw 錄影搬到這條執行緒 (與 YOLO 解耦).

    原因: 舊架構 raw 影片是在 vision_loop 裡寫的, 只要 YOLO/DKF/雲台/CSV
    任一步丟例外, vision_loop 執行緒就死, raw 影片跟著停 —— 2026/09/08
    實飛就是這樣: 主 log 跑到 1075.6s, 視覺只到 754.7s, 影片缺了最後 5 分鐘.
    現在 raw 錄影只依賴「相機讀得到幀」這一件事, YOLO 掛掉不影響它.

    對齊: 每幀給一個 frame_id, vision log 記 raw_frame_idx, 後處理靠它對齊
    (不再假設 raw 影片第 N 幀 = 視覺 log 第 N 列).
    """

    def __init__(self, cfg):
        import cv2
        self.cfg = cfg
        self.cv2 = cv2
        self.cap = None
        self.frame = None
        self.frame_id = -1              # ★ 相機累計幀序 (raw 影片的幀號)
        self.lock = threading.Lock()
        self.running = False
        self.t = None
        self.recorder = None            # ★ 掛上 VideoWriter 就會逐幀錄
        self.captured = 0               # 相機成功讀到的幀數
        self.read_fail = 0              # 連續讀取失敗次數
        self.reopens = 0
        self.last_capture_t = 0.0
        self.frame_cap_mono = None      # ★ 2026/09/28: 最新幀的擷取時刻 (monotonic, 量延遲用)
        self.error = None

    def _open(self):
        cv2 = self.cv2
        for idx in (self.cfg.CAM_INDEX, self.cfg.CAM_INDEX + 1,
                    self.cfg.CAM_INDEX + 2):
            try:
                cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            except Exception:
                cap = cv2.VideoCapture(idx)
            if cap is None or not cap.isOpened():
                continue
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
            # ★ 2026/09/11 test_camera_bench 實測:
            #   這張擷取卡【只有 YUYV】, 舊的 MJPG 要求被無聲忽略 (實測
            #   MJPG 與 YUYV 明示結果完全相同), 改成明示避免誤導.
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
            # ★ 向 V4L2 要求的 fps 要【高於】實際需求.
            #   BUFFERSIZE=1 時, 消費端只要慢一點就會固定錯過下一幀:
            #     要求 30 (間隔 33ms), 執行緒被卡到 40ms → 掉成 15 fps
            #     要求 60 (間隔 17ms), 同樣卡 40ms       → 掉成 20 fps
            #   單獨跑時本路徑實測可達 60 fps; 整個程式跑起來會被 YOLO /
            #   NVENC / 控制迴路搶 CPU, 要求高一點才有排擠的餘裕.
            cap.set(cv2.CAP_PROP_FPS,
                    float(getattr(self.cfg, 'CAM_REQUEST_FPS',
                                  self.cfg.CAM_FPS)))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.cfg.IMG_W))
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.cfg.IMG_H))
            for _ in range(8):
                ok, f = cap.read()
                if ok and f is not None:
                    print(f"[Camera] V4L2 index={idx} OK")
                    return cap
            cap.release()
        return None

    def start(self):
        self.cap = self._open()
        if self.cap is None:
            return False
        self.running = True
        self.t = threading.Thread(target=self._run, daemon=True)
        self.t.start()
        for _ in range(50):
            with self.lock:
                if self.frame is not None:
                    return True
            time.sleep(0.1)
        return False

    def _run(self):
        """★ 這條執行緒只做兩件事: 讀相機 + 寫 raw 影片. 絕不能死."""
        while self.running:
            try:
                ok, f = self.cap.read()
            except Exception as e:
                ok, f = False, None
                self.error = f"cap.read() 例外: {e}"
            if ok and f is not None:
                self.read_fail = 0
                now = time.time()
                now_mono = time.monotonic()
                with self.lock:
                    self.frame = f
                    self.frame_id += 1
                    self.captured += 1
                    self.last_capture_t = now
                    self.frame_cap_mono = now_mono
                # raw 錄影 (與 YOLO 完全解耦)
                if self.recorder is not None:
                    try:
                        self.recorder.write(f, frame_id=self.frame_id)
                    except Exception as e:
                        self.error = f"raw 錄影 write 例外: {e}"
            else:
                self.read_fail += 1
                # 連續讀不到 → 嘗試重開相機 (別讓 raw 影片就此結束)
                if self.read_fail in (150, 600) or \
                        (self.read_fail > 600 and self.read_fail % 600 == 0):
                    print(f"[Camera] ★ 連續 {self.read_fail} 次讀不到幀, 嘗試重開")
                    try:
                        self.cap.release()
                    except Exception:
                        pass
                    cap = self._open()
                    if cap is not None:
                        self.cap = cap
                        self.reopens += 1
                        self.read_fail = 0
                        print(f"[Camera] ✓ 重開成功 (第 {self.reopens} 次)")
                time.sleep(0.001)

    def read(self):
        """相容舊介面: 只回傳最新幀."""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def read_with_id(self, since_id=None, with_t=False):
        """回傳 (最新幀, 該幀的相機幀序). 給 vision log 對齊 raw 影片用.

        since_id: 呼叫端上次拿到的幀序. 給了且幀序沒變 (= 重複幀) 就回
        (None, 同一個 id), 省掉 1080p 整張複製. 不給則維持舊行為.
        with_t: True → 多回傳該幀的擷取時刻 (time.monotonic()), 量延遲用.
                回傳 (frame, id, cap_mono). 預設 False, 舊呼叫端不受影響.
        """
        with self.lock:
            if self.frame is None:
                return (None, -1, None) if with_t else (None, -1)
            if since_id is not None and self.frame_id == since_id:
                return ((None, self.frame_id, self.frame_cap_mono) if with_t
                        else (None, self.frame_id))
            if with_t:
                return self.frame.copy(), self.frame_id, self.frame_cap_mono
            return self.frame.copy(), self.frame_id

    def stats(self):
        with self.lock:
            return {
                'captured': self.captured,
                'frame_id': self.frame_id,
                'read_fail': self.read_fail,
                'reopens': self.reopens,
                'age': (time.time() - self.last_capture_t
                        if self.last_capture_t else None),
                'error': self.error,
            }

    def stop(self):
        self.running = False
        if self.t is not None:
            self.t.join(timeout=1.0)
        try:
            self.cap.release()
        except Exception:
            pass


# ══════════════════════════════════════════════════════════════════
#  選框: 一個目標假設下, 從 YOLO 多框中挑「該追的那一個」
# ══════════════════════════════════════════════════════════════════
def select_detection(boxes, confs, locked, dkf_pred, img_w, img_h,
                     center_norm_max, track_gate_px):
    """
    boxes: list of (cx, cy, w, h);  confs: list;  兩者同長度 (可能為空)
    locked: 是否已鎖定 (已建立 DKF 追蹤)
    dkf_pred: (u, v) DKF 上一拍預測, 鎖定後選框基準; None 用畫面中心

    回傳選中框的 index, 或 None (沒有可接受的框).
      - 未鎖定 (首次截獲): 取「離畫面中心最近」且在 center_norm_max 內
      - 已鎖定: 取「離 dkf_pred 最近」且距離 ≤ track_gate_px
    → 別的車/樹只要不是最近的那個, 就選不上 (不看 conf 排序).
    """
    if not boxes:
        return None
    if locked and dkf_pred is not None:
        ref_u, ref_v = dkf_pred
        gate = track_gate_px
    else:
        ref_u, ref_v = img_w / 2.0, img_h / 2.0
        gate = center_norm_max * math.hypot(img_w / 2.0, img_h / 2.0)
    best_i, best_d = None, float('inf')
    for i, (cx, cy, w, h) in enumerate(boxes):
        d = math.hypot(cx - ref_u, cy - ref_v)
        if d < best_d:
            best_d, best_i = d, i
    if best_d > gate:
        return None
    return best_i


# ══════════════════════════════════════════════════════════════════
#  視覺前端狀態 (給控制迴路 / CSV / overlay 讀)
# ══════════════════════════════════════════════════════════════════
class VisionState:
    __slots__ = (
        'detected', 'conf', 'bbox_u', 'bbox_v', 'bbox_w', 'bbox_h',
        'dkf_used', 'dkf_status', 'u_used', 'v_used',
        'offset_norm', 'gate_pass', 'gate_dist_m',
        'slant_m', 'ground_m', 'raw_slant_m', 'raw_ground_m', 'tgt_n', 'tgt_e', 'range_conf',
        'inf_ms', 'delay_ms',
        'ray_pixel_valid', 'ray_pixel_source', 'ray_u', 'ray_v',
        'ray_estimate_mono', 'ray_last_yolo_mono', 'ray_is_prediction',
    )

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, None)
        self.detected = False
        self.dkf_used = False
        self.gate_pass = False


# ══════════════════════════════════════════════════════════════════
#  VisionFrontend — 對主程式的單一介面
# ══════════════════════════════════════════════════════════════════
class VisionFrontend:
    def __init__(self, cfg):
        self.cfg = cfg
        self.fx, self.fy, self.cx, self.cy = cfg.effective_intrinsics()

        # YOLO (延遲 import, 避免無 GPU 環境掛掉整包)
        from ultralytics import YOLO
        print(f"[YOLO] 載入 {cfg.YOLO_MODEL_PATH} ...")
        self.model = YOLO(cfg.YOLO_MODEL_PATH)
        dummy = np.zeros((cfg.IMG_H, cfg.IMG_W, 3), dtype=np.uint8)
        for _ in range(3):
            self.model.predict(dummy,imgsz=cfg.YOLO_IMGSZ, conf=cfg.YOLO_CONF, verbose=False)
        print("[YOLO] warmup 完成")

        self.reset_mission()

    def reset_mission(self):
        """Called by the vision thread between frames; retain warmed YOLO model."""
        cfg = self.cfg
        # DKF: 延遲補償後估測 → PID (use_yolo_when_available=False)
        self.dkf = DKFTrackerGimbal(
            fps=cfg.CAM_FPS,
            screen_width=cfg.IMG_W, screen_height=cfg.IMG_H,
            fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
            use_yolo_when_available=False,   # ★ 吃延遲補償後估測
            verbose=True)

        # 射線投影測距
        self.ranger = RayProjectionDistanceEstimator(
            fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
            gimbal_pitch_offset_deg=cfg.GIMBAL_PITCH_OFFSET_DEG,
            verbose=True)

        # 鎖定狀態
        self.locked = False
        self._acquire_streak = 0
        self._prev_dkf_pred = None
        self._frames_since_det = 0
        # 目標 NED 先驗 (閘門中心): 初始 = 種點前方; 鎖定後 EMA 更新
        self.tgt_prior = None        # (n, e) 或 None
        self._last_frame_annot = None
        self._ray_last_yolo_mono = None
        # ★ 2026/09/15: 上一幀的 ego 角度來源 (False=雲台角, True=機身姿態).
        #   來源切換那一幀不可相減, 見 step() 的 DKF 段.
        self._prev_dive_mode = False

    # ------------------------------------------------------------------
    def set_prior(self, n, e):
        """主程式在撥 GUIDED 種點時給閘門一個初始中心 (航跡前方 300m)."""
        self.tgt_prior = (float(n), float(e))

    # ------------------------------------------------------------------
    def _yolo(self, frame):
        """回傳 (boxes, confs, annotated_frame, inference_ms).
        boxes: list of (cx, cy, w, h)."""
        import cv2
        t0 = time.perf_counter()
        res = self.model.predict(frame,imgsz=self.cfg.YOLO_IMGSZ,conf=self.cfg.YOLO_CONF,
                                 classes=self.cfg.YOLO_TARGET_CLASS_IDS,
                                 verbose=False)[0]
        inf_ms = (time.perf_counter() - t0) * 1000.0
        boxes, confs = [], []
        annot = frame
        if res.boxes is not None and len(res.boxes) > 0:
            xyxy = res.boxes.xyxy.cpu().numpy()
            cf = res.boxes.conf.cpu().numpy()
            for (x1, y1, x2, y2), c in zip(xyxy, cf):
                boxes.append(((x1 + x2) / 2.0, (y1 + y2) / 2.0,
                              x2 - x1, y2 - y1))
                confs.append(float(c))
        return boxes, confs, annot, inf_ms

    # ------------------------------------------------------------------
    def _draw(self, frame, sel_box, other_boxes, u_used, v_used, dkf_used,
              gate_pass):
        """畫框: 選中綠框, 其他灰框, DKF 補位藍十字."""
        import cv2
        if sel_box is not None:
            cx, cy, w, h = sel_box
            col = (0, 255, 0) if gate_pass else (0, 165, 255)  # 綠=接受 橙=閘門拒
            cv2.rectangle(frame, (int(cx - w / 2), int(cy - h / 2)),
                          (int(cx + w / 2), int(cy + h / 2)), col, 2)
        for (cx, cy, w, h) in other_boxes:
            cv2.rectangle(frame, (int(cx - w / 2), int(cy - h / 2)),
                          (int(cx + w / 2), int(cy + h / 2)), (128, 128, 128), 1)
        if u_used is not None:
            c = (255, 128, 0) if dkf_used else (0, 0, 255)
            cv2.drawMarker(frame, (int(u_used), int(v_used)), c,
                           cv2.MARKER_CROSS, 20, 2)
        return frame

    # ------------------------------------------------------------------
    def step(self, frame, ac, gimbal, dive_mode, dt, cap_mono=None):
        """
        每幀處理一次. 回傳 (VisionState, annotated_frame).

        ac: dict — roll_enu/pitch_enu/yaw_enu (rad, MAVROS) 或已轉好的 NED,
            body_pitch_deg, body_yaw_true_deg, x_ned, y_ned, agl
        gimbal: dict — yaw/pitch (SIYI 原始度)
        dive_mode: True = DIVE 段 (FPV, ego comp 餵機身姿態; 仍測距記錄)
        """
        cfg = self.cfg
        st = VisionState()
        # DKF ego compensation and ray both use this step's aircraft/gimbal
        # snapshot. DKF pixels are propagated estimates, not capture-time bbox.
        reference_mono = ac.get('ray_reference_mono', time.monotonic())

        boxes, confs, annot, inf_ms = self._yolo(frame)
        st.inf_ms = inf_ms

        # ── 選框 (一個目標) ──
        sel = select_detection(
            boxes, confs, self.locked, self._prev_dkf_pred,
            cfg.IMG_W, cfg.IMG_H,
            cfg.ACQUIRE_CENTER_NORM_MAX, self._track_gate_px())
        sel_box = boxes[sel] if sel is not None else None
        other = [b for i, b in enumerate(boxes) if i != sel]

        # ── 地理閘門 (只在有先驗時檢查; 用當幀姿態反投影偵測到 NED) ──
        gate_pass = True
        gate_dist = None
        if sel_box is not None and self.tgt_prior is not None and not dive_mode:
            gate_pass, gate_dist = self._geo_gate(sel_box, ac, gimbal)
        st.gate_pass = gate_pass
        st.gate_dist_m = gate_dist

        det_ok = sel_box is not None and gate_pass
        if det_ok:
            self._frames_since_det = 0
        else:
            self._frames_since_det += 1

        # ── 首次截獲判定 (未鎖定): 連續穩定居中才鎖 ──
        if not self.locked:
            if det_ok:
                cx, cy, w, h = sel_box
                off = math.hypot(cx - cfg.IMG_W / 2.0, cy - cfg.IMG_H / 2.0) \
                    / math.hypot(cfg.IMG_W / 2.0, cfg.IMG_H / 2.0)
                if off <= cfg.ACQUIRE_CENTER_NORM_MAX:
                    self._acquire_streak += 1
                else:
                    self._acquire_streak = 0
                if self._acquire_streak >= cfg.ACQUIRE_STREAK:
                    self.locked = True
                    print(f"[Vision] ★ 鎖定目標 (連續 {cfg.ACQUIRE_STREAK} 幀居中)")
            else:
                self._acquire_streak = 0

        # ── 延遲補償量 (YOLO 推論 + 擷取鏈) ──
        delay_ms = inf_ms + cfg.CAPTURE_LATENCY_MS
        st.delay_ms = delay_ms

        # ── DKF: DIVE 餵機身姿態差 (FPV body-fixed), 其他餵雲台角 ──
        # ★★ 2026/09/15 修正兩件事:
        #   (a) 來源切換跳變: 進/出 DIVE 那一幀, DKF 記的「上一幀角度」還是
        #       另一種來源 (雲台相對角 vs 機身羅盤航向), 直接相減會得到
        #       任意大的 Δ (例: 0° → 90° ≈ 1700 px), DKF 狀態被推飛.
        #       → 切換時清掉 prev, 這一幀不做 ego comp, 下一幀起正常.
        #   (b) yaw 符號: DKF 粗近似沿用 SIYI 慣例 (yaw 正 = 往左, du=+fx·Δyaw).
        #       羅盤航向是【往右為正】, 直接餵進去補償方向剛好相反.
        #       (project_body_camera 驗算: 航向 +1° → 實際 du=-16 px,
        #        舊寫法預測 +18.6 px.) pitch 兩者都是抬頭為正, 不用改.
        if dive_mode != self._prev_dive_mode:
            self.dkf._prev_gimbal_yaw_deg = None
            self.dkf._prev_gimbal_pitch_deg = None
            self._prev_dive_mode = dive_mode
        if dive_mode:
            _hdg = ac.get('body_yaw_true_deg')
            ego_yaw = -_hdg if _hdg is not None else None   # 右正 → SIYI 左正
            ego_pitch = ac.get('body_pitch_deg')
        else:
            ego_yaw, ego_pitch = gimbal['yaw'], gimbal['pitch']

        u_yolo = v_yolo = bw = bh = conf = None
        if det_ok:
            u_yolo, v_yolo, bw, bh = sel_box[0], sel_box[1], sel_box[2], sel_box[3]
            conf = confs[sel]

        u_used, v_used, dkf_used, status = self.dkf.step(
            detected=det_ok,
            u_yolo=u_yolo, v_yolo=v_yolo, bbox_w=bw, bbox_h=bh,
            dt=dt, confidence=conf,
            gimbal_yaw_deg=ego_yaw, gimbal_pitch_deg=ego_pitch,
            measurement_delay_ms=delay_ms)

        st.detected = det_ok
        st.conf = conf
        st.bbox_u, st.bbox_v = (sel_box[0], sel_box[1]) if sel_box else (None, None)
        st.bbox_w, st.bbox_h = (sel_box[2], sel_box[3]) if sel_box else (None, None)
        st.u_used, st.v_used = u_used, v_used
        st.dkf_used, st.dkf_status = dkf_used, status
        if u_used is not None:
            st.offset_norm = math.hypot(u_used - self.cx, v_used - self.cy) \
                / math.hypot(cfg.IMG_W / 2.0, cfg.IMG_H / 2.0)
            self._prev_dkf_pred = (u_used, v_used)

        # Only DKF-accepted measurements refresh the prediction lifetime.
        accepted = det_ok and status in ('YOLO', 'RESET_INIT')
        capture_time = (cap_mono if cap_mono is not None else reference_mono)
        measurement_mono = capture_time - cfg.CAPTURE_LATENCY_MS / 1000.
        if accepted:
            self._ray_last_yolo_mono = measurement_mono
        use_dkf = bool(getattr(cfg, 'RAY_USE_DKF_PIXELS', True))
        predicted = bool(use_dkf and dkf_used)
        st.ray_is_prediction = predicted
        st.ray_pixel_source = ('dkf_prediction' if predicted else 'dkf_estimate') \
            if use_dkf else 'yolo_raw'
        st.ray_u, st.ray_v = (u_used, v_used) if use_dkf else (u_yolo, v_yolo)
        st.ray_estimate_mono = reference_mono if use_dkf else measurement_mono
        st.ray_last_yolo_mono = self._ray_last_yolo_mono
        pixel_finite = all(x is not None and np.isfinite(x)
                           for x in (st.ray_u, st.ray_v))
        prediction_age = (time.monotonic() - self._ray_last_yolo_mono
                          if self._ray_last_yolo_mono is not None else None)
        prediction_fresh = (prediction_age is not None and
                            0 <= prediction_age <= cfg.RAY_DKF_MAX_PREDICTION_AGE_S)
        st.ray_pixel_valid = bool(pixel_finite and (
            (accepted or (predicted and prediction_fresh)) if use_dkf else det_ok))
        if st.ray_pixel_valid and ac.get('agl'):
            self._range(st, st.ray_u, st.ray_v, ac, gimbal)
            # Do not let an unconfirmed prediction reinforce the geo-gate prior.
            if (not predicted and not dive_mode and self.locked
                    and st.tgt_n is not None):
                if self.tgt_prior is None:
                    self.tgt_prior = (st.tgt_n, st.tgt_e)
                else:
                    a = 0.3
                    self.tgt_prior = (
                        (1 - a) * self.tgt_prior[0] + a * st.tgt_n,
                        (1 - a) * self.tgt_prior[1] + a * st.tgt_e)

        annot = self._draw(annot, sel_box, other, u_used, v_used, dkf_used,
                           gate_pass)
        return st, annot

    # ------------------------------------------------------------------
    def _track_gate_px(self):
        """鎖定後選框像素閘門, 隨掉幀放寬."""
        t_lost = self._frames_since_det / max(self.cfg.CAM_FPS, 1.0)
        return self.cfg.TRACK_GATE_PX + self.cfg.TRACK_GATE_PX_PER_S * t_lost

    def _gate_radius_m(self):
        """地理閘門半徑, 動態目標隨丟失時間膨脹.

        ★★ 2026/09/12 修正: 原本寫 cfg.TARGET_MOVING, 但 config 定義的是
        MOVING_TARGET —— AttributeError.
        這不是死碼: 呼叫路徑是 _geo_gate(), 條件 tgt_prior is not None,
        也就是【目標鎖定建立之後每一幀都會踩到】.
        2026/09/08 那趟視覺執行緒在 t=754.7s 猝死, 就是這行:
          t=470 有 3 幀偵測但 ACQUIRE_STREAK=5 沒滿 → 未鎖定 → 走不到這裡
          t=754 連續 8 幀偵測 → streak 滿 → 建立 tgt_prior
                → 下一幀進 _geo_gate → 這行炸 → 舊版無 try/except → 執行緒死
          視覺 log 最後一筆 frame 18254 = 最後一筆偵測的同一幀, 完全吻合.
        """
        r = self.cfg.GATE_RADIUS_M
        if getattr(self.cfg, 'MOVING_TARGET', False):
            t_lost = self._frames_since_det / max(self.cfg.CAM_FPS, 1.0)
            r += self.cfg.GATE_MOVING_VMAX_MPS * t_lost
        return r

    def _ned_attitude(self, ac):
        """取 NED 姿態 (優先用已轉好的, 否則從 ENU rad 轉)."""
        if ac.get('roll_ned_deg') is not None:
            return (ac['roll_ned_deg'], ac['pitch_ned_deg'],
                    ac['yaw_true_deg'])
        return convert_mavros_enu_to_ned_attitude(
            ac['roll_enu'], ac['pitch_enu'], ac['yaw_enu'])

    def _geo_gate(self, sel_box, ac, gimbal):
        """偵測反投影到 NED, 與先驗比距離. 回傳 (pass, dist_m)."""
        cx, cy = sel_box[0], sel_box[1]
        roll, pitch, yaw_t = self._ned_attitude(ac)
        gp, gy = convert_siyi_gimbal_to_estimator(
            gimbal['yaw'], gimbal['pitch'],
            yaw_sign=self.cfg.SIYI_YAW_SIGN, pitch_sign=self.cfg.SIYI_PITCH_SIGN)
        slant, ground, conf = self.ranger.estimate_distance(
            cx, cy, roll, pitch, yaw_t, gp, gy,
            ac['agl'], ac.get('x_ned', 0.0), ac.get('y_ned', 0.0))
        if self.ranger.last_target_ned is None:
            return True, None   # 無法估 (射線不朝下) → 不擋, 交 streak 判斷
        tn, te = self.ranger.last_target_ned[0], self.ranger.last_target_ned[1]
        d = math.hypot(tn - self.tgt_prior[0], te - self.tgt_prior[1])
        return (d <= self._gate_radius_m()), d

    def _range(self, st, u, v, ac, gimbal):
        """射線投影測距, 填 st.slant_m/ground_m/tgt_n/tgt_e/range_conf."""
        roll, pitch, yaw_t = self._ned_attitude(ac)
        gp, gy = convert_siyi_gimbal_to_estimator(
            gimbal['yaw'], gimbal['pitch'],
            yaw_sign=self.cfg.SIYI_YAW_SIGN, pitch_sign=self.cfg.SIYI_PITCH_SIGN)
        slant, ground, conf = self.ranger.estimate_distance(
            u, v, roll, pitch, yaw_t, gp, gy,
            ac['agl'], ac.get('x_ned', 0.0), ac.get('y_ned', 0.0))
        st.slant_m, st.ground_m, st.range_conf = slant, ground, conf
        if conf > 0 and ground is not None:
            st.raw_slant_m = self.ranger.last_raw_slant
            st.raw_ground_m = self.ranger.last_ground_range
        if conf > 0 and self.ranger.last_target_ned is not None:
            st.tgt_n = float(self.ranger.last_target_ned[0])
            st.tgt_e = float(self.ranger.last_target_ned[1])
