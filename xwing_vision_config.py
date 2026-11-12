# -*- coding: utf-8 -*-
"""
xwing_vision_config.py — X-wing 全視覺版全部參數
═══════════════════════════════════════════════════════════════════
任務 (與 hsu_realflight_true 的 xwing_virtual 相同場景):
      AUTO 240 m / 30 m/s → WP3 撥 GUIDED → SEARCH → PRE_DIVE(推頭到 -30°)
      → 30° DIVE → PULLOUT(140 m 拉平) → TECS 140 m 平飛
差別: ★ 只換「目標輸入的來源」: 虛擬軌跡 → 對方雲台的目標偵測 (UART).
      狀態機與判斷邏輯 (dive_guidance) 完全沿用原專案, 沒有新增條件:
      · SEARCH:   航向 = 飛機指向目標位置 (偵測算出; 還沒看到車前用虛擬目標帶路)
      · 推頭:     到目標水平距 ≤ 415.69 m (= 240 / tan30°)
      · PRE_DIVE: 目標遺失 > 1.5 s → ABORT (原 ABORT_TARGET_LOST_S)
      · DIVE:     目標在畫面的位置進 PID; 目標遺失 > 1.5 s → ABORT

任務幾何、PID、方向舵、安全參數全部沿用 xwing_virtual_config.py
(hsu_realflight_true, 2026/11/04 實飛版), 數值與註解保持一致.
"""

import math


class XwingVisionConfig:
    # ════════════════════════════════════════════════════════════
    #  任務幾何 (= xwing_virtual_config)
    # ════════════════════════════════════════════════════════════
    CRUISE_AGL_M = 240.0        # 巡航/進場高度 (AUTO 繞圈高度)
    DIVE_ANGLE_DEG = 30.0       # 俯衝角 (正值)
    PULLOUT_AGL_M = 140.0       # 俯衝 240 → 140 m, 到 140 m 拉平
                                #   (2026/11/01 由 120 改 140: 10/29 實飛改平最低掉到 97~105 m)
    PULLOUT_VMAX_MPS = 60.0     # 速度上限: 空速 ≥ 此值即提前拉平
    TECS_HOLD_ALT_M = 140.0     # 拉平後 TECS 保持高度 (= PULLOUT_AGL)
    TECS_SPEED_MPS = 30.0       # 拉平 / DONE 的 TECS 目標空速
    SEARCH_SPEED_MPS = 30.0     # SEARCH 巡航空速 (與 AUTO 一致)
    #   ⚠ 43000 CHANGE_SPEED 超出飛控空速上下限會被拒: AIRSPEED_MIN ≤ 30 ≤ AIRSPEED_MAX
    GUIDED_ENTRY_ROLL0_S = 0.5

    ALPHA_NOM_DEG = 1.6         # 巡航攻角 (SEARCH 段 pitch 交 TECS, 此值只進 log)
    ALPHA_DIVE_DEG = 0.0        # 俯衝攻角 — 瞄準鏈用這個

    PREDIVE_HANDOFF_PITCH_TOL_DEG = 3.0   # pitch 壓到 -30±3° 內 (且看得到目標) 交棒 DIVE
    ENTRY_MIN_AGL_M = 200.0     # 低於此高度不准進 PRE_DIVE
    PREDIVE_TIMEOUT_S = 45.0

    # ════════════════════════════════════════════════════════════
    #  空速 / 油門 (= xwing_virtual_config)
    # ════════════════════════════════════════════════════════════
    V_CRUISE_MPS = 30.0
    V_STALL_MPS = 12.0
    THR_TRIM = 0.45             # ⚠ 20 m/s 時的配平油門, 30 m/s 實際值待從 .BIN 讀回
    THR_MIN = 0.05
    THR_MAX = 0.85

    # ════════════════════════════════════════════════════════════
    #  安全
    # ════════════════════════════════════════════════════════════
    ABORT_MIN_AGL_M = 100.0     # 低於此高度 ABORT 拉起 (PULLOUT 140 m 先觸發; 這是最後一道)
    ABORT_TARGET_LOST_S = 1.5   # PRE_DIVE / DIVE 看不到目標超過此秒數 → ABORT (原邏輯)

    # ════════════════════════════════════════════════════════════
    #  控制座標系 (虛擬相機) — DIVE PID 的像素 / 角度換算用
    #  對方相機的真實像素會先經雲台補償換到這組內參再進 PID,
    #  所以 PID 增益的物理意義與 xwing_virtual 完全相同.
    # ════════════════════════════════════════════════════════════
    IMG_W = 1920
    IMG_H = 1080
    FX_1X = 1066.7
    FY_1X = 1059.5
    CX_1X = 959.4932
    CY_1X = 545.0292
    VIRTUAL_ZOOM_LEVEL = 1.0

    CONTROL_HZ = 50.0

    # ════════════════════════════════════════════════════════════
    #  DIVE 段 PID (= xwing_virtual_config, 11/04 實飛值)
    # ════════════════════════════════════════════════════════════
    PID_LAT_KP = 2.0
    PID_LAT_KI = 0.20
    PID_LAT_KD = 0.0
    PID_LON_KP = 0.8
    PID_LON_KI = 0.1
    PID_LON_KD = 0.2
    PID_SP_LEAD_S = 0.0
    PID_MEAS_TAU_S = 0.10
    PID_D_TAU_S = 0.20
    PID_OUT_TAU_S = 0.0

    # ════════════════════════════════════════════════════════════
    #  DIVE / PRE_DIVE 橫向: 方向舵迴路 (= xwing_virtual_config)
    # ════════════════════════════════════════════════════════════
    RUDDER_LIMIT_DEG = 30.0
    K_PSI = 0.30
    K_R = 0.15
    K_LAT_TO_RUDDER = 0.50
    RUDDER_SLEW_DPS = 20.0
    RUDDER_VSCHED = ((999.0, 1.00),)
    ROLL_PROT_FULL_DEG = 25.0
    ROLL_PROT_ZERO_DEG = 35.0

    # ════════════════════════════════════════════════════════════
    #  虛擬目標 (= xwing_virtual_config) — ★ 只用來「帶路」與「對照」:
    #  · 還沒偵測到車之前, SEARCH 朝它飛 (讓機頭對著馬路, 相機才看得到車)
    #  · 撥 GUIDED 啟動檢查用起點判斷位置 / 方向 (原邏輯)
    #  · log 的 tgt_* 欄位照記, 可跟偵測到的目標位置 (vg_tgt_*) 比
    # ════════════════════════════════════════════════════════════
    MOVING_TARGET = True
    # ★ 2026/11/10 = B (hsu_realflight_true xwing_virtual_config, 8fd3a6e) 同步:
    #   改成「由南往北」攻擊, 再整條航線往南平移 250 m.
    #   任務: WP2 東南角南 250 m (左轉往北) → 401 m → WP3 23.7235958, 120.3798693 (撥 GUIDED)
    #         → 571 m → WP4 東北角南 250 m (★ 虛擬目標起點). 目標往北 (0.0°), 跟飛機進場同方向.
    #   舊版 (11/01~11/04, 由北往南): 起點 (23.7222419, 120.3798848), 航向點
    #     (23.7211280, 120.3798848) → 180.0°; 要改回去就把下面兩行換回這兩個座標.
    MOVING_START_GPS = (23.7287278, 120.3798473)   # ★ 2026/11/10 = 新 WP4 (東北角南 250 m)
    MOVING_HEADING_GPS = (23.7298417, 120.3798473) # 起點正北 124 m → 方位 0.0° (往北)
    MOVING_HEADING_DEG = None
    TARGET_SPEED_MPS = 10.0
    MOVING_STOP_AT_HEADING_GPS = False
    MOVING_TARGET_LEAD = False  # 前置瞄準 (關: DIVE setpoint = 畫面中心, 純追擊)
    TARGET_GPS = MOVING_START_GPS

    # ════════════════════════════════════════════════════════════
    #  ★ 推頭依據 (2026/11/11 與老師討論後改)
    #   'angle'    : 看車的俯角 (雲台往下角度 + 車在畫面上下偏的角度) ≥ 觸發角 就推頭.
    #                不用高度 → 飛機高度偏差、目標區地面比起飛點高或低都不影響.
    #   'distance' : 偵測水平距 (= 高度 / tan 俯角) ≤ PREDIVE_DIST_M (原本做法).
    #   兩者在飛機剛好 240 m、地面與起飛點等高時是同一個時間點.
    # ════════════════════════════════════════════════════════════
    PREDIVE_TRIGGER = 'angle'
    PREDIVE_TRIGGER_ANGLE_DEG = None  # None = 俯衝角 DIVE_ANGLE_DEG (30°)

    # 推頭距離 = 巡航高度 / tan(俯衝角) = 240 / tan30° = 415.69 m
    #   (同原專案; distance 模式的門檻. angle 模式下狀態機仍比這個數字,
    #    但餵進去的是等效距離, 見 vision_guidance.py)
    PREDIVE_DIST_M = CRUISE_AGL_M / math.tan(math.radians(DIVE_ANGLE_DEG))
    STATIC_PREDIVE_DIST_M = PREDIVE_DIST_M

    # ════════════════════════════════════════════════════════════
    #  ★ 目標偵測 → 目標輸入 (vision_guidance.VisionSource)
    # ════════════════════════════════════════════════════════════
    VISION_PKT_MAX_AGE_S = 0.3        # 最新封包超過這麼久沒更新 → 這拍算「看不到目標」
                                      #   (對方約 30 Hz, 0.3 s ≈ 連續 9 筆沒收到)

    # ── 相機幾何 (對方雲台, 2026/11 確認) ──
    #   pitch: 對地角, 往下為負, 拍照那一刻的值 → 符號 +1
    #   roll:  相機 roll, 雲台維持 0°
    #   yaw:   沒有 yaw 軸, 相機朝機頭 → 相機方位 = 飛機 yaw + 安裝偏差
    GIMBAL_PITCH_SIGN = 1.0
    GIMBAL_PITCH_OFFSET_DEG = 0.0     # ★ 地面校正: 光軸實際俯角 − 回報值 (往上為正)
    GIMBAL_ROLL_SIGN = 1.0
    GIMBAL_ROLL_OFFSET_DEG = 0.0
    GIMBAL_YAW_OFFSET_DEG = 0.0       # ★ 地面校正: 光軸偏機頭右方幾度
    # ── 雲台 yaw (2026/11/09 共用接收端 V6 新增的欄位) ──
    #   'none'  : 不用 yaw, 相機方位 = 飛機 yaw (= 2026/11 確認的「沒有 yaw 軸」, 預設)
    #   'body'  : yaw = 相機相對機頭 (右為正)   → 相機方位 = 飛機 yaw + yaw
    #   'earth' : yaw = 相機真方位 (對地)       → 相機方位 = yaw
    #   ★ 2026/11/10 確認: yaw = 對地角度, 雲台 yaw 保持置中 (相機一直朝機頭).
    #     → 'none' 與 'earth' 理論上同一個方向. 維持 'none' (用飛控的機頭航向:
    #       EKF 融合 GPS / 羅盤, 跟目標位置、導航同一個基準). 雲台 yaw 記在
    #       vg_gimbal_yaw_deg / vg_yaw_diff_deg 對照, 差值穩定才考慮改 'earth'.
    #   yaw 收到 NaN 時一律當 'none'
    GIMBAL_YAW_MODE = 'none'
    GIMBAL_YAW_SIGN = 1.0
    POSE_MAX_GAP_S = 0.15             # 取像時刻找飛機姿態, 前後樣本最遠差距
    # 地面 / 掛機 (2026/11/12): 飛控高度低於此值 → 偵測的角度照算 (地面校正、掛機測推頭用);
    #   距離 / 目標位置要用高度, 高度 ≤ 0.5 m 時留空. 空中 (≥ 此值) 行為不變.
    VISION_GROUND_AGL_M = 20.0        # = ARM_GROUND_AGL_M (啟動檢查也用 20 m 判斷在地面)

    # ════════════════════════════════════════════════════════════
    #  雲台視覺 UART (= xwing_virtual_config; 協定見 uart_mipi_receiver.py: 共用接收端 V6, 新格式 0~5 / 10, 也收舊 v6 / v7 / v8)
    # ════════════════════════════════════════════════════════════
    VISION_UART_ENABLED = True
    VISION_UART_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
    VISION_UART_BAUD = 115200
    VISION_STALE_S = 2.0              # 這麼久沒收到有效封包 → events 記「資料鏈中斷」

    # ── 依任務狀態送模式鍵 ('0' 手動 / '1' 追蹤 / '2' 鎖定 −30° 對地) — 同 xwing_virtual ──
    GIMBAL_MODE_CMD_ENABLED = True
    GIMBAL_MODE_KEYS = {
        'STANDBY':  None,
        'SEARCH':   '1',
        'PRE_DIVE': '2',
        'DIVE':     '2',
        'PULLOUT':  None,
        'DONE':     None,
        'ABORT':    None,
    }
    GIMBAL_MODE_RESEND_S = 1.0
    GIMBAL_MODE_MAX_TRIES = 3
    GIMBAL_LOCK_KEY = '2'
    GIMBAL_LOCK_PITCH_DEG = -30.0     # 鎖定時回報 −30° (對地, 不受機頭影響)
    GIMBAL_LOCK_TOL_DEG = 2.0

    # ════════════════════════════════════════════════════════════
    #  重複撥 GUIDED / 啟動檢查 (= xwing_virtual_config)
    # ════════════════════════════════════════════════════════════
    REARM_OFF_S = 1.0
    ARM_MIN_MARGIN_M = 70.0     # 離起點至少 = 推頭距離 + 此值 (416 + 70 = 486 m)
    ARM_MAX_DIST_M = 1500.0
    ARM_MAX_BEARING_OFF_DEG = 45.0
    ARM_GROUND_AGL_M = 20.0

    # ════════════════════════════════════════════════════════════
    #  輸出 / 斷電保護
    # ════════════════════════════════════════════════════════════
    OUT_DIR = 'flight_logs'
    FSYNC_HZ = 4.0

    def __init__(self, **overrides):
        for k, v in overrides.items():
            if not hasattr(self, k):
                raise KeyError(f"XwingVisionConfig 沒有參數 {k}")
            setattr(self, k, v)
        if ({'CRUISE_AGL_M', 'DIVE_ANGLE_DEG'} & set(overrides)
                and 'PREDIVE_DIST_M' not in overrides):
            self.PREDIVE_DIST_M = self.predive_ground_range_m
            if 'STATIC_PREDIVE_DIST_M' not in overrides:
                self.STATIC_PREDIVE_DIST_M = self.PREDIVE_DIST_M

    # ------------------------------------------------------------------
    def intrinsics_at_zoom(self, z):
        z = float(z)
        fx = self.FX_1X * z
        fy = self.FY_1X * z
        cx = (self.CX_1X - self.IMG_W / 2.0) * z + self.IMG_W / 2.0
        cy = (self.CY_1X - self.IMG_H / 2.0) * z + self.IMG_H / 2.0
        return fx, fy, cx, cy

    def virtual_intrinsics(self):
        """控制座標系內參."""
        return self.intrinsics_at_zoom(self.VIRTUAL_ZOOM_LEVEL)

    @property
    def virtual_aim_v_offset_px(self):
        fx, fy, cx, cy = self.virtual_intrinsics()
        return fy * math.tan(math.radians(self.ALPHA_DIVE_DEG))

    @property
    def predive_ground_range_m(self):
        return self.CRUISE_AGL_M / math.tan(math.radians(self.DIVE_ANGLE_DEG))

    @property
    def dive_ground_range_m(self):
        return ((self.CRUISE_AGL_M - self.PULLOUT_AGL_M)
                / math.tan(math.radians(self.DIVE_ANGLE_DEG)))
