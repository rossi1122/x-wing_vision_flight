# -*- coding: utf-8 -*-
"""
config.py — vision_dive 全部參數集中在這裡
═══════════════════════════════════════════════════════════════════
任務: AUTO 繞圈對準 → 撥 GUIDED 開始 → SEARCH(雲台鎖定目標) → PRE_DIVE
      → 30° DIVE(影像 PID, 雲台 FPV body-fixed) → PULLOUT 拉平 → TECS 平飛
感知: 單目相機 (SIYI A8 mini, 開機固定 ZOOM_LEVEL=2x; 虛擬目標另用 1x) + YOLOv8 + DKF + 射線投影測距
      目標由視覺提供 (真實 bbox → NED 座標), 不依賴 GPS 目標先驗.
機體: HERO2180 VTOL 固定翼 (巡航 17-22 m/s, 失速 12 m/s, MTOW 7kg)

★ 一個目標假設: 全程只鎖一台車. 選框規則 = 「離 DKF 預測點最近」+ 地理閘門,
  別的目標 conf 再高也搶不走. 切 GUIDED 前雲台不追 (FOLLOW yaw 跟機頭), 對準
  即撥 GUIDED, 目標就在畫面中央 → 首次截獲取「離畫面中心最近」.
"""

import math


class Config:
    # ════════════════════════════════════════════════════════════
    #  任務幾何
    # ════════════════════════════════════════════════════════════
    CRUISE_AGL_M = 120.0        # 巡航/進場高度 (AUTO 繞圈高度)
    DIVE_ANGLE_DEG = 25.0       # 俯衝角 (正值)
    PULLOUT_AGL_M = 70.0
    PULLOUT_VMAX_MPS = 30.0     # 超速即提前拉平 (安全上限, 幾乎每趟由此觸發)
    TECS_HOLD_ALT_M = 70.0      # 拉平後 TECS 保持高度 (進 PRE_DIVE 時就先送,
                                #   讓 TECS 內部積分器/濾波器有時間收斂)
    TECS_SPEED_MPS = 18.0       # 拉平 / DONE 的 TECS 目標空速
    # ★ 2026/09/11: SEARCH 段的空速獨立出來, 與 AUTO 巡航一致.
    #   撥 GUIDED 後 SEARCH 的 pitch/高度/速度全交 TECS, 若沿用 18 會與
    #   AUTO 的巡航速度不同, 一撥進去就開始加速, 白白擾動配平.
    #   拉平仍用 TECS_SPEED_MPS (較高的空速對拉起的能量比較有利).
    #   ⚠ 43000 CHANGE_SPEED 若超出飛控的空速上下限會被拒 (result=2).
    #     參數名依韌體版本: ArduPlane 4.4 以後是 AIRSPEED_MIN / AIRSPEED_MAX,
    #     之前叫 ARSPD_FBW_MIN / ARSPD_FBW_MAX. 必須 AIRSPEED_MIN ≤ 這個值.
    #   建議對齊飛控的 AIRSPEED_CRUISE (舊名 TRIM_ARSPD_CM), 也就是 AUTO
    #     實際在飛的巡航空速 — 這樣撥 GUIDED 不會造成加減速擾動.
    SEARCH_SPEED_MPS = 16.0     # SEARCH 巡航空速 (與 AUTO 一致)
    # ★ 2026/09/30: 撥 GUIDED 進入段. 上升沿起送 roll=0 (mask 198, pitch/
    #   油門/方向舵交飛控) 維持這麼久, 同時 43002 從上升沿就開始持續送.
    #   時間到停送姿態, 只剩 43002. ⚠ 停送後飛控沿用 forced roll=0 直到
    #   GUIDED_TIMEOUT 逾時, 43002 才真正接手壓坡度.
    GUIDED_ENTRY_ROLL0_S = 0.5

    # 攻角 (2026/08/18 pitch step log 校準, AOA 隨速度變)
    ALPHA_NOM_DEG = 3.5         # 巡航攻角
    ALPHA_DIVE_DEG = 0.0        #原1.3        # ★ 俯衝攻角 — 瞄準鏈用這個

    # PRE_DIVE: 到目標水平距 ≤ 門檻 → 直接命令 -DIVE_ANGLE → pitch 壓到容差內交棒
    PREDIVE_HANDOFF_PITCH_TOL_DEG = 3.0
    # 靜止/移動共用來源開關; 各 case 的門檻仍為目標水平距 (m).
    # False: 虛擬目標; True: 有效且新鮮的 ray projection 水平距.
    PREDIVE_USE_RAY_PROJECTION = False
    # Ray 使用 DKF 處理/預測像素; False 可切回當幀原始 YOLO bbox.
    RAY_USE_DKF_PIXELS = True
    # 預測距離有效期限: 距最後被 DKF 接受的 YOLO 擷取時間 (含相機延遲).
    RAY_DKF_MAX_PREDICTION_AGE_S = 0.5
    PREDIVE_SLANT_MARGIN = 1.30       # SEARCH→PRE_DIVE: slant ≤ margin·h/sin(dive)

    # ════════════════════════════════════════════════════════════
    #  空速 / 油門 (HERO2180)
    # ════════════════════════════════════════════════════════════
    V_CRUISE_MPS = 20.0
    V_STALL_MPS = 12.0
    THR_TRIM = 0.45
    THR_MIN = 0.05             # DIVE 怠速
    THR_MAX = 0.85

    # ════════════════════════════════════════════════════════════
    #  安全 (與實飛 SOP 對齊)
    # ════════════════════════════════════════════════════════════
    ABORT_MIN_AGL_M = 55.0
    ABORT_TARGET_LOST_S = 1.5   # DIVE 段目標丟失多久 → ABORT

    # ════════════════════════════════════════════════════════════
    #  相機內參 — ★ 1x 標定值 (2026/08 t2 機上 A8 mini)
    #  程式啟動時依 ZOOM_LEVEL 一次縮放成有效內參 (見 effective_intrinsics)
    # ════════════════════════════════════════════════════════════
    IMG_W = 1920
    IMG_H = 1080
    FX_1X = 1066.7
    FY_1X = 1059.5
    CX_1X = 959.4932
    CY_1X = 545.0292

    # ── 變焦 (開機固定, 不做 auto-zoom) ──
    # ★★ 2026/09/15 拆成兩個: 「相機實體」與「虛擬目標」各用各的焦距.
    #
    #   ZOOM_LEVEL         = 相機實體變焦. 影響所有【真實影像】的東西:
    #                        雲台開機變焦命令、raw/標註影片、YOLO、DKF、
    #                        射線測距、地理閘門、雲台追蹤、後處理重放.
    #                        (effective_intrinsics() / dive_aim_v_offset_px)
    #   VIRTUAL_ZOOM_LEVEL = 虛擬目標與控制座標系的焦距. 影響:
    #                        虛擬目標投影、影像 PID、出框判斷、DIVE 偏離中止.
    #                        (virtual_intrinsics() / virtual_aim_v_offset_px)
    #
    #   姿態命令與焦距無關 (投影乘 FX、PID 換角度除 FX, 互相抵銷; 閉迴路
    #   1x/2x 逐拍比對差 0.0000°). 拆開後的差別只有【視野】:
    #     1x: 水平/垂直半角 42.0°/27.0°, DIVE 偏離中止 37.8°
    #     2x: 水平/垂直半角 24.2°/14.3°, DIVE 偏離中止 21.2°
    #   → 相機 2x 看得清楚; 虛擬目標 1x 不會因窄視野提早判出框/中止.
    #
    #   ⚠ 座標系: 主 log 的 virt_bbox_u/v、virt_u_sp/v_sp、img_err_*_pct
    #     是【虛擬 (控制) 像素】. 視覺 log 的 virt_u_sp/v_sp 已換算成
    #     【相機像素】(後處理直接拿來跟 raw 影片的 YOLO 框比), 控制座標
    #     另存在 virt_u_sp_ctrl / virt_v_sp_ctrl.
    #   ⚠ 後處理用 ZOOM_LEVEL 當 raw 影片內參. 重跑【相機 1x 時期】的舊 log
    #     要先把這裡暫時改回 1.0.
    ZOOM_LEVEL = 2.0            # 相機實體 (開機送 requestAbsoluteZoom(2))
    VIRTUAL_ZOOM_LEVEL = 1.0    # 虛擬目標 / 控制座標系

    # ════════════════════════════════════════════════════════════
    #  YOLO
    # ════════════════════════════════════════════════════════════
    YOLO_MODEL_PATH = '/home/hsu/catkin_ws/src/hsu_realflight/hsu_visual/weights/best_car9.pt'
    YOLO_CONF = 0.4            # 偵測信心門檻
    YOLO_TARGET_CLASS_IDS = None   # None = 不過濾類別 (模型只有 1 類車)
    YOLO_IMGSZ = 1920

    # ════════════════════════════════════════════════════════════
    #  影像延遲補償 (餵 DKF; DKF 內部 retrodiction 回溯補償)
    # ════════════════════════════════════════════════════════════
    # DKF 吃「延遲補償後的當前時刻估測」餵給 PID (use_yolo_when_available=False).
    # 量測延遲 = YOLO 推論時間 + 擷取鏈延遲(下方常數). 前者每幀量, 後者校一次填.
    CAPTURE_LATENCY_MS = 0.0   # ★ HDMI 擷取卡 glass-to-frame 延遲 (校一次填;
                               #   0=只補 YOLO 推論. 校法: 對相機閃燈量時間差)

    # ════════════════════════════════════════════════════════════
    #  相機 / 主迴圈頻率
    # ════════════════════════════════════════════════════════════
    CAM_INDEX = 0
    # ★ 2026/09/11 拆成兩個值 (test_camera_bench 實測結論):
    #   CAM_FPS        = 標稱值, 用在 raw 影片檔頭 + DKF 的幀數型參數
    #   CAM_REQUEST_FPS = 向 V4L2 要求的值, 要【刻意設高】
    #   原因: BUFFERSIZE=1 時消費端稍慢就會固定錯過下一幀 ->
    #         要求 30 拿到 15 (剛好一半). 要求 60 則量化變細, 損失較小.
    #   單獨測試取像路徑可達 60 fps; 完整程式跑起來因 CPU 競爭會下降,
    #   實際值每趟都會印在收尾的「相機實際 fps」.
    CAM_FPS = 30              # 標稱 fps (raw 影片時基 / DKF 幀數參數)
    CAM_REQUEST_FPS = 60      # 向 V4L2 要求的 fps (留排擠餘裕)
    # DIVE 使用相機補償後像素控制; 關閉補償時保持上一有效命令.
    GCOMP_ENABLED = True
    # Default follows existing project's horizon pitch/roll + relative yaw model.
    # Select body_relative only if the installed firmware reports all body angles.
    GCOMP_ATTITUDE_FRAME = 'earth_pitch_roll_body_yaw'
    GCOMP_ROLL_SIGN = 1.0
    GCOMP_ROLL_OFFSET_DEG = 0.0
    GCOMP_YAW_OFFSET_DEG = 0.0
    GCOMP_MAX_SYNC_GAP_S = 0.15
    # Receive time minus measurement time, seconds; 0 means uncalibrated.
    GCOMP_GIMBAL_RX_DELAY_S = 0.0
    GCOMP_POSE_RX_DELAY_S = 0.0

    REALVISION_PID_LOG_ENABLED = True  # 只控制 CSV observer，不停用實體相機控制
    REALVISION_PID_MAX_AGE_S = 0.5  # older capture results are treated as lost

    CONTROL_HZ = 50.0        # 控制迴路頻率 (MAVROS + bridge)

    # ════════════════════════════════════════════════════════════
    #  雲台 (SIYI A8 mini)
    # ════════════════════════════════════════════════════════════
    SIYI_IP = '192.168.144.25'
    SIYI_PORT = 37260
    # STANDBY (未 GUIDED, AUTO 繞圈中): FOLLOW 模式, yaw=0 跟機頭, 只低頭看前方
    GIMBAL_STANDBY_PITCH_DEG = -25.0   # atan(140/300)≈25°; 對準 300m 前方地面
    GIMBAL_STANDBY_HOLD_HZ = 2.0       # STANDBY 重送 setAngles 頻率
    # SEARCH/PRE_DIVE 追蹤 (FOLLOW + 分區 P, 與 test_gimbal 一致)
    GIMBAL_TRACK_CMD_HZ = 5.0
    GIMBAL_LOST_TIMEOUT_S = 5.0        # 追蹤丟失多久回 STANDBY 指向
    GIMBAL_LOCK_TIME_S = 0.5           # 截獲後穩定多久才算 TRACKING
    # SIYI → 估測器慣例 (地面校正這幾個; 見 ray_projection_distance)
    SIYI_YAW_SIGN = -1.0
    SIYI_PITCH_SIGN = 1.0
    GIMBAL_PITCH_OFFSET_DEG = 0.0      # ★ 首飛前地面校正 (光軸 vs 報告角常數偏差)

    # ════════════════════════════════════════════════════════════
    #  地理閘門 (防跳目標: 偵測的 NED 位置須落在預期目標半徑內才接受)
    # ════════════════════════════════════════════════════════════
    GATE_RADIUS_M = 60.0       # 靜止目標基礎半徑
    GATE_MOVING_VMAX_MPS = 12.0  # 動態目標: 半徑隨丟失時間膨脹 R = R0 + vmax·t_lost
    # 首次截獲 (剛切 GUIDED): 取「離畫面中心最近」的偵測, 須連續穩定
    ACQUIRE_CENTER_NORM_MAX = 0.35   # 首次截獲: bbox 離中心 ≤ 此比例 (半對角歸一)
    ACQUIRE_STREAK = 5               # 連續幾幀滿足才建立鎖定
    # 鎖定後選框: 取「離 DKF 預測點最近」, 距離超過此像素 → 整幀視為無偵測
    TRACK_GATE_PX = 200.0            # 基礎像素閘門
    TRACK_GATE_PX_PER_S = 300.0      # 隨掉幀放寬 (px/s)

    # ════════════════════════════════════════════════════════════
    #  DIVE 段影像 PID (調校入口; 語意見 dive_guidance.GuidanceConfig)
    # ════════════════════════════════════════════════════════════
    PID_LAT_KP = 1.0  #9>1>2 #原4.5
    PID_LAT_KI = 0.1#0.1>0.2
    PID_LAT_KD = 0.0
    PID_LON_KP = 0.8 #原1.3
    PID_LON_KI = 0.1 #原0.50
    PID_LON_KD = 0.2
    PID_SP_LEAD_S = 0.0
    PID_MEAS_TAU_S = 0.10
    PID_D_TAU_S = 0.20
    PID_OUT_TAU_S = 0.0

    # ════════════════════════════════════════════════════════════
    #  DIVE / PRE_DIVE 橫向: 方向舵迴路 (2026/09 掛機驗證後新增)
    # ════════════════════════════════════════════════════════════
    #  ★ 背景: ArduPlane 的 SET_ATTITUDE_TARGET, yaw 欄位【不是航向目標】,
    #    而是方向舵舵量 (度 ×100 = centidegree, 45° = 滿舵). 閉迴路要自己寫.
    #    掛機實測的伺服映射: SERVO4 MIN=1123 TRIM=1595 MAX=1900,
    #    正向 6.78 PWM/度, 負向 10.49 PWM/度 (TRIM 不在幾何中點).
    #
    #  ★ 限幅換算 (滿舵 45°):
    #      ±3°  =  6.7% 權限  → 正/負 ΔPWM  +20 / -31
    #      ±6°  = 13.3%       → +41 / -63
    #      ±15° = 33.3%       → +102 / -157
    #
    #  ⚠ V 尾 (ruddervator) 行程排擠: 兩片舵面 = 升降舵 ± 方向舵, 各自夾
    #    ±4500. 限幅越大, 俯衝時越容易把其中一片推到端點 —— 那會同時
    #    (a) 損失方向舵差動、(b) 產生不對稱的俯仰擾動. 實測門檻:
    #      ±6°  → 升降舵到 -4000 才開始排擠
    #      ±15° → 升降舵到 -3500 就損失 17% 差動, -4000 損失 33%
    #    實飛務必檢查 rcout2/rcout4 有沒有貼在端點.
    RUDDER_LIMIT_DEG = 30.0      # 方向舵總限幅 (度, 45=滿舵) — 2026/09/23 ×2 (原 15.0)
    K_PSI = 0.30                 # PRE_DIVE 用: 航向誤差(度) → 舵(度) — 目前
                                 #   PRE_DIVE 方向舵強制歸中, 此值暫未生效
    K_R = 0.15                   # PRE_DIVE 用: 偏航角速度阻尼
    K_LAT_TO_RUDDER = 1.0       # DIVE: 影像 PID 輸出 dpsi(度) → 舵(度)
    RUDDER_SLEW_DPS = 20.0       # 舵量變化率上限 (度/s)
    # 空速排程: 動壓 ∝ V², 越快權限越小. (上限空速, 權限倍率)
    # ★ 2026/09/23: 取消空速排程 (原 ((22,1.0),(26,0.7),(999,0.45)) → 俯衝 >26m/s 只剩 0.45 倍)
    RUDDER_VSCHED = ((999.0, 1.00),)
    # roll 保護: 方向舵一定誘發滾轉, roll 太大就收手
    ROLL_PROT_FULL_DEG = 25.0    # |roll| ≤ 此值 → 全權限 (2026/09/23 放寬, 原 3.0)
    ROLL_PROT_ZERO_DEG = 35.0    # |roll| ≥ 此值 → 舵量歸零 (2026/09/23 放寬, 原 8.0)

    # ════════════════════════════════════════════════════════════
    #  虛擬目標 (控制路徑真相來源: 已知 GPS)
    # ════════════════════════════════════════════════════════════
    # ── 靜止版 (--static): 目標 GPS, 進俯衝用「到目標水平距」──
    # ★ 2026/09/30: 不再用固定 PRE_DIVE 點 (PREDIVE_GPS + 20m 半徑).
    #   舊做法飛機必須真的穿過該點 20m 內才觸發, 航線偏了就永遠不俯衝.
    #   現在改成跟移動版一樣: 飛機到目標水平距 ≤ STATIC_PREDIVE_DIST_M 就進 PRE_DIVE.
    #   ★ 注意: 舊的固定點順便決定了「從南邊進場」; 現在從哪個方向進入
    #     這個距離圈就從哪邊觸發, 進場方向要靠 AUTO 航線安排.
    #TARGET_GPS = (22.910215, 120.310426)        # 靜止目標 GPS (lat, lon)
    #TARGET_GPS = (22.9134463, 120.3122777)        # 靜止目標 140m 30度用的
    TARGET_GPS = (22.913677, 120.312260)         # 靜止目標 120m 25度用的
    #TARGET_GPS = (22.9145871, 120.3121805)         # new靜止目標 120m 25度用的
    # 舊 PRE_DIVE 點 (已停用, 留作紀錄): (22.911381, 120.312436)
    #   → 在目標正南約 358m, 所以新門檻預設 358m, 觸發距離跟以前一樣.
    STATIC_PREDIVE_DIST_M = 257.0   # 靜止版推頭點: 飛機到目標水平距 ≤ 此值
                                    #   → 直接命令 pitch = -DIVE_ANGLE

    # ── 移動版 (預設): 目標起點 GPS + 方位 + 速度, 進俯衝用「到目標距離」──
    #   方位由「起點→朝向點」兩座標算死 (= 馬路方向, 不用飛機航向).
    #   實際馬路 (2026 場地): 起點→朝向點 方位約 355.5°, 全長約 358m.
    MOVING_TARGET = True            # True=移動目標 (預設); False=靜止
    MOVING_START_GPS = (22.913647, 120.312259)   # 車起點 (切 GUIDED 時車在此)
    MOVING_HEADING_GPS = (22.9149299, 120.3121517)  # 朝此點開 → 算方位
    MOVING_HEADING_DEG = None       # 直接指定方位 (非 None 則覆寫上面兩點的算法)
    TARGET_SPEED_MPS = 10.0         # 車速 (等速)
    # ★ 2026/09/15: 移動目標前置瞄準 (lead) 開關, 預設【關】.
    #   False: DIVE setpoint = 畫面中心 (+攻角補償), PRE_DIVE 航向直接指車
    #          → 純追擊, 讓車本體留在畫面中心.
    #   True : setpoint 偏移到「車 + 車速×t_go」前置點 (舊行為).
    #   關掉的原因 (2026/09/14 case1 log): 車速 10 m/s × t_go≈12-17 s
    #   → 前置 120-170 m → 進 DIVE 後 v* 被推到 1000-1290 px (畫面底 1080),
    #   PID 把車壓到畫面底 → 進 DIVE 1 秒後車就出畫面, DIVE 86% 時間看不到車,
    #   img_err_v_pct 長時間 120-150%.
    MOVING_TARGET_LEAD = False
    PREDIVE_DIST_M = 257.0          # 移動版進俯衝: 飛機到目標水平距 ≤ 此值
                                    #   (靜止版另用 STATIC_PREDIVE_DIST_M)
                                    #   30° 幾何: 120m 高對應 ~208m; 追擊需實飛調

    # ════════════════════════════════════════════════════════════
    #  輸出 / 斷電保護
    # ════════════════════════════════════════════════════════════
    OUT_DIR = 'flight_logs'    # log 根目錄
    VIDEO_FPS = 25             # YOLO 標註影片 fps
    FSYNC_HZ = 4.0             # 背景 fsync 頻率 (SD 卡防丟; NVMe 可調高)

    def __init__(self, **overrides):
        for k, v in overrides.items():
            if not hasattr(self, k):
                raise KeyError(f"Config 沒有參數 {k}")
            setattr(self, k, v)

    # ------------------------------------------------------------------
    def intrinsics_at_zoom(self, z):
        """依變焦倍率 z 縮放 1x 內參 → (fx, fy, cx, cy).
        zoom z: fx'=fx·z, cx'=(cx-W/2)·z + W/2 (中心不變, 焦距與偏移同縮放)."""
        z = float(z)
        fx = self.FX_1X * z
        fy = self.FY_1X * z
        cx = (self.CX_1X - self.IMG_W / 2.0) * z + self.IMG_W / 2.0
        cy = (self.CY_1X - self.IMG_H / 2.0) * z + self.IMG_H / 2.0
        return fx, fy, cx, cy

    def effective_intrinsics(self):
        """【相機實體】有效內參 (ZOOM_LEVEL). 真實影像一律用這個."""
        return self.intrinsics_at_zoom(self.ZOOM_LEVEL)

    def virtual_intrinsics(self):
        """【虛擬目標 / 控制座標系】內參 (VIRTUAL_ZOOM_LEVEL)."""
        return self.intrinsics_at_zoom(
            getattr(self, 'VIRTUAL_ZOOM_LEVEL', self.ZOOM_LEVEL))

    @property
    def dive_aim_v_offset_px(self):
        """攻角補償 (相機像素): 縱向 setpoint 下移 fy·tan(ALPHA_DIVE).
        給吃真實影像的程式用 (test_dive_ground / 後處理)."""
        fx, fy, cx, cy = self.effective_intrinsics()
        return fy * math.tan(math.radians(self.ALPHA_DIVE_DEG))

    @property
    def virtual_aim_v_offset_px(self):
        """攻角補償 (虛擬/控制像素). run_vision_flight 的 PID 用這個."""
        fx, fy, cx, cy = self.virtual_intrinsics()
        return fy * math.tan(math.radians(self.ALPHA_DIVE_DEG))

    def virtual_to_camera_px(self, u, v):
        """虛擬(控制)像素 → 相機像素. 同一條光軸, 保持視角不變:
            u_cam = cx_cam + (u − cx_virt) · fx_cam / fx_virt
        u/v 為 None 或空字串時原樣回傳."""
        if u in (None, '') or v in (None, ''):
            return u, v
        vfx, vfy, vcx, vcy = self.virtual_intrinsics()
        cfx, cfy, ccx, ccy = self.effective_intrinsics()
        return (ccx + (float(u) - vcx) * cfx / vfx,
                ccy + (float(v) - vcy) * cfy / vfy)

    @property
    def predive_ground_range_m(self):
        return self.CRUISE_AGL_M / math.tan(math.radians(self.DIVE_ANGLE_DEG))
