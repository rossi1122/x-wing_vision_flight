# -*- coding: utf-8 -*-
"""
xwing_vision_common.py — 全視覺版實飛與模擬共用的組裝函式
═══════════════════════════════════════════════════════════════════
來源: hsu_realflight_true/xwing_virtual_common.py (同一組參數覆寫),
加上全視覺開關. 實飛 (xwing_vision_flight.py) 與模擬 (xwing_vision_sim.py)
都從這裡建, 兩邊觸發點一致.
"""

import math

from dive_guidance import GuidanceConfig
from virtual_source import StaticTarget, MovingTarget, gps_to_ned


class StopAtEndMovingTarget(MovingTarget):
    """等速直線目標, 走 max_travel_m 公尺後停住 (MOVING_STOP_AT_HEADING_GPS)."""

    def __init__(self, n0, e0, heading_deg, speed_mps, t0, max_travel_m):
        super().__init__(n0, e0, heading_deg, speed_mps, t0)
        self.max_travel_m = max_travel_m

    def now(self, t):
        dt = max(0.0, t - self.t0)
        if self.speed > 0 and dt * self.speed >= self.max_travel_m:
            dt = self.max_travel_m / self.speed
            n, e = self.n0 + self.vn * dt, self.e0 + self.ve * dt
            return n, e, 0.0, 0.0
        return (self.n0 + self.vn * dt, self.e0 + self.ve * dt,
                self.vn, self.ve)


def target_heading_deg(cfg, start_gps):
    """目標方位: MOVING_HEADING_DEG 優先, 否則 起點→航向點 兩座標算."""
    if cfg.MOVING_HEADING_DEG is not None:
        return float(cfg.MOVING_HEADING_DEG) % 360.0
    hn, he = gps_to_ned(cfg.MOVING_HEADING_GPS[0], cfg.MOVING_HEADING_GPS[1],
                        start_gps[0], start_gps[1])
    return math.degrees(math.atan2(he, hn)) % 360.0


def road_length_m(cfg, start_gps):
    hn, he = gps_to_ned(cfg.MOVING_HEADING_GPS[0], cfg.MOVING_HEADING_GPS[1],
                        start_gps[0], start_gps[1])
    return math.hypot(hn, he)


def build_target(cfg, moving, origin_gps, start_gps, heading_deg, speed_mps,
                 t0=float('inf')):
    """虛擬目標 (帶路 / 對照用). t0 = inf: 撥 GUIDED 前停在起點."""
    sn, se = gps_to_ned(start_gps[0], start_gps[1], origin_gps[0], origin_gps[1])
    if not moving:
        return StaticTarget(sn, se)
    if getattr(cfg, 'MOVING_STOP_AT_HEADING_GPS', False):
        return StopAtEndMovingTarget(sn, se, heading_deg, speed_mps, t0,
                                     road_length_m(cfg, start_gps))
    return MovingTarget(sn, se, heading_deg, speed_mps, t0)


def build_gcfg(cfg, moving, predive_dist, vision=True, bench=False,
               bench_pitch=None, bench_dive=False, ground_test=False):
    """GuidanceConfig. vision=True: 全視覺 (VISION_GUIDANCE + DIVE_USE_REAL_CAMERA);
    False: 與 xwing_virtual 完全相同 (虛擬目標控制, 視覺只記錄)."""
    fx, fy, cx, cy = cfg.virtual_intrinsics()
    return GuidanceConfig(
        DIVE_ANGLE_DEG=cfg.DIVE_ANGLE_DEG,
        ALPHA_NOM_DEG=cfg.ALPHA_NOM_DEG,
        ALPHA_DIVE_DEG=cfg.ALPHA_DIVE_DEG,
        PREDIVE_TRIGGER_MODE='target_dist',
        PREDIVE_DIST_M=predive_dist,
        PREDIVE_HANDOFF_PITCH_TOL_DEG=cfg.PREDIVE_HANDOFF_PITCH_TOL_DEG,
        PREDIVE_TIMEOUT_S=cfg.PREDIVE_TIMEOUT_S,
        PULLOUT_AGL_M=cfg.PULLOUT_AGL_M,
        PULLOUT_VMAX_MPS=cfg.PULLOUT_VMAX_MPS,
        TECS_HOLD_ALT_M=cfg.TECS_HOLD_ALT_M,
        TECS_SPEED_MPS=cfg.TECS_SPEED_MPS,
        ABORT_MIN_AGL_M=(0.0 if bench else cfg.ABORT_MIN_AGL_M),
        ABORT_TARGET_LOST_S=cfg.ABORT_TARGET_LOST_S,
        ENTRY_MIN_AGL_M=(0.0 if bench else cfg.ENTRY_MIN_AGL_M),
        BENCH_GEOMETRY_AGL_M=(cfg.CRUISE_AGL_M if bench else 0.0),
        BENCH_HANDOFF_ANY_PITCH=bench_dive,
        BENCH_DIVE_PITCH_BASE_DEG=(bench_pitch if bench else None),
        FX=fx, FY=fy, CX=cx, CY=cy, IMG_W=cfg.IMG_W, IMG_H=cfg.IMG_H,
        PID_LAT_KP=cfg.PID_LAT_KP, PID_LAT_KI=cfg.PID_LAT_KI, PID_LAT_KD=cfg.PID_LAT_KD,
        PID_LON_KP=cfg.PID_LON_KP, PID_LON_KI=cfg.PID_LON_KI, PID_LON_KD=cfg.PID_LON_KD,
        PID_SP_LEAD_S=cfg.PID_SP_LEAD_S,
        PID_MEAS_TAU_S=cfg.PID_MEAS_TAU_S, PID_D_TAU_S=cfg.PID_D_TAU_S,
        PID_OUT_TAU_S=cfg.PID_OUT_TAU_S,
        DIVE_TARGET_LEAD=(moving and cfg.MOVING_TARGET_LEAD and not vision),
        DIVE_AIM_V_OFFSET_PX=cfg.virtual_aim_v_offset_px,
        GIMBAL_DIVE_TILT_BODY_DEG=0.0,
        GIMBAL_FPV_SETTLE_S=0.0,
        SEARCH_HOLD_HEADING=ground_test,
        DIVE_HOLD_HEADING=False,          # DIVE 一律用方向舵 PID (版本 A)
        # ── 全視覺 ──
        VISION_GUIDANCE=vision,
        DIVE_USE_REAL_CAMERA=vision,
        VISION_ACQUIRE_HITS=cfg.VISION_ACQUIRE_HITS,
        VISION_MEAS_MAX_AGE_S=cfg.VISION_MEAS_MAX_AGE_S,
        VISION_SEARCH_TIMEOUT_S=cfg.VISION_SEARCH_TIMEOUT_S,
        VISION_LOST_SEARCH_S=cfg.VISION_LOST_SEARCH_S,
        VISION_LOST_PREDIVE_S=cfg.VISION_LOST_PREDIVE_S,
        VISION_LOST_DIVE_S=cfg.VISION_LOST_DIVE_S,
        VISION_PREDIVE_MAX_OFF_DEG=cfg.VISION_PREDIVE_MAX_OFF_DEG,
        VISION_PREDIVE_MIN_DIST_M=(0.0 if bench else cfg.VISION_PREDIVE_MIN_DIST_M),
    )


def control_camera(cfg, gcfg):
    """VisionTracker 的控制相機 (FX, FY, CX, CY, tilt)."""
    return (gcfg.FX, gcfg.FY, gcfg.CX, gcfg.CY, gcfg.GIMBAL_DIVE_TILT_BODY_DEG)


BRIDGE_KEYS = ('RUDDER_LIMIT_DEG', 'K_PSI', 'K_R', 'K_LAT_TO_RUDDER',
               'RUDDER_SLEW_DPS', 'RUDDER_VSCHED',
               'ROLL_PROT_FULL_DEG', 'ROLL_PROT_ZERO_DEG',
               'GUIDED_ENTRY_ROLL0_S',
               'TECS_HOLD_ALT_M', 'TECS_SPEED_MPS', 'THR_TRIM', 'THR_MIN',
               'THR_MAX')


def apply_bridge_cfg(bcfg, cfg):
    """把 config 的值寫進 AttitudeBridgeConfig."""
    for k in BRIDGE_KEYS:
        if hasattr(cfg, k):
            setattr(bcfg, k, getattr(cfg, k))
    bcfg.V_REF_MPS = cfg.V_CRUISE_MPS
    return bcfg
