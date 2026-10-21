"""合併版模組；沿用 GUIDED 多趟初始化版本，控制與估測公式保持原樣。"""

import math
from dive_guidance import project_body_camera, R_G2C, rot_y_np


# ========================================================================
# 原模組: virtual_target_geom.py
# ========================================================================
# -*- coding: utf-8 -*-


_R_EARTH = 6378137.0   # WGS-84 赤道半徑 (m)


def gps_to_ned(lat_deg, lon_deg, ref_lat_deg, ref_lon_deg):
    """經緯度 → 相對參考點的 (北, 東) 公尺. 小範圍等距近似 (誤差 << 1m @ 數 km)."""
    dlat = math.radians(lat_deg - ref_lat_deg)
    dlon = math.radians(lon_deg - ref_lon_deg)
    n = dlat * _R_EARTH
    e = dlon * _R_EARTH * math.cos(math.radians(ref_lat_deg))
    return n, e


def extrapolate_impact(n, e, agl, vn, ve, vd):
    """
    拉平觸發瞬間: 沿當下速度向量直線延伸到地面.

    Args:
        n, e: 飛機 NED 水平位置 (m)
        agl:  高度 (m, 向上為正)
        vn, ve: 水平速度 NED (m/s)
        vd: 下沉率 (m/s, 向下為正; 必須 > 0 才有交點)
    Returns:
        (impact_n, impact_e, t_fall) 或 (None, None, None) 若沒在下降
    """
    if vd <= 0.1:
        return None, None, None
    t_f = agl / vd
    return n + vn * t_f, e + ve * t_f, t_f


def miss_distance(impact_n, impact_e, t_fall,
                  tgt_n, tgt_e, tgt_vn, tgt_ve,
                  ac_vn, ac_ve):
    """
    Miss distance (論文慣例: 目標也外插到落地時刻).

    Returns dict:
        miss_total, miss_lon (沿飛機航跡向), miss_lat (垂直航跡向, 右正),
        tgt_impact_n, tgt_impact_e, t_fall
    """
    ti_n = tgt_n + tgt_vn * t_fall
    ti_e = tgt_e + tgt_ve * t_fall
    mn = impact_n - ti_n
    me = impact_e - ti_e
    total = math.hypot(mn, me)
    gs = math.hypot(ac_vn, ac_ve)
    if gs > 0.5:
        un, ue = ac_vn / gs, ac_ve / gs
        lon = mn * un + me * ue
        lat = -mn * ue + me * un
    else:
        lon, lat = total, 0.0
    return {
        'miss_total': total, 'miss_lon': lon, 'miss_lat': lat,
        'tgt_impact_n': ti_n, 'tgt_impact_e': ti_e,
        't_fall': t_fall,
    }

# ========================================================================
# 原模組: virtual_source.py
# ========================================================================
# -*- coding: utf-8 -*-




# ══════════════════════════════════════════════════════════════════
#  目標: 靜止 / 移動 (只負責「當下在哪、速度多少」)
# ══════════════════════════════════════════════════════════════════
class StaticTarget:
    """靜止目標: 固定 NED, 速度 0."""
    moving = False

    def __init__(self, n, e):
        self.n0, self.e0 = n, e

    def now(self, t):
        return self.n0, self.e0, 0.0, 0.0


class MovingTarget:
    """
    等速直線目標: 起點 (n0,e0), 方位 heading_deg, 速度 speed (m/s).
    t0 = 任務起點時刻 (切 GUIDED); 之前停在起點.
    位置 = 起點 + 速度向量 × max(0, t − t0).
    """
    moving = True

    def __init__(self, n0, e0, heading_deg, speed_mps, t0):
        self.n0, self.e0 = n0, e0
        self.t0 = t0
        self.heading_deg = heading_deg
        self.speed = speed_mps
        self.vn = speed_mps * math.cos(math.radians(heading_deg))
        self.ve = speed_mps * math.sin(math.radians(heading_deg))

    def now(self, t):
        dt = max(0.0, t - self.t0)
        return (self.n0 + self.vn * dt, self.e0 + self.ve * dt,
                self.vn, self.ve)


# ══════════════════════════════════════════════════════════════════
#  外殼: 投影 + 進俯衝距離 + get_fields (靜止/移動共用)
# ══════════════════════════════════════════════════════════════════
class VirtualSource:
    def __init__(self, cfg, gcfg, home_lat, home_lon, target,
                 predive_point_ne=None):
        """
        target: StaticTarget 或 MovingTarget.
        predive_point_ne: (n,e) 固定 PRE_DIVE 點 (靜止版用);
                          None → 移動版, 進俯衝改用「到目標當下位置」水平距.
        """
        self.cfg = cfg
        self.gcfg = gcfg
        self.home_lat, self.home_lon = home_lat, home_lon
        self.target = target
        self.pd_point = predive_point_ne
        tilt = math.radians(getattr(gcfg, 'GIMBAL_DIVE_TILT_BODY_DEG', 0.0))
        self.R_bc = R_G2C @ rot_y_np(tilt)
        mode = 'MOVING' if target.moving else 'STATIC'
        if self.pd_point is not None:
            print(f"[VirtualSource] {mode} 目標起點 NED=({target.n0:.1f},{target.e0:.1f}) "
                  f"PRE_DIVE 點 NED=({self.pd_point[0]:.1f},{self.pd_point[1]:.1f})")
        else:
            print(f"[VirtualSource] {mode} 目標起點 NED=({target.n0:.1f},{target.e0:.1f}) "
                  f"進俯衝用「到目標水平距 <= {gcfg.PREDIVE_DIST_M:.0f}m」")

    def reset_mission(self, t=None):
        """None parks a moving target at its configured start; t starts motion."""
        if self.target.moving:
            self.target.t0 = float("inf") if t is None else float(t)

    def target_now(self, t):
        """目標當下 NED + 速度 (n, e, vn, ve)."""
        return self.target.now(t)

    def get_fields(self, ac):
        """
        ac: dict — n, e, agl, theta_deg, chi_deg, gimbal_locked, t
        回傳餵給 MissionStateMachine.update 的視覺欄位 dict.
        """
        g = self.gcfg
        an, ae = ac['n'], ac['e']
        agl = ac['agl']
        t = ac['t']

        tn, te, tvn, tve = self.target_now(t)
        dxn, dye = tn - an, te - ae
        ground = math.hypot(dxn, dye)
        slant = math.hypot(ground, agl) if agl is not None else None

        # 進俯衝距離: 固定點 (靜止) 或 到目標當下位置 (移動)
        if self.pd_point is not None:
            predive_dist = math.hypot(self.pd_point[0] - an, self.pd_point[1] - ae)
        else:
            predive_dist = ground     # 到目標水平距

        half_diag = math.hypot(g.IMG_W / 2.0, g.IMG_H / 2.0)

        if ac.get('gimbal_locked', False) and agl is not None:
            # DIVE: body-fixed 真實投影
            u, v = project_body_camera(
                dxn, dye, agl, ac['theta_deg'], ac['chi_deg'], self.R_bc, g)
            if u is None:
                detected, u, v, off = False, None, None, 1.0
            else:
                detected = (0 <= u < g.IMG_W) and (0 <= v < g.IMG_H)
                off = math.hypot(u - g.CX, v - g.CY) / half_diag
        else:
            # SEARCH / PRE_DIVE: 世界穩定雲台理想指向 → 目標居中
            detected, u, v, off = True, g.CX, g.CY, 0.0

        # bbox 尺寸: 控制不用, 僅供 log/overlay, 粗略隨斜距縮放
        if slant is not None and slant > 3.0:
            bw = max(g.FX * 4.5 / slant, 4.0)
            bh = max(g.FY * 2.0 / slant, 3.0)
        else:
            bw = bh = None

        return {
            'tgt_n': tn, 'tgt_e': te, 'tgt_valid': True,
            'tgt_vn': tvn, 'tgt_ve': tve,          # 移動版有值 → miss 外插算車位移
            'slant_m': slant, 'ground_m': ground,
            'predive_dist_m': predive_dist,
            'bbox_detected': detected,
            'bbox_u': u, 'bbox_v': v,
            'bbox_w': bw if detected else None,
            'bbox_h': bh if detected else None,
            'bbox_center_offset_norm': off,
        }
