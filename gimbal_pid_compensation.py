"""合併版模組；沿用 GUIDED 多趟初始化版本，控制與估測公式保持原樣。"""

import copy
import math
from dive_guidance import DivePID
from types import SimpleNamespace
import numpy as np
from dive_guidance import R_G2C, rot_y_np, rot_z_np


# ========================================================================
# 原模組: realvision_pid_logger.py
# ========================================================================
REAL_PID_FIELDS = [
    'real_pid_enabled', 'real_pid_active', 'real_pid_updated',
    'real_pid_detected', 'real_pid_frame_idx', 'real_pid_raw_frame_idx',
    'real_pid_new_frame', 'real_pid_cap_t', 'real_pid_ready_t',
    'real_pid_age_ms', 'real_pid_dt', 'real_pid_u', 'real_pid_v',
    'real_pid_u_sp', 'real_pid_v_sp', 'real_pid_fx', 'real_pid_fy',
    'real_pid_u_err_deg', 'real_pid_v_err_deg',
    'real_pid_lat_p', 'real_pid_lat_i', 'real_pid_lat_d',
    'real_pid_lon_p', 'real_pid_lon_i', 'real_pid_lon_d',
    'real_pid_dpsi_deg', 'real_pid_dtheta_deg',
    'real_pitch_cmd', 'real_heading_cmd', 'real_pid_compute_ms',
]


def finite(value):
    return value is not None and math.isfinite(float(value))


class RealVisionPIDLogger:
    def __init__(self, cfg, virtual_cfg):
        self.enabled = bool(getattr(cfg, 'REALVISION_PID_LOG_ENABLED', True))
        self.max_age = float(getattr(cfg, 'REALVISION_PID_MAX_AGE_S', 0.5))
        self.capture_latency = float(getattr(cfg, 'CAPTURE_LATENCY_MS', 0.0)) / 1000.0
        self.cfg = copy.deepcopy(virtual_cfg)
        self.cfg.FX, self.cfg.FY, self.cfg.CX, self.cfg.CY = cfg.effective_intrinsics()
        self.vfx, self.vfy = virtual_cfg.FX, virtual_cfg.FY
        self.vcx, self.vcy = virtual_cfg.CX, virtual_cfg.CY
        self.pid = DivePID(self.cfg)
        self.in_dive = False
        self.entry_t = None
        self.last_frame = None

    def tick(self, sm, out, inp, seq_before, st, frame_idx, raw_idx,
             cap_mono, ready_mono, now_mono, now_wall, origin_wall, control_pid=False):
        row = dict.fromkeys(REAL_PID_FIELDS, '')
        row.update(real_pid_enabled=int(self.enabled), real_pid_active=0,
                   real_pid_updated=0, real_pid_detected=0, real_pid_new_frame=0)
        in_dive = out['mode'] == 'DIVE'
        if not self.enabled:
            return row
        if control_pid:
            self.pid = sm.pid  # 記錄真正送往 bridge 的 PID，不再另外積分一次
        if in_dive and not self.in_dive:
            # State machine resets its PID on entry. Copy these exact seeds.
            if not control_pid:
                self.pid.reset(sm.pid.pitch_cmd, sm.pid.heading_cmd)
            self.entry_t = inp['t']
            self.last_frame = None
        self.in_dive = in_dive
        if not in_dive:
            return row

        updated = sm.pid.last_dt > 0 and sm.pid.update_seq != seq_before
        u_sp = self.cfg.CX + (sm._sp_u - self.vcx) * self.cfg.FX / self.vfx
        v_sp = self.cfg.CY + (sm._sp_v - self.vcy) * self.cfg.FY / self.vfy
        age = now_mono - cap_mono + self.capture_latency if cap_mono is not None else None
        cap_t = now_wall - origin_wall - (now_mono - cap_mono) if cap_mono is not None else None
        ready_t = now_wall - origin_wall - (now_mono - ready_mono) if ready_mono is not None else None
        u = getattr(st, 'bbox_u', None)
        v = getattr(st, 'bbox_v', None)
        detected = bool(getattr(st, 'detected', False) and finite(u) and finite(v)
                        and age is not None and 0 <= age <= self.max_age
                        and cap_t - self.capture_latency >= self.entry_t)
        if control_pid:
            detected = bool(inp.get('real_control_detected')
                            and inp.get('real_control_cap_t') is not None
                            and inp['real_control_cap_t'] >= sm._dive_t0)
        if updated and not control_pid:
            pitch = inp.get('pitch_deg')
            if pitch is None:
                pitch = out['pitch_cmd_deg']
            if self.cfg.BENCH_DIVE_PITCH_BASE_DEG is not None:
                pitch = float(self.cfg.BENCH_DIVE_PITCH_BASE_DEG)
            vn, ve = inp.get('vn'), inp.get('ve')
            course = (math.degrees(math.atan2(ve, vn))
                      if vn is not None and ve is not None and vn * vn + ve * ve > 4
                      else (inp.get('yaw_true_deg') or 0.0))
            self.pid.update(u, v, u_sp, v_sp, detected, pitch, course, sm.pid.last_dt)
            if self.cfg.DIVE_HOLD_HEADING:
                self.pid.heading_cmd = out['heading_cmd_deg']
        # Keep PID reset seeds untouched during settle, as the virtual PID does.

        row.update(real_pid_active=int(updated), real_pid_updated=int(updated),
                   real_pid_detected=int(detected), real_pid_frame_idx=frame_idx,
                   real_pid_raw_frame_idx=raw_idx,
                   real_pid_new_frame=int(raw_idx >= 0 and raw_idx != self.last_frame),
                   real_pid_cap_t=cap_t if cap_t is not None else '',
                   real_pid_ready_t=ready_t if ready_t is not None else '',
                   real_pid_age_ms=age * 1000 if age is not None else '',
                   real_pid_dt=sm.pid.last_dt if updated else '',
                   real_pid_u=u if finite(u) else '', real_pid_v=v if finite(v) else '',
                   real_pid_u_sp=u_sp, real_pid_v_sp=v_sp,
                   real_pid_fx=self.cfg.FX, real_pid_fy=self.cfg.FY,
                   real_pitch_cmd=self.pid.pitch_cmd if updated else out['pitch_cmd_deg'],
                   real_heading_cmd=self.pid.heading_cmd if updated else out['heading_cmd_deg'],
                   real_pid_compute_ms=self.pid.last_compute_ms if updated else '')
        self.last_frame = raw_idx
        if updated:
            for axis, pid in [('lat', self.pid.lat), ('lon', self.pid.lon)]:
                for term in ('p', 'i', 'd'):
                    row['real_pid_' + axis + '_' + term] = getattr(pid, term)
            # A lost frame retains internal info; do not label it a fresh error.
            if detected:
                row.update(real_pid_u_err_deg=self.pid.info['u_err_deg'],
                           real_pid_v_err_deg=self.pid.info['v_err_deg'],
                           real_pid_dpsi_deg=self.pid.info['dpsi'],
                           real_pid_dtheta_deg=self.pid.info['dtheta'])
        return row

# ========================================================================
# 原模組: gimbal_pid_compensation.py
# ========================================================================
GEOMETRY_FIELDS = ['gcomp_valid', 'gcomp_reason', 'gcomp_pose_method',
    'gcomp_gimbal_method', 'gcomp_pose_gap_ms', 'gcomp_gimbal_gap_ms',
    'gcomp_capture_mono', 'gcomp_gimbal_yaw', 'gcomp_gimbal_pitch',
    'gcomp_gimbal_roll', 'gcomp_ac_pitch', 'gcomp_ac_roll', 'gcomp_ac_yaw',
    'gcomp_ac_course', 'gcomp_u', 'gcomp_v', 'gcomp_u_err_deg', 'gcomp_v_err_deg']
COMP_PID_FIELDS = [f.replace('real_', 'real_comp_', 1) for f in REAL_PID_FIELDS]
RAY_LOG_FIELDS = ['ray_valid', 'ray_age_ms', 'ray_slant_m', 'ray_ground_m',
                  'ray_raw_slant_m', 'ray_raw_ground_m', 'ray_range_conf',
                  'ray_pixel_source', 'ray_u', 'ray_v', 'ray_dkf_status',
                  'ray_is_prediction', 'ray_last_yolo_age_ms',
                  'ray_estimate_age_ms']

def control_inputs(cfg, st, geometry, cap_mono, now_mono, t, virtual_ground_m):
    """一拍一致的補償像素與水平距來源; 不使用過期估測觸發 predive."""
    age = (now_mono - cap_mono + cfg.CAPTURE_LATENCY_MS / 1000.
           if cap_mono is not None else None)
    fresh = age is not None and 0 <= age <= cfg.REALVISION_PID_MAX_AGE_S
    det = fresh and bool(getattr(st, 'detected', False))
    geom = geometry or {}
    u, v = geom.get('gcomp_u'), geom.get('gcomp_v')
    valid = bool(det and cfg.GCOMP_ENABLED and geom.get('gcomp_valid')
                 and finite_number(u) and finite_number(v))
    ray_ground = getattr(st, 'ground_m', None)
    conf = getattr(st, 'range_conf', None)
    estimate_mono = getattr(st, 'ray_estimate_mono', None)
    estimate_age = now_mono - estimate_mono if finite_number(estimate_mono) else None
    last_yolo_mono = getattr(st, 'ray_last_yolo_mono', None)
    last_yolo_age = now_mono - last_yolo_mono if finite_number(last_yolo_mono) else None
    predicted = bool(getattr(st, 'ray_is_prediction', False))
    prediction_fresh = (not predicted or (last_yolo_age is not None and
                        0 <= last_yolo_age <= cfg.RAY_DKF_MAX_PREDICTION_AGE_S))
    # Packet freshness and age since actual YOLO confirmation are independent.
    # Real-camera PID validity above intentionally still requires detection.
    ray_valid = bool(fresh and getattr(st, 'ray_pixel_valid', False)
                     and estimate_age is not None
                     and 0 <= estimate_age <= cfg.REALVISION_PID_MAX_AGE_S
                     and prediction_fresh and finite_number(ray_ground)
                     and ray_ground >= 0 and finite_number(conf) and conf > 0)
    use_ray = bool(getattr(cfg, 'PREDIVE_USE_RAY_PROJECTION', False))
    fields = {
        'real_control_detected': valid,
        'real_control_u': u if valid else None,
        'real_control_v': v if valid else None,
        'real_control_cap_t': t - age if age is not None else None,
        'predive_dist_m': (ray_ground if ray_valid else None) if use_ray
                          else virtual_ground_m,
    }
    ray = dict.fromkeys(RAY_LOG_FIELDS, '')
    ray.update(ray_valid=int(ray_valid), ray_age_ms=age * 1000 if age is not None else '',
               ray_pixel_source=getattr(st, 'ray_pixel_source', None) or '',
               ray_dkf_status=getattr(st, 'dkf_status', None) or '',
               ray_is_prediction=int(predicted),
               ray_last_yolo_age_ms=last_yolo_age * 1000 if last_yolo_age is not None else '',
               ray_estimate_age_ms=estimate_age * 1000 if estimate_age is not None else '')
    for key in ('ray_u', 'ray_v'):
        value = getattr(st, key, None)
        ray[key] = value if finite_number(value) else ''
    for key, attr in [('ray_slant_m', 'slant_m'), ('ray_ground_m', 'ground_m'),
                      ('ray_raw_slant_m', 'raw_slant_m'),
                      ('ray_raw_ground_m', 'raw_ground_m'),
                      ('ray_range_conf', 'range_conf')]:
        value = getattr(st, attr, None)
        ray[key] = value if finite_number(value) else ''
    ray.update(predive_distance_source='ray_projection' if use_ray else 'virtual',
               predive_used_dist_m=fields['predive_dist_m']
               if fields['predive_dist_m'] is not None else '')
    return fields, ray

def finite_number(value):
    return isinstance(value, (int, float, np.number)) and math.isfinite(float(value))

def rx(deg):
    c,s=math.cos(math.radians(deg)),math.sin(math.radians(deg))
    return np.array([[1.,0.,0.],[0.,c,s],[0.,-s,c]])

def reference_matrix(pitch,course,tilt=0.):
    # Deliberately matches project_body_camera: roll=0 and course, not nose yaw.
    return R_G2C @ rot_y_np(math.radians(tilt)) @ rot_y_np(math.radians(pitch)) @ rot_z_np(math.radians(course))

def camera_matrix(cfg, gy,gp,gr, aircraft):
    roll,pitch,yaw,course=aircraft
    y=cfg.SIYI_YAW_SIGN*gy+cfg.GCOMP_YAW_OFFSET_DEG
    p=cfg.SIYI_PITCH_SIGN*gp+cfg.GIMBAL_PITCH_OFFSET_DEG
    r=cfg.GCOMP_ROLL_SIGN*gr+cfg.GCOMP_ROLL_OFFSET_DEG
    mode=cfg.GCOMP_ATTITUDE_FRAME
    if mode == 'earth_pitch_roll_body_yaw':
        # Project's existing convention: yaw encoder offset from aircraft yaw.
        yaw_world=yaw+y
        return R_G2C @ rot_y_np(math.radians(p)) @ rx(r) @ rot_z_np(math.radians(yaw_world))
    if mode == 'body_relative':
        return (R_G2C @ rot_y_np(math.radians(p)) @ rx(r) @ rot_z_np(math.radians(y))
                @ rx(roll) @ rot_y_np(math.radians(pitch)) @ rot_z_np(math.radians(yaw)))
    raise ValueError('Unknown GCOMP_ATTITUDE_FRAME: '+mode)

def transform_pixel(cfg, virtual_cfg, u,v, aircraft,gimbal):
    fx,fy,cx,cy=cfg.effective_intrinsics()
    ray=np.array([(u-cx)/fx,(v-cy)/fy,1.])
    actual=camera_matrix(cfg,*gimbal,aircraft)
    reference=reference_matrix(aircraft[1],aircraft[3],virtual_cfg.GIMBAL_DIVE_TILT_BODY_DEG)
    converted=reference @ actual.T @ ray
    if not np.isfinite(converted).all() or converted[2] <= 1e-6:
        return None
    return (virtual_cfg.CX+virtual_cfg.FX*converted[0]/converted[2],
            virtual_cfg.CY+virtual_cfg.FY*converted[1]/converted[2])

def capture_geometry(cfg, virtual_cfg, st, cap_mono, pose_history, gimbal):
    result=dict.fromkeys(GEOMETRY_FIELDS,'')
    result.update(gcomp_valid=0,gcomp_reason='missing_capture_time')
    if cap_mono is None:
        return result
    target=cap_mono-float(getattr(cfg,'CAPTURE_LATENCY_MS',0.))/1000.
    result['gcomp_capture_mono']=target
    pose=pose_history.at(target+cfg.GCOMP_POSE_RX_DELAY_S,cfg.GCOMP_MAX_SYNC_GAP_S)
    att=gimbal.get_attitude_at(target+cfg.GCOMP_GIMBAL_RX_DELAY_S,cfg.GCOMP_MAX_SYNC_GAP_S)
    if pose is None or att is None:
        result['gcomp_reason']='missing_or_stale_attitude'
        return result
    pa,ga=pose['angles'],att['angles']
    result.update(gcomp_pose_method=pose['method'],gcomp_gimbal_method=att['method'],
        gcomp_pose_gap_ms=pose['distance_s']*1000,gcomp_gimbal_gap_ms=att['distance_s']*1000,
        gcomp_ac_roll=pa[0],gcomp_ac_pitch=pa[1],gcomp_ac_yaw=pa[2],gcomp_ac_course=pa[3],
        gcomp_gimbal_yaw=ga[0],gcomp_gimbal_pitch=ga[1],gcomp_gimbal_roll=ga[2])
    if not getattr(st,'detected',False):
        result['gcomp_reason']='no_detection'
        return result
    pixel=transform_pixel(cfg,virtual_cfg,st.bbox_u,st.bbox_v,pa,ga)
    if pixel is None:
        result['gcomp_reason']='ray_behind_reference_camera'
        return result
    result.update(gcomp_valid=1,gcomp_reason='ok',gcomp_u=pixel[0],gcomp_v=pixel[1])
    return result

class GimbalCompPIDLogger:
    def __init__(self,cfg,gcfg):
        self.cfg=cfg
        self.raw=RealVisionPIDLogger(cfg,gcfg)
        # Reuse identical PID but feed corrected coordinates at virtual zoom.
        ccfg=copy.copy(cfg)
        ccfg.ZOOM_LEVEL=cfg.VIRTUAL_ZOOM_LEVEL
        self.comp=RealVisionPIDLogger(ccfg,gcfg)
        self.enabled=self.raw.enabled

    def tick(self,sm,out,inp,seq_before,st,frame_idx,raw_idx,cap,ready,now,wall,origin,geometry=None):
        self.raw.enabled=self.enabled
        self.comp.enabled=self.enabled and self.cfg.GCOMP_ENABLED
        raw=self.raw.tick(sm,out,inp,seq_before,st,frame_idx,raw_idx,cap,ready,now,wall,origin)
        geom=dict(geometry or {})
        cst=SimpleNamespace(detected=bool(geom.get('gcomp_valid',False)),
             bbox_u=geom.get('gcomp_u') if geom.get('gcomp_valid') else None,
             bbox_v=geom.get('gcomp_v') if geom.get('gcomp_valid') else None)
        corrected=self.comp.tick(sm,out,inp,seq_before,cst,frame_idx,raw_idx,cap,ready,now,wall,origin,
                                 control_pid=sm.cfg.DIVE_USE_REAL_CAMERA)
        raw.update({k.replace('real_','real_comp_',1):v for k,v in corrected.items()})
        raw.update({k:geom.get(k,'') for k in GEOMETRY_FIELDS})
        if self.comp.enabled and geom.get('gcomp_valid') and sm.pid.update_seq!=seq_before:
            raw['gcomp_u_err_deg']=math.degrees(math.atan2(geom['gcomp_u']-sm._sp_u,sm.cfg.FX))
            raw['gcomp_v_err_deg']=math.degrees(math.atan2(geom['gcomp_v']-sm._sp_v,sm.cfg.FY))
        return raw
