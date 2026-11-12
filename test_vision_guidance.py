#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_vision_guidance.py — 目標偵測版測試 (不需要 ROS / 飛控 / 雲台)
═══════════════════════════════════════════════════════════════════
1. 幾何: 方位、水平距、DIVE 畫面位置的正負號 (手算期望值, 不靠模擬器), 雲台 yaw 用法
   推頭依據: 看車俯角 (angle, 預設) vs 偵測水平距 (distance)
   地面 / 掛機: 高度 ~0 時角度照算 (2026/11/12)
1c. 地面校正 gimbal_calib.py: 建議的雲台 offset 正負號正確
1b. UART 協定: 共用接收端 V6 新格式 (0~5 / 10, 含 yaw) 與舊 v6 / v7 / v8 都解得出來
2. VisionSource: 偵測結果 → 與虛擬目標同名的欄位 (看得到 / 看不到 / 資料過期)
3. 閉迴路模擬 (xwing_vision_sim): 各種車子情境的結果合理
4. 實飛主程式 xwing_vision_flight.main(): 假 rospy + 假時鐘 + 模擬雲台,
   走完「撥 GUIDED → PRE_DIVE → DIVE → PULLOUT → DONE」, 以及沒有車時不推頭

用法: python3 test_vision_guidance.py      (最後印 PASS / FAIL)
"""

import csv
import glob
import math
import os
import sys
import types

REPO = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, REPO)

FAILS = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILS.append(msg)


# ══════════════════════════════════════════════════════════════════
# 1. 幾何
# ══════════════════════════════════════════════════════════════════
def test_geometry():
    print("[1] 幾何")
    from xwing_vision_config import XwingVisionConfig
    from xwing_vision_common import build_gcfg, control_camera
    from vision_guidance import measure

    cfg = XwingVisionConfig()
    gcfg = build_gcfg(cfg, True, cfg.PREDIVE_DIST_M)
    ctrl = control_camera(cfg, gcfg)

    def pkt(u, v, pitch, roll=0.0):
        return {'target_cx': u, 'target_cy': v, 'fx': 1000.0, 'fy': 1000.0,
                'cx': 960.0, 'cy': 540.0, 'pitch': pitch, 'roll': roll}

    # 往南飛 (yaw 180), 雲台 −30°, 車在畫面正中央 → 方位 180, 水平距 240/tan30
    m = measure(pkt(960, 540, -30.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(abs(m['ground_m'] - 415.69) < 0.5, f"正中央: 水平距 {m['ground_m']:.1f} ≈ 415.7")
    check(abs(m['bearing_deg'] - 180.0) < 0.01, f"正中央: 方位 {m['bearing_deg']:.2f} = 180")
    check(abs(m['depress_deg'] - 30.0) < 0.01, "正中央: 俯角 30°")
    # 對地 pitch: 飛機機頭抬 5° 不影響結果
    m5 = measure(pkt(960, 540, -30.0), (0.0, 5.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(abs(m5['ground_m'] - m['ground_m']) < 1e-6, "機頭 pitch 不影響距離 (雲台回報對地角)")
    # 車在畫面右邊 → 方位往右 (往南飛時右邊 = 西, 方位 > 180)
    mr = measure(pkt(960 + 100, 540, -30.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(mr['bearing_off_deg'] > 0 and mr['bearing_deg'] > 180.0,
          f"車在右邊: 偏機頭 {mr['bearing_off_deg']:+.2f}° (右正)")
    # 車在畫面下面 → 比較近
    md = measure(pkt(960, 540 + 100, -30.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(md['ground_m'] < m['ground_m'], f"車在下面: 水平距 {md['ground_m']:.0f} < 416")
    # 雲台偏差校正: 光軸實際往下多 1° (回報 −29 實際 −30) → offset −1 補回
    cfg_b = XwingVisionConfig(GIMBAL_PITCH_OFFSET_DEG=-1.0)
    mb = measure(pkt(960, 540, -29.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg_b, ctrl)
    check(abs(mb['ground_m'] - 415.69) < 0.5, "GIMBAL_PITCH_OFFSET_DEG 正確補償")
    # DIVE 補償: 機頭 −27°, 相機對地 −30°, 車剛好在機頭正前方
    #   機頭方向的俯角 27° → 在相機裡位於中心上方 3°; 補償後應在控制相機正中央
    v_img = 540 - 1000 * math.tan(math.radians(3.0))
    mc = measure(pkt(960, v_img, -30.0), (0.0, -27.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(abs(mc['comp_u'] - ctrl[2]) < 0.5 and abs(mc['comp_v'] - ctrl[3]) < 0.5,
          f"DIVE 補償: 車在機頭正前方 → 控制像素 ({mc['comp_u']:.1f},{mc['comp_v']:.1f}) "
          f"≈ 中心 ({ctrl[2]:.1f},{ctrl[3]:.1f})")
    # 射線朝上 → 無效
    check(measure(pkt(960, 0, 20.0), (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl) is None,
          "射線朝上 → 無效")
    # 地面 (2026/11/12): 高度 ~0 也照算角度, 距離 / 位置留空
    check(measure(pkt(960, 540, -30.0), (0.0, 0.0, 180.0, 180.0), 0.2, cfg, ctrl) is None,
          "空中模式 (ground=False) 高度 0.2 m → 無效 (原本行為不變)")
    mg = measure(pkt(960, 540, -30.0), (0.0, 0.0, 180.0, 180.0), 0.2, cfg, ctrl, ground=True)
    check(mg is not None and abs(mg['depress_deg'] - 30.0) < 0.01 and mg['ground_m'] is None
          and mg['geom'] == 'angle' and mg['comp_u'] is not None,
          "地面: 俯角 30° 照算, 水平距 / 位置留空, DIVE 畫面位置有值")
    m0 = measure(pkt(960 + 50, 540, 0.0), (0.0, 0.0, 90.0, 90.0), 0.0, cfg, ctrl, ground=True)
    check(m0 is not None and abs(m0['depress_deg']) < 0.01
          and abs(m0['bearing_off_deg'] - math.degrees(math.atan(0.05))) < 0.01,
          f"地面平視 (俯角 0°) 也算: 偏機頭 {m0['bearing_off_deg']:+.2f}° (校正方位用)")
    mup = measure(pkt(960, 540, 5.0), (0.0, 0.0, 90.0, 90.0), 0.0, cfg, ctrl, ground=True)
    check(mup is not None and abs(mup['depress_deg'] + 5.0) < 0.01,
          "地面往上看 5° → 俯角 −5° (不再當成無效)")
    # 共用接收端 V6 的雲台 yaw (GIMBAL_YAW_MODE)
    py = dict(pkt(960, 540, -30.0), yaw=20.0)
    m_n = measure(py, (0.0, 0.0, 180.0, 180.0), 240.0, cfg, ctrl)
    check(abs(m_n['bearing_deg'] - 180.0) < 0.01, "yaw 'none' (預設): 不用 yaw, 方位 = 機頭 180")
    m_b = measure(py, (0.0, 0.0, 180.0, 180.0), 240.0,
                  XwingVisionConfig(GIMBAL_YAW_MODE='body'), ctrl)
    check(abs(m_b['bearing_deg'] - 200.0) < 0.01, "yaw 'body': 機頭 180 + 雲台 20 = 200")
    m_e = measure(py, (0.0, 0.0, 180.0, 180.0), 240.0,
                  XwingVisionConfig(GIMBAL_YAW_MODE='earth'), ctrl)
    check(abs(m_e['bearing_deg'] - 20.0) < 0.01, "yaw 'earth': 方位 = 雲台 yaw 20")
    m_nan = measure(dict(py, yaw=float('nan')), (0.0, 0.0, 180.0, 180.0), 240.0,
                    XwingVisionConfig(GIMBAL_YAW_MODE='body'), ctrl)
    check(abs(m_nan['bearing_deg'] - 180.0) < 0.01, "yaw = NaN → 退回 'none'")


# ══════════════════════════════════════════════════════════════════
# 1b. UART 協定 (共用接收端 V6: 新格式 0~5 / 10, 舊 v6 / v7 / v8)
# ══════════════════════════════════════════════════════════════════
def test_uart_protocol():
    print("[1b] UART 協定 (共用接收端 V6)")
    import struct
    import xwing_uart_vision as X
    from uart_mipi_receiver import BASE_FMT, decode_frame
    from xwing_uart_sender_sim import _frame
    if not X.UART_OK:
        check(False, f"缺 pyserial / uart_mipi_receiver: {X.UART_IMPORT_ERROR}")
        return
    base = struct.pack(BASE_FMT, 1000.0, 600.0, 40.5, 55.0, 1066.7, 1066.7, 959.5, 545.0,
                       1920.0, 1080.0, -25.0, 0.0, 1, 7, 30.0, 5.0)
    ts = struct.pack('>QI', 123456789, 0x5A17)
    frames = {
        6: _frame(6, 0, base),
        7: _frame(7, 0, base + ts),
        8: _frame(8, 0, base + ts + struct.pack('>f', 300.0)),
        10: _frame(10, 0, base + ts + struct.pack('>ff', 300.0, 12.5)),
        0: _frame(0, 0, base + ts + struct.pack('>ff', 280.0, -3.0)),
    }
    check(len(frames[10]) == 91, f"新格式封包 {len(frames[10])} B = 91 B")
    link = X.UartVisionLink('/dev/null', log=lambda *a: None)
    for ver, fr in frames.items():
        # 夾雜雜訊 byte, 驗證重新同步
        link._buf.extend(b'\x00\xA5' + fr)
        got = link._extract()
        ok = got is not None and got[0] == fr
        if ok:
            link._handle(got[0], got[1], 1_000_000_000)
            d = link.latest()
            ok = d['ver'] == ver and d['target_cx'] == 1000.0 and d['pitch'] == -25.0
        check(ok, f"v{ver}: 解得出來 (ver / 目標 / pitch 正確)")
    d = link.latest()        # 最後一筆是 v0
    check(d['yaw'] == -3.0 and abs(d['laser_distance_m'] - 280.0) < 1e-3,
          f"新格式: yaw {d['yaw']} / 雷射 {d['laser_distance_m']:.0f} m 讀到正確位置")
    check(link.clock.protocol_version == 0, "新格式: 校時請求用對方的版本號")
    link._buf.extend(frames[8])
    link._handle(*link._extract(), 1_000_000_000)
    check(math.isnan(link.latest()['yaw']) and link.clock.protocol_version == 7,
          "舊 v8: yaw = NaN, 校時請求版本 7 (同原本)")
    # 校時回覆 (flags=2) 新版本號也收得到
    reply = _frame(10, 2, struct.pack('>QQQI', 1, 2, 3, 0x5A17))
    link._buf.extend(reply)
    got = link._extract()
    check(got is not None and got[1] is None and decode_frame(reply)[0] is None,
          "新格式校時回覆 (v10 flags=2) 收得到")
    # CRC 壞掉 → 丟棄
    bad = bytearray(frames[10]); bad[20] ^= 0xFF
    n0 = link.stats['crc']
    link._buf.extend(bytes(bad))
    check(link._extract() is None and link.stats['crc'] > n0, "CRC 錯 → 丟棄")


# ══════════════════════════════════════════════════════════════════
# 1c. 地面校正 (gimbal_calib.py)
# ══════════════════════════════════════════════════════════════════
def test_calib():
    print("[1c] 地面校正 gimbal_calib")
    from xwing_vision_config import XwingVisionConfig
    from xwing_vision_common import build_gcfg, control_camera
    from vision_guidance import measure
    from gimbal_calib import estimate

    def run(cfg, rep_pitch, u, v):
        ctrl = control_camera(cfg, build_gcfg(cfg, True, cfg.PREDIVE_DIST_M))
        pk = {'target_cx': u, 'target_cy': v, 'fx': 1000.0, 'fy': 1000.0,
              'cx': 960.0, 'cy': 540.0, 'pitch': rep_pitch, 'roll': 0.0}
        m = measure(pk, (0.0, 0.0, 0.0, 0.0), None, cfg, ctrl, ground=True)
        return estimate([m['depress_deg']], [m['bearing_off_deg']], 0.0 if rep_pitch > -5 else 30.0,
                        0.0, cfg.GIMBAL_PITCH_OFFSET_DEG, cfg.GIMBAL_YAW_OFFSET_DEG)

    cfg = XwingVisionConfig()
    # 俯角: 光軸實際 −30°, 雲台回報 −29°, 標記在真實俯角 30° (畫面正中央)
    r = run(cfg, -29.0, 960.0, 540.0)
    check(abs(r['new_pitch_off'] + 1.0) < 0.01,
          f"回報 −29° 實際 −30° → 建議 GIMBAL_PITCH_OFFSET_DEG = {r['new_pitch_off']:.2f} (應 −1)")
    r2 = run(XwingVisionConfig(GIMBAL_PITCH_OFFSET_DEG=r['new_pitch_off']), -29.0, 960.0, 540.0)
    check(abs(r2['pitch_err']) < 0.01, "改成建議值再量: 俯角偏差 ≈ 0")
    # 方位: 光軸偏機頭右 2°, 標記在機頭正前方 (等高) → 畫面上偏左 2°
    u = 960.0 - 1000.0 * math.tan(math.radians(2.0))
    r = run(cfg, 0.0, u, 540.0)
    check(abs(r['new_yaw_off'] - 2.0) < 0.01,
          f"光軸偏右 2° → 建議 GIMBAL_YAW_OFFSET_DEG = {r['new_yaw_off']:.2f} (應 +2)")
    r2 = run(XwingVisionConfig(GIMBAL_YAW_OFFSET_DEG=r['new_yaw_off']), 0.0, u, 540.0)
    check(abs(r2['yaw_err']) < 0.01, "改成建議值再量: 方位偏差 ≈ 0")


# ══════════════════════════════════════════════════════════════════
# 2. VisionSource (目標輸入來源)
# ══════════════════════════════════════════════════════════════════
def test_source():
    print("[2] VisionSource")
    from xwing_vision_config import XwingVisionConfig
    from xwing_vision_common import build_gcfg, control_camera
    from vision_guidance import VisionSource
    from attitude_time_history import AngleHistory
    import dive_guidance
    cfg = XwingVisionConfig()
    gcfg = build_gcfg(cfg, True, cfg.PREDIVE_DIST_M)
    check(not hasattr(gcfg, 'VISION_GUIDANCE') and not gcfg.DIVE_USE_REAL_CAMERA,
          "狀態機參數 = 原專案 (沒有新增的視覺開關)")
    check(not hasattr(dive_guidance.GuidanceConfig, 'VISION_GUIDANCE'),
          "dive_guidance.py 是原版 (沒有新增條件)")
    src = VisionSource(cfg, control_camera(cfg, gcfg))
    poses = AngleHistory()
    for k in range(20):
        poses.append(10.0 + 0.02 * k, (0.0, 0.0, 180.0, 180.0))

    def pkt(rid, u, v, t, pitch=-30.0):
        return {'target_cx': u, 'target_cy': v, 'fx': 1000.0, 'fy': 1000.0,
                'cx': 960.0, 'cy': 540.0, 'pitch': pitch, 'roll': 0.0,
                'result_id': rid, 'valid': not math.isnan(u),
                'cap_mono': t - 0.05, 'rx_mono': t}
    # 飛機在 NED (600, 0), 往南, 車在畫面正中央 → 目標位置 ≈ (600 − 415.7, 0)
    f, _ = src.get_fields(pkt(1, 960, 540, 10.2), 10.2, poses, 240.0, 600.0, 0.0)
    check(f['tgt_valid'] and abs(f['predive_dist_m'] - 415.69) < 0.5,
          f"看得到: tgt_valid=1, predive_dist_m={f['predive_dist_m']:.1f} (= 偵測水平距)")
    check(abs(f['tgt_n'] - (600 - 415.69)) < 0.5 and abs(f['tgt_e']) < 0.5,
          f"看得到: 目標位置 ({f['tgt_n']:.1f},{f['tgt_e']:.1f}) → 原邏輯朝它飛")
    check(f['bbox_detected'] and f['bbox_u'] is not None,
          "看得到: bbox_detected / bbox_u/v 有值 (DIVE PID 用)")
    nan = float('nan')
    f, row = src.get_fields(pkt(2, nan, nan, 10.25), 10.25, poses, 240.0, 590.0, 0.0)
    check(not f['tgt_valid'] and f['predive_dist_m'] is None and not f['bbox_detected'],
          "這筆沒目標: tgt_valid=0, 沒有距離 (不會推頭), bbox_detected=0")
    check(f['tgt_n'] is not None and row['vg_reason'] == 'no_target',
          "這筆沒目標: 目標位置保持最後一次看到的 (原邏輯繼續朝它飛)")
    f, _ = src.get_fields(pkt(3, 960, 540, 10.3), 10.3, poses, 240.0, 580.0, 0.0)
    check(f['tgt_valid'], "又看到 → tgt_valid=1")
    f, row = src.get_fields(pkt(3, 960, 540, 10.3), 10.3 + cfg.VISION_PKT_MAX_AGE_S + 0.01,
                            poses, 240.0, 570.0, 0.0)
    check(not f['tgt_valid'] and row['vg_reason'] == 'stale',
          "封包不再更新 (資料鏈斷) → 看不到")
    # 雲台 yaw (對地, 相機朝機頭) 對照: 飛機 180°, 雲台回報 181.5° → 差 +1.5°; 沒車也記
    f, row = src.get_fields(dict(pkt(4, nan, nan, 10.35), yaw=181.5), 10.35, poses,
                            240.0, 570.0, 0.0)
    check(row['vg_ac_yaw_deg'] == 180.0 and row['vg_gimbal_yaw_deg'] == 181.5
          and abs(row['vg_yaw_diff_deg'] - 1.5) < 1e-6,
          "雲台 yaw 對照: 飛機 180 / 雲台 181.5 → 差 +1.5° (沒車也記錄)")
    f, row = src.get_fields(dict(pkt(5, nan, nan, 10.4), yaw=-179.0), 10.4, poses,
                            240.0, 570.0, 0.0)
    check(abs(row['vg_yaw_diff_deg'] - 1.0) < 1e-6, "雲台 yaw 對照: −179° 與 180° 差 +1° (跨 ±180 正確)")
    check(f['tgt_n'] is not None and abs(f['tgt_e']) < 0.5,
          "雲台 yaw 只記錄, 預設 'none' 不影響目標位置")
    src.reset()
    f, _ = src.get_fields(None, 11.0, poses, 240.0, 560.0, 0.0)
    check(f['tgt_n'] is None and not f['tgt_valid'],
          "新的一趟: 忘掉上一趟的目標 (主程式改用虛擬目標帶路)")

    # ── 推頭依據: 角度 (預設) vs 距離 ──
    thr = gcfg.PREDIVE_DIST_M
    check(cfg.PREDIVE_TRIGGER == 'angle', "預設推頭依據 = 看車俯角 (angle)")
    src_d = VisionSource(XwingVisionConfig(PREDIVE_TRIGGER='distance'),
                         control_camera(cfg, gcfg), predive_dist_m=thr)

    def pd(s_, agl, ground, rid):
        # 車在畫面正中央, 雲台往下的角度 = 剛好看到地面 ground m 外的車
        dep = math.degrees(math.atan2(agl, ground))
        s_.reset()
        f_, row_ = s_.get_fields(pkt(rid, 960, 540, 10.2, pitch=-dep), 10.2, poses,
                                 agl, 600.0, 0.0)
        return f_['predive_dist_m'], row_, dep

    a240, row, _ = pd(src, 240.0, 415.69, 10)
    d240, _, _ = pd(src_d, 240.0, 415.69, 10)
    check(abs(a240 - thr) < 0.5 and abs(d240 - thr) < 0.5,
          "240 m、俯角 30°: 角度與距離都剛好到門檻 (兩者等價)")
    check(row['vg_trigger'] == 'angle' and row['vg_trigger_angle_deg'] == 30.0
          and abs(row['vg_predive_in_m'] - thr) < 0.5, "log 記錄推頭依據 / 觸發角 / 餵給狀態機的值")
    a, _, dep = pd(src, 220.0, 400.0, 11)
    d, _, _ = pd(src_d, 220.0, 400.0, 11)
    check(a > thr and d <= thr,
          f"220 m、車 400 m (俯角 {dep:.1f}° < 30°): 角度不推頭, 距離會推頭")
    a, _, dep = pd(src, 260.0, 430.0, 12)
    d, _, _ = pd(src_d, 260.0, 430.0, 12)
    check(a <= thr and d > thr,
          f"260 m、車 430 m (俯角 {dep:.1f}° > 30°): 角度推頭, 距離還不推頭")
    src_35 = VisionSource(XwingVisionConfig(PREDIVE_TRIGGER_ANGLE_DEG=35.0),
                          control_camera(cfg, gcfg), predive_dist_m=thr)
    a, _, _ = pd(src_35, 240.0, 415.69, 13)
    check(a > thr, "觸發角改 35°: 俯角 30° 時還不推頭")
    a, _, _ = pd(src_35, 240.0, 240.0 / math.tan(math.radians(35.5)), 14)
    check(a <= thr, "觸發角改 35°: 俯角 35.5° 推頭")
    check(src.trigger_angle_deg == cfg.DIVE_ANGLE_DEG, "觸發角預設 = 俯衝角")

    # ── 地面 / 掛機: 高度 ~0 時偵測照樣有效 (角度), 角度判斷能推頭 ──
    src.reset()
    f, row = src.get_fields(pkt(20, 960, 540, 10.2, pitch=-31.0), 10.2, poses, 0.2, 0.0, 0.0)
    check(f['tgt_valid'] and f['predive_dist_m'] is not None and f['predive_dist_m'] <= thr
          and row['vg_ground'] == 1 and row['vg_geom'] == 'angle',
          "地面高度 0.2 m、俯角 31°: 看得到, 角度判斷達到推頭條件")
    check(f['tgt_n'] is None and f['ground_m'] is None and row['vg_ground_m'] == '',
          "地面: 沒有距離 / 目標位置 (航向用虛擬目標帶路)")
    f, _ = src_d.get_fields(pkt(21, 960, 540, 10.2, pitch=-31.0), 10.2, poses, 0.2, 0.0, 0.0)
    check(f['tgt_valid'] and f['predive_dist_m'] is None,
          "地面 + 距離判斷: 看得到但沒有距離 → 不推頭")
    src.reset()
    f, row = src.get_fields(pkt(22, 960, 540, 10.2, pitch=-31.0), 10.2, poses, 5.0, 0.0, 0.0)
    check(f['tgt_valid'] and row['vg_geom'] == 'full' and row['vg_ground'] == 1,
          "地面但氣壓高度 5 m: 角度 + 距離都算 (距離僅供參考)")
    src.reset()
    f, row = src.get_fields(pkt(23, 960, 0, 10.2, pitch=20.0), 10.2, poses, 240.0, 0.0, 0.0)
    check(not f['tgt_valid'] and row['vg_reason'] == 'ray_up' and row['vg_ground'] == 0,
          "空中 240 m 射線朝上 → 照舊判無效 (ray_up)")


# ══════════════════════════════════════════════════════════════════
# 3. 閉迴路模擬
# ══════════════════════════════════════════════════════════════════
def test_sim():
    print("[3] 閉迴路模擬")
    from xwing_vision_sim import simulate
    from xwing_vision_config import XwingVisionConfig
    # 推頭依據: 實際高度不是 240 m 時, 角度判斷仍在真實視線 30° 推頭
    for agl in (220.0, 260.0):
        ra = simulate(car_case='stopped', fly_agl=agl)
        rd = simulate(cfg=XwingVisionConfig(PREDIVE_TRIGGER='distance'),
                      car_case='stopped', fly_agl=agl)
        la, ld_ = ra['predive']['los'], rd['predive']['los']
        check(abs(la - 30.0) < 0.7 and abs(ld_ - 30.0) > 1.5,
              f"實際高度 {agl:.0f} m: 角度判斷推頭時真實視線 {la:.1f}° ≈ 30°, "
              f"距離判斷 {ld_:.1f}°")
    # 目標區地面高低不同: 兩種判斷都在 30° (飛控高度 = 240, 誤差互相抵銷)
    for dz in (-15.0, 15.0):
        ra = simulate(car_case='stopped', ground_dz=dz)
        rd = simulate(cfg=XwingVisionConfig(PREDIVE_TRIGGER='distance'),
                      car_case='stopped', ground_dz=dz)
        check(abs(ra['predive']['los'] - 30.0) < 0.7
              and abs(ra['predive']['t'] - rd['predive']['t']) < 0.05,
              f"地面差 {dz:+.0f} m: 角度 / 距離同時推頭 (真實視線 {ra['predive']['los']:.1f}°)")
    # 車照虛擬目標開: 幾何與虛擬控制相同 → miss 應該差不多 (純追擊無前置, 兩者都打在車後)
    r = simulate(vision=True, car_case='virtual')
    rvv = simulate(vision=False, car_case='virtual')
    check(r['miss_car'] is not None
          and abs(r['miss_car']['miss_total'] - rvv['miss_car']['miss_total']) < 10.0,
          f"車照虛擬目標開: 偵測 miss {r['miss_car']['miss_total']:.1f} m ≈ "
          f"虛擬軌跡 {rvv['miss_car']['miss_total']:.1f} m (同一套判斷邏輯)")
    check(r['predive'] and abs(r['predive']['vis'] - r['predive']['true']) < 15.0,
          "推頭時偵測水平距與真實相差 < 15 m")
    # 車慢 (11/04): 偵測在「真實」415 m 推頭; 虛擬軌跡被假想車騙, 推頭時真車只剩 ~340 m
    rv = simulate(vision=True, car_case='slow')
    rb = simulate(vision=False, car_case='slow')
    check(abs(rv['predive']['true'] - 415.7) < 15.0,
          f"車慢: 偵測推頭時真實水平距 {rv['predive']['true']:.0f} m ≈ 416")
    check(abs(rb['predive']['true'] - 415.7) > 50.0,
          f"車慢: 虛擬軌跡推頭時真實水平距 {rb['predive']['true']:.0f} m (偏離 > 50 m)")
    # 車停著沒開: 偵測打得到, 虛擬軌跡打在假想車的位置
    rs = simulate(vision=True, car_case='stopped')
    rsb = simulate(vision=False, car_case='stopped')
    check(rs['miss_car']['miss_total'] < 20.0 < rsb['miss_car']['miss_total'],
          f"車沒開: 偵測 miss {rs['miss_car']['miss_total']:.1f} m, "
          f"虛擬軌跡 {rsb['miss_car']['miss_total']:.1f} m")
    rbo = simulate(vision=True, car_case='slow', blackout=(5.5, 7.5))
    check(rbo['abort'] == 'TARGET_LOST_DIVE' and rbo['miss_car'] is None,
          "DIVE 中資料鏈斷 2 s → 原邏輯 TARGET_LOST_DIVE 拉起中止")
    rn = simulate(vision=True, car_case='none')
    check(rn['predive'] is None and rn['miss_car'] is None,
          "沒有車 → 沒有距離, 不推頭 (原邏輯 SEARCH 沒有逾時)")
    rd = simulate(vision=True, car_case='slow', dropout=0.3)
    check(rd['miss_car'] is not None and abs(rd['predive']['true'] - 415.7) < 20.0,
          "30% 漏偵測: 仍在 ~416 m 推頭並完成俯衝")


# ══════════════════════════════════════════════════════════════════
# 4. 實飛主程式 (假 ROS)
# ══════════════════════════════════════════════════════════════════
def run_flight(script, car_present=True, link_ok_from=0.0, case='vtest', stop_at=60.0,
               bench=False):
    """bench=True: 飛機放在地上 (高度 0.3 m, 不動) + --bench --bench-dive;
    假雲台看著正前方的標記, 每秒往下多看 1° (20° → 40°)."""
    """跑一次 xwing_vision_flight.main(). script = [(t_rel, guided), ...]."""
    import importlib
    import time as _time
    clk = {'t': 1000.0}
    real_time, real_mono = _time.time, _time.monotonic
    _time.time = lambda: clk['t']
    _time.monotonic = lambda: clk['t']
    T0 = clk['t']
    DT = 0.02
    stop = {'v': False}

    rospy = types.ModuleType('rospy')

    class Rate:
        def __init__(self, hz):
            pass

        def sleep(self):
            step()
    rospy.Rate = Rate
    rospy.init_node = lambda *a, **k: None
    rospy.is_shutdown = lambda: stop['v']
    rospy.sleep = lambda s: step()
    sys.modules['rospy'] = rospy
    for mod, names in {
            'mavros_msgs': [], 'mavros_msgs.msg': ['State', 'RCOut', 'AttitudeTarget',
                                                   'NavControllerOutput', 'VFR_HUD',
                                                   'HomePosition'],
            'mavros_msgs.srv': ['ParamGet'], 'sensor_msgs': [],
            'sensor_msgs.msg': ['NavSatFix', 'Imu', 'BatteryState', 'TimeReference'],
            'std_msgs': [], 'std_msgs.msg': ['Float64'], 'geometry_msgs': [],
            'geometry_msgs.msg': ['TwistStamped']}.items():
        m = types.ModuleType(mod)
        for n in names:
            setattr(m, n, type(n, (), {}))
        sys.modules[mod] = m

    import xwing_vision_flight as F
    importlib.reload(F)
    import xwing_uart_vision
    from attitude_time_history import AngleHistory
    from vision_sim_camera import SimCar, SimGimbalCamera

    cfg = F.XwingVisionConfig()
    S0 = cfg.MOVING_START_GPS
    R = 6378137.0

    def ne2gps(n, e):
        return (S0[0] + math.degrees(n / R),
                S0[1] + math.degrees(e / (R * math.cos(math.radians(S0[0])))))

    AC = {'n': 613.0, 'e': 0.0, 'h': 240.0, 'V': 30.0, 'pitch': 0.0, 'chi': 180.0}
    if bench:
        AC.update(n=0.0, e=0.0, h=0.3, V=0.0)
    BR = {'out': None}
    t_guided = next((tr for tr, g in script if g), 0.0)
    car = SimCar(0.0, 0.0, 180.0, t_start=t_guided, vmax=10.0, present=car_present)
    cam = SimGimbalCamera(seed=3)
    poses = AngleHistory(size=3000)

    def guided_now():
        g = False
        for tr, gv in script:
            if clk['t'] - T0 >= tr:
                g = gv
        return g

    def step():
        clk['t'] += DT
        trel = clk['t'] - T0
        if bench:                      # 地上不動, 只記姿態
            poses.append(clk['t'], (0.0, 0.0, AC['chi'] % 360, AC['chi'] % 360))
            if trel > stop_at:
                stop['v'] = True
            return
        out = BR['out']
        g = guided_now()
        active = g and out and out.get('cmd_active')
        if active and out['mode'] not in ('DONE', 'SEARCH'):
            tgt = out['pitch_cmd_deg']
        else:
            tgt = max(-5, min(8, 0.5 * (240 - AC['h'])))
        AC['pitch'] += max(-15 * DT, min(15 * DT, tgt - AC['pitch']))
        if active and out['mode'] in ('SEARCH', 'DIVE'):
            err = (out['heading_cmd_deg'] - AC['chi'] + 180) % 360 - 180
            AC['chi'] += max(-10 * DT, min(10 * DT, err / 2.0 * DT * 50))
        if g and out and out['mode'] == 'DIVE':
            AC['V'] = min(60, AC['V'] + 3 * DT)
        elif not (g and out and out['mode'] == 'PRE_DIVE'):
            AC['V'] += (30 - AC['V']) * 0.02
        v, c = AC['V'], math.radians(AC['chi'])
        AC['h'] += v * math.sin(math.radians(AC['pitch'])) * DT
        hv = v * math.cos(math.radians(AC['pitch']))
        AC['n'] += hv * math.cos(c) * DT
        AC['e'] += hv * math.sin(c) * DT
        poses.append(clk['t'], (0.0, AC['pitch'], AC['chi'] % 360, AC['chi'] % 360))
        cam.enabled = trel >= link_ok_from
        cam.step(trel, DT, {'n': AC['n'], 'e': AC['e'], 'agl': AC['h'],
                            'yaw': AC['chi'] % 360}, car)
        if trel > stop_at:
            stop['v'] = True

    class FakeIO:
        def __init__(self):
            self.pose_history = poses

        def snapshot(self):
            lat, lon = ne2gps(AC['n'], AC['e'])
            g = guided_now()
            c = math.radians(AC['chi'])
            hv = AC['V'] * math.cos(math.radians(AC['pitch']))
            return {'guided': g, 'lat': lat, 'lon': lon, 'rel_alt': AC['h'],
                    'compass_hdg': AC['chi'] % 360, 'pitch_deg': AC['pitch'],
                    'roll_deg': 0.0, 'airspeed': AC['V'],
                    'vn': hv * math.cos(c), 've': hv * math.sin(c),
                    'vd': -AC['V'] * math.sin(math.radians(AC['pitch'])),
                    'yaw_rate_dps': 0.0, 'gps_unix': None, 'p_dps': 0, 'q_dps': 0,
                    'r_dps': 0, 'nz_g': 1, 'fc_pitch_tgt_deg': None,
                    'fc_nav_pitch_deg': None, 'fc_nav_roll_deg': None,
                    'fc_nav_bearing_deg': None, 'fc_mode': 'GUIDED' if g else 'AUTO',
                    'batt_v': 16.0, 'rcout': [1500] * 8}

    class FakeBridge:
        def __init__(self, **k):
            pass

        def start(self):
            pass

        def stop(self):
            pass

        def submit(self, out, *a, **k):
            BR['out'] = out

        def get_debug(self):
            return {}

        def send_tecs_targets_async(self, alt_m=None, speed_mps=None, tag='', on_done=None):
            if on_done:
                on_done(True, True)

    class FakeLink:
        """用 SimGimbalCamera 冒充 UartVisionLink (同一組方法)."""
        def __init__(self, port, baud=115200, **kw):
            self.port, self.baud = port, baud
            self.connected = True
            self.stats = {'frames': 0, 'crc': 0, 'bytes': 0, 'tx_bytes': 0,
                          'serial_err': 0}
            self._req = None

        def start(self):
            pass

        def stop(self):
            pass

        def latest(self):
            if bench:                  # 地面標記: 畫面正中央, 雲台俯角 20° 起每秒 +1°
                trel = clk['t'] - T0
                return {'target_cx': 960.0, 'target_cy': 540.0,
                        'offset_x': 0.0, 'offset_y': 0.0,
                        'fx': 1000.0, 'fy': 1000.0, 'cx': 960.0, 'cy': 540.0,
                        'image_width': 1920.0, 'image_height': 1080.0,
                        'pitch': -min(40.0, 20.0 + trel), 'roll': 0.0, 'yaw': AC['chi'],
                        'mode': int(self._req) if self._req else 0,
                        'result_id': int(trel * 30), 'detection_age_ms': 30.0,
                        'angle_age_ms': 5.0, 'laser_distance_m': float('nan'), 'ver': 10,
                        'cap_mono': clk['t'] - 0.04, 'rx_mono': clk['t'] - 0.01,
                        'e2e_ms': 30.0, 'e2e_err_ms': 1.0, 'valid': True}
            # 時鐘: 模擬相機用相對時間, 封包時間換回假時鐘的 monotonic
            p = cam.latest()
            if p is None:
                return None
            p = dict(p)
            p['rx_mono'] += T0
            p['cap_mono'] += T0
            return p

        def request_mode(self, key):
            self._req = key
            cam.request_mode(key)

        def mode_status(self):
            p = cam.latest()
            return self._req, (self._req is not None and p is not None
                               and p['mode'] == int(self._req))

        def is_locked(self, pkt):
            return (pkt is not None and pkt['mode'] == 2
                    and abs(pkt['pitch'] + 30.0) <= 2.0)

        def mode_timing(self):
            return {'key': self._req, 't_req': None, 't_confirm': None,
                    't_settled': None, 'pitch_settled': None}

    F.MavrosIO = FakeIO
    F.ArduPilotAttitudeBridge = FakeBridge
    F.check_airspeed_params = lambda cfg: None
    F.keyboard_thread = lambda flags: None
    xwing_uart_vision.UartVisionLink = FakeLink

    sys.argv = ['x', '--case', case] + (['--bench', '--bench-dive'] if bench else [])
    try:
        F.main()
    finally:
        _time.time, _time.monotonic = real_time, real_mono
    ev = sorted(glob.glob(os.path.join(REPO, 'flight_logs', f'*{case}_events.txt')))[-1]
    text = open(ev, encoding='utf-8').read()
    c = ev.replace('_events.txt', '.csv')
    rows = list(csv.DictReader(open(c, encoding='utf-8')))
    for p in (ev, c):
        os.remove(p)
    return text, rows


def test_flight():
    print("[4] 實飛主程式 (假 ROS + 模擬雲台)")
    text, rows = run_flight([(2.0, True)], case='vtest_full')
    for key in ('第 1 趟任務開始', '撥 GUIDED 時已偵測到目標', 'MODE: SEARCH → PRE_DIVE',
                'MODE: PRE_DIVE → DIVE', 'MODE: DIVE → PULLOUT', 'MODE: PULLOUT → DONE',
                '偵測到的目標位置', "雲台狀態 → PRE_DIVE: 模式鍵 '2'",
                "雲台狀態 → PULLOUT: 模式鍵 不要求", "雲台狀態 → DONE: 模式鍵 '1'"):
        check(key in text, f"完整一趟: events 有「{key}」")
    pre = next((r for r in rows if r['mode'] == 'PRE_DIVE'), None)
    check(pre is not None and pre['vg_ground_m'] != ''
          and float(pre['vg_ground_m']) <= 416.0,
          "CSV: 推頭那拍的偵測水平距 ≤ 415.69 m")
    check(all(r['control'] == 'vision' for r in rows), "CSV: control = vision")
    check(pre is not None and pre['vg_trigger'] == 'angle' and pre['vg_depress_deg'] != ''
          and float(pre['vg_depress_deg']) >= 29.9,
          "CSV: 推頭那拍的依據 = angle, 看車俯角 ≥ 30°")
    check('推頭: 看車俯角 ≥ 30.0°' in text and '看車俯角=' in text,
          "events: 記錄推頭依據與切換時的看車俯角")
    diffs = [abs(float(r['vg_yaw_diff_deg'])) for r in rows if r.get('vg_yaw_diff_deg')]
    check(len(diffs) > 50 and max(diffs) < 1.0,
          f"CSV: 雲台 yaw (對地, 置中) 對照飛控航向 {len(diffs)} 筆, 最大差 "
          f"{max(diffs) if diffs else float('nan'):.2f}° (模擬兩者一致)")

    # 掛機 (2026/11/12): 飛機在地上, 偵測照樣有效 → 俯角到 30° 推頭
    text, rows = run_flight([(2.0, True)], case='vtest_bench', stop_at=20.0, bench=True)
    # --bench-dive: PRE_DIVE 同一拍就交棒 DIVE, 所以找第一筆離開 SEARCH 的
    pre = next((r for r in rows if r['mode'] in ('PRE_DIVE', 'DIVE')), None)
    check(pre is not None and ('MODE: SEARCH → PRE_DIVE' in text or 'MODE: SEARCH → DIVE' in text)
          and 29.9 <= float(pre['vg_depress_deg']) <= 31.0,
          f"掛機 (高度 0.3 m): 俯角 {pre['vg_depress_deg'] if pre else '?'}° 推頭 (地面偵測有效)")
    srch = [r for r in rows if r['mode'] == 'SEARCH' and r['guided'] in ('1', 'True')]
    check(srch and all(r['vg_valid'] == '1' and r['vg_geom'] == 'angle' for r in srch[5:]),
          "掛機: SEARCH 每拍都看得到 (vg_geom = angle)")

    text, rows = run_flight([(2.0, True)], car_present=False, case='vtest_nocar',
                            stop_at=30.0)
    after = text.split('第 1 趟任務開始')[1]
    check('MODE: SEARCH → PRE_DIVE' not in after and rows[-1]['mode'] == 'SEARCH',
          "沒有車: 一直沒有距離 → 不推頭, 停在 SEARCH (原邏輯)")


def main():
    test_geometry()
    test_uart_protocol()
    test_calib()
    test_source()
    test_sim()
    test_flight()
    print()
    if FAILS:
        print(f"FAIL ({len(FAILS)} 項)")
        for f in FAILS:
            print("  - " + f)
        sys.exit(1)
    print("PASS")


if __name__ == '__main__':
    main()
