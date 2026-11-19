#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gimbal_calib.py — 地面校正雲台俯角 / 方位 (算 GIMBAL_PITCH_OFFSET_DEG / GIMBAL_YAW_OFFSET_DEG)
═══════════════════════════════════════════════════════════════════
不需要 ROS / 飛控, 只接雲台 UART. 讓雲台看著一個「真實角度已知」的標記,
收集幾秒偵測結果, 用跟飛行程式同一套幾何 (vision_guidance.measure, 地面模式) 算出
看標記的俯角與偏機頭角度, 跟真實值比 → 印出 config 該填的值.

擺法 (飛機放水平):
  A. 標記跟鏡頭等高、在機頭正前方的延長線上, 距離 30 m 以上
       → 真實俯角 0°, 真實偏機頭 0° (預設, 什麼都不用加)
  B. 標記在地上 (比鏡頭低), 量好距離 D 與高度差 H
       → --distance D --height H         真實俯角 = atan(H / D)
  C. 標記不在機頭正前方 → --right-m R (標記在機頭延長線右邊 R m, 左邊給負值; 要配 --distance)

用法:
  python3 gimbal_calib.py                                  # 擺法 A, 收 10 s
  python3 gimbal_calib.py --distance 40 --height 1.2       # 擺法 B
  python3 gimbal_calib.py --seconds 20 --port /dev/ttyUSB0
  python3 gimbal_calib.py --from-log flight_logs/xwing_vision_XXXX_bench.csv --distance 40 --height 1.2
       (不接雲台: 拿掛機時 xwing_vision_flight.py --bench 記的 log 來算)

注意:
  · 算的是「現在 config 的值」要再加多少; 改完 config 再跑一次, 偏差應該 ≈ 0.
  · 方位只對 GIMBAL_YAW_MODE='none' (相機朝機頭) 有意義.
  · 雲台在 '1' 追蹤模式會把標記拉到畫面中央, 結果一樣 (程式有把畫面偏移算進去).
"""
import argparse
import csv
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from xwing_vision_config import XwingVisionConfig  # noqa: E402


def true_angles(args):
    """(真實俯角, 真實偏機頭) 度."""
    dep = args.true_depress
    right = args.true_right
    if args.distance is not None:
        if args.height is not None:
            dep = math.degrees(math.atan2(args.height, args.distance))
        if args.right_m is not None:
            right = math.degrees(math.atan2(args.right_m, args.distance))
    elif args.height is not None or args.right_m is not None:
        sys.exit('--height / --right-m 要配 --distance')
    return dep, right


def _median(x):
    s = sorted(x)
    n = len(s)
    if n == 0:
        return None
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def _iqr(x):
    s = sorted(x)
    if len(s) < 4:
        return None
    return s[(3 * len(s)) // 4] - s[len(s) // 4]


def estimate(dep_samples, off_samples, true_dep, true_off, cur_pitch_off, cur_yaw_off):
    """量到的俯角 / 偏機頭 (用現在的 offset 算的) → 建議的新 offset.
    俯角: 量到比真實小 (少算) = 光軸其實更往下 → pitch offset 要更負.
          新值 = 現在 + (量到 − 真實)
    方位: 量到偏左 (負) 而標記其實在正前方 = 光軸偏右 → yaw offset 加回來.
          新值 = 現在 + (真實 − 量到)"""
    md, mo = _median(dep_samples), _median(off_samples)
    res = {'n': len(dep_samples), 'dep_med': md, 'dep_iqr': _iqr(dep_samples),
           'off_med': mo, 'off_iqr': _iqr(off_samples),
           'true_dep': true_dep, 'true_off': true_off,
           'pitch_err': None, 'yaw_err': None, 'new_pitch_off': None, 'new_yaw_off': None}
    if md is not None:
        res['pitch_err'] = md - true_dep
        res['new_pitch_off'] = cur_pitch_off + (md - true_dep)
    if mo is not None:
        res['yaw_err'] = mo - true_off
        res['new_yaw_off'] = cur_yaw_off + (true_off - mo)
    return res


def report(res, cfg):
    def f(x, fmt='{:+.2f}'):
        return fmt.format(x) if x is not None else '—'
    print('=' * 66)
    print(f' 收到 {res["n"]} 筆有目標的偵測')
    if not res['n']:
        print(' ★ 沒有資料: 雲台有沒有看到標記? (對方畫面上要有框) / 串口對不對')
        print('=' * 66)
        return
    print(f' 俯角     量到 {f(res["dep_med"], "{:.2f}")}° (四分位距 {f(res["dep_iqr"], "{:.2f}")}°)'
          f'  真實 {res["true_dep"]:.2f}°  → 量到 − 真實 {f(res["pitch_err"])}°')
    print(f' 偏機頭   量到 {f(res["off_med"])}° (四分位距 {f(res["off_iqr"], "{:.2f}")}°)'
          f'  真實 {res["true_off"]:+.2f}°  → 量到 − 真實 {f(res["yaw_err"])}°')
    print('-' * 66)
    print(' xwing_vision_config.py 建議改成:')
    print(f'    GIMBAL_PITCH_OFFSET_DEG = {res["new_pitch_off"]:.2f}'
          f'     # 現在 {cfg.GIMBAL_PITCH_OFFSET_DEG:.2f}')
    print(f'    GIMBAL_YAW_OFFSET_DEG = {res["new_yaw_off"]:.2f}'
          f'       # 現在 {cfg.GIMBAL_YAW_OFFSET_DEG:.2f}')
    ok_p = abs(res['pitch_err']) < 0.3
    ok_y = abs(res['yaw_err']) < 0.5
    print(f' 俯角 {"✓ 已經準了 (< 0.3°), 不用改" if ok_p else "⚠ 要改 (偏差會讓推頭早 / 晚這麼多度)"}')
    print(f' 方位 {"✓ 已經準了 (< 0.5°), 不用改" if ok_y else "⚠ 要改"}')
    if abs(res['pitch_err']) > 5.0 or abs(res['yaw_err']) > 5.0:
        print(' ★★ 偏差超過 5°: 通常不是安裝誤差, 先檢查 — 雲台看的是不是標記? 距離 / 高度量對了嗎?'
              ' 飛機有沒有放水平? 不確定就不要照填')
    for k, lab in (('dep_iqr', '俯角'), ('off_iqr', '方位')):
        if res[k] is not None and res[k] > 1.0:
            print(f' ⚠ {lab}跳動大 (四分位距 {res[k]:.1f}°): 標記太小 / 太遠 / 雲台還在動, 建議重測')
    print(' 改完 config 再跑一次, 量到 − 真實 應該 ≈ 0.')
    print('=' * 66)


def collect_live(args, cfg):
    from xwing_uart_vision import UartVisionLink, UART_OK, UART_IMPORT_ERROR
    from vision_guidance import measure
    from xwing_vision_common import build_gcfg, control_camera
    if not UART_OK:
        sys.exit(f'需要 pyserial 與 uart_mipi_receiver.py: {UART_IMPORT_ERROR}')
    ctrl = control_camera(cfg, build_gcfg(cfg, True, cfg.PREDIVE_DIST_M))
    port = args.port or cfg.VISION_UART_PORT
    link = UartVisionLink(port, cfg.VISION_UART_BAUD, log=lambda *a: None)
    link.start()
    print(f'[calib] 雲台 UART {port}; 收 {args.seconds:.0f} s (Ctrl+C 提早結束)')
    if args.mode is not None:
        link.request_mode(str(args.mode))
        print(f"[calib] 送模式鍵 '{args.mode}'")
    dep, off = [], []
    last_rid, t_end, t_print = None, time.monotonic() + args.seconds, 0.0
    try:
        while time.monotonic() < t_end:
            p = link.latest()
            now = time.monotonic()
            if p is not None and p['result_id'] != last_rid and now - p['rx_mono'] < 0.5:
                last_rid = p['result_id']
                if p.get('valid'):
                    # 地面模式, 飛機姿態 (0,0,0,0): 偏機頭 = 相機方位 − 機頭 (含現在的 YAW_OFFSET)
                    m = measure(p, (0.0, 0.0, 0.0, 0.0), None, cfg, ctrl, ground=True)
                    if m is not None:
                        dep.append(m['depress_deg'])
                        off.append(m['bearing_off_deg'])
            if now - t_print >= 1.0:
                t_print = now
                tail = (f'俯角 {dep[-1]:6.2f}°  偏機頭 {off[-1]:+6.2f}°' if dep else '還沒看到標記')
                gp = f"{p['pitch']:.2f}" if p is not None else '-'
                md = p['mode'] if p is not None else '-'
                print(f'[calib] {len(dep):4d} 筆  {tail}  (雲台 pitch {gp}, mode {md})')
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
    return dep, off


def collect_log(path):
    dep, off = [], []
    with open(path, newline='', encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if r.get('vg_valid') == '1' and r.get('vg_new') == '1':
                try:
                    dep.append(float(r['vg_depress_deg']))
                    off.append(float(r['vg_bearing_off_deg']))
                except (KeyError, ValueError):
                    pass
    return dep, off


def main():
    ap = argparse.ArgumentParser(description='地面校正雲台俯角 / 方位')
    ap.add_argument('--port', default=None, help='雲台 UART (預設 VISION_UART_PORT)')
    ap.add_argument('--seconds', type=float, default=10.0, help='收集秒數 (預設 10)')
    ap.add_argument('--mode', type=int, choices=(0, 1, 2), default=None,
                    help="開始時送模式鍵 (例 1 = 追蹤; 預設不送, 用雲台現在的模式)")
    ap.add_argument('--distance', type=float, default=None, help='標記到鏡頭的水平距離 m')
    ap.add_argument('--height', type=float, default=None,
                    help='標記比鏡頭低幾 m (高於鏡頭給負值); 要配 --distance')
    ap.add_argument('--right-m', type=float, default=None,
                    help='標記在機頭延長線右邊幾 m (左邊負值); 要配 --distance')
    ap.add_argument('--true-depress', type=float, default=0.0,
                    help='直接給真實俯角 (度, 往下為正; 預設 0 = 標記與鏡頭等高)')
    ap.add_argument('--true-right', type=float, default=0.0,
                    help='直接給真實偏機頭角度 (度, 右為正; 預設 0 = 正前方)')
    ap.add_argument('--from-log', default=None,
                    help='不接雲台: 用掛機 log (xwing_vision_*.csv) 裡地面偵測的角度來算')
    args = ap.parse_args()

    cfg = XwingVisionConfig()
    if cfg.GIMBAL_YAW_MODE != 'none':
        print(f"⚠ GIMBAL_YAW_MODE = {cfg.GIMBAL_YAW_MODE!r}: 方位校正只對 'none' 有意義")
    tdep, toff = true_angles(args)
    print(f'[calib] 真實俯角 {tdep:.2f}°, 真實偏機頭 {toff:+.2f}°; '
          f'現在 GIMBAL_PITCH_OFFSET_DEG={cfg.GIMBAL_PITCH_OFFSET_DEG:.2f}, '
          f'GIMBAL_YAW_OFFSET_DEG={cfg.GIMBAL_YAW_OFFSET_DEG:.2f}')
    if args.from_log:
        dep, off = collect_log(args.from_log)
        print(f'[calib] 從 log 讀到 {len(dep)} 筆 ({args.from_log})')
    else:
        dep, off = collect_live(args, cfg)
    res = estimate(dep, off, tdep, toff, cfg.GIMBAL_PITCH_OFFSET_DEG, cfg.GIMBAL_YAW_OFFSET_DEG)
    report(res, cfg)
    return 0 if res['n'] else 1


if __name__ == '__main__':
    sys.exit(main())
