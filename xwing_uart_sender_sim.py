#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xwing_uart_sender_sim.py — 模擬對方實驗室 Orin NX 的 UART 發送端 (v8, 可選 v7)
═══════════════════════════════════════════════════════════════════
用途: 對方的雲台 / Orin NX 還沒到之前, 在本機驗證接收端與模式鍵流程.

  · 以 --hz 送 v8 遙測封包 (格式與 uart_mipi_receiver.py 完全相同, CRC-16/CCITT-FALSE);
    v8 = v7 + 雷射距離 (--laser-m, 每 3 s 有 1 s 量不到 = NaN); --version 7 送舊格式
  · 回覆 v7 校時請求 (flags=1 → flags=2), 自己的時鐘故意加一個偏移, 驗證校時有效
  · 收到 ASCII '0'/'1'/'2' → 切換回報的 mode 欄位 (模擬雲台切模式)
  · 模擬對方雲台 (2026/10 確認的行為):
      只有 pitch 軸, 往下為負;
      '0' 手動: pitch 停在原角度;
      '1' 追蹤: pitch 自動把目標維持在畫面中心 (偏移量接近 0);
      '2' 鎖定: pitch 以 --slew-dps 轉到 −30° 後固定, 目標在畫面上自由移動
  · --no-target 時目標欄位填 NaN (無目標)

用法:
  # A. 不需要任何硬體 (虛擬串口 pty):
  python3 xwing_uart_sender_sim.py --pty
      → 會印出「接收端請用 /dev/pts/N」, 再開另一個 terminal:
  python3 uart_mipi_receiver.py --port /dev/pts/N          (對方的接收程式)
  python3 xwing_virtual_flight.py --vision-uart --vision-port /dev/pts/N

  # B. 兩個 USB-UART 模組 TX↔RX 交叉 + GND 共接:
  python3 xwing_uart_sender_sim.py --port /dev/ttyUSB1
"""

import argparse
import math
import os
import struct
import sys
import time

from uart_mipi_receiver import (FRAME_MAGIC, HEADER_FMT, HEADER_SIZE, BASE_FMT,
                                BASE_SIZE, CRC_SIZE, crc16_ccitt_false, MODE_NAMES)

CLOCK_OFFSET_NS = 123_456_789_000      # 模擬兩台電腦時鐘不同 (約 123 s)
SESSION = 0x5A17


def _frame(version, flags, payload):
    header = struct.pack(HEADER_FMT, FRAME_MAGIC, version, flags,
                         len(payload), len(payload) ^ 0xFFFF)
    body = header + payload
    return body + struct.pack('>H', crc16_ccitt_false(body[2:]))


def remote_now():
    return time.monotonic_ns() + CLOCK_OFFSET_NS


class Sim:
    def __init__(self, write, args):
        self.write = write
        self.args = args
        self.mode = args.mode
        self.result_id = 0
        self.pitch = args.start_pitch          # 雲台 pitch (度, 往下為負)
        self.t_last = time.monotonic()
        self.rx = bytearray()
        self.t0 = time.monotonic()
        self.sent = 0
        self.clock_replies = 0

    def telemetry(self):
        a = self.args
        t = time.monotonic() - self.t0
        w, h = 1920.0, 1080.0
        fx = fy = 1066.7 * a.zoom
        cx, cy = 959.5, 545.0
        cap = remote_now() - int(a.latency_ms * 1e6)   # 影像是 latency_ms 前拍的
        self.result_id += 1
        now = time.monotonic()
        dt, self.t_last = now - self.t_last, now
        # 目標實際所在的俯角 (度, 往下為負): 在 −20° 附近慢慢變
        los_pitch = -20.0 + 6.0 * math.sin(0.15 * t)
        if self.mode == 1:                     # 追蹤: pitch 追目標俯角
            want = los_pitch
        elif self.mode == 2:                   # 鎖定: 固定 −30°
            want = a.lock_pitch
        else:                                  # 手動: 不動
            want = self.pitch
        step = a.slew_dps * dt
        self.pitch += max(-step, min(step, want - self.pitch))
        pitch = self.pitch
        roll = 0.5 * math.sin(0.3 * t)
        if a.no_target:
            tx = ty = ox = oy = float('nan')
        else:
            # 垂直偏移 = 目標俯角與光軸的差 (往下為正); 追蹤時接近 0
            oy = fy * math.tan(math.radians(pitch - los_pitch))
            ox = 60.0 * math.sin(0.4 * t) if self.mode != 1 else 3.0 * math.sin(2.0 * t)
            if self.mode == 1:
                oy = 3.0 * math.cos(2.0 * t) + oy
            tx, ty = cx + ox, cy + oy
        payload = struct.pack(BASE_FMT, tx, ty, ox, oy, fx, fy, cx, cy, w, h,
                              pitch, roll, self.mode, self.result_id,
                              float(a.latency_ms), 5.0)
        payload += struct.pack('>QI', cap, SESSION)
        if a.version == 8:
            laser = a.laser_m + 5.0 * math.sin(0.2 * t) if (t % 3.0) >= 1.0 else float('nan')
            payload += struct.pack('>f', laser)
        return _frame(a.version, 0, payload)

    def handle_rx(self, data):
        self.rx.extend(data)
        while self.rx:
            if self.rx[:2] == FRAME_MAGIC:
                if len(self.rx) < HEADER_SIZE:
                    return
                _, ver, flags, length, lx = struct.unpack(HEADER_FMT, self.rx[:HEADER_SIZE])
                if not (ver == 7 and flags == 1 and length == 8 and lx == (8 ^ 0xFFFF)):
                    del self.rx[0]
                    continue
                size = HEADER_SIZE + length + CRC_SIZE
                if len(self.rx) < size:
                    return
                frame = bytes(self.rx[:size])
                del self.rx[:size]
                if struct.unpack('>H', frame[-2:])[0] != crc16_ccitt_false(frame[2:-2]):
                    continue
                t2 = remote_now()
                (t1,) = struct.unpack('>Q', frame[HEADER_SIZE:-CRC_SIZE])
                t3 = remote_now()
                self.write(_frame(7, 2, struct.pack('>QQQI', t1, t2, t3, SESSION)))
                self.clock_replies += 1
            elif self.rx[0] in b'012':
                self.mode = self.rx[0] - ord('0')
                print(f"[sim] 收到模式鍵 → {self.mode} ({MODE_NAMES[self.mode]})", flush=True)
                del self.rx[0]
            else:
                if self.rx[0] in b'3456789':
                    print(f"[sim] 收到未定義模式鍵 {chr(self.rx[0])!r} (對方協定目前只有 0/1/2)",
                          flush=True)
                del self.rx[0]


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--pty', action='store_true', help='建立虛擬串口 (不需硬體)')
    g.add_argument('--port', help='實體串口, 例如 /dev/ttyUSB1')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--hz', type=float, default=30.0, help='遙測頻率 (預設 30)')
    ap.add_argument('--latency-ms', type=float, default=40.0,
                    help='模擬取像到送出的延遲 (預設 40 ms)')
    ap.add_argument('--zoom', type=float, default=1.0)
    ap.add_argument('--mode', type=int, choices=(0, 1, 2), default=0,
                    help='開機時回報的模式 (預設 0 手動)')
    ap.add_argument('--no-target', action='store_true', help='目標欄位一律 NaN')
    ap.add_argument('--version', type=int, choices=(7, 8), default=8,
                    help='遙測封包版本 (預設 8 = 含雷射距離)')
    ap.add_argument('--laser-m', type=float, default=300.0, help='v8 模擬雷射距離 m (預設 300)')
    ap.add_argument('--slew-dps', type=float, default=40.0,
                    help='雲台 pitch 轉動速度 °/s (預設 40)')
    ap.add_argument('--lock-pitch', type=float, default=-30.0,
                    help='鎖定模式的 pitch (預設 −30, 往下為負)')
    ap.add_argument('--start-pitch', type=float, default=-10.0,
                    help='開機時雲台 pitch (預設 −10)')
    ap.add_argument('--duration', type=float, default=0.0, help='秒, 0 = 一直跑')
    ap.add_argument('--tx-off-at', type=float, default=None,
                    help='重現 10/29 狀況: 第幾秒起停止送出 (仍照常收模式鍵)')
    ap.add_argument('--tx-off-for', type=float, default=0.0,
                    help='停送幾秒後恢復 (0 = 不恢復)')
    args = ap.parse_args()

    if args.pty:
        import termios
        import tty
        master, slave = os.openpty()
        tty.setraw(slave)
        attrs = termios.tcgetattr(slave)
        attrs[3] &= ~termios.ECHO
        termios.tcsetattr(slave, termios.TCSANOW, attrs)
        os.set_blocking(master, False)
        print(f"[sim] 虛擬串口建立: 接收端請用 {os.ttyname(slave)}", flush=True)

        def write(b):
            os.write(master, b)

        def read():
            try:
                return os.read(master, 4096)
            except BlockingIOError:
                return b''
    else:
        import serial
        ser = serial.Serial(args.port, args.baud, timeout=0)
        print(f"[sim] 使用 {args.port} @ {args.baud}", flush=True)
        write = ser.write

        def read():
            return ser.read(4096)

    t_start = time.monotonic()
    if args.tx_off_at is not None:
        _write_on = write
        _said = {'off': False}

        def write(b):
            el = time.monotonic() - t_start
            off = el >= args.tx_off_at and (args.tx_off_for <= 0
                                            or el < args.tx_off_at + args.tx_off_for)
            if off != _said['off']:
                _said['off'] = off
                print(f"[sim] ★ {el:.1f}s TX {'停止 (仍收模式鍵)' if off else '恢復'}", flush=True)
            if not off:
                _write_on(b)

    sim = Sim(write, args)
    period = 1.0 / args.hz
    nxt = time.monotonic()
    last_print = 0.0
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    print(f"[sim] 以 {args.hz:.0f} Hz 送 v{args.version} 封包 "
          f"({HEADER_SIZE + BASE_SIZE + (16 if args.version == 8 else 12) + CRC_SIZE} B), "
          f"模擬延遲 {args.latency_ms:.0f} ms, Ctrl+C 結束", flush=True)
    try:
        while t_end is None or time.monotonic() < t_end:
            data = read()
            if data:
                sim.handle_rx(data)
            now = time.monotonic()
            if now >= nxt:
                try:
                    write(sim.telemetry())
                    sim.sent += 1
                except OSError:
                    pass                       # pty 另一端還沒開
                nxt += period
                if nxt < now:
                    nxt = now + period
            if now - last_print >= 5.0:
                last_print = now
                print(f"[sim] 已送 {sim.sent} 筆, 校時回覆 {sim.clock_replies} 次, "
                      f"目前模式 {sim.mode} ({MODE_NAMES[sim.mode]}), "
                      f"雲台 pitch {sim.pitch:+.1f}°", flush=True)
            time.sleep(0.002)
    except KeyboardInterrupt:
        pass
    print(f"[sim] 結束: 送 {sim.sent} 筆, 校時回覆 {sim.clock_replies} 次", flush=True)


if __name__ == '__main__':
    sys.exit(main())
