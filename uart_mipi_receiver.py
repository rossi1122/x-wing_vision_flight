# 共用雲台接收端 V6
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共用雲台接收端 V6（台鴞／田屋 30 倍／田屋 10 倍），亦可讀取舊格式 v6／v7／v8，使用 big-endian 與 CRC-16/CCITT-FALSE。

10X_V0 資料區 81 bytes，完整封包 91 bytes；末端新增獨立 yaw；NaN/Inf 顯示為 N/A。
V0 使用 UART 往返校時估算取像至接收驗證延遲，並顯示校時不確定範圍。
"""

import argparse
import math
import os
import sys
from contextlib import contextmanager
import struct
import time
from collections import deque

import serial

DISPLAY_INTERVAL = 0.2  # 中文數值顯示間隔（秒），不影響 UART 收包
WRITE_TIMEOUT = 0.1  # 模式命令寫入逾時（秒）
MODE_NAMES = ("手動", "追蹤", "鎖定")
FIELD_LABELS = (
    "目標中心 X", "目標中心 Y", "水平偏移量", "垂直偏移量",
    "水平焦距 fx", "垂直焦距 fy", "光學中心 X", "光學中心 Y",
    "影像寬度", "影像高度", "雲台 Pitch 角度", "雲台 Roll 角度", "目前模式",
    "result_id", "detection_age_ms", "angle_age_ms", "雷射距離", "雲台 Yaw 角度",
)

# Orin NX 接 USB 轉 UART：請用 python3 -m serial.tools.list_ports -v 確認裝置。
# 常見為 /dev/ttyUSB0 或 /dev/ttyACM0；ttyTHS0 是板載 UART。
# 可用環境變數指定：UART_PORT=/dev/ttyUSB0 python3 uart_mipi_receiver.py
UART_PORT = os.getenv("UART_PORT", "/dev/ttyUSB0")
UART_BAUD = int(os.getenv("UART_BAUD", "115200"))  # 必須與發送端相同
RX_DIAGNOSTIC_SEC = 3.0  # 多久未收到有效封包就顯示診斷
FRAME_MAGIC = b"\xA5\x5A"
HEADER_FMT = ">2sBBHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
CRC_SIZE = 2
BASE_FMT = ">12fBI2f"
BASE_SIZE = struct.calcsize(BASE_FMT)
FRAME_SIZE = HEADER_SIZE + BASE_SIZE + CRC_SIZE
BASE_FIELDS = [
    ("target_cx", "px"),
    ("target_cy", "px"),
    ("offset_x", "px"),
    ("offset_y", "px"),
    ("fx", "px"),
    ("fy", "px"),
    ("cx", "px"),
    ("cy", "px"),
    ("image_width", "px"),
    ("image_height", "px"),
    ("pitch", "deg"),
    ("roll", "deg"),
    ("mode", ""),
    ("result_id", ""),
    ("detection_age_ms", "ms"),
    ("angle_age_ms", "ms"),
    ("laser_distance_m", "m"),
    ("yaw", "deg"),
]


def crc16_ccitt_false(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def decode_base(payload):
    if len(payload) != BASE_SIZE:
        raise ValueError(f"Expected {BASE_SIZE} payload bytes, got {len(payload)}")
    values = struct.unpack(BASE_FMT, payload)
    if values[12] not in (0, 1, 2):
        raise ValueError("Invalid mode")
    return values


def decode_frame(frame):
    if len(frame) < HEADER_SIZE+CRC_SIZE:
        raise ValueError('封包過短')
    magic, version, flags, length, length_xor = struct.unpack(
        HEADER_FMT, frame[:HEADER_SIZE]
    )
    expected = {(6, 0): BASE_SIZE, (7, 0): BASE_SIZE+12, (8, 0): BASE_SIZE+16, (0, 0): BASE_SIZE+20, (1, 0): BASE_SIZE+20, (2, 0): BASE_SIZE+20, (3, 0): BASE_SIZE+20, (4, 0): BASE_SIZE+20, (5, 0): BASE_SIZE+20, (10, 0): BASE_SIZE+20, (0, 2): 28, (1, 2): 28, (2, 2): 28, (3, 2): 28, (4, 2): 28, (5, 2): 28, (10, 2): 28, (7, 2): 28}.get((version, flags))
    if magic != FRAME_MAGIC or expected is None:
        raise ValueError('不支援的封包版本或類型')
    if length != expected or length_xor != (length ^ 0xFFFF) or len(frame) != HEADER_SIZE+length+CRC_SIZE:
        raise ValueError("Invalid payload length / LENGTH_XOR")
    crc_recv = struct.unpack(">H", frame[-CRC_SIZE:])[0]
    if crc_recv != crc16_ccitt_false(frame[2:-CRC_SIZE]):
        raise ValueError("CRC mismatch")
    if flags == 2:
        return None, crc_recv
    values = decode_base(frame[HEADER_SIZE:HEADER_SIZE+BASE_SIZE])
    distance = struct.unpack_from('>f', frame, HEADER_SIZE+BASE_SIZE+12)[0] if version in (0, 1, 2, 3, 4, 5, 10, 8) else float('nan')
    if not math.isfinite(distance) or distance < 0:
        distance = float('nan')
    yaw = struct.unpack_from('>f', frame, HEADER_SIZE+BASE_SIZE+16)[0] if version in (0, 1, 2, 3, 4, 5, 10) else float('nan')
    return values + (distance, yaw), crc_recv


class UARTClock:
    """以四個單調時間估算時差；最短往返樣本優先，不假定結果精確無誤。"""
    def __init__(self):
        self.pending = {}
        self.protocol_version = 0
        self.samples = deque(maxlen=16)
        self.last_request = 0
        self.seen = deque(maxlen=1024)
        # 前段只能填實測校正值；未設定時不假造 MIPI／USB 的相機延遲。
        delay = os.getenv('CAMERA_INPUT_DELAY_MS')
        uncertainty = os.getenv('CAMERA_INPUT_UNCERTAINTY_MS')
        self.input_delay = self.input_uncertainty = None
        if delay is not None or uncertainty is not None:
            if delay is None or uncertainty is None:
                raise ValueError('相機前段校正須同時設定延遲及不確定範圍')
            self.input_delay, self.input_uncertainty = float(delay), float(uncertainty)
            if any(not math.isfinite(v) or v < 0 for v in (self.input_delay, self.input_uncertainty)):
                raise ValueError('相機前段校正值須為非負有限數值')

    def request(self, ser):
        now = time.monotonic_ns()
        if now-self.last_request < 2_000_000_000:
            return
        self.pending = {key: value for key, value in self.pending.items() if now-key < 10_000_000_000}
        header = struct.pack(HEADER_FMT, FRAME_MAGIC, self.protocol_version, 1, 8, 8^0xffff)
        data = header+struct.pack('>Q', now)
        frame = data+struct.pack('>H', crc16_ccitt_false(data[2:]))
        if ser.write(frame) != len(frame):
            raise OSError('校時請求未完整寫入')
        self.pending[now] = now
        self.last_request = now

    def accept(self, frame, received):
        t1, t2, t3, session = struct.unpack('>QQQI', frame[8:-2])
        if t1 not in self.pending:
            return
        del self.pending[t1]
        rtt = (received-t1)-(t3-t2)
        if rtt < 0 or t3 < t2 or received-t1 > 5_000_000_000:
            return
        offset = ((t2-t1)+(t3-received))/2
        self.samples.append((received, session, offset, rtt/2))

    def estimate(self, frame, received, result_id):
        if frame[2] not in (0, 1, 2, 3, 4, 5, 10, 7, 8):
            return '端到端：N/A（v6 沒有取像時間戳）'
        capture, session = struct.unpack('>QI', frame[8+BASE_SIZE:8+BASE_SIZE+12])
        candidates = [s for s in self.samples if s[1] == session and 0 <= received-s[0] <= 10_000_000_000]
        if not capture:
            return '端到端：N/A（發送端影像時間戳為零，請確認辨識端主程式版本）'
        if not candidates:
            return '端到端：N/A（影像時間戳已收到，但 UART 校時尚未成功）'
        sample_time, _, offset, uncertainty = min(candidates, key=lambda s: s[3])
        # 加入兩端合計 200 ppm 的漂移假設；此範圍不是硬體校正保證。
        uncertainty += (received-sample_time)*0.0002
        value = (received+offset-capture)/1e6
        error = uncertainty/1e6
        if value < -error:
            return '端到端：N/A（時差不一致，等待重新校時）'
        key = (session, result_id, capture)
        repeated = key in self.seen
        if not repeated:
            self.seen.append(key)
        label = '同一結果重送' if repeated else '新結果首次接收'
        text = (f'取像完成至 UART 驗證：估計 {value:.2f} ms，校時 ±{error:.2f} ms '
                f'（{label}；不含相機至取像前的延遲）')
        if self.input_delay is None:
            return text
        return (text+f'\n含前段校正的總延遲估計：{value+self.input_delay:.2f} ms '
                f'±{error+self.input_uncertainty:.2f} ms（固定校正值，不是逐幀全鏈路實測）')


def convert_value(index, value):
    if index == 12:
        return f"{value}（{MODE_NAMES[value]}）"
    if index == 13:
        return str(value)
    if not math.isfinite(value):
        return "N/A"
    if index in (10, 11, 17):
        return f"{value:.3f}°"
    unit = BASE_FIELDS[index][1]
    if index == 16:
        return f'{value:.1f} {unit}'
    if index in (8, 9):
        return f"{value:g} {unit}"
    return f"{value:.3f} {unit}"


def resolve_uart_port(configured=UART_PORT):
    """僅使用指定串口，不自動搜尋或試寫其他設備。"""
    port = configured.strip()
    if not port or port.lower() == 'auto':
        raise ValueError('請以 --port 指定接收模組，例如 /dev/ttyUSB0 或 COM3')
    return port


class UARTReceiver:
    def __init__(self, port=UART_PORT):
        self.port = resolve_uart_port(port)
        self.ser = serial.Serial(
            self.port, UART_BAUD,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, timeout=0.01, write_timeout=WRITE_TIMEOUT,
            xonxoff=False, rtscts=False, dsrdtr=False,
        )
        self.last_display = 0.0
        self.last_valid_frame = time.monotonic()
        self.last_diagnostic = self.last_valid_frame
        self.rx_bytes = 0
        self.last_valid_rx_bytes = 0
        self.diagnostic_rx_bytes = 0
        self.raw_sample = bytearray()
        self.rx_errors = {"header": 0, "crc": 0, "payload": 0, "v4": 0, "discarded": 0}
        self.buffer = bytearray()
        self.processing_times = deque(maxlen=100)
        self.clock = UARTClock()
        self.received_ns = 0
        self.clock_enabled = False
        self.requested_mode = None
        self.mode_requested_at = 0
        self.startup_mode = None

    def _extract_frame(self):
        """解析緩衝資料，保留分段標頭並在資料損壞後重新同步。"""
        while True:
            idx = self.buffer.find(FRAME_MAGIC)
            if idx < 0:
                # A5 可能是跨次讀取標頭的第一個位元組。
                keep = 1 if self.buffer.endswith(FRAME_MAGIC[:1]) else 0
                self.rx_errors["discarded"] += len(self.buffer) - keep
                if self.buffer.endswith(FRAME_MAGIC[:1]):
                    self.buffer[:] = FRAME_MAGIC[:1]
                else:
                    self.buffer.clear()
                return None
            if idx:
                self.rx_errors["discarded"] += idx
                del self.buffer[:idx]
            if len(self.buffer) < HEADER_SIZE:
                return None
            _, version, flags, length, length_xor = struct.unpack(
                HEADER_FMT, self.buffer[:HEADER_SIZE]
            )
            # 舊版 v4 只有 12 值；驗證 CRC 後明確提示，不能假造第 13 個模式值。
            if version == 4 and flags == 0 and length == 48 and length_xor == (48 ^ 0xFFFF):
                old_size = HEADER_SIZE + 48 + CRC_SIZE
                if len(self.buffer) < old_size:
                    return None
                old_frame = bytes(self.buffer[:old_size])
                if struct.unpack(">H", old_frame[-2:])[0] == crc16_ccitt_false(old_frame[2:-2]):
                    self.rx_errors["v4"] += 1
                    del self.buffer[:old_size]
                    continue
                self.rx_errors["crc"] += 1
                del self.buffer[0]
                continue
            expected = {(6, 0): BASE_SIZE, (7, 0): BASE_SIZE+12, (8, 0): BASE_SIZE+16, (0, 0): BASE_SIZE+20, (1, 0): BASE_SIZE+20, (2, 0): BASE_SIZE+20, (3, 0): BASE_SIZE+20, (4, 0): BASE_SIZE+20, (5, 0): BASE_SIZE+20, (10, 0): BASE_SIZE+20, (0, 2): 28, (1, 2): 28, (2, 2): 28, (3, 2): 28, (4, 2): 28, (5, 2): 28, (10, 2): 28, (7, 2): 28}.get((version, flags))
            if (expected is None or length != expected
                    or length_xor != (length ^ 0xFFFF)):
                self.rx_errors["header"] += 1
                del self.buffer[0]
                continue
            frame_size = HEADER_SIZE+length+CRC_SIZE
            if len(self.buffer) < frame_size:
                return None
            frame = bytes(self.buffer[:frame_size])
            try:
                decode_frame(frame)
            except ValueError as error:
                self.rx_errors["crc" if str(error) == "CRC mismatch" else "payload"] += 1
                # 向後移動一個位元組重新搜尋，保留可能有效的封包。
                del self.buffer[0]
                continue
            self.received_ns = time.monotonic_ns()
            # 只有看見 10X_V0／V1 或舊格式 v7／v8 資料才送校時，避免舊發送器誤認二進位為模式鍵。
            self.clock_enabled = version in (0, 1, 2, 3, 4, 5, 10, 7, 8)
            self.clock.protocol_version = version if version in (0, 1, 2, 3, 4, 5, 10) else 7
            del self.buffer[:frame_size]
            return frame

    def read_frame(self):
        while True:
            if self.clock_enabled:
                self.clock.request(self.ser)
            self.poll_keyboard()
            if self.requested_mode is not None and time.monotonic()-self.mode_requested_at > 3:
                print('[模式未確認] 3 秒內未收到指定模式；請看發送端是否收到 UART 模式要求，及是否回報角度未就緒。', flush=True)
                self.requested_mode = None
            frame = self._extract_frame()
            if frame is not None:
                self.last_valid_frame = time.monotonic()
                self.last_valid_rx_bytes = self.rx_bytes
                return frame
            # 立即讀取可用資料，避免等待填滿 256 bytes。
            count = min(4096, max(1, self.ser.in_waiting))
            data = self.ser.read(count)
            self.rx_bytes += len(data)
            self.buffer.extend(data)
            self.raw_sample.extend(data)
            del self.raw_sample[:-64]
            now = time.monotonic()
            if (now - self.last_valid_frame >= RX_DIAGNOSTIC_SEC
                    and now - self.last_diagnostic >= RX_DIAGNOSTIC_SEC):
                recent_bytes = self.rx_bytes - self.diagnostic_rx_bytes
                rate = recent_bytes / max(0.001, now - self.last_diagnostic)
                self.last_diagnostic = now
                self.diagnostic_rx_bytes = self.rx_bytes
                print("雷射距離：N/A")
                if recent_bytes == 0:
                    print(f"[等待資料] {self.port} 未收到新資料；請確認串口、TX/RX 交叉接線與發送端。")
                else:
                    print(f"[RX 診斷] {self.port} @ {UART_BAUD}，累計 {self.rx_bytes} bytes，近期 {rate:.0f} B/s")
                    print(f"[錯誤累計] 舊版v4={self.rx_errors['v4']}，封包頭={self.rx_errors['header']}，CRC={self.rx_errors['crc']}，資料={self.rx_errors['payload']}，未同步丟棄bytes={self.rx_errors['discarded']}")
                    print(f"[RAW HEX 最近64bytes] {self.raw_sample.hex(' ')}")
                    if self.rx_errors['v4']:
                        print("[版本不符] 已驗證收到舊版 v4；請在發送端更新 uart_sender.py 與 main_uart.py 並重新啟動。")
                    else:
                        print("[提示] v6 正常封包頭為 a5 5a 06 00 00 3d ff c2；請比對 RAW，確認兩端鮑率與發送程式。")

    def poll_keyboard(self):
        """即使沒有遙測資料，仍能用 0/1/2 切模式；不傳其他控制鍵。"""
        if not sys.stdin.isatty():
            return
        if os.name == "nt":
            import msvcrt
            key = msvcrt.getwch() if msvcrt.kbhit() else ""
        else:
            import select
            key = os.read(sys.stdin.fileno(), 1).decode("ascii", errors="ignore") if select.select([sys.stdin.fileno()], [], [], 0)[0] else ""
        if key in ("0", "1", "2"):
            self.send_mode(int(key))
        elif key in ("q", "Q", "\x1b"):
            raise KeyboardInterrupt

    def send_mode(self, mode):
        """傳送原協定 ASCII 模式鍵，等待遙測確認實際狀態。"""
        if mode not in (0, 1, 2):
            raise ValueError('模式須為 0、1 或 2')
        if self.ser.write(str(mode).encode('ascii')) != 1:
            raise OSError('模式命令寫入不完整')
        self.requested_mode = mode
        self.mode_requested_at = time.monotonic()
        print(f'要求切換：{MODE_NAMES[mode]}；等待發送端實際模式回報', flush=True)

    def process(self, frame):
        start = time.perf_counter_ns()
        try:
            values, crc_recv = decode_frame(frame)
        except ValueError as error:
            print(f"[RX rejected] {error}")
            return
        if values is None:
            self.clock.accept(frame, self.received_ns)
            return
        if self.requested_mode is not None and values[12] == self.requested_mode:
            print(f'[模式已確認] {MODE_NAMES[values[12]]}（軟體模式；不代表機械定位已完成）', flush=True)
            self.requested_mode = None
        if self.startup_mode is not None:
            mode, self.startup_mode = self.startup_mode, None
            self.send_mode(mode)
        latency_text = self.clock.estimate(frame, self.received_ns, values[13])
        processing_ms = (time.perf_counter_ns() - start) / 1e6
        self.processing_times.append(processing_ms)
        wire_ms = len(frame) * 10 / UART_BAUD * 1000.0
        decoded = dict(zip((name for name, _ in BASE_FIELDS), values))
        now = time.monotonic()
        if now - self.last_display < DISPLAY_INTERVAL:
            return decoded
        self.last_display = now
        print("=" * 78)
        print(f"UART v{frame[2]} | {len(values)} values | payload={len(frame)-10} B | frame={len(frame)} B | CRC=OK 0x{crc_recv:04X}")
        print(f"UART wire time (8N1 estimate): {wire_ms:.3f} ms")
        print(f"RX validation/decode: {processing_ms:.3f} ms | avg={sum(self.processing_times) / len(self.processing_times):.3f} ms")
        print(latency_text)
        print(f"{'Idx':<5}{'Field':<20}{'Value':<24}")
        print("-" * 60)
        # 顯示順序獨立於封包索引，保留各型號原有資料與模式對應。
        display_order = (*range(10), 10, 17, 11, *range(12, 17))
        for display_index, index in enumerate(display_order):
            text = convert_value(index, values[index])
            print(f"{display_index:<5}{FIELD_LABELS[index]:<20}{text:<24}")
        print("偏移量：向右／向下為正，向左／向上為負；N/A 表示無目標或角度無效。")
        return dict(zip((name for name, _ in BASE_FIELDS), values))


@contextmanager
def keyboard_input():
    """Linux 終端單鍵讀取、不需 Enter；離開時還原終端設定。"""
    if os.name == "nt" or not sys.stdin.isatty():
        yield
        return
    import termios
    import tty
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def main():
    global UART_BAUD
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default=UART_PORT, help='接收模組串口，例如 /dev/ttyUSB0 或 COM3')
    parser.add_argument('--baud', type=int, default=UART_BAUD, help='須與發送端一致，預設 115200')
    parser.add_argument('--mode', type=int, choices=(0, 1, 2), help='收到首筆遙測後要求切換一次模式，不需互動鍵盤')
    parser.add_argument('--list', action='store_true', help='只列出串口，不開啟或傳送資料')
    args = parser.parse_args()
    if args.list:
        from serial.tools import list_ports
        for port in list_ports.comports():
            print(f'{port.device} | {port.description} | {port.hwid}')
        return
    if args.baud <= 0:
        parser.error('鮑率須大於零')
    UART_BAUD = args.baud
    try:
        rx = UARTReceiver(args.port)
        rx.startup_mode = args.mode
    except (RuntimeError, OSError, ValueError) as error:
        print(f'接收端啟動失敗：{error}')
        raise SystemExit(1)
    print(f"共用雲台接收端 V6（台鴞／田屋 30 倍／田屋 10 倍）： {rx.port} @ {UART_BAUD}, 8N1；v7 自動 UART 校時")
    try:
        print("0：手動；1：追蹤；2：鎖定；Q / Esc：結束。請聚焦此終端。")
        with keyboard_input():
            while True:
                rx.process(rx.read_frame())
    except KeyboardInterrupt:
        print("Receiver stopped")
    finally:
        rx.ser.close()


if __name__ == "__main__":
    main()
