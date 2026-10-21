# -*- coding: utf-8 -*-
"""
video_writer.py — 斷電保護的 YOLO 標註影片寫入器 (vision_dive)
═══════════════════════════════════════════════════════════════════
兩層後端, 自動挑可用的:
  1. NVENC + fragmented MP4 (Jetson 首選): gst-launch 硬體編碼,
     mp4mux streamable=true fragment-duration=1000 → moov 先寫 + 每秒封片,
     斷電最多丟最後幾秒, 之前照樣可播.
  2. cv2.VideoWriter MP4V (通用後援): 無 GStreamer/NVENC 時用. 標準 MP4 的
     索引在 release 才寫, 斷電會報廢整檔 → 每隔幾秒切一段新檔 (segment).

共通:
  - 背景 thread 從 queue 取幀寫, 不擋主視覺迴路
  - .ok 屬性讓主程式判斷是否成功啟動

═══════════════════════════════════════════════════════════════════
★ 2026/09/09 修正 (實飛 flight_20260908_101654 診斷)
═══════════════════════════════════════════════════════════════════
症狀: 12:34 的飛行, 標註影片播起來只有 11:33, 看起來像「錄一半就停」.

真因: 影片其實是完整的, 是【檔頭 fps 標錯】造成的時間壓縮.
  - 視覺迴圈實測只跑 24.2 fps (YOLO 卡住), 不是 config 的 30
  - 25Hz 節流又丟掉約 5% 的幀 (迴圈本來就低於 25, 抖動害的)
  - 實際寫入 17,325 幀, 檔頭卻寫 25 fps → 17325/25 = 693s = 11:33
  - 18,255 幀 / 754.4s 真實時長 → 真實寫入率 23.0 fps

修正三件事:
  1. 全程記錄真實幀數與真實時長, 收尾算出【實際 fps】並印出來
  2. 收尾若有 ffmpeg, 自動用實際 fps 重新封裝 (無損, 只改時基),
     播放時間就會跟 log 的 t 對得上. 沒 ffmpeg 就印出手動指令.
  3. 節流預設關閉 — 迴圈速率本來就低於目標 fps, 節流只是白丟幀

另外補三個「靜默失敗」漏洞 (都是飛完才會發現的那種):
  a. gst-launch 的 stderr 是 PIPE 但從來沒人讀 → 管線緩衝 (~64KB) 塞滿
     會把整條編碼管線卡死. 改成背景 thread 持續吸掉並留最後幾行.
  b. 寫入 thread 遇到例外只 break, 不印任何東西, .ok 還是 True →
     影片停了主程式完全不知道. 改成記錄錯誤 + 標記 ok=False + health().
  c. 沒有磁碟空間檢查. 8Mbps × 兩支影片 ≈ 每分鐘 120MB.
     開檔前檢查, 執行中低於門檻就停止寫入 (保住已寫的部分).

═══════════════════════════════════════════════════════════════════
★ 2026/09/15 真實時間戳 (VFR) — 根治「影片被加速」
═══════════════════════════════════════════════════════════════════
上面 09/09 的修法 (收尾用平均 fps remux) 只修得到「總長」:
  · 局部卡頓 / 相機重開的空窗還是被壓掉
  · 要程式正常結束 + 機上有 ffmpeg/gst 才修得到; 斷電/被 kill 就維持加速

新後端 'vfr' (預設優先):
  每幀帶時間戳, 交給子行程 (就是本檔: python3 video_writer.py --vfr-helper)
  的 appsrc 當 PTS → NVENC → fragmented MP4.
  子行程隔離跟舊版 gst-launch 一樣: 編碼器當掉不會拖垮飛控.
  播放時間 = 真實時間, 邊錄邊生效, 不需要收尾 remux.
  時間戳 = write() 被呼叫的當下 (monotonic, 不受 GPS/NTP 校時跳動影響).
  raw 錄影在取像執行緒 read() 之後立刻呼叫 → 就是擷取時刻;
  呼叫端也可以用 write(..., t_cap=...) 明確給.

上機前自測 (不需相機, 測 NVENC 管線 + 中途卡頓):
  python3 video_writer.py --selftest

後端順序: vfr → nvenc (舊 CFR) → cv2 (CFR 分段). 後兩者仍保留收尾 remux.

另外所有後端都寫 <name>.ts.csv 時間戳旁檔:
  mp4_idx, frame_id, pts_s, t_cap_unix
  (pts_s: vfr = 檔案裡的真實 PTS; CFR 後端 = remux 前的 idx/fps)
  · mp4_idx ↔ frame_id 逐幀對照 → 有掉幀也能精確對齊 (不再依賴 dropped=0)
  · t_cap_unix 可直接對 log (time.time() 基準)
"""

import os
import select
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
from collections import deque

_SELF = os.path.abspath(__file__)


class VideoWriter:
    MIN_FREE_MB_START = 2000.0    # 開檔前至少要有這麼多空間
    MIN_FREE_MB_STOP = 300.0      # 執行中低於此值就停寫 (保住已寫的部分)

    def __init__(self, path, fps, width, height, bitrate_bps=8_000_000,
                 seg_seconds=30.0, throttle=False, remux_on_release=True,
                 queue_len=8, drop_log=True, vfr=True, vfr_encoder='nvenc',
                 ts_sidecar=True):
        self.path = path
        # 標稱 fps: vfr 後端只當編碼器 rate-control 參考 (時間軸看 PTS);
        #          nvenc/cv2 (CFR) 後端則是檔頭時基.
        self.fps = int(fps)
        self.w = int(width)
        self.h = int(height)
        self.bitrate = int(bitrate_bps)
        self.seg_seconds = float(seg_seconds)
        # ★ 預設不節流: 視覺迴圈實測 24 fps < VIDEO_FPS 25, 節流只會白丟幀
        self.throttle = bool(throttle)
        self.remux_on_release = bool(remux_on_release)
        self.ok = False
        self.backend = None
        self.error = None                    # ★ 失敗原因 (None = 正常)

        # ★ queue_len: raw 影片建議 120 (≈5s 緩衝). 舊值 8 只有 0.3s,
        #   編碼器或磁碟稍微一頓就會掉幀, 播放時看到畫面跳掉.
        self._q = deque(maxlen=int(queue_len))   # 滿了掉幀優於卡住主迴圈
        self.drop_log = bool(drop_log)
        self.drop_events = []                    # (t, queue_len, written)
        self._qlock = threading.Lock()
        self._run = True
        self._last_write_t = 0.0
        self._min_dt = 1.0 / max(self.fps, 1)
        self.written = 0
        self.dropped = 0
        self.enqueued = 0        # write() 被呼叫且成功入列的次數
        # ★ 2026/09/12: raw 錄影是在 FrameGrabber 啟動之後才掛上的, 所以
        #   MP4 第 0 幀 ≠ 相機第 0 幀 (實測差 200 幀左右).
        #   記下第一幀的相機幀號, 後處理才能正確換算:
        #     MP4 幀號 N  →  raw_frame_idx = N + first_frame_id  (前提 dropped=0)
        self.first_frame_id = None
        self.stop_requested_backlog = 0   # release() 時 queue 裡還有幾幀
        self.died_at_t = None    # 寫入停止的時刻 (相對第一幀)

        # ★ 真實時基 (算實際 fps 用)
        self.t_first = None                  # 第一幀進 queue 的時刻
        self.t_last = None                   # 最後一幀寫出的時刻
        self.last_written_t = 0.0            # 健康檢查用

        self._proc = None
        self._cv = None
        self._cv2 = None
        self._seg_idx = 0
        self._seg_t0 = 0.0
        self._stderr_tail = deque(maxlen=20)
        self._err_thread = None

        # ★ VFR / 時間戳
        self.vfr_encoder = vfr_encoder
        self._frame_nbytes = self.w * self.h * 3
        self._pts_base = None                # 第一個寫出幀的 t_cap (monotonic)
        self._last_pts_ns = -1
        self.cap_first = None                # 第一/最後寫出幀的擷取時刻 (mono)
        self.cap_last = None
        self.size_mismatch = 0               # 尺寸不符被跳過的幀
        self._ts_fp = None
        self._ts_last_flush = 0.0
        self.ts_path = (os.path.splitext(self.path)[0] + '.ts.csv'
                        if ts_sidecar else None)

        # ── 磁碟空間檢查 ──
        free_mb = self._free_mb()
        if free_mb is not None and free_mb < self.MIN_FREE_MB_START:
            self.error = (f"磁碟只剩 {free_mb:.0f}MB "
                          f"(需要 {self.MIN_FREE_MB_START:.0f}MB)")
            print(f"[Video] ★★★ 不啟動錄影: {self.error}")
            return

        # ── 後端: vfr (真實時間戳) → nvenc (CFR) → cv2 (CFR 分段) ──
        if vfr:
            if self._try_vfr():
                self.backend = 'vfr'
                self.ok = True
                self.error = None
            else:
                print(f"[Video] ★ VFR 後端不可用 ({self.error}) → 退回 CFR "
                      f"(依標稱 fps 排時間, 收尾再 remux)")
                self.error = None
        if not self.ok:
            if self._try_nvenc():
                self.backend = 'nvenc'
                self.ok = True
                self.error = None
            else:
                _nvenc_err = self.error
                self.error = None
                if self._try_cv2():
                    self.backend = 'cv2'
                    self.ok = True
                    self.error = None
                    print(f"[Video] (NVENC 不可用: {_nvenc_err} → 改用 cv2)")
                else:
                    self.error = (f"NVENC: {_nvenc_err} | cv2: {self.error}")

        if self.ok:
            self._open_ts_sidecar()
            self._t = threading.Thread(target=self._loop, daemon=True)
            self._t.start()
            print(f"[Video] 後端={self.backend}"
                  f"{' (真實時間戳)' if self.backend == 'vfr' else ''} "
                  f"標稱fps={self.fps} "
                  f"節流={'ON' if self.throttle else 'OFF'} "
                  f"剩餘空間={free_mb:.0f}MB → {self.path}")

    # ------------------------------------------------------------------
    def _free_mb(self):
        try:
            d = os.path.dirname(os.path.abspath(self.path)) or '.'
            os.makedirs(d, exist_ok=True)
            return shutil.disk_usage(d).free / 1e6
        except Exception:
            return None

    # ---- ★ VFR: 真實時間戳子行程 ----
    def _try_vfr(self):
        # Jetson 上 ROS 的 python3 通常就有 gi; 其他環境退 /usr/bin/python3
        pys = []
        for p in (sys.executable, '/usr/bin/python3'):
            if p and p not in pys and os.path.exists(p):
                pys.append(p)
        errs = []
        for py in pys:
            cmd = [py, _SELF, '--vfr-helper', self.path, str(self.w),
                   str(self.h), str(self.bitrate), str(self.fps),
                   self.vfr_encoder]
            try:
                # start_new_session: Ctrl+C 不會先打死編碼器 (同舊版理由)
                proc = subprocess.Popen(
                    cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE, bufsize=0,
                    start_new_session=True)
            except Exception as e:
                errs.append(f"{py}: 啟動失敗 {e}")
                continue
            self._proc = proc
            self._stderr_tail.clear()
            self._err_thread = threading.Thread(target=self._drain_stderr,
                                                daemon=True)
            self._err_thread.start()
            # 握手: 等子行程印 READY (gi import + NVENC 初始化可能要幾秒)
            # ★ 2026/09/15: 舊版讀到【第一行】不管內容就 break → 子行程若先
            #   印了別的 (函式庫訊息等), 會被誤判失敗而退回 CFR.
            #   改成: 非 READY 的行記下來繼續等, 直到 READY / 子行程結束 / 逾時.
            ready = False
            t_end = time.time() + 8.0
            while time.time() < t_end:
                if proc.poll() is not None:
                    break
                r, _, _ = select.select([proc.stdout], [], [], 0.2)
                if not r:
                    continue
                line = proc.stdout.readline()
                if not line:                 # EOF: 子行程關了 stdout
                    break
                if line.strip() == b'READY':
                    ready = True
                    break
                self._stderr_tail.append(
                    '[stdout] ' + line.decode('utf-8', 'replace').strip())
            if ready:
                return True
            try:
                proc.kill()
                proc.wait(timeout=2.0)
            except Exception:
                pass
            time.sleep(0.05)
            errs.append(f"{py}: " + (" / ".join(self._stderr_tail)
                                     or "未回 READY"))
            self._proc = None
        self.error = " | ".join(errs) or "沒有可用的 python"
        return False

    def _open_ts_sidecar(self):
        if not self.ts_path:
            return
        try:
            self._ts_fp = open(self.ts_path, 'w', buffering=1 << 16)
            self._ts_fp.write('mp4_idx,frame_id,pts_s,t_cap_unix\n')
        except Exception as e:
            print(f"[Video] ★ 時間戳旁檔開不起來: {e}")
            self._ts_fp = None

    def _ts_write(self, idx, frame_id, pts_s, t_unix, now):
        fp = self._ts_fp
        if fp is None:
            return
        try:
            fp.write(f"{idx},{'' if frame_id is None else frame_id},"
                     f"{pts_s:.6f},{t_unix:.6f}\n")
            if now - self._ts_last_flush >= 1.0:     # 每秒落盤一次
                self._ts_last_flush = now
                fp.flush()
                os.fsync(fp.fileno())
        except Exception:
            self._ts_fp = None

    # ---- NVENC fragmented MP4 (舊 CFR) ----
    def _try_nvenc(self):
        self._stderr_tail.clear()
        cmd = [
            'gst-launch-1.0', '-e', '-q',
            'fdsrc', 'fd=0', '!',
            'rawvideoparse', 'format=bgr',
            f'width={self.w}', f'height={self.h}', f'framerate={self.fps}/1', '!',
            'videoconvert', '!', 'nvvidconv', '!',
            'video/x-raw(memory:NVMM),format=NV12', '!',
            'nvv4l2h264enc', f'bitrate={self.bitrate}', '!',
            'h264parse', '!',
            'mp4mux', 'streamable=true', 'fragment-duration=1000', '!',
            'filesink', f'location={self.path}',
        ]
        try:
            # (2026/09/11 實測) start_new_session=True 讓 gst-launch 脫離本
            # 行程的 process group. 否則 Ctrl+C 的 SIGINT 會【先】打死 gst,
            # release() 還沒來得及關 stdin 管線就斷了 -> BrokenPipe, 最後幾
            # 秒丟失, fragmented MP4 也沒正常收尾.
            self._proc = subprocess.Popen(
                cmd, stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, bufsize=0,
                start_new_session=True)
        except Exception as e:
            self._proc = None
            self.error = f"gst-launch 啟動失敗: {e}"
            return False
        # ★ 一定要有人讀 stderr, 否則管線緩衝塞滿會把 gst 整個卡死
        self._err_thread = threading.Thread(target=self._drain_stderr,
                                            daemon=True)
        self._err_thread.start()
        time.sleep(0.15)
        if self._proc.poll() is not None:
            self.error = ("gst-launch 立刻結束 ("
                          + " / ".join(self._stderr_tail) + ")")
            self._proc = None
            return False
        return True

    def _drain_stderr(self):
        """持續吸掉 gst 的 stderr, 只留最後幾行給診斷用."""
        try:
            for line in iter(self._proc.stderr.readline, b''):
                s = line.decode('utf-8', 'replace').strip()
                if s:
                    self._stderr_tail.append(s)
        except Exception:
            pass

    # ---- cv2 segmented MP4V ----
    def _try_cv2(self):
        try:
            import cv2
            self._cv2 = cv2
        except Exception as e:
            self.error = f"cv2 不可用: {e}"
            return False
        return self._open_cv_segment()

    def _open_cv_segment(self):
        cv2 = self._cv2
        base, ext = os.path.splitext(self.path)
        seg_path = (self.path if self._seg_idx == 0
                    else f"{base}_seg{self._seg_idx:02d}{ext}")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self._cv = cv2.VideoWriter(seg_path, fourcc, self.fps, (self.w, self.h))
        if not self._cv.isOpened():
            self._cv = None
            self.error = f"cv2.VideoWriter 開不起來: {seg_path}"
            return False
        self._seg_t0 = time.time()
        return True

    # ---- 對外: 入 queue ----
    def write(self, frame, frame_id=None, t_cap=None):
        """frame_id: 來源的累計幀號 (FrameGrabber.frame_id).
        t_cap: ★ 這幀【相機擷取當下】的 time.monotonic().
               vfr 後端用它當 PTS. 沒給就用呼叫當下 (會混入排隊延遲)."""
        if not self.ok or frame is None:
            return
        _now_m = time.monotonic()
        _now_u = time.time()
        if t_cap is None:
            t_cap = _now_m
        t_unix = _now_u - (_now_m - t_cap)
        if frame_id is not None and self.first_frame_id is None:
            self.first_frame_id = int(frame_id)
        now = time.time()
        if self.throttle and (now - self._last_write_t < self._min_dt):
            return
        self._last_write_t = now
        if self.t_first is None:
            self.t_first = now
        with self._qlock:
            if len(self._q) >= self._q.maxlen:
                self.dropped += 1
                # ★ 不再 silent drop: 記下時間點, 收尾才知道哪幾秒跳掉
                if self.drop_log and len(self.drop_events) < 5000:
                    self.drop_events.append((now, len(self._q), self.written))
            self._q.append((frame, t_cap, t_unix, frame_id))
            self.enqueued += 1

    # ---- 健康檢查 (主迴圈可定期呼叫) ----
    def health(self):
        """回傳 (ok, 訊息). ok=False 代表寫入已經停了, 訊息說明原因."""
        if not self.ok:
            return False, (self.error or "未啟動")
        if self.last_written_t:
            idle = time.time() - self.last_written_t
            if idle > 5.0:
                return False, f"已經 {idle:.0f}s 沒有寫出任何一幀"
        return True, "OK"

    def _write_all(self, data):
        """bufsize=0 的 pipe 寫入可能只寫一部分 (被 signal 打斷時);
        vfr 協定錯位一個 byte 整條串流就廢了, 所以一定要寫滿."""
        mv = memoryview(data)
        while mv:
            n = self._proc.stdin.write(mv)
            if n is None:
                raise BrokenPipeError("stdin write 回傳 None")
            mv = mv[n:]

    # ---- 背景寫 ----
    def _loop(self):
        last_disk_check = 0.0
        while True:
            frame = None
            with self._qlock:
                if self._q:
                    frame, t_cap, t_unix, fid = self._q.popleft()
                _empty = not self._q
            if frame is None:
                # ★ 2026/09/12: 收尾時把 queue 清完再退出.
                #   舊版是 `while self._run:` —— release() 把 _run 設 False,
                #   迴圈立刻結束, queue 裡還沒寫出的幀直接丟掉.
                #   raw queue 有 120 幀, 最壞丟掉影片尾端 4 秒.
                #   (舊版的「收尾時仍在佇列 N」只是報告, 沒有真的處理)
                if not self._run:
                    break
                time.sleep(0.002)
                continue

            # 每 10 秒看一次磁碟
            now = time.time()
            if now - last_disk_check > 10.0:
                last_disk_check = now
                free = self._free_mb()
                if free is not None and free < self.MIN_FREE_MB_STOP:
                    self.error = (f"磁碟剩餘 {free:.0f}MB, "
                                  f"停止寫入以保住已錄部分")
                    print(f"[Video] ★★★ {self.error}")
                    self.ok = False
                    self._run = False
                    break

            try:
                if self.backend == 'vfr':
                    if frame.nbytes != self._frame_nbytes:
                        # 相機重開後解析度變了之類; 硬塞會讓整條串流錯位
                        self.size_mismatch += 1
                        if self.size_mismatch == 1:
                            print(f"[Video] ★ 幀尺寸 {frame.shape} ≠ "
                                  f"{self.w}x{self.h}, 跳過")
                        continue
                    if self._pts_base is None:
                        self._pts_base = t_cap
                    pts_ns = int((t_cap - self._pts_base) * 1e9)
                    if pts_ns <= self._last_pts_ns:
                        pts_ns = self._last_pts_ns + 1_000_000
                    self._last_pts_ns = pts_ns
                    self._write_all(struct.pack('<Q', pts_ns))
                    self._write_all(frame.tobytes())
                    pts_s = pts_ns / 1e9
                elif self.backend == 'nvenc':
                    self._proc.stdin.write(frame.tobytes())
                    pts_s = self.written / max(self.fps, 1)
                else:
                    if time.time() - self._seg_t0 >= self.seg_seconds:
                        try:
                            self._cv.release()
                        except Exception:
                            pass
                        self._seg_idx += 1
                        self._open_cv_segment()
                    if self._cv is not None:
                        self._cv.write(frame)
                    pts_s = self.written / max(self.fps, 1)
                self._ts_write(self.written, fid, pts_s, t_unix, now)
                if self.cap_first is None:
                    self.cap_first = t_cap
                self.cap_last = t_cap
                self.written += 1
                self.t_last = time.time()
                self.last_written_t = self.t_last
            except Exception as e:
                # ★ 舊版這裡只 break, 什麼都不印, .ok 還是 True →
                #   影片停了主程式完全不知道. 現在一定要留下證據.
                tail = " / ".join(self._stderr_tail) if self._stderr_tail else ""
                self.error = (f"寫入失敗於第 {self.written} 幀: "
                              f"{type(e).__name__}: {e}"
                              + (f" | gst: {tail}" if tail else ""))
                print(f"[Video] ★★★ {self.error}")
                self.died_at_t = (now - self.t_first) if self.t_first else None
                self.ok = False
                self._run = False
                break

    # ---- 收尾 ----
    def release(self):
        # ★ 先讓寫入執行緒把 queue 清完 (最多等 drain_timeout), 再關 encoder.
        #   join 的 timeout 要夠久: 120 幀 @ 30fps 約 4s, 給 8s 餘裕.
        self.stop_requested_backlog = 0
        with self._qlock:
            self.stop_requested_backlog = len(self._q)
        if self.stop_requested_backlog:
            print(f"[Video] 收尾: 佇列還有 {self.stop_requested_backlog} 幀, "
                  f"等它寫完 ...")
        self._run = False
        try:
            if hasattr(self, '_t'):
                self._t.join(timeout=8.0)
                if self._t.is_alive():
                    print("[Video] ★ 佇列 8s 內沒清完, 放棄剩餘的幀")
        except Exception:
            pass
        if self.backend in ('vfr', 'nvenc') and self._proc is not None:
            try:
                self._proc.stdin.close()
                self._proc.wait(timeout=10.0)
            except Exception:
                try:
                    self._proc.terminate()
                except Exception:
                    pass
        elif self._cv is not None:
            try:
                self._cv.release()
            except Exception:
                pass
        if self._ts_fp is not None:
            try:
                self._ts_fp.flush()
                os.fsync(self._ts_fp.fileno())
                self._ts_fp.close()
            except Exception:
                pass
            self._ts_fp = None

        # ── 真實 fps 統計 ──
        # ★ 用「擷取時間」算 (舊版用寫出時間, 會混入編碼器延遲)
        real_fps = None
        span = None
        if (self.cap_first is not None and self.cap_last is not None
                and self.cap_last > self.cap_first and self.written > 1):
            span = self.cap_last - self.cap_first
            real_fps = (self.written - 1) / span
        elif self.t_first and self.t_last and self.t_last > self.t_first:
            span = self.t_last - self.t_first
            real_fps = self.written / span
        backlog = self.enqueued - self.written - self.dropped
        msg = (f"[Video] 收尾: 入列 {self.enqueued}, "
               f"寫入 {self.written}, 丟棄 {self.dropped}, "
               f"收尾時仍在佇列 {backlog}")
        if self.died_at_t is not None:
            msg += (f"\n[Video]   ★ 寫入在第 "
                    f"{self.died_at_t:.1f}s 停止")
        if self.dropped and self.drop_events:
            _t0 = self.t_first or self.drop_events[0][0]
            _win = {}
            for _te, _ql, _w in self.drop_events:
                _win[int(_te - _t0)] = _win.get(int(_te - _t0), 0) + 1
            _top = sorted(_win.items(), key=lambda kv: -kv[1])[:5]
            msg += ("\n[Video]   掉幀集中在 t=" +
                    ", ".join(f"{k}s({v}幀)" for k, v in _top))
        if self.size_mismatch:
            msg += f"\n[Video]   ★ 尺寸不符跳過 {self.size_mismatch} 幀"
        if real_fps and self.backend == 'vfr':
            msg += (f"\n[Video]   真實時長 {span:.1f}s "
                    f"({int(span // 60)}:{span % 60:04.1f}), "
                    f"平均 fps={real_fps:.2f} — VFR 時間戳, "
                    f"播放時間 = 真實時間, 不需修正")
        elif real_fps:
            play_s = self.written / max(self.fps, 1)
            msg += (f"\n[Video]   真實時長 {span:.1f}s "
                    f"({int(span // 60)}:{span % 60:04.1f}), "
                    f"真實 fps={real_fps:.2f} (檔頭標 {self.fps})"
                    f"\n[Video]   不修正的話播放時間會是 {play_s:.1f}s "
                    f"({int(play_s // 60)}:{play_s % 60:04.1f}) — "
                    f"誤差 {(play_s / span - 1) * 100:+.1f}%")
        if self.error:
            msg += f"\n[Video]   ★ 錯誤: {self.error}"
        print(msg)

        # ── 用真實 fps 重新封裝 (無損, 只改時基) ──
        if real_fps:
            self._write_fps_sidecar(real_fps, span)
        if (self.remux_on_release and real_fps and self.written > 30
                and self.backend != 'vfr'  # ★ VFR 時間軸本來就對
                and abs(real_fps - self.fps) / max(self.fps, 1) > 0.02):
            self._remux(real_fps)

        return real_fps

    def _write_fps_sidecar(self, real_fps, span):
        """把真實 fps 寫成旁檔; 機上沒有 ffmpeg 時資訊才不會遺失."""
        try:
            scale = float(self.fps) / real_fps
            base = os.path.splitext(self.path)[0]
            with open(base + '.fps.txt', 'w') as fp:
                fp.write('nominal_fps=%d\n' % self.fps)
                fp.write('real_fps=%.4f\n' % real_fps)
                fp.write('frames=%d\n' % self.written)
                fp.write('real_duration_s=%.3f\n' % span)
                # ★ VFR 時間軸本來就對 → itsscale=1, 不需要 fix_cmd
                _vfr = (self.backend == 'vfr')
                fp.write('itsscale=%.6f\n' % (1.0 if _vfr else scale))
                fp.write('first_frame_id=%s\n' %
                         ('' if self.first_frame_id is None
                          else self.first_frame_id))
                fp.write('dropped=%d\n' % self.dropped)
                fp.write('backend=%s\n' % self.backend)
                fp.write('timebase=%s\n' %
                         ('vfr' if self.backend == 'vfr' else 'cfr'))
                if self.ts_path:
                    fp.write('ts_csv=%s\n' % os.path.basename(self.ts_path))
                if not _vfr:
                    fp.write("fix_cmd=ffmpeg -itsscale %.6f -i '%s' -c copy "
                             "'%s_fixed.mp4'\n" % (scale, self.path, base))
        except Exception:
            pass

    def _remux_gst(self, real_fps):
        """沒有 ffmpeg 時的退路: 用 gst-launch 重新封裝並改寫 framerate caps.

        Jetson 上本來就有 gst-launch (NVENC 就是它跑的), 不必裝任何東西.
        h264parse 可以接受下游指定的 framerate, mp4mux 會照這個寫時基.

        ★ 安全設計: 這條路我無法在出貨前驗證輸出正確性, 所以【不覆蓋原檔】,
          只另外產生 <name>_fixed.mp4. 原檔永遠保留.
        """
        gst = shutil.which('gst-launch-1.0')
        if gst is None:
            return False
        base = os.path.splitext(self.path)[0]
        out = base + '_fixed.mp4'
        num = max(1, int(round(real_fps * 1000)))
        cmd = [gst, '-q',
               'filesrc', f'location={self.path}', '!',
               'qtdemux', '!', 'h264parse', '!',
               f'video/x-h264,framerate={num}/1000', '!',
               'mp4mux', '!', 'filesink', f'location={out}']
        try:
            r = subprocess.run(cmd, timeout=300,
                               stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
            if r.returncode == 0 and os.path.exists(out) and \
                    os.path.getsize(out) > 1000:
                print(f'[Video]   ✓ gst 已產生時基修正版 (原檔保留): {out}')
                print(f'[Video]     請開來確認長度是否 ≈ '
                      f'{self.written / real_fps:.1f}s; 不對就用原檔.')
                return True
            if os.path.exists(out):
                os.remove(out)
        except Exception as e:
            print(f'[Video]   gst 重新封裝失敗: {e}')
            try:
                if os.path.exists(out):
                    os.remove(out)
            except Exception:
                pass
        return False

    def _remux(self, real_fps):
        """改時基讓播放時間 = 實際飛行時間 (不重新編碼).

        優先 ffmpeg -itsscale (原地覆蓋, 已驗證);
        沒有就退到 gst-launch (另存 _fixed.mp4, 不覆蓋).
        """
        if shutil.which('ffmpeg') is None:
            print('[Video]   ※ 沒有 ffmpeg — 真實 fps 已寫入 .fps.txt 旁檔.')
            if self._remux_gst(real_fps):
                return
            print('[Video]     (gst 退路也沒成功; 時基請在其他機器上修)')
            print(f"[Video]   (沒有 ffmpeg, 無法自動修正) 手動指令:\n"
                  f"          ffmpeg -itsscale {float(self.fps)/real_fps:.6f} "
                  f"-i '{self.path}' -c copy "
                  f"'{os.path.splitext(self.path)[0]}_fixed.mp4'")
            return
        base, ext = os.path.splitext(self.path)
        tmp = f"{base}_fixfps{ext}"
        # ★ 用 -itsscale 縮放輸入時戳 (不是 -r; -r 對已有時戳的 mp4 無效).
        #   scale = 檔頭標稱 fps / 真實 fps
        #   例: 標 25 實際 12.7 → scale 1.97 → 影片長度變成 1.97 倍 = 真實時長
        scale = float(self.fps) / real_fps
        cmd = ['ffmpeg', '-y', '-loglevel', 'error',
               '-itsscale', f'{scale:.6f}', '-i', self.path,
               '-c', 'copy', tmp]
        try:
            r = subprocess.run(cmd, timeout=180)
            if (r.returncode == 0 and os.path.exists(tmp)
                    and os.path.getsize(tmp) > 0):
                os.replace(tmp, self.path)
                print(f"[Video]   ✓ 已用真實 fps {real_fps:.2f} 重新封裝, "
                      f"播放時間現在對得上 log 的 t")
            else:
                print(f"[Video]   ✗ 重新封裝失敗 (rc={r.returncode}), 原檔保留")
                if os.path.exists(tmp):
                    os.remove(tmp)
        except Exception as e:
            print(f"[Video]   ✗ 重新封裝例外: {e}, 原檔保留")


# ══════════════════════════════════════════════════════════════════
#  ★ VFR 編碼子行程 (python3 video_writer.py --vfr-helper ...)
# ══════════════════════════════════════════════════════════════════
#  由 VideoWriter._try_vfr() 自動啟動, 一般不手動跑.
#  為什麼另開子行程而不是在主程式 import Gst:
#    nvv4l2h264enc 若 segfault 只死這個子行程, 不會拖垮飛控迴圈.
#  stdin 協定 (little-endian), 一幀一筆:
#    [8 bytes uint64 pts_ns][W*H*3 bytes BGR]
#  stdin EOF → 送 EOS → mp4 收尾 → 結束.
#  stdout: 管線 PLAYING 後印 "READY" (父行程握手). stderr: 錯誤訊息.

def _vfr_build_pipeline(path, w, h, bitrate, nominal_fps, encoder):
    # caps 的 framerate 只給編碼器當 rate-control 參考;
    # 真正的時間軸由每個 buffer 的 PTS 決定 (mp4mux 用 PTS/DTS 算 sample 時長).
    src = (f'appsrc name=src is-live=false format=time do-timestamp=false '
           f'block=true max-bytes={w * h * 3 * 4} '
           f'caps=video/x-raw,format=BGR,width={w},height={h},'
           f'framerate={int(nominal_fps)}/1')
    if encoder == 'nvenc':
        enc = ('videoconvert ! nvvidconv ! '
               'video/x-raw(memory:NVMM),format=NV12 ! '
               f'nvv4l2h264enc bitrate={int(bitrate)} ! ')
    elif encoder == 'x264':
        # 無 B-frame (zerolatency), 讓 DTS=PTS, 行為接近 nvv4l2h264enc 預設
        enc = ('videoconvert ! video/x-raw,format=I420 ! '
               f'x264enc bitrate={max(1, int(bitrate) // 1000)} '
               'tune=zerolatency speed-preset=ultrafast ! ')
    else:
        raise ValueError(f'unknown encoder {encoder}')
    mux = ('h264parse ! mp4mux streamable=true fragment-duration=1000 ! '
           f'filesink location={path}')
    return f'{src} ! {enc}{mux}'


def _vfr_helper_main(argv):
    if len(argv) < 6:
        sys.stderr.write('usage: --vfr-helper out.mp4 W H bitrate fps enc\n')
        return 2
    path = argv[0]
    w, h = int(argv[1]), int(argv[2])
    bitrate = int(argv[3])
    nominal_fps = int(argv[4])
    encoder = argv[5]

    # 父行程負責收尾 (關 stdin). Ctrl+C 不能先打死我們, 否則 mp4 沒收尾.
    signal.signal(signal.SIGINT, signal.SIG_IGN)

    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    Gst.init(None)

    try:
        pipe = Gst.parse_launch(_vfr_build_pipeline(path, w, h, bitrate,
                                               nominal_fps, encoder))
    except Exception as e:
        sys.stderr.write(f'parse_launch 失敗: {e}\n')
        return 3
    src = pipe.get_by_name('src')
    bus = pipe.get_bus()

    if pipe.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        sys.stderr.write('管線無法進入 PLAYING\n')
        return 4
    msg = bus.timed_pop_filtered(300 * Gst.MSECOND, Gst.MessageType.ERROR)
    if msg is not None:
        err, dbg = msg.parse_error()
        sys.stderr.write(f'管線錯誤: {err.message} | {dbg}\n')
        pipe.set_state(Gst.State.NULL)
        return 5

    sys.stdout.write('READY\n')
    sys.stdout.flush()

    frame_bytes = w * h * 3
    inp = sys.stdin.buffer
    hdr = struct.Struct('<Q')
    n = 0
    rc = 0
    last_pts = -1
    while True:
        head = inp.read(hdr.size)
        if len(head) < hdr.size:
            break                                   # EOF → 正常收尾
        data = inp.read(frame_bytes)
        if len(data) < frame_bytes:
            sys.stderr.write(f'第 {n} 幀資料不完整 ({len(data)} bytes)\n')
            break
        pts = hdr.unpack(head)[0]
        if pts <= last_pts:                         # 保證嚴格遞增
            pts = last_pts + 1_000_000
        last_pts = pts
        buf = Gst.Buffer.new_wrapped(data)
        buf.pts = pts
        buf.dts = pts
        ret = src.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            sys.stderr.write(f'push-buffer 失敗 ({ret}) 於第 {n} 幀\n')
            rc = 6
            break
        n += 1
        if n % 30 == 0:                             # 偶爾看一下有沒有錯誤
            msg = bus.pop_filtered(Gst.MessageType.ERROR)
            if msg is not None:
                err, dbg = msg.parse_error()
                sys.stderr.write(f'管線錯誤於第 {n} 幀: '
                                 f'{err.message} | {dbg}\n')
                rc = 7
                break

    # ── 收尾: EOS 讓 mp4mux 寫完最後一個 fragment ──
    src.emit('end-of-stream')
    t0 = time.time()
    msg = bus.timed_pop_filtered(10 * Gst.SECOND,
                                 Gst.MessageType.EOS | Gst.MessageType.ERROR)
    if msg is None:
        sys.stderr.write('等 EOS 逾時 (10s)\n')
        rc = rc or 8
    elif msg.type == Gst.MessageType.ERROR:
        err, dbg = msg.parse_error()
        sys.stderr.write(f'收尾錯誤: {err.message} | {dbg}\n')
        rc = rc or 9
    pipe.set_state(Gst.State.NULL)
    sys.stderr.write(f'[vfr] 完成 {n} 幀, 收尾 {time.time() - t0:.2f}s\n')
    return rc


# ══════════════════════════════════════════════════════════════════
#  自測: python3 video_writer.py --selftest [--encoder x264] [--secs 12]
# ══════════════════════════════════════════════════════════════════
def _selftest(argv):
    import argparse
    import numpy as np
    ap = argparse.ArgumentParser(prog='video_writer.py --selftest')
    ap.add_argument('--secs', type=float, default=12.0)
    ap.add_argument('--stall', type=float, default=2.0)
    ap.add_argument('--stall-at', type=float, default=5.0)
    ap.add_argument('--encoder', default='nvenc', choices=['nvenc', 'x264'])
    ap.add_argument('--out', default='/tmp/vw_selftest.mp4')
    a = ap.parse_args(argv)
    W, H = 1920, 1080
    base = os.path.splitext(a.out)[0]
    for ext in ('.mp4', '.ts.csv', '.fps.txt'):
        if os.path.exists(base + ext):
            os.remove(base + ext)

    vw = VideoWriter(a.out, 30, W, H, queue_len=120, vfr_encoder=a.encoder)
    if not vw.ok:
        print(f"✗ 啟動失敗: {vw.error}")
        return 1
    noise = [np.random.randint(0, 255, (H, W, 3), np.uint8) for _ in range(6)]
    t0 = time.monotonic()
    stalled = False
    i = 0
    while time.monotonic() - t0 < a.secs:
        if a.stall > 0 and not stalled and time.monotonic() - t0 >= a.stall_at:
            print(f"  ... 模擬卡住 {a.stall}s")
            time.sleep(a.stall)
            stalled = True
            continue
        vw.write(noise[i % len(noise)], frame_id=i)
        i += 1
        time.sleep(1 / 24)
    backend = vw.backend
    vw.release()

    rows = open(base + '.ts.csv').read().strip().splitlines()[1:]
    span = float(rows[-1].split(',')[3]) - float(rows[0].split(',')[3])
    pts = []
    if shutil.which('ffprobe'):
        out = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v',
                              '-show_entries', 'frame=pts_time',
                              '-of', 'csv=p=0', a.out],
                             capture_output=True, text=True).stdout
        pts = [float(x.strip(',')) for x in out.split() if x.strip(',')
               not in ('', 'N/A')]
    else:
        import cv2
        cap = cv2.VideoCapture(a.out)
        while cap.grab():
            pts.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
        cap.release()

    fails = 0

    def check(ok, msg):
        nonlocal fails
        print(f"  {'✓' if ok else '✗'} {msg}")
        fails += (not ok)
    print("\n" + "=" * 58 + "\n  自測結果\n" + "=" * 58)
    check(backend == 'vfr', f"後端 = {backend}")
    if len(pts) < 2:
        check(False, "讀不到影片幀")
        return 1
    dur = pts[-1] - pts[0]
    err = (dur / span - 1) * 100 if span > 0 else 999
    check(abs(err) < 3.0, f"影片長度 {dur:.2f}s vs 真實 {span:.2f}s "
                          f"(誤差 {err:+.1f}%)")
    check(len(pts) == vw.written, f"影片幀數 {len(pts)} = 寫入 {vw.written}")
    if a.stall > 0:
        gaps = [b - c for b, c in zip(pts[1:], pts[:-1])]
        g = max(gaps)
        check(g > 0.8 * a.stall, f"卡頓空窗 {g:.2f}s 有保留 (預期 ≈{a.stall}s)")
    print("結論: " + ("✓ 通過" if not fails else f"✗ {fails} 項失敗, 先別飛"))
    return 0 if not fails else 1


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--vfr-helper':
        sys.exit(_vfr_helper_main(sys.argv[2:]))
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        sys.exit(_selftest(sys.argv[2:]))
    print(__doc__)
