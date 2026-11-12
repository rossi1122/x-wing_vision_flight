#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_diag.py — 解讀雲台 UART 傳輸診斷檔 (飛行程式 2026/10/30 版以後才有)
═══════════════════════════════════════════════════════════════════
飛行程式會在 flight_logs 另存:
  <log 名>_uart_raw.bin      收 / 送的每一個 byte + 時間
  <log 名>_uart_health.csv   每秒一列健康狀態
  <log 名>_dmesg_*.txt       RX 斷線當下的 kernel 訊息
這支程式把它們整理成「什麼時候斷、斷多久、斷線期間我們有沒有在送、
對方恢復後的封包編號跳了多少 (= 對方是不是一直有在算只是沒送到)」.

用法 (在 xwing_plot 資料夾):
  python uart_diag.py                              # flight_logs 最新那份
  python uart_diag.py xwing_vision_XXXX.csv       # 指定 log (給 csv 或 _uart_raw.bin 都可以)
  python uart_diag.py ... --gap 0.5                # 多久沒收到 byte 算一次中斷 (預設 0.5 s)
輸出: plot_results/<log 名>/uart/  uart_diag.txt + uart_diag.png
"""
import argparse
import csv
import datetime as dt
import glob
import json
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RAW_MAGIC = b'XWUARTRAW1\n'
REC = struct.Struct('<QBH')
KIND = {0: 'RX', 1: 'TX', 2: 'NOTE'}


def read_raw(path):
    with open(path, 'rb') as f:
        data = f.read()
    if not data.startswith(RAW_MAGIC):
        sys.exit(f'{path} 不是診斷檔')
    nl = data.index(b'\n', len(RAW_MAGIC))
    head = json.loads(data[len(RAW_MAGIC):nl])
    recs, i = [], nl + 1
    while i + REC.size <= len(data):
        t, k, n = REC.unpack_from(data, i)
        i += REC.size
        if i + n > len(data):
            break                           # 程式被砍時最後一筆可能不完整
        recs.append((t / 1e9, k, data[i:i + n]))
        i += n
    return head, recs


def read_health(path):
    if not os.path.isfile(path):
        return []
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description='雲台 UART 傳輸診斷')
    ap.add_argument('log', nargs='?')
    ap.add_argument('--gap', type=float, default=0.5, help='多久沒收到任何 byte 算中斷 (s)')
    ap.add_argument('--log-dir', default=os.path.join(ROOT, 'flight_logs'))
    a = ap.parse_args()

    if a.log:
        p = a.log if os.path.isfile(a.log) else os.path.join(a.log_dir, os.path.basename(a.log))
        stem = p[:-len('_uart_raw.bin')] if p.endswith('_uart_raw.bin') else os.path.splitext(p)[0]
    else:
        cs = glob.glob(os.path.join(a.log_dir, '*_uart_raw.bin'))
        if not cs:
            sys.exit(f'{a.log_dir} 裡沒有 *_uart_raw.bin (飛行程式要是 10/30 以後的版本)')
        stem = max(cs, key=os.path.getmtime)[:-len('_uart_raw.bin')]
    raw = stem + '_uart_raw.bin'
    if not os.path.isfile(raw):
        sys.exit(f'找不到 {raw}')
    head, recs = read_raw(raw)
    health = read_health(stem + '_uart_health.csv')
    name = os.path.basename(stem)
    out_dir = os.path.join(HERE, 'plot_results', name, 'uart')
    os.makedirs(out_dir, exist_ok=True)

    m0, w0 = head['mono_ns'] / 1e9, head['wall']
    t_first = recs[0][0] if recs else m0

    def rel(t):
        return t - m0

    def wall(t):
        return dt.datetime.fromtimestamp(w0 + (t - m0)).strftime('%H:%M:%S.%f')[:-3]

    rx = [(t, len(d)) for t, k, d in recs if k == 0]
    tx = [(t, d) for t, k, d in recs if k == 1]
    notes = [(t, d.decode('utf-8', 'replace')) for t, k, d in recs if k == 2]
    t_end = recs[-1][0] if recs else m0

    L = [f'檔案: {raw}', f'埠: {head.get("port")} @ {head.get("baud")}',
         f'開始: {dt.datetime.fromtimestamp(w0)} (Orin 系統時間; 沒對時就只看相對秒數)',
         f'長度: {t_end - m0:.1f} s   RX {sum(n for _, n in rx)} bytes / TX {sum(len(d) for _, d in tx)} bytes',
         '']

    # ── RX 中斷 (沒收到任何 byte) ──
    gaps = []
    prev = m0
    for t, _ in rx:
        if t - prev > a.gap:
            gaps.append((prev, t))
        prev = t
    if t_end - prev > a.gap:
        gaps.append((prev, None))            # 到結束都沒恢復
    L.append(f'【RX 中斷】 (超過 {a.gap:g} s 沒收到任何 byte)  共 {len(gaps)} 次')

    def hrow(t):
        """最接近 t 的健康紀錄."""
        best = None
        for r in health:
            tm = num(r.get('t_mono'))
            if tm is not None and tm <= t + 0.5:
                best = r
        return best

    for g0, g1 in gaps:
        end = g1 if g1 is not None else t_end
        txs = [d for t, d in tx if g0 < t <= end]
        keys = [d.decode('ascii', 'replace') for d in txs if len(d) == 1]
        clk = sum(1 for d in txs if len(d) > 1)
        h0, h1 = hrow(g0), hrow(end)
        L.append(f'  {rel(g0):8.1f} s ({wall(g0)}) → '
                 + (f'{rel(g1):8.1f} s  中斷 {g1 - g0:6.2f} s' if g1 is not None
                    else f'結束都沒恢復 ({end - g0:.1f} s)'))
        L.append(f'      斷線期間我們送出: 模式鍵 {keys or "無"}, 校時請求 {clk} 次'
                 + ('  ← TX 正常, 是「對方 → 我們」這個方向沒資料' if txs else ''))
        if h0 and h1:
            r0, r1 = num(h0.get('partner_rid')), num(h1.get('partner_rid'))
            hz = (r1 - r0) / (end - g0) if (r0 is not None and r1 is not None and g1) else None
            if g1 is not None and r0 is not None and r1 is not None:
                L.append(f'      對方封包編號 {r0:.0f} → {r1:.0f} (跳 {r1 - r0:.0f})'
                         + (f' ≈ 中斷期間對方仍以 {hz:.0f} Hz 在產生結果, 只是沒送到'
                            if hz and hz > 5 else ' ≈ 對方中斷期間沒有在產生結果 (程式停了 / 重開)'
                            if r1 is not None and r0 is not None and r1 < r0 + 5 else ''))
            for k in ('rx', 'frame', 'brk', 'overrun', 'parity'):
                v0, v1 = num(h0.get('drv_' + k)), num(h1.get('drv_' + k))
                if v0 is not None and v1 is not None and v1 != v0:
                    L.append(f'      驅動 {k}: {v0:.0f} → {v1:.0f}'
                             + ('  ← RX 線上有雜訊/斷開 (線鬆或 GND 沒接好)' if k in ('frame', 'brk') else ''))
            ex0, ex1 = h0.get('port_exists'), h1.get('port_exists')
            if '0' in (ex0, ex1) or h0.get('port_real') != h1.get('port_real'):
                L.append(f'      ★ 我們這端 USB-UART 掉了/重新列舉: {h0.get("port_real")} → {h1.get("port_real")}')
    L.append('')

    L.append('【程式訊息 (NOTE)】')
    for t, s in notes:
        L.append(f'  {rel(t):8.1f} s ({wall(t)})  {s}')
    L.append('')

    dm = sorted(glob.glob(stem + '_dmesg_*.txt'))
    L.append('【kernel 訊息 (斷線當下)】' + ('' if dm else '  無'))
    for p in dm:
        with open(p, encoding='utf-8', errors='replace') as f:
            lines = f.read().splitlines()
        hit = [s for s in lines if any(k in s.lower() for k in
                                      ('usb', 'tty', 'cp210', 'serial', 'disconnect', 'error'))]
        L.append(f'  {os.path.basename(p)}: 相關 {len(hit)} 行 (最後 15 行)')
        L += ['    ' + s for s in hit[-15:]]
    txt = '\n'.join(L)
    print(txt)
    with open(os.path.join(out_dir, 'uart_diag.txt'), 'w', encoding='utf-8') as f:
        f.write(txt + '\n')

    # ── 圖 ──
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(沒有 matplotlib, 只輸出文字)')
        return
    if not health:
        return
    t = [num(r['t_mono']) - m0 for r in health]
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    ax[0].plot(t, [num(r['rx_bytes_1s']) for r in health], label='RX bytes/s')
    ax[0].plot(t, [num(r['tx_bytes_1s']) for r in health], label='TX bytes/s')
    ax[0].set_ylabel('bytes/s')
    ax[1].plot(t, [num(r['frames_1s']) for r in health], label='frames/s')
    ax[1].plot(t, [num(r['clock_1s']) for r in health], label='clock replies/s')
    ax[1].set_ylabel('per s')
    ax[2].plot(t, [num(r['partner_rid']) for r in health], label='partner result_id')
    ax[2].set_ylabel('id')
    for k in ('crc', 'header', 'discarded', 'serial_err', 'reopen', 'drv_frame', 'drv_brk', 'drv_overrun'):
        v = [num(r.get(k)) for r in health]
        if any(x for x in v if x):
            ax[3].plot(t, v, label=k)
    ax[3].plot(t, [1 - (num(r['port_exists']) or 0) for r in health], 'k--', label='port missing')
    ax[3].set_ylabel('error counters')
    ax[3].set_xlabel('time since program start (s)')
    for g0, g1 in gaps:
        for x in ax:
            x.axvspan(rel(g0), rel(g1 if g1 is not None else t_end), color='r', alpha=.15)
    for x in ax:
        x.grid(alpha=.3)
        x.legend(loc='upper right', fontsize=8)
    fig.suptitle(f'Gimbal UART link — {name}  (red = no RX bytes)')
    fig.tight_layout()
    png = os.path.join(out_dir, 'uart_diag.png')
    fig.savefig(png, dpi=110)
    print(f'\n→ {out_dir}')


if __name__ == '__main__':
    main()
