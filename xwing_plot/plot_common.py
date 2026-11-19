# -*- coding: utf-8 -*-
"""plot_common.py — xwing_plot 共用: 找 log 檔.

這個專案的飛行程式寫 flight_logs/xwing_vision_<時間>_<case>.csv;
B (hsu_realflight_true) 的 xwing_virtual_*.csv 也一起認, 方便同一套工具畫兩邊的 log.
"""
import glob
import os

LOG_PATTERNS = ('xwing_vision_*.csv', 'xwing_virtual_*.csv')
# 同一個資料夾裡的附屬檔 (不是主 log)
_NOT_MAIN = ('_uart_health.csv', '_events.csv', '_cut.csv')


def is_main_log(path):
    b = os.path.basename(path)
    return (b.endswith('.csv') and not b.endswith(_NOT_MAIN)
            and '_run' not in b and '_part' not in b)


def list_logs(log_dir):
    """log_dir 裡所有主 log (依檔名排序)."""
    out = set()
    for pat in LOG_PATTERNS:
        out.update(p for p in glob.glob(os.path.join(log_dir, pat)) if is_main_log(p))
    return sorted(out)


def latest_log(log_dir):
    cs = list_logs(log_dir)
    return max(cs, key=os.path.getmtime) if cs else None
