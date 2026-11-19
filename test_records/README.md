# 測試紀錄

每次測試一個資料夾：log 原檔 + `測試紀錄.md` (用哪顆雲台、目的、結論)。
大的 CSV / `_uart_raw.bin` 用 gzip 壓縮 (`.gz`)，畫圖前先 `gunzip -k *.gz`。

| 日期 | 雲台 | 測試 | 結論 | 資料夾 |
|---|---|---|---|---|
| 2026/11/16 | **田屋大雲台** | 實驗室地面 UART 連線 (控制板故障中，沒有角度輸出) | 資料傳輸正常；格式 v8；pitch / roll 全 NaN (預期內) | [`20261116_tianwu_large_gimbal_link_test`](20261116_tianwu_large_gimbal_link_test/測試紀錄.md) |
| 2026/11/16 | **田屋小雲台** | 實驗室地面 UART 連線 + 模式鍵 + 偵測換算 (817 s，撥 GUIDED 4 次) | 資料傳輸正常；格式 v10；角度正常、鎖定 −30.1°；yaw 是相對機頭 (≈0)；已確認 yaw 軸不會轉 → 維持 `'none'` | [`20261116_tianwu_small_gimbal_link_test`](20261116_tianwu_small_gimbal_link_test/測試紀錄.md) |
| 2026/11/16 | **台鴞** | 實驗室地面 UART 連線 + 模式鍵 + 偵測換算 (185 s，撥 GUIDED 3 次) | 資料傳輸正常；格式 v8；角度正常、鎖定 −29.9°；⚠ 影像延遲約 264 ms；⚠ SEARCH 時雲台自己切到鎖定 2 次 (各 3.5 s) | [`20261116_taiyao_camera_link_test`](20261116_taiyao_camera_link_test/測試紀錄.md) |
| 2026/11/17 | **田屋小雲台** (第 2 次) | 飛測前再確認：UART 連線 + 模式鍵 + 偵測換算 (438 s，撥 GUIDED 3 次) | **沒有問題** ✅；格式 v10；延遲 87 ms；yaw 沒有轉動；追蹤垂直置中；地面空速已正常 (≈ 1.7 m/s) | [`20261117_tianwu_small_gimbal_link_test2`](20261117_tianwu_small_gimbal_link_test2/測試紀錄.md) |
| 2026/11/17 | **台鴞** (第 2 次) | 飛測前再確認：UART 連線 + 模式鍵 + 偵測換算 (434 s，撥 GUIDED 3 次) | 前 241 s 對方沒送資料 (之後正常、0 中斷)；格式 v8；⚠ 延遲約 259 ms；SEARCH 沒有自己切模式 ✓ | [`20261117_taiyao_camera_link_test2`](20261117_taiyao_camera_link_test2/測試紀錄.md) |
| **2026/11/18** | **田屋小雲台** | **實飛** (`--virtual-control`，3 趟完整俯衝) | 控制正常：miss 對虛擬 49 / 44 / 16 m、拉平後最低 117~124 m；空中連線 100 %、鎖定 0.5 s；⚠ PRE_DIVE / DIVE 0 % 偵測；雲台 roll 跟著機身、yaw 空中 ±5°；沒有車子 GPS | [`20261118_tianwu_small_flight`](20261118_tianwu_small_flight/測試紀錄.md) |
| **2026/11/18** | **台鴞** | **實飛** (`--virtual-control`) | ✗ 撥 GUIDED 2 次都沒俯衝、直接右轉：**飛控在 11:47:26 重開機 (疑似接主電池)，MAVROS 沒重連** (飛控 log 確認)，程式一直以為飛機在地上 / MANUAL，沒開始任務 (飛控 GUIDED 沒命令 → 繞圈)；已加飛控資料看門狗 | [`20261118_taiyao_flight_mavros_lost`](20261118_taiyao_flight_mavros_lost/測試紀錄.md) |
