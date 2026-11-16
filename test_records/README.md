# 測試紀錄

每次測試一個資料夾：log 原檔 + `測試紀錄.md` (用哪顆雲台、目的、結論)。
大的 CSV / `_uart_raw.bin` 用 gzip 壓縮 (`.gz`)，畫圖前先 `gunzip -k *.gz`。

| 日期 | 雲台 | 測試 | 結論 | 資料夾 |
|---|---|---|---|---|
| 2026/11/16 | **田屋大雲台** | 實驗室地面 UART 連線 (控制板故障中，沒有角度輸出) | 資料傳輸正常；格式 v8；pitch / roll 全 NaN (預期內) | [`20261116_tianwu_large_gimbal_link_test`](20261116_tianwu_large_gimbal_link_test/測試紀錄.md) |
| 2026/11/16 | **田屋小雲台** | 實驗室地面 UART 連線 + 模式鍵 + 偵測換算 (817 s，撥 GUIDED 4 次) | 資料傳輸正常；格式 v10；角度正常、鎖定 −30.1°；⚠ yaw 是相對機頭 (≈0)，待對方確認 | [`20261116_tianwu_small_gimbal_link_test`](20261116_tianwu_small_gimbal_link_test/測試紀錄.md) |
