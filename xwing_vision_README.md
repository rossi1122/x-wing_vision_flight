# xwing_vision — X-wing 全視覺俯衝版

> 跟 `hsu_realflight_true` 的 xwing_virtual **同一個場景、同一套飛法、同一組參數**
> (240 m / 30 m/s / 俯衝 30° / 140 m 拉平 / WP3 撥 GUIDED)，
> 差別只有：**車在哪裡，改由對方雲台相機決定**，不再用虛擬軌跡控制。

## 1. 一次看懂

| 階段 | xwing_virtual (B) | **xwing_vision (本版)** |
|---|---|---|
| 程式一開 / AUTO 進場 | 雲台不送鍵 | **送 `'1'` 追蹤**，雲台先開始找車 |
| SEARCH，還沒看到車 | 朝虛擬目標飛 | 朝虛擬目標 (WP4) 飛，**只當帶路** |
| SEARCH，看到車 | — | 最近 15 筆有 10 筆偵測到 → **鎖定** → 航向 = 車的方位 |
| 何時推頭 | 虛擬距離 ≤ 415.69 m | **視覺水平距** (近 5 筆中位數) ≤ 415.69 m，且車偏機頭 ≤ 10° |
| PRE_DIVE | 機頭 −30°，朝虛擬目標 | 機頭 −30°，**朝車**；雲台送 `'2'` 鎖定 −30° |
| 進 DIVE | 機頭 ≤ −27° | 機頭 ≤ −27° **且看得到車** |
| DIVE 瞄準 | 虛擬目標投影 | **對方相機像素 → 雲台補償** → PID |
| 看不到車 | (虛擬目標不會不見) | SEARCH：25 s 沒鎖定 / 鎖定後跟丟 5 s → **平飛交還**<br>PRE_DIVE 跟丟 1.5 s / DIVE 跟丟 1 s → **拉起中止** |
| 車要準時出發嗎 | **要**（車慢 100 m 就打錯地方） | **不用**，只要在馬路上、在飛機前方 |

### 推頭是怎麼判斷的 (白話)
雲台在追蹤模式下會上下轉，讓車停在畫面正中間。所以「雲台往下看幾度」就是「看車的角度」。
240 m 高、離車水平 416 m 時，這個角度剛好是往下 30°：

> **雲台追著車轉到約 −30° (距離 = 240 ÷ tan30° = 416 m)、而且車在機頭正前方 → 開始俯衝。**

## 2. 對方雲台資料 (2026/11 確認)

| 項目 | 內容 | 程式怎麼用 |
|---|---|---|
| `target_cx/cy` | 車在畫面的像素；沒有車 = NaN | NaN → 這筆算「沒看到」 |
| `fx/fy/cx/cy` | 相機內參 (每筆都送) | 像素 → 角度 |
| `pitch` | **對地**角度，往下為負，**拍照那一刻**的值 | 直接當相機俯仰，不加飛機姿態 |
| `roll` | 相機 roll，雲台維持 0° | 直接使用 |
| yaw | **沒有 yaw 軸**，相機朝機頭 | 相機方位 = 飛機 yaw (+ 安裝偏差) |
| `cap_mono` (v7/v8) | 取像時刻 (UART 校時換到本機) | 查「拍照那一刻」的飛機姿態 |
| 雷射 (v8) | 不準 | 只記錄，不用 |

## 3. 檔案

| 檔案 | 狀態 | 內容 |
|---|---|---|
| `xwing_vision_flight.py` | 新增 | **實飛主程式**。由 B 的 `xwing_virtual_flight.py` (11/04 實飛版) 改出，飛法 / 重複撥 GUIDED / 啟動檢查 / 雲台 / log 相同 |
| `xwing_vision_config.py` | 新增 | 全部參數。任務、PID、方向舵、安全值 = B 的 11/04 版；另加全視覺參數 (第 4 節) |
| `xwing_vision_common.py` | 新增 | 建虛擬帶路目標、狀態機參數、bridge 參數 (實飛與模擬共用) |
| `vision_guidance.py` | 新增 | **核心計算**：每筆新辨識結果 → 車的方位、偏機頭角、水平距、DIVE 補償像素、車的地面位置估計 |
| `dive_guidance.py` | 修改 | 加 `VISION_GUIDANCE` 開關 (預設關 → 原本 `run_vision_flight.py` 行為完全不變) |
| `uart_mipi_receiver.py` / `xwing_uart_vision.py` / `xwing_uart_sender_sim.py` | 從 B 複製 (未改) | 對方 UART v8 協定、背景接收 (斷線重開、模式鍵、校時、診斷)、模擬發送端 |
| `vision_sim_camera.py` | 新增 | 桌上測試用：模擬真車 + 對方雲台 (追蹤 / 鎖定 / 30 Hz / 延遲 / 雜訊 / 漏偵測) |
| `xwing_vision_sim.py` | 新增 | 閉迴路任務模擬 (B 的飛機模型 + 模擬雲台 + 真車) |
| `test_vision_guidance.py` | 新增 | 幾何、狀態機、模擬、實飛主程式 (假 ROS) 共 44 項測試 |

原本的 `run_vision_flight.py`、`config.py`、`vision.py` 等**沒有動**。

## 4. 全視覺參數 (`xwing_vision_config.py`)

| 參數 | 值 | 意思 |
|---|---|---|
| `VISION_ACQUIRE_HITS` / `_WINDOW` | 10 / 15 | 最近 15 筆有 10 筆看到車 → 鎖定 (約 0.5 s) |
| `VISION_MEAS_MAX_AGE_S` | 0.5 s | 最後一次看到車在 0.5 s 內 = 「現在看得到」 |
| `VISION_GROUND_MEDIAN_N` | 5 | 推頭用的距離 = 最近 5 筆中位數 |
| `VISION_SEARCH_TIMEOUT_S` | 25 s | 撥 GUIDED 後 25 s 沒鎖定 → 平飛交還 |
| `VISION_LOST_SEARCH_S` | 5 s | SEARCH 鎖定後跟丟 → 平飛交還 (期間保持最後方位) |
| `VISION_LOST_PREDIVE_S` | 1.5 s | PRE_DIVE 跟丟 → 拉起中止 |
| `VISION_LOST_DIVE_S` | 1.0 s | DIVE 跟丟 → 拉起中止 |
| `VISION_PREDIVE_MAX_OFF_DEG` | 10° | 車偏機頭超過就先不推頭 |
| `VISION_PREDIVE_MIN_DIST_M` | 300 m | 鎖定時已 < 300 m → 放棄這趟 (避免太陡) |
| `GIMBAL_PITCH_OFFSET_DEG` | 0 | ★ 地面校正：光軸實際俯角 − 回報值 |
| `GIMBAL_YAW_OFFSET_DEG` | 0 | ★ 地面校正：光軸偏機頭右方幾度 |
| `GIMBAL_MODE_KEYS` | STANDBY/SEARCH `'1'`，PRE_DIVE/DIVE `'2'` | 雲台模式鍵 |

撥 GUIDED 啟動檢查 = B 的檢查 (高度 ≥ 200 m、離起點 486~1500 m、起點在機頭前方 45° 內)
**再加：雲台資料鏈 2 s 內有封包**，否則拒絕啟動。

## 5. 執行

```bash
roslaunch mavros apm.launch
python3 xwing_vision_flight.py                    # 全視覺 (預設)
python3 xwing_vision_flight.py --vision-port /dev/ttyUSB0
python3 xwing_vision_flight.py --virtual-control  # 退回 B 的做法 (虛擬控制, 視覺只記錄)
python3 xwing_vision_flight.py --bench            # 掛機
```

開機會印 `控制: ★ 全視覺`。每秒狀態列最後一段：

```
| VIS link=1 lock=1 hits=15 age=0.1s dist=452m gmode=1(req 1) gpitch=-27.9
```

events 會記：鎖定 (`★ 視覺鎖定目標 ... 水平距 / 方位 / 虛擬目標水平距`)、每次換 mode
(含視覺與虛擬水平距)、中止原因 (`VISION_NOT_FOUND` / `VISION_LOST_*` / `VISION_TOO_CLOSE_*`)、
拉平時對「視覺估計的車位置」的 miss。

**看到「結束這趟」或 ABORT → 切回 AUTO 繞回來，到 WP3 再撥 GUIDED。**

主 CSV 新增欄位：`control`、`vis_locked`、`abort_reason`、`vg_*`
(方位、偏機頭、俯角、水平距、中位數、車的 NED / GPS 估計、取像時刻來源、姿態配對)、
`gcomp_u/v` (DIVE 補償像素)、`real_detected`；B 的 `vis_*` 原始欄位全部保留。

## 6. 測試結果 (桌上，2026/11/04)

```bash
python3 test_vision_guidance.py     # → PASS (44 項)
python3 xwing_vision_sim.py --all   # 各情境總表
```

| 情境 | 推頭時真實距離 | 對真車 miss | 中止原因 |
|---|---|---|---|
| 全視覺 / 車照虛擬目標開 (10 m/s) | 413 m | 79.5 m | |
| 全視覺 / 車慢 (11/04 實測加速) | 409 m | 46.2 m | |
| 全視覺 / 車停著沒開 | 411 m | **2.3 m** | |
| 全視覺 / 沒有車 | — | — | VISION_NOT_FOUND |
| 全視覺 / 車慢 + 30% 漏偵測 | 411 m | 46.3 m | |
| 全視覺 / 車慢 + 雲台 pitch 偏 1° (沒校正) | 427 m | 56.0 m | |
| 全視覺 / 車慢 + yaw 偏 3° (沒校正) | 409 m | 48.5 m | |
| 全視覺 / 車慢 + DIVE 中資料鏈斷 2 s | 409 m | — | VISION_LOST_DIVE |
| 虛擬控制 (B) / 車照虛擬目標開 | 415 m | 80.0 m | |
| 虛擬控制 (B) / 車慢 | **341 m** | 37.0 m | |
| 虛擬控制 (B) / 車停著沒開 | **321 m** | **145.1 m** | |

- 全視覺**一律在真實 ~410 m 推頭**；B 的做法遇到車慢 / 車沒開，推頭點就跑掉 (341 / 321 m)。
- 虛擬控制模式的數字與 B 自己的 `xwing_virtual_sim.py` 完全相同 (−28.8°、45.5 m/s、124 m、80.0 m)，表示原本的控制路徑沒有被改壞。
- ⚠ **移動的車 miss 約 50~80 m (打在車後)**：這是「純追擊、沒有前置瞄準」的幾何特性，B 也一樣
  (`MOVING_TARGET_LEAD = False`)，不是視覺的問題。車停著時只有 2 m。要改善需要前置瞄準
  (用視覺估計車速)，是下一步的工作。
- 雲台 pitch 偏 1° → 推頭點差約 17 m，所以**實飛前一定要校正 `GIMBAL_PITCH_OFFSET_DEG`**。

這些是模擬結果，飛機模型是估算 (同 B)；**實機、實際雲台都還沒測過**。

## 7. 上機前要做的

1. **相機彈射後不失焦** (對方處理)。
2. **地面校正雲台偏差**：飛機水平、車放正前方已知距離 D、高度 H，追蹤模式讓車置中，
   比較 log 的 `vg_ground_m` 與 D，調 `GIMBAL_PITCH_OFFSET_DEG`；車放正前方，看
   `vg_bearing_off_deg` 是否 ≈ 0，調 `GIMBAL_YAW_OFFSET_DEG`。
3. **地面整合**：`python3 xwing_vision_flight.py --bench`，接對方雲台，確認鎖定、模式鍵、跟丟中止。
4. **第一次實飛建議先用 `--virtual-control`** (= B 的做法，控制安全)，同時記錄全部視覺欄位，
   飛完比對 `vg_ground_m` / `vg_tgt_lat/lon` 跟車子 GPS，確認準了再改全視覺。
