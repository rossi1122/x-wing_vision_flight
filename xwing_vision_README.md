# xwing_vision — X-wing 目標偵測版

> 跟 `hsu_realflight_true` 的 xwing_virtual **同一個場景、同一套飛法、同一組參數、
> 同一個狀態機與判斷邏輯** (240 m / 30 m/s / 俯衝 30° / 140 m 拉平 / WP3 撥 GUIDED)。
> **唯一的差別：狀態機的「目標」輸入，從虛擬軌跡換成對方雲台的目標偵測。**

## 1. 一次看懂

`dive_guidance.py` (狀態機) **一行都沒改**。原本它從虛擬目標拿到這些欄位，
現在同名欄位改由偵測結果填入 (`vision_guidance.VisionSource`)：

| 欄位 | 原本 (虛擬軌跡) | 現在 (目標偵測) | 狀態機拿來做什麼 (原邏輯) |
|---|---|---|---|
| `tgt_n`, `tgt_e` | 虛擬車位置 | 偵測算出的車地面位置 (看不到時保持最後一次) | SEARCH / PRE_DIVE 航向 = 飛機指向它 (43002) |
| `predive_dist_m` | 到虛擬車水平距 | 偵測算出的水平距 (看不到 = 無) | ≤ 415.69 m 就推頭 |
| `tgt_valid` | 永遠有效 | 這拍看得到車 | PRE_DIVE / DIVE 看不到 > 1.5 s → ABORT |
| `bbox_u`, `bbox_v`, `bbox_detected` | 虛擬車投影到機身相機 | 車在畫面的位置, 經雲台角度換到同一個機身相機 | DIVE PID 修 pitch / 方向舵；進 DIVE 要看得到 |

**還沒偵測到過車** (剛撥 GUIDED) 時，目標位置先用虛擬目標 (WP4) 帶路，
但距離、畫面位置、有效旗標一律來自偵測，所以**沒看到車就不會推頭**。

### 流程 (同 xwing_virtual)
```
AUTO 240 m / 30 m/s → WP3 撥 GUIDED
→ SEARCH:   飛控 43002 朝目標位置飛, TECS 定高定速
→ 到目標水平距 ≤ 415.69 m (= 240 / tan30°) → PRE_DIVE: pitch −30°, 雲台送 '2' 鎖定 −30°
→ pitch 到 −27° 內且看得到目標 → DIVE: 目標畫面位置 PID
→ AGL ≤ 140 m 或空速 ≥ 60 m/s → PULLOUT → DONE (TECS 140 m / 30 m/s)
PRE_DIVE / DIVE 看不到目標 > 1.5 s → ABORT 拉起 (原 ABORT_TARGET_LOST_S)
```
SEARCH 段原本就沒有逾時：一直沒看到車就不會推頭，**由駕駛員切出 GUIDED**。

## 2. 對方雲台資料 (2026/11 確認)

| 項目 | 內容 | 程式怎麼用 |
|---|---|---|
| `target_cx/cy` | 車在畫面的像素；沒有車 = NaN | NaN → 這拍看不到 |
| `fx/fy/cx/cy` | 相機內參 (每筆都送) | 像素 → 角度 |
| `pitch` | **對地**角度，往下為負，**拍照那一刻**的值 | 直接當相機俯仰 |
| `roll` | 相機 roll，雲台維持 0° | 直接使用 |
| yaw | **沒有 yaw 軸**，相機朝機頭 | 相機方位 = 飛機 yaw (+ 安裝偏差) |
| `cap_mono` (v7/v8) | 取像時刻 (UART 校時換到本機) | 查拍照那一刻的飛機航向 |
| 雷射 (v8) | 不準 | 只記錄 |

水平距 = 高度 ÷ tan(看車的俯角)；看車的俯角 = 雲台往下角度 + 車在畫面上下偏的角度。

## 3. 檔案

| 檔案 | 狀態 | 內容 |
|---|---|---|
| `xwing_vision_flight.py` | 新增 | 實飛主程式。= B 的 `xwing_virtual_flight.py`，只把目標輸入換成 `VisionSource` |
| `xwing_vision_config.py` | 新增 | 參數 = B 的 `xwing_virtual_config.py` (11/04 版)，另加相機校正值與資料過期時間 |
| `xwing_vision_common.py` | 新增 | = B 的 `xwing_virtual_common.py` (狀態機參數相同) |
| `vision_guidance.py` | 新增 | `VisionSource`：偵測結果 → 與虛擬目標同名的欄位 |
| `uart_mipi_receiver.py` / `xwing_uart_vision.py` / `xwing_uart_sender_sim.py` | 從 B 複製 (未改) | 對方 UART v8 協定、背景接收、模擬發送端 |
| `vision_sim_camera.py` / `xwing_vision_sim.py` | 新增 | 桌上閉迴路模擬 (真車 + 模擬對方雲台) |
| `test_vision_guidance.py` | 新增 | 38 項測試 |

`dive_guidance.py`、`attitude_bridge.py`、`run_vision_flight.py` 等原檔**都沒有改**。

## 4. 新增的參數 (`xwing_vision_config.py`)

只有「目標怎麼從偵測算出來」需要的值，沒有新增任何判斷條件：

| 參數 | 值 | 意思 |
|---|---|---|
| `VISION_PKT_MAX_AGE_S` | 0.3 s | 最新封包超過這麼久沒更新 → 這拍算看不到 (資料鏈斷) |
| `GIMBAL_PITCH_OFFSET_DEG` | 0 | ★ 地面校正：光軸實際俯角 − 回報值 |
| `GIMBAL_YAW_OFFSET_DEG` | 0 | ★ 地面校正：光軸偏機頭右方幾度 |
| `POSE_MAX_GAP_S` | 0.15 s | 查拍照那一刻的飛機姿態，前後樣本最遠差距 |

其他 (推頭 415.69 m、ABORT 1.5 s、啟動檢查、雲台模式鍵…) 全部 = B。

## 5. 執行

```bash
roslaunch mavros apm.launch
python3 xwing_vision_flight.py                    # 目標來自偵測 (預設)
python3 xwing_vision_flight.py --vision-port /dev/ttyUSB0
python3 xwing_vision_flight.py --virtual-control  # 目標來自虛擬軌跡 (= B, 偵測只記錄)
python3 xwing_vision_flight.py --bench            # 掛機
```

開機印 `目標來源: ★ 目標偵測`。每秒狀態列最後一段：
`| VIS link=1 det=1 dist=452m gmode=1(req 1) gpitch=-27.9`

主 CSV：B 的欄位全部保留，另加 `control`、`abort_reason`、`vg_*`
(這拍看不看得到、原因、方位、偏機頭、俯角、水平距、車的 NED / GPS 估計)、`gcomp_u/v`。

## 6. 測試結果 (桌上)

```bash
python3 test_vision_guidance.py     # → PASS (38 項)
python3 xwing_vision_sim.py --all   # 各情境總表
```

| 情境 | 推頭時真實距離 | 對真車 miss | 結果 |
|---|---|---|---|
| 偵測 / 車照虛擬目標開 (10 m/s) | 415 m | 80.3 m | |
| 偵測 / 車慢 (11/04 實測加速) | 411 m | 46.7 m | |
| 偵測 / 車停著沒開 | 414 m | **2.7 m** | |
| 偵測 / 沒有車 | — | — | 不推頭 |
| 偵測 / 車慢 + 30% 漏偵測 | 414 m | 45.3 m | |
| 偵測 / 車慢 + 雲台 pitch 偏 1° (沒校正) | 429 m | 56.3 m | |
| 偵測 / 車慢 + DIVE 中資料鏈斷 2 s | 411 m | — | TARGET_LOST_DIVE 拉起 |
| 虛擬軌跡 (B) / 車照虛擬目標開 | 415 m | 80.0 m | |
| 虛擬軌跡 (B) / 車慢 | **341 m** | 37.0 m | |
| 虛擬軌跡 (B) / 車停著沒開 | **321 m** | **145.1 m** | |

- 目標來自偵測時，**一律在真實 ~415 m 推頭**；虛擬軌跡遇到車慢 / 車沒開，推頭點就跑掉。
- 虛擬軌跡模式的數字與 B 的 `xwing_virtual_sim.py` 相同 (−28.8°、80.0 m)。
- ⚠ 移動的車 miss 約 50~80 m (打在車後)：原專案純追擊、沒有前置瞄準 (`MOVING_TARGET_LEAD = False`) 的特性，B 也一樣。
- 雲台 pitch 偏 1° → 推頭點差約 17 m，**實飛前一定要校正 `GIMBAL_PITCH_OFFSET_DEG`**。

這些是模擬結果 (飛機模型同 B 的估算)；**實機、實際雲台都還沒測過**。

## 7. 上機前要做的

1. 相機彈射後不失焦 (對方處理)。
2. 地面校正：飛機水平、車放正前方已知距離 D，追蹤模式讓車置中，比較 log 的
   `vg_ground_m` 與 D，調 `GIMBAL_PITCH_OFFSET_DEG`；看 `vg_bearing_off_deg` 是否 ≈ 0，調 `GIMBAL_YAW_OFFSET_DEG`。
3. 掛機整合：`python3 xwing_vision_flight.py --bench`，接對方雲台。
4. 第一次實飛建議先 `--virtual-control` (= B)，同時記錄偵測欄位，飛完跟車子 GPS 比對再切換。
