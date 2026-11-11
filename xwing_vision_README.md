# xwing_vision — X-wing 目標偵測版

> 跟 `hsu_realflight_true` 的 xwing_virtual **同一個場景、同一套飛法、同一組參數、
> 同一個狀態機與判斷邏輯** (240 m / 30 m/s / 俯衝 30° / 140 m 拉平 / WP3 撥 GUIDED)。
> **唯一的差別：狀態機的「目標」輸入，從虛擬軌跡換成對方雲台的目標偵測。**
>
> ★ 2026/11/11 (與老師討論後)：**推頭依據改成「看車的俯角 ≥ 30°」**，不再用水平距離 (見 §1.1)。

## 1. 一次看懂

`dive_guidance.py` (狀態機) **一行都沒改**。原本它從虛擬目標拿到這些欄位，
現在同名欄位改由偵測結果填入 (`vision_guidance.VisionSource`)：

| 欄位 | 原本 (虛擬軌跡) | 現在 (目標偵測) | 狀態機拿來做什麼 (原邏輯) |
|---|---|---|---|
| `tgt_n`, `tgt_e` | 虛擬車位置 | 偵測算出的車地面位置 (看不到時保持最後一次) | SEARCH / PRE_DIVE 航向 = 飛機指向它 (43002) |
| `predive_dist_m` | 到虛擬車水平距 | **由看車俯角換算的等效距離** (看不到 = 無；見 §1.1) | ≤ 415.69 m 就推頭 ⇔ 俯角 ≥ 30° |
| `tgt_valid` | 永遠有效 | 這拍看得到車 | PRE_DIVE / DIVE 看不到 > 1.5 s → ABORT |
| `bbox_u`, `bbox_v`, `bbox_detected` | 虛擬車投影到機身相機 | 車在畫面的位置, 經雲台角度換到同一個機身相機 | DIVE PID 修 pitch / 方向舵；進 DIVE 要看得到 |

**還沒偵測到過車** (剛撥 GUIDED) 時，目標位置先用虛擬目標 (WP4) 帶路，
但距離、畫面位置、有效旗標一律來自偵測，所以**沒看到車就不會推頭**。

### 流程 (同 xwing_virtual)
```
AUTO 240 m / 30 m/s → WP3 撥 GUIDED
→ SEARCH:   飛控 43002 朝目標位置飛, TECS 定高定速
→ 看車的俯角 ≥ 30° (11/11 起; 原本是水平距 ≤ 415.69 m) → PRE_DIVE: pitch −30°, 雲台送 '2' 鎖定 −30°
→ pitch 到 −27° 內且看得到目標 → DIVE: 目標畫面位置 PID
→ AGL ≤ 140 m 或空速 ≥ 60 m/s → PULLOUT → DONE (TECS 140 m / 30 m/s)
PRE_DIVE / DIVE 看不到目標 > 1.5 s → ABORT 拉起 (原 ABORT_TARGET_LOST_S)
```
SEARCH 段原本就沒有逾時：一直沒看到車就不會推頭，**由駕駛員切出 GUIDED**。

### 1.1 推頭依據：看車的俯角 (2026/11/11)

**看車的俯角 = 雲台往下的角度 + 車在畫面上下偏的角度** (每筆新辨識結果算一次)，
**≥ 30° 就推頭**。這個角度只用到雲台角度與畫面，**不用飛機高度**。

| | 角度判斷 (現在, `PREDIVE_TRIGGER='angle'`) | 距離判斷 (原本, `'distance'`) |
|---|---|---|
| 條件 | 看車俯角 ≥ 30° | 高度 ÷ tan(俯角) ≤ 415.69 m |
| 用到高度 | 不用 | 用飛控相對起飛點高度 |
| 實際高度 = 240 m | 同一個時間點 | 同一個時間點 |
| 實際高度 ≠ 240 m | 推頭時視線仍是 30° | 視線偏離 30° (220 m → 28°；260 m → 32°) |
| 目標區地面比起飛點高 / 低 | 30° | 也是 30° (高度與門檻的誤差互相抵銷)，只是 log 的距離數值會錯 |
| 雲台俯角偏 1° | 推頭時機差 1° | 一樣差 1° |

**狀態機沒改**：`dive_guidance.py` 仍然比「`predive_dist_m` ≤ 415.69 m」，
`VisionSource` 改餵「等效距離」：

> 等效距離 = 415.69 × tan(30°) ÷ tan(看車俯角)

看車俯角 ≥ 30° 時等效距離 ≤ 415.69 m，所以結果就是「俯角 ≥ 30° 推頭」。
因此狀態機自己印的「到目標水平距 XXXm ≤ 416m」在 angle 模式是**等效距離**；
真正的看車俯角和水平距請看主程式的 MODE 橫幅 (`看車俯角=30.1° 偵測水平距=415m`) 與 CSV。

- 切回原本：`--trigger distance`，或 config `PREDIVE_TRIGGER = 'distance'`
- 改觸發角：`--trigger-angle 32`，或 `PREDIVE_TRIGGER_ANGLE_DEG` (預設 = 俯衝角 30°)
- CSV：`vg_depress_deg` (看車俯角)、`vg_ground_m` (水平距)、`vg_trigger`、`vg_trigger_angle_deg`、`vg_predive_in_m` (餵給狀態機的值)
- 比較兩種做法：`python3 xwing_vision_sim.py --trigger-compare --car stopped`

| 情況 (模擬, 車停著) | 角度：推頭時真實視線 | 距離：推頭時真實視線 |
|---|---|---|
| 高度 240 m | 30.1° | 30.1° |
| 高度 220 m | **30.2°** (377 m 推頭) | 28.0° (414 m 推頭) |
| 高度 260 m | **30.2°** (446 m 推頭) | 32.2° (414 m 推頭) |
| 目標區地面低 15 m | 30.2° | 30.2° |
| 目標區地面高 15 m | 30.1° | 30.1° |

## 2. 對方雲台資料 (2026/11 確認)

| 項目 | 內容 | 程式怎麼用 |
|---|---|---|
| `target_cx/cy` | 車在畫面的像素；沒有車 = NaN | NaN → 這拍看不到 |
| `fx/fy/cx/cy` | 相機內參 (每筆都送) | 像素 → 角度 |
| `pitch` | **對地**角度，往下為負，**拍照那一刻**的值 | 直接當相機俯仰 |
| `roll` | 相機 roll，雲台維持 0° | 直接使用 |
| yaw | **沒有 yaw 軸**，相機朝機頭 | 相機方位 = 飛機 yaw (+ 安裝偏差) |
| `yaw` 欄位 (新格式) | **對地**角度；雲台 yaw 保持置中，相機一直朝機頭 (2026/11/10 確認) | 不拿來算 (`GIMBAL_YAW_MODE='none'`，用飛控航向)；記錄到 `vis_gimbal_yaw`，並和飛控航向比對 `vg_yaw_diff_deg` |
| `cap_mono` (v7/v8/新格式) | 取像時刻 (UART 校時換到本機) | 查拍照那一刻的飛機航向 |
| 雷射 (v8/新格式) | 不準 | 只記錄 |

### 2.1 UART 協定：共用接收端 V6 (2026/11/09)
`uart_mipi_receiver.py` 換成對方新給的「共用雲台接收端 V6」(台鴞／田屋 30 倍／田屋 10 倍)。

| 版本號 | 資料區 | 完整封包 | 內容 |
|---|---|---|---|
| 舊 6 | 61 B | 71 B | 基本欄位 |
| 舊 7 | 73 B | 83 B | + 取像時間戳 (校時) |
| 舊 8 | 77 B | 87 B | + 雷射距離 |
| **新 0~5 / 10** | **81 B** | **91 B** | **+ 雲台 yaw** |

- 校時請求要用**對方的版本號**送 (新格式)，舊格式維持送 7；`xwing_uart_vision.py` 已自動處理。
- 舊格式照樣收得到，yaw 記成空白。
- **雲台 yaw 怎麼用** (`xwing_vision_config.py` 的 `GIMBAL_YAW_MODE`)：

  | 值 | 意義 | 相機方位 |
  |---|---|---|
  | `'none'` (預設) | 不用 yaw (= 先前確認的「沒有 yaw 軸」) | 飛機 yaw |
  | `'body'` | yaw = 相機相對機頭 (右為正) | 飛機 yaw + yaw |
  | `'earth'` | yaw = 相機真方位 | yaw |

  yaw 收到 NaN 時一律當 `'none'`。正負號相反就設 `GIMBAL_YAW_SIGN = -1`。

  **目前設定 `'none'` 的理由** (11/10 確認 yaw 對地、相機朝機頭)：
  這時雲台 yaw 和飛控機頭航向是**同一個方向**，兩個來源理論上一樣。
  飛控航向是 EKF 融合 GPS／羅盤的結果，而且和目標位置、導航用同一個基準，所以優先用它。
  雲台 yaw 的北方基準 (磁北／真北／開機方向) 和飄移量都還沒驗證過。

  **怎麼驗證**：看 CSV 的 `vg_yaw_diff_deg` (= 雲台 yaw − 取像那一刻的飛控航向，每筆新封包都算，有沒有車都記)。
  - 差值接近固定、抖動小 → 雲台 yaw 可信，固定的差值是磁北／真北差或安裝偏差。
  - 差值會飄、轉彎時跳動 → 維持 `'none'`。

水平距 = 高度 ÷ tan(看車的俯角)；看車的俯角 = 雲台往下角度 + 車在畫面上下偏的角度。

## 3. 檔案

| 檔案 | 狀態 | 內容 |
|---|---|---|
| `xwing_vision_flight.py` | 新增 | 實飛主程式。= B 的 `xwing_virtual_flight.py`，只把目標輸入換成 `VisionSource` |
| `xwing_vision_config.py` | 新增 | 參數 = B 的 `xwing_virtual_config.py` (11/04 版)，另加相機校正值與資料過期時間 |
| `xwing_vision_common.py` | 新增 | = B 的 `xwing_virtual_common.py` (狀態機參數相同) |
| `vision_guidance.py` | 新增 | `VisionSource`：偵測結果 → 與虛擬目標同名的欄位 |
| `uart_mipi_receiver.py` | 對方 11/09 新版 (未改) | 共用雲台接收端 V6 (新格式 0~5 / 10 + 舊 v6/v7/v8) |
| `xwing_uart_vision.py` / `xwing_uart_sender_sim.py` | 從 B 複製，改成支援新格式 | 背景接收 (新增 `vis_gimbal_yaw`)、模擬發送端 (預設送 v10，`--yaw-deg`) |
| `vision_sim_camera.py` / `xwing_vision_sim.py` | 新增 | 桌上閉迴路模擬 (真車 + 模擬對方雲台) |
| `test_vision_guidance.py` | 新增 | 71 項測試 |

`dive_guidance.py`、`attitude_bridge.py`、`run_vision_flight.py` 等原檔**都沒有改**。

## 4. 新增的參數 (`xwing_vision_config.py`)

「目標怎麼從偵測算出來」需要的值，以及推頭依據的切換：

| 參數 | 值 | 意思 |
|---|---|---|
| `VISION_PKT_MAX_AGE_S` | 0.3 s | 最新封包超過這麼久沒更新 → 這拍算看不到 (資料鏈斷) |
| `GIMBAL_PITCH_OFFSET_DEG` | 0 | ★ 地面校正：光軸實際俯角 − 回報值 |
| `GIMBAL_YAW_OFFSET_DEG` | 0 | ★ 地面校正：光軸偏機頭右方幾度 |
| `POSE_MAX_GAP_S` | 0.15 s | 查拍照那一刻的飛機姿態，前後樣本最遠差距 |
| `PREDIVE_TRIGGER` | `'angle'` | ★ 推頭依據：`'angle'` 看車俯角 / `'distance'` 偵測水平距 (原本) |
| `PREDIVE_TRIGGER_ANGLE_DEG` | None (= 30°) | angle 模式的觸發角 |
| `GIMBAL_YAW_MODE` | `'none'` | 雲台 yaw 只記錄，不參與計算 |

其他 (ABORT 1.5 s、啟動檢查、雲台模式鍵…) 全部 = B。

## 5. 執行

```bash
roslaunch mavros apm.launch
python3 xwing_vision_flight.py                    # 目標來自偵測 (預設)
python3 xwing_vision_flight.py --vision-port /dev/ttyUSB0
python3 xwing_vision_flight.py --trigger distance # 推頭依據改回偵測水平距 (原本)
python3 xwing_vision_flight.py --virtual-control  # 目標來自虛擬軌跡 (= B, 偵測只記錄)
python3 xwing_vision_flight.py --bench            # 掛機
```

開機印 `目標來源: ★ 目標偵測`。每秒狀態列最後一段：
`| VIS link=1 det=1 dep=27.9° dist=452m gmode=1(req 1) gpitch=-27.9`
開機也會印 `推頭門檻: 看車俯角 ≥ 30.0° (不用高度)`。

主 CSV：B 的欄位全部保留，另加 `control`、`abort_reason`、`vg_*`
(這拍看不看得到、原因、方位、偏機頭、俯角、水平距、車的 NED / GPS 估計)、`gcomp_u/v`。

## 6. 測試結果 (桌上)

```bash
python3 test_vision_guidance.py     # → PASS (71 項)
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
- 雲台 pitch 偏 1° → 推頭點差約 17 m (角度判斷也一樣差 1°)，**實飛前一定要校正 `GIMBAL_PITCH_OFFSET_DEG`**。
- 上表在 240 m 飛，角度判斷與距離判斷結果相同；高度不是 240 m 的比較見 §1.1。

這些是模擬結果 (飛機模型同 B 的估算)；**實機、實際雲台都還沒測過**。

## 7. 上機前要做的

1. 相機彈射後不失焦 (對方處理)。
2. 地面校正：飛機水平、車放正前方已知距離 D，追蹤模式讓車置中，比較 log 的
   `vg_ground_m` 與 D，調 `GIMBAL_PITCH_OFFSET_DEG`；看 `vg_bearing_off_deg` 是否 ≈ 0，調 `GIMBAL_YAW_OFFSET_DEG`。
3. 掛機整合：`python3 xwing_vision_flight.py --bench`，接對方雲台。
4. 第一次實飛建議先 `--virtual-control` (= B)，同時記錄偵測欄位，飛完跟車子 GPS 比對再切換。
