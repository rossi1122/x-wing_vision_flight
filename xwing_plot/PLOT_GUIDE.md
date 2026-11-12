# xwing_vision 飛完畫圖 — 使用說明

飛完後一個指令出全部圖、一份 PDF，加上一份 `summary.txt`（時間線、各段數字、哪些參數可能要調）。
Fig 1~9 照學長 realflight_plot 的版面，Fig 10~15 是 X-wing 才有的診斷圖，Fig V1~V5 是這個專案的目標偵測對照，改寫成讀 **xwing_vision 的 log**（B 的 xwing_virtual log 也能畫）。

畫圖相關的檔案全部放在專案的 **`xwing_plot/`** 資料夾，指令都在這個資料夾裡執行：

```
x-wing_vision_flight/         (Orin 與筆電同名)
  flight_logs/                ← 飛機 log (飛行程式寫在這; 筆電上把 Orin 的拷過來放這)
  xwing_vision_config.py     ← 畫圖會讀這裡的參數 (所以要拷整個專案)
  dive_guidance.py ...        ← Fig 5 / Fig 15 重放用
  xwing_plot/
    xwing_plot_report.py      主程式
    xwing_plot_target_gps.py  虛擬目標 vs 真車 (主程式會自動呼叫, 也可單獨跑)
    xwing_plot_vision.py      目標偵測對照 Fig V1~V5 (主程式會自動呼叫)
    plot_common.py            共用: 找 log 檔
    test_plot_report.py       不用飛機的測試
    PLOT_STEPS.txt            筆電照抄步驟
    PLOT_GUIDE.md             本說明
    car_bin/                  ← 車子 .BIN 放這裡 (有 BIN 就自動比對真車)
    plot_results/             ← 輸出 (自動產生)
```

| 程式 | 對應學長的 | 做什麼 |
|---|---|---|
| `xwing_plot_report.py` | `plot_flight_report*.py` / `plot_moving_report*.py` | 主報告：Fig 1~15 + report.pdf + summary.txt |
| `xwing_plot_target_gps.py` | `plot_target_gps_comparison.py` | 虛擬目標 vs 真車 GPS（BIN），可單獨跑 |

學長原本那幾支**不能直接讀 X-wing 的 log**：它們需要 `_vision.csv`（機上 YOLO）、`real_*` / `real_comp_pid_*` 欄位、`_raw.mp4`（`redetect_yolo.py`），X-wing 這版沒有 YOLO 也沒錄影，所以沒有放進來。

只需要 `numpy` + `matplotlib`（不用 pandas、pymavlink、ROS）。

---

## 1. 飛測前先確認（Orin 或筆電）

```bash
cd ~/catkin_ws/src/x-wing_vision_flight/xwing_plot
python3 -c "import numpy, matplotlib; print('OK')"
python3 test_plot_report.py --vision     # 不用飛機: 模擬一趟 + 假雲台 + 假車子 BIN, 再自動畫圖
```
- 印出 `OK`、最後出現 `[test] 完成` 就可以。沒有套件的話（需要網路）：`pip3 install numpy matplotlib`
- 在筆電畫：拷**整個專案**（不能只拷 `xwing_plot/`），指令一樣，Windows 把 `python3` 換成 `python`。筆電步驟見 `PLOT_STEPS.txt`。
- 測試結果在 `xwing_plot/plot_results/xwing_vision_<時間>_plottest_gimbal/run1/report.pdf`。模擬的飛機模型很粗，圖裡的數字不代表實飛結果。

---

## 2. 飛完後怎麼畫（照抄）

```bash
cd ~/catkin_ws/src/x-wing_vision_flight/xwing_plot
python3 xwing_plot_report.py --all          # ★ ../flight_logs/ 每一份、每一趟都畫, 最後印總表
python3 xwing_plot_report.py --latest       # 只畫最新那份
python3 xwing_plot_report.py <檔名.csv> ...  # 只畫指定的 (只給檔名, 會去 flight_logs 找)
```

**真車比較（Fig 3、Fig 14、Fig 15）**：車子飛控 SD 卡 `LOGS/` 裡**當天**的 `.BIN` 放進 `xwing_plot/car_bin/`，就會**自動**比對（程式用 GPS 時間自己挑對的那一個）。不要比對就加 `--no-car`；BIN 放在別的地方用 `--car-bin-dir <資料夾>`。

其他選項：
```bash
python3 xwing_plot_report.py <log.csv> --run 2     # 只畫第 2 趟
python3 xwing_plot_report.py <log.csv> --full      # 地面測試 / 沒有 GUIDED: 整份畫出來
```

一份 log 裡切了幾次 GUIDED（`run` 欄位 1、2、3…），就會**每一趟各出一個資料夾**。

---

## 3. 輸出在哪、給老師看什麼

```
xwing_plot/plot_results/<log 檔名>/run1/
    report.pdf          ← 全部圖合成一份，直接開給老師看
    summary.txt         ← 時間線 + 數字 + 自動檢查（⚠ = 建議看的參數）
    fig1_overview.png ... fig12_gimbal_replay.png
```
`summary.txt` 的內容也會直接印在終端機上。

所有圖的時間軸：**0 s = 撥 GUIDED 的那一刻**。底色：藍 SEARCH、黃 PRE_DIVE、綠 DIVE、紫 PULLOUT、灰 DONE、紅 ABORT。

---

## 4. 每張圖在看什麼 → 對應哪些參數

| 圖 | 內容 | 不對勁時看的參數 |
|---|---|---|
| **Fig 1~9：照學長 realflight_plot 的版面（論文用）** | | |
| Fig 1 影像誤差 | DIVE 中虛擬目標的橫向 / 縱向誤差，單位「半個畫面 %」（±100 % = 畫面邊緣） | `PID_LAT_KP/KI`、`PID_LON_KP/KI/KD` |
| Fig 2 姿態 | pitch 命令 vs 實際、航向（相對進 DIVE 時的固定方位）、roll vs 飛控 roll 目標 | 飛控 pitch 迴路（追不上 → 先調飛控）、`GUIDED_P`、`ROLL_PROT_*` |
| Fig 3 高度剖面 | (a) AGL vs 時間；(b) 慣性座標側視（0 = 進 DIVE 時的目標，目標往右跑），理想 30° 線、DIVE 結束 pitch 延伸線、有車子 BIN 時加真車；(c) DIVE 中飛機到目標（和真車）的水平距離 | `DIVE_ANGLE_DEG`、撥 GUIDED 的高度 / 距離 |
| Fig 4 下滑角 | 航跡角 γ vs 理想 −30°、下滑角誤差 | `ALPHA_DIVE_DEG`、pitch 追蹤 |
| Fig 4b 雲台偵測率 | 各階段雲台偵測率長條圖（有雲台才有） | 雲台 / 對方辨識 |
| Fig 5 PID 重放 | 同一套 PID：虛擬目標 vs 雲台資料的輸入誤差、重算的航向 / pitch 命令（DIVE 有雲台資料才有） | 見第 5.1 節 |
| Fig 6 空速 | 空速 + 各階段平均線與統計表 | `PULLOUT_VMAX_MPS`、`THR_MIN` |
| Fig 7 控制迴路誤差 | DIVE 中影像誤差 → 姿態追蹤誤差 | 哪一段迴路跟不上 |
| Fig 8 雲台姿態 | 雲台 pitch / roll / 模式（有雲台才有）；v8 多一格 (d) 雷射距離 vs 到虛擬目標的斜距 | 有沒有在 PRE_DIVE 鎖到 −30°；雷射有沒有量到 |
| Fig 9 DIVE 軌跡 | DIVE 飛機與目標俯視軌跡（目標方向 = Y 軸，每秒畫機頭箭頭） | 推頭時側偏、方向舵修正 |
| **Fig 10~15：X-wing 才有的診斷圖** | | |
| Fig 10 方向舵 | raw / cmd / sent、±限幅、被空速或 roll 限制的區間、偏航角速度 | `K_LAT_TO_RUDDER`、`RUDDER_LIMIT_DEG`、`RUDDER_SLEW_DPS`、`ROLL_PROT_*` |
| Fig 11 PID 各項 | DIVE 中 P / I / D 各自的大小與輸出 | 哪一項主導；I 一直累積 = 有固定偏差 |
| Fig 12 預測落點 | DIVE 中依當下速度直線外推的落點誤差 | 側向收斂 = 方向舵在作用 |
| Fig 13 系統 | 電壓、控制迴路週期（20 ms = 50 Hz） | Orin 卡頓、電池 |
| Fig 14 真車 GPS | 同一時刻虛擬目標 vs 真車的位置差、車速 vs 10 m/s（car_bin 有 BIN 才有） | 車子出發時機、撥 GUIDED 時機 |
| Fig 15 雲台 vs 真車 | 雲台 / 虛擬 / 真車三者拆解、偵測與延遲（DIVE 有雲台資料才有） | 雲台量得準不準（見第 5.1 節） |
| **Fig V1~V5：目標偵測對照（xwing_vision 的 log 才有，見第 5.2 節）** | | |
| Fig V1 看車俯角 vs 推頭 | 偵測到的看車俯角、觸發角、實際推頭；「角度 / 距離判斷會在什麼時候推頭」；偵測水平距 vs 虛擬 vs 真車 | `GIMBAL_PITCH_OFFSET_DEG`、`PREDIVE_TRIGGER_ANGLE_DEG` |
| Fig V2 地圖 | 飛機航跡（依模式上色）、虛擬目標、偵測到的目標位置、真車；東西向放大 | 撥 GUIDED 位置、真車路線 |
| Fig V3 偵測位置誤差 | 偵測 − 真車（有 BIN）/ 偵測 − 虛擬目標，拆成沿路與橫向 | 沿路偏 → 俯角；橫向偏 → 方位 (`GIMBAL_YAW_OFFSET_DEG`) |
| Fig V4 航向 | 實際航向命令、朝虛擬目標、朝偵測目標的方位 | 視覺控制時 SEARCH 航向會差多少 |
| Fig V5 偵測與資料鏈 | 看不看得到、UART 連線、影像延遲、雲台 yaw − 飛控航向 | 鏡頭 / 辨識、資料鏈、`GIMBAL_YAW_MODE` |

---

## 5. summary.txt 的自動檢查（門檻）

| 檢查 | ⚠ 的條件 | 提示的方向 |
|---|---|---|
| 撥 GUIDED 高度 | 跟 240 m 差 > 20 m（AGL < 20 m 視為地面測試，不檢查） | Mission Planner 航點高度 |
| 推頭前航向誤差 | 推頭前 2 s 平均 > 5° | 撥 GUIDED 太晚 / `GUIDED_P` |
| 推頭用時 | PRE_DIVE > 4 s | 飛控 pitch 角速度上限 / pitch 增益、`PREDIVE_HANDOFF_PITCH_TOL_DEG` |
| 俯衝角 | DIVE 平均 γ 跟 −30° 差 > 3° | pitch 追蹤、`ALPHA_DIVE_DEG` |
| pitch 追蹤 | DIVE RMSE > 3° | 飛控 pitch 迴路 |
| 空速 | ≥ 55 m/s 提醒；到 60 = 超速提前拉平 | 俯衝角、`THR_MIN`、撥 GUIDED 高度 |
| 影像誤差 | 擺盪（過零 > 0.8 Hz 且 RMSE > 1°）/ 後半段一直偏 > 1.5° / RMSE > 5° | 擺盪 → `KP` 太大；一直偏 → `KI` 不夠或方向舵 / pitch 不夠力 |
| 方向舵 | 打滿 > 20 % 時間；被 roll 保護限制 > 10 % | `K_LAT_TO_RUDDER`、`ROLL_PROT_*` |
| roll | DIVE 中 > 25° | 方向舵–滾轉耦合 |
| 拉平 | 最低 AGL < 100 m（✗）；比 120 m 多掉 > 15 m（⚠） | `PULLOUT_AGL_M` |
| 控制迴路 | > 40 ms 的拍數 > 2 %，或單次 > 200 ms | Orin 負載 |
| 電壓 | 讀到 ≈ 0 V 超過 5 % | 第二顆電池監測沒接 |
| 雲台 | UART 連線 < 95 %；PRE_DIVE 後一直沒鎖定到位 | 線材、對方程式 |

這些只是**根據 log 的提示**，判斷前還是要看圖。數字和圖要一起看，例如「影像誤差一直偏」要再看 Fig 10 方向舵是不是已經打滿。

### 5.1 Fig 5 / Fig 15：雲台資料能不能當控制命令

**做法**
1. 雲台模型：pitch / roll **相對地面**（自穩），yaw 跟機頭（沒有 yaw 軸）。
2. 每一筆雲台偵測：用「取像那一刻」（`vis_cap_age_ms`）的飛機航向 / pitch，把目標方向轉到 DIVE 用的虛擬相機（機身固定、光軸 = 機頭），得到虛擬畫面上的 (u, v)。
3. 用同一套 `dive_guidance.DivePID`、同樣的增益和 setpoint，**離線重跑** DIVE 段，算出 pitch 命令和方向舵（= `K_LAT_TO_RUDDER` × dpsi）。
4. 同時用「虛擬量測」再重跑一次當**重放檢查**：這條應該跟實際命令完全重合（差 < 0.5°），代表重放是對的。
5. 有車子 BIN 時，再用真車 GPS 算「車子理論上在虛擬畫面的哪裡」（綠虛線），把差異拆成兩部分：
   - **雲台 − 真車**＝雲台本身量得準不準（偏差、跳動、延遲、座標假設）
   - **真車 − 虛擬目標**＝車子沒有開在虛擬軌跡上（跟 Fig 14 同一件事）

**圖（由上到下）**：橫向偏移、縱向偏移（藍＝虛擬，橘點＝雲台，綠虛線＝真車）、pitch 命令、方向舵命令、雲台偵測與影像延遲。

**summary.txt 的【雲台資料當控制命令】判斷**
| 項目 | ✓ 條件 |
|---|---|
| 重放檢查 | 虛擬量測重放 vs 實際：pitch < 0.5°、方向舵 < 1° |
| 偵測率 | DIVE ≥ 90 % |
| 影像延遲 | 中位數 ≤ 150 ms |
| 雲台 vs 真車 偏差 | 平均 ≤ 1°（有偏差 → 雲台零點 / 安裝角；縱向偏差可用 `--gimbal-pitch-offset <偏差值>` 重跑確認） |
| 雲台 vs 真車 跳動 | RMSE ≤ 2° |

⚠ **命令差是開環重放**：飛機實際是照虛擬目標飛，沒有照雲台命令飛，所以誤差不會被修掉、積分會一直累積。只看「一開始往哪個方向修、修多少」，不要把命令的 RMSE 當成控制效果。真正的控制效果要等實際用雲台閉迴路飛過才知道。

沒有給車子 BIN 時，無法分辨「雲台不準」還是「車子不在虛擬位置」，summary 會提醒把車子 BIN 放進 `xwing_plot/car_bin/`。

選項：`--gimbal-pitch-offset`（pitch 零點修正，度）、`--gimbal-pitch-sign`、`--gimbal-roll-sign`（預設 1，往下為負）。

不用飛機先試：在 `xwing_plot/` 執行 `python3 test_plot_report.py --vision`（假雲台看真車，車子比虛擬目標晚 4 s 出發）。

### 5.2 Fig V1~V5：目標偵測對照（xwing_vision）

飛行程式每一拍都把偵測結果換算好寫進 log（`vg_*` 欄位：看車俯角、水平距、目標位置、方位）。
不管是 `--virtual-control`（偵測只記錄）還是正式模式（目標來自偵測），都會畫。

**summary.txt 的【目標偵測對照】段落**
- 控制來源（虛擬軌跡 / 目標偵測）、推頭依據（angle / distance）、觸發角
- 第一次看到目標的時間與距離；SEARCH / PRE_DIVE / DIVE 看得到目標的比例
- 實際推頭那一刻的偵測俯角、偵測 / 虛擬 / 真車水平距
- **角度判斷 / 距離判斷會在什麼時候推頭**（比實際早或晚幾秒）
  - `--virtual-control` 時實際推頭看的是虛擬距離，所以真車不在虛擬位置就會差
- 偵測位置 vs 真車（中位數、95 %、沿路 / 橫向）
- 雲台校正估計（有真車 BIN 時）：俯角少算幾度、方位偏幾度 → 直接寫建議值

| 自動檢查 | ⚠ 的條件 |
|---|---|
| 看得到目標 | PRE_DIVE / DIVE < 80 %（正式模式 1.5 s 看不到就 ABORT）；SEARCH < 30 % |
| 偵測位置 vs 真車 | 中位數 > 30 m |
| 俯角偏差 | > 0.5° → 建議 `GIMBAL_PITCH_OFFSET_DEG` 加 −偏差 |
| 方位偏差 | > 1° → 建議 `GIMBAL_YAW_OFFSET_DEG` 加 偏差 |
| 雲台 yaw − 飛控航向 | 標準差 ≥ 2° → 維持 `GIMBAL_YAW_MODE = 'none'` |
| 影像延遲 | 中位數 > 150 ms |

校正估計用的是 SEARCH / PRE_DIVE、離車 80~900 m、看得到車的資料；
真實俯角用「飛控高度（相對起飛點）」算，目標區地面比起飛點高或低時估計會一起偏。

不用飛機先試：
```bash
python3 test_plot_report.py --vision                    # 偵測只記錄 (飛測計畫)
python3 test_plot_report.py --vision --vision-control   # 正式模式 (目標來自偵測)
python3 test_plot_report.py --vision --pitch-bias 1     # 假雲台俯角偏 1° → summary 應建議 −1
```

---

## 6. 常見狀況

- **Orin 時鐘是 1970**：檔名會是 `19700101_...`，但不影響畫圖。對真車用的是飛控的 GPS 時間（`gps_unix` 欄位），不是 Orin 的時鐘。所以每趟請用不同的 `--case`，才分得出是哪一趟。
- **Fig 14 沒畫**：`summary.txt` 會寫原因，常見的是：
  - BIN 沒有涵蓋 PRE_DIVE~DIVE 的時間（拿錯天、車子飛控沒錄到）；
  - 資料夾裡有兩個 BIN 都涵蓋（只留當天那個）；
  - 這趟沒有進 DIVE。
- **Fig 14 的前後差**：`summary.txt` 寫「沿目標方位的前後差」，**負 = 車子在虛擬目標後面**（車子出發晚了），往南 / 往北開都一樣。
  圖上和 `_aligned.csv` 的 `err_n` / `err_e` 仍是北 / 東向差。
- **只想單獨比對真車**：
  ```bash
  python3 xwing_plot_target_gps.py ../flight_logs/<log>.csv      (BIN 放 car_bin/)
  ```
