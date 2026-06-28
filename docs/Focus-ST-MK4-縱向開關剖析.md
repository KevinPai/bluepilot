# Ford Focus ST MK4：BluePilot 縱向控制三開關組合深度剖析

> **分析對象**：2020 Ford Focus ST MK4（C519 平台，platform = `FORD_FOCUS_MK4`）
> **三顆開關**：Bypass BP Longitudinal Control、OP Long、Experimental Mode
> **程式碼基準**：BluePilot 分支 `bp-6.0`（sunnypilot / openpilot 衍生）
> **分析方法**：原始碼靜態分析（多 agent 讀碼 + 一致性審查 + 人工驗證）
> **撰寫日期**：2026-06-27

---

## 摘要（TL;DR）

1. **「Bypass BP Long」不是把縱向交回福特原廠。** 交回原廠的唯一開關是 **OP Long**。`disable_BP_long_UI` 的所有程式碼都巢狀在 `if self.CP.openpilotLongitudinalControl`（`carcontroller.py:721`）之內，OP Long 一關它就是 **no-op**。
2. **「Bypass BP Long」實質只是高速巡航微調開關**：只在 >50 mph 速度死區、未踩油煞、且（無前車或前車 >40 mph）時才生效；**塞車低速 stop-and-go 時開或關完全沒差別**。
3. **三顆開關不互斥、不會跳 ACC 故障。** 它們位於縱向管線的不同層級（總開關 / 規劃層 / 致動層），只受 OP Long 一個總開關 gating。
4. **「只開 Experimental」設不起來**：OP Long=OFF 時該開關在 UI 會被灰掉並從 Params 移除。
5. **變速箱才是 stop-and-go 的生死線**：手排 Focus ST → `minEnableSpeed≈32 km/h`、`autoResumeSng=False` → 走停不可行；只有自排才談得上完整 stop-and-go。
6. **openpilot 路徑沒有「3 秒」**：「停等 3 秒需手動」是福特原廠 ACC（OP Long=OFF）的概念；openpilot（OP Long=ON）是「前車一走、`shouldStop` 一清除就起步」。

---

## 目錄

1. [三顆開關的真實定義與層級](#1-三顆開關的真實定義與層級)
2. [三個必須糾正的核心觀念](#2-三個必須糾正的核心觀念)
3. [變速箱：凌駕三顆開關的決定性因素](#3-變速箱凌駕三顆開關的決定性因素)
4. [共通底層機制](#4-共通底層機制)
5. [六種組合逐一剖析](#5-六種組合逐一剖析)
6. [跨組合比較表](#6-跨組合比較表)
7. [給 Focus ST MK4 車主的建議](#7-給-focus-st-mk4-車主的建議)
8. [重要提醒與免責](#8-重要提醒與免責)
9. [附錄 A：程式碼證據索引](#附錄-a程式碼證據索引)
10. [附錄 B：無法由程式碼確認的事項](#附錄-b無法由程式碼確認的事項)

---

## 1. 三顆開關的真實定義與層級

| 俗稱 | UI 名稱 | Param | 定義位置 | 作用層級 |
|---|---|---|---|---|
| Bypass BP Long | Bypass BP Longitudinal Control | `disable_BP_long_UI` | `selfdrive/ui/bp/layouts/settings/bluepilot.py:364-371` | **致動層**（Ford carcontroller 內部子開關） |
| OP Long | sunnypilot Longitudinal Control (Alpha) | `AlphaLongitudinalEnabled` | `selfdrive/ui/layouts/settings/developer.py:74-80` | **總開關**（決定縱向歸 openpilot 還是原廠） |
| Experimental Mode | Experimental Mode | `ExperimentalMode` | `selfdrive/ui/layouts/settings/toggles.py:55-60` | **規劃層**（是否併入 E2E 神經網路輸出） |

層級關係：

```
OP Long (AlphaLongitudinalEnabled)
  │  card.py:100/112 → Ford interface.py:75-77
  │  → ret.openpilotLongitudinalControl = True
  │
  └─ openpilotLongitudinalControl == True 時，以下才有意義：
       ├─ Experimental Mode  → 規劃層：longitudinal_planner.py:163-167 是否採用 E2E
       └─ Bypass BP Long     → 致動層：carcontroller.py:780/895 是否套用 BP 高速微調
```

重點補充：

- **「OP Long」帶 `DEVELOPMENT_ONLY` 旗標**（`params_keys.h:40`），release 分支會被隱藏並移除該 param（`developer.py:120-125`）——屬實驗性 alpha，非長期穩定保證。
- **「Experimental Mode」可用與否，在 Ford 上直接由 OP Long 決定**：因 Ford `alphaLongitudinalAvailable=True`（`interface.py:70`），`ui_state.has_longitudinal_control = AlphaLongitudinalEnabled`（`ui_state.py:188-191`）。

---

## 2. 三個必須糾正的核心觀念

### 糾正 #1：「Bypass BP Long」≠ 把縱向交回福特原廠

- 交回原廠的唯一開關是 **OP Long=OFF**——此時 `interface.py:75-77` 不會設 `openpilotLongitudinalControl=True`，`carcontroller.py:721` 整段縱向區塊不執行，openpilot 完全不送油門/煞車。
- `disable_BP_long_UI` 的兩處使用（`carcontroller.py:780`、`895`）都在 721 之內，OP Long=OFF 時是純 no-op。
- OP Long=ON 時，它只決定**高速（>50 mph）**跟車是否用 `bp_accel/bp_gas` 取代上游 `op_accel/op_gas`；而且兩條 fallback 路徑（`carcontroller.py:902-906` 與 `911-916`）輸出**完全相同**。

### 糾正 #2：「Bypass BP Long + OP Long」不矛盾、不會跳 ACC 故障

- OP Long=ON 時 openpilot 全權縱向、福特原廠 ACC 已退出，不存在兩套訊號並送衝突。
- Bypass=ON 只是走 `carcontroller.py:911` 的 else 直接用 `op_*`。
- 唯一與 cruise fault 相關的是 `INACTIVE_GAS=-5.0` quirk（`carcontroller.py:740-741`、`values.py:48`）：不用油門時必須送 -5.0 否則 PCM 報 fault，這是**正常設計**而非開關衝突。

### 糾正 #3：「只開 Experimental、不開 OP Long」對縱向完全無效，且實務上設不起來

三層 gating 證據：

1. **UI 層**：`toggles.py:178-183` 在 `has_longitudinal_control=False` 時 `set_enabled(False)` + `set_state(False)` + `params.remove("ExperimentalMode")`，並顯示提示「Enable the sunnypilot longitudinal control (alpha) toggle to allow Experimental mode.」
2. **開機清理**：`selfdrived.py:117-118` 在 `not openpilotLongitudinalControl` 時移除 `ExperimentalMode`。
3. **執行期**：`selfdrived.py:597` 與 `card.py:321` 都是 `experimental_mode = params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl`，OP Long=OFF 時恆為 False → `is_e2e` 恆 False → E2E 分支（`longitudinal_planner.py:163-167`）永不執行。

---

## 3. 變速箱：凌駕三顆開關的決定性因素

程式於執行期偵測變速箱（`interface.py:109-132`）：

| | 自排 Focus | 手排 Focus |
|---|---|---|
| 偵測條件 | shiftByWire ECU / 主匯流排含 `0x5A` / docs | 以上皆否 |
| `minEnableSpeed` | -1（CarSpecs 預設，不覆寫） | **20 mph ≈ 8.94 m/s ≈ 32 km/h** |
| `autoResumeSng`（= `minEnableSpeed == -1`） | **True** | **False** |
| 低速可接管？ | 可下探至靜止 | **約 32 km/h 以下無法 engage**（`events.py:79` belowEngageSpeed） |
| 靜止自動起步？ | 可 | **不可**，須踩離合/油門 |
| stop-and-go | **可用**（須 OP Long=ON） | **本質不可行** |

> ⚠️ **2020 Focus ST MK4 在多數市場為 6 速手排（部分市場選配 7AT）。** 若是手排，本文件所有「免手動自動起步」結論**一律不成立**。
>
> repo 內**沒有**能事先區分手排/自排的指紋，只能由實車 ECU 在執行期判定。`values.py` 僅標 "Ford Focus 2018 / mHEV"，無 ST / 6MT / 7AT 字樣。

---

## 4. 共通底層機制

### 4.1 OP Long=ON 時，gas/brake 如何產生

- `op_accel = op_gas = actuators.accel`（`carcontroller.py:723-724`，油門與煞車共用同一目標，accel 專用於煞車）。
- 低速 creep 補償（`carcontroller.py:79-83, 730`）：依 `v_ego` 在 1~3 m/s 間內插 0.6~0 的補償。
- 煞車 jerk 速率限制 **3.5 m/s³**（`carcontroller.py:732-734`，註解自承「仍可能 overshoot」）。
- 夾擠常數（`values.py:45-48`）：`ACCEL_MAX=2.0`、`ACCEL_MIN=-3.5`、`MIN_GAS=-0.5`、`INACTIVE_GAS=-5.0`。
- gas/brake 互斥（`carcontroller.py:919-920`）：`if brake_actuate: gas = INACTIVE_GAS`。

### 4.2 「停等後誰起步」的真實機制（OP Long=ON）

1. 跟停 → 進入 `LongCtrlState.stopping`，`output_accel` 以 `stoppingDecelRate` 緩降到 `stopAccel` 並**持續送煞車請求**保壓（`longcontrol.py:75-80`）。
2. 起步條件：`starting_condition = not should_stop and not cruise_standstill and not brake_pressed`（`longcontrol.py:18-21`）。
3. 自動 resume：`CC.cruiseControl.resume = enabled and cruiseState.standstill and not shouldStop`（`controlsd.py:172`）。
4. **沒有固定 3 秒**——前車一離開、`shouldStop` 一清除就起步，由 `autoResumeSng` 把關。
   > 「停等 3 秒需手動」是**福特原廠 ACC（OP Long=OFF）**的概念，不是 openpilot 的。

### 4.3 靜止保壓不是原廠 EPB 自己鎖死

- Ford 安全層（`ford.h`）**完全沒有 EBB/EPB 代碼**。
- 是 openpilot 持續送 `AccBrkTot_A_Rq` + `brake_actuate`（`AccBrkDecel_B_Rq`）+ `precharge`（`AccBrkPrchg_B_Rq`）位元（`fordcan.py:148/157/158`），由福特 PCM 執行致動。`stopping` 位元（`AccStopStat_B_Rq`）只是「通知」PCM 進入停車保持。

### 4.4 ICBM 沒有 RESUME

- ICBM 只能模擬「加速/減速」鈕（`CcAslButtnSetIncPress` / `DecPress`，`icbm.py:20-23`），**沒有 RESUME**。
- 因此 OP Long=OFF（原廠 ACC）時，openpilot 無法替你從靜止重新起步。

### 4.5 Experimental Mode（E2E）做什麼

- 在 `is_e2e=True` 時把模型輸出併入規劃：`output_a_target = min(e2e, mpc)`、`shouldStop = e2e OR mpc`（`longitudinal_planner.py:163-167`）。
- 效果是讓車「**更保守**」（更小加速、更易停），**不會**比 MPC 更積極。
- 額外能力：(a) 無前車彎道預判減速；(b) 無前車紅燈/停止線/靜止橫向車流停車。Chill（一般）模式兩者皆無。

---

## 5. 六種組合逐一剖析

> 每種組合統一以下結構：**控制權歸屬 / 過彎減速 / 紅綠燈與路口 / Stop & Go / Focus 風險與推薦度**。
> 推薦度分「手排 / 自排」兩值（1=最低，5=最高）。

### 組合 1｜只開 Bypass BP Long（OP Long 關、實驗關）

**前提判定：✅ 結論對，但理由錯。** 縱向交給原廠不是因為 Bypass，而是因為 OP Long=OFF；此時 Bypass 是純 no-op，本組合縱向**完全等同「三顆全關」**。

- **控制權歸屬**：油門 / 煞車 / 過彎 / 紅綠燈 / Stop&Go **全部福特原廠 ACC + PCM**；openpilot 不送任何縱向指令。BluePilot 仍提供**橫向（方向盤）**。
- **過彎減速**：無。原廠 ACC 不讀道路曲率，無前車維持設定速度進彎。
- **紅綠燈與路口**：無。原廠不辨識號誌，無前車不會停。
- **Stop & Go**：跟停舒適度＝原廠調校；停等>3 秒：自排→久停需手動 RES/點油門，手排→須踩離合；**無法完全免手動**。
- **Focus 風險與推薦度**：openpilot 端**零暴衝/急煞風險**（不送命令）。**手排 2 / 自排 4**。

### 組合 2｜只開 OP Long（Chill，Bypass 關、實驗關）

**前提判定：⚠️ 大致對，但「完全免手動起步」只對自排成立；Chill 沒有任何視覺紅燈/彎道能力。**

- **控制權歸屬**：油門/煞車由 **openpilot（Comma 標準 MPC）**，繞過原廠「需按 RESUME」限制；過彎僅被動 `limit_accel_in_turns`；紅綠燈無視覺停車；Stop&Go 自排 openpilot、手排不可用。
- **過彎減速**：弱。`is_e2e` 恆 False，無前車彎道只「不再加速」，不主動踩煞車。
- **紅綠燈與路口**：**無**。純 MPC 只看雷達前車＋cruise 假障礙。
- **Stop & Go**：自排→跟停線性（creep 補償＋3.5 m/s³ jerk 限制）、停等後 **openpilot 自動起步、完全免手動**、暴衝風險低（可能略有起步延遲）；手排→**不可用**。
- **Focus 風險與推薦度**：mHEV 煞車/回充由 PCM 分配，openpilot 無法干預；OP Long 屬 alpha。**手排 1 / 自排 3**。

### 組合 3｜OP Long + Experimental（完全體 E2E，Bypass 關）

**前提判定：⚠️ E2E 確實啟用；但「靜止 Hold 由原廠 EPB 還是 Comma」是假二選一——是 Comma 持續送煞車、PCM 執行，無原廠 EPB latch。**

- **控制權歸屬**：油門/煞車 openpilot；過彎/紅綠燈由 **E2E 主導**；Stop&Go 取決於變速箱。
- **過彎減速**：**有**。模型 `desiredAcceleration` 經 `min(e2e, mpc)` 主動降速（實車可靠度無法由程式碼保證）。
- **紅綠燈與路口**：**有視覺停車能力**（`output_should_stop_e2e`）；綠燈起步：自排免按鍵、手排不可。
- **Stop & Go**：自排→跟停保壓由 openpilot 控制、`shouldStop` 清除即自動起步、**完全免手動**、起步線性；手排→不可用。
- **Focus 風險與推薦度**：急煞 overshoot＋mHEV 手感不可控＋E2E 偵測不保證可靠（須隨時接管）。**手排 1 / 自排 4**。

### 組合 4｜Bypass BP Long + OP Long（你以為的「邏輯矛盾」）

**前提判定：❌ 不矛盾、不會跳 ACC 故障。** OP Long=ON openpilot 全權縱向、原廠已退出；Bypass=ON 只是走 `carcontroller.py:911` 的 else 直接用 `op_*`。

- **控制權歸屬**：與組合 2 幾乎相同；唯一差別是 **>50 mph 高速跟車不套用 BP 微調**。
- **過彎減速 / 紅綠燈**：**無**（Experimental=OFF，同組合 2）。
- **Stop & Go**：**與組合 2 在塞車完全相同**——低速一律 fallback 到 `op_*`，Bypass 在此無作用。
- **Focus 風險與推薦度**：與組合 2 同層級。Bypass=ON 對塞車毫無幫助。**手排 1 / 自排 3**。

### 組合 5｜只開 Experimental（Bypass 關、OP Long 關）

**前提判定：❌ 重大錯誤。** 此設定**實務上設不起來**：OP Long=OFF 時 UI 直接把 Experimental 灰掉並 `params.remove("ExperimentalMode")`。**「UI 顯示可起步卻被原廠 3 秒鎖死」的衝突在程式上不可能發生**——openpilot 此時根本沒有起步 UI 也沒有 RESUME。

- **控制權歸屬**：**縱向 100% 福特原廠 ACC**，與組合 1 **完全等價**。
- **過彎 / 紅綠燈 / Stop&Go**：openpilot 零貢獻。
- **真正風險**：**認知錯誤**——誤以為開了 Experimental 就有 E2E 能力，若因此放鬆監看會有追撞風險。
- **推薦度**：**1 / 1**（縱向等同組合 1，但易誤解，不該當獨立設定）。

### 組合 6｜Bypass BP Long + Experimental（OP Long 關或開）

**前提判定：❌ Bypass 與 Experimental 不互斥**——一個在致動層、一個在規劃層，獨立運作，只共同受 OP Long gating。

- **OP Long=OFF 子情況**：退化為組合 5/1，**縱向全原廠**。
- **OP Long=ON 子情況**：= 啟用 E2E 規劃 + 關閉 BP 高速微調 ≈ **接近原生 sunnypilot E2E 縱向**，邏輯一致、不混亂。
  - 過彎減速、紅綠燈視覺停車：**有**（同組合 3）。
  - Stop & Go：自排免手動自動起步、手排不可用；Bypass=ON 對塞車無影響。
- **推薦度**：OP Long=ON 時 **手排 1 / 自排 4**；OP Long=OFF 時退化為 **1 / 1**。

---

## 6. 跨組合比較表

| 組合 (Bypass / OP Long / Exp) | 油門·煞車控制權 | 過彎主動減速 | 紅綠燈視覺停起 | Stop&Go 免手動起步（手排/自排） | 推薦度（手排/自排） |
|---|---|---|---|---|---|
| 1. ON / OFF / OFF | 福特原廠 ACC+PCM | 無 | 無 | 不可 / 原廠久停需手動 | 2 / 4 |
| 2. OFF / ON / OFF (Chill) | openpilot | 無（僅夾上限） | 無（純 MPC） | 不可 / **可免手動** | 1 / 3 |
| 3. OFF / ON / ON (完全體) | openpilot | **有 (E2E)** | **有 (E2E)** | 不可 / **可免手動** | 1 / 4 |
| 4. ON / ON / OFF | openpilot（無 BP 微調） | 無 | 無 | 不可 / **可免手動**（市區同組合 2） | 1 / 3 |
| 5. 任意 / OFF / ON | 福特原廠（實務設不起來，縱向同組合 1） | 無 | 無 | 同組合 1 | 1 / 1 |
| 6. ON / ON / ON | openpilot E2E（≈原生 sunnypilot） | 有 (E2E) | 有 (E2E) | 不可 / **可免手動** | 1 / 4（OP 關時退化 1/1） |

---

## 7. 給 Focus ST MK4 車主的建議

- **自排車**：
  - 想要塞車免手動跟停起步 ＋ 紅燈/彎道輔助 → **組合 3（OP Long + Experimental）** 最完整。
  - 不要 E2E 視覺停車（偏好純雷達跟車）→ **組合 2**。
  - Bypass BP Long 只在 >50 mph 巡航有微調差異，市區開關無感。
- **手排車**：
  - 三顆開關對 stop-and-go 都救不了你；openpilot 的價值只剩 >32 km/h 的橫向 ＋ 巡航/彎道減速。
  - 建議 **組合 1（原廠縱向 + BluePilot 橫向）** 最穩。

---

## 8. 重要提醒與免責

1. **先確認變速箱**再談 stop-and-go：手排無自動走停、約 32 km/h 以下無法接管。
2. **OP Long 是 `DEVELOPMENT_ONLY` alpha**，release 分支會被移除，須全程準備接管。
3. **E2E 視覺停減速可靠度無法由程式碼保證**（模型權重為二進位），且只會讓車更保守、不會更積極。
4. **急煞手感有限**：3.5 m/s³ jerk 限制仍可能 overshoot；mHEV 煞車/回充由 PCM 決定，openpilot 無法干預。
5. **OP Long=OFF 時 openpilot 無 RESUME**（ICBM 只有加/減速），久停起步永遠要自己處理。
6. **Bypass BP Long 對塞車完全無作用**，別誤以為它會把縱向交回福特原廠。

> 本文件為**原始碼靜態分析**結果，非實車驗證。實車行為另受福特 PCM/韌體、雷達有效性、模型權重、指紋辨識結果等執行期因素影響。任何輔助駕駛功能皆須駕駛全程監看並隨時準備接管。

---

## 附錄 A：程式碼證據索引

> 路徑相對於 repo 根目錄。`opendbc_repo/` 為 opendbc 子模組。

### 開關定義與 gating

- `selfdrive/ui/bp/layouts/settings/bluepilot.py:364-371` — Bypass BP Long 開關定義（`disable_BP_long_UI`）
- `selfdrive/ui/layouts/settings/developer.py:74-80, 120-125` — OP Long 開關（`AlphaLongitudinalEnabled`，DEVELOPMENT_ONLY）
- `selfdrive/ui/layouts/settings/toggles.py:55-60, 173-197` — Experimental Mode 開關與 gating
- `selfdrive/ui/ui_state.py:186-191` — `has_longitudinal_control` 來源
- `common/params_keys.h:40-42, 291` — params 宣告與預設值
- `selfdrive/car/card.py:100, 112, 321` — `alpha_long` 傳入；執行期 experimental gating
- `selfdrive/selfdrived/selfdrived.py:117-118, 533, 597` — 移除/發佈/gating experimental
- `opendbc_repo/opendbc/car/ford/interface.py:70-77` — `alphaLongitudinalAvailable` / `openpilotLongitudinalControl`

### Ford 縱向核心

- `opendbc_repo/opendbc/car/ford/carcontroller.py:721` — **整段縱向控制由 `if openpilotLongitudinalControl` 守門**
- `opendbc_repo/opendbc/car/ford/carcontroller.py:723-741` — `op_accel/op_gas`、creep 補償、jerk 限制、INACTIVE_GAS quirk
- `opendbc_repo/opendbc/car/ford/carcontroller.py:753-759` — brake_actuate 遲滯
- `opendbc_repo/opendbc/car/ford/carcontroller.py:771-777, 895-916` — bpSpeedAllow 速度死區、`apply_bp_long` 條件、兩條 fallback
- `opendbc_repo/opendbc/car/ford/carcontroller.py:139-142` — `MAX_URBAN_SPEED_MPH=45`、`following_accel_ROC=0.002`、brake 門檻
- `opendbc_repo/opendbc/car/ford/values.py:45-48` — `ACCEL_MAX/MIN`、`MIN_GAS`、`INACTIVE_GAS`
- `opendbc_repo/opendbc/car/ford/fordcan.py:147-160` — ACCDATA 訊號對應
- `opendbc_repo/opendbc/sunnypilot/car/ford/icbm.py:20-23` — ICBM 只有加/減速、無 RESUME

### Stop-and-go / standstill / resume

- `selfdrive/controls/lib/longcontrol.py:13-48` — 狀態機（stopping/starting 轉換條件）
- `selfdrive/controls/lib/longcontrol.py:75-92` — stopping 保壓、starting `startAccel`
- `selfdrive/controls/controlsd.py:170-172` — `cruiseControl.resume / override / cancel`
- `opendbc_repo/opendbc/car/ford/interface.py:109-115, 130, 132` — 變速箱偵測、`minEnableSpeed`、`autoResumeSng`
- `opendbc_repo/opendbc/car/interfaces.py:235` — `autoResumeSng` 預設 True（被 Ford 覆寫）
- `selfdrive/selfdrived/events.py:79` — belowEngageSpeed 提示

### Experimental / E2E 規劃

- `selfdrive/controls/lib/longitudinal_planner.py:36-47` — `limit_accel_in_turns`（Chill 唯一彎道處理）
- `selfdrive/controls/lib/longitudinal_planner.py:124-131` — `allow_throttle`（低速 creep 放行）
- `selfdrive/controls/lib/longitudinal_planner.py:158-170` — E2E vs MPC 選擇
- `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py:39-44` — `is_e2e()`
- `selfdrive/modeld/modeld.py:54-75` — 模型 `desiredAcceleration` / `shouldStop` 來源

### Focus 平台

- `opendbc_repo/opendbc/car/ford/values.py:77-82, 196-199` — `FORD_FOCUS_MK4` / Footnote.FOCUS（C519）
- `opendbc_repo/opendbc/car/ford/fingerprints.py:185-198` — Focus 指紋
- `opendbc_repo/opendbc/car/ford/interface.py:38, 54-60` — `radarUnavailable`、`radarDelay`（Delphi MRR）
- `opendbc_repo/opendbc/safety/modes/ford.h` — Ford 安全層（無 EBB/EPB 代碼）

---

## 附錄 B：無法由程式碼確認的事項

1. **特定 Focus ST 會被判成手排或自排**：取決於實車是否回報 shiftByWire ECU / `0x5A` / docs，repo 無對應指紋可事先斷定。
2. **「2020 Focus ST 常見 6MT、選配 7AT」**屬市場知識，`values.py` 僅標 2018 / mHEV。
3. **E2E 模型對紅燈/停止線/彎道的實車可靠度**：模型權重為二進位，程式碼只能確認資料路徑會把 `shouldStop`/`desiredAcceleration` 帶進規劃器。
4. **mHEV 的煞車 vs 回充分配**：由福特 PCM 決定，openpilot 無法影響（`carcontroller.py:717` 註解），實車減速感受可能與請求值有落差。
5. **名詞**：repo 稱此系統為「Adaptive Cruise Control with Lane Centering」，無「Co-Pilot360」；安全層無「EBB / EPB」代碼。
6. **原廠 ACC「停等 3 秒需手動」**屬市場知識，repo 內無此門檻數值。

---

*本文件由原始碼分析自動產出，僅供技術理解參考。*
