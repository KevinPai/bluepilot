# 2022 Ford Kuga 1.5L 旗艦X：BluePilot 縱向控制三開關組合深度剖析

> **分析對象**：2022 年 Ford Kuga 22.5 式 1.5L EcoBoost 旗艦 X（純汽油非 hybrid、8 速自排）
> **對映平台**：`FORD_ESCAPE_MK4`（Ford Kuga 2020-23 / Escape 2020-22，C2/CD4 平台）
> **三顆開關**：Bypass BP Longitudinal Control、OP Long、Experimental Mode
> **程式碼基準**：BluePilot 分支 `bp-6.0`（sunnypilot / openpilot 衍生）
> **分析方法**：原始碼靜態分析（多 agent 讀碼 + 對抗式平台驗證 + 一致性審查 + 人工覆核）
> **撰寫日期**：2026-06-27
> **姊妹文件**：[Focus-ST-MK4-縱向開關剖析.md](./Focus-ST-MK4-縱向開關剖析.md)（手排 mHEV 對照組）

---

## 摘要（TL;DR）

1. **Kuga 1.5L 是 8AT 自排 → stop-and-go 直接可用。** `minEnableSpeed=-1`、`autoResumeSng=True`（interface.py:132）。這是相較手排 Focus ST 最大的差異——Focus「低速無法接管、無自動起步」的痛點在 Kuga 上**消失**。
2. **「Bypass BP Long」不是把縱向交回原廠。** 交回原廠的唯一開關是 **OP Long=OFF**。`disable_BP_long_UI` 整段程式碼巢狀在 `if openpilotLongitudinalControl`（carcontroller.py:721）之內，OP Long 一關它就是 no-op。
3. **Bypass 開關方向務必別記反**：`Bypass=ON` → 停用 BP、**全程純 op_***；`Bypass=OFF` → 在 >50mph 才套用 BP 高速微調。**無論哪個位置，塞車低速 stop-and-go 行為完全相同。**
4. **三顆開關不互斥、不會跳 ACC 故障**（總開關 / 規劃層 / 致動層三層獨立）。
5. **「只開 Experimental」設不起來**：OP Long=OFF 時該開關被 UI 灰掉並從 Params 移除。
6. **本車純汽油非 hybrid → 無 regen 警告**：傳統摩擦煞車，openpilot 的 accel/brake 直接映射摩擦煞車，無「煞車 vs 回充由 PCM 分配」的不確定性（這是 Focus mHEV 才有的限制）。
7. **唯一重大保留：指紋支援不確定性**。repo 收錄的 `FORD_ESCAPE_MK4` 指紋全來自 hybrid/PHEV 車，**1.5L 純汽油版能否被正確辨識需上路前以實車確認**。

---

## 目錄

1. [三顆開關的真實定義與層級](#1-三顆開關的真實定義與層級)
2. [三個必須糾正的核心觀念](#2-三個必須糾正的核心觀念)
3. [Kuga 1.5L 平台特性（與 Focus 的關鍵差異）](#3-kuga-15l-平台特性與-focus-的關鍵差異)
4. [共通底層機制](#4-共通底層機制)
5. [六種組合逐一剖析](#5-六種組合逐一剖析)
6. [跨組合比較表](#6-跨組合比較表)
7. [Kuga（自排非hybrid）vs Focus ST（手排mHEV）](#7-kuga自排非hybridvs-focus-st手排mhev)
8. [給 Kuga 1.5L 車主的建議](#8-給-kuga-15l-車主的建議)
9. [重要提醒與免責](#9-重要提醒與免責)
10. [附錄 A：程式碼證據索引](#附錄-a程式碼證據索引)
11. [附錄 B：無法由程式碼確認的事項](#附錄-b無法由程式碼確認的事項)

---

## 1. 三顆開關的真實定義與層級

> 此節為 Ford 平台通用，Kuga 與 Focus 完全相同（同一份程式碼）。

| 俗稱 | UI 名稱 | Param | 作用層級 |
|---|---|---|---|
| Bypass BP Long | Bypass BP Longitudinal Control | `disable_BP_long_UI` | **致動層**（Ford carcontroller 內部子開關） |
| OP Long | sunnypilot Longitudinal Control (Alpha) | `AlphaLongitudinalEnabled` | **總開關**（決定縱向歸 openpilot 還是原廠） |
| Experimental Mode | Experimental Mode | `ExperimentalMode` | **規劃層**（是否併入 E2E 神經網路輸出） |

```
OP Long (AlphaLongitudinalEnabled)
  │  card.py:100/112 → ford/interface.py:75-77 → openpilotLongitudinalControl = True
  │
  └─ openpilotLongitudinalControl == True 時，以下才有意義：
       ├─ Experimental Mode  → 規劃層：is_e2e = experimentalMode（longitudinal_planner.py:39-44）
       └─ Bypass BP Long     → 致動層：carcontroller.py:780/895 是否套用 BP 高速微調
```

- **「OP Long」帶 `DEVELOPMENT_ONLY` 旗標**（developer.py:74-80），release 分支會被隱藏並移除——屬實驗性 alpha。
- **Ford `alphaLongitudinalAvailable=True`**（interface.py:70），所以 `has_longitudinal_control = AlphaLongitudinalEnabled`（ui_state.py:188-191）。

---

## 2. 三個必須糾正的核心觀念

### 糾正 #1：「Bypass BP Long」≠ 把縱向交回福特原廠

- 交回原廠的唯一開關是 **OP Long=OFF**——此時 `interface.py:75-77` 不設 `openpilotLongitudinalControl=True`，`carcontroller.py:721` 整段縱向區塊不執行，openpilot 不送油門/煞車。
- `disable_BP_long_UI` 的兩處使用（carcontroller.py:780、895）都在 721 之內，OP Long=OFF 時是純 no-op。

### 糾正 #2：「Bypass BP Long + OP Long」不矛盾、不會跳 ACC 故障

- OP Long=ON 時 openpilot 全權縱向、福特原廠 ACC 已退出，不存在兩套訊號並送衝突。
- 唯一與 cruise fault 相關的是 `INACTIVE_GAS=-5.0` quirk（不用油門必送 -5.0 否則 PCM 報 fault），這是**正常設計**。

### 糾正 #3：「只開 Experimental、不開 OP Long」對縱向無效，且設不起來

- 執行期：`selfdrived.py:597`、`card.py:321` 都是 `experimental_mode = get_bool("ExperimentalMode") and openpilotLongitudinalControl` → OP Long=OFF 恆 False。
- UI：`toggles.py:178-183` 在 OP Long=OFF 時 `set_enabled(False)` + `set_state(False)` + `params.remove("ExperimentalMode")`。

### ⚠️ 額外糾正：Bypass 開關的真實方向（極易記反）

| 開關位置 | `disable_BP_long_UI` | 行為 |
|---|---|---|
| **Bypass = ON** | `True` | carcontroller.py:780 `if not ...` 為 False → **跳過 BP 區塊，全程純 op_***（含高速） |
| **Bypass = OFF** | `False` | 在 >50mph 速度死區、未踩油煞、（無前車或前車>40mph）時**套用 BP 高速微調** |

→ 也就是說：**有 BP 高速微調的是 `Bypass=OFF` 的組合（#2/#3），不是 `Bypass=ON` 的組合（#4/#6）。** 但兩種位置在塞車低速一律 fallback 到 `op_*`，**對 stop-and-go 無差別**。

---

## 3. Kuga 1.5L 平台特性（與 Focus 的關鍵差異）

### 3.1 平台對映

- 2022 Kuga → car_docs「Ford Kuga 2020-23」→ **`FORD_ESCAPE_MK4`**（values.py:162-168）。
- **不是** `FORD_ESCAPE_MK4_5`（那是 2023-24 改款 / Kuga 2024 的 **CANFD** 平台，values.py:169-176）。
- `FORD_ESCAPE_MK4` = `FordPlatformConfig` → **一般 CAN（非 CANFD）**、harness `ford_q3`。

### 3.2 變速箱（決定 stop-and-go 的關鍵，Kuga 直接過關）

| | Kuga 1.5L（自排 8AT） |
|---|---|
| 偵測（interface.py:109-115） | shiftByWire ECU / 主匯流排 `0x5A`(Gear_Shift_by_Wire_FD1) → automatic |
| `minEnableSpeed` | -1（CarSpecs 預設，未覆寫，values.py:167） |
| `autoResumeSng`（= `minEnableSpeed==-1`） | **True**（interface.py:132） |
| stop-and-go / 靜止自動起步 | **可用**（須 OP Long=ON） |

### 3.3 動力（純汽油非 hybrid → 無 regen 警告）

- 1.5L EcoBoost 純內燃機，不具 HEV 訊號 → `HEV_CLUSTER_DATA`(0x365)、`HEV_BATTERY_DATA`(0x07A+0x24B+0x24C) 旗標**不會被設定**（interface.py:121-127）。
- `carcontroller.py:717` 的「hybrids/EV 由 Ford PCM 決定煞車踏板 vs 動能回收，openpilot 無法影響」**僅適用油電/EV**；本車 accel 請求直接映射**傳統摩擦煞車**，無回生分配的不確定性——縱向行為反而**更可預測**。

### 3.4 雷達

- `RADAR.DELPHI_MRR` 真實雷達（values.py:118），`radarUnavailable=False`、`radarDelay=0.06`。stop-and-go 前車來自 `radarState.leadOne`（**真實雷達**，非視覺/相機匯流排）。

### 3.5 ⚠️ 唯一重大保留：1.5L 純汽油的指紋支援不確定性

- `FORD_ESCAPE_MK4` 收錄的指紋（EPS `LX6C-14D003`、ABS `LX6C-2D053`、雷達、相機 FW）**全採自標 hybrid/PHEV 的車**；該平台**沒有 engine ECU FW**。
- 精確 FW 比對對未收錄的 1.5L 汽油後綴**可能失敗**；要靠**模糊比對**（`match_fw_to_car_fuzzy` 只比 abs/camera/radar/eps 的 `platform_hint=X6C`、`model_year_hint=L`，engine 不在內，values.py:322）才會匹配——屬**高機率推論而非實證**。
- 佐證匹配可能性高：同款 EPS 料號 `LX6C-14D003` 也用於 Bronco Sport（跨車型/動力共用）。
- **結論：上路前務必以實車確認指紋成功匹配 `FORD_ESCAPE_MK4` 且變速箱被判 automatic。**

---

## 4. 共通底層機制

### 4.1 OP Long=ON 時 gas/brake 如何產生

- `op_accel = op_gas = actuators.accel`（carcontroller.py:723-724）。
- 低速 creep 補償（carcontroller.py:79-83, 730；註解帶 `TODO: verify on EV/hybrid`，本車純 ICE 走摩擦煞車路徑）。
- 煞車 jerk 速率限制 **3.5 m/s³**（carcontroller.py:732-734，註解自承重煞仍會 overshoot）。
- 夾擠常數（values.py:45-48）：`ACCEL_MAX=2.0`、`ACCEL_MIN=-3.5`、`MIN_GAS=-0.5`、`INACTIVE_GAS=-5.0`。

### 4.2 「停等後誰起步」的真實機制（OP Long=ON，Kuga 自排可用）

1. 跟停 → `LongCtrlState.stopping`，`output_accel` 緩降到 `stopAccel` 並持續送煞車保壓（longcontrol.py:75-80）。
2. 起步條件：`starting_condition = not should_stop and not cruise_standstill and not brake_pressed`（longcontrol.py:18-21）。
3. 自動 resume：`cruiseControl.resume = enabled and cruiseState.standstill and not shouldStop`（controlsd.py:172）。
4. **沒有固定 3 秒**——前車一走、`shouldStop` 一清除就起步，由 `autoResumeSng=True` 把關。**「停等 3 秒需手動」是福特原廠 ACC（OP Long=OFF）的概念。**

### 4.3 靜止保壓不是原廠 EPB

- Ford 安全層（ford.h）**無 EBB/EPB 代碼**。是 openpilot 持續送 `AccBrkTot_A_Rq`+`brake_actuate`+`precharge` 位元（fordcan.py:148/157/158）由 PCM 執行；`stopping` 位元只通知 PCM 停車保持。

### 4.4 ICBM 無 RESUME

- ICBM 只有加/減速鈕（icbm.py:20-23）→ OP Long=OFF（原廠 ACC）時 openpilot 無法自動重新起步。

### 4.5 Experimental Mode（E2E）

- `is_e2e = experimentalMode = (ExperimentalMode AND openpilotLongitudinalControl)`（longitudinal_planner.py:39-44；selfdrived.py:597）。**並非結構性恆 False**——Chill 模式無 E2E 是因 `experimentalMode=False`。
- 啟用時：`output_a_target = min(e2e, mpc)`、`shouldStop = e2e OR mpc`（longitudinal_planner.py:163-167）。只會讓車**更保守**，不會更積極。
- **DEC（Dynamic Experimental Control）會進一步調控**：DEC 啟用時 `is_e2e = experimentalMode AND dec.mode()=='blended'`，故 E2E 並非「開了 Experimental 就全程生效」。

---

## 5. 六種組合逐一剖析

> 統一結構：**控制權歸屬 / 過彎減速 / 紅綠燈與路口 / Stop & Go / Kuga 風險與推薦度**。
> 本車為自排，採單一推薦度（1=最低，5=最高）。**已套用一致性審查的 Bypass 極性修正與評分校正。**

### 組合 1｜只開 Bypass BP Long（OP Long 關、實驗關）

**前提判定：✅ 正確。** OP Long=OFF → `carcontroller.py:721` 整段不執行，openpilot 不送縱向，**100% 福特原廠 ACC**；Bypass 與 Experimental 皆 no-op。

- **控制權歸屬**：油門/煞車/過彎/紅綠燈/Stop&Go **全部福特原廠 ACC + PCM**；BluePilot 仍提供**橫向**。
- **過彎減速**：無（原廠不讀曲率）。
- **紅綠燈與路口**：無（原廠不辨識號誌）。
- **Stop & Go**：Kuga 自排原廠 ACC **具備 Stop & Go**，可跟停至靜止；但停等逾時（原廠約 3 秒級）需手動 RES 或輕踩油門。openpilot 的 `autoResumeSng` 在此**不生效**（僅 OP Long=ON 才有意義）；ICBM 無 RESUME，openpilot 不會代按。**無法完全免手動。**
- **Kuga 風險與推薦度**：openpilot 端**零暴衝/急煞風險**（不送命令），縱向風險＝原廠 ACC。指紋/自排辨識的不確定性對本組合幾乎無影響（縱向全原廠）。**推薦度：4**（最低風險，原廠縱向 + BluePilot 橫向）。

### 組合 2｜只開 OP Long（Chill，Bypass 關、實驗關）

**前提判定：⚠️ 大致對。** openpilot 確實接管油煞、繞過原廠「需按 RESUME」限制；但「免手動起步」靠的是**自排 autoResumeSng=True**，與 Bypass 無關。Chill 無任何視覺紅燈/彎道能力。

- **控制權歸屬**：油門/煞車由 **openpilot（Comma MPC）**。**Bypass=OFF → 高速（>50mph）會套用 BP 微調**；塞車低速一律 fallback 到 `op_*`。
- **過彎減速**：弱。`experimentalMode=False → is_e2e=False`，只有被動 `limit_accel_in_turns`（夾正向加速度上限）。
- **紅綠燈與路口**：**無**。純 MPC 只看雷達前車＋cruise 假障礙。
- **Stop & Go**：
  - 跟停線性度尚可（creep 補償＋3.5 m/s³ jerk 限制，但重煞仍會 overshoot）；純摩擦煞車、無 regen 切換頓挫。
  - 停等>3 秒由 **openpilot 自動起步、完全免手動**（autoResumeSng=True，shouldStop 清除即起步）。
  - 起步受 `ACCEL_MAX=2.0` 與 creep 補償約束，**暴衝風險低**、反應線性。
- **Kuga 風險與推薦度**：非 hybrid → 無 regen 警告（利多）；OP Long 屬 alpha。**推薦度：3**。

### 組合 3｜OP Long + Experimental（完全體 E2E，Bypass 關）

**前提判定：✅ E2E 確實啟用。** 但「靜止 Hold 由原廠 EPB 還是 Comma」是假二選一——是 Comma 持續送煞車、PCM 執行（無 EPB latch）；且「停等 3 秒需手動」不適用（OP Long=ON 無固定 3 秒）。

- **控制權歸屬**：油門/煞車 openpilot（**Bypass=OFF → 高速套用 BP 微調**）；過彎/紅綠燈由 **E2E 主導**；Stop&Go openpilot。
- **過彎減速**：**有**。`is_e2e=True → output_a_target=min(e2e, mpc)`，進彎前主動降速（**受 DEC 調控**，並非全程必然啟用）。
- **紅綠燈與路口**：**有視覺停車能力**（`shouldStop=e2e OR mpc`）；綠燈起步 openpilot 自動恢復、免按鍵。**E2E 辨識屬實驗性，可靠度無法由程式碼保證，須全程監看。**
- **Stop & Go**：跟停保壓由 openpilot 控制；停等後 shouldStop 清除即自動起步、**完全免手動**；起步線性、暴衝風險低。
- **Kuga 風險與推薦度**：急煞 overshoot 風險中等；非 hybrid 無 regen 警告；指紋不確定性（見 §3.5）。**推薦度：4**（功能最完整）。

### 組合 4｜Bypass BP Long + OP Long（被誤認為「邏輯矛盾」）

**前提判定：❌ 不矛盾、不會跳 ACC 故障。** Bypass 只是 OP Long 內部的致動層子分支。

- **控制權歸屬**：油門/煞車 openpilot。**Bypass=ON → 全程純 op_*（含高速，無 BP 微調）**（走 carcontroller.py:911 else）。
- **過彎減速 / 紅綠燈**：**無**（Experimental=OFF，同組合 2）。
- **Stop & Go**：**與組合 2 在塞車完全相同**——低速一律 `op_*`，Bypass 在此無作用。自排免手動自動起步。
- **Kuga 風險與推薦度**：與組合 2 同層級；Bypass=ON 對塞車毫無幫助（只在 >50mph 少了 BP 微調）。**推薦度：3**。

### 組合 5｜只開 Experimental（Bypass 關、OP Long 關）

**前提判定：❌ 重大錯誤。** 此設定**實務上設不起來**（UI 灰掉並 `params.remove("ExperimentalMode")`）。**「UI 顯示可起步卻被原廠 3 秒鎖死」的衝突在程式上不可能發生**——openpilot 此時沒有起步判斷在跑（shouldStop/autoResumeSng 只在 OP Long=ON 生效）；所謂逾時需手動純粹是原廠 ACC 自身的 stop-and-go 邏輯。

- **控制權歸屬**：**縱向 100% 福特原廠 ACC**，與組合 1 **完全等價**。
- **過彎 / 紅綠燈 / Stop&Go**：openpilot 零貢獻。
- **真正風險**：**認知錯誤**——誤以為開了 Experimental 就有 E2E 能力，若因此放鬆監看會有追撞風險。
- **推薦度：1**（縱向等同組合 1，但易誤解，不該當獨立設定）。

### 組合 6｜Bypass BP Long + Experimental（OP Long 關或開）

**前提判定：❌ Bypass 與 Experimental 不互斥**（致動層 vs 規劃層，獨立運作）。

- **OP Long=OFF 子情況**：退化為組合 5/1，**縱向全原廠**。
- **OP Long=ON 子情況**：E2E 規劃啟用 + **Bypass=ON 全程純 op_*（高速無 BP 微調）** ≈ 接近原生 sunnypilot E2E 縱向。
  - 過彎減速、紅綠燈視覺停車：**有**（E2E，受 DEC 調控）。
  - Stop & Go：自排免手動自動起步；Bypass=ON 對塞車無影響。
- **推薦度**：OP Long=ON 時 **4**（≈組合 3 但高速無 BP 微調）；OP Long=OFF 時退化為 **1**。

---

## 6. 跨組合比較表

> 已套用 Bypass 極性修正（`Bypass=ON` → 全程 op_*、無 BP 高速微調；`Bypass=OFF` → 高速套用 BP 微調）。本車自排，採單一推薦度。

| 組合 (Bypass / OP Long / Exp) | 油煞控制權 | 過彎主動減速 | 紅綠燈視覺停起 | Stop&Go 自動起步 | 推薦度 |
|---|---|---|---|---|---|
| 1. ON / OFF / OFF（純原廠） | 福特原廠 ACC+PCM | 無 | 無 | 原廠控制；久停需手動 RES/點油門 | **4** |
| 2. OFF / ON / OFF（Chill） | openpilot MPC；**高速(>50mph)套用 BP 微調** | 弱（僅夾正向上限） | 無（純雷達跟車） | **可免手動**（autoResumeSng=True） | **3** |
| 3. OFF / ON / ON（完全體 E2E） | openpilot；高速套用 BP 微調 | **有（E2E，受 DEC 調控）** | **有（E2E，實驗性須監看）** | **可免手動** | **4** |
| 4. ON / ON / OFF | openpilot；**全程純 op_*（無 BP 微調）** | 弱（同組合 2） | 無 | **可免手動**（市區同組合 2） | **3** |
| 5. 任意 / OFF / ON（矛盾） | 福特原廠 ACC（設不起來，縱向同組合 1） | 無 | 無 | 原廠控制；久停需手動 | **1** |
| 6. ON / ON / ON | openpilot E2E；**高速純 op_*（無 BP 微調）** | 有（E2E，受 DEC 調控） | 有（E2E，實驗性） | **可免手動** | **4**（OP 關時退化為 1） |

> 修正重點：「BP 高速微調」屬於 **Bypass=OFF** 的組合（#2/#3），不是 Bypass=ON 的組合（#4/#6）。所有組合的低速 stop-and-go 結論不受開關位置影響。

---

## 7. Kuga（自排非hybrid）vs Focus ST（手排mHEV）

| 面向 | 2022 Kuga 1.5L（FORD_ESCAPE_MK4） | Focus ST MK4（FORD_FOCUS_MK4） |
|---|---|---|
| 變速箱 | **8AT 自排** → minEnableSpeed=-1、autoResumeSng=True | 多為 6MT 手排 → minEnableSpeed=20mph、autoResumeSng=False |
| **Stop-and-go** | **可用**（OP Long=ON 時免手動跟停起步） | **手排不可行**（<32km/h 無法接管、無自動起步） |
| 動力 | 1.5L EcoBoost **純汽油** → 無 HEV 旗標、傳統摩擦煞車、**無 regen 警告** | **mHEV** → carcontroller.py:717 regen/煞車分配警告適用 |
| 匯流排 | 一般 CAN（非 CANFD），ford_q3，Delphi MRR 真雷達 | 一般 CAN，Delphi MRR 真雷達 |
| 三開關邏輯 | **完全相同**（同一份 Ford 程式碼） | 完全相同 |
| 指紋確定性 | 1.5L 汽油僅靠 fuzzy match（需實車確認） | 手排/自排辨識需實車確認 |

**一句話結論**：Kuga 1.5L 自排把 Focus ST 手排「stop-and-go 不可行 + mHEV regen 不可控」兩大限制都**解掉了**，是更適合 openpilot 縱向（含塞車免手動跟停起步）的平台；唯一要先確認的是 1.5L 純汽油能否被正確指紋辨識。

---

## 8. 給 Kuga 1.5L 車主的建議

- **想要塞車免手動跟停起步 ＋ 紅燈/彎道視覺輔助** → **組合 3（OP Long + Experimental）** 最完整。
- **只要 openpilot 縱向、不要 E2E 視覺停車**（偏好純雷達跟車）→ **組合 2**。
- **求最低風險、只要橫向輔助** → **組合 1**（原廠縱向 + BluePilot 橫向）。
- **Bypass BP Long**：對塞車/市區**完全無感**，只在 >50mph 巡航微調有差；一般可保持預設（OFF）。
- **上路前必做**：以實車確認 (a) 指紋成功匹配 `FORD_ESCAPE_MK4`、(b) 變速箱被判 `automatic`（確保 `autoResumeSng=True`），再依賴 stop-and-go 自動起步。

---

## 9. 重要提醒與免責

1. **指紋支援不確定性（最重要）**：1.5L 純汽油 Kuga 能否被辨識為 `FORD_ESCAPE_MK4` 屬高機率推論，**須實車確認**（§3.5）。
2. **stop-and-go「無固定 3 秒、shouldStop 清除即起步」只在 OP Long=ON 成立**；OP Long=OFF 時縱向全原廠、ICBM 無 RESUME，久停起步須自己處理。
3. **E2E 視覺停減速可靠度無法由程式碼保證**（模型權重為二進位），且受 DEC 動態調控、只會讓車更保守；須全程監看準備接管。
4. **OP Long 為 `DEVELOPMENT_ONLY` alpha**，release 分支會被移除。
5. **急煞手感**：3.5 m/s³ jerk 限制仍可能 overshoot（carcontroller.py:732-734）；本車純汽油為摩擦煞車，**無** mHEV 的 regen 分配不確定性。
6. **Bypass 開關方向別記反**：ON=停用 BP（全程 op_*）、OFF=高速啟用 BP 微調；塞車一律無感。

> 本文件為**原始碼靜態分析**結果，非實車驗證。實車行為另受福特 PCM/韌體、指紋辨識結果、雷達有效性、模型權重等執行期因素影響。任何輔助駕駛功能皆須駕駛全程監看並隨時準備接管。

---

## 附錄 A：程式碼證據索引

> 路徑相對於 repo 根目錄。`opendbc_repo/` 為 opendbc 子模組。

### 開關定義與 gating（Ford 通用）

- `selfdrive/ui/bp/layouts/settings/bluepilot.py:364-371` — Bypass BP Long 開關（`disable_BP_long_UI`，put_bool 直接映射）
- `selfdrive/ui/layouts/settings/developer.py:74-80` — OP Long（`AlphaLongitudinalEnabled`，DEVELOPMENT_ONLY）
- `selfdrive/ui/layouts/settings/toggles.py:55-60, 173-197` — Experimental Mode 與 gating
- `selfdrive/ui/ui_state.py:186-191` — `has_longitudinal_control`
- `selfdrive/car/card.py:100, 112, 321` — `alpha_long` 傳入；執行期 experimental gating
- `selfdrive/selfdrived/selfdrived.py:117-118, 533, 597` — 移除/發佈/gating experimental
- `opendbc_repo/opendbc/car/ford/interface.py:70-77` — `alphaLongitudinalAvailable` / `openpilotLongitudinalControl`

### Ford 縱向核心

- `opendbc_repo/opendbc/car/ford/carcontroller.py:721` — 整段縱向由 `if openpilotLongitudinalControl` 守門
- `opendbc_repo/opendbc/car/ford/carcontroller.py:723-741` — op_accel/op_gas、creep 補償、jerk 限制、INACTIVE_GAS quirk
- `opendbc_repo/opendbc/car/ford/carcontroller.py:771-777, 780, 895-916` — bpSpeedAllow 速度死區、`apply_bp_long`、兩條 fallback（Bypass 極性）
- `opendbc_repo/opendbc/car/ford/carcontroller.py:714, 717` — accel=煞車訊號；hybrid/EV regen 註解（本車不適用）
- `opendbc_repo/opendbc/car/ford/values.py:45-48` — ACCEL/GAS 常數
- `opendbc_repo/opendbc/car/ford/fordcan.py:147-160` — ACCDATA 訊號對應
- `opendbc_repo/opendbc/sunnypilot/car/ford/icbm.py:20-23` — ICBM 無 RESUME

### Stop-and-go / standstill / resume

- `selfdrive/controls/lib/longcontrol.py:13-48, 75-92` — 狀態機、stopping 保壓
- `selfdrive/controls/controlsd.py:170-172` — `cruiseControl.resume`
- `opendbc_repo/opendbc/car/ford/interface.py:109-115, 132` — 變速箱偵測、`minEnableSpeed`、`autoResumeSng`

### Experimental / E2E

- `selfdrive/controls/lib/longitudinal_planner.py:36-47, 124-131, 158-170` — limit_accel_in_turns、allow_throttle、E2E vs MPC
- `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py:39-44` — `is_e2e()`（= experimentalMode，DEC 啟用時 AND blended）
- `sunnypilot/selfdrive/controls/lib/dec/dec.py` — DynamicExperimentalControl

### Kuga / ESCAPE_MK4 平台

- `opendbc_repo/opendbc/car/ford/values.py:162-168` — `FORD_ESCAPE_MK4`（Kuga 2020-23）
- `opendbc_repo/opendbc/car/ford/values.py:169-176` — `FORD_ESCAPE_MK4_5`（2024 CANFD，本車**不**屬此）
- `opendbc_repo/opendbc/car/ford/values.py:115-119` — FordPlatformConfig（Delphi MRR、一般 CAN）
- `opendbc_repo/opendbc/car/ford/values.py:91-92` — init_make harness（ford_q3）
- `opendbc_repo/opendbc/car/ford/values.py:322` — `PLATFORM_CODE_ECUS`（fuzzy match 不含 engine）
- `opendbc_repo/opendbc/car/ford/fingerprints.py:42-64` — ESCAPE_MK4 指紋（無 engine FW）
- `opendbc_repo/opendbc/car/ford/interface.py:38, 54-57, 121-127` — radarUnavailable、radarDelay、HEV 旗標
- `opendbc_repo/opendbc/safety/modes/ford.h` — Ford 安全層（無 EBB/EPB）

---

## 附錄 B：無法由程式碼確認的事項

1. **1.5L 純汽油 Kuga 的指紋驗證**：`FORD_ESCAPE_MK4` 無 engine ECU FW，收錄的 EPS/ABS/雷達/相機 FW 後綴均採自 hybrid/PHEV 車。精確比對對汽油版後綴可能失敗，需 fuzzy match（屬推論）。**須實車確認。**
2. **自排辨識依賴實車 live 資料**：`shiftByWire`(0x732) 與 `0x5A`(Gear_Shift_by_Wire_FD1) 不在離線指紋，repo 無法靜態確認；惟 Escape/Kuga MK4 全車系皆自排，誤判 manual 機率極低。
3. **「22.5 式」年式對應**：以 2022 落在 car_docs「2020-23」區間為據；若實為已改款 CANFD 平台（Kuga 2024 類）則應屬 `FORD_ESCAPE_MK4_5`。
4. **E2E 模型對紅燈/停止線/彎道的實車可靠度**：模型權重為二進位，程式碼只能確認資料路徑。
5. **creep 補償**：carcontroller.py:729 帶 `TODO: verify on EV/hybrid`；本車純 ICE 走摩擦煞車路徑，特定 1.5L 8AT 起步平順度無 repo 實測佐證。
6. **名詞**：repo 稱此系統為「Adaptive Cruise Control with Lane Centering」，無「Co-Pilot360」；安全層無「EBB / EPB」代碼。

---

*本文件由原始碼分析（含對抗式平台驗證與一致性審查）產出，僅供技術理解參考。*
