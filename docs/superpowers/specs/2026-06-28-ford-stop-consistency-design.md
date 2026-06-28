# Ford Stop Tuning 設計規格（停車間距一致性）

> **建立日期**：2026-06-28
> **分支**：`ford-stop-consistency`（off `bp-6.0`）
> **作者**：Seven Bai + Claude
> **狀態**：設計核可，待寫實作計畫
> **問題來源**：`C:\Users\swbai\Downloads\sunnypilot\ford-stop-and-go-diagnosis.md`（Ford OP Long 停車間距 run-to-run 不穩的根因分析）

---

## 1. 背景與目標

### 問題
Ford 開啟 openpilot 縱向控制（OP Long）後，**停車與前車的最終間距 run-to-run 不一致**。診斷指出的根因：

- **核心**：車速掉到 `vEgoStopping = 0.5 m/s` 時，縱向控制切到 `LongCtrlState.stopping`，此後改成「從當下 `last_output_accel` 用固定速率 ramp 到固定 `stopAccel = -2.0` 的**開環**」，**完全丟掉前車距離回授**（`selfdrive/controls/lib/longcontrol.py:75-79`）。最終停位 = 從觸發點（0.5 m/s，含雜訊）盲滑到停，故每次不同。
- **而且 Ford 完全沒客製停止參數**——`ford/interface.py` 對 `vEgoStopping/stopAccel/stoppingDecelRate/vEgoStarting` 零匹配，全用 `CarInterfaceBase` 通用預設（`opendbc_repo/opendbc/car/interfaces.py:247-250`）。
- **放大因素**：低速雷達 dRel 不濾波的跳動、Ford 低速致動層（creep 補償、煞車 jerk、bit 遲滯 chatter）。

### 目標
在 **Ford 本地**（只影響 Ford）縮短盲開環窗口、改善停車間距一致性，並順手降低接近停止段的頓挫；**與既有 Stop Smoothness（防點頭）乾淨分工、不互相破壞**。

---

## 2. 範圍

### 採用（B 類 Ford-local override 的子集）
- **B1**：在 `ford/interface.py` override `vEgoStopping`（0.5 → 0.25），縮短盲開環窗口。**（最高槓桿，針對一致性）**
- **B3**：降低 creep 補償量值（0.6 → 0.3 m/s²），緩解 1–3 m/s 接近段頓挫。
- **B4**：停車段 latch 煞車/precharge bit 遲滯，避免 hover 區 chatter。

### 不採用（明確排除）
- **B2（override `stopAccel`/`stoppingDecelRate`）**：與既有 **Stop Smoothness** 滑桿重疊（滑桿已在 carcontroller 把最終煞車從 -2.0 軟化到 -0.8）。為避免破壞滑桿「0 = 原廠」語意，本案**完全不碰 `stopAccel`**，停車舒適度/防點頭仍由滑桿負責。
- **B5（加 `kp`）**：目前 Ford `longitudinalTuning.kpV = [0.]`；加比例追蹤有雜訊權衡，本案不做。
- **放大因素①（雷達 dRel 不濾波）**：屬 shared code（`radard.py`），非 Ford-local，不在本案範圍（列為殘留風險）。

### 設計決策（brainstorming 結論）
1. **架構方案 1**：單一總開關 gate 全部；可調部分抽成 pure helper。
2. **控制介面**：調好的**固定預設值** ＋ **單一總開關**（非逐項 UI 滑桿）。
3. **總開關預設 ON**（`bp_ford_stop_tuning = "1"`）。
4. **驗證**：pure-function 單元測試 ＋ `./tests/smoke/` smoke test ＋ 使用者實車 route log（before/after）。

---

## 3. 詳細設計

### 3.1 總開關（param）
於 `common/params_keys.h`（緊接 `bp_stop_smoothness`，行 293）新增：

```c
{"bp_ford_stop_tuning", {PERSISTENT | BACKUP, BOOL, "1"}},   // 預設 ON
```

- **ON** → 啟用 B1/B3/B4；**OFF** → 完全等同原廠（安全回退）。
- **即時性**：creep（B3）、hysteresis（B4）即時生效；`vEgoStopping`（B1）在 init 讀取，**改值需重開機**（UI 須註明）。

### 3.2 B1 — 縮短盲開環窗口（`ford/interface.py`）
於 `_get_params` 的 `return ret` 前加入：

```python
# BluePilot Ford stop tuning: shorten the open-loop stopping window so the car
# tracks lead distance closer to 0 -> more consistent run-to-run stop gap.
# Read once at init (reboot to change). Guarded for offline docs generation.
if not docs:
    try:
        from openpilot.common.params import Params
        if Params().get_bool("bp_ford_stop_tuning"):
            ret.vEgoStopping = FORD_STOP_VEGO_STOPPING   # 0.25 (預設 0.5)
    except Exception:
        pass
```

- 可行性已驗證：`_get_params` 與 carcontroller 同 process（`carcontroller.py:92`、`carstate.py:27` 已用 `Params()`）；`docs` 旗標 guard 住離線文件產生情境。
- `vEgoStarting` **維持 0.5（不動）**——理由見 §6。

### 3.3 B3 — 降低 creep 補償（`helpers.py` + `carcontroller.py`）
將 `apply_creep_compensation` 從 carcontroller **移到 `helpers.py`** 並參數化（便於單元測試、與 `apply_stop_smoothing` 同處）：

```python
def apply_creep_compensation(accel, v_ego, max_creep=0.6):   # 預設值 = 原廠行為
    creep_accel = interp(v_ego, [1., 3.], [max_creep, 0.])
    creep_accel = interp(accel, [0., 0.2], [creep_accel, 0.])
    return accel - creep_accel
```

carcontroller 呼叫端（原 `:733`）：

```python
op_accel = apply_creep_compensation(op_accel, CS.out.vEgo, self.ford_stop_creep_max)
# self.ford_stop_creep_max = FORD_STOP_CREEP_MAX (0.3) if self.ford_stop_tuning else 0.6
```

- 預設參數 `max_creep=0.6` 保留原廠行為，確保 OFF 時逐 frame 相同。

### 3.4 B4 — 停車段 latch 煞車 bit（`helpers.py` + `carcontroller.py`）
將現有 inline 遲滯（原 `:757-762`）抽成 pure helper：

```python
def brake_request_hysteresis(accel, last, long_active, engage, release):
    if accel > release or not long_active:
        return False
    if accel < engage:
        return True
    return last   # 維持在 [engage, release] 之間
```

carcontroller：把 `stopping = CC.actuators.longControlState == LongCtrlState.stopping`（原 `:764`）**上移**到此區塊之前，然後：

```python
brake_release = self.brake_actuate_release            # -0.06 (一般)
if self.ford_stop_tuning and stopping:
    brake_release = FORD_STOP_BRAKE_RELEASE_LATCHED   # 0.0 → 只有 accel 明顯轉正才放開
op_brake_actuate = brake_request_hysteresis(
    accel_pitch_compensated, self.op_brake_actuate_last, CC.longActive,
    self.brake_actuate_target, brake_release)         # engage 維持 -0.14
```

- 停車段把放開門檻從 -0.06 拉到 0.0 → 遲滯 band 變 `[-0.14, 0.0]`，撐過原本造成 chatter 的 -0.06~0 徘徊區。
- op-path 中 precharge bit = brake bit（原 `:909`），故一併穩定。

### 3.5 常數（集中於 `helpers.py`，與 `_STOP_SMOOTH_*` 同處）

```python
FORD_STOP_VEGO_STOPPING         = 0.25   # B1: 0.5 -> 0.25
FORD_STOP_CREEP_MAX             = 0.3    # B3: 0.6 -> 0.3
FORD_STOP_BRAKE_RELEASE_LATCHED = 0.0    # B4: 停車段放開門檻 -0.06 -> 0.0
```

`interface.py` 由 `helpers.py` import `FORD_STOP_VEGO_STOPPING`，單一來源便於實車微調。

### 3.6 資料流（低速、`apply_bp_long` OFF、tuning ON）

```
longcontrol 停車 ramp（B1：改從 0.25 m/s 才進開環，而非 0.5）
  → actuators.accel
  → apply_creep_compensation(max_creep=0.3)              # B3
  → 3.5 m/s³ jerk limit（不動）
  → brake_request_hysteresis(release=0.0 if stopping)     # B4
  → apply_bp_long：低速 OFF → accel = op_accel
  → apply_stop_smoothing（Stop Smoothness 滑桿，不動）      # comfort
  → clip 到 ford.h 範圍 (ACCEL_MIN=-3.5, ACCEL_MAX=2.0) → ACCDATA
```

---

## 4. 與 Stop Smoothness 的關係

| | 負責 | 機制 | 即時性 |
|---|---|---|---|
| **本案（vEgoStopping/creep/hysteresis）** | 停車間距**一致性** | 縮短盲開環、降頓挫、穩 bit | vEgoStopping 需重開機；creep/hysteresis 即時 |
| **Stop Smoothness（既有）** | 停車**舒適度/防點頭** | 軟化/jerk-limit 最終煞車 | 即時滑桿 |

兩者作用點不同、可疊加；**本案不碰 `stopAccel`**，故不會破壞滑桿「0 = 原廠」語意。

---

## 5. 改動檔案清單

| 檔案 | 改動 |
|---|---|
| `common/params_keys.h` | 註冊 `bp_ford_stop_tuning`（BOOL, "1"）|
| `opendbc_repo/opendbc/car/ford/interface.py` | B1 `vEgoStopping` override（guarded read）|
| `opendbc_repo/opendbc/car/ford/helpers.py` | 移入/參數化 `apply_creep_compensation`、新增 `brake_request_hysteresis`、3 個常數 |
| `opendbc_repo/opendbc/car/ford/carcontroller.py` | `_update_params` 讀開關、上移 `stopping`、改呼叫 helper、移除原 inline creep/遲滯 |
| `selfdrive/ui/bp/layouts/settings/bluepilot.py`（+ `selfdrive/ui/bp/mici/layouts/settings/bluepilot.py`）| Longitudinal Tuning 下新增「Ford Stop Tuning」toggle（描述註明改善停車間距一致性、需重開機才完整生效）|

---

## 6. 為什麼不動 `vEgoStarting`（已驗證）

`vEgoStarting` 與 `vEgoStopping` 控制**不同的狀態轉換**：

- `vEgoStopping` 餵 `should_stop`（`drive_helpers.py:53`，經 `longitudinal_planner.py:159`）→ 決定**進入 `stopping`**。← B1 目標。
- `vEgoStarting`：`started_condition = v_ego > CP.vEgoStarting`（`longcontrol.py:22`），只用於**離開「起步 ramp」回到 `pid`**（`:46`）。← 屬「起步（GO）」側，與停車間距正交。

且 **Ford `startingState` 為 `False`**（`car.capnp:510` 預設 False，Ford 未設定）→ Ford 永不進入 `starting` 狀態，`vEgoStarting` 只剩 `pid → pid` no-op，**改它對 Ford 停/走實質無作用**。

0.25 / 0.5 非對稱安全：離開 `stopping` 靠 `starting_condition`（not should_stop / not standstill / not brake，`longcontrol.py:19-21,40-41`），與 `vEgoStarting` 無關，不會卡死。

---

## 7. 測試計畫

### 單元測試（`opendbc_repo/opendbc/car/ford/tests/`）
- `apply_creep_compensation`：`max_creep=0.3` vs `0.6` 的差異；`v_ego>3` 與 `accel>0.2` 時 creep=0 的邊界。
- `brake_request_hysteresis`：engage(-0.14)/release(-0.06 vs latched 0.0) 的 latch 行為、`not long_active` 強制 False、邊界值。
- `apply_stop_smoothing`：確認仍永不給油（輸出 ≤ 0）且能與本案疊加（不被破壞）。

### Smoke test（`./tests/smoke/`）
建 Ford `CarController` 跑合成「減速到停」情境，**tuning ON vs OFF** 比對：
- **ON**：creep 減量生效、brake bit 撐過 -0.06~0 hover 區、`accel ∈ [-3.5, 2.0]`、煞車時 `gas` 不 > `MIN_GAS`。
- **OFF**：與原廠**逐 frame 相同**。
- param 設定時 `CarParams.vEgoStopping == 0.25`。

> smoke test 需額外資訊（如合成情境的輸入建構方式、是否有現成 Ford CC 測試夾具）時再向使用者確認。

### 實車驗證（使用者）
route log before/after 確認停車間距一致性改善，並重點觀察 §8 風險。

---

## 8. 風險與邊界（實車驗證重點）

1. **雷達雜訊（0.25–0.5 m/s 延長閉環段）**：B1 讓更靠近停止仍走 radar-MPC（dRel 未濾波，放大因素①不在範圍）→ 一致性變好但該段可能更抖；B3/B4 部分緩解。**最主要觀察點。**
2. **`vEgoStopping` 重開機才生效**（UI 註明）。
3. **`vEgoStarting` 不動**：驗證 0.25/0.5 非對稱不會卡住或在 0.25–0.5 間蠕行穿過。
4. **Hybrid/EV creep 減量**：可能煞不夠 → hybrid 實車驗證；必要時改為依 HEV flag 條件化（後續 refinement）。
5. **OP Long 關閉時自然 inert**：整個縱向區塊（`carcontroller.py:724` `if self.CP.openpilotLongitudinalControl`）跳過、`vEgoStopping` 無作用 → tuning 僅在 OP Long ON 時作用。

---

## 9. 非目標 / 後續可能 refinement
- B2（stopAccel override）、B5（kp）、放大因素①（radar dRel 濾波）、架構級閉環停止（改 shared code）。
- creep 依 powertrain（HEV flag）條件化。
- `vEgoStopping` / creep / hysteresis 改成逐項 UI 可調（目前以固定值 + 總開關為準）。

---

## 10. 可調點（實車微調用）
- `FORD_STOP_VEGO_STOPPING`：0.25（保守可改 0.3）。
- `FORD_STOP_CREEP_MAX`：0.3。
- `FORD_STOP_BRAKE_RELEASE_LATCHED`：0.0。
- `bp_ford_stop_tuning` 預設：ON（可改 OFF 採 opt-in）。
