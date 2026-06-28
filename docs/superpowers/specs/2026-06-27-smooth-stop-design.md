# 設計規格：OP Long 停止收尾柔順化（抗點頭）

| 項目 | 內容 |
|---|---|
| 日期 | 2026-06-27 |
| 狀態 | 已核准設計，待寫實作計畫 |
| 範圍 | Ford 平台（carcontroller 收尾整形 + BluePilot UI 可調參數） |
| 適用車型 | 所有 Ford（FordPlatformConfig / CANFD 皆適用，邏輯在 carcontroller 共用路徑） |
| 程式碼基準 | BluePilot 分支 `bp-6.0` |

---

## 1. 背景與問題

開啟 OP Long（`openpilotLongitudinalControl=True`）後，Stop & Go 的「Stop」動作在**完全停止那一瞬間**會讓車頭明顯下沉（點頭），停住後車身回彈，後座不適。

**根因（程式碼證據）**：

- `selfdrive/controls/lib/longcontrol.py:75-80` —— 進入 `LongCtrlState.stopping` 後，`output_accel` 會**朝 `stopAccel` 每秒加深 `stoppingDecelRate`**：
  ```python
  elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
          output_accel = min(output_accel, 0.0)
          output_accel -= self.CP.stoppingDecelRate * DT_CTRL
  ```
- Ford **未覆寫**停止參數，沿用 `opendbc_repo/opendbc/car/interfaces.py:247-250` 預設：`stopAccel = -2.0`、`stoppingDecelRate = 0.8`、`vEgoStopping = 0.5`。
- 淨效果：車快停住時，commanded deceleration **反而加深到 -2.0 m/s²**，把前懸吊壓下去 → 點頭；停住後 -2.0 死壓 → 回彈。

也就是說，問題不是「煞太晚」，而是「**收尾時煞車不減反增、缺少輕柔觸地（ease-out）**」。

## 2. 目標與非目標

**目標**
- 讓**完全停止那一瞬間**的煞車更柔（消除點頭），保留 OP Long 的**所有**功能（Chill / Experimental(E2E) / BP long / stop-and-go / 自動起步等皆不變）。
- 可在車上**即時微調**（不需重開機），由使用者上路把手感調到滿意。

**非目標**
- 不改變一般行駛/中高速的煞車力與制動權威。
- 不改變起步（resume）行為。
- 不處理斜坡長時間保壓的極端情境（使用者幾乎都平地停車；保壓下限見 §5）。
- 不改其他品牌平台行為。

## 3. 決策：採用「Ford carcontroller 收尾整形 + UI 即時可調」（方案 A）

**為何 A**：使用者要上路即時微調 → 必須 live；使用者只有 Ford → 改動鎖在 Ford、風險最小；與本專案既有「Ford carcontroller 即時讀 BP 參數」慣例一致。

**已否決的替代方案**
- **B：改 Ford interface 的 CarParams（`stopAccel`/`stoppingDecelRate`）** —— CarParams 開機才算一次，**不能即時調**，不符需求。
- **C：在控制鏈即時覆寫 `CP.stopAccel`/`stoppingDecelRate`** —— 動到跨平台控制碼，影響所有車、blast radius 過大，且即時 mutate CarParams 不正規。

## 4. 詳細設計

### 4.1 UI 參數（單一滑桿）

| 項目 | 內容 |
|---|---|
| UI 名稱 | **Stop Smoothness（停止柔順度）** |
| Param | `bp_stop_smoothness`，型別 float，範圍 0.0 ~ 1.0（UI step 0.05） |
| 預設值 | **0.0（= 完全等於現狀，opt-in）** |
| 位置 | `selfdrive/ui/bp/layouts/settings/bluepilot.py` 的「Longitudinal Tuning」區段（現有 `Bypass BP Long`、`Disable Downhill Compensation` 之後） |
| 元件 | 沿用既有 `float_control_item`（參照 `vbatt_pause_charging`，`bluepilot.py:335-344`） |

UI 範例：
```python
self._stop_smoothness = float_control_item(
    lambda: tr("Stop Smoothness"),
    lambda: tr("Soften braking at the final stop to reduce nose-dive (0 = stock)."),
    param="bp_stop_smoothness",
    min_value=0.0, max_value=1.0, step=0.05,
    icon="chffr_wheel.png",
)
```

**Param 宣告**：在 `common/params_keys.h` 新增 `bp_stop_smoothness`，預設 `"0.0"`，比照其他 bp_* float 參數（如 `custom_path_offset`、`LC_PID_gain_UI`）的註冊/預設方式。

**mici 變體（選配）**：`selfdrive/ui/bp/mici/layouts/settings/bluepilot.py` 也有縱向開關（line 70）；若要 mici 機型一致，可加對應控制，屬選配、不影響主功能。

### 4.2 整形演算法（純函式，放 `opendbc_repo/opendbc/car/ford/helpers.py`）

抽成純函式以利離線測試；carcontroller 改為 import 使用。

```python
# ford/helpers.py
# interp：使用專案既有用法（np.interp，或 common 的 interp 輔助；實作時與 carcontroller 一致）

STOP_SMOOTH_BP = [0.0, 1.0]
STOP_SMOOTH_ACCEL_V = [-2.0, -0.8]   # m/s^2：最終煞車/保壓目標（s=0→-2.0 現狀；s=1→-0.8 最柔）
STOP_SMOOTH_JERK_V = [3.5, 0.5]      # m/s^3：收尾「加深」速率上限（s=0→3.5 現狀；s=1→0.5 最柔）

def apply_stop_smoothing(accel, stopping, long_active, smoothness, accel_last, dt_step):
    """Soften the final brake when coming to rest. ONLY ever reduces braking. Pure & testable.
    Returns (shaped_accel, new_accel_last)."""
    if smoothness <= 0.0 or not long_active or not stopping:
        return accel, accel  # no-op；accel_last 追蹤當前值以利進入 stopping 時平滑接續
    soft_stop_accel = interp(smoothness, STOP_SMOOTH_BP, STOP_SMOOTH_ACCEL_V)
    soft_jerk = interp(smoothness, STOP_SMOOTH_BP, STOP_SMOOTH_JERK_V)
    # 1) 限制「越踩越深」的速率（jerk）→ 輕柔觸地
    accel = max(accel, accel_last - soft_jerk * dt_step)
    # 2) 封頂最終煞車量，不准比 soft 目標更深 → 去除 -2.0 的下壓
    accel = max(accel, soft_stop_accel)
    return accel, accel
```

### 4.3 整合點（`opendbc_repo/opendbc/car/ford/carcontroller.py`）

- **`__init__`（約 line 131 附近）** 新增：
  ```python
  self.bp_stop_smoothness = 0.0
  self.stop_accel_last = 0.0
  ```
- **即時讀參數區（約 line 254-259）** 新增（比照 `LC_PID_gain_UI` 讀法）：
  ```python
  self.bp_stop_smoothness = float(self.params.get("bp_stop_smoothness", return_default=True))
  ```
- **套用點**：在最終 `accel` 決定之後（`carcontroller.py:897-916` 的 bp/op fallback 之後）、`if brake_actuate`（line 919）與 `clip`（line 923）之前。`stopping` 變數已存在（line 761）：
  ```python
  accel, self.stop_accel_last = apply_stop_smoothing(
      accel, stopping, CC.longActive, self.bp_stop_smoothness, self.stop_accel_last,
      CarControllerParams.ACC_CONTROL_STEP * DT_CTRL,   # = 2 * 0.01 = 0.02s（50Hz）
  )
  ```
- **brake_actuate 一致性**：整形只會讓 `accel` 變得**較不負**，但最柔下限 -0.8 仍遠低於 `brake_actuate_release = -0.06`（line 142），故 `brake_actuate`/`precharge` 維持 engaged，不需重算、無前後不一致。

## 5. 安全設計與取捨

| 設計 | 說明 |
|---|---|
| 唯一作用條件 | `CC.longActive` 且 `longControlState == stopping`。stopping = 平順停到靜止（舒適情境），**非緊急煞車**（緊急走 pid 大減速、不進 stopping）→ 此為安全邊界 |
| 只變柔不變猛 | 全程 `max(accel, …)`，只會讓煞車更輕；永不加深、永不給油/加速 |
| 保壓下限 | 最柔僅到 **-0.8 m/s²**，平地（含輕微坡）仍穩穩 hold，不會放到溜車 |
| 不碰 AEB | 原廠 cmbb/AEB 走獨立路徑（`opendbc_repo/opendbc/safety/modes/ford.h:415` 不阻擋），完全不受影響 |
| 不影響一般煞車 | 非 stopping 狀態完全 no-op → 高速/一般制動權威 100% 保留 |
| 不動任何 OP Long 功能 | 作用在「最終煞車整形」層，Chill / Experimental(E2E) / BP long / stop-and-go / 自動起步全照舊 |
| 預設零改變 | `bp_stop_smoothness=0` → 行為與現狀完全相同，可完全回復 |

**已知取捨**：滑桿開越大，停止越柔，但車**會多滑一點點才完全停住**（停車距離略增）；保壓變軟（平地足夠）。斜坡長坡且高 smoothness 時保壓可能不足 → 使用者情境為平地，floor -0.8 保守。

## 6. 測試

**純函式 smoke test**（依專案慣例放 `./tests/smoke/`，檔名 `tests/smoke/test_ford_stop_smoothing.py`）：

| 案例 | 預期 |
|---|---|
| `smoothness=0`（預設） | 輸出 == 輸入（任何狀態皆 no-op）|
| 非 stopping（s>0） | 輸出 == 輸入（一般煞車制動力 100% 保留）|
| stopping 且 s>0 | 輸出不比 soft 目標更深；每周期加深速率 ≤ soft_jerk；輸出 ≥ 輸入（只變柔不變猛）|
| stopping 餵極深煞車（-3.5） | 仍夾到 soft 目標（永不增加制動力）|
| 進入 stopping 接續 | 從 `accel_last` 平滑開始、不跳動 |
| s=1 最柔 | soft 目標 = -0.8（仍 hold，不為 0）|

**回歸保護**：跑既有 `opendbc_repo/opendbc/car/ford/tests/test_ford.py` 確認未弄壞 Ford 既有行為。

**實車手感驗證**（無法自動化）：使用者上路用滑桿從 0 往上調至滿意（此即選 UI 可調的原因）。

## 7. 影響檔案

| 檔案 | 變更 |
|---|---|
| `opendbc_repo/opendbc/car/ford/helpers.py` | 新增純函式 `apply_stop_smoothing` 與常數 |
| `opendbc_repo/opendbc/car/ford/carcontroller.py` | `__init__` 加狀態變數；參數區讀 `bp_stop_smoothness`；套用點呼叫純函式 |
| `selfdrive/ui/bp/layouts/settings/bluepilot.py` | Longitudinal Tuning 區段加 `Stop Smoothness` 滑桿 |
| `common/params_keys.h` | 宣告 `bp_stop_smoothness`（預設 "0.0"）|
| `tests/smoke/test_ford_stop_smoothing.py` | 新增純函式 smoke test |
| （選配）`selfdrive/ui/bp/mici/layouts/settings/bluepilot.py` | mici 機型一致性 |

## 8. 回復方式

- 設定 `bp_stop_smoothness=0` 即恢復現狀（無需改碼）。
- 完整回復：還原上述檔案變更（純加法，無破壞性修改）。

## 9. 未來可選

- 若單一滑桿手感不夠細，可拆成兩個進階參數：`bp_stop_accel`（保壓目標）、`bp_stop_jerk`（收尾 jerk）。
- 若日後要支援斜坡，可加入 pitch 感知的保壓下限（與既有 `disable_downhill_comp_UI`/pitch 補償整合）。
