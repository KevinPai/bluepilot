# OP Long 停止收尾柔順化（抗點頭）Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 新增一個 Ford 專屬、UI 可即時調整的「Stop Smoothness」控制，讓 OP Long 在完全停止那一瞬間的煞車更柔（消除點頭），預設值為 no-op（完全等同現狀）。

**Architecture:** 把停止收尾整形寫成一個純函式 `apply_stop_smoothing`（放在 `opendbc/car/ford/helpers.py`），只在 `LongCtrlState.stopping` 時對最終 `accel` 做「封頂 + 加深 jerk 限制」，且只會讓煞車更柔、不會更猛。Ford carcontroller 即時讀取 UI 參數 `bp_stop_smoothness` 並呼叫該函式；UI 在 BluePilot → Longitudinal Tuning 加一個滑桿。

**Tech Stack:** Python（opendbc/openpilot 衍生）、numpy、cereal params、pytest、BluePilot raylib UI（`float_control_item`）。

## Global Constraints

- 程式註解一律使用英文（comments in English）。
- Python 檔案沿用既有 snake_case 命名慣例（codebase convention，覆蓋一般 camelCase 偏好）。
- 參數 `bp_stop_smoothness` 預設值必須為 `"0.0"` → 安裝後行為與現狀完全相同（opt-in）。
- 整形邏輯只能**減少**煞車（永不加深、永不給油/加速），且只在 `CC.longActive and longControlState == LongCtrlState.stopping` 作用；最柔保壓下限為 `-0.8 m/s²`。
- 僅限 Ford：**不得**修改 `selfdrive/controls/lib/longcontrol.py`、`opendbc_repo/opendbc/car/interfaces.py` 或任何跨平台控制碼。
- 不得改動原廠 AEB（cmbb）路徑。
- 每個 commit 訊息結尾加上：`Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`。

---

### Task 1: 純函式 `apply_stop_smoothing` + smoke test

**Files:**
- Test: `tests/smoke/test_ford_stop_smoothing.py`（Create）
- Modify: `opendbc_repo/opendbc/car/ford/helpers.py`（新增 import 與函式）

**Interfaces:**
- Consumes: 無（純函式，只用 numpy `interp`）
- Produces：
  ```
  apply_stop_smoothing(accel: float, stopping: bool, long_active: bool,
                       smoothness: float, accel_last: float, dt_step: float) -> tuple[float, float]
  # 回傳 (shaped_accel, new_accel_last)
  ```

- [ ] **Step 1: 寫失敗測試**

建立 `tests/smoke/test_ford_stop_smoothing.py`：

```python
# Smoke tests for the Ford final-stop brake smoothing helper (anti nose-dive).
import pytest

from opendbc.car.ford.helpers import apply_stop_smoothing

DT = 0.02  # ACC_CONTROL_STEP (2) * DT_CTRL (0.01) = 50 Hz step


def test_smoothness_zero_is_noop():
  # Default param value must not change behavior in any state.
  for stopping in (True, False):
    accel, _ = apply_stop_smoothing(-2.0, stopping, True, 0.0, 0.0, DT)
    assert accel == -2.0


def test_not_stopping_preserves_full_braking():
  # Outside the stopping phase, braking authority is untouched even with smoothing maxed.
  accel, _ = apply_stop_smoothing(-3.5, False, True, 1.0, 0.0, DT)
  assert accel == -3.5


def test_not_long_active_is_noop():
  accel, _ = apply_stop_smoothing(-3.5, True, False, 1.0, 0.0, DT)
  assert accel == -3.5


def test_stopping_caps_final_brake():
  # At max smoothness the brake is capped at -0.8, never the stock -2.0 dive.
  accel, last = apply_stop_smoothing(-2.0, True, True, 1.0, -0.8, DT)
  assert accel == pytest.approx(-0.8, abs=1e-9)
  assert last == pytest.approx(-0.8, abs=1e-9)


def test_stopping_never_increases_braking():
  # Feeding a very deep brake while stopping -> clamped softer, never harder.
  accel, _ = apply_stop_smoothing(-3.5, True, True, 1.0, -0.8, DT)
  assert accel >= -0.8 - 1e-9


def test_jerk_limits_deepening_rate():
  # smoothness 1.0 -> soft_jerk 0.5 -> max deepen 0.5 * 0.02 = 0.01 per step.
  accel, last = apply_stop_smoothing(-1.0, True, True, 1.0, -0.5, DT)
  assert accel == pytest.approx(-0.51, abs=1e-9)
  assert last == pytest.approx(-0.51, abs=1e-9)


def test_floor_still_holds_at_max_smoothness():
  # Softest setting still commands a non-zero holding brake.
  accel, _ = apply_stop_smoothing(-2.0, True, True, 1.0, -0.8, DT)
  assert accel < 0.0
```

- [ ] **Step 2: 跑測試確認失敗**

Run: `python -m pytest tests/smoke/test_ford_stop_smoothing.py -v`
Expected: FAIL — collection/ImportError：`cannot import name 'apply_stop_smoothing' from 'opendbc.car.ford.helpers'`

- [ ] **Step 3: 實作純函式**

在 `opendbc_repo/opendbc/car/ford/helpers.py` 頂部 import 區（目前是 `from opendbc.car import DT_CTRL`，line 4）之後新增：

```python
from numpy import interp
```

在檔案中（例如 `apply_creep_compensation` 風格的工具函式區，放在 `hysteresis` 之前或檔尾皆可）新增：

```python
# Stop-smoothing lookup tables: 0.0 = stock behavior, 1.0 = softest stop.
_STOP_SMOOTH_BP = [0.0, 1.0]
_STOP_SMOOTH_ACCEL_V = [-2.0, -0.8]  # m/s^2: cap on final brake / standstill hold
_STOP_SMOOTH_JERK_V = [3.5, 0.5]     # m/s^3: max rate the brake may deepen near the stop


def apply_stop_smoothing(accel, stopping, long_active, smoothness, accel_last, dt_step):
  """Soften the final-stop braking to reduce nose-dive.

  Only ever reduces braking (never deepens, never adds gas), and only while the car is
  coming to rest (LongCtrlState.stopping). Pure function for testability.
  Returns (shaped_accel, new_accel_last).
  """
  # No-op: feature off, not engaged, or not in the stopping phase -> preserve full braking authority.
  if smoothness <= 0.0 or not long_active or not stopping:
    return accel, accel

  soft_stop_accel = float(interp(smoothness, _STOP_SMOOTH_BP, _STOP_SMOOTH_ACCEL_V))
  soft_jerk = float(interp(smoothness, _STOP_SMOOTH_BP, _STOP_SMOOTH_JERK_V))

  # 1) Limit how fast the brake may deepen -> gentle touchdown.
  accel = max(accel, accel_last - soft_jerk * dt_step)
  # 2) Cap the final brake / hold so it never dives to the stock -2.0 m/s^2.
  accel = max(accel, soft_stop_accel)
  return accel, accel
```

- [ ] **Step 4: 跑測試確認通過**

Run: `python -m pytest tests/smoke/test_ford_stop_smoothing.py -v`
Expected: PASS（7 passed）

> 註：smoke test 在 repo 標準 Python 環境執行（comma 裝置 / Linux / CI）。函式為純運算、與環境無關；若在 Windows dev 機因 openpilot import chain 無法載入，請於裝置或 WSL 上跑。

- [ ] **Step 5: Commit**

```bash
git add tests/smoke/test_ford_stop_smoothing.py opendbc_repo/opendbc/car/ford/helpers.py
git commit -m "feat(ford): add apply_stop_smoothing helper for gentle final stop

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: 宣告參數 + 接進 Ford carcontroller

**Files:**
- Modify: `common/params_keys.h:292`（在 `disable_downhill_comp_UI` 之後新增一行）
- Modify: `opendbc_repo/opendbc/car/ford/carcontroller.py`（import、`__init__`、參數讀取、整合呼叫）

**Interfaces:**
- Consumes: `apply_stop_smoothing(...)`（Task 1）；`CarControllerParams.ACC_CONTROL_STEP`、`DT_CTRL`（已 import）；`stopping` 區域變數（carcontroller.py:761 已存在）；`CC.longActive`
- Produces: 無（行為整合，無對外新簽名）

- [ ] **Step 1: 宣告 param**

在 `common/params_keys.h` 第 292 行 `{"disable_downhill_comp_UI", {PERSISTENT | BACKUP, BOOL, "0"}},` 之後新增：

```c
    {"bp_stop_smoothness", {PERSISTENT | BACKUP, FLOAT, "0.0"}},
```

- [ ] **Step 2: 驗證 param 預設可讀取**

Run: `python -c "from openpilot.common.params import Params; print(repr(Params().get('bp_stop_smoothness', return_default=True)))"`
Expected: 印出 `'0.0'`（或 `b'0.0'`），不可拋 `UnknownKeyName`

- [ ] **Step 3: carcontroller import 整形函式**

在 `opendbc_repo/opendbc/car/ford/carcontroller.py:17` 的：

```python
from opendbc.car.ford.helpers import compute_dm_msg_values
```

改為：

```python
from opendbc.car.ford.helpers import compute_dm_msg_values, apply_stop_smoothing
```

- [ ] **Step 4: `__init__` 新增狀態變數**

在 `__init__` 中 `self.disable_BP_long_UI = False`（carcontroller.py:131）之後新增：

```python
    self.bp_stop_smoothness = 0.0  # updated from UI: 0 = stock, 1 = softest final stop
    self.stop_accel_last = 0.0     # tracks shaped stop accel for jerk-limited touchdown
```

- [ ] **Step 5: 參數即時讀取**

在參數讀取區 `self.disable_downhill_comp_UI = self.params.get_bool("disable_downhill_comp_UI")`（carcontroller.py:259）之後新增：

```python
    self.bp_stop_smoothness = float(self.params.get("bp_stop_smoothness", return_default=True))
```

- [ ] **Step 6: 套用整形（最終 accel 決定後、brake/gas 互斥前）**

在 carcontroller.py 第 916 行（縱向 if/else 區塊結束、`accel` 已最終決定）與第 918 行註解 `# no brake and gas at the same timne` 之間插入：

```python
      # BluePilot: soften the final stop to reduce nose-dive. No-op when bp_stop_smoothness == 0,
      # and only ever reduces braking while in LongCtrlState.stopping (see helpers.apply_stop_smoothing).
      accel, self.stop_accel_last = apply_stop_smoothing(
        accel, stopping, CC.longActive, self.bp_stop_smoothness, self.stop_accel_last,
        CarControllerParams.ACC_CONTROL_STEP * DT_CTRL,
      )
```

> 一致性說明：整形只會讓 `accel` 變得較不負；最柔下限 -0.8 仍遠低於 `brake_actuate_release = -0.06`（carcontroller.py:142），故先前算好的 `brake_actuate`/`precharge` 仍維持 engaged，無需重算。此插入點仍在 `if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:`（carcontroller.py:721）區塊內。

- [ ] **Step 7: 語法/匯入 smoke 檢查**

Run: `python -c "import ast; ast.parse(open('opendbc_repo/opendbc/car/ford/carcontroller.py').read()); print('carcontroller OK')"`
Expected: 印出 `carcontroller OK`（無 SyntaxError）

- [ ] **Step 8: Ford 既有測試回歸**

Run: `python -m pytest opendbc_repo/opendbc/car/ford/tests/test_ford.py -q`
Expected: PASS（與修改前相同；無新增失敗）

> 若該測試集在 dev 機環境無法執行，於裝置/CI 跑；Step 7 的 AST 檢查至少保證無語法錯誤。

- [ ] **Step 9: Commit**

```bash
git add common/params_keys.h opendbc_repo/opendbc/car/ford/carcontroller.py
git commit -m "feat(ford): wire bp_stop_smoothness into carcontroller stop shaping

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: BluePilot UI 滑桿「Stop Smoothness」

**Files:**
- Modify: `selfdrive/ui/bp/layouts/settings/bluepilot.py`（新增 `float_control_item` 並加入 Longitudinal Tuning 區段）

**Interfaces:**
- Consumes: param `bp_stop_smoothness`（Task 2 宣告）；既有 `float_control_item`（已 import，bluepilot.py:15）
- Produces: 無

- [ ] **Step 1: 新增滑桿定義**

在 `_initialize_items` 中，`self._disable_dowhill_comp = toggle_item(...)` 區塊（bluepilot.py:374-380）之後新增：

```python
    # Stop smoothness slider (anti nose-dive at final stop; 0 = stock behavior)
    self._stop_smoothness = float_control_item(
      lambda: tr("Stop Smoothness"),
      lambda: tr("Soften braking at the final stop to reduce nose-dive (0 = stock, higher = softer)."),
      param="bp_stop_smoothness",
      min_value=0.0,
      max_value=1.0,
      step=0.05,
      icon="chffr_wheel.png",
    )
```

- [ ] **Step 2: 加入 Longitudinal Tuning 區段清單**

在 return 的 menu list（bluepilot.py:421-423）中：

```python
      SectionHeader(tr("Longitudinal Tuning")),
      self._disable_BP_long,
      self._disable_dowhill_comp,
```

於 `self._disable_dowhill_comp,` 之後新增一行：

```python
      self._stop_smoothness,
```

- [ ] **Step 3: 語法 smoke 檢查**

Run: `python -c "import ast; ast.parse(open('selfdrive/ui/bp/layouts/settings/bluepilot.py').read()); print('bluepilot ui OK')"`
Expected: 印出 `bluepilot ui OK`（無 SyntaxError）

- [ ] **Step 4: 參數來回讀寫驗證**

Run: `python -c "from openpilot.common.params import Params; p=Params(); p.put('bp_stop_smoothness','0.5'); print(p.get('bp_stop_smoothness', return_default=True)); p.put('bp_stop_smoothness','0.0')"`
Expected: 印出 `0.5`（或 `b'0.5'`），最後重設回 `0.0`

- [ ] **Step 5: Commit**

```bash
git add selfdrive/ui/bp/layouts/settings/bluepilot.py
git commit -m "feat(ui): add Stop Smoothness slider under BluePilot Longitudinal Tuning

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## 實車驗證（人工，非自動化）

部署到裝置後，開啟 OP Long，於 BluePilot → Longitudinal Tuning 把「Stop Smoothness」從 0 慢慢往上調，於塞車/紅燈跟停時感受最終停止是否更柔、後座是否改善；確認停住後仍能在平地穩住不溜車、且久停後自動起步行為不變。

## 自我審查結果（plan vs spec）

- **Spec §4.1 UI 滑桿** → Task 3 ✅
- **Spec §4.2 純函式整形** → Task 1 ✅
- **Spec §4.3 carcontroller 整合（init/讀參數/套用點）** → Task 2 Steps 3-6 ✅
- **Spec §4.1 param 宣告（預設 0.0）** → Task 2 Step 1 ✅
- **Spec §5 安全（只變柔、只在 stopping、floor -0.8、預設 no-op）** → 由 `apply_stop_smoothing` 實作 + Task 1 測試涵蓋（`test_not_stopping_preserves_full_braking`、`test_stopping_never_increases_braking`、`test_floor_still_holds_at_max_smoothness`、`test_smoothness_zero_is_noop`）✅
- **Spec §6 測試（smoke + 回歸）** → Task 1 smoke、Task 2 Step 8 回歸 ✅
- **Placeholder 掃描**：無 TBD/TODO；每個 code step 皆有完整程式碼 ✅
- **型別一致性**：`apply_stop_smoothing` 在 Task 1 定義、Task 2 Step 3/6 以相同 6 參數簽名呼叫、回傳 `(accel, stop_accel_last)` 一致 ✅
- mici UI 變體列為 spec 選配，本計畫未納入（YAGNI；需要時另開小 task）
