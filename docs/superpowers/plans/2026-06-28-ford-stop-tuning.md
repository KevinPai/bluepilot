# Ford Stop Tuning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Improve Ford OP Long run-to-run stop-gap consistency by shortening the open-loop stopping window (B1 `vEgoStopping`), reducing low-speed creep compensation (B3), and latching the brake-request bit while stopping (B4), all gated by one `bp_ford_stop_tuning` toggle.

**Architecture:** Two new pure helpers in `opendbc/car/ford/helpers.py` (a parameterized `apply_creep_compensation` and a new `brake_request_hysteresis`) plus three module constants carry the tunable logic and are unit-tested in isolation. `carcontroller.py` reads the toggle live and feeds the helpers; `interface.py` reads the toggle once at init to override `vEgoStopping`. `stopAccel` is left untouched so this composes cleanly with the existing Stop Smoothness slider. A single BluePilot settings toggle (default ON) drives everything.

**Tech Stack:** Python (openpilot/opendbc car port), numpy, pytest, C++ params header.

## Global Constraints

- Branch: `ford-stop-consistency` (already created off `bp-6.0`). Do NOT merge.
- Code comments in English. Match the surrounding Ford code style: local variables are `snake_case` (e.g. `op_accel`), module constants are `UPPER_SNAKE`.
- Toggle param `bp_ford_stop_tuning` default **ON** (`"1"`).
- Do NOT modify `stopAccel`/`stoppingDecelRate` — final-stop softening stays owned by the existing `bp_stop_smoothness` slider.
- Exact tuned values: `FORD_STOP_VEGO_STOPPING = 0.25` (default 0.5), `FORD_STOP_CREEP_MAX = 0.3` (stock 0.6), `FORD_STOP_BRAKE_RELEASE_LATCHED = 0.0` (normal release -0.06).
- Pure helpers keep a stock-equivalent default argument so the OFF path is byte-for-byte identical to today.
- Commit messages end with: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`
- Spec: `docs/superpowers/specs/2026-06-28-ford-stop-consistency-design.md`

---

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `opendbc_repo/opendbc/car/ford/helpers.py` | Pure Ford long helpers + tuning constants | Add 3 constants; relocate+parameterize `apply_creep_compensation`; add `brake_request_hysteresis` |
| `opendbc_repo/opendbc/car/ford/carcontroller.py` | Wire toggle → helpers (live, low-speed path) | Import helpers; read toggle in `_update_params`; move `stopping` up; pass `ford_stop_creep_max`; latch brake bit |
| `opendbc_repo/opendbc/car/ford/interface.py` | B1 `vEgoStopping` override (init) | Import constant; guarded override in `_get_params` |
| `common/params_keys.h` | Param registry | Register `bp_ford_stop_tuning` BOOL "1" |
| `selfdrive/ui/bp/layouts/settings/bluepilot.py` (+ `mici/` variant) | Settings UI | Add "Ford Stop Tuning" toggle under Longitudinal Tuning |
| `tests/smoke/test_ford_stop_tuning.py` | Pure-function tests (matches existing `test_ford_stop_smoothing.py` precedent) | New file |

---

## Task 1: Parameterized `apply_creep_compensation` + tuning constants (helpers.py)

**Files:**
- Modify: `opendbc_repo/opendbc/car/ford/helpers.py` (add constants + relocated function near the `_STOP_SMOOTH_*` block, ~line 167)
- Modify: `opendbc_repo/opendbc/car/ford/carcontroller.py:17` (import) and remove local def at `:79-82`
- Test: `tests/smoke/test_ford_stop_tuning.py` (new)

**Interfaces:**
- Produces: `apply_creep_compensation(accel: float, v_ego: float, max_creep: float = 0.6) -> float`
- Produces: constants `FORD_STOP_VEGO_STOPPING = 0.25`, `FORD_STOP_CREEP_MAX = 0.3`, `FORD_STOP_BRAKE_RELEASE_LATCHED = 0.0`

- [ ] **Step 1: Write the failing test**

Create `tests/smoke/test_ford_stop_tuning.py`:

```python
# Smoke tests for the Ford stop-tuning helpers (B1 vEgoStopping constant, B3 creep, B4 brake-bit latch).
import pytest

from opendbc.car.ford.helpers import (
    apply_creep_compensation,
    FORD_STOP_VEGO_STOPPING,
    FORD_STOP_CREEP_MAX,
    FORD_STOP_BRAKE_RELEASE_LATCHED,
)


def test_tuning_constants():
    # B1: shorter open-loop window than the 0.5 default.
    assert FORD_STOP_VEGO_STOPPING == 0.25
    assert FORD_STOP_VEGO_STOPPING < 0.5
    # B3: reduced from the stock 0.6 creep authority.
    assert FORD_STOP_CREEP_MAX == 0.3
    assert FORD_STOP_CREEP_MAX < 0.6
    # B4: latched release sits above the normal -0.06 release.
    assert FORD_STOP_BRAKE_RELEASE_LATCHED == 0.0


def test_creep_default_arg_is_stock():
    # Default max_creep keeps today's behavior: at v=1.0, accel=0.0 -> -0.6.
    assert apply_creep_compensation(0.0, 1.0) == pytest.approx(-0.6)


def test_creep_reduced_max():
    # Tuned max_creep halves the low-speed brake bias.
    assert apply_creep_compensation(0.0, 1.0, FORD_STOP_CREEP_MAX) == pytest.approx(-0.3)


def test_creep_zero_above_3ms():
    # No creep compensation at/above 3 m/s.
    assert apply_creep_compensation(0.0, 3.0, 0.6) == pytest.approx(0.0)
    assert apply_creep_compensation(0.0, 5.0, 0.6) == pytest.approx(0.0)


def test_creep_zero_when_accel_high():
    # Above accel 0.2 the creep term tapers to zero (no bias on positive accel).
    assert apply_creep_compensation(0.2, 1.0, 0.6) == pytest.approx(0.2)


def test_creep_interpolates_speed_and_accel():
    # v=2 -> creep 0.3; accel=0.1 -> taper to 0.15; 0.1 - 0.15 = -0.05.
    assert apply_creep_compensation(0.1, 2.0, 0.6) == pytest.approx(-0.05)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/smoke/test_ford_stop_tuning.py -v`
Expected: FAIL — `ImportError: cannot import name 'apply_creep_compensation' from 'opendbc.car.ford.helpers'`

- [ ] **Step 3: Add constants + function to `helpers.py`**

In `opendbc_repo/opendbc/car/ford/helpers.py`, directly after the `apply_stop_smoothing` definition (end of file, after line 193), append:

```python

# Ford stop-tuning constants (see docs/superpowers/specs/2026-06-28-ford-stop-consistency-design.md).
FORD_STOP_VEGO_STOPPING         = 0.25   # B1: enter open-loop stopping at 0.25 m/s instead of 0.5
FORD_STOP_CREEP_MAX             = 0.3    # B3: low-speed creep brake authority, reduced from stock 0.6
FORD_STOP_BRAKE_RELEASE_LATCHED = 0.0    # B4: brake-bit release threshold while stopping (normal -0.06)


def apply_creep_compensation(accel: float, v_ego: float, max_creep: float = 0.6) -> float:
  # Compensate for engine creep at low speed. Either the ABS does not account for engine
  # creep, or the correction is very slow. max_creep defaults to the stock 0.6 so callers
  # that do not opt into stop tuning keep today's behavior.
  # TODO: verify this applies to EV/hybrid.
  creep_accel = interp(v_ego, [1., 3.], [max_creep, 0.])
  creep_accel = interp(accel, [0., 0.2], [creep_accel, 0.])
  return accel - creep_accel
```

- [ ] **Step 4: Relocate — update carcontroller import and remove the local def**

In `opendbc_repo/opendbc/car/ford/carcontroller.py` line 17, change:

```python
from opendbc.car.ford.helpers import compute_dm_msg_values, apply_stop_smoothing
```
to:
```python
from opendbc.car.ford.helpers import compute_dm_msg_values, apply_stop_smoothing, apply_creep_compensation
```

Then DELETE the now-duplicate local definition at lines 79-82:

```python
def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return accel
```

The call site at `carcontroller.py:733` stays `apply_creep_compensation(op_accel, CS.out.vEgo)` — with the default `max_creep=0.6` behavior is unchanged. (It gets the tuned value in Task 3.)

- [ ] **Step 5: Run tests to verify they pass + no regression**

Run: `python -m pytest tests/smoke/test_ford_stop_tuning.py tests/smoke/test_ford_stop_smoothing.py -v`
Expected: PASS (all). Then confirm carcontroller still imports:
Run: `python -c "import opendbc.car.ford.carcontroller"`
Expected: no output, exit 0.

- [ ] **Step 6: Commit**

```bash
git add opendbc_repo/opendbc/car/ford/helpers.py opendbc_repo/opendbc/car/ford/carcontroller.py tests/smoke/test_ford_stop_tuning.py
git commit -m "feat(ford): relocate+parameterize creep comp, add stop-tuning constants

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: `brake_request_hysteresis` pure helper (helpers.py)

**Files:**
- Modify: `opendbc_repo/opendbc/car/ford/helpers.py` (add function after `apply_creep_compensation`)
- Test: `tests/smoke/test_ford_stop_tuning.py` (append)

**Interfaces:**
- Consumes: nothing new.
- Produces: `brake_request_hysteresis(accel: float, last: bool, long_active: bool, engage: float, release: float) -> bool`

- [ ] **Step 1: Write the failing test**

Append to `tests/smoke/test_ford_stop_tuning.py`:

```python
from opendbc.car.ford.helpers import brake_request_hysteresis

ENGAGE = -0.14   # brake_actuate_target
RELEASE = -0.06  # brake_actuate_release (normal)


def test_hysteresis_engages_below_target():
    assert brake_request_hysteresis(-0.20, False, True, ENGAGE, RELEASE) is True


def test_hysteresis_releases_above_release():
    assert brake_request_hysteresis(0.0, True, True, ENGAGE, RELEASE) is False


def test_hysteresis_holds_in_band():
    # -0.10 is between engage and release -> hold previous state.
    assert brake_request_hysteresis(-0.10, True, True, ENGAGE, RELEASE) is True
    assert brake_request_hysteresis(-0.10, False, True, ENGAGE, RELEASE) is False


def test_hysteresis_not_long_active_forces_false():
    assert brake_request_hysteresis(-0.20, True, False, ENGAGE, RELEASE) is False


def test_latched_release_holds_through_hover_zone():
    # -0.05 would RELEASE with the normal -0.06 threshold...
    assert brake_request_hysteresis(-0.05, True, True, ENGAGE, RELEASE) is False
    # ...but with the latched release (0.0) it stays engaged through the -0.06..0 hover zone.
    assert brake_request_hysteresis(-0.05, True, True, ENGAGE, FORD_STOP_BRAKE_RELEASE_LATCHED) is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python -m pytest tests/smoke/test_ford_stop_tuning.py -k hysteresis -v`
Expected: FAIL — `ImportError: cannot import name 'brake_request_hysteresis'`

- [ ] **Step 3: Add the function to `helpers.py`**

Append after `apply_creep_compensation` in `helpers.py`:

```python


def brake_request_hysteresis(accel: float, last: bool, long_active: bool, engage: float, release: float) -> bool:
  # Two-threshold latch for the Ford brake-request bit. Engage below `engage`, release above
  # `release`, otherwise hold `last`. Raising `release` toward 0 (stop tuning) keeps the bit
  # latched through the small hover zone near a stop, preventing chatter.
  if accel > release or not long_active:
    return False
  if accel < engage:
    return True
  return last
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python -m pytest tests/smoke/test_ford_stop_tuning.py -v`
Expected: PASS (all, including the hysteresis cases).

- [ ] **Step 5: Commit**

```bash
git add opendbc_repo/opendbc/car/ford/helpers.py tests/smoke/test_ford_stop_tuning.py
git commit -m "feat(ford): add brake_request_hysteresis pure helper for stop-bit latch

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Register param + wire carcontroller (B3 creep + B4 latch)

**Files:**
- Modify: `common/params_keys.h:293` (register, after `bp_stop_smoothness`)
- Modify: `opendbc_repo/opendbc/car/ford/carcontroller.py` (import line 17; `_update_params` after 262; move `stopping` from 764; creep call 733; brake hysteresis block 757-762)

**Interfaces:**
- Consumes: `apply_creep_compensation`, `brake_request_hysteresis`, `FORD_STOP_CREEP_MAX`, `FORD_STOP_BRAKE_RELEASE_LATCHED` (Tasks 1-2).
- Produces: instance attrs `self.ford_stop_tuning: bool`, `self.ford_stop_creep_max: float`.

- [ ] **Step 1: Register the param**

In `common/params_keys.h`, immediately after line 293 (`{"bp_stop_smoothness", ...}`), add:

```c
    {"bp_ford_stop_tuning", {PERSISTENT | BACKUP, BOOL, "1"}},
```

- [ ] **Step 2: Extend the carcontroller import**

`carcontroller.py:17` becomes:

```python
from opendbc.car.ford.helpers import compute_dm_msg_values, apply_stop_smoothing, apply_creep_compensation, brake_request_hysteresis, FORD_STOP_CREEP_MAX, FORD_STOP_BRAKE_RELEASE_LATCHED
```

- [ ] **Step 3: Read the toggle in `_update_params`**

In `carcontroller.py`, after line 262 (`self.bp_stop_smoothness = ...`), add:

```python
    self.ford_stop_tuning = self.params.get_bool("bp_ford_stop_tuning")
    self.ford_stop_creep_max = FORD_STOP_CREEP_MAX if self.ford_stop_tuning else 0.6
```

- [ ] **Step 4: Move the `stopping` computation above the brake block**

Currently `stopping` is computed at line 764, AFTER the brake-bit block. Move it up. DELETE line 764:

```python
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
```

and INSERT it just before line 757 (`op_brake_actuate = self.op_brake_actuate_last`):

```python
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
```

- [ ] **Step 5: Pass the tuned creep value**

`carcontroller.py:733` becomes:

```python
        op_accel = apply_creep_compensation(op_accel, CS.out.vEgo, self.ford_stop_creep_max)
```

- [ ] **Step 6: Replace the inline brake hysteresis with the latching helper**

Replace lines 757-762:

```python
      op_brake_actuate = self.op_brake_actuate_last
      if accel_pitch_compensated > self.brake_actuate_release or not CC.longActive:
        op_brake_actuate = False
      elif accel_pitch_compensated < self.brake_actuate_target:
        op_brake_actuate = True
      # else: keep op_brake_actuate (hysteresis between 0 and 0.3)
```

with:

```python
      # B4: while stopping with tuning on, raise the release threshold so the brake bit stays
      # latched through the hover zone near a stop (anti-chatter). engage stays at -0.14.
      brake_release = self.brake_actuate_release
      if self.ford_stop_tuning and stopping:
        brake_release = FORD_STOP_BRAKE_RELEASE_LATCHED
      op_brake_actuate = brake_request_hysteresis(
        accel_pitch_compensated, self.op_brake_actuate_last, CC.longActive,
        self.brake_actuate_target, brake_release)
```

(The existing `self.op_brake_actuate_last = op_brake_actuate` assignment at line 946 is unchanged and still latches state for next frame.)

- [ ] **Step 7: Verify no regression + import check**

Run: `python -m pytest tests/smoke/test_ford_stop_tuning.py tests/smoke/test_ford_stop_smoothing.py -v`
Expected: PASS (all — pure logic for B3/B4 is covered by Tasks 1-2).
Run: `python -c "import opendbc.car.ford.carcontroller"`
Expected: exit 0, no output.

- [ ] **Step 8: Commit**

```bash
git add common/params_keys.h opendbc_repo/opendbc/car/ford/carcontroller.py
git commit -m "feat(ford): wire bp_ford_stop_tuning to creep reduction + brake-bit latch

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: B1 `vEgoStopping` override (interface.py)

**Files:**
- Modify: `opendbc_repo/opendbc/car/ford/interface.py` (import line ~9; `_get_params` before `return ret` at line 133-134)

**Interfaces:**
- Consumes: `FORD_STOP_VEGO_STOPPING` (Task 1), param `bp_ford_stop_tuning` (Task 3).
- Produces: `CarParams.vEgoStopping == 0.25` when the toggle is ON at boot.

- [ ] **Step 1: Add the helpers import**

In `interface.py`, after the existing `from opendbc.car.ford.values import ...` line (line 9), add:

```python
from opendbc.car.ford.helpers import FORD_STOP_VEGO_STOPPING
```

- [ ] **Step 2: Add the guarded override in `_get_params`**

In `interface.py`, immediately before `ret.autoResumeSng = ret.minEnableSpeed == -1.` (line 132), insert:

```python
    # BluePilot Ford stop tuning (B1): shorten the open-loop stopping window so the car tracks
    # lead distance closer to 0 -> more consistent run-to-run stop gap. Read once at init
    # (reboot to change). Guarded so offline docs generation never touches Params.
    if not docs:
      try:
        from openpilot.common.params import Params
        if Params().get_bool("bp_ford_stop_tuning"):
          ret.vEgoStopping = FORD_STOP_VEGO_STOPPING
      except Exception:
        pass
```

(`vEgoStarting` is intentionally left at 0.5 — see spec §6.)

- [ ] **Step 3: Import check**

Run: `python -c "import opendbc.car.ford.interface"`
Expected: exit 0, no output.

- [ ] **Step 4: Commit**

```bash
git add opendbc_repo/opendbc/car/ford/interface.py
git commit -m "feat(ford): override vEgoStopping to 0.25 under bp_ford_stop_tuning

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: BluePilot settings toggle (UI)

**Files:**
- Modify: `selfdrive/ui/bp/layouts/settings/bluepilot.py` (refresh-map ~line 73-74; toggle def near line 380; menu list at line 435)
- Modify: `selfdrive/ui/bp/mici/layouts/settings/bluepilot.py` (mirror, matching its existing Stop Smoothness/Longitudinal Tuning section)

**Interfaces:**
- Consumes: param `bp_ford_stop_tuning` (Task 3).
- Produces: a user-visible toggle writing `bp_ford_stop_tuning`.

- [ ] **Step 1: Define the toggle (model on `_disable_dowhill_comp`)**

In `selfdrive/ui/bp/layouts/settings/bluepilot.py`, immediately after the `self._disable_dowhill_comp = toggle_item(...)` block (ends line 380), add:

```python
    # Ford stop tuning toggle: shorten open-loop stopping + reduce creep + latch brake bit.
    self._ford_stop_tuning = toggle_item(
      lambda: tr("Ford Stop Tuning"),
      lambda: tr("More consistent stop-gap: shorter open-loop stopping window, reduced creep, latched brake bit. Reboot to fully apply (vEgoStopping). Independent of Stop Smoothness."),
      initial_state=self._safe_get_bool(self._params, "bp_ford_stop_tuning"),
      callback=lambda state: self._toggle_callback(state, "bp_ford_stop_tuning"),
      icon="chffr_wheel.png"
    )
```

- [ ] **Step 2: Add it to the refresh map**

In the toggle refresh map (the tuple list at lines ~73-74 containing `("disable_BP_long_UI", self._disable_BP_long)` and `("disable_downhill_comp_UI", self._disable_dowhill_comp)`), add a line:

```python
      ("bp_ford_stop_tuning", self._ford_stop_tuning),
```

- [ ] **Step 3: Add it to the Longitudinal Tuning menu section**

In the menu list, after `self._stop_smoothness,` (line 435), add:

```python
      self._ford_stop_tuning,
```

- [ ] **Step 4: Mirror in the mici variant**

In `selfdrive/ui/bp/mici/layouts/settings/bluepilot.py`, apply the same three edits (toggle definition, refresh-map entry, menu-section entry) following that file's existing `bp_stop_smoothness` / Longitudinal Tuning pattern. If the mici file has no Longitudinal Tuning section or no `_stop_smoothness`, place the toggle next to its closest existing Ford long toggle and note the location in the commit body.

- [ ] **Step 5: Syntax/compile check**

Run: `python -m py_compile selfdrive/ui/bp/layouts/settings/bluepilot.py selfdrive/ui/bp/mici/layouts/settings/bluepilot.py`
Expected: exit 0, no output.

- [ ] **Step 6: Commit**

```bash
git add selfdrive/ui/bp/layouts/settings/bluepilot.py selfdrive/ui/bp/mici/layouts/settings/bluepilot.py
git commit -m "feat(ui): add Ford Stop Tuning toggle under Longitudinal Tuning

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Validation Handoff (post-implementation, user-owned)

Automated pure-function tests cover B3/B4 logic and the B1 constant. The integration (toggle gating, `vEgoStopping` override on `CarParams`, accel staying within `ford.h` limits, gas never positive while braking) and the actual stop-gap consistency are validated on-vehicle by the user:

1. Enable **Ford Stop Tuning** (default ON), reboot once (for `vEgoStopping`).
2. Record before/after route logs of repeated stop-and-go behind a stopped lead.
3. Watch the spec §8 risks: jitter in the extended 0.25–0.5 m/s closed-loop band (radar dRel noise), clean full stop with the 0.25/0.5 asymmetry, and hybrid creep adequacy.

---

## Self-Review

**Spec coverage:**
- B1 `vEgoStopping` → Task 4 ✓ ; B3 creep → Tasks 1+3 ✓ ; B4 latch → Tasks 2+3 ✓ ; param default ON → Task 3 ✓ ; UI toggle (+mici) → Task 5 ✓ ; constants single-source → Task 1 ✓ ; don't touch stopAccel → respected (no task edits it) ✓ ; unit+smoke tests → Tasks 1-2 ✓ ; on-vehicle → Validation Handoff ✓ ; vEgoStarting untouched → Task 4 note ✓.
- Spec §7 also mentioned `opendbc/.../ford/tests/` unit tests; consolidated into `tests/smoke/` to match the existing `test_ford_stop_smoothing.py` precedent and the user's "smoke tests under ./tests/smoke/" rule. No coverage lost.

**Placeholder scan:** No TBD/TODO-as-work, no "add error handling", every code step shows full code. The one `try/except` is an intentional init guard (spec §3.2), not a placeholder. ✓

**Type consistency:** `apply_creep_compensation(accel, v_ego, max_creep=0.6)` and `brake_request_hysteresis(accel, last, long_active, engage, release)` signatures match between definition (Tasks 1-2), tests (Tasks 1-2), and call sites (Task 3). Constants `FORD_STOP_VEGO_STOPPING`/`FORD_STOP_CREEP_MAX`/`FORD_STOP_BRAKE_RELEASE_LATCHED` named identically across helpers, carcontroller, interface, and tests. ✓
