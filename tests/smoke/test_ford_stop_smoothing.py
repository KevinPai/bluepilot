# Smoke tests for the Ford final-stop brake smoothing helper (anti nose-dive).
import pytest

from opendbc.car.ford.helpers import apply_stop_smoothing

DT = 0.02  # ACC_CONTROL_STEP (2) * DT_CTRL (0.01) = 50 Hz step


def test_smoothness_zero_is_noop():
  # Default param value must not change behavior in any state.
  for stopping in (True, False):
    accel, _ = apply_stop_smoothing(-2.0, stopping, True, 0.0, 0.0, DT)
    assert accel == -2.0
  # negative smoothness is also a no-op
  accel_neg, _ = apply_stop_smoothing(-2.0, True, True, -1.0, 0.0, DT)
  assert accel_neg == -2.0


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


def test_positive_accel_last_never_commands_gas():
  # Even if the previous accel was positive, stopping output must never be gas (> 0).
  accel, _ = apply_stop_smoothing(0.1, True, True, 1.0, 0.05, DT)
  assert accel == 0.0


def test_interior_smoothness_interpolates():
  # smoothness 0.5 -> soft_stop_accel -1.4, soft_jerk 2.0; from accel_last -1.4 the
  # floor governs and output settles at -1.4.
  accel, last = apply_stop_smoothing(-2.0, True, True, 0.5, -1.4, DT)
  assert accel == pytest.approx(-1.4, abs=1e-9)
  assert last == pytest.approx(-1.4, abs=1e-9)


def test_smoothness_above_one_clamps_to_endpoint():
  # np.interp clamps out-of-range smoothness to the table endpoint (-0.8), never extrapolates.
  accel, _ = apply_stop_smoothing(-2.0, True, True, 1.5, -0.8, DT)
  assert accel == pytest.approx(-0.8, abs=1e-9)


def test_entry_continuity_is_jerk_limited():
  # Entering stopping from a light brake (-0.3), the brake deepens by at most
  # soft_jerk*dt (0.5*0.02 = 0.01) per 50 Hz step toward the -0.8 floor.
  last = -0.3
  accel, last = apply_stop_smoothing(-2.0, True, True, 1.0, last, DT)
  assert accel == pytest.approx(-0.31, abs=1e-9)
  accel, last = apply_stop_smoothing(-2.0, True, True, 1.0, last, DT)
  assert accel == pytest.approx(-0.32, abs=1e-9)
