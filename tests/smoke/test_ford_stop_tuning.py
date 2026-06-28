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
