"""Unit tests for feed system calculations."""
from __future__ import annotations

import math

import pytest

from bnte.feed.system import FeedInputs, evaluate_feed


def _base_inputs(**overrides):
    base = dict(
        mass_flow=5.0,
        density=70.0,
        viscosity=1.0e-5,
        length=8.0,
        diameter=0.05,
        roughness=1.0e-5,
        minor_losses=[0.5],
        injector_cd=0.8,
        injector_dp=2.0e6,
        pump_dp=3.0e6,
        pump_eff=0.7,
        motor_eff=0.9,
        tank_pressure=4.0e6,
        vapor_pressure=1.0e5,
        elevation=2.0,
    )
    base.update(overrides)
    return FeedInputs(**base)


def test_colebrook_matches_swamee_high_re():
    inputs = _base_inputs()
    results = evaluate_feed(inputs)
    assert results.friction_model == "colebrook"
    sj = 0.25 / (
        math.log10(inputs.roughness / inputs.diameter / 3.7 + 5.74 / (results.reynolds ** 0.9))
        ** 2
    )
    assert math.isclose(results.friction_factor, sj, rel_tol=0.05)


def test_minor_loss_library_entries():
    inputs = _base_inputs(minor_losses=["elbow_90", "ball_valve", 0.2])
    results = evaluate_feed(inputs)
    dynamic_pressure = 0.5 * inputs.density * (results.velocity ** 2)
    expected_k = 0.75 + 0.05 + 0.2
    assert math.isclose(results.pressure_drop_minor, expected_k * dynamic_pressure)


def test_cavitation_guard_flags_negative_margin():
    inputs = _base_inputs(tank_pressure=1.5e5, vapor_pressure=1.0e5, minor_losses=[0.5, 0.5])
    results = evaluate_feed(inputs)
    assert results.cavitation_risk is True
    assert results.npsh_available < 0.0


def test_unknown_minor_loss_raises():
    inputs = _base_inputs(minor_losses=["unknown_widget"])
    with pytest.raises(KeyError):
        evaluate_feed(inputs)
