from __future__ import annotations

from bnte.ep.system import EPDurationInputs, EPInputs, evaluate_ep, evaluate_ep_duration


def test_ep_performance_matches_targets() -> None:
    inputs = EPInputs(
        power_in=100000.0,
        thruster_efficiency=0.6,
        ppu_efficiency=0.95,
        specific_impulse=3000.0,
        radiator_emissivity=0.9005,
        radiator_temperature=500.0,
    )
    results = evaluate_ep(inputs)
    assert abs(results.thrust - 4.08) < 0.02
    assert abs(results.mass_flow - 1.387e-4) < 1e-6
    assert abs(results.waste_power - 45000.0) < 500.0

    duration = evaluate_ep_duration(
        EPDurationInputs(
            delta_v=3000.0,
            initial_mass=24959.6,
            ep_inputs=inputs,
        )
    )
    assert abs(duration.duration / 86400.0 - 202.0) < 5.0
