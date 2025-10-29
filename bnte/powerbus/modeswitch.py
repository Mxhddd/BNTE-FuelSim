"""Power bus dynamics for mode switching."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class ModeSwitchInputs:
    bus_voltage: float  # V
    step_power: float  # W
    step_time: float  # s
    resistor: float  # Ohm
    droop_voltage: float  # allowable droop V


@dataclass
class ModeSwitchResults:
    required_capacitance: float  # F
    peak_current: float  # A


def evaluate_mode_switch(inputs: ModeSwitchInputs) -> ModeSwitchResults:
    required_capacitance = inputs.step_power * inputs.step_time / (
        inputs.bus_voltage * inputs.droop_voltage
    )
    peak_current = inputs.bus_voltage / inputs.resistor
    return ModeSwitchResults(
        required_capacitance=required_capacitance,
        peak_current=peak_current,
    )


__all__ = ["ModeSwitchInputs", "ModeSwitchResults", "evaluate_mode_switch"]
