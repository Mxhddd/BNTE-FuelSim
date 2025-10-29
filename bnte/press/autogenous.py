"""Autogenous pressurization calculations."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class PressurizationInputs:
    tank_pressure: float  # Pa
    gas_temperature: float  # K
    ullage_volume_flow: float  # m^3/s
    liquid_density: float  # kg/m^3
    liquid_mass_flow: float  # kg/s
    gas_constant: float  # J/(kg*K)


@dataclass
class PressurizationResults:
    gas_mass_flow: float  # kg/s


def evaluate_pressurization(inputs: PressurizationInputs) -> PressurizationResults:
    volumetric_liquid_flow = inputs.liquid_mass_flow / inputs.liquid_density
    ullage_flow = volumetric_liquid_flow + inputs.ullage_volume_flow
    gas_mass_flow = inputs.tank_pressure * ullage_flow / (
        inputs.gas_constant * inputs.gas_temperature
    )
    return PressurizationResults(gas_mass_flow=gas_mass_flow)


__all__ = ["PressurizationInputs", "PressurizationResults", "evaluate_pressurization"]
