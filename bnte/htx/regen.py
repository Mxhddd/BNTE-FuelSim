"""Regenerative cooling margin calculations."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass
class RegenInputs:
    coolant_mass_flow: float  # kg/s
    coolant_latent_heat: float  # J/kg
    coolant_cp: float  # J/(kg*K)
    inlet_temperature: float  # K
    outlet_temperature: float  # K
    wall_temperature: float  # K
    peak_heat_flux: float  # W/m^2
    reference_area: float  # m^2


@dataclass
class RegenResults:
    margin: float
    required_split: float


def evaluate_regen(inputs: RegenInputs) -> RegenResults:
    available = inputs.coolant_mass_flow * (
        inputs.coolant_latent_heat + inputs.coolant_cp * (inputs.outlet_temperature - inputs.inlet_temperature)
    )
    required = inputs.peak_heat_flux * inputs.reference_area
    margin = available / required
    required_split = inputs.peak_heat_flux * inputs.reference_area / (
        inputs.coolant_latent_heat + inputs.coolant_cp * (inputs.outlet_temperature - inputs.inlet_temperature)
    ) / inputs.coolant_mass_flow
    return RegenResults(margin=margin, required_split=required_split)


__all__ = ["RegenInputs", "RegenResults", "evaluate_regen"]
