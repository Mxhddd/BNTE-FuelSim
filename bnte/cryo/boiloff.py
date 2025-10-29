"""Cryogenic boil-off and zero boil-off models."""
from __future__ import annotations

from dataclasses import dataclass

from ..constants import HYDROGEN_LH_VAP


@dataclass
class CryoInputs:
    area: float  # m^2
    heat_flux: float  # W/m^2
    cop: float
    latent_heat: float = HYDROGEN_LH_VAP


@dataclass
class CryoResults:
    total_heat: float  # W
    boiloff_rate: float  # kg/s
    boiloff_rate_per_day: float  # kg/day
    zbo_power: float  # W


def evaluate_cryo(inputs: CryoInputs) -> CryoResults:
    total_heat = inputs.area * inputs.heat_flux
    boiloff_rate = total_heat / inputs.latent_heat
    boiloff_rate_per_day = boiloff_rate * 86400.0
    zbo_power = total_heat / inputs.cop if inputs.cop > 0 else 0.0
    return CryoResults(
        total_heat=total_heat,
        boiloff_rate=boiloff_rate,
        boiloff_rate_per_day=boiloff_rate_per_day,
        zbo_power=zbo_power,
    )


__all__ = ["CryoInputs", "CryoResults", "evaluate_cryo"]
