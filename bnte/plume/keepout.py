"""Plume keep-out calculations."""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..constants import E_CHARGE


@dataclass
class PlumeInputs:
    exponent: float
    reference_flux: float  # W/m^2 at theta=0, r=1 m
    reference_current: float  # A/m^2 at theta=0, r=1 m
    sputter_yield: float
    ion_energy: float  # eV


@dataclass
class PlumeResults:
    heat_flux: float  # W/m^2
    ion_flux: float  # 1/m^2/s
    sputter_rate: float  # m/s (assuming 1 density unit)


def evaluate_plume(inputs: PlumeInputs, theta: float, radius: float) -> PlumeResults:
    angular = math.cos(theta) ** inputs.exponent
    heat_flux = inputs.reference_flux * angular / (radius**2)
    current_density = inputs.reference_current * angular / (radius**2)
    ion_flux = current_density / E_CHARGE
    sputter_rate = ion_flux * inputs.sputter_yield
    return PlumeResults(heat_flux=heat_flux, ion_flux=ion_flux, sputter_rate=sputter_rate)


__all__ = ["PlumeInputs", "PlumeResults", "evaluate_plume"]
