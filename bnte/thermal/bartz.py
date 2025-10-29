"""Bartz heat transfer model."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class BartzInputs:
    throat_diameter: float  # m
    gas_viscosity: float  # Pa*s
    gas_cp: float  # J/(kg*K)
    prandtl: float
    chamber_temperature: float  # K
    wall_temperature: float  # K
    gamma: float
    mach: float
    characteristic_velocity: float  # m/s
    chamber_pressure: float  # Pa
    area_ratio_local: float
    radius_of_curvature: float  # m


@dataclass
class BartzResults:
    heat_transfer_coeff: float  # W/m^2/K
    heat_flux: float  # W/m^2


def evaluate_bartz(inputs: BartzInputs) -> BartzResults:
    sigma = (
        0.5 * (inputs.wall_temperature / inputs.chamber_temperature) + 0.5
    ) ** -0.68 * (1.0 + 0.5 * (inputs.gamma - 1.0) * inputs.mach ** 2) ** -0.12
    term = 0.026 * inputs.throat_diameter ** -0.2 * inputs.gas_viscosity ** 0.2
    term *= inputs.gas_cp / inputs.prandtl ** 0.6
    term *= (inputs.chamber_pressure / inputs.characteristic_velocity) ** 0.8
    term *= inputs.area_ratio_local ** 0.9
    term *= (inputs.throat_diameter / inputs.radius_of_curvature) ** 0.1
    heat_transfer_coeff = term * sigma
    heat_flux = heat_transfer_coeff * (inputs.chamber_temperature - inputs.wall_temperature)
    return BartzResults(heat_transfer_coeff=heat_transfer_coeff, heat_flux=heat_flux)


__all__ = ["BartzInputs", "BartzResults", "evaluate_bartz"]
