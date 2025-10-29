"""Nozzle performance calculations."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from ..constants import G0


@dataclass
class NozzleInputs:
    chamber_pressure: float  # Pa
    chamber_temperature: float  # K
    gamma: float
    gas_constant: float  # J/(kg*K)
    expansion_ratio: float
    mass_flow: float  # kg/s
    ambient_pressure: float  # Pa


@dataclass
class NozzlePerformance:
    thrust: float  # N
    mass_flow: float  # kg/s
    specific_impulse: float  # s
    characteristic_velocity: float  # m/s
    thrust_coefficient: float
    throat_area: float  # m^2
    exit_mach: float
    exit_temperature: float  # K
    exit_pressure: float  # Pa
    exit_density: float  # kg/m^3
    exit_velocity: float  # m/s


def _area_ratio(mach: float, gamma: float) -> float:
    term = (2.0 / (gamma + 1.0)) * (1.0 + 0.5 * (gamma - 1.0) * mach * mach)
    exponent = (gamma + 1.0) / (2.0 * (gamma - 1.0))
    return (1.0 / mach) * term**exponent


def _mach_from_area_ratio(area_ratio: float, gamma: float) -> float:
    lo = 1.0 + 1e-6
    hi = 1.5
    while _area_ratio(hi, gamma) < area_ratio:
        hi *= 1.5
        if hi > 200.0:
            raise ValueError("Unable to bracket supersonic Mach number for given area ratio")
    for _ in range(256):
        mid = 0.5 * (lo + hi)
        if _area_ratio(mid, gamma) > area_ratio:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def _characteristic_velocity(gamma: float, gas_constant: float, temperature: float) -> float:
    term = math.sqrt(gamma) * (2.0 / (gamma + 1.0)) ** ((gamma + 1.0) / (2.0 * (gamma - 1.0)))
    return math.sqrt(temperature * gas_constant) / term


def compute_nozzle_performance(inputs: NozzleInputs) -> NozzlePerformance:
    """Compute nozzle performance metrics for a finite expansion ratio nozzle."""

    at = inputs.mass_flow * _characteristic_velocity(
        inputs.gamma, inputs.gas_constant, inputs.chamber_temperature
    ) / inputs.chamber_pressure
    ae = at * inputs.expansion_ratio
    me = _mach_from_area_ratio(inputs.expansion_ratio, inputs.gamma)
    te = inputs.chamber_temperature / (1.0 + 0.5 * (inputs.gamma - 1.0) * me * me)
    pe = inputs.chamber_pressure * (te / inputs.chamber_temperature) ** (
        inputs.gamma / (inputs.gamma - 1.0)
    )
    rho_e = pe / (inputs.gas_constant * te)
    v_e = me * math.sqrt(inputs.gamma * inputs.gas_constant * te)

    thrust = inputs.mass_flow * v_e + (pe - inputs.ambient_pressure) * ae
    isp = thrust / (inputs.mass_flow * G0)
    c_star = _characteristic_velocity(inputs.gamma, inputs.gas_constant, inputs.chamber_temperature)
    cf = thrust / (inputs.chamber_pressure * at)

    return NozzlePerformance(
        thrust=thrust,
        mass_flow=inputs.mass_flow,
        specific_impulse=isp,
        characteristic_velocity=c_star,
        thrust_coefficient=cf,
        throat_area=at,
        exit_mach=me,
        exit_temperature=te,
        exit_pressure=pe,
        exit_density=rho_e,
        exit_velocity=v_e,
    )


__all__ = ["NozzleInputs", "NozzlePerformance", "compute_nozzle_performance"]
