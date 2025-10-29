"""Feed system computations."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from ..constants import G0


@dataclass
class FeedInputs:
    mass_flow: float  # kg/s
    density: float  # kg/m^3
    viscosity: float  # Pa*s
    length: float  # m
    diameter: float  # m
    roughness: float  # m
    minor_losses: Sequence[float]
    injector_cd: float
    injector_dp: float  # Pa
    pump_dp: float  # Pa
    pump_eff: float
    motor_eff: float
    tank_pressure: float  # Pa
    vapor_pressure: float  # Pa
    elevation: float  # m


@dataclass
class FeedResults:
    velocity: float
    reynolds: float
    friction_factor: float
    pressure_drop_friction: float
    pressure_drop_minor: float
    total_line_drop: float
    injector_area: float
    pump_power: float
    npsh_available: float


def _swamee_jain(reynolds: float, rel_roughness: float) -> float:
    return 0.25 / (
        math.log10(rel_roughness / 3.7 + 5.74 / (reynolds ** 0.9)) ** 2
    )


def evaluate_feed(inputs: FeedInputs) -> FeedResults:
    area = math.pi * (inputs.diameter / 2.0) ** 2
    velocity = inputs.mass_flow / (inputs.density * area)
    reynolds = inputs.density * velocity * inputs.diameter / inputs.viscosity
    rel_roughness = inputs.roughness / inputs.diameter
    friction_factor = _swamee_jain(reynolds, rel_roughness)
    dynamic_pressure = 0.5 * inputs.density * velocity * velocity
    pressure_drop_friction = friction_factor * (inputs.length / inputs.diameter) * dynamic_pressure
    pressure_drop_minor = sum(inputs.minor_losses) * dynamic_pressure
    total_line_drop = pressure_drop_friction + pressure_drop_minor
    injector_area = inputs.mass_flow / (
        inputs.injector_cd * math.sqrt(2.0 * inputs.density * inputs.injector_dp)
    )
    flow_rate = inputs.mass_flow / inputs.density
    pump_power = inputs.pump_dp * flow_rate / (inputs.pump_eff * inputs.motor_eff)
    npsh_available = (
        inputs.tank_pressure - inputs.vapor_pressure - total_line_drop
    ) / 1e5
    return FeedResults(
        velocity=velocity,
        reynolds=reynolds,
        friction_factor=friction_factor,
        pressure_drop_friction=pressure_drop_friction,
        pressure_drop_minor=pressure_drop_minor,
        total_line_drop=total_line_drop,
        injector_area=injector_area,
        pump_power=pump_power,
        npsh_available=npsh_available,
    )


__all__ = ["FeedInputs", "FeedResults", "evaluate_feed"]
