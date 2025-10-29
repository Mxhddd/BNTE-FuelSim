"""Feed system computations."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Union

from ..constants import G0


@dataclass
class FeedInputs:
    mass_flow: float  # kg/s
    density: float  # kg/m^3
    viscosity: float  # Pa*s
    length: float  # m
    diameter: float  # m
    roughness: float  # m
    minor_losses: Sequence[Union[float, str]]
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
    friction_model: str
    pressure_drop_friction: float
    pressure_drop_minor: float
    total_line_drop: float
    injector_area: float
    pump_power: float
    npsh_available: float
    cavitation_risk: bool


def _swamee_jain(reynolds: float, rel_roughness: float) -> float:
    return 0.25 / (
        math.log10(rel_roughness / 3.7 + 5.74 / (reynolds ** 0.9)) ** 2
    )


def _colebrook_white(reynolds: float, rel_roughness: float, initial: float) -> float:
    friction = max(initial, 1e-6)
    for _ in range(25):
        inv_sqrt = -2.0 * math.log10(
            rel_roughness / 3.7 + 2.51 / (reynolds * math.sqrt(friction))
        )
        next_friction = 1.0 / (inv_sqrt * inv_sqrt)
        if abs(next_friction - friction) < 1e-8:
            return next_friction
        friction = next_friction
    return friction


_MINOR_LOSS_LIBRARY = {
    "ball_valve": 0.05,
    "ball_valve_half": 5.0,
    "gate_valve": 0.15,
    "globe_valve": 10.0,
    "check_valve": 2.0,
    "tee_run": 0.6,
    "tee_branch": 1.8,
    "elbow_45": 0.35,
    "elbow_90": 0.75,
    "expander": 0.4,
    "contractor": 0.9,
}


def _resolve_minor_loss(loss: Union[float, str]) -> float:
    if isinstance(loss, str):
        key = loss.strip().lower()
        if key not in _MINOR_LOSS_LIBRARY:
            raise KeyError(f"Unknown minor-loss component '{loss}'")
        return _MINOR_LOSS_LIBRARY[key]
    return float(loss)


def evaluate_feed(inputs: FeedInputs) -> FeedResults:
    area = math.pi * (inputs.diameter / 2.0) ** 2
    velocity = inputs.mass_flow / (inputs.density * area)
    reynolds = inputs.density * velocity * inputs.diameter / inputs.viscosity
    rel_roughness = inputs.roughness / inputs.diameter
    if reynolds <= 0:
        friction_factor = 0.0
        friction_model = "none"
    elif reynolds < 2000.0:
        friction_factor = 64.0 / reynolds
        friction_model = "laminar"
    else:
        initial = _swamee_jain(max(reynolds, 1.0), rel_roughness)
        friction_factor = _colebrook_white(reynolds, rel_roughness, initial)
        friction_model = "colebrook"
    dynamic_pressure = 0.5 * inputs.density * velocity * velocity
    pressure_drop_friction = friction_factor * (inputs.length / inputs.diameter) * dynamic_pressure
    minor_loss_sum = sum(_resolve_minor_loss(loss) for loss in inputs.minor_losses)
    pressure_drop_minor = minor_loss_sum * dynamic_pressure
    total_line_drop = pressure_drop_friction + pressure_drop_minor
    injector_area = inputs.mass_flow / (
        inputs.injector_cd * math.sqrt(2.0 * inputs.density * inputs.injector_dp)
    )
    flow_rate = inputs.mass_flow / inputs.density
    pump_power = inputs.pump_dp * flow_rate / (inputs.pump_eff * inputs.motor_eff)
    npsh_available = (
        (inputs.tank_pressure - inputs.vapor_pressure - total_line_drop)
        / (inputs.density * G0)
        + inputs.elevation
    )
    cavitation_risk = npsh_available < 0.0
    return FeedResults(
        velocity=velocity,
        reynolds=reynolds,
        friction_factor=friction_factor,
        friction_model=friction_model,
        pressure_drop_friction=pressure_drop_friction,
        pressure_drop_minor=pressure_drop_minor,
        total_line_drop=total_line_drop,
        injector_area=injector_area,
        pump_power=pump_power,
        npsh_available=npsh_available,
        cavitation_risk=cavitation_risk,
    )


__all__ = ["FeedInputs", "FeedResults", "evaluate_feed"]
