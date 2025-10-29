"""Electric propulsion models."""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..constants import G0, SIGMA


@dataclass
class EPInputs:
    power_in: float  # W
    thruster_efficiency: float
    ppu_efficiency: float
    specific_impulse: float  # s
    radiator_emissivity: float
    radiator_temperature: float  # K


@dataclass
class EPResults:
    exhaust_velocity: float  # m/s
    thrust: float  # N
    mass_flow: float  # kg/s
    waste_power: float  # W
    radiator_area: float  # m^2


@dataclass
class EPDurationInputs:
    delta_v: float  # m/s
    initial_mass: float  # kg
    ep_inputs: EPInputs


@dataclass
class EPDurationResults:
    duration: float  # s
    propellant_used: float  # kg


def evaluate_ep(inputs: EPInputs) -> EPResults:
    ve = G0 * inputs.specific_impulse
    thrust = 2.0 * inputs.thruster_efficiency * inputs.power_in / ve
    mass_flow = thrust / ve
    waste_power = (1 - inputs.thruster_efficiency + 1 - inputs.ppu_efficiency) * inputs.power_in
    radiator_area = waste_power / (
        inputs.radiator_emissivity * SIGMA * inputs.radiator_temperature ** 4
    )
    return EPResults(
        exhaust_velocity=ve,
        thrust=thrust,
        mass_flow=mass_flow,
        waste_power=waste_power,
        radiator_area=radiator_area,
    )


def evaluate_ep_duration(inputs: EPDurationInputs) -> EPDurationResults:
    ep = evaluate_ep(inputs.ep_inputs)
    duration = (
        inputs.initial_mass
        * (1 - math.exp(-inputs.delta_v / ep.exhaust_velocity))
        * ep.exhaust_velocity**2
        / (2 * inputs.ep_inputs.thruster_efficiency * inputs.ep_inputs.power_in)
    )
    propellant_used = ep.mass_flow * duration
    return EPDurationResults(duration=duration, propellant_used=propellant_used)


def estimate_plane_change_time(
    orbital_speed: float, delta_inclination: float, vehicle_mass: float, inputs: EPInputs
) -> float:
    ep = evaluate_ep(inputs)
    return (
        2
        * orbital_speed
        * math.sin(delta_inclination / 2.0)
        * vehicle_mass
        * ep.exhaust_velocity
        / (2 * inputs.thruster_efficiency * inputs.power_in)
    )


__all__ = [
    "EPInputs",
    "EPResults",
    "EPDurationInputs",
    "EPDurationResults",
    "evaluate_ep",
    "evaluate_ep_duration",
    "estimate_plane_change_time",
]
