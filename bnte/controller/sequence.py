"""Controller timing utilities."""
from __future__ import annotations

from dataclasses import dataclass

from ..constants import G0


@dataclass
class PlenumInputs:
    volume: float  # m^3
    characteristic_velocity: float  # m/s
    gas_constant: float  # J/(kg*K)
    chamber_temperature: float  # K
    throat_area: float  # m^2


@dataclass
class PlenumResults:
    time_constant: float  # s


def evaluate_plenum(inputs: PlenumInputs) -> PlenumResults:
    time_constant = (
        inputs.volume
        * inputs.characteristic_velocity
        / (inputs.gas_constant * inputs.chamber_temperature * inputs.throat_area)
    )
    return PlenumResults(time_constant=time_constant)


@dataclass
class SequenceStep:
    time: float
    description: str


@dataclass
class ModeSequence:
    steps: list[SequenceStep]


def nominal_sequence() -> ModeSequence:
    return ModeSequence(
        steps=[
            SequenceStep(time=0.0, description="Thermal ramp-down initiated"),
            SequenceStep(time=1.0, description="Hydrogen flow transition"),
            SequenceStep(time=2.5, description="EP bus precharge"),
            SequenceStep(time=4.0, description="EP thruster ignition"),
            SequenceStep(time=5.0, description="Stable EP operations"),
        ]
    )


__all__ = ["PlenumInputs", "PlenumResults", "evaluate_plenum", "SequenceStep", "ModeSequence", "nominal_sequence"]
