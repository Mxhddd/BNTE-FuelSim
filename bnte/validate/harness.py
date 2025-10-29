"""Validation harness stubs for key test articles."""
from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Dict, List


@dataclass
class HotGasLoop:
    name: str
    chamber_pressure: float
    target_thrust: float
    instrumentation: List[str]


@dataclass
class EPBench:
    name: str
    power_level: float
    diagnostics: List[str]


@dataclass
class CryoBoiloff:
    name: str
    tank_area: float
    insulation_case: str


def generate_validation_harness() -> Dict[str, Dict[str, object]]:
    hot_gas = HotGasLoop(
        name="Hot-gas loop",
        chamber_pressure=10e5,
        target_thrust=239500.0,
        instrumentation=["fast-thermocouples", "RF reflectometer", "flow meters"],
    )
    ep_bench = EPBench(
        name="EP breadboard",
        power_level=100e3,
        diagnostics=["Faraday probe", "RPA", "thrust stand"],
    )
    cryo = CryoBoiloff(
        name="LN2 tank",
        tank_area=235.1,
        insulation_case="MLI",
    )
    return {
        "hot_gas_loop": asdict(hot_gas),
        "ep_bench": asdict(ep_bench),
        "cryogenic_boiloff": asdict(cryo),
    }


__all__ = ["generate_validation_harness"]
