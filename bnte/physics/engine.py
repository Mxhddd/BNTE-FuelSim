"""Lightweight rigid-body physics engine for BNTE stage dynamics."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence

from ..render3d.stage import ComponentOutput


@dataclass
class RigidBody:
    name: str
    mass: float
    inertia: Sequence[float]
    position: List[float]
    velocity: List[float]


@dataclass
class Thruster:
    name: str
    position: Sequence[float]
    direction: Sequence[float]
    max_thrust: float


@dataclass
class PhysicsResult:
    times: List[float]
    positions: List[Sequence[float]]
    velocities: List[Sequence[float]]
    output_path: Path


class PhysicsEngine:
    def __init__(self, outdir: Path):
        self.outdir = outdir

    def simulate(self, components: Iterable[ComponentOutput], duration: float = 120.0, step: float = 0.5) -> PhysicsResult:
        bodies: List[RigidBody] = []
        thrusters: List[Thruster] = []
        for component in components:
            bodies.append(
                RigidBody(
                    name=component.name,
                    mass=component.mass_kg,
                    inertia=component.inertia_kgm2,
                    position=list(component.pose.position),
                    velocity=[0.0, 0.0, 0.0],
                )
            )
            if "thrust_axis" in component.interfaces:
                thrust_axis = component.interfaces["thrust_axis"]
                thrusters.append(
                    Thruster(
                        name=component.name,
                        position=component.pose.position,
                        direction=thrust_axis,
                        max_thrust=500.0,
                    )
                )
        if not bodies:
            raise ValueError("No rigid bodies supplied to physics engine")
        total_mass = sum(body.mass for body in bodies)
        cg = [
            sum(body.mass * body.position[idx] for body in bodies) / total_mass for idx in range(3)
        ]
        state_position = cg[:]
        state_velocity = [0.0, 0.0, 0.0]
        times: List[float] = []
        positions: List[Sequence[float]] = []
        velocities: List[Sequence[float]] = []
        t = 0.0
        g = -9.80665
        thrust_profile = self._build_thrust_profile(thrusters, duration)
        while t <= duration:
            thrust = thrust_profile.get(round(t, 3), [0.0, 0.0, 0.0])
            acceleration = [thrust[0] / total_mass, thrust[1] / total_mass, thrust[2] / total_mass + g]
            state_velocity = [state_velocity[i] + acceleration[i] * step for i in range(3)]
            state_position = [state_position[i] + state_velocity[i] * step for i in range(3)]
            times.append(t)
            positions.append(tuple(state_position))
            velocities.append(tuple(state_velocity))
            t += step
        output_dir = self.outdir / "physics"
        output_dir.mkdir(parents=True, exist_ok=True)
        rows = [
            {
                "time": times[idx],
                "x": positions[idx][0],
                "y": positions[idx][1],
                "z": positions[idx][2],
                "vx": velocities[idx][0],
                "vy": velocities[idx][1],
                "vz": velocities[idx][2],
            }
            for idx in range(len(times))
        ]
        output_path = output_dir / "trajectory.json"
        output_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")
        return PhysicsResult(times=times, positions=positions, velocities=velocities, output_path=output_path)

    def _build_thrust_profile(self, thrusters: Sequence[Thruster], duration: float) -> Dict[float, List[float]]:
        profile: Dict[float, List[float]] = {}
        if not thrusters:
            return profile
        segment = duration / 4.0
        for thruster in thrusters:
            start = segment
            end = 2 * segment
            direction = thruster.direction
            magnitude = thruster.max_thrust
            step = 0.5
            t = start
            while t <= end:
                key = round(t, 3)
                thrust = profile.setdefault(key, [0.0, 0.0, 0.0])
                thrust[0] += direction[0] * magnitude
                thrust[1] += direction[1] * magnitude
                thrust[2] += direction[2] * magnitude
                t += step
        return profile


__all__ = ["PhysicsEngine", "PhysicsResult"]
