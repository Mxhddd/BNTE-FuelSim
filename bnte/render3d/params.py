"""Configuration dataclasses for the BNTE procedural stage model."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List


@dataclass
class MeshQuality:
    sections: int = 48
    segments: int = 16
    boolean_tolerance: float = 0.001


@dataclass
class StageParameters:
    name: str = "BNTE_v1"
    length: float = 18.0
    diameter: float = 5.0
    wall_thickness_main: float = 0.03
    mass_densities: Dict[str, float] = field(
        default_factory=lambda: {
            "al_li": 2700.0,
            "titanium": 4500.0,
            "cfp": 1600.0,
            "steel": 7850.0,
            "copper": 8960.0,
            "inconel": 8470.0,
        }
    )
    mesh_quality: MeshQuality = field(default_factory=MeshQuality)


@dataclass
class ReactorParameters:
    core_radius: float = 0.75
    core_length: float = 1.5
    reflector_thickness: float = 0.25
    pressure_vessel_thickness: float = 0.15
    axial_margins: float = 0.2
    material_density: float = 7850.0
    nozzle_clearance: float = 0.4


@dataclass
class HeatExchangerParameters:
    length: float = 1.2
    outer_radius: float = 1.0
    inner_radius_fraction: float = 0.45
    tube_count: int = 60
    tube_diameter: float = 0.05
    tube_lattice: str = "hex"
    shell_thickness: float = 0.05
    axial_offset: float = 0.8


@dataclass
class NozzleParameters:
    throat_radius: float = 0.4
    exit_radius: float = 1.2
    length: float = 3.5
    wall_thickness: float = 0.05
    profile_type: str = "rao"
    throat_to_exit_curve_params: Dict[str, float] = field(default_factory=lambda: {"cone_half_angle": 15.0})


@dataclass
class CryoTankParameters:
    count: int = 2
    tank_radius: float = 1.15
    tank_length_cyl: float = 4.2
    endcap_type: str = "ellipsoidal"
    insulation_thickness: float = 0.08
    baffle_count: int = 3
    axial_spacing: float = 2.6
    manifold_height_z: float = 5.0


@dataclass
class PlumbingParameters:
    feedline_id: float = 0.16
    feedline_od: float = 0.2
    line_wall: float = 0.02
    pump_mass: float = 250.0
    valve_mass: float = 18.0
    manifold_radius: float = 1.4
    tee_count: int = 4
    pressurization: str = "helium"
    pressurant_bottle_count: int = 4
    pressurant_bottle_size: Dict[str, float] = field(default_factory=lambda: {"radius": 0.3, "length": 1.5})


@dataclass
class ZBOParameters:
    cooler_power_kw: float = 180.0
    radiator_area_m2: float = 24.0
    evaporator_plate_dims: Dict[str, float] = field(default_factory=lambda: {"x": 1.0, "y": 0.8, "thickness": 0.02})
    mount_on_tanks: bool = True


@dataclass
class RadiatorParameters:
    panel_count: int = 6
    panel_dims: Dict[str, float] = field(default_factory=lambda: {"length": 3.0, "width": 1.4, "thickness": 0.05})
    hinge_arm_length: float = 0.6
    stowed_arc_radius: float = 1.8
    deployment_angle_deg: float = 120.0
    panel_material_density: float = 1600.0


@dataclass
class EPParameters:
    thruster_count: int = 8
    thruster_mount_radius: float = 2.0
    thruster_tilt_deg: float = 5.0
    thruster_spacing_rule: str = "even_circle"
    ppu_count: int = 4
    ppu_mass_each: float = 35.0
    cable_tray_height_z: float = 7.0


@dataclass
class StructureParameters:
    outer_radius: float = 2.5
    wall_thickness: float = 0.03
    ring_count: int = 5
    ring_positions_z: List[float] = field(default_factory=lambda: [0.1, 0.25, 0.5, 0.75, 0.9])
    longeron_count: int = 8
    window_cutouts: bool = False


@dataclass
class ForwardParameters:
    docking_port_radius: float = 1.1
    adapter_length: float = 2.0
    avionics_bay_length: float = 1.2
    bay_rack_count: int = 4


@dataclass
class StageModelConfig:
    stage: StageParameters = field(default_factory=StageParameters)
    reactor: ReactorParameters = field(default_factory=ReactorParameters)
    heat_exchanger: HeatExchangerParameters = field(default_factory=HeatExchangerParameters)
    nozzle: NozzleParameters = field(default_factory=NozzleParameters)
    cryo: CryoTankParameters = field(default_factory=CryoTankParameters)
    plumbing: PlumbingParameters = field(default_factory=PlumbingParameters)
    zbo: ZBOParameters = field(default_factory=ZBOParameters)
    radiators: RadiatorParameters = field(default_factory=RadiatorParameters)
    ep: EPParameters = field(default_factory=EPParameters)
    structure: StructureParameters = field(default_factory=StructureParameters)
    forward: ForwardParameters = field(default_factory=ForwardParameters)


def _merge_dataclass(instance, values: Dict[str, object]) -> None:
    for key, value in values.items():
        if hasattr(instance, key):
            attr = getattr(instance, key)
            if hasattr(attr, "__dict__") and isinstance(value, dict):
                _merge_dataclass(attr, value)
            else:
                setattr(instance, key, value)


def build_config(overrides: Dict[str, object] | None = None) -> StageModelConfig:
    config = StageModelConfig()
    if overrides:
        for key, value in overrides.items():
            if hasattr(config, key):
                attr = getattr(config, key)
                if isinstance(value, dict):
                    _merge_dataclass(attr, value)
                else:
                    setattr(config, key, value)
    return config


__all__ = ["StageModelConfig", "build_config"]
