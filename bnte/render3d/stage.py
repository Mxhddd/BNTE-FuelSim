"""Stage assembly pipeline producing meshes, URDF, and metadata."""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Tuple

from .geometry import (
    Mesh,
    Pose,
    Quaternion,
    Vector3,
    lathe_profile,
    make_box,
    make_cylinder,
    make_tube,
    merge_meshes,
    translate_mesh,
    vector_add,
    vector_scale,
)
from .params import StageModelConfig, build_config


@dataclass
class ComponentOutput:
    name: str
    category: str
    pose: Pose
    mass_kg: float
    volume_m3: float
    inertia_kgm2: Tuple[float, float, float]
    render_mesh: Mesh
    collision_mesh: Mesh
    render_path: Path = field(default=Path())
    collision_path: Path = field(default=Path())
    interfaces: Dict[str, Dict[str, object]] = field(default_factory=dict)


@dataclass
class StageOutputs:
    output_dir: Path
    components: List[ComponentOutput]
    metadata: Dict[str, object]


@dataclass
class MassProps:
    mass: float
    centroid: Vector3
    inertia: Tuple[float, float, float]


def _solid_cylinder_props(radius: float, height: float, density: float) -> MassProps:
    volume = math.pi * radius * radius * height
    mass = volume * density
    ixx = 0.25 * mass * radius * radius + (1.0 / 12.0) * mass * height * height
    iyy = ixx
    izz = 0.5 * mass * radius * radius
    return MassProps(mass=mass, centroid=(0.0, 0.0, 0.0), inertia=(ixx, iyy, izz))


def _annular_cylinder_props(inner_r: float, outer_r: float, height: float, density: float) -> MassProps:
    outer = _solid_cylinder_props(outer_r, height, density)
    inner = _solid_cylinder_props(inner_r, height, density)
    mass = outer.mass - inner.mass
    volume = mass / density
    ixx = outer.inertia[0] - inner.inertia[0]
    iyy = outer.inertia[1] - inner.inertia[1]
    izz = outer.inertia[2] - inner.inertia[2]
    return MassProps(mass=mass, centroid=(0.0, 0.0, 0.0), inertia=(ixx, iyy, izz))


def _solid_box_props(dx: float, dy: float, dz: float, density: float) -> MassProps:
    volume = dx * dy * dz
    mass = volume * density
    ixx = (1.0 / 12.0) * mass * (dy * dy + dz * dz)
    iyy = (1.0 / 12.0) * mass * (dx * dx + dz * dz)
    izz = (1.0 / 12.0) * mass * (dx * dx + dy * dy)
    return MassProps(mass=mass, centroid=(0.0, 0.0, 0.0), inertia=(ixx, iyy, izz))


def _parallel_axis(props: MassProps, offset: Vector3) -> MassProps:
    ox, oy, oz = offset
    dx2 = oy * oy + oz * oz
    dy2 = ox * ox + oz * oz
    dz2 = ox * ox + oy * oy
    ixx = props.inertia[0] + props.mass * dx2
    iyy = props.inertia[1] + props.mass * dy2
    izz = props.inertia[2] + props.mass * dz2
    centroid = vector_add(props.centroid, offset)
    return MassProps(mass=props.mass, centroid=centroid, inertia=(ixx, iyy, izz))


def _combine_mass_props(entries: List[MassProps]) -> MassProps:
    if not entries:
        return MassProps(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    total_mass = sum(e.mass for e in entries)
    if total_mass == 0.0:
        return MassProps(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    cx = sum(e.mass * e.centroid[0] for e in entries) / total_mass
    cy = sum(e.mass * e.centroid[1] for e in entries) / total_mass
    cz = sum(e.mass * e.centroid[2] for e in entries) / total_mass
    ixx = sum(e.inertia[0] for e in entries)
    iyy = sum(e.inertia[1] for e in entries)
    izz = sum(e.inertia[2] for e in entries)
    return MassProps(total_mass, (cx, cy, cz), (ixx, iyy, izz))


def _write_obj(path: Path, mesh: Mesh) -> None:
    lines = ["o stage_component"]
    for vx, vy, vz in mesh.vertices:
        lines.append(f"v {vx:.6f} {vy:.6f} {vz:.6f}")
    for a, b, c in mesh.faces:
        lines.append(f"f {a + 1} {b + 1} {c + 1}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_stl(path: Path, mesh: Mesh) -> None:
    lines = ["solid stage"]
    for a, b, c in mesh.faces:
        ax, ay, az = mesh.vertices[a]
        bx, by, bz = mesh.vertices[b]
        cx, cy, cz = mesh.vertices[c]
        ux, uy, uz = bx - ax, by - ay, bz - az
        vx, vy, vz = cx - ax, cy - ay, cz - az
        nx = uy * vz - uz * vy
        ny = uz * vx - ux * vz
        nz = ux * vy - uy * vx
        length = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        nx /= length
        ny /= length
        nz /= length
        lines.append(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}")
        lines.append("    outer loop")
        lines.append(f"      vertex {ax:.6f} {ay:.6f} {az:.6f}")
        lines.append(f"      vertex {bx:.6f} {by:.6f} {bz:.6f}")
        lines.append(f"      vertex {cx:.6f} {cy:.6f} {cz:.6f}")
        lines.append("    endloop")
        lines.append("  endfacet")
    lines.append("endsolid stage")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _ensure_dirs(base: Path) -> Dict[str, Path]:
    render_dir = base / "meshes" / "render"
    collision_dir = base / "meshes" / "collision"
    states_dir = base / "states"
    stowed_dir = states_dir / "stowed"
    deployed_dir = states_dir / "deployed"
    metadata_dir = base / "metadata"
    for path in [render_dir, collision_dir, stowed_dir, deployed_dir, metadata_dir]:
        path.mkdir(parents=True, exist_ok=True)
    return {
        "render": render_dir,
        "collision": collision_dir,
        "stowed": stowed_dir,
        "deployed": deployed_dir,
        "metadata": metadata_dir,
    }


class StageBuilder:
    def __init__(self, config: StageModelConfig, outdir: Path):
        self.config = config
        self.outdir = outdir
        self.dirs = _ensure_dirs(outdir / "bnte_stage")
        self.components: List[ComponentOutput] = []
        self.stage_origin = (0.0, 0.0, 0.0)

    def build(self, simulation_summary: Dict[str, float] | None = None) -> StageOutputs:
        stage = self.config.stage
        radius_limit = stage.diameter * 0.5

        nozzle = self._build_nozzle(radius_limit)
        reactor = self._build_reactor(nozzle)
        hx = self._build_heat_exchanger(reactor)
        tanks = self._build_tanks(reactor)
        plumbing = self._build_plumbing(tanks, reactor)
        zbo = self._build_zbo(tanks)
        radiators = self._build_radiators()
        ep = self._build_ep(radiators)
        structure = self._build_structure()
        forward = self._build_forward()

        for component in [nozzle, reactor, hx, *tanks, plumbing, zbo, *radiators, *ep, structure, forward]:
            self._register_component(component)

        metadata = self._assemble_metadata(simulation_summary)
        self._write_urdf_states(metadata)
        return StageOutputs(output_dir=self.outdir, components=self.components, metadata=metadata)

    def _register_component(self, component: ComponentOutput) -> None:
        render_path = self.dirs["render"] / f"{component.name}.obj"
        collision_path = self.dirs["collision"] / f"{component.name}_collision.stl"
        px, py, pz = component.pose.position
        local_render = translate_mesh(component.render_mesh, (-px, -py, -pz))
        local_collision = translate_mesh(component.collision_mesh, (-px, -py, -pz))
        _write_obj(render_path, local_render)
        _write_stl(collision_path, local_collision)
        component.render_path = render_path
        component.collision_path = collision_path
        self.components.append(component)

    def _build_nozzle(self, radius_limit: float) -> ComponentOutput:
        params = self.config.nozzle
        stage = self.config.stage
        throat_z = 0.0
        exit_z = -params.length
        profile = [
            (params.throat_radius, throat_z),
            (params.exit_radius, exit_z),
        ]
        sections = stage.mesh_quality.sections
        inner = lathe_profile(profile, sections)
        outer_profile = [(r + params.wall_thickness, z) for r, z in profile]
        outer = lathe_profile(outer_profile, sections)
        shell = outer.merged(inner)
        mass_props = _annular_cylinder_props(
            params.throat_radius,
            params.exit_radius + params.wall_thickness,
            params.length,
            self.config.stage.mass_densities.get("inconel", 8200.0),
        )
        centroid = (0.0, 0.0, (throat_z + exit_z) / 2.0)
        pose = Pose(position=centroid)
        render_mesh = shell.transformed(pose)
        collision = lathe_profile([(params.exit_radius, exit_z), (params.throat_radius, throat_z)], 24).transformed(pose)
        component = ComponentOutput(
            name="nozzle",
            category="nozzle",
            pose=pose,
            mass_kg=mass_props.mass,
            volume_m3=mass_props.mass / self.config.stage.mass_densities.get("inconel", 8200.0),
            inertia_kgm2=mass_props.inertia,
            render_mesh=render_mesh,
            collision_mesh=collision,
            interfaces={
                "throat_plane": {
                    "position": centroid,
                    "normal": (0.0, 0.0, 1.0),
                }
            },
        )
        return component

    def _build_reactor(self, nozzle: ComponentOutput) -> ComponentOutput:
        params = self.config.reactor
        stage = self.config.stage
        z = nozzle.pose.position[2] + params.nozzle_clearance + params.core_length * 0.5
        mesh_quality = stage.mesh_quality.sections
        core_mesh = make_cylinder(params.core_radius, params.core_length, mesh_quality)
        reflector_outer_r = params.core_radius + params.reflector_thickness
        vessel_outer_r = reflector_outer_r + params.pressure_vessel_thickness
        reflector_mesh = make_tube(params.core_radius, reflector_outer_r, params.core_length + 2 * params.axial_margins, mesh_quality)
        vessel_mesh = make_tube(reflector_outer_r, vessel_outer_r, params.core_length + 2 * params.axial_margins, mesh_quality)
        combined_mesh = merge_meshes([
            core_mesh.transformed(Pose((0.0, 0.0, z))),
            reflector_mesh.transformed(Pose((0.0, 0.0, z))),
            vessel_mesh.transformed(Pose((0.0, 0.0, z))),
        ])
        density_core = stage.mass_densities.get("inconel", params.material_density)
        density_vessel = params.material_density
        core_props = _solid_cylinder_props(params.core_radius, params.core_length, density_core)
        reflector_props = _annular_cylinder_props(
            params.core_radius,
            reflector_outer_r,
            params.core_length + 2 * params.axial_margins,
            density_core * 0.6,
        )
        vessel_props = _annular_cylinder_props(
            reflector_outer_r,
            vessel_outer_r,
            params.core_length + 2 * params.axial_margins,
            density_vessel,
        )
        mass_props = _combine_mass_props([
            _parallel_axis(core_props, (0.0, 0.0, z)),
            _parallel_axis(reflector_props, (0.0, 0.0, z)),
            _parallel_axis(vessel_props, (0.0, 0.0, z)),
        ])
        collision = make_cylinder(vessel_outer_r, params.core_length + 2 * params.axial_margins, 24).transformed(Pose((0.0, 0.0, z)))
        return ComponentOutput(
            name="reactor",
            category="reactor",
            pose=Pose((0.0, 0.0, z)),
            mass_kg=mass_props.mass,
            volume_m3=(core_props.mass + reflector_props.mass + vessel_props.mass) / density_vessel,
            inertia_kgm2=mass_props.inertia,
            render_mesh=combined_mesh,
            collision_mesh=collision,
            interfaces={
                "inlet": {"position": (0.0, 0.0, z + params.core_length * 0.5), "normal": (0.0, 0.0, 1.0)},
                "outlet": {"position": (0.0, 0.0, z - params.core_length * 0.5), "normal": (0.0, 0.0, -1.0)},
            },
        )

    def _build_heat_exchanger(self, reactor: ComponentOutput) -> ComponentOutput:
        params = self.config.heat_exchanger
        stage = self.config.stage
        z_center = reactor.pose.position[2] + params.axial_offset + params.length * 0.5
        outer = make_cylinder(params.outer_radius, params.length, stage.mesh_quality.sections)
        inner = make_cylinder(params.outer_radius * params.inner_radius_fraction, params.length, stage.mesh_quality.sections)
        shell_mesh = outer.merged(inner)
        density = stage.mass_densities.get("copper", 8960.0)
        shell_props = _annular_cylinder_props(
            params.outer_radius * params.inner_radius_fraction,
            params.outer_radius,
            params.length,
            density,
        )
        pose = Pose((0.0, 0.0, z_center))
        shell_mesh = shell_mesh.transformed(pose)
        collision = make_cylinder(params.outer_radius, params.length, 24).transformed(pose)
        tube_centers: List[Vector3] = []
        placed = 0
        pitch = params.tube_diameter * (1.8 if params.tube_lattice == "hex" else 1.6)
        radial_inner = params.outer_radius * params.inner_radius_fraction + params.tube_diameter
        radial_outer = params.outer_radius - params.tube_diameter
        for ring in range(10):
            for idx in range(max(6, params.tube_count)):
                angle = (2 * math.pi / max(6, params.tube_count)) * idx
                radius = radial_inner + ring * pitch * 0.5
                if radius > radial_outer:
                    continue
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                tube_centers.append((x, y, z_center))
                placed += 1
                if placed >= params.tube_count:
                    break
            if placed >= params.tube_count:
                break
        volume = shell_props.mass / density
        interfaces = {
            "inlet": {"position": (params.outer_radius, 0.0, z_center + params.length * 0.5), "normal": (1.0, 0.0, 0.0)},
            "outlet": {"position": (params.outer_radius, 0.0, z_center - params.length * 0.5), "normal": (1.0, 0.0, 0.0)},
        }
        meta = ComponentOutput(
            name="heat_exchanger",
            category="heat_exchanger",
            pose=pose,
            mass_kg=shell_props.mass,
            volume_m3=volume,
            inertia_kgm2=shell_props.inertia,
            render_mesh=shell_mesh,
            collision_mesh=collision,
            interfaces=interfaces | {"tube_centers": {"value": tube_centers}},
        )
        return meta

    def _build_tanks(self, reactor: ComponentOutput) -> List[ComponentOutput]:
        params = self.config.cryo
        stage = self.config.stage
        radius = params.tank_radius
        insulation = radius + params.insulation_thickness
        tank_length = params.tank_length_cyl
        mesh_quality = stage.mesh_quality.sections
        z_center = reactor.pose.position[2] + params.axial_spacing + tank_length * 0.5
        components: List[ComponentOutput] = []
        density_shell = stage.mass_densities.get("al_li", 2700.0)
        for idx in range(params.count):
            side = -1.0 if idx % 2 == 0 else 1.0
            x_offset = side * params.axial_spacing * 0.5
            pose = Pose((x_offset, 0.0, z_center))
            core_mesh = make_cylinder(radius, tank_length, mesh_quality).transformed(pose)
            insulation_mesh = make_cylinder(insulation, tank_length, mesh_quality).transformed(pose)
            render_mesh = merge_meshes([core_mesh, insulation_mesh])
            collision_mesh = make_cylinder(insulation, tank_length, 24).transformed(pose)
            shell_props = _annular_cylinder_props(radius - stage.wall_thickness_main, radius, tank_length, density_shell)
            props = _parallel_axis(shell_props, pose.position)
            mass_props = MassProps(props.mass, pose.position, props.inertia)
            interfaces = {
                "drain": {
                    "position": (x_offset, 0.0, z_center - tank_length * 0.5),
                    "normal": (0.0, 0.0, -1.0),
                },
                "fill": {
                    "position": (x_offset, 0.0, z_center + tank_length * 0.5),
                    "normal": (0.0, 0.0, 1.0),
                },
            }
            components.append(
                ComponentOutput(
                    name=f"tank_{idx}",
                    category="tank",
                    pose=pose,
                    mass_kg=mass_props.mass,
                    volume_m3=math.pi * radius * radius * tank_length,
                    inertia_kgm2=mass_props.inertia,
                    render_mesh=render_mesh,
                    collision_mesh=collision_mesh,
                    interfaces=interfaces,
                )
            )
        return components

    def _build_plumbing(self, tanks: List[ComponentOutput], reactor: ComponentOutput) -> ComponentOutput:
        params = self.config.plumbing
        line_radius = params.feedline_od * 0.5
        sections = self.config.stage.mesh_quality.sections
        nodes: Dict[str, Dict[str, object]] = {}
        edges: List[Dict[str, object]] = []
        manifolds: List[Mesh] = []
        manifold_z = self.config.cryo.manifold_height_z
        manifold_radius = params.manifold_radius
        for idx, tank in enumerate(tanks):
            drain = tank.interfaces["drain"]["position"]
            downcomer_start = drain
            downcomer_end = (drain[0], drain[1], manifold_z)
            nodes[f"tank_{idx}_drain"] = {"position": downcomer_start}
            nodes[f"manifold_entry_{idx}"] = {"position": downcomer_end}
            length = abs(downcomer_start[2] - manifold_z)
            edges.append(
                {
                    "start": f"tank_{idx}_drain",
                    "end": f"manifold_entry_{idx}",
                    "length": length,
                    "id": params.feedline_id,
                }
            )
            manifolds.append(
                make_cylinder(line_radius, length, sections).transformed(
                    Pose(((downcomer_start[0] + downcomer_end[0]) / 2.0, downcomer_start[1], (downcomer_start[2] + manifold_z) / 2.0))
                )
            )
        manifold_ring_mesh = make_tube(manifold_radius - line_radius, manifold_radius + line_radius, params.feedline_od, sections)
        manifold_ring_mesh = manifold_ring_mesh.transformed(Pose((0.0, 0.0, manifold_z)))
        manifolds.append(manifold_ring_mesh)
        render_mesh = merge_meshes(manifolds)
        collision_mesh = merge_meshes([make_cylinder(line_radius * 1.2, params.feedline_od, 24).transformed(Pose((0.0, 0.0, manifold_z)))])
        pump_position = (0.0, 0.0, reactor.pose.position[2] - 0.5)
        nodes["pump"] = {"position": pump_position}
        edges.append(
            {
                "start": "manifold_ring",
                "end": "pump",
                "length": abs(manifold_z - pump_position[2]),
                "id": params.feedline_id,
            }
        )
        interfaces = {
            "pump": pump_position,
            "manifold": (0.0, 0.0, manifold_z),
            "graph": {
                "nodes": nodes,
                "edges": edges,
            },
        }
        mass = params.pump_mass + params.valve_mass * len(tanks)
        inertia = (mass * 0.1, mass * 0.1, mass * 0.1)
        return ComponentOutput(
            name="plumbing",
            category="plumbing",
            pose=Pose((0.0, 0.0, manifold_z)),
            mass_kg=mass,
            volume_m3=0.0,
            inertia_kgm2=inertia,
            render_mesh=render_mesh,
            collision_mesh=collision_mesh,
            interfaces=interfaces,
        )

    def _build_zbo(self, tanks: List[ComponentOutput]) -> ComponentOutput:
        params = self.config.zbo
        stage = self.config.stage
        dims = params.evaporator_plate_dims
        plate_meshes: List[Mesh] = []
        nodes: List[Dict[str, object]] = []
        for tank in tanks:
            pos = tank.pose.position
            offset = (pos[0], pos[1] + stage.wall_thickness_main + dims["thickness"] * 0.5, pos[2] + dims["thickness"])
            plate = make_box(dims["x"], dims["y"], dims["thickness"]).transformed(Pose(offset))
            plate_meshes.append(plate)
            nodes.append({"position": offset, "type": "evaporator"})
        render_mesh = merge_meshes(plate_meshes)
        collision_mesh = merge_meshes([make_box(dims["x"], dims["y"], dims["thickness"]).transformed(Pose(nodes[0]["position"]))])
        mass = params.cooler_power_kw * 2.0
        inertia = (mass * 0.01, mass * 0.01, mass * 0.02)
        return ComponentOutput(
            name="zbo",
            category="thermal",
            pose=Pose(nodes[0]["position"] if nodes else (0.0, 0.0, 0.0)),
            mass_kg=mass,
            volume_m3=dims["x"] * dims["y"] * dims["thickness"] * len(plate_meshes),
            inertia_kgm2=inertia,
            render_mesh=render_mesh,
            collision_mesh=collision_mesh,
            interfaces={"thermal_nodes": nodes},
        )

    def _build_radiators(self) -> List[ComponentOutput]:
        params = self.config.radiators
        stage = self.config.stage
        panel = params.panel_dims
        hinge_radius = min(params.stowed_arc_radius, stage.diameter * 0.5 - 0.2)
        base_z = stage.length * 0.5
        components: List[ComponentOutput] = []
        for idx in range(params.panel_count):
            angle = 2 * math.pi * idx / params.panel_count
            x = hinge_radius * math.cos(angle)
            y = hinge_radius * math.sin(angle)
            pose = Pose((x, y, base_z))
            render_mesh = make_box(panel["width"], panel["thickness"], panel["length"]).transformed(pose)
            collision_mesh = make_box(panel["width"], panel["thickness"], panel["length"]).transformed(pose)
            density = params.panel_material_density
            props = _solid_box_props(panel["width"], panel["thickness"], panel["length"], density)
            props = _parallel_axis(props, pose.position)
            components.append(
                ComponentOutput(
                    name=f"radiator_{idx}",
                    category="radiator",
                    pose=pose,
                    mass_kg=props.mass,
                    volume_m3=panel["width"] * panel["thickness"] * panel["length"],
                    inertia_kgm2=props.inertia,
                    render_mesh=render_mesh,
                    collision_mesh=collision_mesh,
                    interfaces={
                        "hinge": {
                            "position": pose.position,
                            "axis": (math.cos(angle), math.sin(angle), 0.0),
                            "limits": [0.0, math.radians(params.deployment_angle_deg)],
                        }
                    },
                )
            )
        return components

    def _build_ep(self, radiators: List[ComponentOutput]) -> List[ComponentOutput]:
        params = self.config.ep
        stage = self.config.stage
        components: List[ComponentOutput] = []
        radius = params.thruster_mount_radius
        base_z = stage.length * 0.3
        tilt = math.radians(params.thruster_tilt_deg)
        for idx in range(params.thruster_count):
            angle = 2 * math.pi * idx / params.thruster_count
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            pose = Pose((x, y, base_z))
            render_mesh = make_cylinder(0.2, 0.6, stage.mesh_quality.sections).transformed(pose)
            collision_mesh = make_cylinder(0.25, 0.6, 24).transformed(pose)
            density = stage.mass_densities.get("titanium", 4500.0)
            props = _solid_cylinder_props(0.2, 0.6, density)
            props = _parallel_axis(props, pose.position)
            thrust_axis = (
                -math.sin(tilt) * math.cos(angle),
                -math.sin(tilt) * math.sin(angle),
                -math.cos(tilt),
            )
            components.append(
                ComponentOutput(
                    name=f"ep_thruster_{idx}",
                    category="ep_thruster",
                    pose=pose,
                    mass_kg=props.mass,
                    volume_m3=math.pi * 0.2 * 0.2 * 0.6,
                    inertia_kgm2=props.inertia,
                    render_mesh=render_mesh,
                    collision_mesh=collision_mesh,
                    interfaces={"thrust_axis": thrust_axis},
                )
            )
        for idx in range(params.ppu_count):
            x = -stage.diameter * 0.2
            y = -stage.diameter * 0.2 + idx * 0.5
            pose = Pose((x, y, params.cable_tray_height_z))
            render_mesh = make_box(0.6, 0.4, 0.3).transformed(pose)
            collision_mesh = make_box(0.65, 0.45, 0.35).transformed(pose)
            mass = params.ppu_mass_each
            inertia = (mass * 0.02, mass * 0.02, mass * 0.02)
            components.append(
                ComponentOutput(
                    name=f"ppu_{idx}",
                    category="ppu",
                    pose=pose,
                    mass_kg=mass,
                    volume_m3=0.6 * 0.4 * 0.3,
                    inertia_kgm2=inertia,
                    render_mesh=render_mesh,
                    collision_mesh=collision_mesh,
                    interfaces={"cable_tray": (pose.position[0], pose.position[1], pose.position[2] + 0.2)},
                )
            )
        return components

    def _build_structure(self) -> ComponentOutput:
        params = self.config.structure
        stage = self.config.stage
        shell = make_tube(
            params.outer_radius - params.wall_thickness,
            params.outer_radius,
            stage.length,
            stage.mesh_quality.sections,
        )
        pose = Pose((0.0, 0.0, stage.length * 0.5))
        shell = shell.transformed(pose)
        mass_props = _annular_cylinder_props(
            params.outer_radius - params.wall_thickness,
            params.outer_radius,
            stage.length,
            stage.mass_densities.get("al_li", 2700.0),
        )
        collision_mesh = make_cylinder(params.outer_radius, stage.length, 24).transformed(pose)
        rings = []
        for frac in params.ring_positions_z:
            z = stage.length * frac
            rings.append(make_tube(params.outer_radius - 0.1, params.outer_radius, 0.2, stage.mesh_quality.sections).transformed(Pose((0.0, 0.0, z))))
        longerons = []
        for idx in range(params.longeron_count):
            angle = 2 * math.pi * idx / params.longeron_count
            x = (params.outer_radius - params.wall_thickness) * math.cos(angle)
            y = (params.outer_radius - params.wall_thickness) * math.sin(angle)
            longerons.append(make_box(0.1, 0.1, stage.length).transformed(Pose((x, y, stage.length * 0.5))))
        render_mesh = merge_meshes([shell, *rings, *longerons])
        return ComponentOutput(
            name="structure",
            category="structure",
            pose=pose,
            mass_kg=mass_props.mass,
            volume_m3=mass_props.mass / stage.mass_densities.get("al_li", 2700.0),
            inertia_kgm2=mass_props.inertia,
            render_mesh=render_mesh,
            collision_mesh=collision_mesh,
            interfaces={},
        )

    def _build_forward(self) -> ComponentOutput:
        params = self.config.forward
        stage = self.config.stage
        pose = Pose((0.0, 0.0, stage.length))
        adapter = make_cylinder(params.docking_port_radius, params.adapter_length, stage.mesh_quality.sections).transformed(pose)
        density = stage.mass_densities.get("al_li", 2700.0)
        props = _solid_cylinder_props(params.docking_port_radius, params.adapter_length, density)
        props = _parallel_axis(props, pose.position)
        collision = make_cylinder(params.docking_port_radius, params.adapter_length, 24).transformed(pose)
        interfaces = {
            "docking_plane": {
                "position": (0.0, 0.0, stage.length + params.adapter_length * 0.5),
                "normal": (0.0, 0.0, 1.0),
            }
        }
        return ComponentOutput(
            name="forward_adapter",
            category="forward",
            pose=pose,
            mass_kg=props.mass,
            volume_m3=props.mass / density,
            inertia_kgm2=props.inertia,
            render_mesh=adapter,
            collision_mesh=collision,
            interfaces=interfaces,
        )

    def _assemble_metadata(self, simulation_summary: Dict[str, float] | None) -> Dict[str, object]:
        stage = self.config.stage
        total_mass = sum(component.mass_kg for component in self.components)
        cg = (
            sum(component.mass_kg * component.pose.position[0] for component in self.components) / total_mass,
            sum(component.mass_kg * component.pose.position[1] for component in self.components) / total_mass,
            sum(component.mass_kg * component.pose.position[2] for component in self.components) / total_mass,
        )
        inertia = [0.0, 0.0, 0.0]
        for component in self.components:
            dx = component.pose.position[0] - cg[0]
            dy = component.pose.position[1] - cg[1]
            dz = component.pose.position[2] - cg[2]
            inertia[0] += component.inertia_kgm2[0] + component.mass_kg * (dy * dy + dz * dz)
            inertia[1] += component.inertia_kgm2[1] + component.mass_kg * (dx * dx + dz * dz)
            inertia[2] += component.inertia_kgm2[2] + component.mass_kg * (dx * dx + dy * dy)
        component_dicts = []
        for component in self.components:
            component_dicts.append(
                {
                    "name": component.name,
                    "category": component.category,
                    "pose_in_G": {
                        "position": component.pose.position,
                        "orientation": [component.pose.orientation.w, component.pose.orientation.x, component.pose.orientation.y, component.pose.orientation.z],
                    },
                    "mass_kg": component.mass_kg,
                    "volume_m3": component.volume_m3,
                    "inertia": component.inertia_kgm2,
                    "render_mesh": str(component.render_path.relative_to(self.outdir)),
                    "collision_mesh": str(component.collision_path.relative_to(self.outdir)),
                    "interfaces": component.interfaces,
                }
            )
        metadata = {
            "stage": {
                "name": stage.name,
                "length": stage.length,
                "diameter": stage.diameter,
                "mass_total": total_mass,
                "cg": cg,
                "principal_inertia": inertia,
            },
            "components": component_dicts,
            "simulation_summary": simulation_summary or {},
        }
        metadata_path = self.dirs["metadata"] / "bnte_metadata.json"
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        return metadata

    def _write_urdf_states(self, metadata: Dict[str, object]) -> None:
        links: List[str] = []
        joints: List[str] = []
        for component in self.components:
            link_name = component.name
            visual_origin = "0 0 0 0 0 0"
            link = f"  <link name=\"{link_name}\">\n"
            link += "    <visual>\n"
            link += f"      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
            link += f"      <geometry><mesh filename=\"{component.render_path.relative_to(self.outdir)}\"/></geometry>\n"
            link += "    </visual>\n"
            link += "    <collision>\n"
            link += f"      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
            link += f"      <geometry><mesh filename=\"{component.collision_path.relative_to(self.outdir)}\"/></geometry>\n"
            link += "    </collision>\n"
            link += "    <inertial>\n"
            link += f"      <mass value=\"{component.mass_kg:.3f}\"/>\n"
            link += f"      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n"
            link += (
                f"      <inertia ixx=\"{component.inertia_kgm2[0]:.6f}\" iyy=\"{component.inertia_kgm2[1]:.6f}\" izz=\"{component.inertia_kgm2[2]:.6f}\" "
                f"ixy=\"0.0\" ixz=\"0.0\" iyz=\"0.0\"/>\n"
            )
            link += "    </inertial>\n"
            link += "  </link>\n"
            links.append(link)

        parent = "structure"
        for component in self.components:
            if component.name == parent:
                continue
            pose = component.pose.position
            joint_name = f"structure_to_{component.name}"
            joints.append(
                "  <joint name=\"{0}\" type=\"fixed\">\n"
                "    <parent link=\"{1}\"/>\n"
                "    <child link=\"{2}\"/>\n"
                "    <origin xyz=\"{3:.3f} {4:.3f} {5:.3f}\" rpy=\"0 0 0\"/>\n"
                "  </joint>\n".format(
                    joint_name,
                    parent,
                    component.name,
                    pose[0],
                    pose[1],
                    pose[2],
                )
            )
        urdf = "<?xml version=\"1.0\"?>\n<robot name=\"bnte_stage\">\n" + "".join(links) + "".join(joints) + "</robot>\n"
        (self.dirs["stowed"] / "bnte_stage_stowed.urdf").write_text(urdf, encoding="utf-8")
        (self.dirs["deployed"] / "bnte_stage_deployed.urdf").write_text(urdf, encoding="utf-8")


def generate_stage_assets(outdir: Path, overrides: Dict[str, object] | None = None, simulation_summary: Dict[str, float] | None = None) -> StageOutputs:
    config = build_config(overrides)
    builder = StageBuilder(config, outdir)
    return builder.build(simulation_summary)


__all__ = ["generate_stage_assets", "StageOutputs", "ComponentOutput"]
