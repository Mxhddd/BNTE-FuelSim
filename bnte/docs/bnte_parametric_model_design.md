# BNTE Parametric Stage Model — Implementation Design Brief

## 0. Goals and Scope
- Generate a procedural 3D assembly of the BNTE upper stage from configuration inputs shared with the analytic simulation pipeline.
- Produce both high-fidelity render meshes and simplified physics/collision meshes for every subsystem.
- Compute engineering metadata (mass, volume, CG, inertia, interface points, network graphs) aligned with BNTE mission analysis outputs.
- Export artefacts into a project directory structure containing meshes (OBJ/STL), URDF assemblies, and a metadata JSON report with validation results.

## 1. Global Conventions
### 1.1 Coordinate Frames
- Global frame **G**: +Z points forward (aft engine/nozzle lies at negative Z), +X starboard, +Y port.
- Each component exposes a local frame at its geometric centre (unless specified) with a pose in G.

### 1.2 Units and Types
- Units: metres, kilograms, seconds, radians, Kelvin, Watts.
- Parameter types: float, int, bool, string, enum.

### 1.3 Mesh Quality
- Global tessellation settings: `mesh_sections`, `curve_segments`, and `boolean_tolerance`.
- All boolean operations must yield watertight, manifold meshes with retry logic (±0.5–2 mm dilation) on failure.

## 2. Stage-Level Parameters
`Stage` group:
- `name`, `length`, `diameter`, `wall_thickness_main`, `mass_densities` (per material).
- `mesh_quality` dict containing tessellation controls and boolean tolerance.
- All subcomponents must fit inside the stowed diameter envelope.

## 3. Subsystem Definitions
### 3.1 Reactor Assembly
- Parameters: `core_radius`, `core_length`, `reflector_thickness`, `pressure_vessel_thickness`, `axial_margins`, `material_density`, `nozzle_clearance`.
- Geometry: coaxial cylinders (core, reflector annulus, pressure-vessel shell) with optional hemispherical endcaps.
- Placement: aft section, clearance to nozzle maintained via `nozzle_clearance`.
- Outputs: render + collision meshes, mass/volume, inertia tensor, pose, metadata bundle.

### 3.2 Primary Heat Exchanger
- Parameters: `length`, `outer_radius`, `inner_radius_fraction`, `tube_count`, `tube_diameter`, `tube_lattice`, `shell_thickness`, `axial_offset`.
- Construct annular shell, populate tube bundle using lattice packing, subtract tubes to create flow passages, add optional outer shell.
- Placement: coaxial with reactor, offset forward by `axial_offset`.
- Outputs: physical properties, tube coordinate list, achieved tube count, flow area.

### 3.3 NTR Nozzle
- Parameters: `throat_radius`, `exit_radius`, `length`, `wall_thickness`, `profile_type`, `throat_to_exit_curve_params`.
- Geometry: revolve inner profile according to `profile_type`, offset for wall thickness, subtract to form bell shell, add throat support collar.
- Placement: apex near `z = 0`, interface ring mates with reactor, enforce nozzle clearance.
- Outputs: meshes, mass properties, mount ring radius, throat plane location.

### 3.4 Cryogenic Tanks
- Parameters: `count`, `tank_radius`, `tank_length_cyl`, `endcap_type`, `insulation_thickness`, `baffle_count`, `axial_spacing`, `manifold_height_z`.
- Geometry: cylinder with matched endcaps; insulation shell by radial/axial offset; internal baffles as thin discs; ports as short bosses.
- Placement: lateral pairs about Z axis (±X offsets), aligned with stage CG target and within diameter constraint.
- Computed: usable prop volume, insulation mass, per-tank CG.

### 3.5 Feed System & Plumbing
- Parameters: `feedline_id`, `feedline_od`, `line_wall`, `pump_mass`, `valve_mass`, `manifold_radius`, `tee_count`, `pressurization`, `pressurant_bottle_count`, `pressurant_bottle_size`.
- Routing rules: downcomers from tank ports to ring manifold, valves near entries/quadrants, pump near reactor plane, lines to reactor → HX → nozzle, pressurant bottles on truss ring with ullage lines.
- Geometry: swept cylinders for lines, thickened valve bodies, pump cylinder, toroidal manifold approximations.
- Computed: per-segment lengths, volumes, component coordinates, plumbing graph (nodes and edges with hydraulic data).

### 3.6 Zero-Boil-Off (ZBO) Management
- Parameters: `cooler_power_kw`, `radiator_area_m2`, `evaporator_plate_dims`, `mount_on_tanks`.
- Geometry: evaporator plates conformal to tank skins, optional cold straps.
- Outputs: contact area, strap lengths, thermal nodes (tank skin, evaporator, radiator root).

### 3.7 Deployable Radiators
- Parameters: `panel_count`, `panel_dims`, `hinge_arm_length`, `stowed_arc_radius`, `deployment_angle_deg`, `panel_material_density`.
- Geometry: panel slabs with ribs, hinge arms, hinge bosses.
- Placement: equally spaced around structural ring, stowed within diameter, deployed by rotating through deployment angle.
- Outputs: stowed & deployed meshes, URDF joint frames, limits.

### 3.8 Electric Propulsion Array & PPUs
- Parameters: `thruster_count`, `thruster_mount_radius`, `thruster_tilt_deg`, `thruster_spacing_rule`, `ppu_count`, `ppu_mass_each`, `cable_tray_height_z`.
- Geometry: thruster bells/bodies, mount struts, PPU boxes, cable trays.
- Placement: distribute per spacing rule, ensure deployed radiator clearance.
- Outputs: thruster pose list with thrust axes, resultant thrust vector, clearance metrics.

### 3.9 Structural Body & Truss Rings
- Parameters: `outer_radius`, `wall_thickness`, `ring_count`, `ring_positions_z`, `longeron_count`, `window_cutouts`.
- Geometry: main shell, toroidal rings, axial longerons, optional cut-outs.
- Placement: integrate with interfaces for subsystems.
- Outputs: structural dry mass estimate, interface bolt-circle data.

### 3.10 Forward Docking Adapter & Avionics
- Parameters: `docking_port_radius`, `adapter_length`, `avionics_bay_length`, `bay_rack_count`.
- Geometry: tapering adapter, docking ring, avionics racks, cable trays.
- Outputs: docking frame pose, avionics bay extents for payload integration.

## 4. Assembly Logic
- Reference plane `z_aft = 0` at nozzle apex.
- Sequential placement: nozzle (z ≤ 0) → reactor (forward with clearance) → heat exchanger → cryogenic tanks near CG → plumbing and manifolds → ZBO hardware → radiator hinge ring → EP array → structural shell → forward adapter.
- After each placement run collision checks (minimum clearance ≈ 10 mm). Adjust offending components within allowed bounds and record adjustments.

## 5. Dual Mesh Strategy
- Every component exports high-detail render mesh and simplified collision hull.
- Provide mapping from component metadata to render/collision filenames.

## 6. Mass Properties & Materials
- Material tags reference densities in `Stage.mass_densities`.
- For each component: volume, mass, inertia tensor (local frame), optional transform to global for URDF.
- Propellant volume per tank; handle baffle displacement explicitly or as metadata assumption.

## 7. URDF Assembly Specification
- Links include `link_name`, render/collision mesh paths, pose offsets, inertial data, material labels.
- Joints define `joint_name`, type (`fixed` or `revolute`), parent/child, origin pose, rotation axis, limits.
- Required links: body shell, reactor, nozzle, heat exchanger, tanks, pump/manifold/valves, radiator panels, EP thrusters, forward adapter, docking ring, avionics bay.

## 8. Metadata JSON Structure
- Top-level parameters (resolved inputs and auto-adjustments).
- Per-component entries: name, category, pose, mass, volume, inertia, mesh paths, interface frames.
- Network graphs:
  - Plumbing nodes/edges with lengths, diameters, loss coefficients.
  - Thermal nodes and conductive links with placeholder properties.
- Clearance report, auto-adjustments, whole-stage CG and inertia breakdown.
- Include thermal solver hooks, plume nodes (nozzle exit, EP thruster exits), control/avionics interface frames.

## 9. Validation & Sanity Checks
- Envelope, clearance, mass budget, CG location, plumbing continuity, URDF integrity.
- On failure, apply minimal permissible tweak (e.g., shrink radiator width, increase stowed radius) and log adjustments.

## 10. Parameter Bounds & Defaults
- Enforce hard limits: stage diameter ≥ twice tank radius + insulation margin; nozzle exit radius ≤ stage radius minus clearance; radiator stowed geometry within envelope; plumbing bend radius ≥ 3×OD.
- Provide soft default values for every parameter and guard against invalid input ranges.

## 11. Moving Item States
- Export both stowed and deployed meshes/URDFs, with stowed as default and joint limits enabling deployment.

## 12. File Layout & Naming
```
./bnte_stage/
  meshes/
    render/
    collision/
  states/
    stowed/
    deployed/
  metadata/
    bnte_metadata.json
```
- File names lowercase with underscores: `<component>_<index>.<ext>`.
- URDFs: `bnte_stage_stowed.urdf`, `bnte_stage_deployed.urdf`.

## 13. Console Summary Output
- Total dry mass and subsystem breakdown.
- Whole-stage CG, principal inertias.
- Tank usable volume total.
- Plumbing total line length and valve count.
- Radiator total area and hinge count.
- EP thruster list with thrust axes.

## 14. Extension Hooks
- Thermal solver integration: each thermal node stores representative area, thickness, and placeholder conductivity.
- CFD plume preparation: record nozzle exit plane pose and EP thruster exit poses for future plume models.
- Controls and avionics: include interface frames for IMU near the CG, tank level sensors, and line pressure taps near pump inlet/outlet.

## 15. Acceptance Criteria
- Successful booleans and manifold meshes.
- URDF loads in standard simulators.
- Collision shells approximate volumes within ±10%.
- Mass totals consistent with densities.
- Plumbing graph connects tank → nozzle with no dangling edges.
- Stowed assembly fits envelope; deployed state exports.
