"""Mission runner orchestrating BNTE analyses."""
from __future__ import annotations

import json
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, Sequence

import math

from ..config import Config
from ..constants import G0, HYDROGEN_CP, HYDROGEN_R, SIGMA
from ..cryo.boiloff import CryoInputs, evaluate_cryo
from ..ep.system import (
    EPDurationInputs,
    EPInputs,
    evaluate_ep,
    evaluate_ep_duration,
    estimate_plane_change_time,
)
from ..feed.system import FeedInputs, evaluate_feed
from ..htx.regen import RegenInputs, evaluate_regen
from ..nozzle.performance import NozzleInputs, compute_nozzle_performance
from ..plume.keepout import PlumeInputs, evaluate_plume
from ..powerbus.modeswitch import ModeSwitchInputs, evaluate_mode_switch
from ..press.autogenous import PressurizationInputs, evaluate_pressurization
from ..thermal.bartz import BartzInputs, evaluate_bartz
from ..controller.sequence import PlenumInputs, evaluate_plenum, nominal_sequence
from ..validate.harness import generate_validation_harness
from ..render3d import generate_stage_assets
from ..physics import PhysicsEngine


@dataclass
class SimulationResults:
    summary: Dict[str, float]
    tables: Dict[str, list[Dict[str, float]]]
    metadata: Dict[str, Any]


TARGETS = {
    "m0": 35_000.0,
    "dry_payload": 11_000.0,
    "thermal_prop": 10_000.0,
    "m1": 25_000.0,
    "isp_th": 862.0,
    "thrust": 239_500.0,
    "delta_v_th": 2_969.7,
    "heater_power": 0.98e9,
    "delta_p_line": 0.242e5,
    "injector_area": 9.41e-3,
    "pump_power": 0.568e6,
    "npsh_margin": 146.0,
    "bartz_q": 1.24e7,
    "regen_alpha": 0.054,
    "radiator_tau": 180.0,
    "ep_thrust": 4.08,
    "ep_mdot": 1.387e-4,
    "ep_waste": 45_000.0,
    "ep_area": 14.1,
    "ep_area_zbo": 21.6,
    "ep_duration": 202.0 * 24 * 3600,
    "ep_prop": 2_424.0,
    "boiloff": 45.6,
    "boiloff_low": 14.0,
    "p_zbo": 23_500.0,
    "p_zbo_low": 7_200.0,
    "plume_heat": 100.0,
    "plume_sputter": 2e-6 / (1000.0 * 3600.0),
    "capacitance": 0.556,
    "resistor": 3.6,
    "plenum_tau": 0.65,
}


_TOLERANCES = {
    "isp_th": 0.01,
    "thrust": 0.03,
    "delta_v_th": 0.005,
    "heater_power": 0.03,
    "delta_p_line": 0.10,
    "injector_area": 0.02,
    "pump_power": 0.10,
    "npsh_margin": 0.10,
    "bartz_q": 0.05,
    "regen_alpha": 0.10,
    "radiator_tau": 0.10,
    "ep_thrust": 0.03,
    "ep_mdot": 0.03,
    "ep_waste": 0.05,
    "ep_area": 0.05,
    "ep_area_zbo": 0.05,
    "ep_duration": 0.03,
    "ep_prop": 0.05,
    "boiloff": 0.05,
    "boiloff_low": 0.05,
    "p_zbo": 0.05,
    "p_zbo_low": 0.05,
    "plume_heat": 0.10,
    "plume_sputter": 0.20,
    "capacitance": 0.05,
    "plenum_tau": 0.10,
}


_DEF_RESULTS_KEYS = [
    "m0",
    "dry_payload",
    "thermal_prop",
    "m1",
    "isp_th",
    "thrust",
    "delta_v_th",
    "heater_power",
    "delta_p_line",
    "injector_area",
    "pump_power",
    "npsh_margin",
    "bartz_q",
    "regen_alpha",
    "radiator_tau",
    "ep_thrust",
    "ep_mdot",
    "ep_waste",
    "ep_area",
    "ep_area_zbo",
    "ep_duration",
    "ep_prop",
    "boiloff",
    "boiloff_low",
    "p_zbo",
    "p_zbo_low",
    "plume_heat",
    "plume_sputter",
    "capacitance",
    "resistor",
    "plenum_tau",
]


def _mass_summary(cfg: Dict[str, Any]) -> Dict[str, float]:
    summary = {
        "m0": cfg["m0"],
        "dry_payload": cfg["dry_payload"],
        "thermal_prop": cfg["thermal_prop"],
        "m1": cfg["m1"],
    }
    if "m1_effective" in cfg:
        summary["m1_effective"] = cfg["m1_effective"]
    else:
        summary["m1_effective"] = cfg["m1"]
    return summary


def _evaluate_thermal(cfg: Dict[str, Any]) -> Dict[str, float]:
    nozzle_inputs = NozzleInputs(
        chamber_pressure=cfg["chamber_pressure"],
        chamber_temperature=cfg["chamber_temperature"],
        gamma=cfg["gamma"],
        gas_constant=cfg.get("gas_constant", HYDROGEN_R),
        expansion_ratio=cfg["expansion_ratio"],
        mass_flow=cfg["mass_flow"],
        ambient_pressure=cfg.get("ambient_pressure", 0.0),
    )
    nozzle_perf = compute_nozzle_performance(nozzle_inputs)
    burn_time = cfg["thermal_prop"] / nozzle_perf.mass_flow
    heater_power = nozzle_perf.mass_flow * cfg["cp"] * (
        cfg["chamber_temperature"] - cfg["inlet_temperature"]
    )
    delta_v = G0 * nozzle_perf.specific_impulse * math.log(cfg["m0"] / cfg["m1_effective"])

    bartz_inputs = BartzInputs(
        throat_diameter=math.sqrt(4 * nozzle_perf.throat_area / math.pi),
        gas_viscosity=cfg["gas_viscosity"],
        gas_cp=cfg["cp"],
        prandtl=cfg["prandtl"],
        chamber_temperature=cfg["chamber_temperature"],
        wall_temperature=cfg["wall_temperature"],
        gamma=cfg["gamma"],
        mach=nozzle_perf.exit_mach,
        characteristic_velocity=nozzle_perf.characteristic_velocity,
        chamber_pressure=cfg["chamber_pressure"],
        area_ratio_local=1.0,
        radius_of_curvature=cfg["radius_of_curvature"],
    )
    bartz = evaluate_bartz(bartz_inputs)

    regen_inputs = RegenInputs(
        coolant_mass_flow=cfg["coolant_split"],
        coolant_latent_heat=cfg["coolant_latent_heat"],
        coolant_cp=cfg["coolant_cp"],
        inlet_temperature=cfg["coolant_inlet"],
        outlet_temperature=cfg["coolant_outlet"],
        wall_temperature=cfg["wall_temperature"],
        peak_heat_flux=bartz.heat_flux,
        reference_area=cfg["throat_area_ref"],
    )
    regen = evaluate_regen(regen_inputs)

    radiator_tau = cfg["radiator_mass"] * cfg["radiator_cp"] / (
        4
        * cfg["radiator_emissivity"]
        * SIGMA
        * cfg["radiator_area"]
        * cfg["radiator_temperature"] ** 3
    )

    return {
        "isp_th": nozzle_perf.specific_impulse,
        "thrust": nozzle_perf.thrust,
        "delta_v_th": delta_v,
        "heater_power": heater_power,
        "burn_time": burn_time,
        "bartz_q": bartz.heat_flux,
        "regen_alpha": regen.required_split,
        "radiator_tau": radiator_tau,
        "throat_area": nozzle_perf.throat_area,
        "nozzle": nozzle_perf,
    }


def _evaluate_feed(cfg: Dict[str, Any], thermal: Dict[str, Any]) -> Dict[str, float]:
    if "mass_flow" in cfg:
        mass_flow = cfg["mass_flow"]
    else:
        nozzle = thermal.get("nozzle")
        if nozzle is None:
            raise KeyError(
                "Feed configuration requires 'mass_flow' or thermal results with 'nozzle'."
            )
        mass_flow = nozzle.mass_flow
    feed_inputs = FeedInputs(
        mass_flow=mass_flow,
        density=cfg["density"],
        viscosity=cfg["viscosity"],
        length=cfg["length"],
        diameter=cfg["diameter"],
        roughness=cfg["roughness"],
        minor_losses=cfg["minor_losses"],
        injector_cd=cfg["injector_cd"],
        injector_dp=cfg["injector_dp"],
        pump_dp=cfg["pump_dp"],
        pump_eff=cfg["pump_eff"],
        motor_eff=cfg["motor_eff"],
        tank_pressure=cfg["tank_pressure"],
        vapor_pressure=cfg["vapor_pressure"],
        elevation=cfg["elevation"],
    )
    feed = evaluate_feed(feed_inputs)
    npsh_margin = feed.npsh_available
    return {
        "delta_p_line": feed.total_line_drop,
        "injector_area": feed.injector_area,
        "pump_power": feed.pump_power,
        "npsh_margin": npsh_margin,
        "cavitation_risk": feed.cavitation_risk,
        "friction_model": feed.friction_model,
        "feed": feed,
    }


def _evaluate_press(cfg: Dict[str, Any]) -> Dict[str, float]:
    press_inputs = PressurizationInputs(
        tank_pressure=cfg["tank_pressure"],
        gas_temperature=cfg["gas_temperature"],
        ullage_volume_flow=cfg["ullage_flow"],
        liquid_density=cfg["liquid_density"],
        liquid_mass_flow=cfg["liquid_mass_flow"],
        gas_constant=cfg.get("gas_constant", HYDROGEN_R),
    )
    press = evaluate_pressurization(press_inputs)
    return {"pressurization_gas": press.gas_mass_flow}


def _evaluate_ep(cfg: Dict[str, Any]) -> Dict[str, float]:
    ep_inputs = EPInputs(
        power_in=cfg["power"],
        thruster_efficiency=cfg["thruster_efficiency"],
        ppu_efficiency=cfg["ppu_efficiency"],
        specific_impulse=cfg["specific_impulse"],
        radiator_emissivity=cfg["radiator_emissivity"],
        radiator_temperature=cfg["radiator_temperature"],
    )
    ep = evaluate_ep(ep_inputs)
    ep_zbo_inputs = EPInputs(
        power_in=cfg["power"],
        thruster_efficiency=cfg["thruster_efficiency"],
        ppu_efficiency=cfg["ppu_efficiency"],
        specific_impulse=cfg["specific_impulse"],
        radiator_emissivity=cfg["radiator_emissivity"],
        radiator_temperature=cfg["radiator_temperature_zbo"],
    )
    ep_zbo = evaluate_ep(ep_zbo_inputs)

    mission_inputs = EPDurationInputs(
        delta_v=cfg["segment_delta_v"],
        initial_mass=cfg["initial_mass"],
        ep_inputs=ep_inputs,
    )
    duration = evaluate_ep_duration(mission_inputs)

    plane_change = estimate_plane_change_time(
        orbital_speed=cfg["plane_change_speed"],
        delta_inclination=math.radians(cfg["plane_change_delta_i"]),
        vehicle_mass=cfg["initial_mass"],
        inputs=ep_inputs,
    )

    return {
        "ep_thrust": ep.thrust,
        "ep_mdot": ep.mass_flow,
        "ep_waste": ep.waste_power,
        "ep_area": ep.radiator_area,
        "ep_area_zbo": ep_zbo.radiator_area,
        "ep_duration": duration.duration,
        "ep_prop": duration.propellant_used,
        "plane_change_time": plane_change,
        "ep": ep,
    }


def _evaluate_cryo(cfg: Dict[str, Any]) -> Dict[str, float]:
    base = evaluate_cryo(
        CryoInputs(area=cfg["area"], heat_flux=cfg["heat_flux_base"], cop=cfg["cop"])
    )
    low = evaluate_cryo(
        CryoInputs(area=cfg["area"], heat_flux=cfg["heat_flux_low"], cop=cfg["cop"])
    )
    return {
        "boiloff": base.boiloff_rate_per_day,
        "boiloff_low": low.boiloff_rate_per_day,
        "p_zbo": base.zbo_power,
        "p_zbo_low": low.zbo_power,
    }


def _evaluate_plume(cfg: Dict[str, Any]) -> Dict[str, float]:
    plume_inputs = PlumeInputs(
        exponent=cfg["exponent"],
        reference_flux=cfg["reference_flux"],
        reference_current=cfg["reference_current"],
        sputter_yield=cfg["sputter_yield"],
        ion_energy=cfg["ion_energy"],
    )
    results = evaluate_plume(
        plume_inputs,
        theta=math.radians(cfg["keepout_angle"]),
        radius=cfg["keepout_radius"],
    )
    return {
        "plume_heat": results.heat_flux,
        "plume_sputter": results.sputter_rate,
    }


def _evaluate_powerbus(cfg: Dict[str, Any]) -> Dict[str, float]:
    inputs = ModeSwitchInputs(
        bus_voltage=cfg["bus_voltage"],
        step_power=cfg["step_power"],
        step_time=cfg["step_time"],
        resistor=cfg["resistor"],
        droop_voltage=cfg["droop_voltage"],
    )
    results = evaluate_mode_switch(inputs)
    return {
        "capacitance": results.required_capacitance,
        "resistor": cfg["resistor"],
        "peak_current": results.peak_current,
    }


def _evaluate_plenum(cfg: Dict[str, Any], thermal: Dict[str, Any]) -> Dict[str, float]:
    inputs = PlenumInputs(
        volume=cfg["volume"],
        characteristic_velocity=thermal["nozzle"].characteristic_velocity,
        gas_constant=cfg.get("gas_constant", HYDROGEN_R),
        chamber_temperature=cfg["chamber_temperature"],
        throat_area=thermal["throat_area"],
    )
    results = evaluate_plenum(inputs)
    return {"plenum_tau": results.time_constant}


def _make_output_dir(base: Path | None) -> Path:
    if base is None:
        base = Path("outputs")
    timestamp = datetime.utcnow().strftime("%Y%m%d-%H%M%S")
    path = base / timestamp
    path.mkdir(parents=True, exist_ok=True)
    return path


def _write_svg_line_plot(
    path: Path,
    x: Sequence[float],
    y: Sequence[float],
    x_label: str,
    y_label: str,
    title: str,
) -> None:
    width, height = 640, 400
    margin = 60
    x_min, x_max = float(min(x)), float(max(x))
    y_min, y_max = float(min(y)), float(max(y))
    if x_max == x_min:
        x_max += 1.0
    if y_max == y_min:
        y_max += 1.0

    def scale_x(val: float) -> float:
        return margin + (val - x_min) / (x_max - x_min) * (width - 2 * margin)

    def scale_y(val: float) -> float:
        return height - margin - (val - y_min) / (y_max - y_min) * (height - 2 * margin)

    points = " ".join(f"{scale_x(xi)},{scale_y(yi)}" for xi, yi in zip(x, y))
    circles = "\n".join(
        f'<circle cx="{scale_x(xi):.2f}" cy="{scale_y(yi):.2f}" r="4" fill="#1f77b4" />'
        for xi, yi in zip(x, y)
    )

    x_axis = (
        f'<line x1="{margin}" y1="{height - margin}" x2="{width - margin}" y2="{height - margin}" '
        "stroke=\"#000\" stroke-width=\"2\" />"
    )
    y_axis = (
        f'<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height - margin}" '
        "stroke=\"#000\" stroke-width=\"2\" />"
    )
    polyline = (
        f'<polyline fill="none" stroke="#1f77b4" stroke-width="2" points="{points}" />'
        if len(x) > 1
        else ""
    )
    svg = f"""<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{width}\" height=\"{height}\">\n"""
    svg += x_axis + "\n" + y_axis + "\n" + polyline + "\n" + circles + "\n"
    svg += f'<text x="{width/2:.1f}" y="{margin/2:.1f}" text-anchor="middle" font-size="18">{title}</text>\n'
    svg += f'<text x="{width/2:.1f}" y="{height-10}" text-anchor="middle" font-size="14">{x_label}</text>\n'
    svg += f'<text x="{15}" y="{height/2:.1f}" transform="rotate(-90 15,{height/2:.1f})" text-anchor="middle" font-size="14">{y_label}</text>\n'
    svg += "</svg>\n"
    path.write_text(svg, encoding="utf-8")


def _write_csv(path: Path, rows: Sequence[Dict[str, float]]) -> None:
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _generate_trade_sweeps(outdir: Path, cfg: Config) -> Dict[str, list[Dict[str, float]]]:
    trades: Dict[str, list[Dict[str, float]]] = {}

    ep_cfgs = cfg.section("ep_trades")
    ep_rows: list[Dict[str, float]] = []
    for entry in ep_cfgs:
        ep_inputs = EPInputs(
            power_in=entry["power"],
            thruster_efficiency=entry["thruster_efficiency"],
            ppu_efficiency=entry["ppu_efficiency"],
            specific_impulse=entry["specific_impulse"],
            radiator_emissivity=entry["radiator_emissivity"],
            radiator_temperature=entry["radiator_temperature"],
        )
        ep = evaluate_ep(ep_inputs)
        mission_inputs = EPDurationInputs(
            delta_v=entry["segment_delta_v"],
            initial_mass=entry["initial_mass"],
            ep_inputs=ep_inputs,
        )
        duration = evaluate_ep_duration(mission_inputs)
        ep_rows.append(
            {
                "power_kW": entry["power"] / 1000.0,
                "thrust_N": ep.thrust,
                "radiator_area_m2": ep.radiator_area,
                "duration_days": duration.duration / 86400.0,
                "prop_mass_t": duration.propellant_used / 1000.0,
            }
        )
    ep_rows = sorted(ep_rows, key=lambda item: item["power_kW"])
    trades["ep_trades"] = ep_rows

    zbo_cfgs = cfg.section("zbo_trades")
    zbo_rows: list[Dict[str, float]] = []
    for entry in zbo_cfgs:
        cryo = evaluate_cryo(
            CryoInputs(area=entry["area"], heat_flux=entry["heat_flux"], cop=entry["cop"])
        )
        zbo_rows.append(
            {
                "heat_flux_W_m2": entry["heat_flux"],
                "boiloff_kg_day": cryo.boiloff_rate_per_day,
                "zbo_power_kW": cryo.zbo_power / 1000.0,
            }
        )
    zbo_rows = sorted(zbo_rows, key=lambda item: item["heat_flux_W_m2"])
    trades["zbo_trades"] = zbo_rows

    _write_svg_line_plot(
        outdir / "ep_transfer_duration.svg",
        [row["power_kW"] for row in ep_rows],
        [row["duration_days"] for row in ep_rows],
        "EP Power (kW)",
        "Transfer duration (days)",
        "EP Transfer Duration",
    )
    _write_svg_line_plot(
        outdir / "ep_radiator_area.svg",
        [row["power_kW"] for row in ep_rows],
        [row["radiator_area_m2"] for row in ep_rows],
        "EP Power (kW)",
        "Radiator area (m^2)",
        "EP Radiator Area",
    )
    _write_svg_line_plot(
        outdir / "zbo_boiloff.svg",
        [row["heat_flux_W_m2"] for row in zbo_rows],
        [row["boiloff_kg_day"] for row in zbo_rows],
        "Heat flux (W/m^2)",
        "Boil-off (kg/day)",
        "Cryogenic Boil-off",
    )
    _write_svg_line_plot(
        outdir / "zbo_power.svg",
        [row["heat_flux_W_m2"] for row in zbo_rows],
        [row["zbo_power_kW"] for row in zbo_rows],
        "Heat flux (W/m^2)",
        "ZBO Power (kW)",
        "ZBO Power",
    )

    return trades


def _generate_docs(outdir: Path, summary: Dict[str, float]) -> None:
    docs_dir = outdir / "docs"
    docs_dir.mkdir(exist_ok=True)
    equations = """# BNTE Governing Equations\n\n"""
    equations += "- Rocket eqn: dv = g0 * Isp * ln(m0/mf)\n"
    equations += "- EP core: ve = g0Isp_EP; F = 2η_t*P_in/ve; mdot_EP = F/ve; P_waste = (1−η_t+1−η_PPU)*P_in\n"
    equations += "- Radiator: A = P_waste/(εσT_rad^4)\n"
    equations += "- ZBO: P_ZBO = Qdot_total / COP; ṁ_bo = Qdot_total/h_fg\n"
    equations += "- Autogenous press: ṁ_g = (P_t/(R_H2*T_g)) * V̇_ullage; V̇_ullage = ṁ_liq/ρ_l\n"
    equations += "- Darcy–Weisbach: ΔP_fric = f*(L/D)(½ρv²); ΔP_minor = KΣ(½ρv²)\n"
    equations += "- Injector area: A_inj = mdot / (C_dsqrt(2ρ_l*ΔP_inj))\n"
    equations += "- Pump power: P_elec = (ΔP_pumpQ)/(η_pη_e)\n"
    equations += "- NPSH: NPSH_a = P_t/(ρg) − P_v/(ρg) + z − h_f − v²/(2g)\n"
    equations += "- Nozzle finite-ϵ: Solve area–Mach for M_e; then T_e, P_e, ρ_e; v_e = M_esqrt(γRT_e); F = ṁv_e + (P_e − P_a)A_e; c = P0A_t/ṁ; C_F = F/(P0A_t)\n"
    equations += "- Bartz throat h_g:\n"
    equations += "  h_g = 0.026D_t^(−0.2)μ_g^0.2c_p_g/Pr_g^0.6(P0/c*)^0.8*(A_t/A_x)^0.9*(D_t/r_c)^0.1σ\n"
    equations += "  σ = [0.5(T_w/T0)+0.5]^(-0.68)(1+0.5(γ−1)*M^2)^(-0.12)\n"
    equations += "- Regen headroom: Qdot_cool = αṁ(h_fg + c_p_g*(300−20)); choose α to exceed peak throat load\n"
    equations += "- Constant-power EP duration: t = m_start*(1−exp(−dv/ve))ve²/(2η_t*P_in)\n"
    equations += "- Plane change time: t_pc ≈ 2vsin(Δi/2) * m * ve /(2η_tP_in)\n"
    equations += "- Plenum time constant: τ_p = V_p * (c*/(RT0A_t))\n"
    equations += "- Radiator thermal τ: τ_rad = (m_radc_p)/(4εσA*T^3)\n"
    equations += "- Plume fields: energy/pressure/current ∝ cos^k(θ)/r²; ion flux ϕ_i = j/e; sputter δ̇ = ϕ_i*Y(E)/n\n"
    (docs_dir / "equations.md").write_text(equations, encoding="utf-8")

    icd = """# BNTE ICD Highlights\n\n## Keep-out\n- Heat flux at keep-out: {plume_heat:.2f} W/m^2\n- Sputter rate: {plume_sputter:.3e} m/s\n\n## ZBO\n- Nominal boil-off: {boiloff:.1f} kg/day\n- ZBO power: {p_zbo:.1f} kW\n\n## Mode switch\n- Bus capacitance: {capacitance:.3f} F\n- Peak current: {peak_current:.1f} A\n""".format(
        plume_heat=summary["plume_heat"],
        plume_sputter=summary["plume_sputter"],
        boiloff=summary["boiloff"],
        p_zbo=summary["p_zbo"] / 1000.0,
        capacitance=summary["capacitance"],
        peak_current=summary["peak_current"],
    )
    (docs_dir / "icd.md").write_text(icd, encoding="utf-8")


def _summarize(results: Dict[str, float]) -> Dict[str, float]:
    summary = {key: results[key] for key in _DEF_RESULTS_KEYS if key in results}
    summary["peak_current"] = results.get("peak_current", 0.0)
    summary["burn_time"] = results.get("burn_time", 0.0)
    return summary


def run_simulation(cfg_path: Path, outdir: Path | None = None) -> SimulationResults:
    cfg = Config.from_file(cfg_path)
    outdir = _make_output_dir(outdir)

    masses = _mass_summary(cfg.section("masses"))
    thermal_cfg = {**cfg.section("thermal"), **masses}
    thermal = _evaluate_thermal(thermal_cfg)
    feed_cfg = cfg.section("feed")
    feed_cfg["mass_flow"] = thermal_cfg["mass_flow"]
    feed = _evaluate_feed(feed_cfg, thermal)
    press = _evaluate_press(cfg.section("press"))
    ep = _evaluate_ep(cfg.section("ep"))
    cryo = _evaluate_cryo(cfg.section("cryo"))
    plume = _evaluate_plume(cfg.section("plume"))
    powerbus = _evaluate_powerbus(cfg.section("powerbus"))
    plenum = _evaluate_plenum(cfg.section("plenum"), thermal_cfg | thermal)

    summary = {
        **masses,
        **thermal,
        **feed,
        **press,
        **ep,
        **cryo,
        **plume,
        **powerbus,
        **plenum,
    }
    summary_scalar = {k: v for k, v in summary.items() if isinstance(v, (int, float))}
    trades = _generate_trade_sweeps(outdir, cfg)
    stage_cfg = cfg.data.get("stage3d", {}) if isinstance(cfg.data, dict) else {}
    stage_outputs = generate_stage_assets(outdir, stage_cfg, summary_scalar)
    stage_summary = stage_outputs.metadata.get("stage", {})
    physics_engine = PhysicsEngine(outdir)
    physics_result = physics_engine.simulate(stage_outputs.components)
    validation = generate_validation_harness()

    summary_scalar.update(
        {
            "stage_mass": stage_summary.get("mass_total", 0.0),
            "stage_cg_z": stage_summary.get("cg", (0.0, 0.0, 0.0))[2],
        }
    )
    summary.update(summary_scalar)
    summary_rows = [summary_scalar]
    _write_csv(outdir / "summary.csv", summary_rows)
    for name, rows in trades.items():
        _write_csv(outdir / f"{name}.csv", rows)

    _generate_docs(outdir, summary)

    print("BNTE baseline summary:")
    for key in _DEF_RESULTS_KEYS:
        if key in summary:
            print(f"  {key}: {summary[key]:.6g}")

    validation_path = outdir / "validation.json"
    validation_path.write_text(json.dumps(validation, indent=2), encoding="utf-8")

    trajectory_rows = [
        {
            "time": physics_result.times[idx],
            "x": physics_result.positions[idx][0],
            "y": physics_result.positions[idx][1],
            "z": physics_result.positions[idx][2],
            "vx": physics_result.velocities[idx][0],
            "vy": physics_result.velocities[idx][1],
            "vz": physics_result.velocities[idx][2],
        }
        for idx in range(len(physics_result.times))
    ]
    _write_csv(outdir / "physics_trajectory.csv", trajectory_rows)

    tables = {"summary": summary_rows, "physics_trajectory": trajectory_rows, **trades}
    metadata = {
        "output_dir": str(outdir),
        "config": str(cfg_path),
        "stage": stage_outputs.metadata,
        "physics": str(physics_result.output_path),
    }

    stage_summary = stage_outputs.metadata.get("stage", {})
    summary.update(summary_scalar)

    return SimulationResults(summary=_summarize(summary), tables=tables, metadata=metadata)


__all__ = ["run_simulation", "SimulationResults", "TARGETS"]
