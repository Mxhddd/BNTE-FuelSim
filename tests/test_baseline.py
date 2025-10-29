from __future__ import annotations

from pathlib import Path
import math

from bnte.mission.runner import TARGETS, run_simulation


def test_baseline_targets(tmp_path: Path) -> None:
    results = run_simulation(Path("configs/baseline.yml"), outdir=tmp_path)
    summary = results.summary
    for key, target in TARGETS.items():
        if key not in summary:
            continue
        value = summary[key]
        if target == 0:
            assert abs(value) < 1e-6
            continue
        if key in {"m0", "dry_payload", "thermal_prop", "m1"}:
            assert math.isclose(value, target, rel_tol=0.0, abs_tol=1e-6)
        elif key == "resistor":
            assert math.isclose(value, target, rel_tol=0.0, abs_tol=1e-3)
        else:
            rel_err = abs(value - target) / abs(target)
            tol = 0.05
            if key == "delta_v_th":
                tol = 0.005
            elif key in {"heater_power", "ep_waste"}:
                tol = 0.03
            elif key in {"ep_area", "ep_area_zbo", "pump_power"}:
                tol = 0.05
            elif key in {"delta_p_line", "ep_thrust", "ep_mdot"}:
                tol = 0.03
            elif key in {"bartz_q", "regen_alpha", "radiator_tau"}:
                tol = 0.05
            elif key in {"boiloff", "boiloff_low", "p_zbo", "p_zbo_low"}:
                tol = 0.05
            elif key in {"plume_heat", "plume_sputter", "capacitance", "plenum_tau"}:
                tol = 0.10
            assert rel_err <= tol, f"{key}={value} target={target}"

    output_dir = Path(results.metadata["output_dir"])
    assert (output_dir / "summary.csv").exists()
    assert (output_dir / "docs" / "equations.md").exists()
    assert (output_dir / "docs" / "icd.md").exists()


