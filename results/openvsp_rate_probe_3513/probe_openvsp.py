from __future__ import annotations

import json
import sys
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT / "src"))

from config_loader import load_project_config
from coordinate_system import map_stability_derivatives, map_unsteady_derivatives
from main import _startup
from validation import calculate_condition


if len(sys.argv) != 2 or sys.argv[1].lower() not in {"steady", "p", "q", "r"}:
    raise SystemExit("usage: probe_openvsp.py steady|p|q|r")
mode = sys.argv[1].lower()
config = load_project_config(PROJECT / "config" / "aircraft.yaml")
context = _startup(config)
condition = calculate_condition(
    speed_mps=8.0,
    alpha_deg=4.268906787204264,
    beta_deg=0.0,
    atmosphere=config["atmosphere"],
    cref_m=context["reference"]["cref_m"],
)
controls = {"aileron": 0.0, "elevator": -5.952486745549647, "rudder": 0.0}
raw_root = Path(__file__).resolve().parent / f"raw_verified_{mode}"
raw_root.mkdir(parents=True, exist_ok=True)
result = context["runner"].run(
    condition,
    raw_root,
    mode,
    stability=True,
    stability_mode=mode,
    include_thick=True,
    control_deflections_deg=controls,
    wake_iterations=3,
)
mapped = (
    map_stability_derivatives(result.raw_data)
    if mode == "steady"
    else map_unsteady_derivatives(result.raw_data, mode)
)
report = {
    "status": "PASS",
    "openvsp_version": context["version"],
    "analysis_input_names": list(context["runner"].vsp.GetAnalysisInputNames("VSPAEROSweep")),
    "enums": {
        name: int(getattr(context["runner"].vsp, name))
        for name in (
            "STABILITY_OFF",
            "STABILITY_DEFAULT",
            "STABILITY_P_ANALYSIS",
            "STABILITY_Q_ANALYSIS",
            "STABILITY_R_ANALYSIS",
        )
    },
    "mode": mode,
    "condition": condition,
    "controls": controls,
    "wake_iterations": 3,
    "duration_sec": result.duration_sec,
    "result_id": result.data_result_id,
    "result_fields": result.raw_data,
    "mapped_derivatives": mapped,
    "files": sorted(path.name for path in result.case_dir.iterdir() if path.is_file()),
}
(Path(__file__).resolve().parent / f"probe_{mode}.json").write_text(
    json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
)
print(mode, result.duration_sec, len(result.raw_data), sorted(result.raw_data), flush=True)
