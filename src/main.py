from __future__ import annotations

import argparse
import hashlib
import json
import math
import platform
import shutil
import traceback
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

from case_generator import (
    CaseSpec,
    generate_grid_cases,
    generate_trim_cases,
    load_completed_result,
    stable_signature,
    trim_case_id,
)
from config_loader import ConfigError, load_openvsp_config, load_project_config, locate_openvsp
from coordinate_system import (
    COORDINATE_CONVENTION,
    RATE_CASE,
    map_control_derivatives,
    map_polar_coefficients,
    map_stability_baseline,
    map_stability_case,
    map_stability_derivatives,
)
from export_results import build_database, export_database
from finite_difference import calculate_trim_derivatives, combine_status
from numerical_convergence import (
    convergence_identity,
    load_production_settings,
    production_gate,
    query_wake_schedule,
    run_numerical_convergence,
    trim_wake_decision,
)
from openvsp_interface import GeometrySelection, OpenVSPError, OpenVSPModel, load_openvsp_api
from regression import compare_regression
from trim_solver import solve_longitudinal_trim
from validation import (
    ValidationError,
    calculate_condition,
    scan_portability,
    summarize_dataset,
    validate_analysis_payload,
    validate_trim_result,
)
from vspaero_runner import AeroRunResult, VSPAERORunner


INTERNAL_SCHEMA_VERSION = "6.0.0"
AUTOTUNE_SCHEMA_VERSION = "1.0"
TOOL_VERSION = "6.0"
COEFFICIENT_NAMES = ("CL", "CD", "Cm", "CY", "Cl", "Cn")


def _now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items() if not str(key).startswith("_")}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(_json_safe(payload), ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    temporary.replace(path)


def _reset_raw(case_dir: Path) -> Path:
    case_dir.mkdir(parents=True, exist_ok=True)
    raw_dir = (case_dir / "raw").resolve()
    if not raw_dir.is_relative_to(case_dir.resolve()):
        raise RuntimeError(f"Unsafe raw output path: {raw_dir}")
    if raw_dir.exists():
        shutil.rmtree(raw_dir)
    raw_dir.mkdir(parents=True)
    return raw_dir


def _reference_values(model_values: dict[str, float], config: dict[str, Any]) -> dict[str, float]:
    reference_config = config["reference"]
    values = dict(model_values)
    if str(reference_config["source"]).lower() == "config":
        for name in ("sref_m2", "bref_m", "cref_m"):
            values[name] = float(reference_config[name])
    if str(reference_config.get("cg_source", reference_config["source"])).lower() == "config":
        for name in ("xcg_m", "ycg_m", "zcg_m"):
            values[name] = float(reference_config[name])
    for name in ("sref_m2", "bref_m", "cref_m"):
        if values[name] <= 0 or not math.isfinite(values[name]):
            raise ConfigError(f"Reference value is invalid: {name}={values[name]}")
    return values


def _condition(
    spec: CaseSpec, alpha_deg: float, config: dict[str, Any], reference: dict[str, float]
) -> dict[str, float]:
    return calculate_condition(
        speed_mps=spec.speed_mps,
        alpha_deg=alpha_deg,
        beta_deg=spec.beta_deg,
        atmosphere=config["atmosphere"],
        cref_m=reference["cref_m"],
    )


def _analysis_payload(raw_run: AeroRunResult, controls: dict[str, str]) -> dict[str, Any]:
    rate_cases: dict[str, Any] = {}
    for variable, case_name in RATE_CASE.items():
        try:
            rate_cases[variable] = map_stability_case(raw_run.raw_data, case_name)
        except ValueError:
            rate_cases[variable] = {}
    payload = {
        "coefficients": map_stability_baseline(raw_run.raw_data),
        "native_derivative_diagnostics": {
            "stability": map_stability_derivatives(raw_run.raw_data),
            "controls": map_control_derivatives(raw_run.raw_data, controls),
            "positive_rate_cases": rate_cases,
            "diagnostic_only": True,
            "enters_production_gate": False,
            "enters_trim_jacobian": False,
        },
    }
    payload["warnings"] = validate_analysis_payload(payload)
    payload["solver_run"] = {
        "analysis": raw_run.analysis,
        "geometry_result_id": raw_run.geometry_result_id,
        "sweep_result_id": raw_run.sweep_result_id,
        "data_result_id": raw_run.data_result_id,
        "duration_sec": raw_run.duration_sec,
        "raw_directory": str(raw_run.case_dir),
    }
    return payload


def _signature_context(
    config: dict[str, Any], openvsp_version: str, model_sha256: str,
    reference: dict[str, float], geometry: GeometrySelection,
) -> dict[str, Any]:
    derivative_signature = dict(config["derivatives"])
    # The loaded manifest content is already signed below.  Use only its file
    # name here so equivalent configs in different directories share cache.
    derivative_signature["manifest"] = config["_paths"]["manifest"].name
    return {
        "schema_version": INTERNAL_SCHEMA_VERSION,
        "tool_version": TOOL_VERSION,
        "openvsp_version": openvsp_version,
        "model_sha256": model_sha256,
        "reference": reference,
        "geometry": {
            "thin_set_index": geometry.thin_set_index,
            "thick_set_index": geometry.thick_set_index,
            "thin_ids": [item["id"] for item in geometry.thin],
            "thick_ids": [item["id"] for item in geometry.thick],
        },
        "solver": config["solver"],
        "atmosphere": config["atmosphere"],
        "controls": config["controls"],
        "manifest": _json_safe(config["_manifest"]),
        "derivatives": derivative_signature,
        "validation": config["validation"],
        "numerical_convergence": config["numerical_convergence"],
    }


def _grid_case(
    spec: CaseSpec, *, runner: VSPAERORunner, config: dict[str, Any],
    reference: dict[str, float], cases_root: Path,
    signature_context: dict[str, Any], control_names: dict[str, str],
    production_settings: dict[str, Any] | None,
) -> tuple[dict[str, Any], bool]:
    if spec.alpha_deg is None:
        raise RuntimeError("GRID case has no alpha")
    condition = _condition(spec, spec.alpha_deg, config, reference)
    wake_iterations = int(config["solver"]["wake_iterations"])
    schedule_query: dict[str, Any] = {"source": "configured_fallback"}
    if production_settings is not None:
        schedule_query = query_wake_schedule(production_settings["wake_schedule"], condition)
        wake_iterations = int(schedule_query["wake_iterations"])
    numerical = {
        "wake_iterations": wake_iterations,
        "wake_schedule_query": schedule_query,
        "mesh_source": "current_openvsp_model",
    }
    signature = stable_signature({
        **signature_context, "mode": spec.mode, "condition": condition, "numerical": numerical,
    })
    case_dir = cases_root / spec.case_id
    result_path = case_dir / "result.json"
    if config["_resume_enabled"]:
        cached = load_completed_result(result_path, signature)
        if cached is not None:
            return cached, True
    raw_dir = _reset_raw(case_dir)
    started = _now()
    try:
        raw = runner.run(
            condition, raw_dir, "stability", stability=True, include_thick=True,
            wake_iterations=wake_iterations,
        )
        outputs = _analysis_payload(raw, control_names)
        result = {
            "schema_version": INTERNAL_SCHEMA_VERSION,
            "signature": signature,
            "case_id": spec.case_id,
            "mode": spec.mode,
            "status": "PASS",
            "timestamp": started,
            "inputs": condition,
            "outputs": outputs,
            "solver": {"status": "SUCCESS", "duration_sec": raw.duration_sec},
            "numerical_settings": numerical,
        }
    except Exception as exc:
        result = {
            "schema_version": INTERNAL_SCHEMA_VERSION,
            "signature": signature,
            "case_id": spec.case_id,
            "mode": spec.mode,
            "status": "FAIL",
            "timestamp": started,
            "inputs": condition,
            "outputs": {},
            "solver": {"status": "FAIL"},
            "error": f"{type(exc).__name__}: {exc}",
        }
    _write_json(result_path, result)
    return result, False


def _trim_case(
    spec: CaseSpec, *, runner: VSPAERORunner, config: dict[str, Any],
    reference: dict[str, float], cases_root: Path,
    signature_context: dict[str, Any], control_names: dict[str, str],
    production_settings: dict[str, Any] | None,
) -> tuple[dict[str, Any], bool]:
    trim_config = config["trim"]
    fallback_wake = int(config["solver"]["wake_iterations"])
    if production_settings is None:
        schedule = None
        pretrim_wake = fallback_wake
        max_upgrades = 1
    else:
        schedule = production_settings["wake_schedule"]
        pretrim_wake = int(production_settings["trim"]["pretrim_wake_iterations"])
        max_upgrades = int(production_settings["trim"]["max_wake_upgrades"])
    signature = stable_signature({
        **signature_context, "mode": spec.mode,
        "speed_mps": spec.speed_mps, "beta_deg": spec.beta_deg, "trim": trim_config,
        "production_numerical_settings": production_settings,
    })
    case_dir = cases_root / spec.case_id
    result_path = case_dir / "result.json"
    if config["_resume_enabled"]:
        cached = load_completed_result(result_path, signature)
        if cached is not None:
            return cached, True
    raw_dir = _reset_raw(case_dir)
    started = _now()
    durations: list[float] = []
    last_condition = _condition(spec, float(trim_config["alpha"]["initial_deg"]), config, reference)

    try:
        def solve_stage(
            stage: str, wake_iterations: int, alpha_initial: float, elevator_initial: float,
        ) -> tuple[Any, dict[str, float]]:
            stage_condition = _condition(spec, alpha_initial, config, reference)
            stage_config = deepcopy(trim_config)
            stage_config["alpha"]["initial_deg"] = alpha_initial
            stage_config["elevator"]["initial_deg"] = elevator_initial

            def evaluate(
                alpha_deg: float, elevator_deg: float, label: str, stability: bool,
            ) -> dict[str, Any]:
                nonlocal stage_condition
                stage_condition = _condition(spec, alpha_deg, config, reference)
                raw = runner.run(
                    stage_condition, raw_dir, f"{stage}_{label}",
                    stability=stability, include_thick=True,
                    control_deflections_deg={"elevator": elevator_deg},
                    wake_iterations=wake_iterations,
                )
                durations.append(raw.duration_sec)
                if stability:
                    return _analysis_payload(raw, control_names)
                return {"coefficients": map_polar_coefficients(raw.raw_data)}

            solved = solve_longitudinal_trim(
                evaluate=evaluate, trim_config=stage_config,
                derivative_config=config["derivatives"],
                dynamic_pressure_pa=stage_condition["dynamic_pressure_pa"], reference=reference,
            )
            return solved, stage_condition

        pretrim, _ = solve_stage(
            "pretrim", pretrim_wake,
            float(trim_config["alpha"]["initial_deg"]),
            float(trim_config["elevator"]["initial_deg"]),
        )
        if not pretrim.converged:
            raise RuntimeError(f"Pre-trim failed: {pretrim.failure_reason}")

        base_state = {
            "speed_mps": spec.speed_mps,
            "alpha_deg": pretrim.alpha_deg,
            "beta_deg": spec.beta_deg,
        }
        production_wake = (
            int(trim_wake_decision(schedule, base_state, config["derivatives"])["wake_iterations"])
            if schedule is not None else fallback_wake
        )
        production_history: list[dict[str, Any]] = []
        alpha_initial = pretrim.alpha_deg
        elevator_initial = pretrim.elevator_deg
        trim = pretrim
        for upgrade_index in range(max_upgrades + 1):
            trim, last_condition = solve_stage(
                f"production_{upgrade_index:02d}", production_wake,
                alpha_initial, elevator_initial,
            )
            production_history.append({
                "wake_iterations": production_wake,
                "converged": bool(trim.converged),
                "alpha_trim_deg": trim.alpha_deg,
                "elevator_trim_deg": trim.elevator_deg,
                "iterations": trim.iterations,
                "history": list(trim.history),
                "failure_reason": trim.failure_reason,
            })
            if not trim.converged or schedule is None:
                break
            final_state = {
                "speed_mps": spec.speed_mps,
                "alpha_deg": trim.alpha_deg,
                "beta_deg": spec.beta_deg,
            }
            wake_decision = trim_wake_decision(
                schedule, final_state, config["derivatives"], production_wake
            )
            if wake_decision["action"] == "ACCEPT":
                break
            if upgrade_index >= max_upgrades:
                raise RuntimeError(
                    "Production trim crossed into a higher Wake region after the configured upgrade limit"
                )
            production_wake = int(wake_decision["wake_iterations"])
            alpha_initial = trim.alpha_deg
            elevator_initial = trim.elevator_deg

        outputs = trim.analysis_payload
        coefficients = outputs.get("coefficients", {})
        controls = {
            "aileron": float(config["controls"]["aileron"].get("neutral_deg", 0.0)),
            "elevator": float(trim.elevator_deg),
            "rudder": float(config["controls"]["rudder"].get("neutral_deg", 0.0)),
        }
        trim_record = {
            "converged": bool(trim.converged),
            "speed_mps": spec.speed_mps,
            "alpha_trim_deg": trim.alpha_deg,
            "beta_trim_deg": spec.beta_deg,
            "elevator_trim_deg": trim.elevator_deg,
            **{
                f"{name}_trim": coefficients.get(name, {}).get("standard_value")
                for name in COEFFICIENT_NAMES
            },
            "trim_force_residual_n": trim.force_residual_n,
            "trim_moment_residual_nm": trim.moment_residual_nm,
            "trim_iterations": trim.iterations,
            "force_tolerance_n": float(trim_config["force_tolerance_n"]),
            "moment_tolerance_nm": float(trim_config["moment_tolerance_nm"]),
            "alpha_min_deg": float(trim_config["alpha"]["min_deg"]),
            "alpha_max_deg": float(trim_config["alpha"]["max_deg"]),
            "elevator_min_deg": float(trim_config["elevator"]["min_deg"]),
            "elevator_max_deg": float(trim_config["elevator"]["max_deg"]),
            "history": list(trim.history),
            "failure_reason": trim.failure_reason,
            "solver_method": "bounded Newton using real centered-difference Jacobian and backtracking line search",
            "pretrim": {
                "wake_iterations": pretrim_wake,
                "alpha_trim_deg": pretrim.alpha_deg,
                "elevator_trim_deg": pretrim.elevator_deg,
                "iterations": pretrim.iterations,
                "history": list(pretrim.history),
            },
            "production_history": production_history,
            "production_wake_iterations": production_wake,
            "derivative_bundle_wake_iterations": production_wake,
            "wake_upgrade_count": max(0, len(production_history) - 1),
            "mesh_source": "current_openvsp_model",
        }
        result: dict[str, Any] = {
            "schema_version": INTERNAL_SCHEMA_VERSION,
            "signature": signature,
            "case_id": spec.case_id,
            "mode": spec.mode,
            "status": "FAIL",
            "timestamp": started,
            "inputs": last_condition,
            "control_deflections_deg": controls,
            "outputs": outputs,
            "trim": trim_record,
            "derivatives": {
                "production_fd_derivatives": {},
                "native_derivative_diagnostics": {},
                "required_derivatives_manifest": {"items": [], "summary": {}},
                "run_count": 0,
                "solver_duration_sec": 0.0,
            },
            "solver": {
                "status": "SUCCESS" if trim.converged else "TRIM_NOT_CONVERGED",
                "duration_sec": sum(durations),
            },
            "numerical_settings": {
                "wake_iterations": production_wake,
                "derivative_bundle_rule": "max Wake over the complete centered-difference bundle",
                "mesh_source": "current_openvsp_model",
                "two_stage_trim": True,
            },
        }
        if trim.converged:
            def run_polar(
                condition: dict[str, float], label: str, deflections: dict[str, float]
            ) -> dict[str, Any]:
                raw = runner.run(
                    condition, raw_dir, label, stability=False, include_thick=True,
                    control_deflections_deg=deflections,
                    wake_iterations=production_wake,
                )
                return {
                    "coefficients": map_polar_coefficients(raw.raw_data),
                    "solver_duration_sec": raw.duration_sec,
                }

            derivative_config = deepcopy(config["derivatives"])
            derivative_config["_controls"] = config["controls"]
            derivative_config["_bundle_wake_iterations"] = production_wake
            if production_settings is not None:
                derivative_config["_preferred_fd_steps"] = dict(
                    production_settings.get("selected_fd_steps_deg", {})
                )
            derivative_package = calculate_trim_derivatives(
                condition=last_condition,
                base_outputs=outputs,
                base_controls=controls,
                manifest=config["_manifest"],
                derivative_config=derivative_config,
                run_polar=run_polar,
            )
            result["derivatives"] = derivative_package
            result["solver"]["duration_sec"] += derivative_package["solver_duration_sec"]
        else:
            result["error"] = trim.failure_reason
        validation, rows = validate_trim_result(result, config["_manifest"], config["validation"])
        result["validation"] = validation
        result["validation_rows"] = rows
        result["status"] = validation["overall_status"]
    except Exception as exc:
        result = {
            "schema_version": INTERNAL_SCHEMA_VERSION,
            "signature": signature,
            "case_id": spec.case_id,
            "mode": spec.mode,
            "status": "FAIL",
            "timestamp": started,
            "inputs": last_condition,
            "outputs": {},
            "derivatives": {
                "production_fd_derivatives": {},
                "native_derivative_diagnostics": {},
                "required_derivatives_manifest": {"items": [], "summary": {}},
            },
            "solver": {"status": "FAIL", "duration_sec": sum(durations)},
            "validation": {
                "solver_status": "FAIL", "trim_status": "FAIL", "numerical_status": "FAIL",
                "derivative_status": "FAIL", "physics_status": "FAIL", "overall_status": "FAIL",
            },
            "validation_rows": [],
            "error": f"{type(exc).__name__}: {exc}",
        }
    _write_json(result_path, result)
    return result, False


def _load_validation(path: Path, signature: str) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return data if data.get("signature") == signature and data.get("status") == "PASS" else None


def _fuselage_validation(
    *, nominal: dict[str, Any], runner: VSPAERORunner, validation_root: Path,
    signature_context: dict[str, Any], config: dict[str, Any],
    production_settings: dict[str, Any] | None,
) -> tuple[dict[str, Any], bool]:
    condition = nominal["inputs"]
    controls = nominal.get("control_deflections_deg")
    settings = {"minimum_absolute_change": 1.0e-8}
    signature = stable_signature({
        **signature_context, "validation": "fuselage_effect",
        "condition": condition, "controls": controls, "settings": settings,
        "numerical_settings": nominal.get("numerical_settings", {}),
        "production_identity": (production_settings or {}).get("identity"),
    })
    result_path = validation_root / "fuselage_effect" / "result.json"
    if config["_resume_enabled"]:
        cached = _load_validation(result_path, signature)
        if cached is not None:
            return cached, True
    raw_dir = _reset_raw(result_path.parent)
    try:
        thin_run = runner.run(
            condition, raw_dir, "thin_only", stability=False,
            include_thick=False, control_deflections_deg=controls,
            wake_iterations=int(
                nominal.get("numerical_settings", {}).get(
                    "wake_iterations", config["solver"]["wake_iterations"]
                )
            ),
        )
        thin_mapped = map_polar_coefficients(thin_run.raw_data)
        thin = {name: float(thin_mapped[name]["standard_value"]) for name in COEFFICIENT_NAMES}
        mixed = {
            name: float(nominal["outputs"]["coefficients"][name]["standard_value"])
            for name in COEFFICIENT_NAMES
        }
        delta = {name: mixed[name] - thin[name] for name in COEFFICIENT_NAMES}
        threshold = float(settings["minimum_absolute_change"])
        passed = all(math.isfinite(value) for value in delta.values()) and any(
            abs(value) > threshold for value in delta.values()
        )
        result = {
            "signature": signature,
            "status": "PASS" if passed else "FAIL",
            "case_id": nominal["case_id"],
            "condition": condition,
            "thin_only": thin,
            "thin_plus_thick": mixed,
            "delta": delta,
            "minimum_absolute_change": threshold,
            "max_absolute_change": max(abs(value) for value in delta.values()),
            "solver_duration_sec": thin_run.duration_sec,
        }
        if not passed:
            result["error"] = "No finite coefficient change exceeded the configured threshold"
    except Exception as exc:
        result = {
            "signature": signature, "status": "FAIL", "case_id": nominal.get("case_id"),
            "condition": condition, "error": f"{type(exc).__name__}: {exc}",
        }
    _write_json(result_path, result)
    return result, False


def _summary_text(summary: dict[str, Any], output_dir: Path) -> str:
    derivative = summary["derivatives"]
    lines = [
        "AERO DATASET BUILD SUMMARY",
        "",
        f"OpenVSP                 : {summary['openvsp_version']}",
        f"Geometry                : {summary['geometry_status']}",
        f"GRID                     : {summary['grid']['status']} "
        f"({summary['grid']['completed']}/{summary['grid']['requested']})",
        f"TRIM                     : {summary['trim']['status']} "
        f"({summary['trim']['completed']}/{summary['trim']['requested']})",
        f"Required derivatives     : {derivative['required_derivatives']}",
        f"Required instances       : {derivative['required_instances']}",
        f"Calculated               : {derivative['calculated']}",
        f"Missing                  : {derivative['missing']}",
        f"Invalid                  : {derivative['invalid']}",
        f"Validation failed        : {derivative['validation_failed']}",
        f"PASS                     : {derivative.get('PASS', 0)}",
        f"WARN_NUMERICAL           : {derivative.get('WARN_NUMERICAL', 0)}",
        f"METHOD_LIMITATION        : {derivative.get('METHOD_LIMITATION', 0)}",
        f"FAIL                     : {derivative.get('FAIL', 0)}",
        f"DERIVATIVE_SET           : {derivative['derivative_set_status']}",
        f"Fuselage effect          : {summary['fuselage_effect_status']}",
        f"Portability              : {summary['portability_status']}",
        f"Production Gate          : {summary['production_gate_status']}",
        f"DATASET                   : {summary['final_status']}",
        "",
        f"Output                   : {output_dir}",
    ]
    if summary.get("limitations"):
        lines.extend(["", "Known limitations:", *[f"- {item}" for item in summary["limitations"]]])
    return "\n".join(lines) + "\n"


def _startup(config: dict[str, Any]) -> dict[str, Any]:
    project = config["_paths"]["project_root"]
    location = locate_openvsp(load_openvsp_config(project))
    vsp = load_openvsp_api(location.root)
    version = str(vsp.GetVSPVersion())
    expected = str(load_openvsp_config(project).get("openvsp", {}).get("expected_version", "")).strip()
    if expected and expected not in version:
        raise OpenVSPError(f"Expected OpenVSP {expected}, API reports {version}")
    model = OpenVSPModel(vsp, config["_paths"]["model"])
    model.load()
    geometry = model.resolve_geometry_sets(config["geometry_sets"])
    model.apply_geometry_sets(geometry)
    if not geometry.thick:
        raise OpenVSPError("No thick geometry is selected; the fuselage would be excluded")
    reference = _reference_values(model.reference_quantities(), config)
    control_map = model.validate_control_mapping(config["controls"])
    control_names = {role: group.name for role, group in control_map.items()}
    model_sha256 = _sha256(config["_paths"]["model"])
    signature_context = _signature_context(config, version, model_sha256, reference, geometry)
    runner = VSPAERORunner(vsp, config["_paths"]["model"], config, reference, geometry)
    return {
        "project": project, "location": location, "version": version, "model": model,
        "geometry": geometry, "reference": reference, "control_map": control_map,
        "control_names": control_names, "model_sha256": model_sha256,
        "signature_context": signature_context, "runner": runner,
    }


def _print_startup(context: dict[str, Any], config: dict[str, Any]) -> None:
    geometry = context["geometry"]
    print(f"Loading aircraft: {config['aircraft'].get('name', config['_paths']['model'].stem)}")
    print(f"OpenVSP: {context['version']}")
    print(
        "Geometry validation: PASS "
        f"(thin={','.join(item['name'] for item in geometry.thin)}; "
        f"thick={','.join(item['name'] for item in geometry.thick)})"
    )
    print(f"Control mapping: PASS ({', '.join(context['control_names'].values())})")
    print(f"Required derivatives: {len(config['_manifest']['_required'])}")


def _run_openvsp_smoke(config: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    point = next(
        item for item in config["numerical_convergence"]["wake"]["representative_states"]
        if not bool(item.get("trim", False))
    )
    condition = calculate_condition(
        speed_mps=float(point["speed_mps"]), alpha_deg=float(point["alpha_deg"]),
        beta_deg=float(point.get("beta_deg", 0.0)), atmosphere=config["atmosphere"],
        cref_m=context["reference"]["cref_m"],
    )
    output_dir = config["_paths"]["results"] / "numerical_convergence" / "smoke"
    raw_dir = _reset_raw(output_dir)
    wake = int(config["numerical_convergence"]["wake"]["candidates"][0])
    raw = context["runner"].run(
        condition, raw_dir, "stability", stability=True, wake_iterations=wake,
    )
    payload = _analysis_payload(raw, context["control_names"])
    values = {
        name: float(payload["coefficients"][name]["standard_value"])
        for name in COEFFICIENT_NAMES
    }
    status = "PASS" if all(math.isfinite(value) for value in values.values()) else "FAIL"
    report = {
        "schema_version": "1.0", "status": status, "timestamp": _now(),
        "openvsp_version": context["version"], "model_sha256": context["model_sha256"],
        "condition": condition, "wake_iterations": wake,
        "mesh_source": "current_openvsp_model", "coefficients": values,
        "solver_duration_sec": raw.duration_sec,
    }
    _write_json(output_dir / "smoke_test.json", report)
    print(f"OpenVSP end-to-end smoke: {status}")
    return {"success": status == "PASS", "summary": {"final_status": status}, "report": report}


def run_workflow(
    command: str, config_path: str | Path | None = None, *, force: bool = False
) -> dict[str, Any]:
    if command == "regression" and config_path is None:
        config_path = Path(__file__).resolve().parents[1] / "tests" / "regression" / "regression.yaml"
    config = load_project_config(config_path)
    context = _startup(config)
    _print_startup(context, config)
    if command == "check":
        return {"success": True, "summary": {"final_status": "PASS"}}
    if command == "regression":
        return _run_regression(config, context)
    if command == "smoke":
        return _run_openvsp_smoke(config, context)
    if command == "numerical-convergence":
        report = run_numerical_convergence(
            config=config, runner=context["runner"], reference=context["reference"],
            manifest=config["_manifest"],
            analysis_mapper=lambda raw: _analysis_payload(raw, context["control_names"]),
        )
        status = str(report["production_gate_status"])
        print(f"Numerical convergence: {report['convergence_status']}")
        print(f"Production gate: {status}")
        return {"success": status != "FAIL", "summary": {"final_status": status}, "report": report}

    results_root = config["_paths"]["results"]
    production_settings = None
    gate_status = "NOT_REQUESTED"
    gate_config = config["numerical_convergence"]["production_gate"]
    if command in {"all", "grid", "trim"} and bool(gate_config["enabled"]):
        production_settings = load_production_settings(config["_paths"]["production_settings"])
        adaptive = str(config.get("grid", {}).get("mode", "uniform")).lower() == "adaptive"
        gate = production_gate(
            production_settings, force=force, adaptive=adaptive,
            expected_identity=convergence_identity(config, context["version"]),
        )
        gate_status = str(gate["status"])
        if not gate["allowed"]:
            raise RuntimeError(
                f"Production Gate {gate['status']}: {gate['reason']}. "
                "Run 'python run.py numerical-convergence' first, or use --force with an explicit audit record."
            )
        print(f"Production Gate: {gate['status']}{' (FORCED)' if gate['forced'] else ''}")
        if gate["forced"]:
            _write_json(results_root / "numerical_convergence" / "production_force_override.json", {
                "timestamp": _now(), "command": command, "gate": gate,
                "config_file": config["_paths"]["config_file"],
            })
    cases_root = results_root / "cases"
    grid_specs = generate_grid_cases(config) if command in {"all", "grid"} else []
    trim_specs = generate_trim_cases(config) if command in {"all", "trim"} else []
    grid_results: list[dict[str, Any]] = []
    trim_results: list[dict[str, Any]] = []
    grid_skipped = 0
    trim_skipped = 0

    for index, spec in enumerate(grid_specs, 1):
        result, skipped = _grid_case(
            spec, runner=context["runner"], config=config, reference=context["reference"],
            cases_root=cases_root, signature_context=context["signature_context"],
            control_names=context["control_names"],
            production_settings=production_settings,
        )
        grid_results.append(result)
        grid_skipped += int(skipped)
        print(f"GRID {index}/{len(grid_specs)} V={spec.speed_mps:g}: {'CACHED' if skipped else result['status']}")

    for index, spec in enumerate(trim_specs, 1):
        result, skipped = _trim_case(
            spec, runner=context["runner"], config=config, reference=context["reference"],
            cases_root=cases_root, signature_context=context["signature_context"],
            control_names=context["control_names"],
            production_settings=production_settings,
        )
        trim_results.append(result)
        trim_skipped += int(skipped)
        print(f"TRIM {index}/{len(trim_specs)} V={spec.speed_mps:g}: {'CACHED' if skipped else result['status']}")
        if result.get("derivatives", {}).get("required_derivatives_manifest", {}).get("items"):
            counts = result["validation"]["required_derivatives"]
            print(f"Derivative calculation: {counts['calculated']}/{counts['required']}")
            print(f"Convergence check: {result['validation']['numerical_status']}")

    nominal = next((item for item in grid_results if item.get("status") == "PASS"), None)
    if nominal is None:
        nominal = next((item for item in trim_results if item.get("status") in {"PASS", "WARN"}), None)
    if nominal is None:
        fuselage_validation = {"status": "FAIL", "error": "No successful point is available"}
        fuselage_skipped = False
    else:
        fuselage_validation, fuselage_skipped = _fuselage_validation(
            nominal=nominal, runner=context["runner"], validation_root=results_root / "validation",
            signature_context=context["signature_context"], config=config,
            production_settings=production_settings,
        )
    print(f"Fuselage participation: {fuselage_validation['status']}")

    portability = scan_portability(context["project"])
    trim_dataset = summarize_dataset(trim_results, config["_manifest"]) if trim_results else {
        "required_derivatives": len(config["_manifest"]["_required"]),
        "flight_points": 0, "required_instances": 0, "calculated": 0,
        "missing": 0, "invalid": 0, "validation_failed": 0,
        "PASS": 0, "WARN_NUMERICAL": 0, "METHOD_LIMITATION": 0, "FAIL": 0,
        "derivative_set_status": "NOT_REQUESTED", "overall_status": "PASS",
    }
    grid_failed = [item for item in grid_results if item.get("status") != "PASS"]
    trim_failed = [item for item in trim_results if item.get("status") == "FAIL"]
    grid_status = "FAIL" if grid_failed else "PASS"
    trim_status = trim_dataset["overall_status"] if trim_results else "NOT_REQUESTED"
    final_parts = [fuselage_validation["status"], "PASS" if portability["portable"] else "FAIL"]
    if grid_results:
        final_parts.append(grid_status)
    if trim_results:
        final_parts.append(trim_status)
    if gate_status != "NOT_REQUESTED":
        final_parts.append(gate_status)
    final_status = combine_status(final_parts)
    validation_rows = [row for item in trim_results for row in item.get("validation_rows", [])]
    validation_rows.append({
        "speed_mps": "", "level": "DATASET", "check": "fuselage participation",
        "status": fuselage_validation["status"],
        "value": fuselage_validation.get("max_absolute_change", ""),
        "limit": fuselage_validation.get("minimum_absolute_change", ""),
        "message": fuselage_validation.get("error", "thin+thick differs from thin-only"),
    })
    validation_rows.append({
        "speed_mps": "", "level": "DATASET", "check": "portable paths",
        "status": "PASS" if portability["portable"] else "FAIL",
        "value": portability["hard_coded_paths_found"], "limit": 0,
        "message": "no scattered absolute paths outside config/openvsp.yaml",
    })
    if gate_status != "NOT_REQUESTED":
        validation_rows.append({
            "speed_mps": "", "level": "DATASET", "check": "production numerical gate",
            "status": gate_status, "value": gate_status, "limit": "PASS/WARN",
            "message": "Wake, boundary, FD-step, and required-derivative policy gate; mesh excluded",
        })

    summary = {
        "command": command,
        "openvsp_version": context["version"],
        "geometry_status": "PASS",
        "grid": {
            "requested": len(grid_specs), "completed": len(grid_specs) - len(grid_failed),
            "failed": len(grid_failed), "skipped": grid_skipped,
            "status": grid_status if grid_specs else "NOT_REQUESTED",
        },
        "trim": {
            "requested": len(trim_specs), "completed": len(trim_specs) - len(trim_failed),
            "failed": len(trim_failed), "skipped": trim_skipped,
            "status": trim_status,
        },
        "derivatives": trim_dataset,
        "fuselage_effect_status": fuselage_validation["status"],
        "fuselage_validation_skipped": fuselage_skipped,
        "portability_status": "PASS" if portability["portable"] else "FAIL",
        "production_gate_status": gate_status,
        "final_status": final_status,
        "limitations": [
            "OpenVSP/VSPAERO 3.51.3 cannot form true centered steady-rate derivatives; p/q/r items are METHOD_LIMITATION and native values remain diagnostic-only.",
            "Adaptive GRID exposes the mode and midpoint-error evaluator but does not yet perform automatic refinement.",
        ],
    }
    metadata = {
        "schema_version": INTERNAL_SCHEMA_VERSION,
        "autotune_schema_version": AUTOTUNE_SCHEMA_VERSION,
        "tool_version": TOOL_VERSION,
        "generated_at_local": _now(),
        "openvsp_version": context["version"],
        "openvsp_root_source": context["location"].source,
        "python_version": platform.python_version(),
        "aircraft_name": config["aircraft"].get("name", config["_paths"]["model"].stem),
        "model_file": config["_paths"]["model"].relative_to(context["project"]).as_posix(),
        "model_sha256": context["model_sha256"],
        "solver": "VSPAERO mixed ThinGeomSet + GeomSet",
        "coordinate_system": COORDINATE_CONVENTION,
        "production_numerical_settings": production_settings,
    }
    geometry_data = {
        "thin_set_index": context["geometry"].thin_set_index,
        "thick_set_index": context["geometry"].thick_set_index,
        "thin": list(context["geometry"].thin),
        "thick": list(context["geometry"].thick),
        "controls": {
            role: {"group": group.name, "surfaces": list(group.surfaces), "gains": list(group.gains)}
            for role, group in context["control_map"].items()
        },
    }
    database = build_database(
        metadata=metadata,
        reference={"source": config["reference"]["source"], **context["reference"]},
        geometry=geometry_data,
        manifest=_json_safe(config["_manifest"]),
        grid_results=grid_results,
        trim_results=trim_results,
        validation={
            "rows": validation_rows, "fuselage_effect": fuselage_validation,
            "portability": portability, "dataset_status": final_status,
        },
        summary=summary,
    )
    summary_text = _summary_text(summary, results_root)
    paths = export_database(database, results_root, config["export"], config["validation"], summary_text)
    print(f"Validation: {final_status}")
    print(f"MAT export: {paths.get('mat_status', 'NOT_REQUESTED')}")
    print(summary_text)
    return {
        "database": database, "paths": paths, "summary": summary,
        "success": final_status != "FAIL",
    }


def _run_regression(config: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    regression = config["regression"]
    speed = float(regression["speed_mps"])
    beta = float(regression.get("beta_deg", 0.0))
    spec = CaseSpec(trim_case_id(speed, beta), "TRIM_DATABASE", speed, None, beta)
    result, skipped = _trim_case(
        spec, runner=context["runner"], config=config, reference=context["reference"],
        cases_root=config["_paths"]["results"] / "cases",
        signature_context=context["signature_context"], control_names=context["control_names"],
        production_settings=None,
    )
    report = compare_regression(
        current=result,
        baseline_path=config["_paths"]["regression_baseline"],
        settings=regression,
        output_dir=config["_paths"]["results"] / "regression",
        current_model_sha256=context["model_sha256"],
        current_openvsp_version=context["version"],
    )
    print(f"Regression V={speed:g} m/s: {report['status']} ({'CACHED' if skipped else 'CALCULATED'})")
    return {"success": report["status"] == "PASS", "summary": {"final_status": report["status"]}, "report": report}


def main() -> int:
    parser = argparse.ArgumentParser(description="OpenVSP/VSPAERO aerodynamic derivative dataset generator")
    parser.add_argument(
        "command", nargs="?", default="all",
        choices=("all", "grid", "trim", "numerical-convergence", "smoke", "regression", "check"),
    )
    parser.add_argument("--config", help="Path to the unified aircraft YAML configuration")
    parser.add_argument(
        "--force", action="store_true",
        help="Run GRID/TRIM despite a FAIL Production Gate and write an audit record",
    )
    parser.add_argument("--debug", action="store_true", help="Show a traceback on a fatal workflow error")
    args = parser.parse_args()
    try:
        result = run_workflow(args.command, args.config, force=args.force)
        return 0 if result["success"] else 1
    except (ConfigError, OpenVSPError, ValidationError, RuntimeError, OSError, ValueError) as exc:
        print("RUN FAILED")
        print(f"Reason: {exc}")
        print("Suggestion: run 'python run.py check' and inspect the failing case under results/cases.")
        if args.debug:
            traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
