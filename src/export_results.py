from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
from scipy.io import loadmat, savemat


def _standard_values(items: dict[str, dict[str, Any]]) -> dict[str, float]:
    return {name: float(item["standard_value"]) for name, item in items.items()}


def flatten_case(result: dict[str, Any]) -> dict[str, Any]:
    inputs = result.get("inputs", {})
    row: dict[str, Any] = {
        "case_id": result.get("case_id", ""),
        "mode": result.get("mode", ""),
        "status": result.get("status", ""),
        "error": result.get("error", ""),
    }
    for name in (
        "speed_mps", "alpha_deg", "beta_deg", "mach", "reynolds_cref",
        "rho_kg_m3", "dynamic_viscosity_pa_s", "dynamic_pressure_pa",
    ):
        row[name] = inputs.get(name)
    outputs = result.get("outputs", {})
    if outputs.get("coefficients"):
        row.update(_standard_values(outputs["coefficients"]))
    diagnostics = outputs.get("native_derivative_diagnostics", {})
    native_values = _standard_values(diagnostics.get("stability", {}))
    for control in diagnostics.get("controls", {}).values():
        native_values.update(_standard_values(control.get("derivatives", {})))
    row.update({f"native_diagnostic_{name}": value for name, value in native_values.items()})
    if result.get("mode") == "TRIM_DATABASE":
        trim = result.get("trim", {})
        for name in (
            "alpha_trim_deg", "beta_trim_deg", "elevator_trim_deg",
            "CL_trim", "CD_trim", "CY_trim", "Cl_trim", "Cm_trim", "Cn_trim",
            "trim_force_residual_n", "trim_moment_residual_nm", "trim_iterations",
        ):
            row[name] = trim.get(name)
        derivative_package = result.get("derivatives", {})
        for name, record in derivative_package.get("production_fd_derivatives", {}).items():
            row[f"production_fd_{name}"] = record.get("derivative_value")
        for name, record in derivative_package.get("native_derivative_diagnostics", {}).items():
            row[f"native_diagnostic_{name}"] = record.get("diagnostic_value")
        validation = result.get("validation", {})
        for name in (
            "overall_status", "trim_status", "numerical_status", "derivative_status", "physics_status",
        ):
            row[name] = validation.get(name)
    row["solver_duration_sec"] = result.get("solver", {}).get("duration_sec")
    return row


def _derivative_rows(
    trim_results: list[dict[str, Any]], coordinate_system: dict[str, Any]
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for result in trim_results:
        speed = result.get("inputs", {}).get("speed_mps")
        derivative_package = result.get("derivatives", {})
        production = derivative_package.get("production_fd_derivatives", {})
        native = derivative_package.get("native_derivative_diagnostics", {})
        items = derivative_package.get("required_derivatives_manifest", {}).get("items", [])
        for item in items:
            name = str(item["name"])
            record = production.get(name, {})
            diagnostic = native.get(name, {})
            samples = record.get("samples", {})
            row = {
                "case_id": result.get("case_id"),
                "speed_mps": speed,
                "name": name,
                "category": item.get("category"),
                "coefficient": item.get("coefficient"),
                "perturbation": item.get("perturbation"),
                "required": item.get("required"),
                "derivative_value": item.get("value"),
                "source": item.get("source"),
                "units": item.get("units"),
                "definition": item.get("definition"),
                "method": item.get("method"),
                "selected_fd_step": item.get("selected_fd_step"),
                "selected_fd_step_unit": item.get("selected_fd_step_unit"),
                "convergence_status": record.get("convergence_status"),
                "validation_status": item.get("validation_status"),
                "wake_level": item.get("wake_level"),
                "production_included": item.get("production_included"),
                "gate_action": item.get("gate_action"),
                "minus_sample_available": all(
                    sample.get("minus") is not None for sample in samples.values()
                ) if samples else False,
                "native_diagnostic_value": diagnostic.get("diagnostic_value"),
                "reason": item.get("reason"),
                "coordinate_sign_convention": item.get("coordinate_sign_convention"),
            }
            rows.append(row)
    return rows


def build_database(
    *, metadata: dict[str, Any], reference: dict[str, Any], geometry: dict[str, Any],
    manifest: dict[str, Any], grid_results: list[dict[str, Any]],
    trim_results: list[dict[str, Any]], validation: dict[str, Any],
    summary: dict[str, Any],
) -> dict[str, Any]:
    return {
        "metadata": metadata,
        "reference": reference,
        "geometry": geometry,
        "manifest": manifest,
        "units": {
            "state_angles": "deg",
            "angle_and_control_derivatives": "1/rad",
            "p_derivatives": "per p_hat; p_hat=p*bref/(2*V)",
            "q_derivatives": "per q_hat; q_hat=q*cref/(2*V)",
            "r_derivatives": "per r_hat; r_hat=r*bref/(2*V)",
            "forces": "N", "moments": "N*m", "speed": "m/s",
        },
        "grid": {"results": grid_results, "flat_table": [flatten_case(item) for item in grid_results]},
        "trim": {"results": trim_results, "flat_table": [flatten_case(item) for item in trim_results]},
        "derivative_table": _derivative_rows(trim_results, metadata["coordinate_system"]),
        "validation": validation,
        "summary": summary,
    }


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        if not fieldnames:
            return
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


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


def _row_array(values: list[Any], *, numeric: bool = True) -> np.ndarray:
    if numeric:
        return np.asarray([[float(value) for value in values]], dtype=float)
    return np.asarray([[str(value) for value in values]], dtype=object)


def _aero_mat(database: dict[str, Any], accepted: list[dict[str, Any]]) -> dict[str, Any]:
    accepted = sorted(
        accepted,
        key=lambda item: (
            float(item["inputs"]["speed_mps"]), float(item["inputs"].get("beta_deg", 0.0))
        ),
    )
    manifest_rows = list(database["manifest"]["derivatives"])
    derivative_meta = {
        str(row["name"]): {
            "category": str(row["category"]),
            "unit": str(row["unit"]),
            "perturbation": str(row["perturbation"]),
            "definition": str(row["definition"]),
        }
        for row in manifest_rows
    }

    def derivative_arrays(categories: set[str]) -> dict[str, np.ndarray]:
        return {
            str(row["name"]): _row_array([
                item["derivatives"]["production_fd_derivatives"][str(row["name"])]["derivative_value"]
                for item in accepted
            ])
            for row in manifest_rows
            if str(row["category"]) in categories
            and all(
                str(row["name"]) in item["derivatives"]["production_fd_derivatives"]
                for item in accepted
            )
        }

    native_diagnostics = {
        str(row["name"]): _row_array([
            item["derivatives"]["native_derivative_diagnostics"][str(row["name"])].get(
                "diagnostic_value", math.nan
            )
            if item["derivatives"]["native_derivative_diagnostics"][str(row["name"])].get(
                "diagnostic_value"
            ) is not None else math.nan
            for item in accepted
        ])
        for row in manifest_rows
    }

    flight_points = {
        "V_mps": _row_array([item["inputs"]["speed_mps"] for item in accepted]),
        "rho_kg_m3": _row_array([item["inputs"]["rho_kg_m3"] for item in accepted]),
        "qbar_pa": _row_array([item["inputs"]["dynamic_pressure_pa"] for item in accepted]),
        "Mach": _row_array([item["inputs"]["mach"] for item in accepted]),
        "Reynolds_cref": _row_array([item["inputs"]["reynolds_cref"] for item in accepted]),
        "alpha_trim_deg": _row_array([item["trim"]["alpha_trim_deg"] for item in accepted]),
        "beta_trim_deg": _row_array([item["trim"]["beta_trim_deg"] for item in accepted]),
        "elevator_trim_deg": _row_array([item["trim"]["elevator_trim_deg"] for item in accepted]),
    }
    trim = {
        name: _row_array([item["trim"][name] for item in accepted])
        for name in (
            "CL_trim", "CD_trim", "CY_trim", "Cl_trim", "Cm_trim", "Cn_trim",
            "trim_force_residual_n", "trim_moment_residual_nm", "trim_iterations",
        )
    }
    validation = {
        "overall_status": _row_array([item["validation"]["overall_status"] for item in accepted], numeric=False),
        "trim_status": _row_array([item["validation"]["trim_status"] for item in accepted], numeric=False),
        "convergence_status": _row_array([item["validation"]["numerical_status"] for item in accepted], numeric=False),
        "derivative_status": _row_array([item["validation"]["derivative_status"] for item in accepted], numeric=False),
        "physics_status": _row_array([item["validation"]["physics_status"] for item in accepted], numeric=False),
        "accepted": _row_array([1 for _ in accepted]),
        "required_derivative_status": {
            str(row["name"]): _row_array([
                next(
                    item for item in result["derivatives"]["required_derivatives_manifest"]["items"]
                    if item["name"] == str(row["name"])
                )["validation_status"]
                for result in accepted
            ], numeric=False)
            for row in manifest_rows
        },
        "rate_derivative_limitation": (
            "p/q/r centered steady-rate finite differences are unavailable in OpenVSP/VSPAERO 3.51.3; "
            "native values are diagnostic-only and excluded from production derivative arrays"
        ),
    }
    return {
        "meta": {
            "schema_version": "1.0",
            "aircraft_name": database["metadata"]["aircraft_name"],
            "creation_time": database["metadata"]["generated_at_local"],
            "openvsp_version": database["metadata"]["openvsp_version"],
            "solver": database["metadata"]["solver"],
            "geometry_sha256": database["metadata"]["model_sha256"],
            "coordinate_system": database["metadata"]["coordinate_system"],
            "derivatives": derivative_meta,
            "production_numerical_settings": {
                "gate_status": (
                    database["metadata"].get("production_numerical_settings") or {}
                ).get("production_gate", {}).get("status", "NOT_REQUESTED"),
                "mesh_source": "current_openvsp_model",
                "mesh_numerically_certified": False,
                "wake_rule": "discrete state schedule plus derivative-bundle maximum",
            },
        },
        "reference": database["reference"],
        "flight_points": flight_points,
        "trim": trim,
        "longitudinal": derivative_arrays({"longitudinal"}),
        "lateral": derivative_arrays({"lateral"}),
        "controls": {
            "aileron": derivative_arrays({"aileron"}),
            "elevator": derivative_arrays({"elevator"}),
            "rudder": derivative_arrays({"rudder"}),
        },
        "native_derivative_diagnostics": native_diagnostics,
        "validation": validation,
    }


def export_database(
    database: dict[str, Any], results_root: Path, export_config: dict[str, Any],
    validation_config: dict[str, Any], summary_text: str,
) -> dict[str, Any]:
    latest_dir = results_root / "latest"
    validation_dir = results_root / "validation"
    autotune_dir = results_root / "autotune"
    latest_dir.mkdir(parents=True, exist_ok=True)
    validation_dir.mkdir(parents=True, exist_ok=True)
    paths: dict[str, Any] = {}

    all_rows = database["grid"]["flat_table"] + database["trim"]["flat_table"]
    if export_config.get("csv", True):
        _write_csv(latest_dir / "aero_database.csv", all_rows)
        _write_csv(latest_dir / "trim_derivatives.csv", database["derivative_table"])
        _write_csv(validation_dir / "validation_report.csv", database["validation"]["rows"])
        fuselage = database["validation"].get("fuselage_effect", {})
        _write_csv(validation_dir / "fuselage_effect_validation.csv", [
            {
                "coefficient": name,
                "thin_only": fuselage.get("thin_only", {}).get(name),
                "thin_plus_thick": fuselage.get("thin_plus_thick", {}).get(name),
                "delta": delta,
                "status": fuselage.get("status"),
            }
            for name, delta in fuselage.get("delta", {}).items()
        ])
        paths["csv"] = latest_dir / "aero_database.csv"
        paths["derivative_csv"] = latest_dir / "trim_derivatives.csv"
        paths["validation_csv"] = validation_dir / "validation_report.csv"

    if export_config.get("json", True):
        json_path = latest_dir / "aero_database.json"
        json_path.write_text(
            json.dumps(_json_safe(database), ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        validation_summary = {
            "dataset_status": database["validation"]["dataset_status"],
            "summary": database["summary"],
            "flight_points": [
                {
                    "speed_mps": item.get("inputs", {}).get("speed_mps"),
                    **item.get("validation", {}),
                }
                for item in database["trim"]["results"]
            ],
        }
        (validation_dir / "validation_summary.json").write_text(
            json.dumps(_json_safe(validation_summary), ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        paths["json"] = json_path
        paths["validation_json"] = validation_dir / "validation_summary.json"

    command = database["summary"].get("command")
    allow_warn = bool(validation_config.get("autotune_allow_warn", False))
    accepted_statuses = {"PASS", "WARN"} if allow_warn else {"PASS"}
    accepted = [
        item for item in database["trim"]["results"]
        if item.get("validation", {}).get("overall_status") in accepted_statuses
    ]
    mat_path = autotune_dir / "aircraft_aero.mat"
    if export_config.get("mat", True) and command in {"all", "trim"}:
        if accepted and len(accepted) == len(database["trim"]["results"]):
            autotune_dir.mkdir(parents=True, exist_ok=True)
            savemat(
                mat_path, {"AERO": _aero_mat(database, accepted)},
                long_field_names=True, do_compression=True,
            )
            loaded = loadmat(mat_path, squeeze_me=True, struct_as_record=False)
            aero = loaded.get("AERO")
            if aero is None or not hasattr(aero, "meta") or not hasattr(aero, "flight_points"):
                raise RuntimeError("MAT verification failed: AERO.meta/flight_points is unreadable")
            if not hasattr(aero.meta, "schema_version") or str(aero.meta.schema_version) != "1.0":
                raise RuntimeError("MAT verification failed: AERO.meta.schema_version is not 1.0")
            if not hasattr(aero, "longitudinal") or not hasattr(aero.longitudinal, "Cm_alpha"):
                raise RuntimeError("MAT verification failed: AERO.longitudinal.Cm_alpha is unreadable")
            paths["mat"] = mat_path
            paths["mat_status"] = "PASS"
        else:
            if mat_path.is_file():
                mat_path.unlink()
            paths["mat_status"] = "FAIL (no complete accepted TRIM dataset)"
    else:
        paths["mat_status"] = "NOT_REQUESTED"

    summary_path = latest_dir / "run_summary.txt"
    summary_path.write_text(summary_text, encoding="utf-8")
    paths["summary"] = summary_path
    return paths
