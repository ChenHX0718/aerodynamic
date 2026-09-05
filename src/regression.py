from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

from finite_difference import combine_status


def _current_values(result: dict[str, Any]) -> dict[str, float]:
    trim = result.get("trim", {})
    values = {
        "alpha_trim_deg": float(trim["alpha_trim_deg"]),
        "elevator_trim_deg": float(trim["elevator_trim_deg"]),
    }
    production = result.get("derivatives", {}).get("production_derivatives", {})
    for name, record in production.items():
        value = record.get("value")
        if value is not None:
            values[str(name)] = float(value)
    return values


def compare_regression(
    *, current: dict[str, Any], baseline_path: Path,
    settings: dict[str, Any], output_dir: Path,
    current_model_sha256: str, current_openvsp_version: str,
) -> dict[str, Any]:
    if not baseline_path.is_file():
        raise RuntimeError(
            f"Regression baseline is missing: {baseline_path}. "
            "Create it only from a reviewed PASS/WARN trim dataset."
        )
    baseline = json.loads(baseline_path.read_text(encoding="utf-8"))
    baseline_values = baseline.get("values")
    if not isinstance(baseline_values, dict) or not baseline_values:
        raise RuntimeError(f"Regression baseline contains no values: {baseline_path}")
    rows: list[dict[str, Any]] = []
    for quantity, baseline_value, current_value in (
        ("model_sha256", baseline.get("model_sha256"), current_model_sha256),
        ("openvsp_version", baseline.get("openvsp_version"), current_openvsp_version),
    ):
        matched = bool(baseline_value) and str(baseline_value) == str(current_value)
        rows.append({
            "quantity": quantity, "baseline": baseline_value or "",
            "current": current_value, "absolute_difference": "",
            "relative_difference": "", "pass_limit": "exact match",
            "warn_limit": "exact match", "status": "PASS" if matched else "FAIL",
            "message": "fixed regression identity" if matched else "baseline identity mismatch",
        })
    current_values = _current_values(current) if current.get("status") in {"PASS", "WARN"} else {}
    for name, baseline_value_raw in baseline_values.items():
        baseline_value = float(baseline_value_raw)
        if name not in current_values or not math.isfinite(float(current_values[name])):
            rows.append({
                "quantity": name, "baseline": baseline_value, "current": "",
                "absolute_difference": "", "relative_difference": "",
                "pass_limit": "", "warn_limit": "", "status": "FAIL",
                "message": "current value is missing or non-finite",
            })
            continue
        value = float(current_values[name])
        difference = abs(value - baseline_value)
        relative = difference / max(abs(baseline_value), 1.0e-9)
        pass_limit = float(settings["pass_absolute"]) + float(settings["pass_relative"]) * abs(baseline_value)
        warn_limit = float(settings["warn_absolute"]) + float(settings["warn_relative"]) * abs(baseline_value)
        if difference <= pass_limit:
            status = "PASS"
        elif difference <= warn_limit:
            status = "WARN"
        else:
            status = "FAIL"
        rows.append({
            "quantity": name, "baseline": baseline_value, "current": value,
            "absolute_difference": difference, "relative_difference": relative,
            "relative_difference_pct": relative * 100.0,
            "pass_limit": pass_limit, "warn_limit": warn_limit,
            "status": status, "message": "",
        })
    status = combine_status([row["status"] for row in rows]) if rows else "FAIL"
    if current.get("status") == "FAIL":
        status = "FAIL"
    report = {
        "schema_version": "1.0",
        "status": status,
        "baseline_file": str(baseline_path),
        "baseline_source": baseline.get("source", "unspecified"),
        "model_sha256": current_model_sha256,
        "openvsp_version": current_openvsp_version,
        "current_case_id": current.get("case_id"),
        "current_dataset_status": current.get("status"),
        "rows": rows,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "regression_report.csv"
    fieldnames = sorted({key for row in rows for key in row})
    with csv_path.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "regression_summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8"
    )
    return report
