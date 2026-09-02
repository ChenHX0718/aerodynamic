from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

from finite_difference import combine_status


class ValidationError(RuntimeError):
    """Raised when solver output fails a required acceptance check."""


def require_finite(name: str, value: Any) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{name} is not numeric: {value!r}") from exc
    if not math.isfinite(numeric):
        raise ValidationError(f"{name} is not finite: {numeric}")
    return numeric


def calculate_condition(
    *, speed_mps: float, alpha_deg: float, beta_deg: float,
    atmosphere: dict[str, Any], cref_m: float,
) -> dict[str, float]:
    rho = require_finite("atmosphere.rho_kg_m3", atmosphere["rho_kg_m3"])
    viscosity = require_finite("atmosphere.dynamic_viscosity_pa_s", atmosphere["dynamic_viscosity_pa_s"])
    speed_of_sound = require_finite("atmosphere.speed_of_sound_mps", atmosphere["speed_of_sound_mps"])
    speed = require_finite("speed_mps", speed_mps)
    cref = require_finite("cref_m", cref_m)
    if min(rho, viscosity, speed_of_sound, speed, cref) <= 0:
        raise ValidationError("Atmosphere, speed, and reference chord values must be positive")
    return {
        "speed_mps": speed,
        "alpha_deg": require_finite("alpha_deg", alpha_deg),
        "beta_deg": require_finite("beta_deg", beta_deg),
        "rho_kg_m3": rho,
        "dynamic_viscosity_pa_s": viscosity,
        "speed_of_sound_mps": speed_of_sound,
        "mach": speed / speed_of_sound,
        "reynolds_cref": rho * speed * cref / viscosity,
        "dynamic_pressure_pa": 0.5 * rho * speed * speed,
    }


def validate_analysis_payload(payload: dict[str, Any]) -> list[str]:
    warnings: list[str] = []
    coefficients = payload.get("coefficients", {})
    required_coefficients = ("CX", "CY", "CZ", "CL", "CD", "Cl", "Cm", "Cn")
    missing = [name for name in required_coefficients if name not in coefficients]
    if missing:
        raise ValidationError(f"Coefficient(s) missing: {', '.join(missing)}")
    for name, item in coefficients.items():
        require_finite(f"coefficient.{name}", item["standard_value"])
    stability = payload.get("stability_derivatives", {})
    for name, item in stability.items():
        require_finite(f"stability.{name}", item["standard_value"])
    controls = payload.get("control_derivatives", {})
    for role in ("aileron", "elevator", "rudder"):
        derivatives = controls.get(role, {}).get("derivatives", {})
        if not derivatives:
            raise ValidationError(f"{role} control derivatives are missing")
        for name, item in derivatives.items():
            require_finite(f"control.{role}.{name}", item["standard_value"])
    if coefficients["CD"]["standard_value"] < 0:
        warnings.append("CD is negative; inspect mesh, axes, and convergence")
    return warnings


def _row(
    *, speed: float, level: str, check: str, status: str,
    value: Any = "", limit: Any = "", message: str = "",
) -> dict[str, Any]:
    return {
        "speed_mps": float(speed), "level": level, "check": check,
        "status": status, "value": value, "limit": limit, "message": message,
    }


def _sample_integrity(record: dict[str, Any]) -> tuple[str, str]:
    samples = record.get("samples")
    if not isinstance(samples, dict) or any(key not in samples for key in ("0.5", "1", "2")):
        return "FAIL", "0.5/1/2 finite-difference samples are not complete"
    method = str(record.get("method", ""))
    for key in ("0.5", "1", "2"):
        sample = samples[key]
        if not isinstance(sample, dict) or not isinstance(sample.get("plus"), dict):
            return "FAIL", f"scale {key} has no real plus sample"
        try:
            valid_numbers = (
                math.isfinite(float(sample["derivative"]))
                and math.isfinite(float(sample["derivative_denominator"]))
                and float(sample["derivative_denominator"]) > 0
            )
        except (KeyError, TypeError, ValueError):
            valid_numbers = False
        if not valid_numbers:
            return "FAIL", f"scale {key} has an invalid derivative or denominator"
        if method == "centered_finite_difference" and not isinstance(sample.get("minus"), dict):
            return "FAIL", f"centered scale {key} has no real minus sample"
        if method == "vspaero_native_forward_rate" and not isinstance(sample.get("scale_base"), dict):
            return "FAIL", f"native rate scale {key} has no matching solver baseline"
    if method == "centered_finite_difference":
        return "PASS", "all three scales contain real plus and minus samples"
    if method == "vspaero_native_forward_rate":
        status = str(record.get("method_status", "WARN")).upper()
        if status not in {"WARN", "FAIL"}:
            status = "FAIL"
        return status, "three real positive-rate samples and baselines exist; negative rate is unavailable"
    return "FAIL", f"unsupported derivative method: {method or 'missing'}"


def validate_trim_result(
    result: dict[str, Any], manifest: dict[str, Any], validation_config: dict[str, Any]
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    speed = float(result.get("inputs", {}).get("speed_mps", math.nan))
    rows: list[dict[str, Any]] = []

    solver_ok = result.get("solver", {}).get("status") == "SUCCESS"
    solver_status = "PASS" if solver_ok else "FAIL"
    rows.append(_row(
        speed=speed, level="SOLVER", check="VSPAERO completion", status=solver_status,
        message="all requested solver runs completed" if solver_ok else str(result.get("error", "solver failed")),
    ))

    trim = result.get("trim", {})
    converged = bool(trim.get("converged"))
    force = trim.get("trim_force_residual_n")
    moment = trim.get("trim_moment_residual_nm")
    force_limit = trim.get("force_tolerance_n")
    moment_limit = trim.get("moment_tolerance_nm")
    residuals_finite = False
    if converged:
        residuals_finite = all(
            math.isfinite(float(value)) for value in (force, moment, force_limit, moment_limit)
        )
    force_ok = residuals_finite and abs(float(force)) <= float(force_limit)
    moment_ok = residuals_finite and abs(float(moment)) <= float(moment_limit)
    try:
        alpha = float(trim["alpha_trim_deg"])
        elevator = float(trim["elevator_trim_deg"])
        alpha_range = (float(trim["alpha_min_deg"]), float(trim["alpha_max_deg"]))
        elevator_range = (float(trim["elevator_min_deg"]), float(trim["elevator_max_deg"]))
        alpha_ok = math.isfinite(alpha) and alpha_range[0] <= alpha <= alpha_range[1]
        elevator_ok = math.isfinite(elevator) and elevator_range[0] <= elevator <= elevator_range[1]
    except (KeyError, TypeError, ValueError):
        alpha, elevator = math.nan, math.nan
        alpha_range, elevator_range = (math.nan, math.nan), (math.nan, math.nan)
        alpha_ok = elevator_ok = False
    trim_status = "PASS" if all((converged, force_ok, moment_ok, alpha_ok, elevator_ok)) else "FAIL"
    rows.extend([
        _row(speed=speed, level="TRIM", check="converged", status="PASS" if converged else "FAIL",
             message=str(trim.get("failure_reason") or "Lift=Weight and Cm=0 residuals converged")),
        _row(speed=speed, level="TRIM", check="lift residual", status="PASS" if force_ok else "FAIL",
             value=force, limit=force_limit, message="absolute force residual in N"),
        _row(speed=speed, level="TRIM", check="pitch moment residual", status="PASS" if moment_ok else "FAIL",
             value=moment, limit=moment_limit, message="absolute pitch moment residual in N*m"),
        _row(speed=speed, level="TRIM", check="alpha range", status="PASS" if alpha_ok else "FAIL",
             value=alpha, limit=f"{alpha_range[0]:g}..{alpha_range[1]:g} deg",
             message="trim alpha is inside the configured search range"),
        _row(speed=speed, level="TRIM", check="elevator range", status="PASS" if elevator_ok else "FAIL",
             value=elevator, limit=f"{elevator_range[0]:g}..{elevator_range[1]:g} deg",
             message="trim elevator is inside the configured search range"),
    ])

    records = result.get("derivatives", {}).get("records", {})
    required_rows = list(manifest["_required"])
    missing: list[str] = []
    invalid: list[str] = []
    failed: list[str] = []
    warned: list[str] = []
    for definition in required_rows:
        name = str(definition["name"])
        record = records.get(name)
        if not isinstance(record, dict):
            missing.append(name)
            continue
        value = record.get("value")
        try:
            finite = math.isfinite(float(value))
        except (TypeError, ValueError):
            finite = False
        if not finite:
            invalid.append(name)
        status = str(record.get("validation_status", "FAIL")).upper()
        integrity_status, integrity_message = _sample_integrity(record)
        status = combine_status([status, integrity_status, "PASS" if finite else "FAIL"])
        record["validation_status"] = status
        if status == "FAIL":
            failed.append(name)
        elif status == "WARN":
            warned.append(name)
        convergence = record.get("convergence", {})
        rows.append(_row(
            speed=speed, level="NUMERICAL", check=name,
            status=status, value=value,
            limit=f"PASS {convergence.get('pass_limit', '')}; WARN {convergence.get('warn_limit', '')}",
            message=(
                f"{record.get('method')}: {convergence.get('reason', '')}; "
                f"sample integrity: {integrity_message}"
            ),
        ))
    calculated = len(required_rows) - len(missing) - len(invalid)
    if missing or invalid or failed:
        derivative_status = "FAIL"
    elif warned:
        derivative_status = "WARN"
    else:
        derivative_status = "PASS"
    manifest_summary = {
        "required": len(required_rows),
        "calculated": calculated,
        "missing": len(missing),
        "invalid": len(invalid),
        "validation_failed": len(failed),
        "validation_warned": len(warned),
        "missing_names": missing,
        "invalid_names": invalid,
        "failed_names": failed,
        "warned_names": warned,
        "status": derivative_status,
    }
    rows.append(_row(
        speed=speed, level="DERIVATIVE", check="required derivative set", status=derivative_status,
        value=f"{calculated}/{len(required_rows)}",
        limit="no missing, invalid, or failed required derivative",
        message=f"missing={len(missing)}, invalid={len(invalid)}, failed={len(failed)}, warned={len(warned)}",
    ))

    numerical_status = combine_status([
        str(record.get("validation_status", "FAIL")) for record in records.values()
    ]) if records else "FAIL"

    physics_status, physics_rows = _physics_checks(
        speed=speed, result=result, records=records,
        manifest_rows=required_rows, config=validation_config,
    )
    rows.extend(physics_rows)
    overall = combine_status([
        solver_status, trim_status, numerical_status, derivative_status, physics_status,
    ])
    levels = {
        "solver_status": solver_status,
        "trim_status": trim_status,
        "numerical_status": numerical_status,
        "derivative_status": derivative_status,
        "physics_status": physics_status,
        "overall_status": overall,
    }
    rows.append(_row(
        speed=speed, level="DATASET", check="trim flight point overall", status=overall,
        message="worst status across solver, trim, numerical, derivative, and physics validation",
    ))
    return {**levels, "required_derivatives": manifest_summary}, rows


def _physics_checks(
    *, speed: float, result: dict[str, Any], records: dict[str, Any],
    manifest_rows: list[dict[str, Any]], config: dict[str, Any],
) -> tuple[str, list[dict[str, Any]]]:
    if not bool(config.get("physics", {}).get("enabled", True)):
        return "PASS", [_row(speed=speed, level="PHYSICS", check="physics checks", status="PASS", message="disabled by config")]
    rows: list[dict[str, Any]] = []
    statuses: list[str] = []
    limit = float(config.get("physics", {}).get("derivative_absolute_limit", 100.0))
    conventional = bool(config.get("conventional_fixed_wing", True))
    for definition in manifest_rows:
        name = str(definition["name"])
        record = records.get(name)
        if not record:
            continue
        value = float(record["value"])
        status = "PASS"
        message = "finite and below configured magnitude limit"
        if abs(value) > limit:
            status = "FAIL"
            message = "derivative magnitude exceeds configured physical sanity limit"
        expected = definition.get("expected_sign") if conventional else None
        if expected == "positive" and value <= 0:
            status = "FAIL"
            message = "expected positive for configured conventional fixed-wing checks"
        elif expected == "negative" and value >= 0:
            status = "FAIL"
            message = "expected negative for configured conventional fixed-wing checks"
        statuses.append(status)
        if expected or status != "PASS":
            rows.append(_row(
                speed=speed, level="PHYSICS", check=name, status=status,
                value=value, limit=f"expected {expected or f'abs <= {limit:g}'}", message=message,
            ))

    symmetry = config.get("symmetry", {})
    beta = float(result.get("inputs", {}).get("beta_deg", 0.0))
    if bool(symmetry.get("enabled", True)) and math.isclose(beta, 0.0, abs_tol=1.0e-9):
        coefficients = result.get("outputs", {}).get("coefficients", {})
        warn_limit = float(symmetry.get("base_coefficient_absolute_warn", 0.02))
        fail_limit = float(symmetry.get("base_coefficient_absolute_fail", 0.05))
        for name in ("CY", "Cl", "Cn"):
            value = abs(float(coefficients[name]["standard_value"]))
            status = "PASS" if value <= warn_limit else "WARN" if value <= fail_limit else "FAIL"
            statuses.append(status)
            rows.append(_row(
                speed=speed, level="PHYSICS", check=f"beta=0 symmetry {name}", status=status,
                value=value, limit=f"WARN>{warn_limit:g}; FAIL>{fail_limit:g}",
                message="absolute trim coefficient for a nominally symmetric airplane",
            ))
        base = {
            name: float(coefficients[name]["standard_value"])
            for name in ("CY", "Cl", "Cn")
        }
        for variable, record_name in (
            ("beta", "CY_beta"), ("aileron", "CY_delta_a"), ("rudder", "CY_delta_r")
        ):
            sample = records.get(record_name, {}).get("samples", {}).get("1", {})
            plus = sample.get("plus", {}).get("coefficients", {})
            minus = sample.get("minus", {}).get("coefficients", {})
            if not all(name in plus and name in minus for name in base):
                status = "FAIL"
                bias = math.nan
            else:
                bias = max(
                    abs(0.5 * (float(plus[name]) + float(minus[name])) - base[name])
                    for name in base
                )
                status = "PASS" if bias <= warn_limit else "WARN" if bias <= fail_limit else "FAIL"
            statuses.append(status)
            rows.append(_row(
                speed=speed, level="PHYSICS", check=f"{variable} centered-pair symmetry",
                status=status, value=bias, limit=f"WARN>{warn_limit:g}; FAIL>{fail_limit:g}",
                message="maximum CY/Cl/Cn midpoint bias of the nominal plus/minus pair",
            ))
    return combine_status(statuses), rows


def summarize_dataset(trim_results: list[dict[str, Any]], manifest: dict[str, Any]) -> dict[str, Any]:
    validations = [item.get("validation", {}) for item in trim_results]
    overall = combine_status([str(item.get("overall_status", "FAIL")) for item in validations]) if validations else "FAIL"
    required_per_point = len(manifest["_required"])
    required_instances = required_per_point * len(trim_results)
    calculated = sum(int(item.get("required_derivatives", {}).get("calculated", 0)) for item in validations)
    missing = sum(int(item.get("required_derivatives", {}).get("missing", 0)) for item in validations)
    invalid = sum(int(item.get("required_derivatives", {}).get("invalid", 0)) for item in validations)
    failed = sum(int(item.get("required_derivatives", {}).get("validation_failed", 0)) for item in validations)
    derivative_status = combine_status([
        str(item.get("derivative_status", "FAIL")) for item in validations
    ]) if validations else "FAIL"
    return {
        "required_derivatives": required_per_point,
        "flight_points": len(trim_results),
        "required_instances": required_instances,
        "calculated": calculated,
        "missing": missing,
        "invalid": invalid,
        "validation_failed": failed,
        "derivative_set_status": "FAIL" if missing or invalid or failed else derivative_status,
        "overall_status": overall,
    }


def scan_portability(project_root: Path) -> dict[str, Any]:
    root = project_root.resolve()
    candidates = set(root.glob("*.bat")) | set(root.glob("*.py"))
    for directory in ("src", "tests"):
        base = root / directory
        if base.is_dir():
            candidates.update(base.rglob("*.py"))
    config_dir = root / "config"
    if config_dir.is_dir():
        candidates.update(config_dir.rglob("*.yaml"))
        candidates.update(config_dir.rglob("*.yml"))
    candidates = {path for path in candidates if path.name.lower() != "openvsp.yaml"}
    drive_path = re.compile(r"(?i)(?<![A-Z0-9_])([A-Z]:[\\/][^\s\"'<>|]*)")
    unc_path = re.compile(r"(?<![:\\/])([\\/]{2}[A-Za-z0-9._-]+[\\/][^\s\"'<>|]+)")
    findings: list[dict[str, Any]] = []
    for path in sorted(candidates):
        text = path.read_text(encoding="utf-8-sig")
        for line_number, line in enumerate(text.splitlines(), start=1):
            for pattern in (drive_path, unc_path):
                for match in pattern.finditer(line):
                    findings.append({
                        "file": path.relative_to(root).as_posix(), "line": line_number,
                        "matched_path": match.group(1),
                    })
    return {
        "scanned_files": len(candidates), "hard_coded_paths_found": len(findings),
        "portable": not findings, "findings": findings,
    }
