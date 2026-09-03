from __future__ import annotations

import csv
import hashlib
import json
import math
import shutil
import struct
import time
import zlib
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Iterable

import yaml

from coordinate_system import map_polar_coefficients
from finite_difference import (
    centered_derivative,
    combine_status,
    dual_tolerance_result,
    perturb_state,
)
from trim_solver import solve_longitudinal_trim
from validation import calculate_condition


COEFFICIENTS = ("CL", "CD", "CY", "Cl", "Cm", "Cn")
PRESET_ORDER = ("COARSE", "MEDIUM", "FINE")
NUMERICAL_ALGORITHM_VERSION = "2.0"
WAKE_FD_DERIVATIVES = {
    "alpha": (("CL", "CL_alpha"), ("Cm", "Cm_alpha")),
    "elevator": (("Cm", "Cm_delta_e"),),
    "beta": (("CY", "CY_beta"), ("Cl", "Cl_beta"), ("Cn", "Cn_beta")),
    "aileron": (("Cl", "Cl_delta_a"),),
    "rudder": (("Cn", "Cn_delta_r"),),
}
WAKE_FD_NAMES = tuple(
    derivative for rows in WAKE_FD_DERIVATIVES.values() for _, derivative in rows
)


def convergence_identity(config: dict[str, Any], openvsp_version: str) -> dict[str, str]:
    digest = hashlib.sha256(config["_paths"]["model"].read_bytes()).hexdigest()
    payload = {
        "algorithm_version": NUMERICAL_ALGORITHM_VERSION,
        "numerical_convergence": config["numerical_convergence"],
        "manifest": {
            key: value for key, value in config["_manifest"].items()
            if not str(key).startswith("_")
        },
        "controls": config["controls"], "reference": config["reference"],
        "atmosphere": config["atmosphere"], "solver": config["solver"],
        "derivatives": config["derivatives"], "geometry_sets": config["geometry_sets"],
        "trim": config["trim"],
    }
    configuration_sha256 = hashlib.sha256(
        json.dumps(payload, sort_keys=True, ensure_ascii=False).encode("utf-8")
    ).hexdigest()
    return {
        "model_sha256": digest,
        "openvsp_version": str(openvsp_version),
        "convergence_configuration_sha256": configuration_sha256,
    }


def minimum_converged_level(
    levels: list[Any],
    values_by_level: dict[Any, dict[str, float]],
    quantities: Iterable[str],
    settings: dict[str, Any],
    *,
    allow_unverified_highest: bool,
) -> dict[str, Any]:
    """Choose the first level for which every later adjacent transition remains stable."""
    if len(levels) < 2:
        raise ValueError("At least two ordered convergence levels are required")
    transitions: list[dict[str, Any]] = []
    names = tuple(str(item) for item in quantities)
    for lower, higher in zip(levels, levels[1:]):
        checks: dict[str, Any] = {}
        for name in names:
            lower_values = values_by_level.get(lower, {})
            higher_values = values_by_level.get(higher, {})
            if name not in lower_values or name not in higher_values:
                checks[name] = {"status": "FAIL", "reason": "quantity is missing"}
            else:
                checks[name] = dual_tolerance_result(
                    float(lower_values[name]), float(higher_values[name]), settings
                )
        transitions.append({
            "lower": lower,
            "higher": higher,
            "status": combine_status([row["status"] for row in checks.values()]),
            "quantities": checks,
        })
    for index, level in enumerate(levels[:-1]):
        if all(row["status"] == "PASS" for row in transitions[index:]):
            return {
                "status": "PASS",
                "required_level": level,
                "reason": "all subsequent higher-level transitions satisfy PASS tolerance",
                "transitions": transitions,
            }
    last_status = transitions[-1]["status"]
    status = "WARN" if allow_unverified_highest and last_status == "WARN" else "FAIL"
    return {
        "status": status,
        "required_level": levels[-1],
        "reason": (
            "highest level is selected conservatively but remains unverified"
            if status == "WARN"
            else "the highest adjacent transition is not converged; the highest level is not accepted as correct"
        ),
        "transitions": transitions,
    }


def promote_discrete_level(level: int, candidates: list[int], margin_levels: int = 1) -> int:
    ordered = sorted({int(item) for item in candidates})
    index = ordered.index(int(level))
    return ordered[min(index + max(0, int(margin_levels)), len(ordered) - 1)]


def _schedule_scales(points: list[dict[str, Any]]) -> dict[str, float]:
    scales: dict[str, float] = {}
    for name in ("speed_mps", "alpha_deg", "beta_deg"):
        values = [float(point[name]) for point in points]
        scales[name] = max(max(values) - min(values), 1.0)
    return scales


def query_wake_schedule(schedule: dict[str, Any], state: dict[str, float]) -> dict[str, Any]:
    """Conservative discrete nearest-region lookup; wake values are never interpolated."""
    points = list(schedule.get("sample_points", []))
    candidates = [int(item) for item in schedule["candidates"]]
    if not points:
        raise ValueError("Wake schedule contains no sample points")
    scales = dict(schedule.get("axis_scales") or _schedule_scales(points))

    def distance(point: dict[str, Any]) -> float:
        return math.sqrt(sum(
            ((float(state[name]) - float(point[name])) / float(scales[name])) ** 2
            for name in ("speed_mps", "alpha_deg", "beta_deg")
        ))

    ranked = sorted(((distance(point), point) for point in points), key=lambda item: item[0])
    if ranked[0][0] <= 1.0e-12:
        wake = int(ranked[0][1]["required_wake"])
        return {"wake_iterations": wake, "source": "tested_point", "neighbors": [ranked[0][1]["name"]]}
    boundary_buffer = float(schedule.get("boundary_buffer_normalized", 0.12))
    neighbor_limit = max(1, int(schedule.get("neighbor_count", 4)))
    nearest_distance = ranked[0][0]
    local = [
        (dist, point) for dist, point in ranked[:neighbor_limit]
        if dist <= nearest_distance + boundary_buffer
    ]
    wake = max(int(point["required_wake"]) for _, point in local)
    wake = promote_discrete_level(
        wake, candidates, int(schedule.get("safety_margin_levels_for_untested", 1))
    )
    return {
        "wake_iterations": wake,
        "source": "conservative_discrete_neighbors",
        "neighbors": [point["name"] for _, point in local],
        "nearest_distance": nearest_distance,
        "boundary_buffer_normalized": boundary_buffer,
    }


def derivative_bundle_states(
    base_state: dict[str, float], derivative_config: dict[str, Any]
) -> list[dict[str, float]]:
    max_scale = max(float(item) for item in derivative_config["scales"])
    alpha_step = max_scale * float(derivative_config["perturbations"]["alpha_deg"])
    beta_step = max_scale * float(derivative_config["perturbations"]["beta_deg"])
    states = [dict(base_state)]
    for name, step in (("alpha_deg", alpha_step), ("beta_deg", beta_step)):
        for sign in (-1.0, 1.0):
            state = dict(base_state)
            state[name] = float(state[name]) + sign * step
            states.append(state)
    return states


def derivative_bundle_wake(
    schedule: dict[str, Any], base_state: dict[str, float], derivative_config: dict[str, Any]
) -> int:
    return max(
        int(query_wake_schedule(schedule, state)["wake_iterations"])
        for state in derivative_bundle_states(base_state, derivative_config)
    )


def trim_wake_decision(
    schedule: dict[str, Any], state: dict[str, float], derivative_config: dict[str, Any],
    current_wake: int | None = None,
) -> dict[str, Any]:
    required = derivative_bundle_wake(schedule, state, derivative_config)
    if current_wake is None:
        return {"action": "START_PRODUCTION", "wake_iterations": required, "required_wake": required}
    if required > int(current_wake):
        return {"action": "UPGRADE_AND_RETRIM", "wake_iterations": required, "required_wake": required}
    return {"action": "ACCEPT", "wake_iterations": int(current_wake), "required_wake": required}


def boundary_continuity_result(
    low_values: dict[str, float], high_values: dict[str, float], quantities: Iterable[str],
    settings: dict[str, Any],
) -> dict[str, Any]:
    checks = {
        name: dual_tolerance_result(
            float(low_values.get(name, math.nan)), float(high_values.get(name, math.nan)), settings
        )
        for name in quantities
    }
    return {"status": combine_status([item["status"] for item in checks.values()]), "quantities": checks}


def production_gate(
    settings: dict[str, Any] | None, *, force: bool = False, adaptive: bool = False,
    expected_identity: dict[str, str] | None = None,
) -> dict[str, Any]:
    if settings is None:
        status, reason = "FAIL", "production_numerical_settings.yaml is missing"
    else:
        gate = settings.get("production_gate", {})
        status = str(gate.get("status", settings.get("convergence_status", "FAIL"))).upper()
        reason = str(gate.get("reason", "numerical convergence result"))
        if expected_identity is not None and settings.get("identity") != expected_identity:
            status = "FAIL"
            reason = "Production Numerical Settings do not match the current model/OpenVSP/convergence configuration"
        if adaptive and status != "PASS":
            return {
                "allowed": False, "status": "FAIL", "forced": False,
                "reason": "adaptive GRID requires a PASS numerical convergence gate",
            }
    allowed = status in {"PASS", "WARN"} or bool(force)
    return {"allowed": allowed, "status": status, "forced": bool(force and status == "FAIL"), "reason": reason}


def diagnose_cy_delta_r(
    samples: dict[float, dict[str, dict[str, float]]], settings: dict[str, Any]
) -> dict[str, Any]:
    steps = sorted(float(item) for item in samples)
    if len(steps) < 4:
        return {"status": "FAIL", "classification": "NONLINEAR_OR_NUMERICALLY_UNRESOLVED", "recommended_delta_r_deg": None}
    derivatives: dict[float, dict[str, float]] = {}
    for step in steps:
        pair = samples[step]
        denominator = 2.0 * math.radians(step)
        derivatives[step] = {
            f"{coefficient}_delta_r":
            (float(pair["plus"][coefficient]) - float(pair["minus"][coefficient])) / denominator
            for coefficient in ("CY", "Cl", "Cn")
        }
    pair_statuses: list[str] = []
    comparisons: list[dict[str, Any]] = []
    for lower, higher in zip(steps, steps[1:]):
        checks = {
            name: dual_tolerance_result(derivatives[lower][name], derivatives[higher][name], settings)
            for name in derivatives[lower]
        }
        status = combine_status([item["status"] for item in checks.values()])
        pair_statuses.append(status)
        comparisons.append({"lower_step_deg": lower, "higher_step_deg": higher, "status": status, "quantities": checks})
    if pair_statuses[0] == "PASS" and pair_statuses[1] == "PASS":
        classification, recommended, status = "LOCAL_LINEAR_VALID", steps[1], "PASS"
        reason = "0.5-to-1 and 1-to-2 degree derivative changes satisfy PASS tolerance"
        if pair_statuses[2] != "PASS":
            classification, status = "LARGE_DEFLECTION_NONLINEAR", "WARN"
            reason = "the local small-step range is stable but the 2-to-4 degree transition is not"
    elif pair_statuses[0] != "PASS" and pair_statuses[1] == "PASS":
        classification, recommended, status = "NUMERICAL_NOISE", steps[1], "WARN"
        reason = "the 0.5-degree result is unstable while the 1-to-2 degree transition is stable"
    elif pair_statuses[0] == "PASS" and pair_statuses[1] != "PASS":
        classification, recommended, status = "LARGE_DEFLECTION_NONLINEAR", steps[1], "WARN"
        reason = "the 0.5-to-1 degree range is stable but the next transition is nonlinear"
    else:
        classification, recommended, status = "NONLINEAR_OR_NUMERICALLY_UNRESOLVED", None, "WARN"
        reason = "no consecutive local rudder-step range is clearly stable"
    return {
        "status": status,
        "classification": classification,
        "recommended_delta_r_deg": recommended,
        "reason": reason,
        "derivatives": derivatives,
        "comparisons": comparisons,
    }


def diagnose_cm_q(
    wake_values: dict[Any, float], tessellation_values: dict[Any, float], settings: dict[str, Any]
) -> dict[str, Any]:
    checks: list[dict[str, Any]] = []
    for source, values in (("wake", wake_values), ("tessellation", tessellation_values)):
        keys = list(values)
        for lower, higher in zip(keys, keys[1:]):
            result = dual_tolerance_result(float(values[lower]), float(values[higher]), settings)
            checks.append({"source": source, "lower": lower, "higher": higher, **result})
    numerical_status = combine_status([item["status"] for item in checks]) if checks else "FAIL"
    return {
        "status": "WARN",
        "numerical_status": numerical_status,
        "checks": checks,
        "api_limitation": (
            "OpenVSP 3.51.3 does not expose a negative steady q input; Cm_q remains a native "
            "positive-rate derivative and cannot be represented as a fabricated centered difference."
        ),
    }


def midpoint_interpolation_error(
    lower: dict[str, float], upper: dict[str, float], actual_midpoint: dict[str, float],
    quantities: Iterable[str], settings: dict[str, Any],
) -> dict[str, Any]:
    checks = {}
    for name in quantities:
        interpolated = 0.5 * (float(lower[name]) + float(upper[name]))
        checks[name] = {"interpolated": interpolated, "actual": float(actual_midpoint[name]), **dual_tolerance_result(interpolated, float(actual_midpoint[name]), settings)}
    return {"status": combine_status([item["status"] for item in checks.values()]), "quantities": checks}


def load_production_settings(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8-sig")) or {}
    return data if isinstance(data, dict) else None


def _plain_values(payload: dict[str, Any], manifest: dict[str, Any]) -> dict[str, float]:
    values = {
        name: float(payload["coefficients"][name]["standard_value"])
        for name in COEFFICIENTS
    }
    stability = payload.get("stability_derivatives", {})
    controls = payload.get("control_derivatives", {})
    for row in manifest["_required"]:
        name = str(row["name"])
        if name in stability:
            values[name] = float(stability[name]["standard_value"])
            continue
        for control in controls.values():
            derivative = control.get("derivatives", {}).get(name)
            if derivative is not None:
                values[name] = float(derivative["standard_value"])
                break
    return values


def _value_from_payload(payload: dict[str, Any], coefficient: str) -> float:
    return float(payload["coefficients"][coefficient]["standard_value"])


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        if fields:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _write_line_png(path: Path, series: list[tuple[list[float], list[float], tuple[int, int, int]]]) -> None:
    """Write a small dependency-free convergence plot."""
    width, height, pad = 720, 420, 48
    pixels = bytearray([255] * width * height * 3)
    all_x = [x for xs, _, _ in series for x in xs]
    all_y = [y for _, ys, _ in series for y in ys if math.isfinite(y)]
    if not all_x or not all_y:
        return
    x0, x1 = min(all_x), max(all_x)
    y0, y1 = min(all_y), max(all_y)
    if math.isclose(x0, x1):
        x1 = x0 + 1.0
    if math.isclose(y0, y1):
        y0, y1 = y0 - 0.5, y1 + 0.5

    def point(x: float, y: float) -> tuple[int, int]:
        px = pad + round((x - x0) / (x1 - x0) * (width - 2 * pad))
        py = height - pad - round((y - y0) / (y1 - y0) * (height - 2 * pad))
        return px, py

    def set_pixel(x: int, y: int, color: tuple[int, int, int]) -> None:
        if 0 <= x < width and 0 <= y < height:
            offset = (y * width + x) * 3
            pixels[offset:offset + 3] = bytes(color)

    def line(a: tuple[int, int], b: tuple[int, int], color: tuple[int, int, int]) -> None:
        x_a, y_a = a
        x_b, y_b = b
        steps = max(abs(x_b - x_a), abs(y_b - y_a), 1)
        for index in range(steps + 1):
            x = round(x_a + (x_b - x_a) * index / steps)
            y = round(y_a + (y_b - y_a) * index / steps)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    set_pixel(x + dx, y + dy, color)

    line((pad, pad), (pad, height - pad), (60, 60, 60))
    line((pad, height - pad), (width - pad, height - pad), (60, 60, 60))
    for xs, ys, color in series:
        plotted = [point(x, y) for x, y in zip(xs, ys) if math.isfinite(y)]
        for first, second in zip(plotted, plotted[1:]):
            line(first, second, color)
        for px, py in plotted:
            for dx in range(-3, 4):
                for dy in range(-3, 4):
                    if dx * dx + dy * dy <= 9:
                        set_pixel(px + dx, py + dy, color)
    raw = b"".join(b"\x00" + bytes(pixels[row * width * 3:(row + 1) * width * 3]) for row in range(height))
    chunks = []
    for name, data in ((b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)), (b"IDAT", zlib.compress(raw, 9)), (b"IEND", b"")):
        chunks.append(struct.pack(">I", len(data)) + name + data + struct.pack(">I", zlib.crc32(name + data) & 0xFFFFFFFF))
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + b"".join(chunks))


class _ConvergenceCache:
    def __init__(
        self, *, config: dict[str, Any], runner: Any, raw_root: Path,
        analysis_mapper: Callable[[Any], dict[str, Any]],
    ) -> None:
        self.runner = runner
        self.raw_root = raw_root
        self.analysis_mapper = analysis_mapper
        self.enabled = bool(config.get("_resume_enabled", True))
        self.identity = convergence_identity(config, str(runner.vsp.GetVSPVersion()))
        self.hits = 0
        self.misses = 0
        self.failures = 0
        self.solver_duration_sec = 0.0

    @staticmethod
    def _complete(payload: Any, stability: bool) -> bool:
        if not isinstance(payload, dict):
            return False
        coefficients = payload.get("coefficients")
        if not isinstance(coefficients, dict) or any(name not in coefficients for name in COEFFICIENTS):
            return False
        try:
            if not all(math.isfinite(float(coefficients[name]["standard_value"])) for name in COEFFICIENTS):
                return False
        except (KeyError, TypeError, ValueError):
            return False
        return not stability or all(
            isinstance(payload.get(name), dict)
            for name in ("stability_derivatives", "control_derivatives")
        )

    def run(
        self, *, condition: dict[str, float], controls: dict[str, float], wake: int,
        tessellation: list[dict[str, Any]], stability: bool, perturbation: dict[str, Any],
    ) -> dict[str, Any]:
        request = {
            "identity": self.identity,
            "condition": condition,
            "control_deflections_deg": controls,
            "wake_iterations": int(wake),
            "tessellation": tessellation,
            "analysis_type": "stability" if stability else "polar",
            "perturbation": perturbation,
            "include_thick": True,
        }
        signature = hashlib.sha256(
            json.dumps(request, sort_keys=True, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        case_dir = (self.raw_root / signature).resolve()
        if not case_dir.is_relative_to(self.raw_root.resolve()):
            raise RuntimeError(f"Unsafe numerical convergence case path: {case_dir}")
        result_path = case_dir / "case_result.json"
        if self.enabled and result_path.is_file():
            try:
                cached = json.loads(result_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                cached = {}
            payload = cached.get("payload")
            if (
                cached.get("signature") == signature
                and cached.get("status") == "SUCCESS"
                and self._complete(payload, stability)
            ):
                self.hits += 1
                return payload
        self.misses += 1
        if case_dir.exists():
            shutil.rmtree(case_dir)
        try:
            raw = self.runner.run(
                condition, self.raw_root, signature, stability=stability,
                control_deflections_deg=controls, wake_iterations=wake,
                tessellation_overrides=tessellation,
            )
            payload = (
                self.analysis_mapper(raw)
                if stability
                else {"coefficients": map_polar_coefficients(raw.raw_data)}
            )
            if not self._complete(payload, stability):
                raise RuntimeError("VSPAERO case returned an incomplete mapped payload")
            duration = float(getattr(raw, "duration_sec", 0.0))
            self.solver_duration_sec += duration
            record = {
                "schema_version": "1.0", "signature": signature, "status": "SUCCESS",
                "request": request, "payload": payload, "solver_duration_sec": duration,
            }
        except Exception as exc:
            self.failures += 1
            case_dir.mkdir(parents=True, exist_ok=True)
            record = {
                "schema_version": "1.0", "signature": signature, "status": "FAIL",
                "request": request, "error": f"{type(exc).__name__}: {exc}",
            }
            result_path.write_text(
                json.dumps(_json_safe(record), ensure_ascii=False, indent=2, allow_nan=False),
                encoding="utf-8",
            )
            raise
        case_dir.mkdir(parents=True, exist_ok=True)
        temporary = result_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(_json_safe(record), ensure_ascii=False, indent=2, allow_nan=False),
            encoding="utf-8",
        )
        temporary.replace(result_path)
        return payload

    def summary(self, wall_duration_sec: float) -> dict[str, Any]:
        return {
            "enabled": self.enabled, "hits": self.hits, "misses": self.misses,
            "failed_cases": self.failures, "solver_duration_sec": self.solver_duration_sec,
            "wall_duration_sec": wall_duration_sec,
        }


def _trim_representative(
    point: dict[str, Any], config: dict[str, Any], reference: dict[str, float],
    cache: _ConvergenceCache,
    wake: int, tessellation: list[dict[str, Any]],
) -> tuple[float, float, dict[str, Any]]:
    trim_config = deepcopy(config["trim"])
    last_condition = calculate_condition(
        speed_mps=float(point["speed_mps"]), alpha_deg=float(trim_config["alpha"]["initial_deg"]),
        beta_deg=float(point.get("beta_deg", 0.0)), atmosphere=config["atmosphere"],
        cref_m=reference["cref_m"],
    )

    def evaluate(
        alpha_deg: float, elevator_deg: float, label: str, stability: bool,
    ) -> dict[str, Any]:
        nonlocal last_condition
        last_condition = calculate_condition(
            speed_mps=float(point["speed_mps"]), alpha_deg=alpha_deg,
            beta_deg=float(point.get("beta_deg", 0.0)), atmosphere=config["atmosphere"],
            cref_m=reference["cref_m"],
        )
        controls = {
            role: float(config["controls"][role].get("neutral_deg", 0.0))
            for role in ("aileron", "elevator", "rudder")
        }
        controls["elevator"] = elevator_deg
        return cache.run(
            condition=last_condition, controls=controls, wake=wake,
            tessellation=tessellation, stability=stability,
            perturbation={"purpose": "pretrim", "state": str(point["name"]), "label": label},
        )

    result = solve_longitudinal_trim(
        evaluate=evaluate, trim_config=trim_config,
        derivative_config=config["derivatives"],
        dynamic_pressure_pa=last_condition["dynamic_pressure_pa"], reference=reference,
    )
    if not result.converged:
        return result.alpha_deg, result.elevator_deg, {
            "status": "FAIL", "iterations": result.iterations,
            "alpha_deg": result.alpha_deg, "elevator_deg": result.elevator_deg,
            "force_residual_n": result.force_residual_n,
            "moment_residual_nm": result.moment_residual_nm,
            "reason": result.failure_reason, "history": list(result.history),
        }
    return result.alpha_deg, result.elevator_deg, {
        "status": "PASS", "iterations": result.iterations,
        "alpha_deg": result.alpha_deg, "elevator_deg": result.elevator_deg,
        "force_residual_n": result.force_residual_n,
        "moment_residual_nm": result.moment_residual_nm,
        "history": list(result.history),
    }


def _report_markdown(report: dict[str, Any]) -> str:
    def formatted(value: Any) -> str:
        return f"{value:g}" if isinstance(value, (int, float)) and value is not None else str(value)

    lines = [
        "# Numerical Convergence Report", "",
        f"Generated: {report['generated_at_local']}", "",
        f"Production gate: **{report['production_gate_status']}**", "",
        "## Wake convergence map", "",
        "| State | V (m/s) | alpha (deg) | beta (deg) | Required Wake | Base | FD derivatives | Native diagnostic | Status | Why |",
        "|---|---:|---:|---:|---:|---|---|---|---|---|",
    ]
    for point in report["wake"]["sample_points"]:
        lines.append(
            f"| {point['name']} | {formatted(point.get('speed_mps'))} | "
            f"{formatted(point.get('alpha_deg'))} | {formatted(point.get('beta_deg'))} | "
            f"{formatted(point.get('required_wake'))} | {point.get('base_coefficient_status', 'SKIPPED')} | "
            f"{point.get('fd_derivative_status', 'SKIPPED')} | "
            f"{point.get('native_derivative_diagnostic', 'SKIPPED')} | "
            f"{point['status']} | {point['reason']} |"
        )
    lines.extend([
        "", "The map is generated from solver comparisons at the representative states. Untested "
        "states use a conservative discrete nearest-region lookup, a boundary buffer, and a configured "
        "one-level safety margin; no alpha threshold or linear Wake interpolation is used.", "",
        "## Boundary continuity", "",
        f"Status: **{report['wake']['boundary_status']}**. "
        f"Checks performed: {len(report['wake']['boundary_checks'])}. "
        "Any discontinuous low-Wake endpoint is automatically upgraded in the final map.", "",
        "## Tessellation", "",
        f"Recommended preset: **{report['tessellation']['recommended_preset']}** "
        f"({report['tessellation']['status']}). {report['tessellation']['reason']}", "",
        "## CY_delta_r diagnostic", "",
        f"Status: **{report['cy_delta_r']['status']}**; classification: "
        f"**{report['cy_delta_r']['classification']}**; recommended delta_r: "
        f"{report['cy_delta_r'].get('recommended_delta_r_deg')} deg. "
        f"{report['cy_delta_r'].get('reason', '')} The extra rudder points are diagnostic-only "
        "and are not expanded over the GRID.", "",
        "## Cm_q diagnostic", "",
        f"Status: **{report['cm_q']['status']}**; numerical sensitivity: "
        f"**{report['cm_q']['numerical_status']}**. {report['cm_q']['api_limitation']}", "",
        "## Production numerical settings", "",
        f"Uniform tessellation: **{report['tessellation']['recommended_preset']}**. Wake is selected "
        "per flight state, then promoted to the maximum required by the complete derivative bundle. "
        "TRIM uses a low-cost pre-trim followed by a production trim and only upgrades Wake between "
        "complete trim solves.", "",
        f"Final convergence status: **{report['convergence_status']}**; production gate: "
        f"**{report['production_gate_status']}**.", "",
        "## Cache / resume", "",
        f"Enabled: **{report['cache']['enabled']}**; hits: **{report['cache']['hits']}**; "
        f"misses: **{report['cache']['misses']}**; failed real cases: "
        f"**{report['cache']['failed_cases']}**; wall time: "
        f"**{report['cache']['wall_duration_sec']:.1f} s**.", "",
    ])
    return "\n".join(lines)


def run_numerical_convergence(
    *, config: dict[str, Any], runner: Any, reference: dict[str, float],
    manifest: dict[str, Any], analysis_mapper: Callable[[Any], dict[str, Any]],
) -> dict[str, Any]:
    started = time.perf_counter()
    settings = config["numerical_convergence"]
    wake_config = settings["wake"]
    tess_config = settings["tessellation"]
    output = config["_paths"]["results"] / "numerical_convergence"
    raw_root = (output / "raw").resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not raw_root.is_relative_to(output.resolve()):
        raise RuntimeError(f"Unsafe numerical convergence raw path: {raw_root}")
    raw_root.mkdir(parents=True, exist_ok=True)
    cache = _ConvergenceCache(
        config=config, runner=runner, raw_root=raw_root, analysis_mapper=analysis_mapper
    )

    candidates = [int(item) for item in wake_config["candidates"]]
    tolerance = wake_config["tolerance"]
    preset_map = {str(name).upper(): list(rows) for name, rows in tess_config["presets"].items()}
    study_preset = str(wake_config.get("study_tessellation_preset", "MEDIUM")).upper()
    monitored = [*COEFFICIENTS, *WAKE_FD_NAMES]
    controls_neutral = {
        role: float(config["controls"][role].get("neutral_deg", 0.0))
        for role in ("aileron", "elevator", "rudder")
    }

    def wake_bundle(
        condition: dict[str, float], controls: dict[str, float], wake: int,
    ) -> dict[str, Any]:
        coefficients: dict[str, float] = {}
        native: dict[str, float] = {}
        fd: dict[str, float] = {}
        errors: list[str] = []
        try:
            base = cache.run(
                condition=condition, controls=controls, wake=wake,
                tessellation=preset_map[study_preset], stability=True,
                perturbation={"purpose": "wake_gate", "variable": "base", "offset_deg": 0.0},
            )
            plain = _plain_values(base, manifest)
            coefficients = {name: plain[name] for name in COEFFICIENTS if name in plain}
            native = {name: plain[name] for name in WAKE_FD_NAMES if name in plain}
        except Exception as exc:
            errors.append(f"base stability: {type(exc).__name__}: {exc}")

        steps = config["derivatives"]["perturbations"]
        for variable, derivative_rows in WAKE_FD_DERIVATIVES.items():
            step_deg = float(steps[f"{variable}_deg"])
            control_cfg = config["controls"].get(variable)
            pair: dict[str, dict[str, Any]] = {}
            for direction, sign in (("minus", -1.0), ("plus", 1.0)):
                try:
                    varied_condition, varied_controls = perturb_state(
                        condition, controls, variable, sign * step_deg, control_cfg
                    )
                    pair[direction] = cache.run(
                        condition=varied_condition, controls=varied_controls, wake=wake,
                        tessellation=preset_map[study_preset], stability=False,
                        perturbation={
                            "purpose": "wake_gate", "variable": variable,
                            "direction": direction, "step_deg": step_deg,
                        },
                    )
                except Exception as exc:
                    errors.append(f"{variable} {direction}: {type(exc).__name__}: {exc}")
            if set(pair) != {"minus", "plus"}:
                continue
            for coefficient, derivative in derivative_rows:
                fd[derivative] = centered_derivative(
                    _value_from_payload(pair["plus"], coefficient),
                    _value_from_payload(pair["minus"], coefficient),
                    step_deg,
                )
        return {
            "gate_values": {**coefficients, **fd},
            "coefficients": coefficients,
            "fd_derivatives": fd,
            "native_derivatives": native,
            "errors": errors,
        }

    wake_details: list[dict[str, Any]] = []
    sample_points: list[dict[str, Any]] = []
    point_context: dict[str, dict[str, Any]] = {}

    for point_config in wake_config["representative_states"]:
        point = dict(point_config)
        name = str(point["name"])
        controls = dict(controls_neutral)
        pretrim = None
        try:
            if bool(point.get("trim", False)):
                alpha, elevator, pretrim = _trim_representative(
                    point, config, reference, cache,
                    int(settings["trim"]["pretrim_wake_iterations"]), preset_map["COARSE"],
                )
                if pretrim["status"] != "PASS":
                    sample = {
                        "name": name, "speed_mps": float(point["speed_mps"]),
                        "alpha_deg": None, "beta_deg": float(point.get("beta_deg", 0.0)),
                        "required_wake": None, "status": "FAIL",
                        "reason": f"Representative pre-trim failed: {pretrim.get('reason')}",
                        "source": "failed_case", "pretrim": pretrim,
                    }
                    sample_points.append(sample)
                    wake_details.append({
                        "point": sample, "values": {}, "status": "FAIL",
                        "error": sample["reason"],
                    })
                    continue
                point["alpha_deg"] = alpha
                controls["elevator"] = elevator
            condition = calculate_condition(
                speed_mps=float(point["speed_mps"]), alpha_deg=float(point["alpha_deg"]),
                beta_deg=float(point.get("beta_deg", 0.0)), atmosphere=config["atmosphere"],
                cref_m=reference["cref_m"],
            )
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"
            sample = {
                "name": name, "speed_mps": float(point["speed_mps"]),
                "alpha_deg": point.get("alpha_deg"),
                "beta_deg": float(point.get("beta_deg", 0.0)),
                "required_wake": None, "status": "FAIL", "reason": reason,
                "source": "failed_case", "pretrim": {
                    "status": "FAIL", "reason": reason,
                } if bool(point.get("trim", False)) else None,
            }
            sample_points.append(sample)
            wake_details.append({"point": sample, "values": {}, "status": "FAIL", "error": reason})
            continue

        bundles: dict[int, dict[str, Any]] = {}
        values_by_wake: dict[int, dict[str, float]] = {}
        native_by_wake: dict[int, dict[str, float]] = {}
        for wake in candidates:
            bundle = wake_bundle(condition, controls, wake)
            bundles[wake] = bundle
            values_by_wake[wake] = bundle["gate_values"]
            native_by_wake[wake] = bundle["native_derivatives"]
        base_decision = minimum_converged_level(
            candidates, values_by_wake, COEFFICIENTS, tolerance, allow_unverified_highest=True
        )
        fd_decision = minimum_converged_level(
            candidates, values_by_wake, WAKE_FD_NAMES, tolerance, allow_unverified_highest=True
        )
        decision = minimum_converged_level(
            candidates, values_by_wake, monitored, tolerance, allow_unverified_highest=True
        )
        native_decision = minimum_converged_level(
            candidates, native_by_wake, WAKE_FD_NAMES, tolerance, allow_unverified_highest=True
        )
        native_status = "PASS" if native_decision["status"] == "PASS" else "WARN"
        sample = {
            "name": name, "speed_mps": float(point["speed_mps"]),
            "alpha_deg": float(point["alpha_deg"]), "beta_deg": float(point.get("beta_deg", 0.0)),
            "required_wake": int(decision["required_level"]), "status": decision["status"],
            "reason": decision["reason"], "source": "solver_convergence", "pretrim": pretrim,
            "base_coefficient_status": base_decision["status"],
            "fd_derivative_status": fd_decision["status"],
            "native_derivative_diagnostic": native_status,
        }
        sample_points.append(sample)
        wake_details.append({
            "point": sample, "values": values_by_wake, "bundles": bundles,
            "decision": decision, "base_coefficient_decision": base_decision,
            "fd_derivative_decision": fd_decision,
            "native_derivative_diagnostic": {
                "status": native_status, "numerical_status": native_decision["status"],
                "decision": native_decision,
            },
        })
        point_context[name] = {
            "condition": condition, "controls": controls,
            "wake_values": native_by_wake, "gate_values": values_by_wake,
        }

    schedule_points = [point for point in sample_points if point.get("required_wake") is not None]
    schedule = {
        "algorithm": "conservative_discrete_nearest_regions",
        "candidates": candidates,
        "sample_points": schedule_points,
        "axis_scales": _schedule_scales(schedule_points) if schedule_points else {
            "speed_mps": 1.0, "alpha_deg": 1.0, "beta_deg": 1.0,
        },
        "neighbor_count": int(wake_config.get("neighbor_count", 4)),
        "boundary_buffer_normalized": float(wake_config["boundary_buffer_normalized"]),
        "safety_margin_levels_for_untested": int(wake_config["safety_margin_levels_for_untested"]),
    }

    boundary_checks: list[dict[str, Any]] = []
    pairs = []
    for index, first in enumerate(schedule_points):
        for second in schedule_points[index + 1:]:
            if int(first["required_wake"]) == int(second["required_wake"]):
                continue
            distance = math.sqrt(sum(
                ((float(first[key]) - float(second[key])) / float(schedule["axis_scales"][key])) ** 2
                for key in ("speed_mps", "alpha_deg", "beta_deg")
            ))
            pairs.append((distance, first, second))
    for _, first, second in sorted(pairs, key=lambda item: item[0])[:int(wake_config["max_boundary_checks"])]:
        midpoint = {
            key: 0.5 * (float(first[key]) + float(second[key]))
            for key in ("speed_mps", "alpha_deg", "beta_deg")
        }
        condition = calculate_condition(
            speed_mps=midpoint["speed_mps"], alpha_deg=midpoint["alpha_deg"],
            beta_deg=midpoint["beta_deg"], atmosphere=config["atmosphere"], cref_m=reference["cref_m"],
        )
        low, high = sorted((int(first["required_wake"]), int(second["required_wake"])))
        if low == high:
            continue
        values = {}
        for wake in (low, high):
            values[wake] = wake_bundle(condition, controls_neutral, wake)["gate_values"]
        continuity = boundary_continuity_result(values[low], values[high], monitored, tolerance)
        checks = continuity["quantities"]
        status = continuity["status"]
        action = "none"
        if status != "PASS":
            for endpoint in (first, second):
                if int(endpoint["required_wake"]) == low:
                    endpoint["required_wake"] = high
                    endpoint["source"] = "solver_convergence+boundary_upgrade"
                    endpoint["reason"] = (
                        f"boundary continuity required upgrade from Wake {low} to Wake {high}"
                    )
            action = f"upgraded low-Wake endpoint from {low} to {high}"
        boundary_checks.append({"first": first["name"], "second": second["name"], "midpoint": midpoint, "low_wake": low, "high_wake": high, "status": status, "action": action, "quantities": checks})
    boundary_status = combine_status([item["status"] for item in boundary_checks]) if boundary_checks else "PASS"
    if boundary_status == "FAIL" and all(item["action"] != "none" for item in boundary_checks if item["status"] == "FAIL"):
        boundary_status = "WARN"
    wake_status = combine_status([point["status"] for point in sample_points] + [boundary_status])

    tess_points = [str(item) for item in tess_config["representative_state_names"]]
    tess_details: list[dict[str, Any]] = []
    tess_monitored = [*COEFFICIENTS, *[str(row["name"]) for row in manifest["_required"]]]
    for name in tess_points:
        if name not in point_context or not schedule_points:
            tess_details.append({
                "point": name, "wake_iterations": None, "values": {},
                "decision": {
                    "status": "SKIPPED_DEPENDENCY", "required_level": None,
                    "reason": "representative aerodynamic state or Wake schedule is unavailable",
                    "transitions": [],
                },
            })
            continue
        context = point_context[name]
        condition = context["condition"]
        wake = derivative_bundle_wake(schedule, condition, config["derivatives"])
        values_by_preset: dict[str, dict[str, float]] = {}
        errors: dict[str, str] = {}
        for preset in PRESET_ORDER:
            try:
                payload = cache.run(
                    condition=condition, controls=context["controls"], wake=wake,
                    tessellation=preset_map[preset], stability=True,
                    perturbation={"purpose": "tessellation", "state": name, "preset": preset},
                )
                values_by_preset[preset] = _plain_values(payload, manifest)
            except Exception as exc:
                values_by_preset[preset] = {}
                errors[preset] = f"{type(exc).__name__}: {exc}"
        decision = minimum_converged_level(
            list(PRESET_ORDER), values_by_preset, tess_monitored, tess_config["tolerance"],
            allow_unverified_highest=False,
        )
        tess_details.append({
            "point": name, "wake_iterations": wake, "values": values_by_preset,
            "decision": decision, "errors": errors,
        })
    tess_statuses = [str(item["decision"]["status"]) for item in tess_details]
    tess_status = (
        "FAIL" if "SKIPPED_DEPENDENCY" in tess_statuses else combine_status(tess_statuses)
    )
    resolved_tess = [
        str(item["decision"]["required_level"]) for item in tess_details
        if item["decision"].get("required_level") in PRESET_ORDER
    ]
    recommended_preset = (
        PRESET_ORDER[max(PRESET_ORDER.index(item) for item in resolved_tess)]
        if resolved_tess else None
    )
    tess_reason = (
        "lowest uniform preset satisfying all later transitions at every representative state"
        if tess_status == "PASS"
        else "one or more tessellation cases failed, were dependency-skipped, or FINE remains unverified"
    )

    diagnostic_name = str(settings["diagnostics"]["representative_state_name"])
    rudder_samples: dict[float, dict[str, dict[str, float]]] = {}
    diagnostic_rows: list[dict[str, Any]] = []
    if diagnostic_name not in point_context or not schedule_points or recommended_preset is None:
        reason = "diagnostic representative state, Wake schedule, or tessellation preset is unavailable"
        cy_result = {
            "status": "SKIPPED_DEPENDENCY", "classification": "SKIPPED_DEPENDENCY",
            "recommended_delta_r_deg": None, "reason": reason,
            "derivatives": {}, "comparisons": [],
        }
        cm_q_result = {
            "status": "SKIPPED_DEPENDENCY", "numerical_status": "SKIPPED_DEPENDENCY",
            "checks": [], "api_limitation": reason,
        }
    else:
        diagnostic_context = point_context[diagnostic_name]
        diagnostic_condition = diagnostic_context["condition"]
        diagnostic_controls = diagnostic_context["controls"]
        diagnostic_wake = derivative_bundle_wake(
            schedule, diagnostic_condition, config["derivatives"]
        )
        diagnostic_errors: list[str] = []
        for step in [float(item) for item in settings["diagnostics"]["rudder_steps_deg"]]:
            pair: dict[str, dict[str, float]] = {}
            for direction, sign in (("minus", -1.0), ("plus", 1.0)):
                try:
                    varied_condition, controls = perturb_state(
                        diagnostic_condition, diagnostic_controls, "rudder", sign * step,
                        config["controls"]["rudder"],
                    )
                    payload = cache.run(
                        condition=varied_condition, controls=controls, wake=diagnostic_wake,
                        tessellation=preset_map[recommended_preset], stability=False,
                        perturbation={
                            "purpose": "CY_delta_r_diagnostic", "variable": "rudder",
                            "direction": direction, "step_deg": step,
                        },
                    )
                    pair[direction] = {
                        coefficient: _value_from_payload(payload, coefficient)
                        for coefficient in ("CY", "Cl", "Cn")
                    }
                    diagnostic_rows.append({
                        "diagnostic": "CY_delta_r", "step_deg": step,
                        "direction": direction, "rudder_deg": controls["rudder"],
                        **pair[direction],
                    })
                except Exception as exc:
                    diagnostic_errors.append(
                        f"rudder {step:g} {direction}: {type(exc).__name__}: {exc}"
                    )
            if set(pair) == {"minus", "plus"}:
                rudder_samples[step] = pair
        if len(rudder_samples) == len(settings["diagnostics"]["rudder_steps_deg"]):
            cy_result = diagnose_cy_delta_r(
                rudder_samples, settings["diagnostics"]["tolerance"]
            )
        else:
            cy_result = {
                "status": "FAIL", "classification": "INCOMPLETE_REAL_CASES",
                "recommended_delta_r_deg": None,
                "reason": "; ".join(diagnostic_errors) or "rudder diagnostic cases are incomplete",
                "derivatives": {}, "comparisons": [],
            }

        wake_cm_q = {
            wake: values.get("Cm_q", math.nan)
            for wake, values in diagnostic_context["wake_values"].items()
        }
        tess_context = next(item for item in tess_details if item["point"] == diagnostic_name)
        tess_cm_q = {
            preset: values.get("Cm_q", math.nan)
            for preset, values in tess_context["values"].items()
        }
        cm_q_result = diagnose_cm_q(
            wake_cm_q, tess_cm_q, settings["diagnostics"]["tolerance"]
        )
        for item in cm_q_result["checks"]:
            diagnostic_rows.append({"diagnostic": "Cm_q", **item})

    gate_status = combine_status([wake_status, tess_status, boundary_status])
    convergence_status = combine_status([gate_status, cy_result["status"], cm_q_result["status"]])
    native_diagnostic_status = combine_status([
        str(detail.get("native_derivative_diagnostic", {}).get("status", "WARN"))
        for detail in wake_details if "decision" in detail
    ])
    recommended_steps = {
        "aileron_deg": float(config["derivatives"]["perturbations"]["aileron_deg"]),
        "elevator_deg": float(config["derivatives"]["perturbations"]["elevator_deg"]),
    }
    if cy_result.get("recommended_delta_r_deg") is not None:
        recommended_steps["rudder_deg"] = float(cy_result["recommended_delta_r_deg"])
    production = {
        "schema_version": "2.0",
        "generated_at_local": datetime.now().astimezone().isoformat(timespec="seconds"),
        "identity": convergence_identity(config, str(runner.vsp.GetVSPVersion())),
        "convergence_status": convergence_status,
        "production_tessellation": {
            "preset": recommended_preset,
            "overrides": preset_map[recommended_preset] if recommended_preset is not None else [],
            "status": tess_status, "reason": tess_reason,
        },
        "wake_schedule": schedule,
        "derivative_bundle": {"rule": "maximum required Wake over base and every +/- derivative state", "enabled": True},
        "trim": {
            "rule": "pre-trim then fixed-Wake production trim using a centered-difference Jacobian and backtracking",
            "pretrim_wake_iterations": int(settings["trim"]["pretrim_wake_iterations"]),
            "max_wake_upgrades": int(settings["trim"]["max_wake_upgrades"]),
        },
        "recommended_control_perturbations_deg": recommended_steps,
        "recommendation_reasons": {
            "wake_schedule": "minimum level whose every later transition remains stable, then conservative untested-point and boundary rules",
            "tessellation": tess_reason,
            "aileron_deg": "retains the configured production delta and its per-Trim 0.5/1/2-scale validation",
            "elevator_deg": "retains the configured production delta and its per-Trim 0.5/1/2-scale validation",
            "rudder_deg": cy_result["reason"],
        },
        "diagnostics": {"CY_delta_r": cy_result, "Cm_q": cm_q_result},
        "production_gate": {
            "status": gate_status,
            "reason": "combined base-coefficient + key centered-FD Wake convergence, boundary continuity, and tessellation convergence",
            "force_option": "--force",
        },
    }
    cache_summary = cache.summary(time.perf_counter() - started)
    report = {
        "schema_version": "2.0", "generated_at_local": production["generated_at_local"],
        "convergence_status": convergence_status, "production_gate_status": gate_status,
        "wake": {"status": wake_status, "sample_points": sample_points, "details": wake_details, "boundary_status": boundary_status, "boundary_checks": boundary_checks},
        "tessellation": {"status": tess_status, "recommended_preset": recommended_preset, "reason": tess_reason, "details": tess_details},
        "cy_delta_r": {**cy_result, "raw_samples": rudder_samples}, "cm_q": cm_q_result,
        "native_derivative_diagnostic": {"status": native_diagnostic_status},
        "cache": cache_summary,
        "production_numerical_settings": production,
    }

    (output / "numerical_convergence_report.json").write_text(json.dumps(_json_safe(report), ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    (output / "numerical_convergence_report.md").write_text(_report_markdown(report), encoding="utf-8")
    (output / "production_numerical_settings.yaml").write_text(yaml.safe_dump(_json_safe(production), allow_unicode=True, sort_keys=False), encoding="utf-8")
    _write_csv(output / "wake_convergence_map.csv", [{key: value for key, value in point.items() if key != "pretrim"} for point in sample_points])
    tess_rows = []
    for detail in tess_details:
        for preset, values in detail["values"].items():
            for quantity, value in values.items():
                tess_rows.append({"state": detail["point"], "preset": preset, "quantity": quantity, "value": value, "recommended_preset": recommended_preset, "status": detail["decision"]["status"]})
    _write_csv(output / "tessellation_convergence.csv", tess_rows)
    _write_csv(output / "derivative_diagnostics.csv", diagnostic_rows)

    colors = [(36, 99, 235), (220, 38, 38), (5, 150, 105), (147, 51, 234), (234, 88, 12), (8, 145, 178)]
    valid_wake_points = [point for point in sample_points if point.get("required_wake") is not None]
    if valid_wake_points:
        _write_line_png(output / "wake_convergence_map.png", [(
            [float(index) for index in range(len(valid_wake_points))],
            [float(point["required_wake"]) for point in valid_wake_points], colors[0],
        )])
    tess_series = [
        (
            [0.0, 1.0, 2.0],
            [float(detail["values"].get(preset, {}).get("CL", math.nan)) for preset in PRESET_ORDER],
            colors[index % len(colors)],
        )
        for index, detail in enumerate(tess_details) if detail["values"]
    ]
    if tess_series:
        _write_line_png(output / "tessellation_convergence.png", tess_series)
    if rudder_samples:
        _write_line_png(output / "CY_vs_delta_r.png", [
            (
                [-step, step],
                [rudder_samples[step]["minus"]["CY"], rudder_samples[step]["plus"]["CY"]],
                colors[index % len(colors)],
            )
            for index, step in enumerate(sorted(rudder_samples))
        ])
    boundary_series = []
    for index, item in enumerate(boundary_checks):
        differences = [
            float(check.get("difference", math.nan)) for check in item["quantities"].values()
            if math.isfinite(float(check.get("difference", math.nan)))
        ]
        if differences:
            boundary_series.append((
                [float(item["low_wake"]), float(item["high_wake"])],
                [0.0, max(differences)], colors[index % len(colors)],
            ))
    if boundary_series:
        _write_line_png(output / "wake_boundary_check.png", boundary_series)
    return report
