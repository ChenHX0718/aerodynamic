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
    build_required_manifest_items,
    calculate_trim_derivatives,
    centered_derivative,
    combine_status,
    dual_tolerance_result,
    perturb_state,
    required_derivative_summary,
)
from trim_solver import solve_longitudinal_trim
from validation import calculate_condition


COEFFICIENTS = ("CL", "CD", "CY", "Cl", "Cm", "Cn")
NUMERICAL_ALGORITHM_VERSION = "3.0"
WAKE_FD_DERIVATIVES = {
    "alpha": (("CL", "CL_alpha"), ("CD", "CD_alpha"), ("Cm", "Cm_alpha")),
    "elevator": (
        ("CL", "CL_delta_e"), ("CD", "CD_delta_e"), ("Cm", "Cm_delta_e"),
    ),
    "beta": (("CY", "CY_beta"), ("Cl", "Cl_beta"), ("Cn", "Cn_beta")),
    "aileron": (
        ("CY", "CY_delta_a"), ("Cl", "Cl_delta_a"), ("Cn", "Cn_delta_a"),
    ),
    "rudder": (
        ("CY", "CY_delta_r"), ("Cl", "Cl_delta_r"), ("Cn", "Cn_delta_r"),
    ),
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
        "controls": config["controls"],
        "reference": config["reference"],
        "atmosphere": config["atmosphere"],
        "solver": config["solver"],
        "derivatives": config["derivatives"],
        "geometry_sets": config["geometry_sets"],
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
    """Choose the first level for which every later production transition is PASS."""
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
                "reason": "all subsequent production-level transitions satisfy PASS tolerance",
                "transitions": transitions,
            }
    last_status = transitions[-1]["status"]
    status = "WARN" if allow_unverified_highest and last_status == "WARN" else "FAIL"
    return {
        "status": status,
        "required_level": levels[-1],
        "reason": (
            "highest production level is selected conservatively but needs verification"
            if status == "WARN"
            else "the highest production transition is not acceptable without verification"
        ),
        "transitions": transitions,
    }


def terminal_wake_verification(
    *,
    production_levels: list[int],
    production_values: dict[int, dict[str, float]],
    quantities: Iterable[str],
    settings: dict[str, Any],
    verification_level: int,
    verification_values: dict[str, float] | None = None,
) -> dict[str, Any]:
    """Apply a verification-only level without making it a production Wake candidate."""
    decision = minimum_converged_level(
        production_levels,
        production_values,
        quantities,
        settings,
        allow_unverified_highest=True,
    )
    terminal = decision["transitions"][-1]
    if terminal["status"] == "PASS":
        return {
            **decision,
            "verification": {
                "triggered": False,
                "verification_level": verification_level,
                "status": "NOT_NEEDED",
                "reason": "the final production transition already satisfies PASS",
            },
        }
    if verification_values is None:
        return {
            **decision,
            "verification": {
                "triggered": True,
                "verification_level": verification_level,
                "status": "PENDING",
                "reason": "the final production transition is not PASS",
            },
        }
    highest = int(production_levels[-1])
    check = boundary_continuity_result(
        production_values.get(highest, {}), verification_values, quantities, settings
    )
    missing = any(
        name not in production_values.get(highest, {}) or name not in verification_values
        for name in quantities
    )
    if missing:
        status = "FAIL"
        reason = "Wake verification case is incomplete; required aerodynamic data are missing"
    elif check["status"] == "PASS":
        status = "PASS"
        reason = (
            f"Wake {highest}->{verification_level} satisfies PASS; production Wake remains {highest}"
        )
    else:
        status = "WARN"
        reason = (
            f"Wake {highest}->{verification_level} remains outside PASS tolerance; "
            f"production Wake remains {highest} with a numerical warning"
        )
    return {
        **decision,
        "status": status,
        "required_level": highest,
        "reason": reason,
        "verification": {
            "triggered": True,
            "production_level": highest,
            "verification_level": verification_level,
            "status": check["status"] if not missing else "FAIL",
            "quantities": check["quantities"],
            "production_level_unchanged": True,
        },
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
    """Conservative discrete nearest-region lookup; Wake values are never interpolated."""
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
    candidates = derivative_config["fd_step_candidates_deg"]
    states = [dict(base_state)]
    for name, variable in (("alpha_deg", "alpha"), ("beta_deg", "beta")):
        step = max(float(item) for item in candidates[variable])
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
    schedule: dict[str, Any],
    state: dict[str, float],
    derivative_config: dict[str, Any],
    current_wake: int | None = None,
) -> dict[str, Any]:
    required = derivative_bundle_wake(schedule, state, derivative_config)
    if current_wake is None:
        return {"action": "START_PRODUCTION", "wake_iterations": required, "required_wake": required}
    if required > int(current_wake):
        return {"action": "UPGRADE_AND_RETRIM", "wake_iterations": required, "required_wake": required}
    return {"action": "ACCEPT", "wake_iterations": int(current_wake), "required_wake": required}


def boundary_continuity_result(
    low_values: dict[str, float],
    high_values: dict[str, float],
    quantities: Iterable[str],
    settings: dict[str, Any],
) -> dict[str, Any]:
    checks = {
        name: dual_tolerance_result(
            float(low_values.get(name, math.nan)),
            float(high_values.get(name, math.nan)),
            settings,
        )
        for name in quantities
    }
    return {"status": combine_status([item["status"] for item in checks.values()]), "quantities": checks}


def production_gate(
    settings: dict[str, Any] | None,
    *,
    force: bool = False,
    adaptive: bool = False,
    expected_identity: dict[str, str] | None = None,
) -> dict[str, Any]:
    if settings is None:
        status, reason = "FAIL", "production_numerical_settings.yaml is missing"
    else:
        gate = settings.get("production_gate", {})
        status = str(gate.get("status", settings.get("convergence_status", "FAIL"))).upper()
        reason = str(gate.get("reason", "numerical validation result"))
        if expected_identity is not None and settings.get("identity") != expected_identity:
            status = "FAIL"
            reason = "Production Numerical Settings do not match the current model/OpenVSP/validation configuration"
        if adaptive and status != "PASS":
            return {
                "allowed": False,
                "status": "FAIL",
                "forced": False,
                "reason": "adaptive GRID requires a PASS production validation gate",
            }
    allowed = status in {"PASS", "WARN"} or bool(force)
    return {"allowed": allowed, "status": status, "forced": bool(force and status == "FAIL"), "reason": reason}


def midpoint_interpolation_error(
    lower: dict[str, float],
    upper: dict[str, float],
    actual_midpoint: dict[str, float],
    quantities: Iterable[str],
    settings: dict[str, Any],
) -> dict[str, Any]:
    checks = {}
    for name in quantities:
        interpolated = 0.5 * (float(lower[name]) + float(upper[name]))
        checks[name] = {
            "interpolated": interpolated,
            "actual": float(actual_midpoint[name]),
            **dual_tolerance_result(interpolated, float(actual_midpoint[name]), settings),
        }
    return {"status": combine_status([item["status"] for item in checks.values()]), "quantities": checks}


def load_production_settings(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    data = yaml.safe_load(path.read_text(encoding="utf-8-sig")) or {}
    return data if isinstance(data, dict) else None


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


def _write_line_png(
    path: Path,
    series: list[tuple[list[float], list[float], tuple[int, int, int]]],
) -> None:
    """Write a small dependency-free diagnostic plot."""
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
    raw = b"".join(
        b"\x00" + bytes(pixels[row * width * 3:(row + 1) * width * 3])
        for row in range(height)
    )
    chunks = []
    for name, data in (
        (b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)),
        (b"IDAT", zlib.compress(raw, 9)),
        (b"IEND", b""),
    ):
        chunks.append(
            struct.pack(">I", len(data)) + name + data
            + struct.pack(">I", zlib.crc32(name + data) & 0xFFFFFFFF)
        )
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + b"".join(chunks))


class _ConvergenceCache:
    def __init__(
        self,
        *,
        config: dict[str, Any],
        runner: Any,
        raw_root: Path,
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
        return not stability or isinstance(payload.get("native_derivative_diagnostics"), dict)

    def run(
        self,
        *,
        condition: dict[str, float],
        controls: dict[str, float],
        wake: int,
        stability: bool,
        perturbation: dict[str, Any],
    ) -> dict[str, Any]:
        # The signature contains every solver-affecting input. The human-readable
        # diagnostic purpose is recorded but deliberately does not split identical cases.
        signed_request = {
            "identity": self.identity,
            "condition": condition,
            "control_deflections_deg": controls,
            "wake_iterations": int(wake),
            "mesh_source": "current_openvsp_model",
            "analysis_type": "stability" if stability else "polar",
            "include_thick": True,
        }
        signature = hashlib.sha256(
            json.dumps(signed_request, sort_keys=True, ensure_ascii=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest()
        request = {**signed_request, "usage": perturbation}
        case_dir = (self.raw_root / signature).resolve()
        if not case_dir.is_relative_to(self.raw_root.resolve()):
            raise RuntimeError(f"Unsafe numerical validation case path: {case_dir}")
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
                condition,
                self.raw_root,
                signature,
                stability=stability,
                control_deflections_deg=controls,
                wake_iterations=wake,
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
                "schema_version": "2.0",
                "signature": signature,
                "status": "SUCCESS",
                "request": request,
                "payload": payload,
                "solver_duration_sec": duration,
            }
        except Exception as exc:
            self.failures += 1
            case_dir.mkdir(parents=True, exist_ok=True)
            record = {
                "schema_version": "2.0",
                "signature": signature,
                "status": "FAIL",
                "request": request,
                "error": f"{type(exc).__name__}: {exc}",
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
            "enabled": self.enabled,
            "hits": self.hits,
            "misses": self.misses,
            "solver_runs": self.misses,
            "failed_cases": self.failures,
            "solver_duration_sec": self.solver_duration_sec,
            "wall_duration_sec": wall_duration_sec,
        }


def _trim_representative(
    point: dict[str, Any],
    config: dict[str, Any],
    reference: dict[str, float],
    cache: _ConvergenceCache,
    wake: int,
) -> tuple[float, float, dict[str, Any]]:
    trim_config = deepcopy(config["trim"])
    last_condition = calculate_condition(
        speed_mps=float(point["speed_mps"]),
        alpha_deg=float(trim_config["alpha"]["initial_deg"]),
        beta_deg=float(point.get("beta_deg", 0.0)),
        atmosphere=config["atmosphere"],
        cref_m=reference["cref_m"],
    )

    def evaluate(alpha_deg: float, elevator_deg: float, label: str, stability: bool) -> dict[str, Any]:
        nonlocal last_condition
        last_condition = calculate_condition(
            speed_mps=float(point["speed_mps"]),
            alpha_deg=alpha_deg,
            beta_deg=float(point.get("beta_deg", 0.0)),
            atmosphere=config["atmosphere"],
            cref_m=reference["cref_m"],
        )
        controls = {
            role: float(config["controls"][role].get("neutral_deg", 0.0))
            for role in ("aileron", "elevator", "rudder")
        }
        controls["elevator"] = elevator_deg
        return cache.run(
            condition=last_condition,
            controls=controls,
            wake=wake,
            stability=stability,
            perturbation={"purpose": "pretrim", "state": str(point["name"]), "label": label},
        )

    result = solve_longitudinal_trim(
        evaluate=evaluate,
        trim_config=trim_config,
        derivative_config=config["derivatives"],
        dynamic_pressure_pa=last_condition["dynamic_pressure_pa"],
        reference=reference,
    )
    status = "PASS" if result.converged else "FAIL"
    return result.alpha_deg, result.elevator_deg, {
        "status": status,
        "iterations": result.iterations,
        "alpha_deg": result.alpha_deg,
        "elevator_deg": result.elevator_deg,
        "force_residual_n": result.force_residual_n,
        "moment_residual_nm": result.moment_residual_nm,
        "reason": result.failure_reason,
        "history": list(result.history),
    }


def _report_markdown(report: dict[str, Any]) -> str:
    def formatted(value: Any) -> str:
        return f"{value:g}" if isinstance(value, (int, float)) and value is not None else str(value)

    summary = report["required_derivatives_manifest"]["summary"]
    lines = [
        "# Numerical Convergence / Validation Report",
        "",
        f"Generated: {report['generated_at_local']}",
        "",
        f"Production gate: **{report['production_gate_status']}**",
        "",
        "Tessellation / mesh quality is user responsibility and is not numerically certified by this workflow.",
        "",
        "The workflow uses the mesh currently saved in the OpenVSP model and only checks that required VSPAERO cases and outputs are valid.",
        "",
        "## Wake convergence map",
        "",
        "| State | V (m/s) | alpha (deg) | beta (deg) | Production Wake | Base | FD derivatives | Wake 16 verification | Native diagnostic | Status | Why |",
        "|---|---:|---:|---:|---:|---|---|---|---|---|---|",
    ]
    for point in report["wake"]["sample_points"]:
        verification = point.get("wake16_verification", {})
        verify_text = (
            verification.get("status", "NOT_NEEDED")
            if verification.get("triggered") else "NOT_NEEDED"
        )
        lines.append(
            f"| {point['name']} | {formatted(point.get('speed_mps'))} | "
            f"{formatted(point.get('alpha_deg'))} | {formatted(point.get('beta_deg'))} | "
            f"{formatted(point.get('required_wake'))} | {point.get('base_coefficient_status', 'SKIPPED')} | "
            f"{point.get('fd_derivative_status', 'SKIPPED')} | {verify_text} | "
            f"{point.get('native_derivative_diagnostic', 'SKIPPED')} | "
            f"{point['status']} | {point['reason']} |"
        )
    lines.extend([
        "",
        "Wake 16 is verification-only. It is run only when the 8->12 transition is not PASS, never enters the production candidate list, and never changes a production Wake above 12.",
        "",
        "## FD step selection",
        "",
        "| Derivative | Selected step (deg) | Status | Value | Method |",
        "|---|---:|---|---:|---|",
    ])
    for name, record in report["fd_step_selection"].items():
        lines.append(
            f"| {name} | {formatted(record.get('selected_fd_step'))} | "
            f"{record.get('convergence_status')} | {formatted(record.get('derivative_value'))} | "
            f"{record.get('method')} |"
        )
    lines.extend([
        "",
        "Each derivative selects its own centered-FD step from the configured local candidates. Native derivatives are diagnostic references only and do not enter the Wake gate, production derivative set, or TRIM Jacobian.",
        "",
        "## Required derivatives manifest",
        "",
        f"Required: **{summary['required']}**; PASS: **{summary['PASS']}**; "
        f"WARN_NUMERICAL: **{summary['WARN_NUMERICAL']}**; "
        f"METHOD_LIMITATION: **{summary['METHOD_LIMITATION']}**; FAIL: **{summary['FAIL']}**.",
        "",
        "METHOD_LIMITATION follows the explicit policy in config/required_derivatives.yaml. In particular, OpenVSP/VSPAERO 3.51.3 cannot provide a true negative steady-q input, so Cm_q is not represented as a centered FD and its native value remains diagnostic-only.",
        "",
        "## Boundary continuity",
        "",
        f"Status: **{report['wake']['boundary_status']}**. Checks performed: "
        f"{len(report['wake']['boundary_checks'])}.",
        "",
        "## Cache / resume",
        "",
        f"Enabled: **{report['cache']['enabled']}**; cache hits: **{report['cache']['hits']}**; "
        f"new solver runs: **{report['cache']['solver_runs']}**; failed real cases: "
        f"**{report['cache']['failed_cases']}**; solver time: "
        f"**{report['cache']['solver_duration_sec']:.1f} s**; wall time: "
        f"**{report['cache']['wall_duration_sec']:.1f} s**.",
        "",
    ])
    return "\n".join(lines)


def run_numerical_convergence(
    *,
    config: dict[str, Any],
    runner: Any,
    reference: dict[str, float],
    manifest: dict[str, Any],
    analysis_mapper: Callable[[Any], dict[str, Any]],
) -> dict[str, Any]:
    started = time.perf_counter()
    settings = config["numerical_convergence"]
    wake_config = settings["wake"]
    output = config["_paths"]["results"] / "numerical_convergence"
    raw_root = (output / "raw").resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not raw_root.is_relative_to(output.resolve()):
        raise RuntimeError(f"Unsafe numerical validation raw path: {raw_root}")
    raw_root.mkdir(parents=True, exist_ok=True)
    cache = _ConvergenceCache(
        config=config,
        runner=runner,
        raw_root=raw_root,
        analysis_mapper=analysis_mapper,
    )

    candidates = [int(item) for item in wake_config["candidates"]]
    verification_level = int(wake_config["verification_only_level"])
    tolerance = wake_config["tolerance"]
    monitored = [*COEFFICIENTS, *WAKE_FD_NAMES]
    controls_neutral = {
        role: float(config["controls"][role].get("neutral_deg", 0.0))
        for role in ("aileron", "elevator", "rudder")
    }

    point_context: dict[str, dict[str, Any]] = {}
    unresolved_points: list[dict[str, Any]] = []
    for point_config in wake_config["representative_states"]:
        point = dict(point_config)
        name = str(point["name"])
        controls = dict(controls_neutral)
        pretrim = None
        try:
            if bool(point.get("trim", False)):
                alpha, elevator, pretrim = _trim_representative(
                    point,
                    config,
                    reference,
                    cache,
                    int(settings["trim"]["pretrim_wake_iterations"]),
                )
                if pretrim["status"] != "PASS":
                    raise RuntimeError(f"Representative pre-trim failed: {pretrim.get('reason')}")
                point["alpha_deg"] = alpha
                controls["elevator"] = elevator
            condition = calculate_condition(
                speed_mps=float(point["speed_mps"]),
                alpha_deg=float(point["alpha_deg"]),
                beta_deg=float(point.get("beta_deg", 0.0)),
                atmosphere=config["atmosphere"],
                cref_m=reference["cref_m"],
            )
            point_context[name] = {
                "point": point,
                "condition": condition,
                "controls": controls,
                "pretrim": pretrim,
            }
        except Exception as exc:
            unresolved_points.append({
                "name": name,
                "speed_mps": float(point["speed_mps"]),
                "alpha_deg": point.get("alpha_deg"),
                "beta_deg": float(point.get("beta_deg", 0.0)),
                "required_wake": None,
                "status": "FAIL",
                "reason": f"{type(exc).__name__}: {exc}",
                "source": "failed_case",
                "pretrim": pretrim,
            })

    diagnostic_name = str(settings["fd_step"]["representative_state_name"])
    diagnostic_package: dict[str, Any]
    diagnostic_error = ""
    if diagnostic_name in point_context:
        context = point_context[diagnostic_name]
        diagnostic_wake = candidates[-1]
        try:
            base = cache.run(
                condition=context["condition"],
                controls=context["controls"],
                wake=diagnostic_wake,
                stability=True,
                perturbation={"purpose": "fd_step_selection", "variable": "base"},
            )

            def diagnostic_polar(
                condition: dict[str, float], label: str, controls: dict[str, float]
            ) -> dict[str, Any]:
                payload = cache.run(
                    condition=condition,
                    controls=controls,
                    wake=diagnostic_wake,
                    stability=False,
                    perturbation={"purpose": "fd_step_selection", "label": label},
                )
                return {"coefficients": payload["coefficients"], "solver_duration_sec": 0.0}

            derivative_config = deepcopy(config["derivatives"])
            derivative_config["_controls"] = config["controls"]
            derivative_config["_bundle_wake_iterations"] = diagnostic_wake
            diagnostic_package = calculate_trim_derivatives(
                condition=context["condition"],
                base_outputs=base,
                base_controls=context["controls"],
                manifest=manifest,
                derivative_config=derivative_config,
                run_polar=diagnostic_polar,
            )
        except Exception as exc:
            diagnostic_error = f"{type(exc).__name__}: {exc}"
            native_records: dict[str, dict[str, Any]] = {}
            items = build_required_manifest_items(
                manifest=manifest,
                production_records={},
                native_records=native_records,
                wake_level=candidates[-1],
            )
            diagnostic_package = {
                "production_fd_derivatives": {},
                "native_derivative_diagnostics": native_records,
                "required_derivatives_manifest": {
                    "items": items,
                    "summary": required_derivative_summary(items, manifest),
                },
            }
    else:
        diagnostic_error = "FD-step representative state is unavailable"
        items = build_required_manifest_items(
            manifest=manifest,
            production_records={},
            native_records={},
            wake_level=candidates[-1],
        )
        diagnostic_package = {
            "production_fd_derivatives": {},
            "native_derivative_diagnostics": {},
            "required_derivatives_manifest": {
                "items": items,
                "summary": required_derivative_summary(items, manifest),
            },
        }

    fd_selection = diagnostic_package["production_fd_derivatives"]
    selected_steps: dict[str, float] = {}
    for rows in WAKE_FD_DERIVATIVES.values():
        for _, derivative in rows:
            record = fd_selection.get(derivative, {})
            if record.get("selected_fd_step") is not None:
                selected_steps[derivative] = float(record["selected_fd_step"])
            else:
                variable = str(manifest["_by_name"][derivative]["perturbation"])
                selected_steps[derivative] = float(
                    config["derivatives"]["fd_step_candidates_deg"][variable][1]
                )

    def wake_bundle(
        condition: dict[str, float], controls: dict[str, float], wake: int
    ) -> dict[str, Any]:
        coefficients: dict[str, float] = {}
        fd: dict[str, float] = {}
        errors: list[str] = []
        try:
            base = cache.run(
                condition=condition,
                controls=controls,
                wake=wake,
                stability=False,
                perturbation={"purpose": "wake_gate", "variable": "base"},
            )
            coefficients = {
                name: _value_from_payload(base, name) for name in COEFFICIENTS
            }
        except Exception as exc:
            errors.append(f"base polar: {type(exc).__name__}: {exc}")

        groups: dict[tuple[str, float], list[tuple[str, str]]] = {}
        for variable, derivative_rows in WAKE_FD_DERIVATIVES.items():
            for coefficient, derivative in derivative_rows:
                groups.setdefault((variable, selected_steps[derivative]), []).append(
                    (coefficient, derivative)
                )
        for (variable, step_deg), derivative_rows in groups.items():
            pair: dict[str, dict[str, Any]] = {}
            control_cfg = config["controls"].get(variable)
            for direction, sign in (("minus", -1.0), ("plus", 1.0)):
                try:
                    varied_condition, varied_controls = perturb_state(
                        condition, controls, variable, sign * step_deg, control_cfg
                    )
                    pair[direction] = cache.run(
                        condition=varied_condition,
                        controls=varied_controls,
                        wake=wake,
                        stability=False,
                        perturbation={
                            "purpose": "wake_gate",
                            "variable": variable,
                            "direction": direction,
                            "step_deg": step_deg,
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
            "production_fd_derivatives": fd,
            "errors": errors,
        }

    wake_details: list[dict[str, Any]] = []
    sample_points: list[dict[str, Any]] = list(unresolved_points)
    for name, context in point_context.items():
        condition = context["condition"]
        controls = context["controls"]
        bundles: dict[int, dict[str, Any]] = {}
        values_by_wake: dict[int, dict[str, float]] = {}
        for wake in candidates:
            bundle = wake_bundle(condition, controls, wake)
            bundles[wake] = bundle
            values_by_wake[wake] = bundle["gate_values"]
        pending = terminal_wake_verification(
            production_levels=candidates,
            production_values=values_by_wake,
            quantities=monitored,
            settings=tolerance,
            verification_level=verification_level,
        )
        verification_bundle = None
        verification_values = None
        if pending["verification"]["triggered"]:
            verification_bundle = wake_bundle(condition, controls, verification_level)
            verification_values = verification_bundle["gate_values"]
        decision = terminal_wake_verification(
            production_levels=candidates,
            production_values=values_by_wake,
            quantities=monitored,
            settings=tolerance,
            verification_level=verification_level,
            verification_values=verification_values,
        )
        base_decision = terminal_wake_verification(
            production_levels=candidates,
            production_values=values_by_wake,
            quantities=COEFFICIENTS,
            settings=tolerance,
            verification_level=verification_level,
            verification_values=verification_values,
        )
        fd_decision = terminal_wake_verification(
            production_levels=candidates,
            production_values=values_by_wake,
            quantities=WAKE_FD_NAMES,
            settings=tolerance,
            verification_level=verification_level,
            verification_values=verification_values,
        )
        native_status = "DIAGNOSTIC_ONLY"
        point = context["point"]
        required_wake = int(decision["required_level"])
        sample = {
            "name": name,
            "speed_mps": float(point["speed_mps"]),
            "alpha_deg": float(point["alpha_deg"]),
            "beta_deg": float(point.get("beta_deg", 0.0)),
            "required_wake": required_wake,
            "status": decision["status"],
            "reason": decision["reason"],
            "source": "solver_convergence",
            "pretrim": context["pretrim"],
            "base_coefficient_status": base_decision["status"],
            "fd_derivative_status": fd_decision["status"],
            "native_derivative_diagnostic": native_status,
            "wake16_verification": decision["verification"],
        }
        sample_points.append(sample)
        wake_details.append({
            "point": sample,
            "production_values": values_by_wake,
            "production_bundles": bundles,
            "verification_only_bundle": verification_bundle,
            "decision": decision,
            "base_coefficient_decision": base_decision,
            "fd_derivative_decision": fd_decision,
            "native_derivative_diagnostic": {
                "status": native_status,
                "enters_production_gate": False,
            },
        })
        context["gate_values"] = values_by_wake

    schedule_points = [
        point for point in sample_points
        if point.get("required_wake") is not None and point.get("status") != "FAIL"
    ]
    schedule = {
        "algorithm": "conservative_discrete_nearest_regions",
        "candidates": candidates,
        "verification_only_level": verification_level,
        "sample_points": schedule_points,
        "axis_scales": _schedule_scales(schedule_points) if schedule_points else {
            "speed_mps": 1.0,
            "alpha_deg": 1.0,
            "beta_deg": 1.0,
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
            speed_mps=midpoint["speed_mps"],
            alpha_deg=midpoint["alpha_deg"],
            beta_deg=midpoint["beta_deg"],
            atmosphere=config["atmosphere"],
            cref_m=reference["cref_m"],
        )
        low, high = sorted((int(first["required_wake"]), int(second["required_wake"])))
        values = {
            wake: wake_bundle(condition, controls_neutral, wake)["gate_values"]
            for wake in (low, high)
        }
        continuity = boundary_continuity_result(values[low], values[high], monitored, tolerance)
        status = continuity["status"]
        action = "none"
        if status != "PASS":
            for endpoint in (first, second):
                if int(endpoint["required_wake"]) == low:
                    endpoint["required_wake"] = high
                    endpoint["source"] = "solver_convergence+boundary_upgrade"
                    endpoint["reason"] = f"boundary continuity required upgrade from Wake {low} to Wake {high}"
            action = f"upgraded low-Wake endpoint from {low} to {high}"
        boundary_checks.append({
            "first": first["name"],
            "second": second["name"],
            "midpoint": midpoint,
            "low_wake": low,
            "high_wake": high,
            "status": status,
            "action": action,
            "quantities": continuity["quantities"],
        })
    boundary_status = combine_status([item["status"] for item in boundary_checks]) if boundary_checks else "PASS"
    if boundary_status == "FAIL" and all(
        item["action"] != "none" for item in boundary_checks if item["status"] == "FAIL"
    ):
        boundary_status = "WARN"
    wake_status = combine_status([point["status"] for point in sample_points] + [boundary_status])

    cm_q_diagnostic = {
        "status": "METHOD_LIMITATION",
        "numerical_diagnostic_status": "NOT_APPLICABLE",
        "diagnostic_value": diagnostic_package.get("native_derivative_diagnostics", {}).get(
            "Cm_q", {}
        ).get("diagnostic_value"),
        "reason": manifest["method_limitations"]["steady_rate_centered_difference"]["reason"],
        "enters_production_gate": False,
    }

    required_manifest = diagnostic_package["required_derivatives_manifest"]
    manifest_gate_status = str(required_manifest["summary"]["gate_status"])
    gate_status = combine_status([wake_status, boundary_status, manifest_gate_status])
    convergence_status = gate_status
    native_diagnostic_status = "DIAGNOSTIC_ONLY"
    selected_fd_steps = {
        name: float(record["selected_fd_step"])
        for name, record in fd_selection.items()
        if record.get("selected_fd_step") is not None
    }
    generated = datetime.now().astimezone().isoformat(timespec="seconds")
    production = {
        "schema_version": "3.0",
        "generated_at_local": generated,
        "identity": convergence_identity(config, str(runner.vsp.GetVSPVersion())),
        "convergence_status": convergence_status,
        "mesh": {
            "source": "current_openvsp_model",
            "numerically_certified": False,
            "responsibility": "user",
            "statement": "Tessellation / mesh quality is user responsibility and is not numerically certified by this workflow.",
        },
        "wake_schedule": schedule,
        "derivative_bundle": {
            "rule": "maximum required production Wake over base and every +/- derivative state",
            "enabled": True,
            "verification_only_level": verification_level,
        },
        "trim": {
            "rule": "pre-trim then fixed-Wake production trim using a centered-difference Jacobian and backtracking",
            "pretrim_wake_iterations": int(settings["trim"]["pretrim_wake_iterations"]),
            "max_wake_upgrades": int(settings["trim"]["max_wake_upgrades"]),
        },
        "selected_fd_steps_deg": selected_fd_steps,
        "required_derivatives_manifest": required_manifest,
        "diagnostics": {
            "native_derivatives": {"status": native_diagnostic_status, "enters_production_gate": False},
            "Cm_q": cm_q_diagnostic,
        },
        "production_gate": {
            "status": gate_status,
            "reason": (
                "Wake coefficient/required-FD validation, boundary continuity, and the explicit "
                "required-derivative manifest policy; mesh convergence is excluded"
            ),
            "force_option": "--force",
        },
    }
    cache_summary = cache.summary(time.perf_counter() - started)
    report = {
        "schema_version": "3.0",
        "generated_at_local": generated,
        "convergence_status": convergence_status,
        "production_gate_status": gate_status,
        "mesh": production["mesh"],
        "wake": {
            "status": wake_status,
            "sample_points": sample_points,
            "details": wake_details,
            "boundary_status": boundary_status,
            "boundary_checks": boundary_checks,
        },
        "fd_step_selection": fd_selection,
        "fd_step_diagnostic_error": diagnostic_error,
        "required_derivatives_manifest": required_manifest,
        "cm_q": cm_q_diagnostic,
        "native_derivative_diagnostic": {"status": native_diagnostic_status},
        "cache": cache_summary,
        "production_numerical_settings": production,
    }

    (output / "numerical_convergence_report.json").write_text(
        json.dumps(_json_safe(report), ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    (output / "numerical_convergence_report.md").write_text(
        _report_markdown(report), encoding="utf-8"
    )
    (output / "production_numerical_settings.yaml").write_text(
        yaml.safe_dump(_json_safe(production), allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    _write_csv(
        output / "wake_convergence_map.csv",
        [{key: value for key, value in point.items() if key not in {"pretrim", "wake16_verification"}}
         | {
             "wake16_triggered": point.get("wake16_verification", {}).get("triggered", False),
             "wake16_status": point.get("wake16_verification", {}).get("status", "NOT_NEEDED"),
         }
         for point in sample_points],
    )
    _write_csv(output / "fd_step_selection.csv", [
        {
            "name": name,
            "selected_fd_step": record.get("selected_fd_step"),
            "convergence_status": record.get("convergence_status"),
            "derivative_value": record.get("derivative_value"),
            "method": record.get("method"),
        }
        for name, record in fd_selection.items()
    ])
    _write_csv(output / "required_derivatives_manifest.csv", required_manifest["items"])

    colors = [(36, 99, 235), (220, 38, 38), (5, 150, 105), (147, 51, 234), (234, 88, 12)]
    valid_wake_points = [point for point in sample_points if point.get("required_wake") is not None]
    if valid_wake_points:
        _write_line_png(output / "wake_convergence_map.png", [(
            [float(index) for index in range(len(valid_wake_points))],
            [float(point["required_wake"]) for point in valid_wake_points],
            colors[0],
        )])
    fd_series = []
    for index, record in enumerate(fd_selection.values()):
        values = record.get("convergence", {}).get("values_by_step", {})
        if values:
            ordered = sorted((float(step), float(value)) for step, value in values.items())
            fd_series.append((
                [step for step, _ in ordered],
                [value for _, value in ordered],
                colors[index % len(colors)],
            ))
    if fd_series:
        _write_line_png(output / "fd_step_convergence.png", fd_series)
    return report
