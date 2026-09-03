from __future__ import annotations

import math
from typing import Any, Callable

from coordinate_system import COORDINATE_CONVENTION


STATUS_RANK = {
    "PASS": 0,
    "METHOD_LIMITATION": 1,
    "WARN": 2,
    "WARN_NUMERICAL": 2,
    "FAIL": 3,
}


def combine_status(statuses: list[str]) -> str:
    normalized = [str(item).upper() for item in statuses if str(item).upper() in STATUS_RANK]
    return max(normalized, key=STATUS_RANK.get) if normalized else "PASS"


def dual_tolerance_result(
    reference: float, candidate: float, settings: dict[str, Any]
) -> dict[str, Any]:
    """Common absolute-plus-relative comparison with a near-zero reference floor."""
    if not math.isfinite(float(reference)) or not math.isfinite(float(candidate)):
        return {"status": "FAIL", "difference": math.nan, "reason": "non-finite value"}
    scale = max(abs(float(reference)), abs(float(candidate)), float(settings["near_zero_reference"]))
    difference = abs(float(candidate) - float(reference))
    pass_limit = float(settings["pass_absolute"]) + float(settings["pass_relative"]) * scale
    warn_limit = float(settings["warn_absolute"]) + float(settings["warn_relative"]) * scale
    status = "PASS" if difference <= pass_limit else "WARN" if difference <= warn_limit else "FAIL"
    return {
        "status": status,
        "difference": difference,
        "relative_difference": difference / scale,
        "pass_limit": pass_limit,
        "warn_limit": warn_limit,
        "reference_scale": scale,
    }


def convergence_result(values: dict[str, float], settings: dict[str, Any]) -> dict[str, Any]:
    """Compare the traditional 0.5/1/2 scale triplet."""
    required = ("0.5", "1", "2")
    if any(key not in values or not math.isfinite(float(values[key])) for key in required):
        return {
            "status": "FAIL",
            "reason": "one or more finite-difference scales are missing or non-finite",
            "values": values,
        }
    nominal = float(values["1"])
    comparisons = [
        dual_tolerance_result(nominal, float(values[key]), settings) for key in ("0.5", "2")
    ]
    worst = max(comparisons, key=lambda item: (STATUS_RANK[item["status"]], item["difference"]))
    status = combine_status([item["status"] for item in comparisons])
    relative = max(float(item["relative_difference"]) for item in comparisons)
    return {
        "status": status,
        "reason": (
            "step variation is within PASS absolute+relative tolerance"
            if status == "PASS"
            else "step variation exceeds PASS but is within WARN tolerance"
            if status == "WARN"
            else "step variation exceeds WARN absolute+relative tolerance"
        ),
        "values": {key: float(values[key]) for key in required},
        "max_absolute_change": max(float(item["difference"]) for item in comparisons),
        "relative_variation": relative,
        "relative_variation_pct": 100.0 * relative,
        "pass_limit": float(worst["pass_limit"]),
        "warn_limit": float(worst["warn_limit"]),
        "near_zero_reference": float(settings["near_zero_reference"]),
    }


def perturb_state(
    condition: dict[str, float],
    controls: dict[str, float],
    variable: str,
    offset_deg: float,
    control_config: dict[str, Any] | None = None,
) -> tuple[dict[str, float], dict[str, float]]:
    """Return one aerodynamic state perturbed in the project's degree convention."""
    perturbed_condition = dict(condition)
    perturbed_controls = dict(controls)
    if variable in {"alpha", "beta"}:
        perturbed_condition[f"{variable}_deg"] = (
            float(perturbed_condition[f"{variable}_deg"]) + float(offset_deg)
        )
    elif variable in perturbed_controls:
        value = float(perturbed_controls[variable]) + float(offset_deg)
        if control_config is not None:
            lower = float(control_config.get("min_deg", -90.0))
            upper = float(control_config.get("max_deg", 90.0))
            if not lower <= value <= upper:
                raise ValueError(
                    f"{variable} perturbation {value:g} deg exceeds configured limits "
                    f"{lower:g}..{upper:g} deg"
                )
        perturbed_controls[variable] = value
    else:
        raise ValueError(f"Unsupported centered-difference variable: {variable}")
    return perturbed_condition, perturbed_controls


def centered_derivative(plus: float, minus: float, step_deg: float) -> float:
    """Centered derivative per radian for a symmetric step expressed in degrees."""
    step = float(step_deg)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError(f"Centered-difference step must be positive, got {step_deg}")
    return (float(plus) - float(minus)) / (2.0 * math.radians(step))


def _step_key(step: float) -> str:
    return f"{float(step):g}"


def select_fd_step(
    values_by_step: dict[float, float],
    settings: dict[str, Any],
    *,
    preferred_step: float | None = None,
) -> dict[str, Any]:
    """Select the interior of the first stable local range, avoiding the smallest noisy step."""
    ordered = sorted(float(step) for step in values_by_step)
    if len(ordered) < 2 or any(not math.isfinite(float(values_by_step[step])) for step in ordered):
        return {
            "status": "FAIL",
            "selected_fd_step": None,
            "derivative_value": None,
            "reason": "fewer than two finite centered-difference step results are available",
            "comparisons": [],
            "values_by_step": {_step_key(step): values_by_step[step] for step in ordered},
        }
    comparisons = []
    for lower, higher in zip(ordered, ordered[1:]):
        comparison = dual_tolerance_result(
            float(values_by_step[lower]), float(values_by_step[higher]), settings
        )
        comparisons.append({
            "lower_step_deg": lower,
            "higher_step_deg": higher,
            **comparison,
        })

    def stable_candidates(target: str) -> list[float]:
        return [
            float(item["higher_step_deg"])
            for item in comparisons
            if item["status"] == target
        ]

    pass_candidates = stable_candidates("PASS")
    warn_candidates = stable_candidates("WARN")
    selected: float
    if preferred_step is not None and any(
        math.isclose(float(preferred_step), item, abs_tol=1.0e-12) for item in pass_candidates
    ):
        selected = float(preferred_step)
        status = "PASS"
        reason = "preferred representative-point step remains inside a local PASS-stable range"
    elif pass_candidates:
        selected = pass_candidates[0]
        status = "PASS"
        reason = "selected the larger endpoint of the first local PASS-stable range"
    elif preferred_step is not None and any(
        math.isclose(float(preferred_step), item, abs_tol=1.0e-12) for item in warn_candidates
    ):
        selected = float(preferred_step)
        status = "WARN_NUMERICAL"
        reason = "preferred step is only WARN-stable at this flight point"
    elif warn_candidates:
        selected = warn_candidates[0]
        status = "WARN_NUMERICAL"
        reason = "no PASS-stable range exists; selected the first WARN-stable local range"
    else:
        selected = ordered[1]
        status = "FAIL"
        reason = "no adjacent finite-difference step range satisfies the WARN tolerance"
    return {
        "status": status,
        "selected_fd_step": selected,
        "derivative_value": float(values_by_step[selected]),
        "reason": reason,
        "comparisons": comparisons,
        "values_by_step": {_step_key(step): float(values_by_step[step]) for step in ordered},
    }


def _plain_coefficients(mapped: dict[str, dict[str, Any]]) -> dict[str, float]:
    return {name: float(item["standard_value"]) for name, item in mapped.items()}


def _state_snapshot(condition: dict[str, float], controls: dict[str, float]) -> dict[str, Any]:
    return {
        "speed_mps": float(condition["speed_mps"]),
        "alpha_deg": float(condition["alpha_deg"]),
        "beta_deg": float(condition["beta_deg"]),
        "aileron_deg": float(controls["aileron"]),
        "elevator_deg": float(controls["elevator"]),
        "rudder_deg": float(controls["rudder"]),
    }


def _native_value(payload: dict[str, Any], name: str) -> float | None:
    diagnostics = payload.get("native_derivative_diagnostics", {})
    stability = diagnostics.get("stability", {})
    if name in stability:
        return float(stability[name]["standard_value"])
    for control in diagnostics.get("controls", {}).values():
        derivative = control.get("derivatives", {}).get(name)
        if derivative is not None:
            return float(derivative["standard_value"])
    return None


def _coordinate_sign_convention() -> str:
    return (
        f"{COORDINATE_CONVENTION['internal_axes']}; {COORDINATE_CONVENTION['angles']}; "
        f"{COORDINATE_CONVENTION['controls']}; {COORDINATE_CONVENTION['conversion']}"
    )


def required_derivative_summary(
    items: list[dict[str, Any]], manifest: dict[str, Any]
) -> dict[str, Any]:
    statuses = ("PASS", "WARN_NUMERICAL", "METHOD_LIMITATION", "FAIL")
    counts = {
        status: sum(item.get("validation_status") == status for item in items)
        for status in statuses
    }
    policy = {key: str(value).upper() for key, value in manifest["status_policy"].items()}
    actions = [policy[str(item["validation_status"])] for item in items if item.get("required")]
    gate_status = (
        "FAIL" if "REJECT" in actions
        else "WARN" if "ACCEPT_WITH_WARNING" in actions
        else "PASS"
    )
    return {
        "required": sum(bool(item.get("required")) for item in items),
        **counts,
        "production_included": sum(bool(item.get("production_included")) for item in items),
        "gate_status": gate_status,
        "status_policy": policy,
        "pass_names": [item["name"] for item in items if item["validation_status"] == "PASS"],
        "warn_numerical_names": [
            item["name"] for item in items if item["validation_status"] == "WARN_NUMERICAL"
        ],
        "method_limitation_names": [
            item["name"] for item in items if item["validation_status"] == "METHOD_LIMITATION"
        ],
        "fail_names": [item["name"] for item in items if item["validation_status"] == "FAIL"],
    }


def build_required_manifest_items(
    *,
    manifest: dict[str, Any],
    production_records: dict[str, dict[str, Any]],
    native_records: dict[str, dict[str, Any]],
    wake_level: int | None,
) -> list[dict[str, Any]]:
    convention = _coordinate_sign_convention()
    items = []
    for definition in manifest["_required"]:
        name = str(definition["name"])
        production = production_records.get(name)
        native = native_records.get(name, {})
        limitation_key = definition.get("method_limitation")
        if production is not None:
            status = str(production["convergence_status"])
            value = production.get("derivative_value")
            source = "production_centered_fd"
            method = "centered_finite_difference"
            step = production.get("selected_fd_step")
            production_included = status != "FAIL"
            reason = production.get("convergence", {}).get("reason", "")
        elif limitation_key is not None:
            limitation = manifest["method_limitations"][str(limitation_key)]
            status = str(limitation["status"])
            value = native.get("diagnostic_value")
            source = "vspaero_native_diagnostic_reference"
            method = "centered_finite_difference_unavailable"
            step = None
            production_included = bool(limitation["production_included"])
            reason = str(limitation["reason"])
        else:
            status = "FAIL"
            value = None
            source = "missing"
            method = "centered_finite_difference"
            step = None
            production_included = False
            reason = "required production centered-FD derivative is missing"
        items.append({
            "name": name,
            "category": str(definition["category"]),
            "coefficient": str(definition["coefficient"]),
            "perturbation": str(definition["perturbation"]),
            "definition": str(definition["definition"]),
            "required": bool(definition["required"]),
            "value": value,
            "source": source,
            "method": method,
            "selected_fd_step": step,
            "selected_fd_step_unit": "deg" if step is not None else None,
            "units": str(definition["unit"]),
            "coordinate_sign_convention": convention,
            "wake_level": wake_level,
            "validation_status": status,
            "production_included": production_included,
            "gate_action": str(manifest["status_policy"][status]).upper(),
            "reason": reason,
        })
    return items


def calculate_trim_derivatives(
    *,
    condition: dict[str, float],
    base_outputs: dict[str, Any],
    base_controls: dict[str, float],
    manifest: dict[str, Any],
    derivative_config: dict[str, Any],
    run_polar: Callable[[dict[str, float], str, dict[str, float]], dict[str, Any]],
) -> dict[str, Any]:
    """Build production centered-FD data, native diagnostics, and the required manifest."""
    candidates = derivative_config["fd_step_candidates_deg"]
    convergence_settings = derivative_config["convergence"]
    preferred_steps = derivative_config.get("_preferred_fd_steps", {})
    rows = list(manifest["derivatives"])
    production_records: dict[str, dict[str, Any]] = {}
    native_records: dict[str, dict[str, Any]] = {}
    run_count = 0
    solver_duration = 0.0

    for row in rows:
        name = str(row["name"])
        value = _native_value(base_outputs, name)
        native_records[name] = {
            "name": name,
            "diagnostic_value": value,
            "source": "vspaero_native_derivative",
            "method": "vspaero_native_derivative",
            "units": str(row["unit"]),
            "diagnostic_only": True,
            "enters_production_gate": False,
            "enters_trim_jacobian": False,
            "production_included": False,
        }

    for variable in ("alpha", "beta", "aileron", "elevator", "rudder"):
        derivative_rows = [row for row in rows if row["perturbation"] == variable]
        if not derivative_rows:
            continue
        sample_outputs: dict[float, dict[str, Any]] = {}
        for step_deg in [float(item) for item in candidates[variable]]:
            control_cfg = derivative_config["_controls"].get(variable)
            plus_condition, plus_controls = perturb_state(
                condition, base_controls, variable, step_deg, control_cfg
            )
            minus_condition, minus_controls = perturb_state(
                condition, base_controls, variable, -step_deg, control_cfg
            )
            key = _step_key(step_deg)
            plus_payload = run_polar(plus_condition, f"fd_{variable}_{key}_plus", plus_controls)
            minus_payload = run_polar(minus_condition, f"fd_{variable}_{key}_minus", minus_controls)
            run_count += 2
            solver_duration += float(plus_payload.get("solver_duration_sec", 0.0))
            solver_duration += float(minus_payload.get("solver_duration_sec", 0.0))
            sample_outputs[step_deg] = {
                "step_deg": step_deg,
                "derivative_denominator_rad": 2.0 * math.radians(step_deg),
                "plus": {
                    "state": _state_snapshot(plus_condition, plus_controls),
                    "coefficients": _plain_coefficients(plus_payload["coefficients"]),
                },
                "minus": {
                    "state": _state_snapshot(minus_condition, minus_controls),
                    "coefficients": _plain_coefficients(minus_payload["coefficients"]),
                },
            }
        for row in derivative_rows:
            name = str(row["name"])
            coefficient = str(row["coefficient"])
            values = {
                step: centered_derivative(
                    sample["plus"]["coefficients"][coefficient],
                    sample["minus"]["coefficients"][coefficient],
                    step,
                )
                for step, sample in sample_outputs.items()
            }
            selection = select_fd_step(
                values,
                convergence_settings,
                preferred_step=preferred_steps.get(name),
            )
            samples = {
                _step_key(step): {**sample, "derivative_value": values[step]}
                for step, sample in sample_outputs.items()
            }
            production_records[name] = {
                **{key: value for key, value in row.items()},
                "derivative_value": selection["derivative_value"],
                "source": "production_centered_fd",
                "method": "centered_finite_difference",
                "selected_fd_step": selection["selected_fd_step"],
                "selected_fd_step_unit": "deg",
                "convergence_status": selection["status"],
                "validation_status": selection["status"],
                "production_included": selection["status"] != "FAIL",
                "wake_level": derivative_config.get("_bundle_wake_iterations"),
                "coordinate_sign_convention": _coordinate_sign_convention(),
                "base_state": _state_snapshot(condition, base_controls),
                "samples": samples,
                "convergence": selection,
            }

    items = build_required_manifest_items(
        manifest=manifest,
        production_records=production_records,
        native_records=native_records,
        wake_level=derivative_config.get("_bundle_wake_iterations"),
    )
    return {
        "production_fd_derivatives": production_records,
        "native_derivative_diagnostics": native_records,
        "required_derivatives_manifest": {
            "items": items,
            "summary": required_derivative_summary(items, manifest),
        },
        "run_count": run_count,
        "solver_duration_sec": float(solver_duration),
        "bundle_wake_iterations": derivative_config.get("_bundle_wake_iterations"),
        "bundle_rule": "base and every +/- perturbation use one fixed production Wake level",
    }
