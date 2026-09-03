from __future__ import annotations

import math
from typing import Any, Callable

from coordinate_system import nondimensional_rate_step


STATUS_RANK = {"PASS": 0, "WARN": 1, "FAIL": 2}


def combine_status(statuses: list[str]) -> str:
    normalized = [str(item).upper() for item in statuses if str(item).upper() in STATUS_RANK]
    return max(normalized, key=STATUS_RANK.get) if normalized else "PASS"


def dual_tolerance_result(reference: float, candidate: float, settings: dict[str, Any]) -> dict[str, Any]:
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
    required = ("0.5", "1", "2")
    if any(key not in values or not math.isfinite(float(values[key])) for key in required):
        return {
            "status": "FAIL", "reason": "one or more finite-difference scales are missing or non-finite",
            "values": values,
        }
    nominal = float(values["1"])
    comparisons = [
        dual_tolerance_result(nominal, float(values[key]), settings) for key in ("0.5", "2")
    ]
    worst = max(comparisons, key=lambda item: (STATUS_RANK[item["status"]], item["difference"]))
    max_change = max(float(item["difference"]) for item in comparisons)
    relative = max(float(item["relative_difference"]) for item in comparisons)
    status = combine_status([item["status"] for item in comparisons])
    if status == "PASS":
        reason = "step variation is within PASS absolute+relative tolerance"
    elif status == "WARN":
        reason = "step variation exceeds PASS but is within WARN tolerance"
    else:
        reason = "step variation exceeds WARN absolute+relative tolerance"
    return {
        "status": status,
        "reason": reason,
        "values": {key: float(values[key]) for key in required},
        "max_absolute_change": float(max_change),
        "relative_variation": float(relative),
        "relative_variation_pct": float(relative * 100.0),
        "pass_limit": float(worst["pass_limit"]),
        "warn_limit": float(worst["warn_limit"]),
        "near_zero_reference": float(settings["near_zero_reference"]),
    }


def perturb_state(
    condition: dict[str, float], controls: dict[str, float], variable: str, offset_deg: float,
    control_config: dict[str, Any] | None = None,
) -> tuple[dict[str, float], dict[str, float]]:
    """Return one aerodynamic state perturbed in the project's standard degree convention."""
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
    """Centered derivative per radian for a symmetric perturbation expressed in degrees."""
    step = float(step_deg)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError(f"Centered-difference step must be positive, got {step_deg}")
    return (float(plus) - float(minus)) / (2.0 * math.radians(step))


def _plain_coefficients(mapped: dict[str, dict[str, Any]]) -> dict[str, float]:
    return {name: float(item["standard_value"]) for name, item in mapped.items()}


def _scale_key(scale: float) -> str:
    if math.isclose(scale, 1.0):
        return "1"
    return f"{scale:g}"


def _state_snapshot(
    condition: dict[str, float], controls: dict[str, float], *,
    vspaero_vinf_mps: float | None = None,
    rate_variable: str | None = None,
    rate_rad_s: float = 0.0,
) -> dict[str, Any]:
    state = {
        "speed_mps": float(condition["speed_mps"]),
        "vspaero_vinf_mps": float(vspaero_vinf_mps or condition["speed_mps"]),
        "alpha_deg": float(condition["alpha_deg"]),
        "beta_deg": float(condition["beta_deg"]),
        "p_rad_s": 0.0,
        "q_rad_s": 0.0,
        "r_rad_s": 0.0,
        "aileron_deg": float(controls["aileron"]),
        "elevator_deg": float(controls["elevator"]),
        "rudder_deg": float(controls["rudder"]),
    }
    if rate_variable in {"p", "q", "r"}:
        state[f"{rate_variable}_rad_s"] = float(rate_rad_s)
    return state


def calculate_trim_derivatives(
    *,
    condition: dict[str, float],
    base_outputs: dict[str, Any],
    base_controls: dict[str, float],
    manifest: dict[str, Any],
    derivative_config: dict[str, Any],
    validation_config: dict[str, Any],
    reference: dict[str, float],
    run_polar: Callable[[dict[str, float], str, dict[str, float]], dict[str, Any]],
    run_stability: Callable[[dict[str, float], str, dict[str, float]], dict[str, Any]],
) -> dict[str, Any]:
    """Generate the authoritative trim-point derivative set with one reusable engine."""
    scales = [float(item) for item in derivative_config["scales"]]
    perturbations = derivative_config["perturbations"]
    convergence_settings = derivative_config["convergence"]
    rows = list(manifest["derivatives"])
    base_coefficients = _plain_coefficients(base_outputs["coefficients"])
    records: dict[str, dict[str, Any]] = {}
    run_count = 0
    solver_duration = 0.0

    central_variables = ("alpha", "beta", "aileron", "elevator", "rudder")
    for variable in central_variables:
        derivative_rows = [row for row in rows if row["perturbation"] == variable]
        if not derivative_rows:
            continue
        base_step = float(perturbations[f"{variable}_deg"])
        sample_outputs: dict[str, dict[str, Any]] = {}
        for scale in scales:
            key = _scale_key(scale)
            step_deg = base_step * scale
            control_cfg = derivative_config["_controls"].get(variable)
            plus_condition, plus_controls = perturb_state(
                condition, base_controls, variable, step_deg, control_cfg
            )
            minus_condition, minus_controls = perturb_state(
                condition, base_controls, variable, -step_deg, control_cfg
            )
            plus_payload = run_polar(plus_condition, f"fd_{variable}_{key}_plus", plus_controls)
            minus_payload = run_polar(minus_condition, f"fd_{variable}_{key}_minus", minus_controls)
            run_count += 2
            solver_duration += float(plus_payload.get("solver_duration_sec", 0.0))
            solver_duration += float(minus_payload.get("solver_duration_sec", 0.0))
            plus = _plain_coefficients(plus_payload["coefficients"])
            minus = _plain_coefficients(minus_payload["coefficients"])
            sample_outputs[key] = {
                "step": float(step_deg),
                "step_unit": "deg",
                "derivative_denominator": float(2.0 * math.radians(step_deg)),
                "derivative_denominator_unit": "rad",
                "plus": {"state": _state_snapshot(plus_condition, plus_controls), "coefficients": plus},
                "minus": {"state": _state_snapshot(minus_condition, minus_controls), "coefficients": minus},
            }
        for row in derivative_rows:
            name = str(row["name"])
            coefficient = str(row["coefficient"])
            samples: dict[str, Any] = {}
            values: dict[str, float] = {}
            for key, sample in sample_outputs.items():
                value = centered_derivative(
                    float(sample["plus"]["coefficients"][coefficient]),
                    float(sample["minus"]["coefficients"][coefficient]),
                    float(sample["step"]),
                )
                values[key] = value
                samples[key] = {**sample, "derivative": float(value)}
            convergence = convergence_result(values, convergence_settings)
            native = _native_value(base_outputs, name)
            records[name] = {
                **{key: value for key, value in row.items()},
                "value": float(values["1"]),
                "method": "centered_finite_difference",
                "method_status": "PASS",
                "base_state": _state_snapshot(condition, base_controls),
                "base_coefficient": float(base_coefficients[coefficient]),
                "native_vspaero_value": native,
                "samples": samples,
                "convergence": convergence,
                "validation_status": convergence["status"],
            }

    # OpenVSP 3.51.3 exposes normalized p/q/r derivatives but no negative-rate
    # steady Sweep input.  The three effective steps below are real solver runs;
    # only the unavailable negative side is left explicit instead of fabricated.
    rate_payloads: dict[str, dict[str, Any]] = {"1": base_outputs}
    speed = float(condition["speed_mps"])
    for scale in scales:
        key = _scale_key(scale)
        if key == "1":
            continue
        rate_condition = dict(condition)
        rate_condition["_vspaero_vinf_mps"] = speed / scale
        payload = run_stability(rate_condition, f"rate_{key}", base_controls)
        rate_payloads[key] = payload
        run_count += 1
        solver_duration += float(payload.get("solver_run", {}).get("duration_sec", 0.0))
    method_status = str(validation_config.get("rate_derivative_method_status", "WARN")).upper()
    for variable in ("p", "q", "r"):
        derivative_rows = [row for row in rows if row["perturbation"] == variable]
        if not derivative_rows:
            continue
        base_rate = float(perturbations[f"{variable}_rad_s"])
        for row in derivative_rows:
            name = str(row["name"])
            coefficient = str(row["coefficient"])
            values: dict[str, float] = {}
            samples: dict[str, Any] = {}
            for scale in scales:
                key = _scale_key(scale)
                payload = rate_payloads[key]
                value = _native_value(payload, name)
                if value is None:
                    value = math.nan
                values[key] = float(value)
                effective_vinf = speed / scale
                step_hat = nondimensional_rate_step(variable, base_rate, effective_vinf, reference)
                plus_coefficients = payload.get("native_rate_cases", {}).get(variable, {})
                scale_base_coefficients = _plain_coefficients(payload["coefficients"])
                samples[key] = {
                    "step": float(base_rate),
                    "step_unit": "rad/s",
                    "derivative_denominator": float(step_hat),
                    "derivative_denominator_unit": f"{variable}_hat",
                    "plus": {
                        "state": _state_snapshot(
                            condition, base_controls, vspaero_vinf_mps=effective_vinf,
                            rate_variable=variable, rate_rad_s=base_rate,
                        ),
                        "coefficients": _plain_coefficients(plus_coefficients) if plus_coefficients else {},
                        "source": "VSPAERO native positive rate case",
                    },
                    "minus": None,
                    "scale_base": {
                        "state": _state_snapshot(
                            condition, base_controls, vspaero_vinf_mps=effective_vinf,
                        ),
                        "coefficients": scale_base_coefficients,
                    },
                    "derivative": float(value),
                }
            convergence = convergence_result(values, convergence_settings)
            status = combine_status([convergence["status"], method_status])
            records[name] = {
                **{key: value for key, value in row.items()},
                "value": float(values["1"]),
                "method": "vspaero_native_forward_rate",
                "method_status": method_status,
                "method_limitation": (
                    "OpenVSP 3.51.3 public VSPAEROSweep API has no negative p/q/r single-point input; "
                    "three real positive-rate step sizes are checked, and minus samples remain null."
                ),
                "base_state": _state_snapshot(condition, base_controls),
                "base_coefficient": float(base_coefficients[coefficient]),
                "samples": samples,
                "convergence": convergence,
                "validation_status": status,
            }

    return {
        "records": records,
        "run_count": run_count,
        "solver_duration_sec": float(solver_duration),
        "bundle_wake_iterations": derivative_config.get("_bundle_wake_iterations"),
        "bundle_rule": "base and every +/- perturbation use one fixed Wake Iteration",
    }


def _native_value(payload: dict[str, Any], name: str) -> float | None:
    stability = payload.get("stability_derivatives", {})
    if name in stability:
        return float(stability[name]["standard_value"])
    for control in payload.get("control_derivatives", {}).values():
        derivatives = control.get("derivatives", {})
        if name in derivatives:
            return float(derivatives[name]["standard_value"])
    return None
