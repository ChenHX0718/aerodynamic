from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Callable


COEFFICIENTS = ("CL", "CD", "CY", "Cl", "Cm", "Cn")
CONTROL_VARIABLES = ("elevator", "aileron", "rudder")
RATE_VARIABLES = ("p", "q", "r")


@dataclass(frozen=True)
class ResponseCaseSpec:
    case_id: str
    base_grid_case_id: str
    variable: str
    perturbation_value: float
    perturbation_unit: str = "deg"

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def response_case_id(base_case_id: str, variable: str, value: float) -> str:
    scaled = int(round(abs(float(value)) * 1000.0))
    sign = "P" if float(value) >= 0.0 else "M"
    return f"RESP_{base_case_id}_{variable.upper()}_{sign}{scaled:09d}"


def _point(result: dict[str, Any]) -> tuple[float, float, float]:
    inputs = result["inputs"]
    return (
        float(inputs["speed_mps"]),
        float(inputs["alpha_deg"]),
        float(inputs["beta_deg"]),
    )


def _automatic_representatives(results: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Choose deterministic low/mid/high-alpha/beta audit points without using TRIM."""
    if not results:
        return []
    ordered = sorted(results, key=_point)
    speeds = sorted({_point(item)[0] for item in ordered})
    alphas = sorted({_point(item)[1] for item in ordered})
    mid_speed = speeds[len(speeds) // 2]
    mid_alpha = alphas[len(alphas) // 2]

    def nearest(target: tuple[float, float, float]) -> dict[str, Any]:
        spans = (
            max(speeds[-1] - speeds[0], 1.0),
            max(alphas[-1] - alphas[0], 1.0),
            max(max(abs(_point(item)[2]) for item in ordered), 1.0),
        )
        return min(
            ordered,
            key=lambda item: (
                sum(
                    ((value - target[index]) / spans[index]) ** 2
                    for index, value in enumerate(_point(item))
                ),
                _point(item),
            ),
        )

    targets = [
        (speeds[0], alphas[0], 0.0),
        (mid_speed, mid_alpha, 0.0),
        (speeds[-1], mid_alpha, 0.0),
        (mid_speed, alphas[-1], 0.0),
        (mid_speed, mid_alpha, max((_point(item)[2] for item in ordered), key=abs)),
    ]
    selected: list[dict[str, Any]] = []
    seen: set[str] = set()
    for target in targets:
        item = nearest(target)
        case_id = str(item["case_id"])
        if case_id not in seen:
            selected.append(item)
            seen.add(case_id)
    return selected


def select_response_bases(
    grid_results: list[dict[str, Any]], settings: dict[str, Any]
) -> list[dict[str, Any]]:
    passed = [item for item in grid_results if item.get("status") == "PASS"]
    if str(settings["scope"]).lower() == "full_grid":
        return sorted(passed, key=_point)
    requested = settings.get("representative_states") or []
    if not requested:
        return _automatic_representatives(passed)
    selected: list[dict[str, Any]] = []
    seen: set[str] = set()
    for state in requested:
        target = (
            float(state["speed_mps"]),
            float(state["alpha_deg"]),
            float(state.get("beta_deg", 0.0)),
        )
        exact = [item for item in passed if all(
            math.isclose(value, target[index], abs_tol=1.0e-9)
            for index, value in enumerate(_point(item))
        )]
        if not exact:
            raise ValueError(
                "response_scan representative state is not an accepted GRID point: "
                f"V={target[0]:g}, alpha={target[1]:g}, beta={target[2]:g}"
            )
        case_id = str(exact[0]["case_id"])
        if case_id not in seen:
            selected.append(exact[0])
            seen.add(case_id)
    return selected


def generate_control_scan_cases(
    bases: list[dict[str, Any]], settings: dict[str, Any]
) -> list[ResponseCaseSpec]:
    cases: list[ResponseCaseSpec] = []
    variables = set(str(item) for item in settings["variables"])
    for base in bases:
        for variable in CONTROL_VARIABLES:
            if variable not in variables:
                continue
            for value in settings["control_levels_deg"][variable]:
                cases.append(ResponseCaseSpec(
                    case_id=response_case_id(str(base["case_id"]), variable, float(value)),
                    base_grid_case_id=str(base["case_id"]),
                    variable=variable,
                    perturbation_value=float(value),
                ))
    return cases


def _coefficients(result: dict[str, Any]) -> dict[str, float]:
    mapped = result.get("outputs", {}).get("coefficients", {})
    values = {name: float(mapped[name]["standard_value"]) for name in COEFFICIENTS}
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("response coefficient is non-finite")
    return values


def _sample(
    spec: ResponseCaseSpec,
    base: dict[str, Any],
    coefficients: dict[str, float],
    solver: dict[str, Any],
    *,
    neutral_deg: float,
    cache_hit: bool,
    source: str,
) -> dict[str, Any]:
    base_values = _coefficients(base)
    inputs = base["inputs"]
    return {
        **spec.as_dict(),
        "speed_mps": float(inputs["speed_mps"]),
        "alpha_deg": float(inputs["alpha_deg"]),
        "beta_deg": float(inputs["beta_deg"]),
        "neutral_value": float(neutral_deg),
        "offset_from_neutral_deg": float(spec.perturbation_value - neutral_deg),
        "coefficients": dict(coefficients),
        "delta_coefficients": {
            name: float(coefficients[name] - base_values[name]) for name in COEFFICIENTS
        },
        "wake_iterations": solver.get("wake_iterations"),
        "solver_status": solver.get("solver_status", "SUCCESS"),
        "solver_duration_sec": float(solver.get("solver_duration_sec", 0.0)),
        "raw_directory": solver.get("raw_directory"),
        "status": "PASS",
        "cache_hit": bool(cache_hit),
        "source": source,
    }


def collect_control_responses(
    *,
    bases: list[dict[str, Any]],
    settings: dict[str, Any],
    controls: dict[str, Any],
    evaluator: Callable[[ResponseCaseSpec, dict[str, Any]], tuple[dict[str, Any], bool]],
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    by_id = {str(item["case_id"]): item for item in bases}
    samples: list[dict[str, Any]] = []
    cache_hits = 0
    solver_runs = 0
    baseline_reuses = 0
    for spec in generate_control_scan_cases(bases, settings):
        base = by_id[spec.base_grid_case_id]
        neutral = float(controls[spec.variable].get("neutral_deg", 0.0))
        if math.isclose(spec.perturbation_value, neutral, abs_tol=1.0e-12):
            samples.append(_sample(
                spec, base, _coefficients(base), {
                    "wake_iterations": base.get("numerical_settings", {}).get("wake_iterations"),
                    "solver_status": base.get("solver", {}).get("status", "SUCCESS"),
                }, neutral_deg=neutral, cache_hit=True, source="base_grid_reuse",
            ))
            baseline_reuses += 1
            continue
        evaluated, cached = evaluator(spec, base)
        if evaluated.get("status", "PASS") != "PASS":
            inputs = base["inputs"]
            samples.append({
                **spec.as_dict(),
                "speed_mps": float(inputs["speed_mps"]),
                "alpha_deg": float(inputs["alpha_deg"]),
                "beta_deg": float(inputs["beta_deg"]),
                "neutral_value": neutral,
                "offset_from_neutral_deg": float(spec.perturbation_value - neutral),
                "coefficients": {},
                "delta_coefficients": {},
                "wake_iterations": evaluated.get("wake_iterations"),
                "solver_status": evaluated.get("solver_status", "FAIL"),
                "solver_duration_sec": float(evaluated.get("solver_duration_sec", 0.0)),
                "raw_directory": evaluated.get("raw_directory"),
                "status": "FAIL",
                "cache_hit": bool(cached),
                "source": "failed_response_case",
                "error": evaluated.get("error", "response evaluator failed"),
            })
            cache_hits += int(cached)
            solver_runs += int(not cached)
            continue
        samples.append(_sample(
            spec, base, {name: float(evaluated["coefficients"][name]) for name in COEFFICIENTS},
            evaluated, neutral_deg=neutral, cache_hit=cached,
            source="response_cache" if cached else "vspaero_polar",
        ))
        cache_hits += int(cached)
        solver_runs += int(not cached)
    return samples, {
        "baseline_reuses": baseline_reuses,
        "persistent_cache_hits": cache_hits,
        "new_solver_runs": solver_runs,
    }


def _linear_fit(xs: list[float], ys: list[float]) -> tuple[float, float, list[float], float]:
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    denominator = sum((value - x_mean) ** 2 for value in xs)
    if denominator <= 0.0:
        raise ValueError("response variable has no span")
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denominator
    intercept = y_mean - slope * x_mean
    residuals = [y - (intercept + slope * x) for x, y in zip(xs, ys)]
    sse = sum(value * value for value in residuals)
    sst = sum((value - y_mean) ** 2 for value in ys)
    r_squared = 1.0 if sst <= 1.0e-30 and sse <= 1.0e-30 else 0.0 if sst <= 1.0e-30 else 1.0 - sse / sst
    return slope, intercept, residuals, r_squared


def audit_response_curve(
    samples: list[dict[str, Any]], coefficient: str, settings: dict[str, Any]
) -> dict[str, Any]:
    ordered = sorted(samples, key=lambda item: float(item["offset_from_neutral_deg"]))
    minimum = int(settings["minimum_samples"])
    if len(ordered) < minimum:
        return {
            "classification": "INSUFFICIENT_DATA",
            "recommended_representation": "response_table",
            "reason": f"{len(ordered)} samples are fewer than configured minimum {minimum}",
        }
    xs = [math.radians(float(item["offset_from_neutral_deg"])) for item in ordered]
    ys = [float(item["coefficients"][coefficient]) for item in ordered]
    deltas = [float(item["delta_coefficients"][coefficient]) for item in ordered]
    negative = [(x, y) for x, y in zip(xs, ys) if x < 0.0]
    positive = [(x, y) for x, y in zip(xs, ys) if x > 0.0]
    if not negative or not positive:
        return {
            "classification": "INSUFFICIENT_DATA",
            "recommended_representation": "response_table",
            "reason": "centered samples on both sides of the neutral point are required",
        }
    minus_x, minus_y = max(negative, key=lambda item: item[0])
    plus_x, plus_y = min(positive, key=lambda item: item[0])
    centered_slope = (plus_y - minus_y) / (plus_x - minus_x)
    fit_slope, fit_intercept, residuals, r_squared = _linear_fit(xs, ys)
    response_range = max(ys) - min(ys)
    signal_floor = float(settings["minimum_response_range"])
    scale = max(response_range, max(abs(value) for value in deltas), signal_floor)
    normalized_deviation = max(abs(value) for value in residuals) / scale
    local_slopes = [
        (right_y - left_y) / (right_x - left_x)
        for (left_x, left_y), (right_x, right_y) in zip(zip(xs, ys), zip(xs[1:], ys[1:]))
        if right_x > left_x
    ]
    slope_scale = max(abs(fit_slope), float(settings["slope_floor"]))
    slope_variation = max((abs(value - fit_slope) for value in local_slopes), default=math.inf) / slope_scale
    pairs = []
    for magnitude in sorted({round(abs(x), 12) for x in xs if x != 0.0}):
        negative_delta = next((delta for x, delta in zip(xs, deltas) if round(x, 12) == -magnitude), None)
        positive_delta = next((delta for x, delta in zip(xs, deltas) if round(x, 12) == magnitude), None)
        if negative_delta is not None and positive_delta is not None:
            pairs.append(abs(float(positive_delta) + float(negative_delta)) / scale)
    asymmetry = max(pairs, default=math.nan)
    metrics = {
        "normalized_max_deviation": normalized_deviation,
        "slope_variation": slope_variation,
        "r_squared": r_squared,
        "asymmetry": asymmetry,
    }

    def within(thresholds: dict[str, Any]) -> bool:
        return (
            normalized_deviation <= float(thresholds["normalized_max_deviation"])
            and slope_variation <= float(thresholds["slope_variation"])
            and r_squared >= float(thresholds["r_squared_min"])
            and (math.isnan(asymmetry) or asymmetry <= float(thresholds["asymmetry"]))
        )

    if response_range < signal_floor:
        classification = "INSUFFICIENT_DATA"
        reason = "response range is below the configured resolvable signal floor"
    elif within(settings["linear"]):
        classification = "LINEAR"
        reason = "all configured linearity metrics satisfy the LINEAR thresholds"
    elif within(settings["weakly_nonlinear"]):
        classification = "WEAKLY_NONLINEAR"
        reason = "metrics exceed LINEAR but satisfy WEAKLY_NONLINEAR thresholds"
    else:
        classification = "NONLINEAR"
        reason = "one or more metrics exceed the WEAKLY_NONLINEAR thresholds"
    return {
        "classification": classification,
        "recommended_representation": "derivative" if classification == "LINEAR" else "response_table",
        "centered_slope_per_rad": centered_slope,
        "global_fit_slope_per_rad": fit_slope,
        "global_fit_intercept": fit_intercept,
        "maximum_absolute_residual": max(abs(value) for value in residuals),
        "response_range": response_range,
        **metrics,
        "sample_count": len(ordered),
        "reason": reason,
    }


def audit_control_responses(
    samples: list[dict[str, Any]], settings: dict[str, Any]
) -> list[dict[str, Any]]:
    groups: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for sample in samples:
        if sample.get("status") != "PASS":
            continue
        groups.setdefault(
            (str(sample["base_grid_case_id"]), str(sample["variable"])), []
        ).append(sample)
    rows: list[dict[str, Any]] = []
    for (base_case_id, variable), group in sorted(groups.items()):
        first = group[0]
        for coefficient in COEFFICIENTS:
            rows.append({
                "base_grid_case_id": base_case_id,
                "speed_mps": first["speed_mps"],
                "alpha_deg": first["alpha_deg"],
                "beta_deg": first["beta_deg"],
                "variable": variable,
                "coefficient": coefficient,
                **audit_response_curve(group, coefficient, settings),
            })
    return rows


def collect_rate_derivatives(
    bases: list[dict[str, Any]], settings: dict[str, Any]
) -> dict[str, list[dict[str, Any]]]:
    requested = set(str(item) for item in settings["variables"])
    output: dict[str, list[dict[str, Any]]] = {name: [] for name in RATE_VARIABLES if name in requested}
    for base in bases:
        inputs = base["inputs"]
        stability = base.get("outputs", {}).get("native_derivative_diagnostics", {}).get("stability", {})
        for variable in output:
            derivatives: dict[str, Any] = {}
            missing: list[str] = []
            for coefficient in COEFFICIENTS:
                record = stability.get(f"{coefficient}_{variable}")
                try:
                    value = float(record["standard_value"])
                except (KeyError, TypeError, ValueError):
                    value = math.nan
                if not isinstance(record, dict) or not math.isfinite(value):
                    missing.append(coefficient)
                else:
                    derivatives[coefficient] = {
                        "value": value,
                        "unit": str(record["standard_unit"]),
                        "source_field": str(record["raw_field"]),
                    }
            output[variable].append({
                "base_grid_case_id": str(base["case_id"]),
                "speed_mps": float(inputs["speed_mps"]),
                "alpha_deg": float(inputs["alpha_deg"]),
                "beta_deg": float(inputs["beta_deg"]),
                "representation": "local_linear_derivative",
                "normalization": {
                    "p": "p_hat=p*bref/(2*V)",
                    "q": "q_hat=q*cref/(2*V)",
                    "r": "r_hat=r*bref/(2*V)",
                }[variable],
                "source": "VSPAERO steady .stab",
                "derivatives": derivatives,
                "status": "PASS" if not missing else "FAIL",
                "missing_coefficients": missing,
            })
    return output


def response_gate(
    *, bases: list[dict[str, Any]], samples: list[dict[str, Any]],
    rates: dict[str, list[dict[str, Any]]], settings: dict[str, Any],
) -> dict[str, Any]:
    if not bases:
        return {"status": "FAIL", "reason": "no accepted GRID point was selected for response scan"}
    expected_control = len(generate_control_scan_cases(bases, settings))
    sample_failures = [item for item in samples if item.get("status") != "PASS"]
    rate_failures = [item for rows in rates.values() for item in rows if item.get("status") != "PASS"]
    if len(samples) != expected_control:
        return {
            "status": "FAIL",
            "reason": f"response sample set is incomplete: {len(samples)}/{expected_control}",
        }
    if sample_failures or rate_failures:
        return {
            "status": "FAIL",
            "reason": (
                f"{len(sample_failures)} control samples and {len(rate_failures)} local rate fields failed"
            ),
        }
    return {
        "status": "PASS",
        "reason": (
            f"{len(samples)} control samples and "
            f"{sum(len(rows) for rows in rates.values())} local rate fields are complete"
        ),
    }


def run_response_scan(
    *, grid_results: list[dict[str, Any]], settings: dict[str, Any],
    controls: dict[str, Any],
    evaluator: Callable[[ResponseCaseSpec, dict[str, Any]], tuple[dict[str, Any], bool]],
) -> dict[str, Any]:
    bases = select_response_bases(grid_results, settings)
    samples, cache = collect_control_responses(
        bases=bases, settings=settings, controls=controls, evaluator=evaluator,
    )
    linearity = audit_control_responses(samples, settings["linearity"])
    rates = collect_rate_derivatives(bases, settings)
    gate = response_gate(bases=bases, samples=samples, rates=rates, settings=settings)
    return {
        "enabled": True,
        "scope": str(settings["scope"]),
        "selected_base_case_ids": [str(item["case_id"]) for item in bases],
        "controls": {
            variable: [item for item in samples if item["variable"] == variable]
            for variable in CONTROL_VARIABLES
            if variable in set(str(item) for item in settings["variables"])
        },
        "rates": rates,
        "linearity": linearity,
        "gate": gate,
        "cache": cache,
        "assumptions": {
            "additive_response_assumption": True,
            "baseline_requires_trim": False,
            "rate_representation": "local_linear_derivative",
            "arbitrary_independent_rate_sweep_supported": False,
            "rate_capability_reason": (
                "OpenVSP 3.51.3 VSPAEROSweep exposes UnsteadyType but no independent "
                "P/Q/R value inputs; verified steady .stab derivatives are used"
            ),
            "ignored_cross_terms": [
                "q_hat*elevator", "p_hat*aileron", "elevator*aileron", "q_hat*r_hat",
            ],
        },
    }
