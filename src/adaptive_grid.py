from __future__ import annotations

import csv
import json
import math
from itertools import product
from pathlib import Path
from typing import Any, Callable, Iterable

from finite_difference import STATUS_RANK, combine_status, dual_tolerance_result


AXES = ("speed_mps", "alpha_deg", "beta_deg")
AXIS_CONFIG_NAMES = {
    "speed_mps": "speed_mps",
    "alpha_deg": "alpha_deg",
    "beta_deg": "beta_deg",
}


def point_key(point: dict[str, float]) -> tuple[float, float, float]:
    return tuple(float(point[name]) for name in AXES)


def midpoint(bounds: dict[str, tuple[float, float]]) -> dict[str, float]:
    return {name: 0.5 * (float(bounds[name][0]) + float(bounds[name][1])) for name in AXES}


def cell_corners(bounds: dict[str, tuple[float, float]]) -> list[dict[str, float]]:
    choices = [
        (float(bounds[name][0]),)
        if math.isclose(float(bounds[name][0]), float(bounds[name][1]), abs_tol=1.0e-12)
        else (float(bounds[name][0]), float(bounds[name][1]))
        for name in AXES
    ]
    return [dict(zip(AXES, values)) for values in product(*choices)]


def multilinear_midpoint(
    corner_values: Iterable[dict[str, float]], quantities: Iterable[str]
) -> dict[str, float]:
    corners = list(corner_values)
    if not corners:
        raise ValueError("At least one corner value is required")
    return {
        name: sum(float(item[name]) for item in corners) / len(corners)
        for name in quantities
    }


def interpolation_error(
    predicted: dict[str, float],
    actual: dict[str, float],
    quantities: Iterable[str],
    tolerance: dict[str, Any],
) -> dict[str, Any]:
    checks: dict[str, Any] = {}
    for name in quantities:
        comparison = dual_tolerance_result(float(predicted[name]), float(actual[name]), tolerance)
        status = "WARN" if comparison["status"] == "WARN_NUMERICAL" else comparison["status"]
        checks[name] = {
            "interpolated": float(predicted[name]),
            "actual": float(actual[name]),
            **comparison,
            "status": status,
        }
    return {
        "status": combine_status([str(item["status"]) for item in checks.values()]),
        "quantities": checks,
    }


def _cell_bounds(
    axes: dict[str, list[float]], indices: tuple[int, ...], active: list[str]
) -> dict[str, tuple[float, float]]:
    index_by_axis = dict(zip(active, indices))
    return {
        name: (
            (float(axes[name][index_by_axis[name]]), float(axes[name][index_by_axis[name] + 1]))
            if name in index_by_axis else (float(axes[name][0]), float(axes[name][0]))
        )
        for name in AXES
    }


def seed_cells(axes: dict[str, list[float]]) -> tuple[list[str], list[dict[str, tuple[float, float]]]]:
    active = [name for name in AXES if len(axes[name]) > 1]
    if not active:
        return active, []
    ranges = [range(len(axes[name]) - 1) for name in active]
    return active, [_cell_bounds(axes, indices, active) for indices in product(*ranges)]


def choose_split_axis(
    bounds: dict[str, tuple[float, float]],
    active: list[str],
    domain_span: dict[str, float],
    min_spacing: dict[str, float],
) -> str | None:
    eligible = [
        name for name in active
        if 0.5 * (float(bounds[name][1]) - float(bounds[name][0]))
        >= float(min_spacing[name]) - 1.0e-12
    ]
    if not eligible:
        return None
    order = {name: index for index, name in enumerate(AXES)}
    return min(
        eligible,
        key=lambda name: (
            -(float(bounds[name][1]) - float(bounds[name][0])) / domain_span[name],
            order[name],
        ),
    )


def split_cell(
    bounds: dict[str, tuple[float, float]], axis: str
) -> tuple[dict[str, tuple[float, float]], dict[str, tuple[float, float]]]:
    lower, upper = bounds[axis]
    center = 0.5 * (float(lower) + float(upper))
    first = dict(bounds)
    second = dict(bounds)
    first[axis] = (float(lower), center)
    second[axis] = (center, float(upper))
    return first, second


def _coefficient_values(result: dict[str, Any], quantities: list[str]) -> dict[str, float]:
    if result.get("status") != "PASS":
        raise ValueError(str(result.get("error", "GRID solver point did not PASS")))
    coefficients = result.get("outputs", {}).get("coefficients", {})
    values = {
        name: float(coefficients[name]["standard_value"])
        for name in quantities
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError("GRID solver returned a non-finite adaptive quantity")
    return values


def run_adaptive_grid(
    *,
    axes: dict[str, list[float]],
    settings: dict[str, Any],
    evaluator: Callable[[dict[str, float], str], tuple[dict[str, Any], bool]],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Run deterministic local bisection over a 1D/2D/3D seed grid."""
    normalized_axes = {name: sorted(float(value) for value in axes[name]) for name in AXES}
    quantities = [str(name) for name in settings["quantities"]]
    tolerance = settings["tolerance"]
    max_depth = int(settings["max_depth"])
    max_cases = int(settings["max_cases"])
    min_spacing = {
        name: float(settings["min_spacing"][AXIS_CONFIG_NAMES[name]]) for name in AXES
    }
    active, initial_bounds = seed_cells(normalized_axes)
    domain_span = {
        name: float(normalized_axes[name][-1] - normalized_axes[name][0])
        for name in active
    }
    initial_points = [
        dict(zip(AXES, values))
        for values in product(*(normalized_axes[name] for name in AXES))
    ]
    if max_cases < len(initial_points):
        raise ValueError(
            f"grid.adaptive.max_cases={max_cases} is smaller than the "
            f"{len(initial_points)}-point seed grid"
        )

    evaluated: dict[tuple[float, float, float], dict[str, Any]] = {}
    persistent_cache_hits = 0
    deduplicated_reuses = 0
    new_solver_runs = 0
    new_solver_duration_sec = 0.0

    def evaluate(point: dict[str, float], source: str) -> dict[str, Any]:
        nonlocal persistent_cache_hits, deduplicated_reuses, new_solver_runs
        nonlocal new_solver_duration_sec
        key = point_key(point)
        if key in evaluated:
            deduplicated_reuses += 1
            return evaluated[key]
        if len(evaluated) >= max_cases:
            raise RuntimeError("adaptive max_cases reached before a required point could be evaluated")
        result, cache_hit = evaluator(dict(zip(AXES, key)), source)
        result["grid_source"] = "seed" if source == "seed" else "adaptive_midpoint"
        evaluated[key] = result
        if cache_hit:
            persistent_cache_hits += 1
        else:
            new_solver_runs += 1
            new_solver_duration_sec += float(result.get("solver", {}).get("duration_sec", 0.0))
        return result

    for point in initial_points:
        evaluate(point, "seed")

    if not active:
        only = next(iter(evaluated.values()))
        status = "PASS" if only.get("status") == "PASS" else "FAIL"
        report = {
            "mode": "adaptive",
            "active_dimensions": [],
            "initial_seed_point_count": 1,
            "initial_cell_count": 0,
            "final_unique_point_count": 1,
            "new_solver_runs": new_solver_runs,
            "new_solver_duration_sec": new_solver_duration_sec,
            "persistent_cache_hits": persistent_cache_hits,
            "deduplicated_point_reuses": deduplicated_reuses,
            "cache_hits": persistent_cache_hits + deduplicated_reuses,
            "accepted_cell_count": 0,
            "refined_cell_count": 0,
            "max_depth_reached": 0,
            "max_interpolation_error": 0.0,
            "per_coefficient_worst_error": {},
            "termination_reason": "single seed point; no active dimensions",
            "status": status,
            "cells": [],
            "refinement_history": [],
        }
        return list(evaluated.values()), report

    next_id = 1

    def cell(bounds: dict[str, tuple[float, float]], depth: int, parent: str | None) -> dict[str, Any]:
        nonlocal next_id
        item = {
            "cell_id": f"cell_{next_id:06d}",
            "parent_cell_id": parent,
            "depth": int(depth),
            "bounds": {name: [float(value[0]), float(value[1])] for name, value in bounds.items()},
        }
        next_id += 1
        return item

    queue = [cell(bounds, 0, None) for bounds in initial_bounds]
    final_cells: list[dict[str, Any]] = []
    history: list[dict[str, Any]] = []
    worst: dict[str, dict[str, Any]] = {}
    limit_reasons: set[str] = set()
    max_depth_reached = 0

    while queue:
        current = queue.pop(0)
        depth = int(current["depth"])
        max_depth_reached = max(max_depth_reached, depth)
        bounds = {name: tuple(current["bounds"][name]) for name in AXES}
        corners = cell_corners(bounds)
        center = midpoint(bounds)
        try:
            corner_results = [evaluate(point, "adaptive") for point in corners]
            center_result = evaluate(center, "adaptive")
            prediction = multilinear_midpoint(
                [_coefficient_values(item, quantities) for item in corner_results], quantities
            )
            actual = _coefficient_values(center_result, quantities)
            error = interpolation_error(prediction, actual, quantities, tolerance)
            status = str(error["status"])
            for name, check in error["quantities"].items():
                candidate = {
                    "absolute_error": float(check["difference"]),
                    "relative_error": float(check["relative_difference"]),
                    "status": str(check["status"]),
                    "cell_id": current["cell_id"],
                }
                previous = worst.get(name)
                if previous is None or (
                    STATUS_RANK[candidate["status"]], candidate["absolute_error"]
                ) > (
                    STATUS_RANK[previous["status"]], previous["absolute_error"]
                ):
                    worst[name] = candidate
            current["midpoint"] = center
            current["interpolation"] = error
        except Exception as exc:
            status = "FAIL"
            current["midpoint"] = center
            current["interpolation"] = {
                "status": "FAIL", "quantities": {},
                "reason": f"{type(exc).__name__}: {exc}",
            }

        current["status"] = status
        if status == "PASS":
            current["termination_reason"] = "interpolation tolerance satisfied"
            final_cells.append(current)
            continue

        if depth >= max_depth:
            current["termination_reason"] = "max_depth"
            limit_reasons.add("max_depth")
            final_cells.append(current)
            continue
        split_axis = choose_split_axis(bounds, active, domain_span, min_spacing)
        if split_axis is None:
            current["termination_reason"] = "min_spacing"
            limit_reasons.add("min_spacing")
            final_cells.append(current)
            continue
        child_bounds = split_cell(bounds, split_axis)
        required_points = {
            point_key(point)
            for child in child_bounds
            for point in [*cell_corners(child), midpoint(child)]
            if point_key(point) not in evaluated
        }
        if len(evaluated) + len(required_points) > max_cases:
            current["termination_reason"] = "max_cases"
            limit_reasons.add("max_cases")
            final_cells.append(current)
            continue
        children = [cell(child, depth + 1, current["cell_id"]) for child in child_bounds]
        history.append({
            "parent_cell_id": current["cell_id"],
            "depth": depth,
            "status_before_refine": status,
            "split_axis": split_axis,
            "split_value": midpoint(bounds)[split_axis],
            "child_cell_ids": [item["cell_id"] for item in children],
            "parent_bounds": current["bounds"],
        })
        queue.extend(children)

    final_status = combine_status([str(item["status"]) for item in final_cells])
    max_error = max(
        (float(item["absolute_error"]) for item in worst.values()), default=0.0
    )
    termination = (
        "all final cells satisfy interpolation tolerance"
        if final_status == "PASS"
        else "limits reached: " + ", ".join(sorted(limit_reasons))
        if limit_reasons
        else "one or more solver/interpolation evaluations failed"
    )
    report = {
        "mode": "adaptive",
        "active_dimensions": active,
        "initial_seed_point_count": len(initial_points),
        "initial_cell_count": len(initial_bounds),
        "final_unique_point_count": len(evaluated),
        "new_solver_runs": new_solver_runs,
        "new_solver_duration_sec": new_solver_duration_sec,
        "persistent_cache_hits": persistent_cache_hits,
        "deduplicated_point_reuses": deduplicated_reuses,
        "cache_hits": persistent_cache_hits + deduplicated_reuses,
        "accepted_cell_count": sum(item["status"] == "PASS" for item in final_cells),
        "refined_cell_count": len(history),
        "max_depth_reached": max_depth_reached,
        "max_interpolation_error": max_error,
        "per_coefficient_worst_error": worst,
        "termination_reason": termination,
        "status": final_status,
        "cells": final_cells,
        "refinement_history": history,
    }
    results = sorted(evaluated.values(), key=lambda item: point_key(item["inputs"]))
    return results, report


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def write_adaptive_report(output_dir: Path, report: dict[str, Any], results: list[dict[str, Any]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "adaptive_grid_report.json").write_text(
        json.dumps(_json_safe(report), ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    point_rows = []
    for result in results:
        coefficients = result.get("outputs", {}).get("coefficients", {})
        point_rows.append({
            "case_id": result.get("case_id"),
            "speed_mps": result.get("inputs", {}).get("speed_mps"),
            "alpha_deg": result.get("inputs", {}).get("alpha_deg"),
            "beta_deg": result.get("inputs", {}).get("beta_deg"),
            "grid_source": result.get("grid_source"),
            "status": result.get("status"),
            **{
                name: item.get("standard_value")
                for name, item in coefficients.items()
                if name in {"CL", "CD", "Cm", "CY", "Cl", "Cn"}
            },
        })
    _write_csv(output_dir / "adaptive_grid_points.csv", point_rows)
    _write_csv(output_dir / "adaptive_grid_cells.csv", [
        {
            "cell_id": item["cell_id"],
            "parent_cell_id": item.get("parent_cell_id"),
            "depth": item["depth"],
            "status": item["status"],
            "termination_reason": item.get("termination_reason"),
            **{
                f"{axis}_{side}": item["bounds"][axis][index]
                for axis in AXES for side, index in (("min", 0), ("max", 1))
            },
        }
        for item in report["cells"]
    ])


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8-sig") as stream:
        if not fields:
            return
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
