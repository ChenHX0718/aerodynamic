from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


@dataclass(frozen=True)
class TrimResult:
    converged: bool
    alpha_deg: float
    elevator_deg: float
    force_residual_n: float
    moment_residual_nm: float
    iterations: int
    failure_reason: str | None
    history: tuple[dict[str, float], ...]
    analysis_payload: dict[str, Any]


def _value(payload: dict[str, Any], category: str, name: str) -> float:
    return float(payload[category][name]["standard_value"])


def solve_longitudinal_trim(
    *,
    evaluate: Callable[[float, float, int], dict[str, Any]],
    trim_config: dict[str, Any],
    dynamic_pressure_pa: float,
    reference: dict[str, float],
) -> TrimResult:
    """Bounded Newton solve using VSPAERO's local alpha and elevator derivatives."""
    alpha_cfg = trim_config["alpha"]
    elevator_cfg = trim_config["elevator"]
    alpha = float(alpha_cfg["initial_deg"])
    elevator = float(elevator_cfg["initial_deg"])
    alpha_bounds = (float(alpha_cfg["min_deg"]), float(alpha_cfg["max_deg"]))
    elevator_bounds = (float(elevator_cfg["min_deg"]), float(elevator_cfg["max_deg"]))
    max_iterations = int(trim_config["max_iterations"])
    force_tolerance = float(trim_config["force_tolerance_n"])
    moment_tolerance = float(trim_config["moment_tolerance_nm"])
    max_step_deg = float(trim_config.get("max_step_deg", 5.0))
    weight = float(trim_config["mass_kg"]) * float(trim_config["gravity_m_s2"])
    q_s = dynamic_pressure_pa * float(reference["sref_m2"])
    q_s_c = q_s * float(reference["cref_m"])
    history: list[dict[str, float]] = []
    payload: dict[str, Any] = {}
    failure: str | None = None
    force_residual = math.nan
    moment_residual = math.nan

    for iteration in range(1, max_iterations + 1):
        payload = evaluate(alpha, elevator, iteration)
        stability = payload["stability_derivatives"]
        elevator_derivatives = payload["control_derivatives"]["elevator"]["derivatives"]
        cl = _value(payload, "coefficients", "CL")
        cm = _value(payload, "coefficients", "Cm")
        force_residual = q_s * cl - weight
        moment_residual = q_s_c * cm
        history.append(
            {
                "iteration": float(iteration), "alpha_deg": alpha, "elevator_deg": elevator,
                "CL": cl, "Cm": cm, "force_residual_n": force_residual,
                "moment_residual_nm": moment_residual,
            }
        )
        if abs(force_residual) <= force_tolerance and abs(moment_residual) <= moment_tolerance:
            return TrimResult(
                True, alpha, elevator, force_residual, moment_residual, iteration,
                None, tuple(history), payload,
            )
        if any(name not in stability for name in ("CL_alpha", "Cm_alpha")):
            failure = "VSPAERO stability output lacks CL_alpha or Cm_alpha"
            break
        if "CL_delta_e" not in elevator_derivatives or "Cm_delta_e" not in elevator_derivatives:
            failure = "VSPAERO stability output lacks elevator lift or pitch derivative"
            break
        jacobian = np.array(
            [
                [q_s * float(stability["CL_alpha"]["standard_value"]),
                 q_s * float(elevator_derivatives["CL_delta_e"]["standard_value"])],
                [q_s_c * float(stability["Cm_alpha"]["standard_value"]),
                 q_s_c * float(elevator_derivatives["Cm_delta_e"]["standard_value"])],
            ], dtype=float,
        )
        if not np.all(np.isfinite(jacobian)) or np.linalg.cond(jacobian) > 1.0e10:
            failure = "Trim Jacobian is singular or ill-conditioned"
            break
        step_rad = np.linalg.solve(jacobian, -np.array([force_residual, moment_residual]))
        step_deg = np.degrees(step_rad)
        largest = max(abs(float(step_deg[0])), abs(float(step_deg[1])))
        if largest > max_step_deg:
            step_deg *= max_step_deg / largest
        next_alpha = float(np.clip(alpha + step_deg[0], *alpha_bounds))
        next_elevator = float(np.clip(elevator + step_deg[1], *elevator_bounds))
        if math.isclose(next_alpha, alpha, abs_tol=1.0e-10) and math.isclose(
            next_elevator, elevator, abs_tol=1.0e-10
        ):
            failure = "Trim update is blocked by the configured alpha/elevator bounds"
            break
        alpha, elevator = next_alpha, next_elevator

    if failure is None:
        failure = f"Trim did not meet both residual tolerances in {max_iterations} iterations"
    return TrimResult(
        False, alpha, elevator, force_residual, moment_residual, len(history),
        failure, tuple(history), payload,
    )
