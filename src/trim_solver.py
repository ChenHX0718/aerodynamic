from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np

from finite_difference import centered_derivative


@dataclass(frozen=True)
class TrimResult:
    converged: bool
    alpha_deg: float
    elevator_deg: float
    force_residual_n: float
    moment_residual_nm: float
    iterations: int
    failure_reason: str | None
    history: tuple[dict[str, Any], ...]
    analysis_payload: dict[str, Any]


def _value(payload: dict[str, Any], category: str, name: str) -> float:
    return float(payload[category][name]["standard_value"])


def _native_derivative(payload: dict[str, Any], name: str) -> float | None:
    diagnostics = payload.get("native_derivative_diagnostics", {})
    stability = diagnostics.get("stability", {})
    if name in stability:
        return float(stability[name]["standard_value"])
    elevator = diagnostics.get("controls", {}).get("elevator", {}).get("derivatives", {})
    if name in elevator:
        return float(elevator[name]["standard_value"])
    return None


def solve_longitudinal_trim(
    *,
    evaluate: Callable[[float, float, str, bool], dict[str, Any]],
    trim_config: dict[str, Any],
    derivative_config: dict[str, Any],
    dynamic_pressure_pa: float,
    reference: dict[str, float],
) -> TrimResult:
    """Bounded Newton trim with a real centered-difference Jacobian and backtracking."""
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
    jacobian_steps = derivative_config["trim_jacobian_steps_deg"]
    alpha_step = float(jacobian_steps["alpha"])
    elevator_step = float(jacobian_steps["elevator"])
    line_search_lambdas = (1.0, 0.5, 0.25, 0.125)
    weight = float(trim_config["mass_kg"]) * float(trim_config["gravity_m_s2"])
    q_s = dynamic_pressure_pa * float(reference["sref_m2"])
    q_s_c = q_s * float(reference["cref_m"])
    history: list[dict[str, Any]] = []
    payload: dict[str, Any] = {}
    carried_payload: dict[str, Any] | None = None
    failure: str | None = None
    force_residual = math.nan
    moment_residual = math.nan

    def residuals(analysis: dict[str, Any]) -> tuple[float, float, float, float, float]:
        cl = _value(analysis, "coefficients", "CL")
        cm = _value(analysis, "coefficients", "Cm")
        force = q_s * cl - weight
        moment = q_s_c * cm
        normalized = max(abs(force) / force_tolerance, abs(moment) / moment_tolerance)
        return cl, cm, force, moment, normalized

    for iteration in range(1, max_iterations + 1):
        prefix = f"iter_{iteration:02d}"
        if carried_payload is None:
            payload = evaluate(alpha, elevator, f"{prefix}_base", True)
        else:
            payload = carried_payload
            carried_payload = None
        cl, cm, force_residual, moment_residual, residual_norm = residuals(payload)
        record: dict[str, Any] = {
            "iteration": iteration,
            "alpha_deg": alpha,
            "elevator_deg": elevator,
            "CL": cl,
            "Cm": cm,
            "force_residual_n": force_residual,
            "moment_residual_nm": moment_residual,
            "residual_norm": residual_norm,
            "native_derivative_diagnostics": {
                name: _native_derivative(payload, name)
                for name in ("CL_alpha", "Cm_alpha", "CL_delta_e", "Cm_delta_e")
            },
        }
        history.append(record)
        if abs(force_residual) <= force_tolerance and abs(moment_residual) <= moment_tolerance:
            record["decision"] = "PASS_BOTH_RESIDUALS"
            return TrimResult(
                True, alpha, elevator, force_residual, moment_residual, iteration,
                None, tuple(history), payload,
            )
        if iteration == max_iterations:
            record["decision"] = "FAIL_MAX_ITERATIONS"
            break
        if not (
            alpha_bounds[0] <= alpha - alpha_step <= alpha + alpha_step <= alpha_bounds[1]
            and elevator_bounds[0]
            <= elevator - elevator_step
            <= elevator + elevator_step
            <= elevator_bounds[1]
        ):
            failure = "Centered-difference Trim Jacobian perturbation is blocked by configured bounds"
            record["decision"] = "FAIL_JACOBIAN_BOUNDS"
            break

        alpha_plus = evaluate(alpha + alpha_step, elevator, f"{prefix}_fd_alpha_plus", False)
        alpha_minus = evaluate(alpha - alpha_step, elevator, f"{prefix}_fd_alpha_minus", False)
        elevator_plus = evaluate(
            alpha, elevator + elevator_step, f"{prefix}_fd_elevator_plus", False
        )
        elevator_minus = evaluate(
            alpha, elevator - elevator_step, f"{prefix}_fd_elevator_minus", False
        )
        fd_derivatives = {
            "CL_alpha": centered_derivative(
                _value(alpha_plus, "coefficients", "CL"),
                _value(alpha_minus, "coefficients", "CL"),
                alpha_step,
            ),
            "Cm_alpha": centered_derivative(
                _value(alpha_plus, "coefficients", "Cm"),
                _value(alpha_minus, "coefficients", "Cm"),
                alpha_step,
            ),
            "CL_delta_e": centered_derivative(
                _value(elevator_plus, "coefficients", "CL"),
                _value(elevator_minus, "coefficients", "CL"),
                elevator_step,
            ),
            "Cm_delta_e": centered_derivative(
                _value(elevator_plus, "coefficients", "Cm"),
                _value(elevator_minus, "coefficients", "Cm"),
                elevator_step,
            ),
        }
        jacobian = np.array(
            [
                [q_s * fd_derivatives["CL_alpha"], q_s * fd_derivatives["CL_delta_e"]],
                [q_s_c * fd_derivatives["Cm_alpha"], q_s_c * fd_derivatives["Cm_delta_e"]],
            ],
            dtype=float,
        )
        record["fd_derivatives"] = fd_derivatives
        record["jacobian"] = jacobian.tolist()
        if not np.all(np.isfinite(jacobian)):
            failure = "Centered-difference Trim Jacobian contains a non-finite value"
            record["decision"] = "FAIL_JACOBIAN"
            break
        condition_number = float(np.linalg.cond(jacobian))
        record["jacobian_condition_number"] = condition_number
        if not math.isfinite(condition_number) or condition_number > 1.0e10:
            failure = "Centered-difference Trim Jacobian is singular or ill-conditioned"
            record["decision"] = "FAIL_JACOBIAN"
            break

        raw_step_deg = np.degrees(
            np.linalg.solve(jacobian, -np.array([force_residual, moment_residual]))
        )
        limited_step_deg = raw_step_deg.copy()
        largest = max(abs(float(limited_step_deg[0])), abs(float(limited_step_deg[1])))
        if largest > max_step_deg:
            limited_step_deg *= max_step_deg / largest
        record["raw_newton_step_deg"] = {
            "alpha": float(raw_step_deg[0]), "elevator": float(raw_step_deg[1])
        }
        record["limited_newton_step_deg"] = {
            "alpha": float(limited_step_deg[0]), "elevator": float(limited_step_deg[1])
        }

        trials: list[dict[str, Any]] = []
        accepted: tuple[float, float, dict[str, Any], float, float, float] | None = None
        for attempt, step_lambda in enumerate(line_search_lambdas, 1):
            next_alpha = float(np.clip(alpha + step_lambda * limited_step_deg[0], *alpha_bounds))
            next_elevator = float(
                np.clip(elevator + step_lambda * limited_step_deg[1], *elevator_bounds)
            )
            if math.isclose(next_alpha, alpha, abs_tol=1.0e-12) and math.isclose(
                next_elevator, elevator, abs_tol=1.0e-12
            ):
                trials.append({"lambda": step_lambda, "status": "BLOCKED_BY_BOUNDS"})
                continue
            trial_payload = evaluate(
                next_alpha, next_elevator, f"{prefix}_line_search_{attempt:02d}", True
            )
            _, _, trial_force, trial_moment, trial_norm = residuals(trial_payload)
            improved = trial_norm < residual_norm
            trials.append(
                {
                    "lambda": step_lambda,
                    "alpha_deg": next_alpha,
                    "elevator_deg": next_elevator,
                    "force_residual_n": trial_force,
                    "moment_residual_nm": trial_moment,
                    "residual_norm": trial_norm,
                    "improved": improved,
                }
            )
            if improved:
                accepted = (
                    next_alpha, next_elevator, trial_payload,
                    trial_force, trial_moment, trial_norm,
                )
                break

        record["line_search_trials"] = trials
        record["line_search_attempts"] = len(trials)
        if accepted is None:
            failure = "Line search failed: no candidate reduced the normalized residual; state retained"
            record["decision"] = "FAIL_LINE_SEARCH_NO_IMPROVEMENT"
            break
        next_alpha, next_elevator, carried_payload, new_force, new_moment, new_norm = accepted
        final_lambda = float(trials[-1]["lambda"])
        record["line_search_lambda"] = final_lambda
        record["applied_step_deg"] = {
            "alpha": next_alpha - alpha, "elevator": next_elevator - elevator
        }
        record["new_force_residual_n"] = new_force
        record["new_moment_residual_nm"] = new_moment
        record["new_residual_norm"] = new_norm
        record["decision"] = "ACCEPT_NEWTON_STEP" if final_lambda == 1.0 else "ACCEPT_BACKTRACKED_STEP"
        alpha, elevator = next_alpha, next_elevator

    if failure is None:
        failure = f"Trim did not meet both residual tolerances in {max_iterations} iterations"
    return TrimResult(
        False, alpha, elevator, force_residual, moment_residual, len(history),
        failure, tuple(history), payload,
    )
