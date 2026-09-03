from __future__ import annotations

import json
import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from scipy.io import loadmat


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from case_generator import expand_axis, generate_grid_cases, generate_trim_cases
from config_loader import load_project_config
from coordinate_system import map_polar_coefficients
from export_results import _aero_mat, flatten_case
from finite_difference import (
    calculate_trim_derivatives,
    convergence_result,
    dual_tolerance_result,
    select_fd_step,
)
from numerical_convergence import (
    WAKE_FD_NAMES,
    boundary_continuity_result,
    derivative_bundle_wake,
    midpoint_interpolation_error,
    minimum_converged_level,
    production_gate,
    promote_discrete_level,
    query_wake_schedule,
    run_numerical_convergence,
    terminal_wake_verification,
    trim_wake_decision,
)
from trim_solver import solve_longitudinal_trim


class ConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = load_project_config(PROJECT_ROOT / "config" / "aircraft.yaml")

    def test_inclusive_range(self) -> None:
        self.assertEqual(
            expand_axis({"start": 8, "step": 2, "end": 22}, "speed"),
            [8, 10, 12, 14, 16, 18, 20, 22],
        )

    def test_smoke_case_counts_and_safe_ids(self) -> None:
        grid = generate_grid_cases(self.config)
        trim = generate_trim_cases(self.config)
        self.assertEqual(len(grid), 4)
        self.assertEqual(len(trim), 1)
        for case in grid + trim:
            self.assertRegex(case.case_id, r"^[A-Z0-9_]+$")

    def test_authoritative_manifest_has_23_unique_required_derivatives(self) -> None:
        manifest = self.config["_manifest"]
        names = [row["name"] for row in manifest["derivatives"]]
        self.assertEqual(len(names), 23)
        self.assertEqual(len(set(names)), 23)
        self.assertEqual(set(names), {row["name"] for row in manifest["_required"]})

    def test_regression_fixture_is_fixed_to_delivered_aircraft(self) -> None:
        fixture = load_project_config(PROJECT_ROOT / "tests" / "regression" / "regression.yaml")
        self.assertEqual(fixture["_paths"]["model"], PROJECT_ROOT / "aircraft" / "test_aircraft.vsp3")
        self.assertEqual(fixture["regression"]["speed_mps"], 8)


class ConventionAndNumericsTests(unittest.TestCase):
    def test_coordinate_sign_conversion_is_centralized(self) -> None:
        raw = {
            "CFxtot": 1.0, "CFytot": 2.0, "CFztot": 3.0,
            "CLtot": 4.0, "CDtot": 5.0,
            "CMxtot": 6.0, "CMytot": 7.0, "CMztot": 8.0,
        }
        mapped = map_polar_coefficients(raw)
        values = {name: item["standard_value"] for name, item in mapped.items()}
        self.assertEqual(values, {
            "CX": -1.0, "CY": 2.0, "CZ": -3.0,
            "CL": 4.0, "CD": 5.0, "Cl": -6.0, "Cm": 7.0, "Cn": -8.0,
        })

    def test_convergence_uses_absolute_plus_relative_tolerance(self) -> None:
        settings = {
            "near_zero_reference": 0.05,
            "pass_relative": 0.10, "warn_relative": 0.25,
            "pass_absolute": 0.02, "warn_absolute": 0.08,
        }
        passed = convergence_result({"0.5": 1.02, "1": 1.0, "2": 0.98}, settings)
        failed = convergence_result({"0.5": 1.5, "1": 1.0, "2": 0.5}, settings)
        self.assertEqual(passed["status"], "PASS")
        self.assertEqual(failed["status"], "FAIL")

    def test_native_diagnostic_does_not_overwrite_production_fd(self) -> None:
        config = deepcopy(load_project_config(PROJECT_ROOT / "config" / "aircraft.yaml"))
        controls = {"aileron": 0.0, "elevator": 0.0, "rudder": 0.0}
        condition = {"speed_mps": 8.0, "alpha_deg": 0.0, "beta_deg": 0.0}
        base = {
            "coefficients": {
                name: {"standard_value": 0.0}
                for name in ("CL", "CD", "CY", "Cl", "Cm", "Cn")
            },
            "native_derivative_diagnostics": {
                "stability": {"CL_alpha": {"standard_value": 999.0}},
                "controls": {},
            },
        }

        def run_polar(state, _label, deflections):
            alpha = math.radians(state["alpha_deg"])
            beta = math.radians(state["beta_deg"])
            aileron = math.radians(deflections["aileron"])
            elevator = math.radians(deflections["elevator"])
            rudder = math.radians(deflections["rudder"])
            values = {
                "CL": alpha + 0.2 * elevator,
                "CD": 0.1 * alpha + 0.01 * elevator,
                "CY": -0.3 * beta + 0.1 * aileron - 0.4 * rudder,
                "Cl": -0.1 * beta - 0.2 * aileron + 0.02 * rudder,
                "Cm": -0.5 * alpha - elevator,
                "Cn": 0.2 * beta + 0.01 * aileron + 0.2 * rudder,
            }
            return {
                "coefficients": {
                    name: {"standard_value": value} for name, value in values.items()
                },
                "solver_duration_sec": 0.0,
            }

        derivative_config = deepcopy(config["derivatives"])
        derivative_config["_controls"] = config["controls"]
        derivative_config["_bundle_wake_iterations"] = 12
        package = calculate_trim_derivatives(
            condition=condition, base_outputs=base, base_controls=controls,
            manifest=config["_manifest"], derivative_config=derivative_config,
            run_polar=run_polar,
        )
        production = package["production_fd_derivatives"]["CL_alpha"]
        native = package["native_derivative_diagnostics"]["CL_alpha"]
        self.assertAlmostEqual(production["derivative_value"], 1.0)
        self.assertEqual(native["diagnostic_value"], 999.0)
        self.assertFalse(native["enters_production_gate"])
        summary = package["required_derivatives_manifest"]["summary"]
        self.assertEqual(summary["required"], 23)
        self.assertEqual(summary["PASS"], 15)
        self.assertEqual(summary["METHOD_LIMITATION"], 8)
        self.assertEqual(summary["FAIL"], 0)

        trim_result = {
            "case_id": "synthetic_trim",
            "mode": "TRIM_DATABASE",
            "status": "WARN",
            "inputs": {
                "speed_mps": 8.0, "rho_kg_m3": 1.225,
                "dynamic_pressure_pa": 39.2, "mach": 0.02,
                "reynolds_cref": 1.0e6,
            },
            "outputs": base,
            "trim": {
                "alpha_trim_deg": 0.0, "beta_trim_deg": 0.0,
                "elevator_trim_deg": 0.0, "CL_trim": 0.0, "CD_trim": 0.0,
                "CY_trim": 0.0, "Cl_trim": 0.0, "Cm_trim": 0.0, "Cn_trim": 0.0,
                "trim_force_residual_n": 0.0, "trim_moment_residual_nm": 0.0,
                "trim_iterations": 1,
            },
            "derivatives": package,
            "validation": {
                "overall_status": "WARN", "trim_status": "PASS",
                "numerical_status": "PASS", "derivative_status": "WARN",
                "physics_status": "PASS",
            },
            "solver": {"duration_sec": 0.0},
        }
        flat = flatten_case(trim_result)
        self.assertAlmostEqual(flat["production_fd_CL_alpha"], 1.0)
        self.assertEqual(flat["native_diagnostic_CL_alpha"], 999.0)
        mat = _aero_mat({
            "metadata": {
                "aircraft_name": "synthetic", "generated_at_local": "now",
                "openvsp_version": "OpenVSP 3.51.3", "solver": "VSPAERO",
                "model_sha256": "synthetic", "coordinate_system": {},
                "production_numerical_settings": {"production_gate": {"status": "WARN"}},
            },
            "manifest": config["_manifest"],
            "reference": {
                "sref_m2": 1.0, "bref_m": 1.0, "cref_m": 1.0,
                "xcg_m": 0.0, "ycg_m": 0.0, "zcg_m": 0.0,
            },
        }, [trim_result])
        self.assertIn("CL_alpha", mat["longitudinal"])
        self.assertNotIn("Cl_p", mat["lateral"])
        self.assertIn("Cl_p", mat["native_derivative_diagnostics"])


class TrimSolverTests(unittest.TestCase):
    @staticmethod
    def trim_config(**overrides) -> dict:
        config = {
            "alpha": {"initial_deg": 0.0, "min_deg": -120.0, "max_deg": 120.0},
            "elevator": {"initial_deg": 0.0, "min_deg": -30.0, "max_deg": 30.0},
            "max_iterations": 15, "force_tolerance_n": 1.0e-6,
            "moment_tolerance_nm": 1.0e-6, "max_step_deg": 90.0,
            "mass_kg": 0.0, "gravity_m_s2": 1.0,
        }
        config.update(overrides)
        return config

    @staticmethod
    def derivatives(alpha_step: float = 0.1, elevator_step: float = 0.1) -> dict:
        return {"trim_jacobian_steps_deg": {"alpha": alpha_step, "elevator": elevator_step}}

    @staticmethod
    def payload(cl: float, cm: float, stability: bool, native_scale: float = 1.0) -> dict:
        result = {
            "coefficients": {
                "CL": {"standard_value": cl}, "Cm": {"standard_value": cm},
            }
        }
        if stability:
            result["native_derivative_diagnostics"] = {
                "stability": {
                    "CL_alpha": {"standard_value": 99.0 * native_scale},
                    "Cm_alpha": {"standard_value": -99.0 * native_scale},
                },
                "controls": {"elevator": {"derivatives": {
                    "CL_delta_e": {"standard_value": 55.0 * native_scale},
                    "Cm_delta_e": {"standard_value": -55.0 * native_scale},
                }}},
            }
        return result

    def test_centered_jacobian_converges_and_native_is_diagnostic_only(self) -> None:
        config = self.trim_config(
            alpha={"initial_deg": 2.0, "min_deg": -20.0, "max_deg": 20.0},
            elevator={"initial_deg": 0.0, "min_deg": -20.0, "max_deg": 20.0},
            mass_kg=1.0, gravity_m_s2=10.0,
        )

        def evaluate(alpha_deg, elevator_deg, _label, stability):
            alpha = math.radians(alpha_deg)
            elevator = math.radians(elevator_deg)
            return self.payload(
                0.05 + alpha + 0.2 * elevator,
                0.01 - 0.5 * alpha - elevator,
                stability,
            )

        result = solve_longitudinal_trim(
            evaluate=evaluate, trim_config=config, derivative_config=self.derivatives(),
            dynamic_pressure_pa=10.0, reference={"sref_m2": 10.0, "cref_m": 1.0},
        )
        self.assertTrue(result.converged)
        self.assertEqual(result.iterations, 2)
        self.assertEqual(result.history[0]["line_search_lambda"], 1.0)
        self.assertAlmostEqual(result.history[0]["fd_derivatives"]["CL_alpha"], 1.0, places=9)
        self.assertEqual(result.history[0]["native_derivative_diagnostics"]["CL_alpha"], 99.0)

    def test_backtracking_uses_first_improving_candidate(self) -> None:
        def evaluate(alpha_deg, elevator_deg, _label, stability):
            alpha = math.radians(alpha_deg)
            elevator = math.radians(elevator_deg)
            return self.payload(alpha ** 3 - 2.0 * alpha + 2.0, elevator, stability)

        result = solve_longitudinal_trim(
            evaluate=evaluate, trim_config=self.trim_config(),
            derivative_config=self.derivatives(0.01, 0.1),
            dynamic_pressure_pa=1.0, reference={"sref_m2": 1.0, "cref_m": 1.0},
        )
        backtracked = [
            row for row in result.history if float(row.get("line_search_lambda", 1.0)) < 1.0
        ]
        self.assertTrue(backtracked)
        self.assertEqual(backtracked[0]["line_search_lambda"], 0.25)
        self.assertEqual(backtracked[0]["line_search_attempts"], 3)
        self.assertLess(backtracked[0]["new_residual_norm"], backtracked[0]["residual_norm"])

    def test_iteration_cap_is_not_convergence(self) -> None:
        config = self.trim_config(
            mass_kg=2.0, max_step_deg=5.0,
            force_tolerance_n=1.0e-9, moment_tolerance_nm=1.0e-9,
        )

        def evaluate(alpha_deg, elevator_deg, _label, stability):
            return self.payload(math.radians(alpha_deg), math.radians(elevator_deg), stability)

        result = solve_longitudinal_trim(
            evaluate=evaluate, trim_config=config, derivative_config=self.derivatives(),
            dynamic_pressure_pa=1.0, reference={"sref_m2": 1.0, "cref_m": 1.0},
        )
        self.assertFalse(result.converged)
        self.assertEqual(result.iterations, 15)
        self.assertEqual(result.history[-1]["decision"], "FAIL_MAX_ITERATIONS")
        self.assertIn("15 iterations", result.failure_reason)


class NumericalConvergenceLogicTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tolerance = {
            "near_zero_reference": 0.05,
            "pass_relative": 0.05, "warn_relative": 0.15,
            "pass_absolute": 0.01, "warn_absolute": 0.03,
        }
        self.derivatives = {
            "scales": [0.5, 1.0, 2.0],
            "perturbations": {"alpha_deg": 0.5, "beta_deg": 0.5},
        }

    @staticmethod
    def schedule(*, safety: int = 0, buffer: float = 0.12) -> dict:
        return {
            "candidates": [3, 5, 8, 12],
            "axis_scales": {"speed_mps": 1.0, "alpha_deg": 10.0, "beta_deg": 1.0},
            "neighbor_count": 4,
            "boundary_buffer_normalized": buffer,
            "safety_margin_levels_for_untested": safety,
            "sample_points": [
                {"name": "low", "speed_mps": 8.0, "alpha_deg": 0.0, "beta_deg": 0.0, "required_wake": 5},
                {"name": "high", "speed_mps": 8.0, "alpha_deg": 10.0, "beta_deg": 0.0, "required_wake": 8},
            ],
        }

    def test_wake_candidates_are_centralized(self) -> None:
        config = load_project_config(PROJECT_ROOT / "config" / "aircraft.yaml")
        self.assertEqual(config["numerical_convergence"]["wake"]["candidates"], [3, 5, 8, 12])
        self.assertEqual(config["numerical_convergence"]["wake"]["verification_only_level"], 16)
        self.assertNotIn("tessellation", config["numerical_convergence"])
        self.assertNotIn("tessellation_overrides", config["solver"])
        self.assertEqual(len(WAKE_FD_NAMES), 15)
        self.assertEqual(set(WAKE_FD_NAMES), {
            row["name"] for row in config["_manifest"]["_required"]
            if row["perturbation"] in {"alpha", "beta", "aileron", "elevator", "rudder"}
        })

    def test_minimum_wake_requires_every_later_transition(self) -> None:
        values = {3: {"CL": 1.00}, 5: {"CL": 1.005}, 8: {"CL": 1.20}, 12: {"CL": 1.201}}
        result = minimum_converged_level(
            [3, 5, 8, 12], values, ["CL"], self.tolerance,
            allow_unverified_highest=True,
        )
        self.assertEqual(result["required_level"], 8)
        self.assertEqual(result["status"], "PASS")

    def test_highest_wake_is_not_assumed_converged(self) -> None:
        values = {3: {"CL": 1.0}, 5: {"CL": 1.2}, 8: {"CL": 1.4}, 12: {"CL": 1.8}}
        result = minimum_converged_level(
            [3, 5, 8, 12], values, ["CL"], self.tolerance,
            allow_unverified_highest=True,
        )
        self.assertEqual(result["required_level"], 12)
        self.assertEqual(result["status"], "FAIL")

    def test_wake16_is_verification_only_and_keeps_production_at_12(self) -> None:
        values = {
            3: {"CL": 1.0}, 5: {"CL": 1.1},
            8: {"CL": 1.20}, 12: {"CL": 1.35},
        }
        pending = terminal_wake_verification(
            production_levels=[3, 5, 8, 12], production_values=values,
            quantities=["CL"], settings=self.tolerance, verification_level=16,
        )
        self.assertTrue(pending["verification"]["triggered"])
        verified = terminal_wake_verification(
            production_levels=[3, 5, 8, 12], production_values=values,
            quantities=["CL"], settings=self.tolerance, verification_level=16,
            verification_values={"CL": 1.355},
        )
        self.assertEqual(verified["status"], "PASS")
        self.assertEqual(verified["required_level"], 12)
        self.assertTrue(verified["verification"]["production_level_unchanged"])

    def test_wake16_not_triggered_after_passed_8_to_12(self) -> None:
        result = terminal_wake_verification(
            production_levels=[3, 5, 8, 12],
            production_values={
                3: {"CL": 1.0}, 5: {"CL": 1.1},
                8: {"CL": 1.2}, 12: {"CL": 1.205},
            },
            quantities=["CL"], settings=self.tolerance, verification_level=16,
        )
        self.assertFalse(result["verification"]["triggered"])

    def test_wake16_nonpass_stays_warn_and_never_promotes_production(self) -> None:
        values = {
            3: {"CL": 1.0}, 5: {"CL": 1.1},
            8: {"CL": 1.20}, 12: {"CL": 1.35},
        }
        result = terminal_wake_verification(
            production_levels=[3, 5, 8, 12], production_values=values,
            quantities=["CL"], settings=self.tolerance, verification_level=16,
            verification_values={"CL": 1.50},
        )
        self.assertEqual(result["status"], "WARN")
        self.assertEqual(result["required_level"], 12)
        self.assertTrue(result["verification"]["production_level_unchanged"])

    def test_dual_tolerance_handles_near_zero(self) -> None:
        result = dual_tolerance_result(0.0, 0.005, self.tolerance)
        self.assertEqual(result["status"], "PASS")

    def test_safety_margin_and_discrete_map_query(self) -> None:
        self.assertEqual(promote_discrete_level(5, [3, 5, 8, 12], 1), 8)
        direct = query_wake_schedule(self.schedule(safety=1), {"speed_mps": 8, "alpha_deg": 0, "beta_deg": 0})
        boundary = query_wake_schedule(self.schedule(safety=1), {"speed_mps": 8, "alpha_deg": 5, "beta_deg": 0})
        self.assertEqual(direct["wake_iterations"], 5)
        self.assertEqual(boundary["wake_iterations"], 12)
        self.assertEqual(boundary["source"], "conservative_discrete_neighbors")

    def test_derivative_bundle_uses_maximum_wake(self) -> None:
        wake = derivative_bundle_wake(
            self.schedule(safety=0),
            {"speed_mps": 8, "alpha_deg": 1, "beta_deg": 0},
            {"fd_step_candidates_deg": {"alpha": [1, 5, 10], "beta": [0.1, 0.25, 0.5]}},
        )
        self.assertEqual(wake, 8)

    def test_pretrim_selection_and_cross_region_upgrade(self) -> None:
        no_perturbation = {"fd_step_candidates_deg": {"alpha": [0, 0, 0], "beta": [0, 0, 0]}}
        start = trim_wake_decision(
            self.schedule(), {"speed_mps": 8, "alpha_deg": 0, "beta_deg": 0}, no_perturbation
        )
        crossed = trim_wake_decision(
            self.schedule(), {"speed_mps": 8, "alpha_deg": 10, "beta_deg": 0},
            no_perturbation, current_wake=5,
        )
        self.assertEqual(start["action"], "START_PRODUCTION")
        self.assertEqual(crossed["action"], "UPGRADE_AND_RETRIM")
        self.assertEqual(crossed["wake_iterations"], 8)

    def test_boundary_continuity(self) -> None:
        passed = boundary_continuity_result({"CL": 1.0}, {"CL": 1.005}, ["CL"], self.tolerance)
        failed = boundary_continuity_result({"CL": 1.0}, {"CL": 1.3}, ["CL"], self.tolerance)
        self.assertEqual(passed["status"], "PASS")
        self.assertEqual(failed["status"], "FAIL")

    def test_fd_step_selection_is_independent_per_derivative(self) -> None:
        stable_small = select_fd_step(
            {0.5: 1.0, 1.0: 1.002, 2.0: 1.5}, self.tolerance
        )
        noisy_small = select_fd_step(
            {0.5: 0.5, 1.0: 1.0, 2.0: 1.002}, self.tolerance
        )
        self.assertEqual(stable_small["selected_fd_step"], 1.0)
        self.assertEqual(noisy_small["selected_fd_step"], 2.0)

    def test_production_gate_and_adaptive_interface(self) -> None:
        settings = {"production_gate": {"status": "WARN", "reason": "review"}}
        self.assertTrue(production_gate(settings)["allowed"])
        self.assertFalse(production_gate(settings, adaptive=True)["allowed"])
        self.assertFalse(production_gate(
            {**settings, "identity": {"model_sha256": "old"}},
            expected_identity={"model_sha256": "new"},
        )["allowed"])
        self.assertTrue(production_gate(None, force=True)["forced"])
        midpoint = midpoint_interpolation_error(
            {"CL": 0.0}, {"CL": 1.0}, {"CL": 0.505}, ["CL"], self.tolerance
        )
        self.assertEqual(midpoint["status"], "PASS")

    def test_mocked_full_convergence_writes_unified_outputs(self) -> None:
        config = deepcopy(load_project_config(PROJECT_ROOT / "config" / "aircraft.yaml"))
        reference = {
            "sref_m2": 10.0, "bref_m": 10.0, "cref_m": 1.0,
            "xcg_m": 0.0, "ycg_m": 0.0, "zcg_m": 0.0,
        }
        manifest = config["_manifest"]

        class FakeRunner:
            vsp = SimpleNamespace(GetVSPVersion=lambda: "OpenVSP 3.51.3")

            def __init__(self):
                self.wakes = []

            def run(self, condition, parent, label, *, stability, control_deflections_deg=None, wake_iterations=None, **_):
                self.wakes.append(wake_iterations)
                controls = control_deflections_deg or {}
                rudder = math.radians(float(controls.get("rudder", 0.0)))
                if not stability:
                    raw_data = {
                        "CFxtot": -0.01, "CFytot": 0.4 * rudder, "CFztot": -0.2,
                        "CLtot": 0.2, "CDtot": 0.01,
                        "CMxtot": 0.1 * rudder, "CMytot": 0.0, "CMztot": -0.2 * rudder,
                    }
                    return SimpleNamespace(raw_data=raw_data, duration_sec=0.001)
                q_s = float(condition["dynamic_pressure_pa"]) * reference["sref_m2"]
                cl_trim = float(config["trim"]["mass_kg"]) * float(config["trim"]["gravity_m_s2"]) / q_s
                coefficients = {
                    name: {"standard_value": value}
                    for name, value in {
                        "CL": cl_trim, "CD": 0.02, "CY": 0.0,
                        "Cl": 0.0, "Cm": 0.0, "Cn": 0.0,
                    }.items()
                }
                stability_derivatives = {}
                control_derivatives = {
                    role: {"derivatives": {}} for role in ("aileron", "elevator", "rudder")
                }
                for index, row in enumerate(manifest["derivatives"], 1):
                    measurement = {"standard_value": -0.1 - 0.001 * index}
                    perturbation = str(row["perturbation"])
                    if perturbation in control_derivatives:
                        control_derivatives[perturbation]["derivatives"][str(row["name"])] = measurement
                    else:
                        stability_derivatives[str(row["name"])] = measurement
                stability_derivatives["CL_alpha"] = {"standard_value": 1.0}
                stability_derivatives["Cm_alpha"] = {"standard_value": -0.5}
                control_derivatives["elevator"]["derivatives"]["CL_delta_e"] = {"standard_value": 0.2}
                control_derivatives["elevator"]["derivatives"]["Cm_delta_e"] = {"standard_value": -1.0}
                payload = {
                    "coefficients": coefficients,
                    "native_derivative_diagnostics": {
                        "stability": stability_derivatives,
                        "controls": control_derivatives,
                        "positive_rate_cases": {},
                        "diagnostic_only": True,
                    },
                }
                return SimpleNamespace(raw_data={}, payload=payload, duration_sec=0.001)

        with TemporaryDirectory() as temporary:
            config["_paths"]["results"] = Path(temporary)
            runner = FakeRunner()
            report = run_numerical_convergence(
                config=config, runner=runner, reference=reference, manifest=manifest,
                analysis_mapper=lambda raw: raw.payload,
            )
            output = Path(temporary) / "numerical_convergence"
            self.assertEqual(report["production_gate_status"], "WARN")
            self.assertNotIn(16, runner.wakes)
            for name in (
                "numerical_convergence_report.md", "numerical_convergence_report.json",
                "wake_convergence_map.csv", "fd_step_selection.csv",
                "required_derivatives_manifest.csv", "production_numerical_settings.yaml",
                "wake_convergence_map.png", "fd_step_convergence.png",
            ):
                self.assertTrue((output / name).is_file(), name)
            self.assertFalse((output / "tessellation_convergence.csv").exists())
            self.assertFalse(report["mesh"]["numerically_certified"])
            self.assertNotIn(
                "tessellation",
                report["production_numerical_settings"]["production_gate"]["reason"].lower(),
            )
            cm_q = next(
                item for item in report["required_derivatives_manifest"]["items"]
                if item["name"] == "Cm_q"
            )
            self.assertEqual(cm_q["validation_status"], "METHOD_LIMITATION")
            self.assertFalse(cm_q["production_included"])
            resumed = run_numerical_convergence(
                config=config, runner=FakeRunner(), reference=reference, manifest=manifest,
                analysis_mapper=lambda raw: raw.payload,
            )
            self.assertGreater(resumed["cache"]["hits"], 0)
            self.assertEqual(resumed["cache"]["misses"], 0)

            failed_pretrim = (
                2.0, 0.0,
                {
                    "status": "FAIL", "iterations": 15, "alpha_deg": 2.0,
                    "elevator_deg": 0.0, "force_residual_n": 10.0,
                    "moment_residual_nm": 2.0, "reason": "simulated non-convergence",
                    "history": [{"iteration": 15, "decision": "FAIL_MAX_ITERATIONS"}],
                },
            )
            with patch("numerical_convergence._trim_representative", return_value=failed_pretrim):
                isolated = run_numerical_convergence(
                    config=config, runner=FakeRunner(), reference=reference, manifest=manifest,
                    analysis_mapper=lambda raw: raw.payload,
                )
            wake_status = {
                point["name"]: point["status"] for point in isolated["wake"]["sample_points"]
            }
            self.assertEqual(wake_status["cruise_trim"], "FAIL")
            self.assertEqual(wake_status["linear_low_alpha"], "PASS")
            self.assertEqual(wake_status["medium_alpha"], "PASS")
            self.assertEqual(wake_status["high_alpha_beta"], "PASS")
            self.assertEqual(isolated["production_gate_status"], "FAIL")
            self.assertTrue((output / "numerical_convergence_report.json").is_file())


class DeliveredResultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        path = PROJECT_ROOT / "results" / "latest" / "aero_database.json"
        if not path.is_file():
            raise unittest.SkipTest("Run python run.py all before checking delivered results")
        cls.database = json.loads(path.read_text(encoding="utf-8"))
        results = cls.database.get("trim", {}).get("results", [])
        if not results or results[0].get("schema_version") != "6.0.0":
            raise unittest.SkipTest("Run the current workflow before checking delivered results")

    def test_final_status_and_case_counts(self) -> None:
        summary = self.database["summary"]
        self.assertEqual(summary["command"], "all")
        self.assertIn(summary["final_status"], {"PASS", "WARN"})
        self.assertEqual(summary["grid"]["completed"], 4)
        self.assertEqual(summary["trim"]["completed"], 1)
        self.assertEqual(summary["derivatives"]["calculated"], 15)
        self.assertEqual(summary["derivatives"]["METHOD_LIMITATION"], 8)
        self.assertEqual(summary["derivatives"]["missing"], 0)
        self.assertEqual(summary["derivatives"]["invalid"], 0)
        self.assertEqual(summary["derivatives"]["validation_failed"], 0)

    def test_grid_and_trim_numerics(self) -> None:
        grid = self.database["grid"]["results"]
        self.assertEqual(len(grid), 4)
        self.assertTrue(all(row["status"] == "PASS" for row in grid))
        trim = self.database["trim"]["results"][0]
        self.assertLessEqual(abs(trim["trim"]["trim_force_residual_n"]), 1.0)
        self.assertLessEqual(abs(trim["trim"]["trim_moment_residual_nm"]), 1.0)
        self.assertNotEqual(trim["trim"]["elevator_trim_deg"], 0.0)
        records = trim["derivatives"]["production_fd_derivatives"]
        self.assertEqual(len(records), 15)
        self.assertTrue(all(math.isfinite(float(row["derivative_value"])) for row in records.values()))

    def test_centered_samples_and_explicit_rate_limitation(self) -> None:
        package = self.database["trim"]["results"][0]["derivatives"]
        records = package["production_fd_derivatives"]
        self.assertGreaterEqual(len(records["CL_alpha"]["samples"]), 3)
        self.assertTrue(all(
            sample["plus"] is not None and sample["minus"] is not None
            for sample in records["CL_alpha"]["samples"].values()
        ))
        self.assertNotIn("Cl_p", records)
        self.assertTrue(package["native_derivative_diagnostics"]["Cl_p"]["diagnostic_only"])
        manifest_item = next(
            item for item in package["required_derivatives_manifest"]["items"]
            if item["name"] == "Cl_p"
        )
        self.assertEqual(manifest_item["validation_status"], "METHOD_LIMITATION")

    def test_multilevel_validation_and_fuselage(self) -> None:
        validation = self.database["validation"]
        self.assertNotEqual(validation["dataset_status"], "FAIL")
        self.assertEqual(validation["fuselage_effect"]["status"], "PASS")
        levels = {row["level"] for row in validation["rows"]}
        self.assertTrue({"SOLVER", "TRIM", "NUMERICAL", "DERIVATIVE", "PHYSICS", "DATASET"} <= levels)
        checks = {row["check"]: row["status"] for row in validation["rows"]}
        self.assertEqual(checks["alpha range"], "PASS")
        self.assertEqual(checks["elevator range"], "PASS")
        self.assertEqual(checks["beta centered-pair symmetry"], "PASS")
        self.assertEqual(checks["aileron centered-pair symmetry"], "PASS")
        self.assertEqual(checks["rudder centered-pair symmetry"], "PASS")

    def test_matlab_aero_schema_is_loadable(self) -> None:
        path = PROJECT_ROOT / "results" / "autotune" / "aircraft_aero.mat"
        loaded = loadmat(path, squeeze_me=True, struct_as_record=False)
        self.assertIn("AERO", loaded)
        aero = loaded["AERO"]
        self.assertEqual(str(aero.meta.schema_version), "1.0")
        self.assertTrue(hasattr(aero.flight_points, "V_mps"))
        self.assertTrue(hasattr(aero.longitudinal, "Cm_alpha"))
        self.assertFalse(hasattr(aero.lateral, "Cl_p"))
        self.assertTrue(hasattr(aero.native_derivative_diagnostics, "Cl_p"))
        self.assertTrue(hasattr(aero.controls.elevator, "Cm_delta_e"))
        self.assertNotEqual(str(aero.validation.overall_status), "FAIL")


if __name__ == "__main__":
    unittest.main(verbosity=2)
