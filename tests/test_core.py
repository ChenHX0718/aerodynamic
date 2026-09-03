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
from coordinate_system import map_polar_coefficients, nondimensional_rate_step
from finite_difference import convergence_result, dual_tolerance_result
from numerical_convergence import (
    boundary_continuity_result,
    derivative_bundle_wake,
    diagnose_cm_q,
    diagnose_cy_delta_r,
    midpoint_interpolation_error,
    minimum_converged_level,
    production_gate,
    promote_discrete_level,
    query_wake_schedule,
    run_numerical_convergence,
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

    def test_nondimensional_rate_definition(self) -> None:
        reference = {"bref_m": 11.0, "cref_m": 1.5}
        self.assertAlmostEqual(nondimensional_rate_step("p", 0.01, 8.0, reference), 0.006875)
        self.assertAlmostEqual(nondimensional_rate_step("q", 0.01, 8.0, reference), 0.0009375)

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
        return {"perturbations": {"alpha_deg": alpha_step, "elevator_deg": elevator_step}}

    @staticmethod
    def payload(cl: float, cm: float, stability: bool, native_scale: float = 1.0) -> dict:
        result = {
            "coefficients": {
                "CL": {"standard_value": cl}, "Cm": {"standard_value": cm},
            }
        }
        if stability:
            result["stability_derivatives"] = {
                "CL_alpha": {"standard_value": 99.0 * native_scale},
                "Cm_alpha": {"standard_value": -99.0 * native_scale},
            }
            result["control_derivatives"] = {"elevator": {"derivatives": {
                "CL_delta_e": {"standard_value": 55.0 * native_scale},
                "Cm_delta_e": {"standard_value": -55.0 * native_scale},
            }}}
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
        self.assertEqual(result.history[0]["native_derivatives"]["CL_alpha"], 99.0)

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
            {"scales": [0.5, 1, 2], "perturbations": {"alpha_deg": 5, "beta_deg": 0.5}},
        )
        self.assertEqual(wake, 8)

    def test_pretrim_selection_and_cross_region_upgrade(self) -> None:
        no_perturbation = {"scales": [1], "perturbations": {"alpha_deg": 0, "beta_deg": 0}}
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

    def test_tessellation_recommends_lowest_stable_preset(self) -> None:
        result = minimum_converged_level(
            ["COARSE", "MEDIUM", "FINE"],
            {"COARSE": {"CL": 0.8}, "MEDIUM": {"CL": 1.0}, "FINE": {"CL": 1.005}},
            ["CL"], self.tolerance, allow_unverified_highest=False,
        )
        self.assertEqual(result["required_level"], "MEDIUM")
        self.assertEqual(result["status"], "PASS")

    def test_cy_delta_r_diagnostic(self) -> None:
        samples = {}
        for step in (0.5, 1.0, 2.0, 4.0):
            angle = math.radians(step)
            samples[step] = {
                "minus": {"CY": -0.4 * angle, "Cl": 0.1 * angle, "Cn": -0.2 * angle},
                "plus": {"CY": 0.4 * angle, "Cl": -0.1 * angle, "Cn": 0.2 * angle},
            }
        result = diagnose_cy_delta_r(samples, self.tolerance)
        self.assertEqual(result["classification"], "LOCAL_LINEAR_VALID")
        self.assertEqual(result["recommended_delta_r_deg"], 1.0)

    def test_cm_q_limitation_remains_warn(self) -> None:
        result = diagnose_cm_q(
            {3: -8.0, 5: -8.01, 8: -8.02, 12: -8.02},
            {"COARSE": -8.0, "MEDIUM": -8.01, "FINE": -8.02}, self.tolerance,
        )
        self.assertEqual(result["numerical_status"], "PASS")
        self.assertEqual(result["status"], "WARN")
        self.assertIn("negative steady q", result["api_limitation"])

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

            def run(self, condition, parent, label, *, stability, control_deflections_deg=None, **_):
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
                    "stability_derivatives": stability_derivatives,
                    "control_derivatives": control_derivatives,
                    "native_rate_cases": {},
                }
                return SimpleNamespace(raw_data={}, payload=payload, duration_sec=0.001)

        with TemporaryDirectory() as temporary:
            config["_paths"]["results"] = Path(temporary)
            report = run_numerical_convergence(
                config=config, runner=FakeRunner(), reference=reference, manifest=manifest,
                analysis_mapper=lambda raw: raw.payload,
            )
            output = Path(temporary) / "numerical_convergence"
            self.assertEqual(report["production_gate_status"], "PASS")
            for name in (
                "numerical_convergence_report.md", "numerical_convergence_report.json",
                "wake_convergence_map.csv", "tessellation_convergence.csv",
                "derivative_diagnostics.csv", "production_numerical_settings.yaml",
                "wake_convergence_map.png", "tessellation_convergence.png", "CY_vs_delta_r.png",
            ):
                self.assertTrue((output / name).is_file(), name)
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
            tess_status = {
                detail["point"]: detail["decision"]["status"]
                for detail in isolated["tessellation"]["details"]
            }
            self.assertEqual(tess_status["cruise_trim"], "SKIPPED_DEPENDENCY")
            self.assertIn(tess_status["high_alpha_beta"], {"PASS", "WARN", "FAIL"})
            self.assertEqual(isolated["production_gate_status"], "FAIL")
            self.assertTrue((output / "numerical_convergence_report.json").is_file())


class DeliveredResultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        path = PROJECT_ROOT / "results" / "latest" / "aero_database.json"
        if not path.is_file():
            raise unittest.SkipTest("Run python run.py all before checking delivered results")
        cls.database = json.loads(path.read_text(encoding="utf-8"))

    def test_final_status_and_case_counts(self) -> None:
        summary = self.database["summary"]
        self.assertEqual(summary["command"], "all")
        self.assertIn(summary["final_status"], {"PASS", "WARN"})
        self.assertEqual(summary["grid"]["completed"], 4)
        self.assertEqual(summary["trim"]["completed"], 1)
        self.assertEqual(summary["derivatives"]["calculated"], 23)
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
        records = trim["derivatives"]["records"]
        self.assertEqual(len(records), 23)
        self.assertTrue(all(math.isfinite(float(row["value"])) for row in records.values()))

    def test_centered_samples_and_explicit_rate_limitation(self) -> None:
        records = self.database["trim"]["results"][0]["derivatives"]["records"]
        self.assertEqual(set(records["CL_alpha"]["samples"]), {"0.5", "1", "2"})
        self.assertTrue(all(
            sample["plus"] is not None and sample["minus"] is not None
            for sample in records["CL_alpha"]["samples"].values()
        ))
        self.assertEqual(records["Cl_p"]["method"], "vspaero_native_forward_rate")
        self.assertEqual(records["Cl_p"]["method_status"], "WARN")
        self.assertTrue(all(
            sample["plus"] is not None and sample["minus"] is None
            and sample["scale_base"] is not None
            for sample in records["Cl_p"]["samples"].values()
        ))

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
        self.assertTrue(hasattr(aero.lateral, "Cl_p"))
        self.assertTrue(hasattr(aero.controls.elevator, "Cm_delta_e"))
        self.assertNotEqual(str(aero.validation.overall_status), "FAIL")


if __name__ == "__main__":
    unittest.main(verbosity=2)
