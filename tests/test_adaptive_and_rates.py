from __future__ import annotations

import json
import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory

from scipy.io import loadmat, savemat


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from adaptive_grid import run_adaptive_grid, write_adaptive_report
from config_loader import load_project_config
from coordinate_system import map_stability_derivatives, map_unsteady_derivatives
from export_results import _aero_mat, _derivative_rows
from finite_difference import calculate_trim_derivatives
from numerical_convergence import production_gate
from regression import _current_values


COEFFICIENTS = ("CL", "CD", "Cm", "CY", "Cl", "Cn")


def _adaptive_settings(**overrides):
    settings = {
        "quantities": ["CL"],
        "tolerance": {
            "near_zero_reference": 0.01,
            "pass_relative": 0.0,
            "warn_relative": 0.0,
            "pass_absolute": 1.0e-10,
            "warn_absolute": 1.0e-8,
        },
        "max_depth": 3,
        "max_cases": 100,
        "min_spacing": {"speed_mps": 1.0e-6, "alpha_deg": 1.0e-6, "beta_deg": 1.0e-6},
    }
    settings.update(overrides)
    return settings


def _evaluator(function, persistent_keys=()):
    calls = []
    persistent = set(persistent_keys)

    def evaluate(point, source):
        key = tuple(point[name] for name in ("speed_mps", "alpha_deg", "beta_deg"))
        calls.append((key, source))
        value = float(function(*key))
        return {
            "case_id": "case_" + "_".join(f"{item:g}" for item in key),
            "mode": "GRID_DATABASE",
            "status": "PASS",
            "inputs": dict(point),
            "outputs": {
                "coefficients": {
                    name: {"standard_value": value if name == "CL" else 0.0}
                    for name in COEFFICIENTS
                }
            },
        }, key in persistent

    return evaluate, calls


class AdaptiveGridTests(unittest.TestCase):
    def test_linear_1d_2d_and_3d_need_no_refinement(self):
        cases = [
            (
                {"speed_mps": [0.0, 1.0], "alpha_deg": [0.0], "beta_deg": [0.0]},
                lambda speed, alpha, beta: speed,
                ["speed_mps"],
                3,
            ),
            (
                {"speed_mps": [0.0, 1.0], "alpha_deg": [0.0, 2.0], "beta_deg": [0.0]},
                lambda speed, alpha, beta: speed + 2.0 * alpha + 3.0 * speed * alpha,
                ["speed_mps", "alpha_deg"],
                5,
            ),
            (
                {"speed_mps": [0.0, 1.0], "alpha_deg": [0.0, 2.0], "beta_deg": [-1.0, 1.0]},
                lambda speed, alpha, beta: (
                    speed + 2.0 * alpha + 3.0 * beta
                    + 4.0 * speed * alpha - 5.0 * alpha * beta
                    + 6.0 * speed * alpha * beta
                ),
                ["speed_mps", "alpha_deg", "beta_deg"],
                9,
            ),
        ]
        for axes, function, active, expected_points in cases:
            with self.subTest(active=active):
                evaluate, calls = _evaluator(function)
                results, report = run_adaptive_grid(
                    axes=axes, settings=_adaptive_settings(), evaluator=evaluate
                )
                self.assertEqual(report["status"], "PASS")
                self.assertEqual(report["active_dimensions"], active)
                self.assertEqual(report["refined_cell_count"], 0)
                self.assertEqual(len(results), expected_points)
                self.assertEqual(len(calls), expected_points)
                self.assertEqual(report["new_solver_duration_sec"], 0.0)
                self.assertEqual(
                    {item["grid_source"] for item in results}, {"seed", "adaptive_midpoint"}
                )

    def test_nonlinear_cell_refines_and_is_deterministic(self):
        axes = {"speed_mps": [0.0, 1.0], "alpha_deg": [0.0], "beta_deg": [0.0]}
        settings = _adaptive_settings(max_depth=1)
        first, first_report = run_adaptive_grid(
            axes=axes, settings=settings, evaluator=_evaluator(lambda x, _a, _b: x * x)[0]
        )
        second, second_report = run_adaptive_grid(
            axes=axes, settings=settings, evaluator=_evaluator(lambda x, _a, _b: x * x)[0]
        )
        self.assertEqual(first_report["refined_cell_count"], 1)
        self.assertEqual(first_report["status"], "FAIL")
        self.assertIn("max_depth", first_report["termination_reason"])
        self.assertEqual(first_report["refinement_history"], second_report["refinement_history"])
        self.assertEqual(
            [item["inputs"] for item in first], [item["inputs"] for item in second]
        )

    def test_normalized_span_tie_break_is_speed_then_alpha_then_beta(self):
        axes = {
            "speed_mps": [0.0, 1.0],
            "alpha_deg": [0.0, 10.0],
            "beta_deg": [-100.0, 100.0],
        }
        _results, report = run_adaptive_grid(
            axes=axes,
            settings=_adaptive_settings(max_depth=1),
            evaluator=_evaluator(lambda speed, alpha, beta: speed**2 + alpha**2 + beta**2)[0],
        )
        self.assertEqual(report["refinement_history"][0]["split_axis"], "speed_mps")

    def test_every_limit_and_cache_accounting_are_honest(self):
        axes = {"speed_mps": [0.0, 1.0], "alpha_deg": [0.0], "beta_deg": [0.0]}
        scenarios = (
            ("max_depth", _adaptive_settings(max_depth=0)),
            (
                "min_spacing",
                _adaptive_settings(
                    min_spacing={"speed_mps": 0.6, "alpha_deg": 1.0e-6, "beta_deg": 1.0e-6}
                ),
            ),
            ("max_cases", _adaptive_settings(max_cases=3)),
        )
        for reason, settings in scenarios:
            with self.subTest(reason=reason):
                evaluate, calls = _evaluator(
                    lambda speed, _alpha, _beta: speed * speed,
                    persistent_keys={(0.0, 0.0, 0.0)},
                )
                results, report = run_adaptive_grid(
                    axes=axes, settings=settings, evaluator=evaluate
                )
                self.assertNotEqual(report["status"], "PASS")
                self.assertIn(reason, report["termination_reason"])
                self.assertEqual(report["persistent_cache_hits"], 1)
                self.assertGreater(report["deduplicated_point_reuses"], 0)
                self.assertEqual(report["new_solver_runs"], len(calls) - 1)
                self.assertEqual(len(results), len(calls))

    def test_adaptive_report_json_and_csv(self):
        axes = {"speed_mps": [0.0, 1.0], "alpha_deg": [0.0], "beta_deg": [0.0]}
        results, report = run_adaptive_grid(
            axes=axes,
            settings=_adaptive_settings(),
            evaluator=_evaluator(lambda speed, _alpha, _beta: speed)[0],
        )
        with TemporaryDirectory() as directory:
            output = Path(directory)
            write_adaptive_report(output, report, results)
            loaded = json.loads((output / "adaptive_grid_report.json").read_text(encoding="utf-8"))
            self.assertEqual(loaded["status"], "PASS")
            points = (output / "adaptive_grid_points.csv").read_text(encoding="utf-8-sig")
            cells = (output / "adaptive_grid_cells.csv").read_text(encoding="utf-8-sig")
            self.assertIn("grid_source", points)
            self.assertIn("termination_reason", cells)


class RateDerivativeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_project_config(PROJECT_ROOT / "config" / "aircraft.yaml")

    def test_steady_rate_field_mapping_units_and_signs(self):
        raw = {
            "CL_q": 1.25,
            "CMm_q": -2.5,
            "CFy_p": 0.2,
            "CMl_p": -0.4,
            "CMn_p": 0.1,
            "CFy_r": -0.3,
            "CMl_r": 0.15,
            "CMn_r": -0.8,
        }
        mapped = map_stability_derivatives(raw)
        self.assertEqual(set(mapped), {"CL_q", "Cm_q", "CY_p", "Cl_p", "Cn_p", "CY_r", "Cl_r", "Cn_r"})
        self.assertEqual(mapped["Cl_p"]["standard_value"], -0.4)
        self.assertEqual(mapped["Cm_q"]["standard_unit"], "1/q_hat")
        self.assertEqual(mapped["CY_p"]["standard_unit"], "1/p_hat")
        self.assertEqual(mapped["Cn_r"]["standard_unit"], "1/r_hat")
        self.assertEqual(mapped["Cn_r"]["raw_field"], "VSPAERO_Stab.CMn_r")

    def test_normalized_rate_denominators_reproduce_stab_values(self):
        base = {"CFy": 0.0002556, "CL": 0.3952698, "CMl": -0.0006787,
                "CMm": 0.0222365, "CMn": -0.0004179}
        cases = {
            "p": {"CFy": -0.0002194, "CMl": -0.0039491, "CMn": -0.0007658},
            "q": {"CL": 0.4035955, "CMm": 0.0100495},
            "r": {"CFy": 0.0019463, "CMl": -0.0000699, "CMn": -0.0009946},
        }
        denominators = {"p": 0.01 * 11.0 / 16.0,
                        "q": 0.01 * 1.49 / 16.0,
                        "r": 0.01 * 11.0 / 16.0}
        expected = {
            ("p", "CFy"): -0.0690865, ("p", "CMl"): -0.4756984,
            ("p", "CMn"): -0.0506006, ("q", "CL"): 8.9404220,
            ("q", "CMm"): -13.0867678, ("r", "CFy"): 0.2459173,
            ("r", "CMl"): 0.0885498, ("r", "CMn"): -0.0838761,
        }
        for mode, values in cases.items():
            for coefficient, value in values.items():
                with self.subTest(mode=mode, coefficient=coefficient):
                    actual = (value - base[coefficient]) / denominators[mode]
                    # The source .stab table prints only seven decimal places, so
                    # the reconstructed ratio is limited by that text precision.
                    self.assertAlmostEqual(actual, expected[(mode, coefficient)], delta=1.0e-4)

    def test_unsteady_combined_terms_cannot_replace_classical_rates(self):
        q = map_unsteady_derivatives(
            {"CL_q+alpha_dot": 1.0, "CMm_q+alpha_dot": -2.0}, "q"
        )
        r = map_unsteady_derivatives(
            {"CFy_r-beta_dot": 0.3, "CMn_r-beta_dot": -0.7}, "r"
        )
        self.assertIn("Cm_q_plus_alpha_dot_unsteady", q)
        self.assertIn("Cn_r_minus_beta_dot_unsteady", r)
        self.assertTrue(q["Cm_q_plus_alpha_dot_unsteady"]["combined_derivative"])
        self.assertTrue(r["Cn_r_minus_beta_dot_unsteady"]["diagnostic_only"])
        self.assertNotIn("Cm_q", q)
        self.assertNotIn("Cn_r", r)

    def _production_package(self):
        manifest = self.config["_manifest"]
        diagnostics = {"stability": {}, "controls": {}}
        for row in manifest["_required"]:
            if row["perturbation"] in {"p", "q", "r"}:
                raw_name = row["name"].replace("CY_", "CFy_").replace("Cl_", "CMl_").replace("Cm_", "CMm_").replace("Cn_", "CMn_")
                diagnostics["stability"][row["name"]] = {
                    "standard_value": -0.25,
                    "standard_unit": row["unit"],
                    "raw_field": f"VSPAERO_Stab.{raw_name}",
                }
        base_outputs = {
            "coefficients": {
                name: {"standard_value": 0.0} for name in COEFFICIENTS
            },
            "native_derivative_diagnostics": diagnostics,
        }
        condition = {"speed_mps": 8.0, "alpha_deg": 0.0, "beta_deg": 0.0}
        controls = {"aileron": 0.0, "elevator": 0.0, "rudder": 0.0}

        def run_polar(state, _label, deflections):
            alpha = math.radians(state["alpha_deg"])
            beta = math.radians(state["beta_deg"])
            aileron = math.radians(deflections["aileron"])
            elevator = math.radians(deflections["elevator"])
            rudder = math.radians(deflections["rudder"])
            values = {
                "CL": alpha + 0.2 * elevator,
                "CD": 0.1 * alpha + 0.01 * elevator,
                "Cm": -0.5 * alpha - elevator,
                "CY": -0.3 * beta + 0.1 * aileron - 0.4 * rudder,
                "Cl": -0.1 * beta - 0.2 * aileron + 0.02 * rudder,
                "Cn": 0.2 * beta + 0.01 * aileron + 0.2 * rudder,
            }
            return {
                "coefficients": {
                    name: {"standard_value": value} for name, value in values.items()
                },
                "solver_duration_sec": 0.0,
            }

        derivative_config = deepcopy(self.config["derivatives"])
        derivative_config["_controls"] = self.config["controls"]
        derivative_config["_bundle_wake_iterations"] = 12
        return calculate_trim_derivatives(
            condition=condition,
            base_outputs=base_outputs,
            base_controls=controls,
            manifest=manifest,
            derivative_config=derivative_config,
            run_polar=run_polar,
        ), base_outputs

    def test_unified_production_has_23_and_mat_contains_rates_and_grid(self):
        package, base_outputs = self._production_package()
        self.assertEqual(len(package["production_derivatives"]), 23)
        self.assertEqual(sum(
            item["method"] == "centered_finite_difference"
            for item in package["production_derivatives"].values()
        ), 15)
        self.assertEqual(sum(
            item["method"] == "vspaero_steady_rate_derivative"
            for item in package["production_derivatives"].values()
        ), 8)
        self.assertNotIn("production_fd_derivatives", package)
        self.assertNotIn("production_rate_derivatives", package)
        required_fields = {
            "name", "value", "units", "method", "source", "source_field",
            "wake_iterations", "validation_status", "coordinate_sign_convention",
            "production_included",
        }
        self.assertTrue(all(
            required_fields <= set(record)
            for record in package["production_derivatives"].values()
        ))
        self.assertTrue(all(
            item["production_included"]
            for item in package["required_derivatives_manifest"]["items"]
        ))
        trim = {
            "case_id": "synthetic",
            "mode": "TRIM_DATABASE",
            "status": "PASS",
            "inputs": {
                "speed_mps": 8.0,
                "rho_kg_m3": 1.225,
                "dynamic_pressure_pa": 39.2,
                "mach": 0.02,
                "reynolds_cref": 1.0e6,
            },
            "outputs": base_outputs,
            "trim": {
                "alpha_trim_deg": 0.0,
                "beta_trim_deg": 0.0,
                "elevator_trim_deg": 0.0,
                "CL_trim": 0.0,
                "CD_trim": 0.0,
                "CY_trim": 0.0,
                "Cl_trim": 0.0,
                "Cm_trim": 0.0,
                "Cn_trim": 0.0,
                "trim_force_residual_n": 0.0,
                "trim_moment_residual_nm": 0.0,
                "trim_iterations": 1,
            },
            "derivatives": package,
            "validation": {
                "overall_status": "PASS",
                "trim_status": "PASS",
                "numerical_status": "PASS",
                "derivative_status": "PASS",
                "physics_status": "PASS",
            },
        }
        grid_result = {
            "case_id": "grid_center",
            "status": "PASS",
            "grid_source": "adaptive_midpoint",
            "inputs": {"speed_mps": 10.0, "alpha_deg": 1.0, "beta_deg": 0.0},
            "outputs": {
                "coefficients": {
                    name: {"standard_value": float(index)}
                    for index, name in enumerate(COEFFICIENTS)
                }
            },
        }
        database = {
            "metadata": {
                "aircraft_name": "synthetic",
                "generated_at_local": "now",
                "openvsp_version": "OpenVSP 3.51.3",
                "solver": "VSPAERO",
                "model_sha256": "synthetic",
                "coordinate_system": {},
                "production_numerical_settings": {"production_gate": {"status": "PASS"}},
            },
            "reference": {},
            "manifest": self.config["_manifest"],
            "grid": {"mode": "adaptive", "results": [grid_result]},
        }
        mat = _aero_mat(database, [trim])
        with TemporaryDirectory() as directory:
            path = Path(directory) / "aircraft_aero.mat"
            savemat(path, {"AERO": mat}, do_compression=True, long_field_names=True)
            loaded = loadmat(path, squeeze_me=True, struct_as_record=False)["AERO"]
            self.assertTrue(hasattr(loaded.longitudinal, "Cm_q"))
            self.assertTrue(hasattr(loaded.lateral, "Cl_p"))
            self.assertTrue(hasattr(loaded.lateral, "Cn_r"))
            for name in ("CL_q", "Cm_q"):
                self.assertTrue(hasattr(loaded.longitudinal, name), name)
            for name in ("CY_p", "Cl_p", "Cn_p", "CY_r", "Cl_r", "Cn_r"):
                self.assertTrue(hasattr(loaded.lateral, name), name)
            self.assertEqual(str(loaded.grid.mode), "adaptive")
            self.assertEqual(str(loaded.grid.grid_source), "adaptive_midpoint")

        rows = _derivative_rows([trim], {})
        self.assertEqual(len(rows), 23)
        self.assertTrue(all("source_field" in row for row in rows))

    def test_gate_layers_and_adaptive_eligibility(self):
        settings = {
            "solver_gate": {"status": "PASS", "reason": "solver evidence"},
            "derivative_gate": {"status": "FAIL", "reason": "derivative evidence"},
            "production_gate": {"status": "FAIL", "reason": "combined evidence"},
        }
        adaptive = production_gate(settings, adaptive=True, force=True)
        delivery = production_gate(settings, adaptive=False)
        self.assertTrue(adaptive["allowed"])
        self.assertFalse(adaptive["forced"])
        self.assertEqual(adaptive["gate"], "solver_gate")
        self.assertFalse(delivery["allowed"])

        warning = production_gate({
            **settings,
            "production_gate": {"status": "WARN_NUMERICAL", "reason": "usable sensitivity"},
        })
        self.assertTrue(warning["allowed"])
        self.assertFalse(warning["forced"])

    def test_regression_reads_only_production_derivatives(self):
        result = {
            "trim": {"alpha_trim_deg": 1.0, "elevator_trim_deg": -2.0},
            "derivatives": {
                "production_derivatives": {"Cm_q": {"value": -4.0}},
                "required_derivatives_manifest": {
                    "items": [{"name": "Cm_q", "value": 999.0}]
                },
                "native_derivative_diagnostics": {
                    "Cm_q": {"diagnostic_value": 888.0}
                },
            },
        }
        values = _current_values(result)
        self.assertEqual(values["Cm_q"], -4.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
