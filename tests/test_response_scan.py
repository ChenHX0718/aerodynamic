from __future__ import annotations

import math
import sys
import unittest
from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory

from scipy.io import loadmat


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from config_loader import (
    ConfigError,
    _validate_response_scan,
    load_project_config,
    resolve_analysis_selection,
)
from export_results import build_database, export_database
from main import _grid_response_signature
from response_scan import (
    COEFFICIENTS,
    audit_response_curve,
    collect_control_responses,
    generate_control_scan_cases,
)


def _mapped(values: dict[str, float]) -> dict[str, dict[str, float]]:
    return {name: {"standard_value": float(value)} for name, value in values.items()}


def _base(case_id: str = "CASE_BASE") -> dict:
    values = {name: float(index) for index, name in enumerate(COEFFICIENTS)}
    stability = {
        f"{name}_{rate}": {
            "standard_value": 0.1 * (index + 1),
            "standard_unit": f"1/{rate}_hat",
            "raw_field": f"VSPAERO_Stab.{name}_{rate}",
        }
        for rate in ("p", "q", "r")
        for index, name in enumerate(COEFFICIENTS)
    }
    return {
        "case_id": case_id,
        "mode": "GRID_DATABASE",
        "grid_source": "seed",
        "status": "PASS",
        "inputs": {
            "speed_mps": 8.0,
            "alpha_deg": 1.0,
            "beta_deg": 0.0,
            "rho_kg_m3": 1.225,
            "dynamic_pressure_pa": 39.2,
            "mach": 0.0235,
            "reynolds_cref": 8.0e5,
        },
        "outputs": {
            "coefficients": _mapped(values),
            "native_derivative_diagnostics": {"stability": stability, "controls": {}},
        },
        "solver": {"status": "SUCCESS", "duration_sec": 0.0},
        "numerical_settings": {"wake_iterations": 5},
    }


class AnalysisSelectionTests(unittest.TestCase):
    def test_all_honors_every_grid_trim_combination(self):
        self.assertEqual(resolve_analysis_selection(
            "all", {"grid_enabled": True, "trim_enabled": False}
        ), (True, False))
        self.assertEqual(resolve_analysis_selection(
            "all", {"grid_enabled": False, "trim_enabled": True}
        ), (False, True))
        self.assertEqual(resolve_analysis_selection(
            "all", {"grid_enabled": True, "trim_enabled": True}
        ), (True, True))
        with self.assertRaisesRegex(ConfigError, "both false"):
            resolve_analysis_selection("all", {"grid_enabled": False, "trim_enabled": False})

    def test_explicit_grid_and_trim_commands_remain_available(self):
        disabled = {"grid_enabled": False, "trim_enabled": False}
        self.assertEqual(resolve_analysis_selection("grid", disabled), (True, False))
        self.assertEqual(resolve_analysis_selection("trim", disabled), (False, True))


class ResponseLogicTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_project_config(PROJECT_ROOT / "config" / "aircraft.yaml")

    def test_control_case_generation_baseline_reuse_and_delta(self):
        settings = deepcopy(self.config["response_scan"])
        settings["variables"] = ["elevator"]
        settings["control_levels_deg"]["elevator"] = [-5.0, 0.0, 5.0]
        base = _base()
        cases = generate_control_scan_cases([base], settings)
        self.assertEqual(len(cases), 3)
        calls = []

        def evaluate(spec, _base_result):
            calls.append(spec.perturbation_value)
            baseline = {
                name: float(index) for index, name in enumerate(COEFFICIENTS)
            }
            coefficients = {
                name: value + 0.01 * spec.perturbation_value
                for name, value in baseline.items()
            }
            return {
                "coefficients": coefficients,
                "wake_iterations": 5,
                "solver_status": "SUCCESS",
                "solver_duration_sec": 0.0,
            }, False

        samples, cache = collect_control_responses(
            bases=[base], settings=settings, controls=self.config["controls"],
            evaluator=evaluate,
        )
        self.assertEqual(calls, [-5.0, 5.0])
        zero = next(item for item in samples if item["perturbation_value"] == 0.0)
        plus = next(item for item in samples if item["perturbation_value"] == 5.0)
        self.assertEqual(zero["source"], "base_grid_reuse")
        self.assertTrue(all(value == 0.0 for value in zero["delta_coefficients"].values()))
        self.assertAlmostEqual(plus["delta_coefficients"]["Cm"], 0.05)
        self.assertEqual(cache["baseline_reuses"], 1)

    def test_linearity_audit_separates_linear_and_curved_data(self):
        settings = self.config["response_scan"]["linearity"]

        def samples(function):
            return [
                {
                    "offset_from_neutral_deg": x,
                    "coefficients": {"CL": function(math.radians(x))},
                    "delta_coefficients": {"CL": function(math.radians(x)) - function(0.0)},
                }
                for x in (-10.0, -5.0, 0.0, 5.0, 10.0)
            ]

        linear = audit_response_curve(samples(lambda x: 0.2 + 2.0 * x), "CL", settings)
        nonlinear = audit_response_curve(samples(lambda x: 0.2 + x + 80.0 * x**3), "CL", settings)
        self.assertEqual(linear["classification"], "LINEAR")
        self.assertEqual(linear["recommended_representation"], "derivative")
        self.assertEqual(nonlinear["classification"], "NONLINEAR")
        self.assertEqual(nonlinear["recommended_representation"], "response_table")

    def test_control_levels_outside_aircraft_limits_are_rejected(self):
        data = {"response_scan": deepcopy(self.config["response_scan"])}
        data["response_scan"]["control_levels_deg"]["elevator"] = [-5.0, 0.0, 25.0]
        with self.assertRaisesRegex(ConfigError, "outside"):
            _validate_response_scan(data, self.config["controls"])

    def test_signature_context_changes_when_scan_levels_change(self):
        first = deepcopy(self.config)
        second = deepcopy(self.config)
        second["response_scan"]["control_levels_deg"]["elevator"] = [-10.0, 0.0, 10.0]
        base = _base()
        spec = generate_control_scan_cases([base], first["response_scan"])[0]
        arguments = {
            "signature_context": {"model_sha256": "model", "openvsp_version": "3.51.3"},
            "base": base, "spec": spec, "condition": base["inputs"],
            "deflections": {"aileron": 0.0, "elevator": spec.perturbation_value, "rudder": 0.0},
            "wake_iterations": 5,
        }
        one = _grid_response_signature(
            **arguments, response_settings=first["response_scan"]
        )
        two = _grid_response_signature(
            **arguments, response_settings=second["response_scan"]
        )
        self.assertNotEqual(one, two)


class GridOnlyExportTests(unittest.TestCase):
    def test_grid_only_json_csv_and_mat_do_not_require_trim(self):
        config = load_project_config(PROJECT_ROOT / "config" / "aircraft.yaml")
        base = _base()
        samples = []
        for variable in ("elevator", "aileron", "rudder"):
            for value in (-5.0, 0.0, 5.0):
                coefficients = {
                    name: float(index) + 0.01 * value
                    for index, name in enumerate(COEFFICIENTS)
                }
                samples.append({
                    "case_id": f"{variable}_{value:g}",
                    "base_grid_case_id": base["case_id"],
                    "speed_mps": 8.0, "alpha_deg": 1.0, "beta_deg": 0.0,
                    "variable": variable, "perturbation_value": value,
                    "perturbation_unit": "deg", "neutral_value": 0.0,
                    "offset_from_neutral_deg": value,
                    "coefficients": coefficients,
                    "delta_coefficients": {name: 0.01 * value for name in COEFFICIENTS},
                    "wake_iterations": 5, "solver_status": "SUCCESS",
                    "solver_duration_sec": 0.0, "status": "PASS",
                    "cache_hit": value == 0.0, "source": "synthetic", "raw_directory": "",
                })
        rates = {
            variable: [{
                "base_grid_case_id": base["case_id"],
                "speed_mps": 8.0, "alpha_deg": 1.0, "beta_deg": 0.0,
                "representation": "local_linear_derivative",
                "normalization": f"{variable}_hat",
                "source": "VSPAERO steady .stab", "status": "PASS",
                "missing_coefficients": [],
                "derivatives": {
                    name: {"value": 0.1, "unit": f"1/{variable}_hat", "source_field": f"field.{name}_{variable}"}
                    for name in COEFFICIENTS
                },
            }]
            for variable in ("p", "q", "r")
        }
        responses = {
            "enabled": True, "scope": "audit", "selected_base_case_ids": [base["case_id"]],
            "controls": {
                variable: [item for item in samples if item["variable"] == variable]
                for variable in ("elevator", "aileron", "rudder")
            },
            "rates": rates, "linearity": [],
            "gate": {"status": "PASS", "reason": "synthetic complete"},
            "cache": {},
            "assumptions": {"additive_response_assumption": True, "baseline_requires_trim": False},
        }
        database = build_database(
            metadata={
                "aircraft_name": "synthetic", "generated_at_local": "now",
                "openvsp_version": "3.51.3", "solver": "VSPAERO",
                "model_sha256": "model", "coordinate_system": {},
                "production_numerical_settings": {"solver_gate": {"status": "PASS"}},
                "analysis_selection": {"grid_enabled": True, "trim_enabled": False},
            },
            reference={"sref_m2": 1.0, "bref_m": 2.0, "cref_m": 0.5},
            geometry={}, manifest=config["_manifest"], grid_results=[base], trim_results=[],
            responses=responses,
            validation={"rows": [], "dataset_status": "PASS"},
            summary={"command": "grid"}, grid_mode="uniform",
        )
        with TemporaryDirectory() as directory:
            paths = export_database(
                database, Path(directory), {"csv": True, "json": True, "mat": True},
                {"autotune_allow_warn": True}, "PASS\n",
            )
            self.assertEqual(paths["mat_status"], "PASS")
            self.assertTrue(paths["json"].is_file())
            self.assertTrue(paths["response_csv"].is_file())
            aero = loadmat(paths["mat"], squeeze_me=True, struct_as_record=False)["AERO"]
            self.assertEqual(str(aero.meta.schema_version), "2.0")
            self.assertTrue(hasattr(aero, "grid"))
            self.assertTrue(hasattr(aero, "responses"))
            self.assertTrue(hasattr(aero.responses.controls, "elevator"))
            self.assertTrue(bool(aero.assumptions.additive_response_assumption))


if __name__ == "__main__":
    unittest.main(verbosity=2)
