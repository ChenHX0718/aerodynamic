from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

from scipy.io import loadmat


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC = PROJECT_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from case_generator import expand_axis, generate_grid_cases, generate_trim_cases
from config_loader import load_project_config
from coordinate_system import map_polar_coefficients, nondimensional_rate_step
from finite_difference import convergence_result


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
