from __future__ import annotations

import os
import time
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any

from openvsp_interface import GeometrySelection, OpenVSPError, OpenVSPModel, pop_api_errors


@dataclass(frozen=True)
class AeroRunResult:
    analysis: str
    case_dir: Path
    geometry_result_id: str
    sweep_result_id: str
    data_result_id: str
    raw_data: dict[str, Any]
    duration_sec: float


class VSPAERORunner:
    def __init__(
        self,
        vsp: ModuleType,
        model_path: Path,
        config: dict[str, Any],
        reference: dict[str, float],
        geometry_selection: GeometrySelection,
    ):
        self.vsp = vsp
        self.model = OpenVSPModel(vsp, model_path)
        self.config = config
        self.reference = reference
        self.geometry_selection = geometry_selection

    def _require_inputs(self, analysis: str, names: set[str]) -> None:
        available = set(self.vsp.GetAnalysisInputNames(analysis))
        missing = sorted(names - available)
        if missing:
            raise OpenVSPError(
                f"OpenVSP {self.vsp.GetVSPVersion()} analysis {analysis} is missing input(s): "
                + ", ".join(missing)
            )

    def _set_int(self, analysis: str, name: str, value: int) -> None:
        self.vsp.SetIntAnalysisInput(analysis, name, [int(value)], 0)

    def _set_double(self, analysis: str, name: str, value: float) -> None:
        self.vsp.SetDoubleAnalysisInput(analysis, name, [float(value)], 0)

    def _set_string(self, analysis: str, name: str, value: str) -> None:
        self.vsp.SetStringAnalysisInput(analysis, name, [str(value)], 0)

    def _configure_geometry(self, include_thick: bool) -> str:
        analysis = "VSPAEROComputeGeometry"
        self._require_inputs(analysis, {"GeomSet", "ThinGeomSet", "Symmetry"})
        self.vsp.SetAnalysisInputDefaults(analysis)
        self._set_int(
            analysis, "GeomSet",
            self.geometry_selection.thick_set_index if include_thick else -1,
        )
        self._set_int(analysis, "ThinGeomSet", self.geometry_selection.thin_set_index)
        self._set_int(analysis, "Symmetry", 0)
        result_id = self.vsp.ExecAnalysis(analysis)
        errors = pop_api_errors(self.vsp)
        if not result_id or errors:
            detail = "\n".join(errors) if errors else "empty Results ID"
            raise OpenVSPError(f"VSPAEROComputeGeometry failed:\n{detail}")
        return result_id

    def _configure_sweep(
        self,
        case_dir: Path,
        condition: dict[str, float],
        stability: bool,
        include_thick: bool,
        wake_iterations: int | None,
    ) -> str:
        analysis = "VSPAEROSweep"
        required = {
            "GeomSet", "ThinGeomSet", "AlphaStart", "AlphaEnd", "AlphaNpts",
            "BetaStart", "BetaEnd", "BetaNpts", "MachStart", "MachEnd", "MachNpts",
            "Vinf", "Vref", "ManualVrefFlag", "Rho", "ReCref", "ReCrefEnd", "ReCrefNpts",
            "WakeNumIter", "NCPU", "UnsteadyType", "RedirectFile", "RefFlag",
            "Sref", "bref", "cref", "Xcg", "Ycg", "Zcg", "Symmetry",
        }
        self._require_inputs(analysis, required)
        solver = self.config["solver"]
        self.vsp.SetAnalysisInputDefaults(analysis)
        self._set_int(
            analysis, "GeomSet",
            self.geometry_selection.thick_set_index if include_thick else -1,
        )
        self._set_int(analysis, "ThinGeomSet", self.geometry_selection.thin_set_index)
        self._set_int(analysis, "Symmetry", 0)
        for stem, key in (("Alpha", "alpha_deg"), ("Beta", "beta_deg"), ("Mach", "mach")):
            value = condition[key]
            self._set_double(analysis, f"{stem}Start", value)
            self._set_double(analysis, f"{stem}End", value)
            self._set_int(analysis, f"{stem}Npts", 1)
        speed = condition["speed_mps"]
        self._set_double(analysis, "Vinf", speed)
        self._set_double(analysis, "Vref", speed)
        self._set_int(analysis, "ManualVrefFlag", 1)
        self._set_double(analysis, "Rho", condition["rho_kg_m3"])
        self._set_double(analysis, "ReCref", condition["reynolds_cref"])
        self._set_double(analysis, "ReCrefEnd", condition["reynolds_cref"])
        self._set_int(analysis, "ReCrefNpts", 1)
        wake = int(wake_iterations if wake_iterations is not None else solver["wake_iterations"])
        if wake <= 0:
            raise OpenVSPError(f"WakeNumIter must be positive, got {wake}")
        self._set_int(analysis, "WakeNumIter", wake)
        self._set_int(analysis, "NCPU", int(solver.get("ncpu", 4)))
        self._set_int(analysis, "UnsteadyType", self.vsp.STABILITY_DEFAULT if stability else self.vsp.STABILITY_OFF)
        self._set_string(analysis, "RedirectFile", str((case_dir / "vspaero_console.txt").resolve()))
        self._set_int(analysis, "RefFlag", 0)
        for input_name, key in (
            ("Sref", "sref_m2"), ("bref", "bref_m"), ("cref", "cref_m"),
            ("Xcg", "xcg_m"), ("Ycg", "ycg_m"), ("Zcg", "zcg_m"),
        ):
            self._set_double(analysis, input_name, self.reference[key])
        result_id = self.vsp.ExecAnalysis(analysis)
        errors = pop_api_errors(self.vsp)
        if not result_id or errors:
            detail = "\n".join(errors) if errors else "empty Results ID"
            raise OpenVSPError(f"VSPAEROSweep failed:\n{detail}")
        return result_id

    def _prepare_case(
        self,
        case_dir: Path,
        control_deflections_deg: dict[str, float] | None,
    ) -> None:
        case_dir.mkdir(parents=True, exist_ok=False)
        self.model.load()
        self.model.apply_geometry_sets(self.geometry_selection)
        requested = control_deflections_deg or {}
        for role in ("aileron", "elevator", "rudder"):
            control_config = self.config["controls"][role]
            value = float(requested.get(role, control_config.get("neutral_deg", 0.0)))
            self.model.set_control_deflection(str(control_config["group"]), value)
        self.model.write_case_model(case_dir / self.model.model_path.name)

    def _read_result(self, result_id: str) -> dict[str, Any]:
        raw: dict[str, Any] = {}
        for field in self.vsp.GetAllDataNames(result_id):
            result_type = self.vsp.GetResultsType(result_id, field)
            if result_type == self.vsp.DOUBLE_DATA:
                values = list(self.vsp.GetDoubleResults(result_id, field, 0))
            elif result_type == self.vsp.INT_DATA:
                values = list(self.vsp.GetIntResults(result_id, field, 0))
            elif result_type == self.vsp.STRING_DATA:
                values = list(self.vsp.GetStringResults(result_id, field, 0))
            else:
                continue
            if values:
                raw[field] = values[-1]
        return raw

    @staticmethod
    def _validate_raw_files(case_dir: Path, stability: bool) -> None:
        required = [".vspgeom", ".vspaero", ".history", ".adb"]
        if stability:
            required.append(".stab")
        names = [path.name.lower() for path in case_dir.iterdir() if path.is_file()]
        missing = [suffix for suffix in required if not any(name.endswith(suffix) for name in names)]
        if missing:
            raise OpenVSPError(f"Raw VSPAERO output missing in {case_dir}: {', '.join(missing)}")

    def run(
        self,
        condition: dict[str, float],
        parent_dir: Path,
        label: str,
        *,
        stability: bool,
        include_thick: bool = True,
        control_deflections_deg: dict[str, float] | None = None,
        wake_iterations: int | None = None,
    ) -> AeroRunResult:
        start = time.perf_counter()
        case_dir = parent_dir.resolve() / label
        self._prepare_case(case_dir, control_deflections_deg)
        old_cwd = Path.cwd()
        try:
            os.chdir(case_dir)
            geometry_id = self._configure_geometry(include_thick)
            sweep_id = self._configure_sweep(
                case_dir, condition, stability, include_thick, wake_iterations
            )
            result_name = "VSPAERO_Stab" if stability else "VSPAERO_Polar"
            data_id = self.vsp.FindLatestResultsID(result_name)
            if not data_id:
                raise OpenVSPError(f"{result_name} Results ID is missing")
            raw = self._read_result(data_id)
        finally:
            os.chdir(old_cwd)
        self._validate_raw_files(case_dir, stability)
        return AeroRunResult(
            "stability" if stability else "polar", case_dir, geometry_id, sweep_id,
            data_id, raw, time.perf_counter() - start,
        )
