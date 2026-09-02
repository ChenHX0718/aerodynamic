from __future__ import annotations

import importlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType
from typing import Any


class OpenVSPError(RuntimeError):
    """Raised for an OpenVSP API, model, or solver validation failure."""


_DLL_HANDLES: list[Any] = []


def load_openvsp_api(openvsp_root: Path) -> ModuleType:
    root = openvsp_root.resolve()
    python_root = root / "python"
    package_root = python_root / "openvsp"
    binding = package_root / "openvsp" / "_vsp.pyd"
    if not binding.is_file():
        raise OpenVSPError(f"OpenVSP Python binding not found: {binding}")
    if hasattr(os, "add_dll_directory"):
        for dll_dir in (root, package_root / "openvsp"):
            if dll_dir.is_dir():
                _DLL_HANDLES.append(os.add_dll_directory(str(dll_dir)))
    package_dirs = [path for path in python_root.iterdir() if path.is_dir()]
    for path in [package_root, *package_dirs]:
        text = str(path)
        if text not in sys.path:
            sys.path.insert(0, text)
    try:
        vsp = importlib.import_module("openvsp")
    except Exception as exc:
        raise OpenVSPError(
            "OpenVSP Python API import failed. The bundled binding must match the active "
            f"Python version ({sys.version.split()[0]}). Binding path: {binding}. Error: {exc}"
        ) from exc
    if not vsp.CheckForVSPAERO(str(root)):
        raise OpenVSPError(f"OpenVSP could not validate VSPAERO in: {root}")
    vsp.SetVSPAEROPath(str(root))
    return vsp


def pop_api_errors(vsp: ModuleType) -> list[str]:
    manager = vsp.ErrorMgrSingleton.getInstance()
    errors: list[str] = []
    while manager.GetNumTotalErrors() > 0:
        errors.append(manager.PopLastError().GetErrorString())
    return errors


@dataclass(frozen=True)
class ControlGroup:
    index: int
    name: str
    surfaces: tuple[str, ...]
    gains: tuple[float, ...]


@dataclass(frozen=True)
class GeometrySelection:
    thin_set_index: int
    thick_set_index: int
    thin: tuple[dict[str, str], ...]
    thick: tuple[dict[str, str], ...]


class OpenVSPModel:
    def __init__(self, vsp: ModuleType, model_path: Path):
        self.vsp = vsp
        self.model_path = model_path.resolve()

    def load(self) -> None:
        self.vsp.ClearVSPModel()
        self.vsp.DeleteAllResults()
        pop_api_errors(self.vsp)
        self.vsp.ReadVSPFile(str(self.model_path))
        self.vsp.Update()
        errors = pop_api_errors(self.vsp)
        if errors:
            raise OpenVSPError("OpenVSP failed to load the model:\n" + "\n".join(errors))
        if not list(self.vsp.FindGeoms()):
            raise OpenVSPError(f"The model contains no geometry: {self.model_path}")

    def geometries(self) -> list[dict[str, str]]:
        return [
            {"id": geom_id, "name": self.vsp.GetGeomName(geom_id), "type": self.vsp.GetGeomTypeName(geom_id)}
            for geom_id in self.vsp.FindGeoms()
        ]

    def _group_gains(self, index: int) -> tuple[float, ...]:
        container_id = self.vsp.FindContainer("VSPAEROSettings", 0)
        if not container_id:
            return ()
        group_name = f"ControlSurfaceGroup_{index}"
        gains: list[tuple[str, float]] = []
        for parm_id in self.vsp.FindContainerParmIDs(container_id):
            parm_name = self.vsp.GetParmName(parm_id)
            if (
                parm_name.lower().endswith("_gain")
                and self.vsp.GetParmDisplayGroupName(parm_id) == group_name
            ):
                gains.append((parm_name, float(self.vsp.GetParmVal(parm_id))))
        return tuple(value for _, value in sorted(gains))

    def control_groups(self) -> list[ControlGroup]:
        return [
            ControlGroup(
                index=index,
                name=self.vsp.GetVSPAEROControlGroupName(index),
                surfaces=tuple(self.vsp.GetActiveCSNameVec(index)),
                gains=self._group_gains(index),
            )
            for index in range(self.vsp.GetNumControlSurfaceGroups())
        ]

    def validate_control_mapping(self, controls: dict[str, dict[str, Any]]) -> dict[str, ControlGroup]:
        by_name = {group.name: group for group in self.control_groups()}
        mapped: dict[str, ControlGroup] = {}
        for role in ("aileron", "elevator", "rudder"):
            configured = str(controls[role]["group"])
            if configured not in by_name:
                available = "\n".join(f"  - {name}" for name in by_name) or "  - none"
                raise OpenVSPError(
                    f'Configured {role} group "{configured}" was not found.\nAvailable groups:\n{available}'
                )
            if not by_name[configured].surfaces:
                raise OpenVSPError(f'Control group "{configured}" contains no active surfaces')
            mapped[role] = by_name[configured]
        return mapped

    @staticmethod
    def _matches(geometry: dict[str, str], selector: dict[str, Any]) -> bool:
        return all(
            str(geometry[key]).casefold() == str(value).casefold()
            for key, value in selector.items()
            if key in {"id", "name", "type"} and value not in {None, ""}
        )

    def _resolve_selectors(self, selectors: list[dict[str, Any]], label: str) -> tuple[dict[str, str], ...]:
        inventory = self.geometries()
        selected: list[dict[str, str]] = []
        for selector in selectors:
            matches = [item for item in inventory if self._matches(item, selector)]
            if len(matches) != 1:
                description = ", ".join(f"{key}={value}" for key, value in selector.items())
                available = "; ".join(
                    f"{item['name']} [{item['type']}, {item['id']}]" for item in inventory
                )
                raise OpenVSPError(
                    f"{label} geometry selector ({description}) matched {len(matches)} items. Available: {available}"
                )
            if matches[0]["id"] in {item["id"] for item in selected}:
                raise OpenVSPError(f"Duplicate geometry in {label}: {matches[0]['name']}")
            selected.append(matches[0])
        return tuple(selected)

    def resolve_geometry_sets(self, geometry_config: dict[str, Any]) -> GeometrySelection:
        thin = self._resolve_selectors(geometry_config["thin"], "thin")
        thick = self._resolve_selectors(geometry_config["thick"], "thick")
        overlap = {item["id"] for item in thin} & {item["id"] for item in thick}
        if overlap:
            names = [item["name"] for item in self.geometries() if item["id"] in overlap]
            raise OpenVSPError(f"Geometry cannot be both thin and thick: {', '.join(names)}")
        return GeometrySelection(
            thin_set_index=int(geometry_config["thin_set_index"]),
            thick_set_index=int(geometry_config["thick_set_index"]),
            thin=thin,
            thick=thick,
        )

    def apply_geometry_sets(self, selection: GeometrySelection) -> None:
        for geom in self.geometries():
            self.vsp.SetSetFlag(geom["id"], selection.thin_set_index, False)
            self.vsp.SetSetFlag(geom["id"], selection.thick_set_index, False)
        for geom in selection.thin:
            self.vsp.SetSetFlag(geom["id"], selection.thin_set_index, True)
        for geom in selection.thick:
            self.vsp.SetSetFlag(geom["id"], selection.thick_set_index, True)
        self.vsp.SetSetName(selection.thin_set_index, "AERO_THIN")
        self.vsp.SetSetName(selection.thick_set_index, "AERO_THICK")
        self.vsp.Update()

    def apply_tessellation_overrides(self, overrides: list[dict[str, Any]]) -> None:
        parameter_names = {"tess_u": "Tess_U", "tess_w": "Tess_W"}
        for override in overrides:
            selector = {key: override[key] for key in ("id", "name", "type") if override.get(key)}
            geometry = self._resolve_selectors([selector], "tessellation")[0]
            for config_name, parm_name in parameter_names.items():
                if config_name not in override:
                    continue
                parm_id = self.vsp.GetParm(geometry["id"], parm_name, "Shape")
                if not parm_id:
                    raise OpenVSPError(
                        f"Geometry {geometry['name']} has no Shape/{parm_name} parameter"
                    )
                self.vsp.SetParmVal(parm_id, float(override[config_name]))
        self.vsp.Update()

    def reference_quantities(self) -> dict[str, float]:
        container_id = self.vsp.FindContainer("VSPAEROSettings", 0)
        if not container_id:
            raise OpenVSPError("VSPAEROSettings container is missing from the model")
        mapping = {
            "sref_m2": "Sref", "bref_m": "bref", "cref_m": "cref",
            "xcg_m": "Xcg", "ycg_m": "Ycg", "zcg_m": "Zcg",
        }
        values: dict[str, float] = {}
        for output_name, parm_name in mapping.items():
            parm_id = self.vsp.FindParm(container_id, parm_name, "VSPAERO")
            if not parm_id:
                raise OpenVSPError(f"Model reference parameter is missing: {parm_name}")
            values[output_name] = float(self.vsp.GetParmVal(parm_id))
        for name in ("sref_m2", "bref_m", "cref_m"):
            if values[name] <= 0:
                raise OpenVSPError(f"Model reference quantity must be positive: {name}={values[name]}")
        return values

    def set_control_deflection(self, group_name: str, deflection_deg: float) -> None:
        matches = [group for group in self.control_groups() if group.name == group_name]
        if len(matches) != 1:
            raise OpenVSPError(f'Expected one control group named "{group_name}", found {len(matches)}')
        container_id = self.vsp.FindContainer("VSPAEROSettings", 0)
        parm_id = self.vsp.FindParm(container_id, "DeflectionAngle", f"ControlSurfaceGroup_{matches[0].index}")
        if not parm_id:
            raise OpenVSPError(f'DeflectionAngle is missing for control group "{group_name}"')
        self.vsp.SetParmVal(parm_id, float(deflection_deg))
        self.vsp.Update()

    def write_case_model(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.vsp.SetVSP3FileName(str(path.resolve()))
        self.vsp.WriteVSPFile(str(path.resolve()), self.vsp.SET_ALL)
