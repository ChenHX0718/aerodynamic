from __future__ import annotations

import os
import shutil
import string
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ConfigError(RuntimeError):
    """Raised when a project configuration is missing or inconsistent."""


@dataclass(frozen=True)
class OpenVSPLocation:
    root: Path
    source: str
    checked: tuple[Path, ...]


def project_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ConfigError(f"Configuration file not found: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8-sig")) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"Invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"Top-level YAML value must be a mapping: {path}")
    return data


def load_openvsp_config(root: Path | None = None) -> dict[str, Any]:
    base = (root or project_root()).resolve()
    return _read_yaml(base / "config" / "openvsp.yaml")


def load_derivative_manifest(path: Path) -> dict[str, Any]:
    manifest = _read_yaml(path)
    policy = manifest.get("status_policy")
    if not isinstance(policy, dict) or set(policy) != {
        "PASS", "WARN_NUMERICAL", "METHOD_LIMITATION", "FAIL"
    }:
        raise ConfigError(
            "Derivative manifest status_policy must explicitly define PASS, "
            "WARN_NUMERICAL, METHOD_LIMITATION, and FAIL"
        )
    if any(str(policy[name]).upper() not in {"ACCEPT", "ACCEPT_WITH_WARNING", "REJECT"}
           for name in policy):
        raise ConfigError("Derivative manifest status_policy contains an invalid gate action")
    limitations = manifest.get("method_limitations")
    if not isinstance(limitations, dict):
        raise ConfigError("Derivative manifest method_limitations must be a mapping")
    rows = manifest.get("derivatives")
    if not isinstance(rows, list) or not rows:
        raise ConfigError(f"Derivative manifest has no derivatives: {path}")
    required_keys = {"name", "category", "required", "coefficient", "perturbation", "unit", "definition"}
    names: set[str] = set()
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise ConfigError(f"derivatives[{index}] must be a mapping in {path}")
        missing = sorted(required_keys - set(row))
        if missing:
            raise ConfigError(f"derivatives[{index}] is missing: {', '.join(missing)}")
        name = str(row["name"])
        if name in names:
            raise ConfigError(f"Duplicate derivative in manifest: {name}")
        names.add(name)
        if str(row["coefficient"]) not in {"CL", "CD", "CY", "Cl", "Cm", "Cn"}:
            raise ConfigError(f"Unsupported coefficient for {name}: {row['coefficient']}")
        if str(row["perturbation"]) not in {
            "alpha", "beta", "p", "q", "r", "aileron", "elevator", "rudder"
        }:
            raise ConfigError(f"Unsupported perturbation for {name}: {row['perturbation']}")
        expected_sign = row.get("expected_sign")
        if expected_sign not in {None, "positive", "negative"}:
            raise ConfigError(f"expected_sign for {name} must be positive or negative")
        limitation = row.get("method_limitation")
        if limitation is not None and str(limitation) not in limitations:
            raise ConfigError(f"Unknown method limitation for {name}: {limitation}")
        if str(row["perturbation"]) in {"p", "q", "r"} and limitation is None:
            raise ConfigError(f"Rate derivative {name} must declare its method limitation")
    manifest["_path"] = path.resolve()
    manifest["_by_name"] = {str(row["name"]): row for row in rows}
    manifest["_required"] = [row for row in rows if bool(row["required"])]
    return manifest


def _require_mapping(data: dict[str, Any], name: str) -> dict[str, Any]:
    value = data.get(name)
    if not isinstance(value, dict):
        raise ConfigError(f"Missing configuration section: {name}")
    return value


def _require_positive(section: dict[str, Any], name: str, prefix: str) -> float:
    if name not in section:
        raise ConfigError(f"{prefix}.{name} is required")
    value = float(section[name])
    if value <= 0:
        raise ConfigError(f"{prefix}.{name} must be positive")
    return value


def _validate_axis(axis: Any, name: str) -> None:
    if not isinstance(axis, dict):
        raise ConfigError(f"{name} must be a mapping")
    has_values = "values" in axis
    has_range = all(key in axis for key in ("start", "step", "end"))
    if has_values == has_range:
        raise ConfigError(f"{name} must define either values or start/step/end (but not both)")
    if has_values:
        values = axis["values"]
        if not isinstance(values, list) or not values:
            raise ConfigError(f"{name}.values must be a non-empty list")
        for value in values:
            float(value)
    else:
        start, step, end = (float(axis[key]) for key in ("start", "step", "end"))
        if step == 0:
            raise ConfigError(f"{name}.step cannot be zero")
        if (end - start) * step < 0:
            raise ConfigError(f"{name}.step points away from end")


def _validate_tolerance(settings: Any, prefix: str) -> None:
    if not isinstance(settings, dict):
        raise ConfigError(f"{prefix} must be a mapping")
    for name in (
        "near_zero_reference", "pass_relative", "warn_relative",
        "pass_absolute", "warn_absolute",
    ):
        _require_positive(settings, name, prefix)
    if float(settings["pass_relative"]) >= float(settings["warn_relative"]):
        raise ConfigError(f"{prefix}.pass_relative must be less than warn_relative")
    if float(settings["pass_absolute"]) >= float(settings["warn_absolute"]):
        raise ConfigError(f"{prefix}.pass_absolute must be less than warn_absolute")


def _validate_numerical_convergence(data: dict[str, Any]) -> None:
    numerical = _require_mapping(data, "numerical_convergence")
    wake = _require_mapping(numerical, "wake")
    candidates = [int(item) for item in wake.get("candidates", [])]
    if len(candidates) < 2 or any(item <= 0 for item in candidates) or candidates != sorted(set(candidates)):
        raise ConfigError("numerical_convergence.wake.candidates must be unique increasing positive integers")
    states = wake.get("representative_states")
    if not isinstance(states, list) or len(states) < 3:
        raise ConfigError("wake.representative_states must contain at least three aerodynamic states")
    names: set[str] = set()
    for index, state in enumerate(states):
        if not isinstance(state, dict) or not state.get("name"):
            raise ConfigError(f"wake.representative_states[{index}].name is required")
        name = str(state["name"])
        if name in names:
            raise ConfigError(f"Duplicate representative state name: {name}")
        names.add(name)
        _require_positive(state, "speed_mps", f"wake.representative_states[{index}]")
        if not bool(state.get("trim", False)) and "alpha_deg" not in state:
            raise ConfigError(f"wake.representative_states[{index}].alpha_deg is required unless trim=true")
        float(state.get("beta_deg", 0.0))
    _validate_tolerance(wake.get("tolerance"), "numerical_convergence.wake.tolerance")
    verification = int(wake.get("verification_only_level", 0))
    if verification <= candidates[-1] or verification in candidates:
        raise ConfigError(
            "wake.verification_only_level must be greater than, and separate from, production candidates"
        )
    if float(wake.get("boundary_buffer_normalized", 0.0)) <= 0:
        raise ConfigError("wake.boundary_buffer_normalized must be positive")
    if int(wake.get("safety_margin_levels_for_untested", -1)) < 0:
        raise ConfigError("wake.safety_margin_levels_for_untested cannot be negative")
    if int(wake.get("max_boundary_checks", 0)) <= 0:
        raise ConfigError("wake.max_boundary_checks must be positive")

    trim = _require_mapping(numerical, "trim")
    if int(trim.get("pretrim_wake_iterations", 0)) not in candidates:
        raise ConfigError("numerical_convergence.trim.pretrim_wake_iterations must be a Wake candidate")
    if int(trim.get("max_wake_upgrades", 0)) <= 0:
        raise ConfigError("numerical_convergence.trim.max_wake_upgrades must be positive")
    fd_step = _require_mapping(numerical, "fd_step")
    if str(fd_step.get("representative_state_name", "")) not in names:
        raise ConfigError("fd_step.representative_state_name must identify a Wake representative")
    gate = _require_mapping(numerical, "production_gate")
    if not isinstance(gate.get("enabled"), bool):
        raise ConfigError("numerical_convergence.production_gate.enabled must be boolean")


def load_project_config(config_path: str | Path | None = None) -> dict[str, Any]:
    root = project_root().resolve()
    path = Path(config_path).resolve() if config_path else root / "config" / "aircraft.yaml"
    data = _read_yaml(path)

    aircraft = _require_mapping(data, "aircraft")
    analysis = _require_mapping(data, "analysis")
    operating = _require_mapping(data, "operating_conditions")
    atmosphere = _require_mapping(data, "atmosphere")
    geometry_sets = _require_mapping(data, "geometry_sets")
    solver = _require_mapping(data, "solver")
    reference = _require_mapping(data, "reference")
    controls = _require_mapping(data, "controls")
    derivatives = _require_mapping(data, "derivatives")
    validation = _require_mapping(data, "validation")
    regression = _require_mapping(data, "regression")
    export = _require_mapping(data, "export")
    resume = _require_mapping(data, "resume")
    grid = _require_mapping(data, "grid")
    if str(grid.get("mode", "")).lower() not in {"uniform", "adaptive"}:
        raise ConfigError("grid.mode must be uniform or adaptive")
    _validate_numerical_convergence(data)

    model_value = aircraft.get("model")
    if not model_value:
        raise ConfigError("aircraft.model is required")
    model_path = (path.parent / str(model_value)).resolve()
    if not model_path.is_file():
        raise ConfigError(f"Aircraft model not found: {model_path}")

    modes = analysis.get("modes")
    if isinstance(modes, str):
        modes = [modes]
    if not isinstance(modes, list) or not modes:
        raise ConfigError("analysis.modes must be a non-empty list")
    normalized_modes = [str(item).upper() for item in modes]
    allowed_modes = {"GRID_DATABASE", "TRIM_DATABASE"}
    invalid = [item for item in normalized_modes if item not in allowed_modes]
    if invalid:
        raise ConfigError(f"Unsupported analysis mode(s): {', '.join(invalid)}")
    data["analysis"]["modes"] = normalized_modes

    for name in ("speed", "alpha", "beta"):
        _validate_axis(operating.get(name), f"operating_conditions.{name}")

    _require_positive(atmosphere, "rho_kg_m3", "atmosphere")
    _require_positive(atmosphere, "dynamic_viscosity_pa_s", "atmosphere")
    _require_positive(atmosphere, "speed_of_sound_mps", "atmosphere")

    for set_name in ("thin", "thick"):
        items = geometry_sets.get(set_name)
        if not isinstance(items, list):
            raise ConfigError(f"geometry_sets.{set_name} must be a list")
        for index, selector in enumerate(items):
            if not isinstance(selector, dict) or not any(
                selector.get(key) for key in ("id", "name", "type")
            ):
                raise ConfigError(f"geometry_sets.{set_name}[{index}] needs id, name, and/or type")
    if not geometry_sets["thin"]:
        raise ConfigError("geometry_sets.thin cannot be empty")
    thin_index = int(geometry_sets.get("thin_set_index", -1))
    thick_index = int(geometry_sets.get("thick_set_index", -1))
    if thin_index <= 0 or thick_index <= 0 or thin_index == thick_index:
        raise ConfigError("thin_set_index and thick_set_index must be different positive set indices")

    if int(solver.get("wake_iterations", 0)) <= 0:
        raise ConfigError("solver.wake_iterations must be positive")
    if int(solver.get("ncpu", 0)) <= 0:
        raise ConfigError("solver.ncpu must be positive")

    source = str(reference.get("source", "")).lower()
    if source not in {"model", "config"}:
        raise ConfigError("reference.source must be model or config")
    if source == "config":
        for name in ("sref_m2", "bref_m", "cref_m"):
            _require_positive(reference, name, "reference")
    cg_source = str(reference.get("cg_source", source)).lower()
    if cg_source not in {"model", "config"}:
        raise ConfigError("reference.cg_source must be model or config")
    if cg_source == "config":
        for name in ("xcg_m", "ycg_m", "zcg_m"):
            if name not in reference:
                raise ConfigError(f"reference.{name} is required when cg_source is config")

    for role in ("aileron", "elevator", "rudder"):
        item = controls.get(role)
        if not isinstance(item, dict) or not item.get("group"):
            raise ConfigError(f"controls.{role}.group is required")
        float(item.get("neutral_deg", 0.0))
        lower = float(item.get("min_deg", -90.0))
        upper = float(item.get("max_deg", 90.0))
        if lower >= upper:
            raise ConfigError(f"controls.{role}.min_deg must be less than max_deg")
        neutral = float(item.get("neutral_deg", 0.0))
        if not lower <= neutral <= upper:
            raise ConfigError(f"controls.{role}.neutral_deg must be inside min_deg/max_deg")

    if "TRIM_DATABASE" in normalized_modes:
        trim = _require_mapping(data, "trim")
        if not bool(trim.get("enabled", False)):
            raise ConfigError("TRIM_DATABASE is requested but trim.enabled is false")
        _require_positive(trim, "mass_kg", "trim")
        _require_positive(trim, "gravity_m_s2", "trim")
        trim_conditions = _require_mapping(trim, "operating_conditions")
        for name in ("speed", "beta"):
            _validate_axis(trim_conditions.get(name), f"trim.operating_conditions.{name}")
        for variable in ("alpha", "elevator"):
            bounds = _require_mapping(trim, variable)
            for key in ("initial_deg", "min_deg", "max_deg"):
                if key not in bounds:
                    raise ConfigError(f"trim.{variable}.{key} is required")
            if float(bounds["min_deg"]) >= float(bounds["max_deg"]):
                raise ConfigError(f"trim.{variable}.min_deg must be less than max_deg")
            if not float(bounds["min_deg"]) <= float(bounds["initial_deg"]) <= float(bounds["max_deg"]):
                raise ConfigError(f"trim.{variable}.initial_deg must be inside min_deg/max_deg")
        _require_positive(trim, "force_tolerance_n", "trim")
        _require_positive(trim, "moment_tolerance_nm", "trim")
        if int(trim.get("max_iterations", 0)) <= 0:
            raise ConfigError("trim.max_iterations must be positive")

    manifest_value = derivatives.get("manifest")
    if not manifest_value:
        raise ConfigError("derivatives.manifest is required")
    manifest_path = (path.parent / str(manifest_value)).resolve()
    manifest = load_derivative_manifest(manifest_path)
    candidates = _require_mapping(derivatives, "fd_step_candidates_deg")
    for variable in ("alpha", "beta", "aileron", "elevator", "rudder"):
        steps = [float(item) for item in candidates.get(variable, [])]
        if len(steps) < 3 or steps != sorted(set(steps)) or any(step <= 0 for step in steps):
            raise ConfigError(
                f"derivatives.fd_step_candidates_deg.{variable} must contain at least "
                "three unique increasing positive values"
            )
    jacobian_steps = _require_mapping(derivatives, "trim_jacobian_steps_deg")
    for variable in ("alpha", "elevator"):
        _require_positive(jacobian_steps, variable, "derivatives.trim_jacobian_steps_deg")
    convergence = _require_mapping(derivatives, "convergence")
    _validate_tolerance(convergence, "derivatives.convergence")

    baseline_value = regression.get("baseline")
    if not baseline_value:
        raise ConfigError("regression.baseline is required")
    baseline_path = (path.parent / str(baseline_value)).resolve()
    if not baseline_path.is_relative_to(root):
        raise ConfigError("regression.baseline must stay inside the project directory")
    _require_positive(regression, "speed_mps", "regression")
    for name in ("pass_relative", "warn_relative", "pass_absolute", "warn_absolute"):
        _require_positive(regression, name, "regression")
    if float(regression["pass_relative"]) >= float(regression["warn_relative"]):
        raise ConfigError("regression pass_relative must be less than warn_relative")
    if float(regression["pass_absolute"]) >= float(regression["warn_absolute"]):
        raise ConfigError("regression pass_absolute must be less than warn_absolute")

    output_root = Path(str(export.get("output_root", "../results")))
    results_path = (path.parent / output_root).resolve()
    if not results_path.is_relative_to(root):
        raise ConfigError("export.output_root must stay inside the project directory")

    data["_paths"] = {
        "project_root": root,
        "config_file": path,
        "model": model_path,
        "results": results_path,
        "manifest": manifest_path,
        "regression_baseline": baseline_path,
        "production_settings": results_path / "numerical_convergence" / "production_numerical_settings.yaml",
    }
    data["_manifest"] = manifest
    data["_resume_enabled"] = bool(resume.get("enabled", True))
    return data


def _common_openvsp_candidates() -> list[Path]:
    candidates: list[Path] = []
    program_dirs = {
        Path(os.environ.get("ProgramFiles", os.environ.get("SystemDrive", "C:") + os.sep + "Program Files")),
        Path(os.environ.get("ProgramFiles(x86)", os.environ.get("SystemDrive", "C:") + os.sep + "Program Files (x86)")),
        Path.home() / "OpenVSP",
    }
    for base in program_dirs:
        candidates.append(base / "OpenVSP")
        if base.is_dir():
            candidates.extend(sorted(base.glob("OpenVSP*"), reverse=True))
    for letter in string.ascii_uppercase:
        drive = Path(f"{letter}:/")
        if not drive.exists():
            continue
        parent = drive / "OpenVSP"
        candidates.append(parent)
        if parent.is_dir():
            candidates.extend(sorted(parent.glob("OpenVSP-*-win64"), reverse=True))
        candidates.extend(sorted(drive.glob("OpenVSP-*-win64"), reverse=True))
    return candidates


def locate_openvsp(openvsp_config: dict[str, Any] | None = None) -> OpenVSPLocation:
    config = openvsp_config or load_openvsp_config()
    checked: list[Path] = []
    proposed: list[tuple[str, Path]] = []
    configured = config.get("openvsp", {}).get("root") if isinstance(config.get("openvsp"), dict) else None
    if configured and str(configured).strip().lower() not in {"auto", "none"}:
        proposed.append(("config/openvsp.yaml", Path(str(configured)).expanduser()))
    env_root = os.environ.get("OPENVSP_ROOT")
    if env_root:
        proposed.append(("OPENVSP_ROOT", Path(env_root).expanduser()))
    path_vsp = shutil.which("vsp.exe") or shutil.which("vsp")
    if path_vsp:
        proposed.append(("Windows PATH", Path(path_vsp).resolve().parent))
    proposed.extend(("automatic common-directory search", path) for path in _common_openvsp_candidates())
    seen: set[str] = set()
    for source, candidate in proposed:
        candidate = candidate.resolve()
        key = os.path.normcase(str(candidate))
        if key in seen:
            continue
        seen.add(key)
        checked.append(candidate)
        if (candidate / "vsp.exe").is_file() and (candidate / "vspaero.exe").is_file():
            return OpenVSPLocation(candidate, source, tuple(checked))
    checked_text = "\n".join(f"  - {path}" for path in checked) or "  - no candidates"
    raise ConfigError(
        "OpenVSP root was not found. Checked:\n"
        f"{checked_text}\nSet openvsp.root in config/openvsp.yaml or define OPENVSP_ROOT."
    )
