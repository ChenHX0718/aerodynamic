from __future__ import annotations

from typing import Any


# 本模块是 OpenVSP/VSPAERO -> internal/Simulink 坐标、符号和单位的唯一实现位置。
COORDINATE_CONVENTION: dict[str, Any] = {
    "internal_axes": "+X forward, +Y right, +Z down; right-handed body axes",
    "forces": "CX forward, CY right, CZ down; CL up and CD aft are positive",
    "moments": "Cl/Cm/Cn are right-hand moments about internal +X/+Y/+Z",
    "angles": "+alpha gives positive incoming-flow Z component; +beta gives positive incoming-flow Y component",
    "controls": (
        "+delta_a/+delta_e/+delta_r increase the matching OpenVSP Control Surface Group "
        "DeflectionAngle; physical trailing-edge direction depends on model gains and hinge orientation"
    ),
    "openvsp_axes": "+X aft, +Y right, +Z up in the geometry/aerodynamic component fields",
    "conversion": "CX=-CFx, CY=CFy, CZ=-CFz, Cl=-CMx=CMl, Cm=CMy=CMm, Cn=-CMz=CMn",
    "angle_unit": "configuration states use deg; derivative denominators use rad",
    "rate_definitions": {
        "p_hat": "p*bref/(2*V)",
        "q_hat": "q*cref/(2*V)",
        "r_hat": "r*bref/(2*V)",
    },
    "source_audit": (
        "OpenVSP 3.51.3 VSPAERO source CalculateStabilityDerivatives uses "
        "Delta_P*bref/(2*Vinf), Delta_Q*cref/(2*Vinf), Delta_R*bref/(2*Vinf); "
        "the same source writes CMl=-CMx and CMn=-CMz."
    ),
}


POLAR_FIELDS: dict[str, tuple[str, float]] = {
    "CX": ("CFxtot", -1.0),
    "CY": ("CFytot", 1.0),
    "CZ": ("CFztot", -1.0),
    "CL": ("CLtot", 1.0),
    "CD": ("CDtot", 1.0),
    "Cl": ("CMxtot", -1.0),
    "Cm": ("CMytot", 1.0),
    "Cn": ("CMztot", -1.0),
}

STABILITY_BASELINE_FIELDS: dict[str, tuple[str, float]] = {
    "CX": ("Base_Aero_CFx", -1.0),
    "CY": ("Base_Aero_CFy", 1.0),
    "CZ": ("Base_Aero_CFz", -1.0),
    "CL": ("Base_Aero_CL", 1.0),
    "CD": ("Base_Aero_CD", 1.0),
    "Cl": ("Base_Aero_CMl", 1.0),
    "Cm": ("Base_Aero_CMm", 1.0),
    "Cn": ("Base_Aero_CMn", 1.0),
}

STABILITY_CASE_FIELDS: dict[str, tuple[str, float]] = {
    "CX": ("CFx", -1.0), "CY": ("CFy", 1.0), "CZ": ("CFz", -1.0),
    "CL": ("CL", 1.0), "CD": ("CD", 1.0),
    "Cl": ("CMl", 1.0), "Cm": ("CMm", 1.0), "Cn": ("CMn", 1.0),
}

STABILITY_PREFIXES: dict[str, str] = {
    "CD": "CD", "CY": "CFy", "CL": "CL",
    "Cl": "CMl", "Cm": "CMm", "Cn": "CMn",
}

PERTURBATIONS: dict[str, str] = {
    "alpha": "Alpha", "beta": "Beta", "p": "p", "q": "q", "r": "r",
}

ROLE_SUFFIX = {"aileron": "a", "elevator": "e", "rudder": "r"}
RATE_CASE = {"p": "Roll__Rate", "q": "Pitch_Rate", "r": "Yaw___Rate"}
UNSTEADY_FIELDS: dict[str, tuple[str, str, bool]] = {
    "p": ("p", "p", False),
    "q": ("q+alpha_dot", "q_plus_alpha_dot", True),
    "r": ("r-beta_dot", "r_minus_beta_dot", True),
}


def measurement(
    *, raw_field: str, raw_value: float, raw_unit: str,
    standard_value: float, standard_unit: str, conversion: str,
) -> dict[str, Any]:
    return {
        "raw_field": raw_field,
        "raw_value": float(raw_value),
        "raw_unit": raw_unit,
        "standard_value": float(standard_value),
        "standard_unit": standard_unit,
        "conversion": conversion,
    }


def _coefficient_measurement(source: str, raw_field: str, raw_value: float, sign: float) -> dict[str, Any]:
    conversion = "none" if sign == 1.0 else COORDINATE_CONVENTION["conversion"]
    return measurement(
        raw_field=f"{source}.{raw_field}", raw_value=raw_value,
        raw_unit="dimensionless", standard_value=sign * raw_value,
        standard_unit="dimensionless", conversion=conversion,
    )


def _map_fields(raw: dict[str, Any], fields: dict[str, tuple[str, float]], source: str) -> dict[str, dict[str, Any]]:
    missing = [raw_field for raw_field, _ in fields.values() if raw_field not in raw]
    if missing:
        raise ValueError(f"{source} field(s) missing: {', '.join(missing)}")
    return {
        name: _coefficient_measurement(source, raw_field, float(raw[raw_field]), sign)
        for name, (raw_field, sign) in fields.items()
    }


def map_polar_coefficients(raw_polar: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return _map_fields(raw_polar, POLAR_FIELDS, "VSPAERO_Polar")


def map_stability_baseline(raw_stab: dict[str, Any]) -> dict[str, dict[str, Any]]:
    return _map_fields(raw_stab, STABILITY_BASELINE_FIELDS, "VSPAERO_Stab")


def map_stability_case(raw_stab: dict[str, Any], case_name: str) -> dict[str, dict[str, Any]]:
    fields = {
        name: (f"{case_name}_{raw_suffix}", sign)
        for name, (raw_suffix, sign) in STABILITY_CASE_FIELDS.items()
    }
    return _map_fields(raw_stab, fields, "VSPAERO_Stab")


def rate_unit(variable: str) -> str:
    return {"p": "1/p_hat", "q": "1/q_hat", "r": "1/r_hat"}[variable]


def map_stability_derivatives(raw_stab: dict[str, Any]) -> dict[str, dict[str, Any]]:
    mapped: dict[str, dict[str, Any]] = {}
    for standard_prefix, raw_prefix in STABILITY_PREFIXES.items():
        for variable, raw_suffix in PERTURBATIONS.items():
            raw_field = f"{raw_prefix}_{raw_suffix}"
            if raw_field not in raw_stab:
                continue
            unit = "1/rad" if variable in {"alpha", "beta"} else rate_unit(variable)
            mapped[f"{standard_prefix}_{variable}"] = measurement(
                raw_field=f"VSPAERO_Stab.{raw_field}", raw_value=float(raw_stab[raw_field]),
                raw_unit=unit, standard_value=float(raw_stab[raw_field]), standard_unit=unit,
                conversion="VSPAERO standard aerodynamic coefficient and documented derivative denominator",
            )
    return mapped


def map_control_derivatives(
    raw_stab: dict[str, Any], control_groups: dict[str, str],
) -> dict[str, dict[str, Any]]:
    controls: dict[str, dict[str, Any]] = {}
    for role, group_name in control_groups.items():
        suffix = ROLE_SUFFIX[role]
        derivatives: dict[str, dict[str, Any]] = {}
        for standard_prefix, raw_prefix in STABILITY_PREFIXES.items():
            raw_field = f"{raw_prefix}_{group_name}"
            if raw_field not in raw_stab:
                continue
            name = f"{standard_prefix}_delta_{suffix}"
            derivatives[name] = measurement(
                raw_field=f"VSPAERO_Stab.{raw_field}", raw_value=float(raw_stab[raw_field]),
                raw_unit="1/rad", standard_value=float(raw_stab[raw_field]), standard_unit="1/rad",
                conversion="positive group DeflectionAngle, per radian",
            )
        controls[role] = {"group_name": group_name, "derivatives": derivatives}
    return controls


def map_unsteady_derivatives(raw_stab: dict[str, Any], mode: str) -> dict[str, dict[str, Any]]:
    """Map P/Q/R damping-analysis outputs without treating combined terms as classical rates."""
    if mode not in UNSTEADY_FIELDS:
        raise ValueError(f"Unsupported unsteady stability mode: {mode}")
    raw_suffix, standard_suffix, combined = UNSTEADY_FIELDS[mode]
    prefixes = {
        "CL": "CL", "CD": "CD", "CY": "CFy",
        "Cl": "CMl", "Cm": "CMm", "Cn": "CMn",
    }
    mapped: dict[str, dict[str, Any]] = {}
    for coefficient, raw_prefix in prefixes.items():
        raw_field = f"{raw_prefix}_{raw_suffix}"
        if raw_field not in raw_stab:
            continue
        name = f"{coefficient}_{standard_suffix}_unsteady"
        mapped[name] = {
            **measurement(
                raw_field=f"VSPAERO_Stab.{raw_field}",
                raw_value=float(raw_stab[raw_field]),
                raw_unit="VSPAERO damping derivative",
                standard_value=float(raw_stab[raw_field]),
                standard_unit="VSPAERO damping derivative",
                conversion="VSPAERO standard aerodynamic coefficient axes; no classical-rate substitution",
            ),
            "analysis": mode.upper(),
            "source_expression": raw_suffix,
            "combined_derivative": combined,
            "diagnostic_only": True,
            "production_included": False,
        }
    return mapped


def standard_values(items: dict[str, dict[str, Any]]) -> dict[str, float]:
    return {name: float(item["standard_value"]) for name, item in items.items()}
