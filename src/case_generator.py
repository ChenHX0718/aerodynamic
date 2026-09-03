from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass
from decimal import Decimal
from itertools import product
from pathlib import Path
from typing import Any

from config_loader import ConfigError


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    mode: str
    speed_mps: float
    alpha_deg: float | None
    beta_deg: float

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def expand_axis(spec: dict[str, Any], name: str) -> list[float]:
    if "values" in spec:
        values = [float(value) for value in spec["values"]]
    else:
        start = Decimal(str(spec["start"]))
        step = Decimal(str(spec["step"]))
        end = Decimal(str(spec["end"]))
        values_decimal: list[Decimal] = []
        value = start
        limit = 1_000_000
        compare = (lambda item: item <= end) if step > 0 else (lambda item: item >= end)
        while compare(value):
            values_decimal.append(value)
            if len(values_decimal) > limit:
                raise ConfigError(f"{name} expands to more than {limit} points")
            value += step
        values = [float(item) for item in values_decimal]
    if not values:
        raise ConfigError(f"{name} generated no values")
    if name.endswith("speed") and any(value <= 0 for value in values):
        raise ConfigError(f"{name} values must be positive")
    if len(set(values)) != len(values):
        raise ConfigError(f"{name} contains duplicate values")
    return values


def _token(prefix: str, value: float, signed: bool = True) -> str:
    scaled = int(round(abs(float(value)) * 1000.0))
    if scaled > 999_999_999:
        raise ConfigError(f"Case ID value is too large: {prefix}={value}")
    sign = "P" if value >= 0 else "M"
    return f"{prefix}{sign if signed else ''}{scaled:09d}"


def grid_case_id(speed: float, alpha: float, beta: float) -> str:
    return "CASE_" + "_".join(
        (_token("V", speed, signed=False), _token("A", alpha), _token("B", beta))
    )


def trim_case_id(speed: float, beta: float) -> str:
    return "TRIM_" + "_".join((_token("V", speed, signed=False), _token("B", beta)))


def generate_grid_cases(config: dict[str, Any]) -> list[CaseSpec]:
    conditions = config["operating_conditions"]
    speeds = expand_axis(conditions["speed"], "operating_conditions.speed")
    alphas = expand_axis(conditions["alpha"], "operating_conditions.alpha")
    betas = expand_axis(conditions["beta"], "operating_conditions.beta")
    return [
        CaseSpec(grid_case_id(speed, alpha, beta), "GRID_DATABASE", speed, alpha, beta)
        for speed, alpha, beta in product(speeds, alphas, betas)
    ]


def generate_trim_cases(config: dict[str, Any]) -> list[CaseSpec]:
    conditions = config["trim"]["operating_conditions"]
    speeds = expand_axis(conditions["speed"], "trim.operating_conditions.speed")
    betas = expand_axis(conditions["beta"], "trim.operating_conditions.beta")
    return [
        CaseSpec(trim_case_id(speed, beta), "TRIM_DATABASE", speed, None, beta)
        for speed, beta in product(speeds, betas)
    ]


def stable_signature(payload: dict[str, Any]) -> str:
    serialized = json.dumps(payload, ensure_ascii=True, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(serialized.encode("utf-8")).hexdigest()


def load_completed_result(path: Path, signature: str) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if data.get("signature") != signature or data.get("status") not in {"PASS", "WARN", "CONVERGED"}:
        return None
    outputs = data.get("outputs")
    if not isinstance(outputs, dict):
        return None
    if not isinstance(outputs.get("coefficients"), dict) or not outputs["coefficients"]:
        return None
    diagnostics = outputs.get("native_derivative_diagnostics")
    if not isinstance(diagnostics, dict):
        return None
    if data.get("mode") == "TRIM_DATABASE":
        required_manifest = data.get("derivatives", {}).get("required_derivatives_manifest", {})
        if not isinstance(required_manifest.get("items"), list) or not required_manifest["items"]:
            return None
    return data
