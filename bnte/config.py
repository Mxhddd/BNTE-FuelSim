"""Configuration loading utilities."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict

try:  # pragma: no cover - import guarded for optional dependency
    import yaml  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - fallback tested separately
    yaml = None

from ._yaml_fallback import safe_load as _fallback_safe_load


def _load_yaml(text: str) -> Dict[str, Any]:
    if yaml is not None:
        loaded = yaml.safe_load(text)
        if loaded is None:
            return {}
        return loaded
    return _fallback_safe_load(text)


@dataclass
class Config:
    """Container for simulation configuration."""

    data: Dict[str, Any]

    @classmethod
    def from_file(cls, path: str | Path) -> "Config":
        with open(path, "r", encoding="utf-8") as stream:
            raw_text = stream.read()
        raw = _load_yaml(raw_text)
        return cls(data=raw)

    def section(self, name: str) -> Dict[str, Any]:
        if name not in self.data:
            raise KeyError(f"Missing configuration section '{name}'")
        value = self.data[name]
        if isinstance(value, dict):
            return dict(value)
        return value


__all__ = ["Config"]
