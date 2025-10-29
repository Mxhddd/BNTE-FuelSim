"""Configuration loading utilities."""
from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Tuple


def _parse_value(token: str) -> Any:
    token = token.strip()
    if token == "" or token.lower() == "null":
        return None
    if token.lower() == "true":
        return True
    if token.lower() == "false":
        return False
    try:
        return ast.literal_eval(token)
    except Exception:
        return token


def _next_meaningful_line(lines: List[str], start: int) -> Tuple[str, int] | Tuple[str, None]:
    for idx in range(start + 1, len(lines)):
        stripped = lines[idx].strip()
        if stripped == "" or stripped.startswith("#"):
            continue
        indent = len(lines[idx]) - len(lines[idx].lstrip(" "))
        return stripped, indent
    return "", None


def _parse_simple_yaml(text: str) -> Dict[str, Any]:
    root: Dict[str, Any] = {}
    stack: List[Tuple[Any, int]] = [(root, -1)]
    lines = text.splitlines()

    for idx, raw in enumerate(lines):
        stripped = raw.strip()
        if stripped == "" or stripped.startswith("#"):
            continue
        indent = len(raw) - len(raw.lstrip(" "))
        while stack and indent <= stack[-1][1]:
            stack.pop()
        parent, _ = stack[-1]

        if stripped.startswith("- "):
            if not isinstance(parent, list):
                raise ValueError("List item without list parent in configuration")
            content = stripped[2:].strip()
            if content == "":
                item: Dict[str, Any] = {}
                parent.append(item)
                stack.append((item, indent))
                continue
            if ":" in content:
                key, value_token = content.split(":", 1)
                item = {key.strip(): _parse_value(value_token)}
                parent.append(item)
                stack.append((item, indent))
                if value_token.strip() == "":
                    next_line, _ = _next_meaningful_line(lines, idx)
                    if next_line.startswith("- "):
                        item[key.strip()] = []
                        stack.append((item[key.strip()], indent + 2))
                    else:
                        item[key.strip()] = {}
                        stack.append((item[key.strip()], indent + 2))
                continue
            parent.append(_parse_value(content))
            continue

        if ":" in stripped:
            key, value_token = stripped.split(":", 1)
            key = key.strip()
            value_token = value_token.strip()
            if value_token == "":
                next_line, _ = _next_meaningful_line(lines, idx)
                container: Any
                if next_line.startswith("- "):
                    container = []
                else:
                    container = {}
                parent[key] = container
                stack.append((container, indent))
            else:
                parent[key] = _parse_value(value_token)
        else:
            raise ValueError(f"Unsupported line in configuration: {raw}")

    return root


@dataclass
class Config:
    """Container for simulation configuration."""

    data: Dict[str, Any]

    @classmethod
    def from_file(cls, path: str | Path) -> "Config":
        with open(path, "r", encoding="utf-8") as stream:
            raw_text = stream.read()
        raw = _parse_simple_yaml(raw_text)
        return cls(data=raw)

    def section(self, name: str) -> Dict[str, Any]:
        if name not in self.data:
            raise KeyError(f"Missing configuration section '{name}'")
        value = self.data[name]
        if isinstance(value, dict):
            return dict(value)
        return value


__all__ = ["Config"]
