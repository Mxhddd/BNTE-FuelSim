"""Minimal YAML fallback parser."""
from __future__ import annotations

import ast
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


def safe_load(text: str) -> Any:
    """Parse a minimal subset of YAML into Python objects."""

    lines = text.splitlines()
    first_line, _ = _next_meaningful_line(lines, -1)
    if first_line.startswith("- "):
        root: Any = []
    else:
        root = {}
    stack: List[Tuple[Any, int]] = [(root, -1)]

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
                key = key.strip()
                value_token = value_token.strip()
                item: Dict[str, Any] = {}
                parent.append(item)
                stack.append((item, indent))
                if value_token == "":
                    next_line, next_indent = _next_meaningful_line(lines, idx)
                    container: Any
                    if next_line.startswith("- "):
                        container = []
                    else:
                        container = {}
                    item[key] = container
                    if next_indent is None:
                        child_indent = indent + 2
                    else:
                        child_indent = next_indent
                    stack.append((container, child_indent - 1))
                else:
                    item[key] = _parse_value(value_token)
                continue
            parent.append(_parse_value(content))
            continue

        if ":" in stripped:
            key, value_token = stripped.split(":", 1)
            key = key.strip()
            value_token = value_token.strip()
            if value_token == "":
                next_line, next_indent = _next_meaningful_line(lines, idx)
                container: Any
                if next_line.startswith("- "):
                    container = []
                else:
                    container = {}
                parent[key] = container
                if next_indent is None:
                    child_indent = indent + 2
                else:
                    child_indent = next_indent
                stack.append((container, child_indent - 1))
            else:
                parent[key] = _parse_value(value_token)
        else:
            raise ValueError(f"Unsupported line in configuration: {raw}")

    return root


__all__ = ["safe_load"]
