from __future__ import annotations

from textwrap import dedent

import pytest

from bnte import config as config_mod
from bnte._yaml_fallback import safe_load as fallback_safe_load


NESTED_YAML = dedent(
    """
    sequence:
      - stage:
          mass: 1
          thrust: 42
      - stage:
          mass: 2
          thrust: 84
    """
)


def _parse_with_config(text: str) -> dict[str, object]:
    # Ensure the loader always returns a dictionary for the test fixture
    loaded = config_mod._load_yaml(text)
    assert isinstance(loaded, dict)
    return loaded


def test_fallback_handles_nested_list_of_mappings(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(config_mod, "yaml", None)
    data = _parse_with_config(NESTED_YAML)
    stages = data["sequence"]
    assert isinstance(stages, list)
    assert stages[0]["stage"]["mass"] == 1
    assert stages[1]["stage"]["thrust"] == 84


def test_yaml_and_fallback_equivalence() -> None:
    yaml_mod = pytest.importorskip("yaml")
    expected = yaml_mod.safe_load(NESTED_YAML)
    assert expected == fallback_safe_load(NESTED_YAML)
