"""The solve's registry fixture is every registry body with a tool frame, as rendered."""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("numpy")

TOOL = Path(__file__).resolve().parents[1] / "tools/render_ik_registry.py"


def test_the_ik_registry_fixture_is_current() -> None:
    """Regenerate with `python tools/render_ik_registry.py` when this fails."""
    spec = importlib.util.spec_from_file_location("render_ik_registry", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.stale() == []
