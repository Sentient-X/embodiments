"""Driver crates compose known bodies from exactly the registry's bindings."""

import importlib.util
from pathlib import Path

TOOL = Path(__file__).resolve().parents[1] / "tools/render_driver_bindings.py"


def test_every_rendered_bindings_file_is_current() -> None:
    """Regenerate with `python tools/render_driver_bindings.py` when this fails."""
    spec = importlib.util.spec_from_file_location("render_driver_bindings", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.stale() == []
