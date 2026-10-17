"""Driver crates compose known bodies from exactly the registry's bindings."""

import importlib.util
import json
from pathlib import Path

TOOL = Path(__file__).resolve().parents[1] / "tools/render_driver_bindings.py"


def test_every_rendered_bindings_file_is_current() -> None:
    """Regenerate with `python tools/render_driver_bindings.py` when this fails."""
    spec = importlib.util.spec_from_file_location("render_driver_bindings", TOOL)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.stale() == []


def test_the_damiao_transcripts_were_minted_from_the_current_bindings() -> None:
    """Re-mint with `drivers/sx-damiao-can/tests/fixtures/dm_can/mint.py` when this fails."""
    root = TOOL.parents[1] / "drivers/sx-damiao-can"
    transcripts = json.loads((root / "tests/fixtures/dm_can/transcripts.json").read_text())
    for minted in transcripts["bindings"]:
        current = json.loads((root / "bindings" / f"{minted['embodiment']}.json").read_text())
        assert minted["id"] == current["id"], minted["embodiment"]
