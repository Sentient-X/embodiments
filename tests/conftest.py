"""Every test runs against its own asset cache, never the developer's."""

from pathlib import Path

import pytest


@pytest.fixture(autouse=True)
def _isolated_asset_cache(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("SX_EMBODIMENTS_ASSET_CACHE", str(tmp_path / "sx-embodiments-cache"))
