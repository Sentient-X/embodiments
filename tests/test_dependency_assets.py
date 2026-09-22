"""Pinned source closure must survive checkout and wheel packaging byte for byte."""

import hashlib
import json

from sx_embodiments.assets import asset_root


def test_restored_description_dependencies_match_their_source_manifest():
    root = asset_root()
    rows = json.loads((root / "dependency-assets.json").read_text())
    assert rows
    assert len({row["path"] for row in rows}) == len(rows)
    for row in rows:
        data = (root / row["path"]).read_bytes()
        assert len(data) == row["byte_size"], row["path"]
        assert hashlib.sha256(data).hexdigest() == row["sha256"], row["path"]
        assert len(row["revision"]) == 40
        assert row["repository"].startswith("https://github.com/")
        assert row["license"]
