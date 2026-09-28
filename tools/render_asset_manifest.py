#!/usr/bin/env python3
"""Render ``sx_embodiments/asset-manifest.json`` from the asset tree and the declarations.

    python tools/render_asset_manifest.py           # rewrite it
    python tools/render_asset_manifest.py --check   # exit 1 when it disagrees with either

The manifest is generated, never edited: every file under ``assets/`` with its sha256, size
and ``license_id`` (see ``sx_embodiments.manifest``). ``tests/test_asset_manifest.py``
runs the same check.
"""

from __future__ import annotations

import argparse
import sys

from sx_embodiments.assets import MANIFEST_PATH, asset_root, render_manifest_json
from sx_embodiments.manifest import (
    ManifestDisagreementError,
    manifest_disagreements,
    render_manifest,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="compare instead of writing")
    arguments = parser.parse_args()
    root = asset_root()
    if arguments.check:
        disagreements = manifest_disagreements(root)
        for line in disagreements:
            sys.stderr.write(f"{line}\n")
        return 1 if disagreements else 0
    try:
        entries = render_manifest(root)
    except ManifestDisagreementError as error:
        sys.stderr.write(f"error: {error}\n")
        return 1
    MANIFEST_PATH.write_text(render_manifest_json(entries), encoding="utf-8")
    sys.stderr.write(f"{MANIFEST_PATH.name}: {len(entries)} files\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
