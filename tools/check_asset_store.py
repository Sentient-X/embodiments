#!/usr/bin/env python3
"""Prove the asset store serves every digest the manifest names, through its front door.

    SX_EMBODIMENTS_ASSET_STORE=https://catalog.sentientx.io/api/embodiment-assets \\
    SX_EMBODIMENTS_ASSET_STORE_TOKEN=... python tools/check_asset_store.py

Each distinct digest is read one byte at a time with the configured bearer, so the check
exercises what a fetch meets — the credential, the audience decision, the signed redirect
and the object behind it — for about one request per digest. It exits 1 naming each
digest the store refused or lacks. A public-only credential is refused every entitled
digest by design; the complete check runs with an entitled one.
"""

from __future__ import annotations

import sys
from collections.abc import Iterable
from concurrent.futures import ThreadPoolExecutor

from sx_contracts.content import ContentBlob

from sx_embodiments.assets import asset_manifest, probe_store
from sx_embodiments.errors import AssetsUnavailableError

_WIDTH = 16


def unserved(contents: Iterable[ContentBlob]) -> list[str]:
    """Every refusal the store gives for ``contents``, in digest order; empty when all serve."""

    def probe(content: ContentBlob) -> str | None:
        try:
            probe_store(content)
        except AssetsUnavailableError as error:
            return str(error)
        return None

    ordered = sorted(set(contents), key=lambda content: content.sha256)
    with ThreadPoolExecutor(max_workers=_WIDTH) as pool:
        return [refusal for refusal in pool.map(probe, ordered) if refusal is not None]


def main() -> int:
    contents = {entry.content for entry in asset_manifest().values()}
    refusals = unserved(contents)
    for refusal in refusals:
        sys.stderr.write(f"{refusal}\n")
    sys.stdout.write(f"{len(contents) - len(refusals)} of {len(contents)} digests served\n")
    return 1 if refusals else 0


if __name__ == "__main__":
    sys.exit(main())
