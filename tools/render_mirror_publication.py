#!/usr/bin/env python3
"""Render the two asset-publication file lists, split by declared audience.

The mirror is two renderings of one tree, because the tree holds two kinds of thing:
upstream robot descriptions under permissive licences, and this company's own hardware
geometry. Publishing them together to a public-read bucket gives away the CAD.

    python tools/render_mirror_publication.py --out build/mirror

Writes `public.txt` and `entitled.txt`, each an rclone `--files-from` list. The entitled
list is the FULL tree, deliberately: an entitled customer configures exactly one mirror
URL and finds everything there, so no client ever needs a second source to fall back to.

Audience comes from each asset's declared `license_id` (see `sx_embodiments.assets`),
never from a list kept here — a hand-kept list is a second place to forget. A file in a
directory no declaration claims is fatal: the safe reading of "nobody said" is never
"public".
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from sx_embodiments.assets import AssetAudience, asset_root
from sx_embodiments.known import asset_audiences


class UnclassifiedAssetError(RuntimeError):
    """A file on disk that no embodiment declaration accounts for."""


def render(root: Path) -> tuple[list[str], list[str]]:
    """Return (public, entitled) relative paths; entitled is the whole tree."""
    audiences = asset_audiences()
    public: list[str] = []
    entitled: list[str] = []
    unclassified: list[str] = []
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relpath = path.relative_to(root).as_posix()
        directory = relpath.split("/", 1)[0]
        known = audiences.get(directory)
        if known is None:
            unclassified.append(relpath)
            continue
        entitled.append(relpath)
        if known is AssetAudience.PUBLIC:
            public.append(relpath)
    if unclassified:
        shown = "\n  ".join(unclassified[:10])
        raise UnclassifiedAssetError(
            f"{len(unclassified)} asset files belong to no declared embodiment:\n  {shown}\n"
            "Declare them, or remove them from the tree. They cannot be published."
        )
    return public, entitled


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=Path("build/mirror"))
    arguments = parser.parse_args()

    root = asset_root()
    public, entitled = render(root)
    arguments.out.mkdir(parents=True, exist_ok=True)
    (arguments.out / "public.txt").write_text("\n".join(public) + "\n", encoding="utf-8")
    (arguments.out / "entitled.txt").write_text("\n".join(entitled) + "\n", encoding="utf-8")

    # The two files are this tool's output; the summary is a diagnostic, so it goes to
    # stderr and never contaminates a caller that pipes the lists.
    held_back = len(entitled) - len(public)
    sys.stderr.write(
        f"asset root: {root}\n"
        f"  public.txt   {len(public):5d} files\n"
        f"  entitled.txt {len(entitled):5d} files ({held_back} withheld from public)\n"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except UnclassifiedAssetError as error:
        sys.stderr.write(f"error: {error}\n")
        sys.exit(1)
