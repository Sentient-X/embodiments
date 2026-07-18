# sx-embodiments

The canonical Sentient-X hardware registry: shared, dependency-free records for identifying a
robot or capture-hardware embodiment and the immutable assets that describe it. The package owns
portable facts, not runtime behavior.

This is its own repository (`Sentient-X/embodiments`), consumed by the
[`glued`](https://github.com/Sentient-X/glued) superproject as a git submodule mounted at
`packages/sx-embodiments` (a uv workspace member; the import path is `sx_embodiments`). Its CI is
standalone — the package has zero runtime dependencies.

An `EmbodimentManifest` schema-v2 document carries identity, kind/lineage, exact flat channel
layout, cameras, control rates, and content-addressed assets. Its canonical JSON SHA-256 is the
portable revision pin used by Worlds, Autonomy, Fleet, and the catalog. Packaged assets use stable
`package://sx-embodiments/...` URIs, so identical checkouts produce identical manifest digests.
Consumers remain responsible for resolving bytes and for their own adapters: forward kinematics,
simulator loading, validation policy, and Rerun visualization.

The digest is mandatory. A mutable URL is a location, not an asset identity; catalog registration
must hash bytes before producing an `AssetRef`.

## Assets

Canonical robot/capture-hardware *description* assets (URDF/MJCF and their meshes) live under
`assets/` at the repo root — see `THIRD_PARTY_NOTICES.md` for provenance and licensing. Wheels
and sdists include that tree under `sx_embodiments/_assets`, so an installed package is complete.
`sx_embodiments.assets.asset_root()` honors the `SX_EMBODIMENTS_ASSETS` environment variable,
then resolves the installed tree, then the repo-relative `assets/` directory for editable installs,
and otherwise raises a typed error (no silent fallback).

The registry includes Piper, ALOHA, RBY1, Unitree G1, UR10e, UR5e, YOR, the Sentient humanoid,
Franka/Panda variants, SO-101 variants, DAS/Quest/YUBI capture rigs, and the supported teleop
stations. Declaration order is the wire action order and is pinned by layout-law tests.
