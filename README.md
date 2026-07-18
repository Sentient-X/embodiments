# sx-embodiments

The canonical Sentient-X hardware registry: shared, dependency-free records for identifying a
robot or capture-hardware embodiment and the immutable assets that describe it. The package owns
portable facts, not runtime behavior.

This is its own repository (`Sentient-X/embodiments`), consumed by the
[`glued`](https://github.com/Sentient-X/glued) superproject as a git submodule mounted at
`packages/sx-embodiments` (a uv workspace member; the import path is `sx_embodiments`). Its CI is
standalone — the package has zero runtime dependencies.

An `EmbodimentManifest` can reference URDF, MJCF, USD, meshes, calibration bundles, and related
assets by URI and SHA-256 digest. Consumers remain responsible for fetching assets and for their
own adapters: forward kinematics, simulator loading, validation policy, and Rerun visualization.

The digest is mandatory. A mutable URL is a location, not an asset identity; catalog registration
must hash bytes before producing an `AssetRef`.

## Assets

Canonical robot/capture-hardware *description* assets (URDF/MJCF and their meshes) live under
`assets/` at the repo root — see `THIRD_PARTY_NOTICES.md` for provenance and licensing. They are
deliberately excluded from wheels and sdists; code resolves them through
`sx_embodiments.assets.asset_root()`, which honors the `SX_EMBODIMENTS_ASSETS` environment
variable, falls back to the repo-relative `assets/` directory for editable/workspace installs,
and otherwise raises a typed error (no silent fallback).
