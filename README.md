# sx-embodiments

The canonical Sentient-X hardware registry: shared, dependency-free records for identifying a
robot or capture-hardware embodiment and the immutable assets that describe it. The package owns
portable facts, not runtime behavior.

This is its own repository (`Sentient-X/embodiments`), consumed by the
[`glued`](https://github.com/Sentient-X/glued) superproject as a git submodule mounted at
`packages/sx-embodiments` (a uv workspace member; the import path is `sx_embodiments`). Its CI is
standalone — the package has zero runtime dependencies.

An `EmbodimentManifest` schema-v3 document carries identity, kind/lineage, exact flat channel
layout, cameras and their mounting frames, nominal projection/resolution, control rates, and
content-addressed assets with pinned source provenance. Every v3 manifest contains exactly one
authoritative URDF description. Its canonical JSON SHA-256 is the portable revision pin used by Worlds,
Autonomy, Fleet, RRD recordings, and the catalog. Packaged assets use stable
`package://sx-embodiments/...` URIs, so identical checkouts produce identical manifest digests.
Consumers remain responsible for resolving bytes and for runtime adapters such as forward
kinematics and simulator loading; they do not redefine embodiment facts.

External-corpus boundaries use `manifest_for_assets(...)`: callers provide only observed,
content-addressed asset references and the exact authoritative URDF bytes. The package re-hashes
the URDF and derives identity, layout, capabilities, cameras, rates, DoF, and link count from the
registered `EmbodimentSpec`; consumers cannot construct a parallel body description.

The digest is mandatory. A mutable URL is a location, not an asset identity; catalog registration
must hash bytes before producing an `AssetRef`.

## Assets

Canonical robot/capture-hardware *description* assets (URDF/MJCF and their meshes) live under
`assets/` at the repo root — see `THIRD_PARTY_NOTICES.md` for provenance and licensing. Wheels
and sdists include that tree under `sx_embodiments/_assets`, so an installed package is complete.
`sx_embodiments.assets.asset_root()` honors the `SX_EMBODIMENTS_ASSETS` environment variable,
then resolves the installed tree, then the repo-relative `assets/` directory for editable installs,
and otherwise raises a typed error (no silent fallback).

The episode-ready registry includes Piper, NERO, ALOHA, RBY1, Unitree G1, UR10e, UR5e, YOR, the Sentient
humanoid, Franka/Panda variants, SO-101 variants, DAS/YUBI capture rigs, and supported teleop
stations. Entries without authoritative kinematics are deliberately not advertised as
episode-ready. Declaration order is the wire action order and is pinned against the URDF joint set
by layout-law tests.
