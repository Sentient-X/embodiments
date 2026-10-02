# sx-embodiments

The canonical Sentient-X hardware registry. The normal API is ordinary Python:

```python
from sx_embodiments import embodiments

robot = embodiments["franka"]

robot.id            # content identity for compact boundaries
robot.components    # the physical component graph
robot.state         # derived ordered, named native coordinates
robot.cameras       # derived nominal camera bindings
robot.capabilities  # derived component capabilities
robot.urdf          # authoritative content-addressed description
```

`Embodiment` is one complete immutable hardware revision. Registry lookup, validated
assembly, and composition return the same object. Storage boundaries carry `robot.id`;
code that needs physical facts receives the complete body.

Compose any number of existing bodies by placing named instances. A pair is an ordinary
composition; parts keep their identity, local coordinates, optics and drive facts.

```python
from sx_embodiments import PlacedEmbodiment, compose_embodiments, embodiments

robot = compose_embodiments(
    "workcell",
    {
        "left": PlacedEmbodiment(embodiments["so101"], xyz=(-0.3, 0.0, 0.0)),
        "right": PlacedEmbodiment(embodiments["so101"], xyz=(0.3, 0.0, 0.0)),
    },
    label="Workcell",
)
```

Component instances and URDF frames are namespaced. `robot.joint_names` resolves the
unique description joints in native state order. Composition writes its generated URDF
to the asset cache under its digest. Sharing it with another host requires distributing
the generated URDF and its dependency assets; a local JSON round-trip does not prove
Catalog publication or remote asset availability. The assembly grammar also accepts camera
and force/torque parts; camera optics remain required facts on each sensor part.

Observation and controller semantics are owned by `sx-actions`, which depends on this
package and derives spaces from the complete object:

```python
from sx_actions import embodiment_spaces, joint_position, observation_space

observation = observation_space(robot)
observation.state_shape       # (12,)
observation.image_shapes      # each named camera's native raster
observation.cameras           # optics, modality, frame and rate
commands = joint_position(robot, control_hz=30)
commands.channels             # exact component, coordinate, unit and bounds

inventory = embodiment_spaces(control_hz=30)
# Every known definition, including development records, with kind,
# collection_method (UMI/Ego/Teleop when declared), spaces and blockers.
```

A declared space does not establish a simulator, and a qualified direct drive names its
driver only through its bus (see [Drivers](#drivers)); whether a body runs on a given station
is checked by the execution owner against the exact body and requested interface.

Schema v14 stores one topologically ordered component graph plus nominal rates,
content-addressed assets, and independent per-axis observation and actuation facts. A
joint axis may be observed through actuator feedback, a vendor interface, or an encoder
and may be driven directly, through an integrated controller, passively, or remain
explicitly undocumented. A qualified direct drive binds an `ActuatorModel` on an
`ActuatorBus` with its chain address and drive map. State order, camera bindings, capabilities, arm/gripper convenience
views, and the ID are derived from those facts. This keeps one source for morphology while
remaining ergonomic for drivers, simulation, training, and task admission.

Controller semantics stay in `sx-actions`; one embodiment can expose several action interfaces
without changing hardware identity. Per-unit calibration and runtime status belong to episodes
or sessions, not the nominal embodiment.

External-corpus ingest starts from a registered object and replaces only its verified asset
bundle:

```python
from sx_contracts import ProvenancedAsset
from sx_embodiments import Embodiment, embodiments


def bind_external_assets(
    assets: tuple[ProvenancedAsset, ...], urdf_bytes: bytes
) -> Embodiment:
    return embodiments["franka"].with_assets(assets, urdf=urdf_bytes)
```

The caller supplies provenance-bearing, content-addressed assets and exact URDF bytes; the registered component
semantics remain authoritative. A mutable URL is a location, not asset identity, so ingest
boundaries hash bytes before producing an `AssetRef`.

## Drivers

This repository holds actuator drivers as Rust crates under `drivers/`, one per bus, in one
Cargo workspace pinned to the station's toolchain (`rust-toolchain.toml`).
`drivers/sx-embodiment-drivers` is the surface each implements: `ActuatorChain` drives the
actuators an axis's `ActuatorBinding` names, indexed in the body's native state order, and a
stop becomes a `ProvenStop` only when every actuator answered stopped.

A bus crate is a translation of its vendor's own driver — the same structure, names, frames,
clamps and command order — pinned to transcripts recorded by running that driver, with each
departure listed in the ported module's header. A crate that composes a known body holds its
axes equal to the bindings `tools/render_driver_bindings.py` renders from the registry.

| Crate | Bus | Vendor driver it translates | Consumer |
|---|---|---|---|
| `sx-damiao-can` | `damiao_can` | Damiao's `DM_CAN.py` ([kit-miao/motor-sdk](https://gitee.com/kit-miao/motor-sdk) `fb0e9fc5`) | intended: the `sx` station's B601 actuator, switched to this crate in the lane that lands right after this crate and the `sx` pin advance to it; nothing consumes it before then |

The `feetech_serial` bus is driven by the station's own `feetech` adapter in `sx`.

```bash
cargo test && cargo clippy --all-targets -- -D warnings && cargo fmt --check
```

Nothing in this repository runs those checks: it has no workflow. The `sx` superproject's
`just check-rust` runs them on the pinned commit, and that is the only enforcement.

## Assets

Canonical robot and capture-hardware descriptions live under `assets/`; provenance and licensing
are recorded in `THIRD_PARTY_NOTICES.md`. Standard wheels and sdists include the tree under
`sx_embodiments/_assets`; an editable install reads it from `assets/` in the checkout
(`hatch_build.py`). `sx_embodiments.assets.asset_root()` resolves the environment override,
installed tree, or editable-checkout tree and otherwise raises `AssetsUnavailableError`.

`sx_embodiments/asset-manifest.json` names every file of the tree with its sha256, size and
`license_id`. It is generated from the tree and the declarations (`python
tools/render_asset_manifest.py`), and `tests/test_asset_manifest.py` fails when it disagrees
with either. A declared asset keeps its own licence; an undeclared mesh takes the licences
declared in its nearest ancestor directory, so its audience is its directory's.

`materialize(embodiment)` returns a directory holding that body's verified closure (its
declared assets and every file its descriptions name) under the tree's own relpaths, so
`package://sx-embodiments/<relpath>` names and relative mesh references read unchanged:

```python
from sx_embodiments import embodiments, materialize, materialized

root = materialize(embodiments["so101"])
urdf = root / "so101/so101.urdf"
urdf = materialized(embodiments["so101"])   # the same file, named by the body's description
```

Every consumer that follows a description's references (MuJoCo, a URDF loader, a mesh
preview) reads it from the closure, never from `asset_root()`, whose tree a host may not
carry. A description that names meshes through its upstream ROS package
(`package://DAS_Gripper_urdf/meshes/...`) resolves them in the one ancestor directory of the
description that holds them, and every registered body's closure is checked to consist of
manifest rows (`tests/test_asset_integrity.py`).

On a host that carries the tree and whose tree holds every closure file with its declared
bytes, `materialize` returns the tree itself and writes nothing, so a container with a
read-only root reads its image's tree as before. Otherwise each file is hardlinked from a
digest-keyed cache (`~/.cache/sx-embodiments/sha256/`, or
`SX_EMBODIMENTS_ASSET_CACHE`), filled once from the local tree or from the asset store and
verified against the declared sha256 and size before it is cached. The store is
`SX_EMBODIMENTS_ASSET_STORE` (`https://` or `file://`, objects at `sha256/<xx>/<digest>`),
read with the bearer `SX_EMBODIMENTS_ASSET_STORE_TOKEN` in an unredirected header; the token
never appears in a URL, a message or a traceback, and 401, 403 and 404 all raise
`AssetsUnavailableError`. Tampered bytes raise `AssetDigestMismatchError` and never enter the
cache.

The hosted store is the SentientX catalog door:

```bash
export SX_EMBODIMENTS_ASSET_STORE=https://catalog.sentientx.io/api/embodiment-assets
export SX_EMBODIMENTS_ASSET_STORE_TOKEN=sxk_...   # a SentientX API key
```

It redirects each digest to a short-lived signed read of the object, after checking the key
holds a `(catalog, asset_reader)` grant whose scope reaches the file's audience: `public`
reaches files under public licences, `entitled` also reaches `LicenseRef-*` files. A digest
the manifest does not name is never served. `python tools/check_asset_store.py` reads one
byte of every manifest digest through the configured store and names each one it refuses
or lacks.

Recordings are immutable and name their embodiment by content, so a document this registry
published under an earlier schema (`sx_embodiments/historical/`, read by `read_recorded`) must
keep resolving. When a canonical file changes, keep its published revision at
`<package>/_by_digest/<sha256>/<filename>` (`assets.superseded_relpath`); `resolve_asset` serves
it locally, or fetches the revision from the store by its digest, when the file at the
declared path no longer matches. `tests/test_recorded.py` fails when a historical document's
asset stops resolving.

The registry covers Piper, ALOHA, RBY1, Unitree G1, UR10e, UR5e, YOR, Sentient Humanoid,
Franka/Panda variants, SO-101 variants, DAS/YUBI capture rigs, and supported teleop stations.
Declaration order is the native physical coordinate order and is pinned against each URDF.

## Development and validation

This repository is mounted at `packages/sx-embodiments` in the private `sx` superproject.
`sx-contracts` is an internal workspace package: it is deliberately declared with
`workspace = true`, is not published as a separate distribution, and must not be replaced by a
Git, path, or package-index fallback here.

Consequently, an embodiment commit becomes admissible only through an `sx` submodule pin
advance. The trusted `sx` workflow checks that the pinned commit is on this repository's `main`
branch, resolves the one workspace lock, runs strict typing, and executes this package's full
test suite against the exact pinned bytes. The useful local equivalent is run from the `sx`
checkout:

```bash
uv sync --locked
uv run pyright -p packages/sx-embodiments/pyproject.toml
uv run --package sx-embodiments pytest packages/sx-embodiments/tests -q
```

This repository intentionally has no standalone Python behavior workflow. Such a workflow cannot
resolve the private workspace dependency from this public repository, and a substitute contract
copy would create a second source of truth. Repository-local review protects this source history;
the exact `sx` pin-advance workflow is the executable integration gate.

Display previews are separate from hardware identity. `preview_asset(body)` returns a
content-pinned, transparent WebP for that exact body, or `None` if no preview was published.
Consumers serve its verified bytes through their own asset endpoint and use the digest for
caching; they do not infer source-tree paths. Worlds generates these small publications with
`worlds/sim-envs/scripts/render_embodiment_previews.py` using the arm-practice initial pose.
The image retains the source assets' publication audience.
