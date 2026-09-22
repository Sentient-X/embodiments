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

A declared space does not establish an installed driver or simulator. Runtime support is
checked by the execution owner against the exact body and requested interface.

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

## Assets

Canonical robot and capture-hardware descriptions live under `assets/`; provenance and licensing
are recorded in `THIRD_PARTY_NOTICES.md`. Wheels and sdists include the tree under
`sx_embodiments/_assets`. `sx_embodiments.assets.asset_root()` resolves the environment override,
installed tree, or editable-checkout tree and otherwise raises `AssetsUnavailableError`.

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
