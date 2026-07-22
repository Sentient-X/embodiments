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

`Embodiment` is one complete immutable hardware revision. There is no public manifest,
reference, digest, structure, or kinematics wrapper to assemble. Code that needs hardware facts
receives the object. Storage and service boundaries that need only identity carry `robot.id`, a
64-character SHA-256 of the complete canonical object. `embodiments[...]` resolves either a
friendly registry name or that content ID.

Schema v7 stores one topologically ordered component graph plus nominal rates and
content-addressed assets. State order, camera bindings, capabilities, arm/gripper convenience
views, and the ID are derived from those facts. This keeps one source for morphology while
remaining ergonomic for drivers, simulation, training, and task admission.

Controller semantics stay in `sx-actions`; one embodiment can expose several action interfaces
without changing hardware identity. Per-unit calibration and runtime status belong to episodes
or sessions, not the nominal embodiment.

External-corpus ingest starts from a registered object and replaces only its verified asset
bundle:

```python
robot = embodiments["franka"].with_assets(assets, urdf=urdf_bytes)
```

The caller supplies content-addressed assets and exact URDF bytes; the registered component
semantics remain authoritative. A mutable URL is a location, not asset identity, so ingest
boundaries hash bytes before producing an `AssetRef`.

## Assets

Canonical robot and capture-hardware descriptions live under `assets/`; provenance and licensing
are recorded in `THIRD_PARTY_NOTICES.md`. Wheels and sdists include the tree under
`sx_embodiments/_assets`. `sx_embodiments.assets.asset_root()` resolves the environment override,
installed tree, or editable-checkout tree and otherwise raises `AssetsUnavailableError`.

The registry covers Piper, NERO, ALOHA, RBY1, Unitree G1, UR10e, UR5e, YOR, Sentient Humanoid,
Franka/Panda variants, SO-101 variants, DAS/YUBI capture rigs, and supported teleop stations.
Declaration order is the native physical coordinate order and is pinned against each URDF.
