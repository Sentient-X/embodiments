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

A member may mount on any link of a member placed before it, not only on the root:
`PlacedEmbodiment(embodiments["so101"], parent="body/head_link")` carries an arm on another
body's head. Composition and the registry share one renderer, `sx_embodiments.assembly.Assembly`:
members mounted on links at nominal poses, plus named frames for cameras and leader sites.
Assembly poses are nominal; a measured mount is calibration and never edits the assembly.

Every registered body is a `Build` of **modules** (`sx_embodiments.mechanical`). A module is
one mechanical unit (an arm, a gripper, a camera, a pan-tilt neck, an adapter plate, a fingertip
pad): its description, the **mechanical interface** its root link joins by (its plug; an ISO
9409-1 flange, a vendor's wrist flange, the 1/4-20 camera thread), the **sockets** it offers
(an interface at a pose on one of its links, and the component that moves that link), and the
components it carries in its own names. A build places modules at installation poses and mates
modules plug to socket; two sites join only when their interface ids are equal, and a wrong
pairing, a taken socket or an unknown one is a typed refusal (`InterfaceMismatchError`,
`SocketError`). Interfaces are an open registry, `MECHANICAL_INTERFACES`, like the camera models.
A passive element is a module with no components (`rigid_module`, a solid with its mass and
inertia), and `known/realsense.py` makes Intel's D405 and D435 descriptions camera modules.

A vendor that ships an arm with its gripper in one file has its flange **fitted**: the socket
carries the gripper module, cut from that file by `Subtree` below the joint it hangs from. A
build that keeps every fitted module renders the vendor's bytes, so no identity moves; mating
something else on a fitted socket cuts the fitted module out, which is a gripper swap.
`tests/test_mechanical.py` holds that every fitted module is detachable (cut out and mated back,
every link sits where the vendor put it and the components are the same), that every socket
names the component that moves it, and that swaps compose: a Franka Hand on a UR5e's ISO flange,
a B601 gripper on a Piper through an adapter, a D435 on a wrist through a bracket, an SO-101 in
the humanoid's head's place. `composition.compose_build` makes any build a normal `Embodiment`
at runtime. A registered rooted build's committed description is its rendering by
`tools/compose_registered_urdfs.py`; `tests/test_assembly.py` fails when the two differ.

`Subtree` keeps the cut joint under a `<joint>_mount` root, so a revolute hip still moves, and a
part mated with `namespaced=False` keeps its source's names. The Sentient humanoid is built this
way from its one vendor URDF: the upper body (the robot in service) offers a neck seat, two hip
seats and two wrists; the full humanoid mates the legs back on, and `tests/test_humanoid.py`
holds the rejoined parts to the vendor's robot by forward kinematics. The head's camera holder is
a socket fitted with the CAD's D435, a camera module that adds its own optical frame (a module's
`frames`) and carries the colour stream, riding on the torso that turns the chest. Its parallel
grippers are modules whose servo and jaw no source describes, so the body with them,
`sentient-humanoid-upper-body-grippers`, is a development body.

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

Schema v15 stores one topologically ordered component graph plus nominal rates,
content-addressed assets, and independent per-axis observation and actuation facts. Every
articulated chain (arm, torso, head, leg, lift) is one `ChainSpec`; the `Limb` it serves is
a fact of its mount, from which its channel kind and capabilities derive, so one design can be
an arm on one body and a head on another. Every component names its mount frame, and that
frame is a link of the description with the chain's joints below it: `tests/test_model_laws.py`
composes every registered body to prove it. A joint axis may be observed through actuator
feedback, a vendor interface, or an encoder and may be driven directly, through an integrated
controller, passively, or remain explicitly undocumented. A qualified direct drive binds an
`ActuatorModel` on an `ActuatorBus` with its chain address and its design drive map (`sign`,
`reduction`); a motor's zero is measured per unit and is joint calibration, never a design
fact. Cameras name their product by id in the open registry `sx_embodiments.cameras`, so a new
camera is a record, not a schema change. State order, camera bindings, capabilities, arm and
gripper convenience views, and the ID are derived from those facts.

Controller semantics stay in `sx-actions`; one embodiment can expose several action interfaces
without changing hardware identity. Runtime status belongs to episodes or sessions, not the
nominal embodiment.

## Calibration and robot instances

What was measured about one physical unit is kept apart from the design, in three kinds
(`sx_embodiments.calibration`), each identified by the digest of its `sx.calibration/v1`
document and carrying its method, time, residual and evidence:

- `CameraUnitCalibration`: one camera unit's intrinsics at one raster.
- `RigidCalibration`: one transform between two links of a body, absolute or a correction of
  one exact description's own transform (mount corrections, hand-eye, stereo pairs, depth to
  colour).
- `JointUnitCalibration`: one actuator or encoder unit's zero, direction and measured range.
  Every driver applies `actuator = sign * direction * (joint - zero_offset) * reduction`, the
  unit's zero and direction entering at the one Rust seam `BoundAxis::calibrated`; unmeasured
  they are `UNCALIBRATED_ZERO` and 1.

A driver opens a built robot from its instance: `drive_document(body, resolve(...))` renders
`sx.instance-drive/v1`, each driven joint's zero and direction in state order, and
`sx_embodiment_drivers::InstanceDrive::parse(document, instance)` reads it only for the robot
instance the caller is configured as, so another unit's zeros are refused;
`BodyBindings::calibrated` then refuses a drive of another body. Every Damiao opener takes one.

What a built robot streams comes from the same resolution: `sx_embodiments.media.media_document`
renders `sx.media-plan/v2`, one track per declared camera with its resolved optics and mount
(the description's joints from root to mount link, and its pose in the root when they are all
fixed, or the independent joints it rides on), a `MediaRig`'s declared
stereo pair as one track left then right, and its fitted microphones.
`sx_embodiment_drivers::MediaPlan::parse(document, instance)` reads it for one robot instance,
and a camera or microphone adapter implements `FrameSource` or `AudioSource` for one planned
track. `sx_embodiment_drivers::kinematics::Chain::pose` evaluates a view's rendered chain at
joint positions exactly as `Kinematics.pose` does, so a station places a camera on moving joints
without reading the description. `tools/render_driver_bindings.py` renders the crate's
`media-plans.json` fixture and `media-poses.json`, seeded mount poses from the description that
the chain evaluation is held equal to.

A `RobotInstance` (`sx_embodiments.instance`, `sx.robot-instance/v1`) is one physical robot:
a content-addressed body, the serial unit in each of its slots (each non-camera component,
each directly driven joint, each encoder-read joint, each declared camera) and the
calibrations selected for those units. `resolve(body, instance, calibrations)` binds them,
refusing a unit in a slot the body lacks, a calibration of a unit the instance does not place,
two calibrations of one slot, a frame that is not a link of the description, or a correction
of another description; every slot left unmeasured resolves to the design, labelled nominal.
Neither object moves the embodiment's id. Calibrations recorded before these kinds keep their
sealed bytes, and their readers parse them into the kinds with the sealed document as
evidence; `tests/test_calibration.py` resolves every registered body through an instance.

The solvers that measure those kinds are `sx_embodiments.calibrate`, behind the `calibrate`
extra (NumPy, SciPy, OpenCV); nothing else in the package imports them. `solve_intrinsics`
fits a camera unit's lens from checkerboard or ChArUco views, dropping a view only when the
fit names it ill-conditioned or it is the worst and over the limit. `solve_mounts` places
cameras on the links that carry them from views of a target another link holds, with every
view's carrier-from-holder transform known: one model covers eye-in-hand, eye-to-base, a head
camera watching a target in the hand and a tracked rig watching a static board, cameras that
saw the same target are solved together, and a checkerboard's half-turn ambiguity is resolved
per view. `measure_intrinsics` and `measure_mount` bind both to a robot instance: the unit is
the one the instance places in the camera's slot, the raster is the body's, and each view's
transform comes from forward kinematics over the body's own description at the joint
positions recorded with it (`sx_embodiments.kinematics`, the one URDF chain reader). A
measured mount is a correction of the description's transform from the camera's carrier link,
which `resolve` then accepts; `tests/test_calibrate.py` recovers known lenses and mounts,
including the humanoid's head camera through its instance, and holds every declared camera
calibratable.

A stored document of any earlier schema is read with `migrate(document)`, which runs the chain
of pure converters (13 → 14 → 15) and returns one verified identity edge: the id the bytes
carry and the body they read as today. A converter changes only what its schema changed and
refuses what it cannot carry (a schema-14 design zero other than 0) rather than dropping it.
A schema bump snapshots every published body into `sx_embodiments/historical/` in the same
change, and `tests/test_model_laws.py` reads every one of them.

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
actuators an axis's `ActuatorBinding` names, indexed in the body's native state order, each
failure classified as a `ChainFailure`, and a stop becomes a `ProvenStop` only when every
actuator answered stopped. A body spanning two devices or buses is one `Composite` chain, and
`can::SharedCanPort` lets chains take turns on one bridge. The station drives every body through
this trait and names no vendor.

A bus crate is a translation of its vendor's own driver — the same structure, names, frames,
clamps and command order — pinned to transcripts recorded by running that driver, with each
departure listed in the ported module's header. A crate that composes a known body embeds the
bindings `tools/render_driver_bindings.py` renders from the registry (`drivers/<crate>/bindings/`)
and parses them with `BodyBindings`, so no joint bound, address, model, sign or reduction is
spelled in Rust; it adds only its vendor's controller configuration, keyed by joint name. A
joint's zero is the unit's calibration, applied at one seam (`BoundAxis::calibrated`) from the
robot instance's `InstanceDrive`; a nominal instance carries `UNCALIBRATED_ZERO`. Only directly
driven joints are rendered: a joint driven any other way (the humanoid's waist linkage) is in no
driver's chain.

| Crate | Bus | Vendor driver it translates | Consumer |
|---|---|---|---|
| `sx-damiao-can` | `damiao_can` | Damiao's `DM_CAN.py` ([kit-miao/motor-sdk](https://gitee.com/kit-miao/motor-sdk) `fb0e9fc5`; by cmjang, MIT) | the `sx` station's `ChainActuator`, over `b601::open_b601` and `humanoid::open_humanoid_upper_body` |
| `sx-embodiment-bodies` | any of the above | none: it opens a registered body from its `InstanceDrive` and the station's `sx.station-commissioning/v1` (port name to USB bridge serial), dispatching to the body's opener and sharing one bridge two ports name; feature `usb-serial` finds Damiao USB2CAN bridges by serial in sysfs (through [`serialport`](https://github.com/serialport/serialport-rs), MPL-2.0) | the `sx` station's `StationRobot::open_actuator` |
| `sx-feetech-serial` | `feetech_serial` | Feetech's `scservo_sdk` ([ftservo/FTServo_Python](https://github.com/ftservo/FTServo_Python) `cbcfa646`; by ftservo, MIT) | the `sx` station's `ChainActuator`, over `so101::open_so101` |

Cameras follow the same rule through `sx_embodiment_drivers::FrameSource`: a robot's
`MediaPlan` names each track's views and the camera unit placed in each, and a camera crate
captures exactly that.

| Crate | Devices | Vendor SDK it drives | Consumer |
|---|---|---|---|
| `sx-realsense` | Intel RealSense D405, D435, D435i (colour, and the infrared pair as one stereo track) | librealsense2 2.x through [`realsense-rust`](https://gitlab.com/tangram-vision/oss/realsense-rust) 1.3.0 (Apache-2.0), behind the `sdk` feature | the `sx` station's media publisher, over `sdk::open`, once the station opens sources from its media plan |

```bash
cargo test && cargo clippy --all-targets -- -D warnings && cargo fmt --check
# with librealsense2 installed (Homebrew's pkg-config file names a Linux libdir: add
# RUSTFLAGS="-L /opt/homebrew/lib" on a Mac)
cargo test -p sx-realsense --features sdk
```

Nothing in this repository runs those checks: it has no workflow. The `sx` superproject's
`just check-rust` runs them on the pinned commit, and that is the only enforcement.

## Quest capture device I/O

`sx_embodiments.quest_capture.QuestLink` owns the existing OpenQuestCapture USB/adb
and HTTP protocol: recovery, APK-byte identity, recording control, pulls and raw clock
replies. `sx_embodiments.quest` owns build-bound hardware/pose facts and historical APK
conventions. The sx Factory recorder consumes the driver; capture and conversion consume
these same facts. Factory owns clock-fit acceptance, episode orchestration and recording.
The driver uses only the standard library; adb remains a host prerequisite.

Loopback HTTP tests verify the request/response boundary and refuse malformed device
clock fields. They do not qualify a headset, a Pico runtime or remote robot control.
The superproject records these source bytes in its recorder build fingerprint, so moving
or changing the driver requires a new accepted capture build before production use.

## Standard YUBI encoder interpretation

`sx_embodiments.yubi.YubiEncoder` parses the existing standard YUBI AS5601 stream,
normalizes its signed zero-relative angle and refuses range or firmware-identity changes
within a connection. Capture preflight and recording use this same parser. The module also
owns the existing encoder envelope and the canonical four-bar jaw conversion used by the
recording converter. `encoder_serial.open_encoder_lines` owns UART access and host receipt timestamps;
install `sx-embodiments[capture]` for its pyserial dependency. Factory retains spooling
and freshness. The shared
`encoder_serial.EncoderLineDecoder` admits a frame only at its newline, retains timeout
fragments, and bounds a line to 4096 bytes. Capture preflight and recording use it for
both existing encoder profiles; invalid UTF-8 and oversized lines refuse capture.

These are the existing standard-YUBI facts. They neither define Parallel YUBI's geometry
nor replace measured per-unit calibration. The nominal CAD aperture curve retains its
existing clamping behavior; this migration adds no physical qualification.

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

The registry covers Piper, ALOHA, RBY1, Unitree G1, UR10e, UR5e, YOR, the Sentient humanoid
(upper body and full),
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
