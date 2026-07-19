# How the stack consumes this registry — and what it absorbs next

The registry learns from its consumers: this document is the standing map of who
reads what, which embodiment facts still live as local copies in consumer repos,
and the deliberate boundaries that keep some facts out. Re-audit and update it
when a consumer's usage changes shape; every absorption lands with its consumer
in the same change (the glued surface-with-consumer rule). Last full audit:
2026-07-19, all nine repos + three frontends.

## The consumer contracts (who reads what, today)

| Consumer | Registry surface it binds | Enforcement |
|---|---|---|
| **train** (serl loop) | `embodiment_spec` (identity, fail-closed), `flat_layout` (channel identity: env joint names matched per slot; records emit/seed split at the declared slots — the same scatter convention as the catalog), `camera_names` (declared-camera membership), `manifest_for` (full manifest into the export artifact, with the layout's channel names in metadata) | typed errors before training; round-trip law test over interleaved layouts; artifact dof+channels cross-checks at load |
| **train** (pi0 export, serve) | `manifest_for`/`manifest_from_dict`, `EmbodimentRef`/`EmbodimentManifestDigest` | round-trip validation at export |
| **supervisors** | `layout_for(id).uniform_arm_blocks()` → the bimanual channel split | registry/layout drift is a startup crash — the model consumer |
| **data-catalog** | `layout_for` + `ChannelKind.indices` (flat-vector reconstruction at ingest), manifest wire + digest store, `SensorModel`/`CameraModality` in API schemas | fail-closed manifest parsing; layout validation at joint-space wires |
| **data-factory** | `camera_bindings`/`embodiment_spec` (built-in seeds), `EmbodimentRef` resolution against the catalog, `AssetRef` content-addressing of customer URDFs | wire-bridge correlation by asset sha256 |
| **sim-envs** | `PIPER`/`PANDA_OMRON` kinematic views on `SceneSpec.embodiment_id` (the env↔robot binding), `AssetRef` for scene assets | env registry binds task → body |
| **real2sim** | `AssetRef`/`EmbodimentManifest`/`validate_logical_path` — the bundle ingest producer | fail-closed closure validator |
| **enpire** | `PIPER`/`PANDA_OMRON`/`Embodiment` for drivers, safety derivation (joint limits → constraints), grasp/station typing | safety constraints derive from registry limits |
| **sxd** (contracts-by-format) | none at runtime (accepted asymmetry) — a **registry parity test** pins the insta360 converter's fisheye model/fps/camera names byte-equal to `insta360-umi` | the parity-test pattern IS the enforcement for standalone consumers |

Vocabulary owned elsewhere on purpose: rerun entity-path grammar
(`obs/image/<camera>` etc.) belongs to `sx-telemetry` — the camera **names**
inside those paths are this registry's fact; the path grammar is not.

## The absorb queue (ranked, from the 2026-07-19 audit)

Each item lands only with its consumer, one change per row:

1. **supervisors' half-adopted so101 body** — the action layout already derives
   from the registry, but home joints, joint limits, and control rate are
   hand-copied in `sim/env.py`, and the UI's joint labels re-spell the channel
   vocabulary (`ui/src/lib/teleop.ts`). Absorb: read `home_joints`/limits/
   `ControlRates` from the spec; derive UI labels from `flat_layout.channel_names()`.
2. **data-factory's parallel `Embodiment` DB** — dof/link_count re-parsed from
   customer URDFs, `hands`/`mobile_base` capability re-declared. Absorb:
   customer intake mints a registry-shaped `EmbodimentManifest` (the catalog
   bundle path) instead of a bespoke row joined by URDF sha256.
3. **enpire's camera instance set + RoboCasa action slices** — `CameraMount`
   name/role/serial sets and the hand-declared robosuite action layout could
   bind to `camera_bindings`/`flat_layout` views.
4. **sim-envs per-robot scene metadata** — `_PRIMITIVE_META`/`_PANDA_META`
   re-state `ArmSpec.joint_names` and gripper travel for bodies the registry
   declares.
5. **the three-way capability vocabulary** — taskpedia's `arms=N mobile_base=…`
   task profiles, data-factory's `HandCount`, and the registry's part
   decomposition encode the same body-capability facts three ways; one derived
   view should serve all three.
6. **the BC-family suffix split** — `train/data/lerobot_ingest.py` splits
   actions by suffix and `loops/{bc,ki,ttt_bc,lap,a2a}.py` + `data/clap.py`
   re-concatenate; internally consistent for LeRobot suffix data, divergent
   from the layout scatter convention on interleaved bodies. Migrates to
   `FlatLayout.indices` when a BC loop first consumes an interleaved catalog
   dataset (the serl records boundary is the template).
7. **train action-encoding vocabulary** — `ActionSpace`/`ActionMode`/
   `ActionEncoding` live in `train.sim` today (env-declared, artifact-carried);
   they promote here when a second repo consumes encodings (the record
   vocabulary in sx-episodes is the likely trigger: emitted episodes currently
   ride env-encoded actions on `joint_targets` without saying so).

## Deliberate boundaries (what the registry will NOT absorb)

- **Per-unit calibration** (K/D matrices, hand-eye extrinsics, encoder zeros):
  capture data that travels with the recording (MCAP `camera_info`, calibration
  registries, plain-text hand-eye files). The registry states the nominal,
  per-product facts only: projection family, fps, resolution, mounts. Numbers
  not on a datasheet do not enter the spec-sheet-of-record.
- **Sim scene cameras** (`overhead`/`front`/`frontview` in MJCF scenes,
  ManiSkill sensor names): properties of a *scene*, not a *body*. A robot entry
  deliberately declares no cameras — eyes arrive by composing the body into a
  rig or station (the character laws in `tests/test_known.py`).
- **Controller-owned action encodings**: which encoding an env/controller uses
  is the env's declaration (train's door derives it per control mode); the
  registry declares the channels, never the command semantics over them.
- **Episode/recording facts** (speed bins, quality ratings, per-episode
  control_hz): recording metadata, not hardware.

One migration norm rides on the registry: pre-convention episodes are never
reinterpreted or edited — if genuinely-unreproducible data ever needs a
convention migration, a derived successor is minted with `layout_for` as the
sole split authority and `derived_from` lineage (train DECISIONS 2026-07-19;
sim evals regenerate from checkpoints instead).
