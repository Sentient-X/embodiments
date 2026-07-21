# How the stack consumes this registry — and what it absorbs next

The registry learns from its consumers: this document is the standing map of who
reads what, which embodiment facts still live as local copies in consumer repos,
and the deliberate boundaries that keep some facts out. Re-audit and update it
when a consumer's usage changes shape; every absorption lands with its consumer
in the same change (the glued surface-with-consumer rule). Last full audit:
2026-07-21, component-capability migration.

## The consumer contracts (who reads what, today)

| Consumer | Registry surface it binds | Enforcement |
|---|---|---|
| **train** | `embodiment_spec`/`manifest_for` for physical identity and `sx-actions` for dataset, simulator, artifact, and service controller semantics | exact interface/manifest refs at ingest, export, catalog publication, and load; no width-derived controller inference |
| **supervisors** | physical layout for deploy-record joint splits; exact `ActionInterfaceRef` on releases, targets, and deployments | release/target body, manifest, controller, and Worlds evidence must all agree before motion |
| **data-catalog** | schema-v5 component-graph manifest wire plus immutable `sx-actions` documents and refs | fail-closed parsing, tenant-local digest resolution, segment/checkpoint foreign keys and lineage |
| **data-factory** | `camera_bindings`/`embodiment_spec` (built-in seeds), `EmbodimentRef` resolution against the catalog, `AssetRef` content-addressing of customer URDFs | wire-bridge correlation by asset sha256 |
| **sim-envs** | kinematic views and `AssetRef` for scene assets; exact action interface on `SceneSpec`, stores, and evaluation evidence | task, world, checkpoint, and controller revisions must agree |
| **real2sim** | `AssetRef`/`EmbodimentManifest`/`validate_logical_path` — the bundle ingest producer | fail-closed closure validator |
| **sx-telemetry** | manifest/ref parsing, physical layout for URDF animation, and digest-verified scene geometry; `sx-actions` owns the episode command contract | manifest and interface refs must agree; joint episode tensors are checked against exact interface channel kinds/order |
| **enpire** | `PIPER`/`PANDA_OMRON`/`Embodiment` for drivers, safety derivation (joint limits → constraints), grasp/station typing | safety constraints derive from registry limits |
| **sxd** (contracts-by-format) | none at runtime (accepted asymmetry) — a **registry parity test** pins the insta360 converter's fisheye model/fps/camera names byte-equal to `insta360-umi` | the parity-test pattern IS the enforcement for standalone consumers |

Vocabulary owned elsewhere on purpose: rerun entity-path grammar
(`obs/image/<camera>` etc.) belongs to `sx-telemetry` — the camera **names**
inside those paths are this registry's fact; the path grammar is not.

## The absorb queue (ranked, from the 2026-07-19 audit)

Each item lands only with its consumer, one change per row:

1. **supervisors' half-adopted so101 body** — the physical layout already derives
   from the registry, but home joints, joint limits, and control rate are
   hand-copied in `sim/env.py`, and the UI's joint labels re-spell the channel
   vocabulary (`ui/src/lib/teleop.ts`). Absorb: read `home_joints`/limits/
   `ControlRates` from the spec; derive UI labels from `flat_layout.channel_names()`.
2. ~~**data-factory's parallel capability columns**~~ — resolved 2026-07-21:
   task admission matches schema-v5 manifest component profiles directly; the
   hand-count/mobile-base columns and response fields are no longer read.
3. **enpire's camera instance set** — `CameraMount` name/role/serial sets could
   bind to `camera_bindings`; action semantics now belong to `sx-actions`.
4. **sim-envs per-robot scene metadata** — `_PRIMITIVE_META`/`_PANDA_META`
   re-state `ArmSpec.joint_names` and gripper travel for bodies the registry
   declares.
5. ~~**the three-way capability vocabulary**~~ — resolved 2026-07-21 by
   `sx-capabilities`: Taskpedia authors typed role requirements, schema-v5 manifests
   attach capabilities to exact component nodes, and the factory returns the resulting
   `TaskEmbodimentBinding`.
6. **the flat-vector split/join transform** — the wire↔(joints, grippers)
   isomorphism is re-expressed in three consumers (train's
   `data/embodiment_actions.py` split/join, supervisors' `ChannelLayout`
   uniform-block re-encoding, sx-telemetry's scene column walk), each correct,
   each its own loop over `FlatLayout`. The shared owner is **sx-episodes**
   (it owns `JointAction` and carries numpy); it lands there when a second
   consumer migrates off its local copy. Controller vectors are deliberately
   excluded: they remain flat and are governed by `sx-actions`.
7. **the distortion-model vocabulary** — `sx_telemetry.DistortionModel` and
   data-pipeline's hardcoded allow-set in `embodiment_projection.py` spell the
   same string set twice (the sxd copy is forced by the contracts-by-format
   asymmetry — standalone workers cannot import workspace packages). Converges
   when sx-embodiments/sx-telemetry publish wheels; until then the sxd parity
   tests are the guard.
8. **`EmbodimentRef` pydantic wrappers** — catalog/factory/sim-envs each keep
    a 2-field OpenAPI model (mandated by FastAPI codegen; each converts through
    the registry's validating types). The registry's `ref_to_dict`/
    `ref_from_dict` (first consumer: sx-telemetry's envelope wire) is the
    canonical wire form those wrappers must stay byte-aligned with; sim-envs'
    scene-store flattening (`embodiment_manifest` key) is versioned store
    metadata — realigning it is a `_METADATA_VERSION` bump, not a drive-by.

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
- **Controller-owned action interfaces**: the registry declares physical body channels,
  never command semantics. `sx-actions` is the shared owner of controller space, mode,
  normalization, bounds, frame, rate, and wire order.
- **Episode/recording facts** (speed bins, quality ratings, per-episode
  control_hz): recording metadata, not hardware.

One migration norm rides on the registry: pre-convention episodes are never
reinterpreted or edited — if genuinely-unreproducible data ever needs a
convention migration, a derived successor is minted with `layout_for` as the
sole split authority and `derived_from` lineage (train DECISIONS 2026-07-19;
sim evals regenerate from checkpoints instead).
