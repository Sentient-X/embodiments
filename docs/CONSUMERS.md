# Consumer boundaries

This is the standing map of how the stack uses the object-first registry. A new shared surface
lands only with a production consumer.

| Consumer | What crosses the boundary | Enforcement |
|---|---|---|
| Train | full `Embodiment` in artifacts and simulator bindings; `EmbodimentId` in run/checkpoint metadata | exact action interface and embodiment at ingest, export, publication, and load |
| Fleet | `EmbodimentId` on releases, targets, and deployments | release, target, controller, and Worlds evidence agree before motion |
| Catalog | full schema-v7 object as the governed resource; `EmbodimentId` in segment/checkpoint rows | fail-closed parsing, ID resolution, asset promotion, and lineage |
| Experience | full object resolved from Catalog; `EmbodimentId` on capture requests and orders | task roles resolve against component capabilities and the exact revision |
| Worlds | full object in executable bindings and RRD evidence; `EmbodimentId` in scenes and run records | task, scene, checkpoint, served controller, and evidence agree |
| real2sim | `robot.with_assets(...)` and `validate_logical_path` | bundle closure and output bytes fail closed |
| sx-telemetry | full object in each envelope; `EmbodimentId` in summaries | object/URDF/calibration/action agreement and tensor coordinate order |
| enpire | full object for drivers, safety, telemetry, and simulation | runtime limits and constraints derive directly from the object |
| SXD | generated full-object projection because standalone workers cannot import workspace packages | byte-for-byte projection parity check |

Rerun entity paths belong to `sx-telemetry`; camera names placed in those paths belong to the
embodiment. Per-unit calibration belongs to the recording.

Deliberate boundaries:

- Per-unit intrinsics/extrinsics and encoder zeros are recording/session calibration.
- Simulation scene cameras are scene facts, not body facts.
- Controller semantics and command ordering belong to `sx-actions`.
- Episode quality, speed bins, and measured control timing are episode facts.
- Service APIs carry one content ID, not local `{name, digest}` wrapper models.
