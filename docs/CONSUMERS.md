# Consumer boundaries

This is the standing map of how the stack uses the object-first registry. A new shared surface
lands only with a production consumer.

| Consumer | What crosses the boundary | Enforcement |
|---|---|---|
| Train | full `Embodiment` in artifacts and simulator bindings; `EmbodimentId` in run/checkpoint metadata | exact action interface and embodiment at ingest, export, publication, and load |
| Fleet | `EmbodimentId` on releases, targets, and deployments | release, target, controller, and Worlds evidence agree before motion |
| Catalog | full schema-v8 object as the governed resource; `EmbodimentId` in segment/checkpoint rows | fail-closed parsing, ID resolution, asset promotion, and lineage |
| Experience | full object resolved from Catalog; `EmbodimentId` on capture requests and orders | task roles resolve against component capabilities and the exact revision |
| Worlds | full object in executable bindings and RRD evidence; `EmbodimentId` in scenes and run records | task, scene, checkpoint, served controller, and evidence agree |
| real2sim | registered object rebound by `robot.with_assets(...)` | the bundle's own closure check fails closed before rebinding |
| sx-episodes | full object in each episode; `EmbodimentId` in summaries | object/URDF/calibration/action agreement and tensor coordinate order |
| auto-perfect | full object for drivers, safety, telemetry, and simulation | runtime limits and constraints derive directly from the object |
| SXD | generated full-object projection because standalone workers cannot import workspace packages | byte-for-byte projection parity check |

Rerun episode entity paths belong to `sx-episodes`; nominal camera names/mounts/rates belong to
the embodiment. Per-unit measured calibration belongs to the episode.

Deliberate boundaries:

- Every consumer above *obtains* an embodiment; none assembles one. The public surface
  (`sx_embodiments.__all__`, pinned by `tests/test_structure.py`) exports no component,
  attachment, part, joint layout, lineage, or kind, so `Embodiment(...)` has no reachable
  spelling outside this package. A new revision is a change to `sx_embodiments.known`.
- Per-unit intrinsics/extrinsics and encoder zeros are recording/session calibration.
- Simulation scene cameras are scene facts, not body facts.
- Controller semantics and command ordering belong to `sx-actions`.
- Episode quality, speed bins, and measured control timing are episode facts.
- Service APIs carry one content ID, not local `{name, digest}` wrapper models.
