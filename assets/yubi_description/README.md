# Yubi controller-holder reference frames

The fixed `controller_holder` frame in each source URDF is a CAD reference for
inspecting the controller mount and comparing holder-relative controller poses.
The combined URDF prefixes it as `left_controller_holder` or
`right_controller_holder`, attached to the corresponding `*_base_link`.
These are CAD-derived frames, not measured mount calibrations. Existing meshes,
moving joints, camera frames and controller tracking frames are unchanged.

## Source and reproducibility

The authoring session used the `controller_holder` mate connectors in these
operator-supplied Onshape assemblies:

- [Left Yubi assembly](https://cad.onshape.com/documents/b807fc587efa728d9cf496a6/w/7176242b14d954b35e9a0f8d/e/c659d653cfd1965addfdf1a0)
- [Right Yubi assembly](https://cad.onshape.com/documents/a49048be0a418509ea642b3d/w/0a6170e4e5b19ae4d8d75a37/e/9cb1639fa99dd40bf32608f7)

The supplied links identify mutable workspaces. The immutable Onshape revision
and original extraction response were not retained; this change does not claim
that today's workspace reproduces the historical extraction. The exact imported
origins are preserved in `source/yubi_left_gripper.urdf` and
`source/yubi_right_gripper.urdf`, joint `controller_holder_frame`. They were first
committed in `80ea80a3fb32084fd9eea61546b71a8dc7041f6c` and are unchanged here.
Units are metres for `xyz` and radians for URDF fixed-axis `rpy`.

Each origin expresses the holder frame in its hand's `base_link`, not in the
Onshape assembly world. During import the conversion was:

```text
T_base_holder = inverse(T_assembly_base) * T_assembly_holder
```

The translations are approximately (-38, +7.5, +14) mm on the left and
(-38, -7.5, +14) mm on the right, with near-zero rotation. The source URDFs,
not these rounded values, define the imported transforms. From the repository
root, regenerate the combined description with `python tools/compose_yubi_urdf.py`.
Its SHA-256, byte size and lineage are pinned in `sx_embodiments/known/yubi.py`.

## Intended consumer and scope

The consumer is the development CAD-frame inspection and alignment workflow.
Piper controller poses were authored by transferring each Yubi controller's
holder-relative pose to the corresponding Piper holder:

```text
T_piper_root_controller = T_piper_root_holder
                        * inverse(T_yubi_root_holder)
                        * T_yubi_root_controller
```

Here `controller` is Yubi's `quest_<side>_controller` tracking frame. Piper stores
the resulting transform in its own `<side>_vrcontroller` chain; its composer,
recording path and viewer do not read these Yubi holder frames at runtime.
This independent change requires neither Piper assets nor Piper support.

Adding frames changes the Yubi URDF bytes and therefore its asset and embodiment
identities. Hash-bound mount calibration records must be reviewed against the
selected identity; a hash update alone does not establish physical calibration.
Existing recordings and calibration files are not rewritten by this change.
