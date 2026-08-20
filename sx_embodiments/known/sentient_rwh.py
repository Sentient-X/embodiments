"""Sentient RWH — the rugged wheeled humanoid, from the ``v3`` SolidWorks export.

Development-only, and the reason is structural rather than cosmetic: **the export declares
no joint limits at all**. All 56 movable joints are ``continuous`` with no ``<limit>``
element, so no torso, arm, or hand axis has a stop the stack can enforce. Three
consequences are visible in this module, and each disappears the moment the CAD/hardware
team publishes a limit table:

* Torso and arm axes use :func:`unbounded_layout`. Their limits are *missing*, not absent —
  unlike the wheels below, whose ``continuous`` type genuinely means "no position stop".
* The two 16-DOF hands are **not** body components here. :class:`GripperSpec` requires
  finite bounds on every actuated axis, so a hand cannot be expressed at all against this
  description. Both hands stay complete in the description asset, in the shape of the
  humanoid's neck/head: present in the URDF, never an action row.
* No :class:`PhysicalSpec`. The description sums to 69.5 kg, but the four tyre inertials
  are 14.6 g apiece under that mass, so the CAD mass properties are not a datasheet fact.

Two further export defects are recorded here because they change the layout below, and
should be confirmed against the CAD before this is promoted:

* **The arms are asymmetric.** ``RA_7`` is ``fixed`` while its mirror ``LA_7`` is
  ``continuous``, so the right arm contributes seven channels and the left eight.
  ``config/joint_names_v3.yaml`` omits ``RA_7`` as well, so the export is self-consistent —
  which is what makes a CAD mate error more likely than a transcription slip.
* **The left shoulder is tilted.** ``LA_1``'s axis is ``(0.966, 0, -0.259)`` — 15° — against
  ``RA_1``'s ``(-1, 0, 0)``, while shoulder-to-fingertip reach mirrors to 2 mm (0.933 m
  right, 0.931 m left). Either a deliberate angled shoulder mount or a mate artifact.

Joint names are the exported CAD part names, deliberately unrenamed. ``Arm_link1`` through
``Arm_link5_torso`` are the torso column, not an arm — the semantics live in the component
instances (``torso``, ``right_arm``) rather than in a rewritten description, so the next
export diffs cleanly against this one and nothing is renamed twice when the limits land.

The vendored description differs from ``v3/urdf/v3.urdf`` by a package rename only: the ROS
package and robot name ``v3`` become ``sentient_rwh``, so mesh references read
``package://sentient_rwh/meshes/`` (the ``humanoid_pkg`` vendoring precedent — a vendored ROS
export keeps ROS-native addressing). No kinematic value is patched.
"""

from typing import Final

from ..assets import AssetFormat, AssetProvenance, AssetRole, packaged_asset
from ..compose import BaseMount, EmbodimentDefinition, MountedOn, MountKind, body_component
from ..identity import EmbodimentKind, EmbodimentName, Lineage, PartId
from ..layout import CoordinateUnit
from ..parts import ArmSpec, JointGroupSpec, MobileBaseSpec
from ._authoring import unbounded_layout

SENTIENT_RWH_URDF: Final = packaged_asset(
    relpath="sentient_rwh/urdf/sentient_rwh.urdf",
    sha256="b960e38c1034537f93c4364548272a6f9bdeb0fdfddd7cf735a926c2244324ad",
    size_bytes=77370,
    format=AssetFormat.URDF,
    role=AssetRole.DESCRIPTION,
    provenance=AssetProvenance(
        repository="https://github.com/Sentient-X/sentient-rwh",
        revision="b846911b0c877d2b8e80aebc030280852ccfcfc4",
        path="urdf/v3.urdf",
        license_id="LicenseRef-Sentient-Proprietary",
    ),
    media_type="application/xml",
)

SENTIENT_RWH_BASE: Final = MobileBaseSpec(
    part_id=PartId("sentient-rwh-base"),
    # Four independently driven wheels, all spinning about the chassis x axis, so the body
    # travels along y. No steering or caster joint exists: the base is skid-steer, and the
    # body-velocity command it actually takes is `sx-actions`' vocabulary, not a layout fact.
    # These four axes are unbounded because a drive wheel HAS no position stop — the one
    # place in this body where `continuous` is a complete fact rather than a missing one.
    layout=unbounded_layout(
        names=("RL_tyre", "FL_tyre", "FR_tyre", "RR_tyre"),
        units=(CoordinateUnit.RADIAN,) * 4,
    ),
)

SENTIENT_RWH_TORSO: Final = JointGroupSpec(
    part_id=PartId("sentient-rwh-torso"),
    # The lift/spine column carrying the shoulders from z=0.29 m to z=1.29 m in `base_link`.
    layout=unbounded_layout(
        names=("Arm_link1", "Arm_link2", "Arm_link3", "Arm_link4", "Arm_link5_torso"),
        units=(CoordinateUnit.RADIAN,) * 5,
    ),
    home=(0.0,) * 5,
)

SENTIENT_RWH_RIGHT_ARM: Final = ArmSpec(
    part_id=PartId("sentient-rwh-right-arm"),
    # Seven channels, not eight: `RA_7` is `fixed` in the export (see the module docstring).
    layout=unbounded_layout(
        names=("RA_1", "RA_2", "RA_3", "RA_4", "RA_5", "RA_6", "RA_8"),
        units=(CoordinateUnit.RADIAN,) * 7,
    ),
    home=(0.0,) * 7,
)

SENTIENT_RWH_LEFT_ARM: Final = ArmSpec(
    part_id=PartId("sentient-rwh-left-arm"),
    layout=unbounded_layout(
        names=("LA_1", "LA_2", "LA_3", "LA_4", "LA_5", "LA_6", "LA_7", "LA_8_wrist"),
        units=(CoordinateUnit.RADIAN,) * 8,
    ),
    home=(0.0,) * 8,
)

SENTIENT_RWH_SPEC: Final = EmbodimentDefinition(
    name=EmbodimentName("sentient-rwh"),
    label="Sentient RWH (rugged wheeled humanoid)",
    kind=EmbodimentKind.ROBOT,
    lineage=Lineage(family="sentient-rwh"),
    attachments=(
        body_component("base", SENTIENT_RWH_BASE),
        body_component("torso", SENTIENT_RWH_TORSO, MountedOn("base", "base_link")),
        body_component("right_arm", SENTIENT_RWH_RIGHT_ARM, MountedOn("torso", "Arm_link5_torso")),
        body_component("left_arm", SENTIENT_RWH_LEFT_ARM, MountedOn("torso", "Arm_link5_torso")),
    ),
    # Footprint measured as the xy AABB of the chassis AND the four wheels' collision
    # geometry, posed into `base_link`. The wheels are the part that protrudes: the chassis
    # alone spans x +/-0.215, but the tyres reach x +/-0.312, so a chassis-only footprint
    # would understate each side by ~0.1 m and let placement drive them into an obstacle.
    # Their 0.16 m radius puts the floor at z = -0.108 in this frame.
    base_mount=BaseMount(
        kind=MountKind.MOBILE,
        frame="base_link",
        half_extents=(0.312, 0.340),
        centre=(0.001, 0.011),
    ),
    extra_assets=(SENTIENT_RWH_URDF,),
)
