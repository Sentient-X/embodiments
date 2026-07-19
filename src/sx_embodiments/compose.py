"""Composition: an embodiment is an ordered tuple of part attachments; views are derived.

Composites are built by helper functions returning attachment tuples (``so101_side("left")``),
never by nesting specs — Python composition of tuples IS the compositional model. The
declared attachment order is the wire order (see :mod:`sx_embodiments.layout`).
"""

from dataclasses import dataclass
from enum import StrEnum

from .assets import PackagedAsset
from .errors import CompositionError, LayoutError
from .identity import EmbodimentId, EmbodimentKind, Lineage
from .kinematic import Embodiment
from .layout import ChannelKind, ChannelSlot, FlatLayout
from .parts import (
    ArmSpec,
    CameraBinding,
    CameraSpec,
    ControlRates,
    DeviceSpec,
    ForceTorqueSpec,
    GripperSpec,
    JointGroupSpec,
    MobileBaseSpec,
    Part,
)


class AttachmentRole(StrEnum):
    BODY = "body"  # contributes channels to the flat action vector
    LEADER = "leader"  # teleop input device: identity + assets, zero channels
    SENSOR = "sensor"  # cameras/FT: zero channels; cameras define the camera-name set


@dataclass(frozen=True, slots=True)
class MountFrame:
    """Where a part attaches. Informational (FK/renderers); NEVER affects channel order."""

    parent_instance: str = ""  # "" = the world/root
    frame: str = ""  # frame/link name in the parent's description


@dataclass(frozen=True, slots=True)
class Attachment:
    instance: str  # unique within the spec: "left_arm", "wrist_left"
    part: Part
    role: AttachmentRole
    mount: MountFrame = MountFrame()


_SENSOR_PARTS = (CameraSpec, ForceTorqueSpec)
_BODY_PARTS = (ArmSpec, JointGroupSpec, GripperSpec, MobileBaseSpec, DeviceSpec)


@dataclass(frozen=True, slots=True)
class EmbodimentSpec:
    """THE canonical hardware record. Everything else is a derived view."""

    embodiment_id: EmbodimentId
    name: str
    kind: EmbodimentKind
    lineage: Lineage
    attachments: tuple[Attachment, ...]
    rates: ControlRates | None = None
    extra_assets: tuple[PackagedAsset, ...] = ()
    # THE EXPLICIT-LAYOUT LAW: ``action_layout`` overrides derivation when the wire order
    # is a controller's, not the description's. Its slots may mix declared attachment
    # instances with minted virtual controller channels (libero_panda: virtual
    # ``eef``/``libero-cartesian-controller`` deltas + the real ``gripper``) — so slots
    # are deliberately NOT cross-validated against attachments; only the embodiment_id
    # must match. Pinned by
    # ``tests/test_layout_laws.py::test_explicit_layout_may_mint_controller_channels``.
    action_layout: FlatLayout | None = None

    def __post_init__(self) -> None:
        eid = str(self.embodiment_id)
        if not eid.strip():
            raise CompositionError(eid, "embodiment_id must not be empty")
        if not self.name.strip():
            raise CompositionError(eid, "name must not be empty")
        if not self.attachments:
            raise CompositionError(eid, "an embodiment needs at least one attachment")
        if (
            self.action_layout is not None
            and self.action_layout.embodiment_id != self.embodiment_id
        ):
            raise CompositionError(eid, "action layout embodiment_id must match the spec")
        seen: set[str] = set()
        for attachment in self.attachments:
            if not attachment.instance.strip():
                raise CompositionError(eid, "attachment instance names must not be empty")
            if attachment.instance in seen:
                raise CompositionError(
                    eid, f"duplicate attachment instance {attachment.instance!r}"
                )
            parent = attachment.mount.parent_instance
            if parent and parent not in seen:
                raise CompositionError(
                    eid,
                    f"{attachment.instance!r} mounts on {parent!r}, which is not declared "
                    "before it (attachments are topologically ordered by declaration)",
                )
            seen.add(attachment.instance)
            if isinstance(attachment.part, _SENSOR_PARTS):
                if attachment.role is not AttachmentRole.SENSOR:
                    raise CompositionError(
                        eid, f"{attachment.instance!r}: sensor parts must use role SENSOR"
                    )
            elif attachment.role is AttachmentRole.SENSOR:
                raise CompositionError(
                    eid, f"{attachment.instance!r}: body parts cannot use role SENSOR"
                )
        leaders = [a for a in self.attachments if a.role is AttachmentRole.LEADER]
        if self.kind is not EmbodimentKind.TELEOP_STATION and leaders:
            raise CompositionError(eid, f"{self.kind.value} embodiments cannot have leaders")
        if self.kind is EmbodimentKind.TELEOP_STATION:
            if not leaders:
                raise CompositionError(eid, "a teleop station needs at least one leader")
            if not any(a.role is AttachmentRole.BODY for a in self.attachments):
                raise CompositionError(eid, "a teleop station needs a follower body")
        if self.kind is EmbodimentKind.ROBOT and not any(
            a.role is AttachmentRole.BODY for a in self.attachments
        ):
            raise CompositionError(eid, "a robot needs at least one body attachment")

    def body_attachments(self) -> tuple[Attachment, ...]:
        return tuple(a for a in self.attachments if a.role is AttachmentRole.BODY)

    def layout_declared(self) -> bool:
        """False when any body part's channel contribution is unknown (a DeviceSpec)."""
        return not any(isinstance(a.part, DeviceSpec) for a in self.body_attachments())


def flat_layout(spec: EmbodimentSpec) -> FlatLayout:
    """Derive the flat-vector layout per the ordering law, or fail closed."""
    if spec.action_layout is not None:
        return spec.action_layout
    if not spec.layout_declared():
        raise LayoutError(
            str(spec.embodiment_id),
            "channel layout is not declared (a body part has no captured description)",
        )
    slots: list[ChannelSlot] = []
    for attachment in spec.body_attachments():
        part = attachment.part
        if isinstance(part, ArmSpec):
            names, kind = part.joint_names, ChannelKind.ARM_JOINT
        elif isinstance(part, JointGroupSpec):
            names, kind = part.joint_names, ChannelKind.BODY_JOINT
        elif isinstance(part, GripperSpec):
            names, kind = part.joint_names, ChannelKind.GRIPPER
        elif isinstance(part, MobileBaseSpec):
            names, kind = part.channel_names, ChannelKind.BASE
        else:  # pragma: no cover - excluded by layout_declared()
            raise LayoutError(str(spec.embodiment_id), f"unlayoutable part {part!r}")
        base = len(slots)
        slots.extend(
            ChannelSlot(
                index=base + offset,
                instance=attachment.instance,
                part_id=part.part_id,
                joint_name=name,
                kind=kind,
            )
            for offset, name in enumerate(names)
        )
    return FlatLayout(embodiment_id=spec.embodiment_id, slots=tuple(slots))


def total_dof(spec: EmbodimentSpec) -> int:
    return flat_layout(spec).action_dim


def camera_bindings(spec: EmbodimentSpec) -> tuple[CameraBinding, ...]:
    """Camera instances in declared order: the embodiment's canonical stream-name set."""
    return tuple(
        CameraBinding(
            name=a.instance,
            camera=a.part,
            parent_instance=a.mount.parent_instance,
            frame=a.mount.frame,
        )
        for a in spec.attachments
        if isinstance(a.part, CameraSpec)
    )


def camera_names(spec: EmbodimentSpec) -> tuple[str, ...]:
    return tuple(binding.name for binding in camera_bindings(spec))


def kinematic_view(spec: EmbodimentSpec) -> Embodiment:
    """Project the flat single-arm :class:`Embodiment` runtimes bind, or fail closed.

    Defined exactly for bodies with one arm + one single-joint gripper with known travel
    (plus optional channel-less base); ``dof`` is the ARM dof, matching how enpire drivers
    and safety have always read the record.
    """
    eid = str(spec.embodiment_id)
    arms = [a.part for a in spec.body_attachments() if isinstance(a.part, ArmSpec)]
    grippers = [a.part for a in spec.body_attachments() if isinstance(a.part, GripperSpec)]
    bases = [a.part for a in spec.body_attachments() if isinstance(a.part, MobileBaseSpec)]
    if len(arms) != 1 or len(grippers) != 1:
        raise LayoutError(eid, "kinematic view requires exactly one arm and one gripper")
    arm, gripper = arms[0], grippers[0]
    if gripper.travel_m is None:
        raise LayoutError(eid, "kinematic view requires a known gripper travel")
    if any(base.channel_names for base in bases):
        raise LayoutError(eid, "kinematic view excludes bases with joint-space channels")
    if spec.rates is None:
        raise LayoutError(eid, "kinematic view requires declared control rates")
    return Embodiment(
        embodiment_id=spec.embodiment_id,
        dof=arm.dof,
        joint_lower=arm.joint_lower,
        joint_upper=arm.joint_upper,
        home_joints=arm.home_joints,
        gripper_travel_m=gripper.travel_m,
        policy_hz=spec.rates.policy_hz,
        mobile_base=bool(bases),
    )
