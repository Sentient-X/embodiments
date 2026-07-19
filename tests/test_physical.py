"""PhysicalSpec and camera-optics pins: only datasheet-verified values, validated typed."""

import pytest

from sx_embodiments import (
    DEVELOPMENT_EMBODIMENTS,
    CameraModality,
    CameraSpec,
    EmbodimentId,
    LensProjection,
    PartId,
    PartValidationError,
    PhysicalSpec,
    SensorModel,
    camera_bindings,
)
from sx_embodiments.known.nero import NERO_ARM
from sx_embodiments.known.panda import PANDA_ARM
from sx_embodiments.known.piper import PIPER_ARM
from sx_embodiments.known.so101 import SO101_ARM
from sx_embodiments.known.yubi import D405_30


def test_piper_datasheet_facts() -> None:
    """AgileX PiPER datasheet: 1.5 kg payload, 626 mm reach, 4.2 kg mass."""
    assert PIPER_ARM.physical == PhysicalSpec(payload_kg=1.5, reach_m=0.626, mass_kg=4.2)


def test_nero_datasheet_facts() -> None:
    """AgileX NERO datasheet: 3 kg payload, 580 mm reach, 4.8 kg mass."""
    assert NERO_ARM.physical == PhysicalSpec(payload_kg=3.0, reach_m=0.58, mass_kg=4.8)


def test_panda_datasheet_facts() -> None:
    """Franka Emika Panda datasheet: 3 kg payload, 855 mm reach, 18 kg mass."""
    assert PANDA_ARM.physical == PhysicalSpec(payload_kg=3.0, reach_m=0.855, mass_kg=18.0)


def test_unverified_facts_stay_none() -> None:
    """No manufacturer figure exists for the SO-101 — the spec states nothing."""
    assert SO101_ARM.physical is None


def test_physical_spec_rejects_nonpositive_values() -> None:
    with pytest.raises(PartValidationError):
        PhysicalSpec(payload_kg=0.0)
    with pytest.raises(PartValidationError):
        PhysicalSpec(reach_m=-0.1)


def test_d405_optics() -> None:
    assert D405_30.resolution == (1280, 720)
    assert D405_30.projection is LensProjection.PINHOLE


def test_camera_resolution_rejects_nonpositive() -> None:
    with pytest.raises(PartValidationError):
        CameraSpec(
            part_id=PartId("bad-cam"),
            model=SensorModel.UVC_MONO,
            modality=CameraModality.RGB,
            fps=30.0,
            resolution=(0, 720),
        )


def test_insta360_umi_entry() -> None:
    """The X5 rig: equidistant fisheye wire fact, undeclared jaw layout, wrist camera pair."""
    spec = DEVELOPMENT_EMBODIMENTS[EmbodimentId("insta360-umi")].spec
    assert not spec.layout_declared()  # jaw kinematics uncaptured -> enforcement skips
    bindings = camera_bindings(spec)
    assert tuple(binding.name for binding in bindings) == ("left_wrist", "right_wrist")
    assert all(binding.camera.projection is LensProjection.EQUIDISTANT for binding in bindings)
    assert all(binding.camera.fps == 30.0 for binding in bindings)
