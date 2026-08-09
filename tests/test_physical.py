"""PhysicalSpec and camera-optics pins: only datasheet-verified values, validated typed."""

import pytest

from sx_embodiments import CameraOptics, CameraOpticsAuthority, EmbodimentName, FactSource
from sx_embodiments.compose import camera_bindings
from sx_embodiments.errors import PartValidationError
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS
from sx_embodiments.known.das import QUEST3_REFERENCE_OPTICS
from sx_embodiments.known.panda import PANDA_ARM
from sx_embodiments.known.piper import PIPER_ARM
from sx_embodiments.known.so101 import SO101_ARM
from sx_embodiments.parts import PhysicalSpec


def test_piper_datasheet_facts() -> None:
    """AgileX PiPER datasheet: 1.5 kg payload, 626 mm reach, 4.2 kg mass."""
    assert PIPER_ARM.physical == PhysicalSpec(payload_kg=1.5, reach_m=0.626, mass_kg=4.2)


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


def test_quest_reference_optics_are_complete_and_revision_pinned() -> None:
    assert (QUEST3_REFERENCE_OPTICS.width, QUEST3_REFERENCE_OPTICS.height) == (1280, 960)
    assert QUEST3_REFERENCE_OPTICS.authority is CameraOpticsAuthority.REFERENCE_UNIT
    assert QUEST3_REFERENCE_OPTICS.image_from_camera[:6] == (
        868.31,
        0.0,
        640.18,
        0.0,
        868.31,
        482.07,
    )
    assert len(QUEST3_REFERENCE_OPTICS.source.revision) == 40


def test_camera_optics_reject_nonpositive_raster() -> None:
    with pytest.raises(PartValidationError):
        CameraOptics(
            width=0,
            height=720,
            image_from_camera=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            distortion_model="none",
            distortion_coefficients=(),
            authority=CameraOpticsAuthority.MODEL_NOMINAL,
            source=FactSource("https://example.test", "revision", "camera"),
        )


@pytest.mark.parametrize(
    ("model", "coefficients"),
    (("invented", ()), ("none", (0.0,)), ("equidistant", (0.0, 0.0, 0.0))),
)
def test_camera_optics_reject_incomplete_distortion_facts(
    model: str,
    coefficients: tuple[float, ...],
) -> None:
    with pytest.raises(PartValidationError):
        CameraOptics(
            width=1280,
            height=960,
            image_from_camera=(868.0, 0.0, 640.0, 0.0, 868.0, 480.0, 0.0, 0.0, 1.0),
            distortion_model=model,
            distortion_coefficients=coefficients,
            authority=CameraOpticsAuthority.MODEL_NOMINAL,
            source=FactSource("https://example.test", "revision", "camera"),
        )


def test_insta360_umi_entry_is_explicitly_incomplete() -> None:
    spec = DEVELOPMENT_EMBODIMENTS[EmbodimentName("insta360-umi")].spec
    assert not spec.layout_declared()  # jaw kinematics uncaptured -> enforcement skips
    assert camera_bindings(spec.attachments) == ()
