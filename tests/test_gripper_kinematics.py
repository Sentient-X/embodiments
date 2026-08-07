"""The one drive↔aperture relation, derived from declared gripper facts only."""

import pytest

from sx_embodiments import CoordinateUnit, GripperKinematicsError, embodiments
from sx_embodiments.curves import Curve1D, Knot
from sx_embodiments.identity import PartId
from sx_embodiments.layout import bounded_joint_layout
from sx_embodiments.parts import GripperSpec


def test_piper_affine_map_pins_the_parallel_jaw() -> None:
    gripper = embodiments["piper"].single_gripper
    assert gripper.aperture_from_drive(0.035) == pytest.approx(0.07)
    assert gripper.aperture_from_drive(0.0) == pytest.approx(0.0)
    assert gripper.drive_from_aperture(0.07) == pytest.approx(0.035)
    assert gripper.drive_from_aperture(0.05) == pytest.approx(0.025)


def test_round_trip_across_the_joint_box() -> None:
    gripper = embodiments["piper"].single_gripper
    lo, hi = gripper.joint_lower[0], gripper.joint_upper[0]
    for i in range(11):
        q = lo + (hi - lo) * i / 10
        assert gripper.drive_from_aperture(gripper.aperture_from_drive(q)) == pytest.approx(q)


def test_out_of_range_values_clamp_like_the_curve_path() -> None:
    gripper = embodiments["piper"].single_gripper
    assert gripper.aperture_from_drive(1.0) == pytest.approx(0.07)
    assert gripper.drive_from_aperture(-0.1) == pytest.approx(0.0)


def test_gap_curve_wins_over_the_affine_map() -> None:
    curved = GripperSpec(
        part_id=PartId("curved-jaw"),
        layout=bounded_joint_layout((("drive", CoordinateUnit.RADIAN, 0.0, 1.0),)),
        travel_m=(0.0, 0.08),
        gap_curve=Curve1D((Knot(0.0, 0.0), Knot(0.5, 0.06), Knot(1.0, 0.08))),
    )
    assert curved.aperture_from_drive(0.5) == pytest.approx(0.06)
    assert curved.drive_from_aperture(0.06) == pytest.approx(0.5)
    assert curved.drive_from_aperture(curved.aperture_from_drive(0.25)) == pytest.approx(0.25)


def test_underdeclared_gripper_refuses_instead_of_guessing() -> None:
    bare = GripperSpec(
        part_id=PartId("bare-jaw"),
        layout=bounded_joint_layout(
            (
                ("left", CoordinateUnit.RADIAN, 0.0, 1.0),
                ("right", CoordinateUnit.RADIAN, 0.0, 1.0),
            )
        ),
        travel_m=(0.0, 0.08),  # two drive joints: the affine single-joint map does not apply
    )
    with pytest.raises(GripperKinematicsError):
        bare.aperture_from_drive(0.5)
    with pytest.raises(GripperKinematicsError):
        bare.drive_from_aperture(0.04)
