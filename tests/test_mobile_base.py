"""A mobile base designed from its specs: its drive model, its description, and four Seeed
arms mated on its deck."""

import math
import xml.etree.ElementTree as ET

import pytest

from sx_embodiments import Embodiment
from sx_embodiments.composition import compose_build
from sx_embodiments.errors import CompositionError, InterfaceMismatchError
from sx_embodiments.known.b601 import B601_DM_MODULE
from sx_embodiments.known.so101 import SO101_MODULE
from sx_embodiments.layout import UndocumentedDrive
from sx_embodiments.mechanical import Build, InterfaceId, Mate, ModuleId, Placed
from sx_embodiments.mobile_base import (
    NO_QUALIFIED_BASE_MOTOR,
    DeckSocket,
    Drive,
    DriveError,
    DriveKind,
    MobileBase,
    Twist,
)

B601_BASE = InterfaceId("seeed-b601-base")
CORNERS = (
    ("front_left", 0.3, 0.22),
    ("front_right", 0.3, -0.22),
    ("rear_left", -0.3, 0.22),
    ("rear_right", -0.3, -0.22),
)


def drive(kind: DriveKind = DriveKind.MECANUM) -> Drive:
    return Drive(
        kind,
        wheel_radius_m=0.076,
        wheel_width_m=0.05,
        track_m=0.62,
        wheelbase_m=0.7,
        max_speed_m_s=1.0,
        max_acceleration_m_s2=0.8,
    )


def four_arm_base(*, payload_kg: float = 30.0, kind: DriveKind = DriveKind.MECANUM) -> MobileBase:
    return MobileBase(
        ModuleId("four-arm-base"),
        "Four-arm base",
        drive(kind),
        length_m=0.9,
        width_m=0.7,
        deck_height_m=0.4,
        mass_kg=45.0,
        payload_kg=payload_kg,
        deck=tuple(DeckSocket(name, B601_BASE, (x, y)) for name, x, y in CORNERS),
    )


def four_arms(base: MobileBase) -> Build:
    return Build(
        placed=(Placed("base", base.module),),
        mates=tuple(Mate(f"arm_{name}", B601_DM_MODULE, "base", name) for name, _, _ in CORNERS),
        robot="four_arm_base",
        root="world",
    )


@pytest.mark.parametrize("kind", tuple(DriveKind))
@pytest.mark.parametrize(
    "twist", (Twist(0.4, 0.0, 0.0), Twist(0.0, 0.0, 0.8), Twist(0.3, 0.0, -0.5))
)
def test_wheel_speeds_and_odometry_are_inverse(kind: DriveKind, twist: Twist) -> None:
    model = drive(kind)
    wheels = model.wheel_velocities(twist)
    assert len(wheels) == len(model.wheels)
    back = model.twist(wheels)
    assert math.isclose(back.vx, twist.vx, abs_tol=1e-12)
    assert math.isclose(back.vy, twist.vy, abs_tol=1e-12)
    assert math.isclose(back.wz, twist.wz, abs_tol=1e-12)


def test_a_mecanum_base_moves_sideways_and_a_differential_one_refuses_to() -> None:
    sideways = Twist(0.0, 0.3, 0.0)
    assert drive().twist(drive().wheel_velocities(sideways)) == pytest.approx(sideways)
    with pytest.raises(DriveError, match="sideways"):
        drive(DriveKind.DIFFERENTIAL).wheel_velocities(sideways)


def test_a_twist_past_the_wheels_rated_speed_is_refused() -> None:
    with pytest.raises(DriveError, match="faster"):
        drive().wheel_velocities(Twist(1.5, 0.0, 0.0))
    with pytest.raises(DriveError, match="expected 4"):
        drive().twist((0.0, 0.0))


@pytest.mark.parametrize("kind", tuple(DriveKind))
def test_the_description_is_ready_for_simulation(kind: DriveKind) -> None:
    base = four_arm_base(kind=kind)
    robot = ET.fromstring(base.module.description)  # type: ignore[arg-type]
    wheels = {
        joint.attrib["name"]
        for joint in robot.findall("joint")
        if joint.attrib["type"] == "continuous"
    }
    assert wheels == set(base.drive.wheels)
    for link in robot.findall("link"):
        if link.attrib["name"] == "deck":
            continue
        mass = link.find("inertial/mass")
        assert mass is not None and float(mass.attrib["value"]) > 0.0
        assert link.find("collision/geometry") is not None
    masses = sum(float(mass.attrib["value"]) for mass in robot.iter("mass"))
    expected = base.mass_kg * (1.02 if kind is DriveKind.DIFFERENTIAL else 1.0)
    assert masses == pytest.approx(expected)


def test_no_base_motor_is_qualified_so_the_wheels_have_no_driver() -> None:
    axes = four_arm_base().spec.layout.axes
    assert [axis.name for axis in axes] == list(four_arm_base().drive.wheels)
    assert all(axis.actuation == UndocumentedDrive(NO_QUALIFIED_BASE_MOTOR) for axis in axes)
    assert all(axis.actuator is None for axis in axes)


def test_four_seeed_arms_compose_on_the_base_through_its_deck_sockets() -> None:
    body = compose_build(four_arms(four_arm_base()), name="four-arm-base", label="Four arms")
    assert body.state.width == 4 + 4 * (6 + 1)
    assert {component.instance for component in body.components} >= {
        "base_drive",
        *(f"arm_{name}_arm" for name, _, _ in CORNERS),
    }
    assert Embodiment.from_json(body.to_json()) == body


def test_a_deck_socket_takes_only_its_interface() -> None:
    with pytest.raises(InterfaceMismatchError):
        Build(
            placed=(Placed("base", four_arm_base().module),),
            mates=(Mate("arm", SO101_MODULE, "base", "front_left"),),
            robot="wrong",
            root="world",
        )


@pytest.mark.parametrize(
    ("change", "reason"),
    (
        ({"deck_height_m": 0.05}, "above the wheels"),
        ({"length_m": 0.5}, "within the chassis"),
        (
            {"deck": (DeckSocket("far", B601_BASE, (1.0, 0.0)),)},
            "off the deck",
        ),
        ({"payload_kg": 0.0}, "positive"),
    ),
)
def test_an_impossible_base_is_refused(change: dict[str, object], reason: str) -> None:
    facts: dict[str, object] = {
        "module_id": ModuleId("bad"),
        "label": "Bad",
        "drive": drive(),
        "length_m": 0.9,
        "width_m": 0.7,
        "deck_height_m": 0.4,
        "mass_kg": 45.0,
        "payload_kg": 30.0,
    }
    with pytest.raises(CompositionError, match=reason):
        MobileBase(**(facts | change))  # type: ignore[arg-type]


def test_a_base_alone_composes_as_a_first_party_body() -> None:
    """A base made from its specs has no packaged source: its description is first-party."""
    build = Build(placed=(Placed("base", four_arm_base().module),), robot="cart", root="world")
    body = compose_build(build, name="cart", label="Cart")
    (description,) = (asset for asset in body.assets if asset.uri.endswith("body.urdf"))
    assert description.provenance.license_id == "Apache-2.0"
    assert body.state.width == 4
