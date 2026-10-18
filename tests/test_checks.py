"""The physical checks of a composition, on the owner's example: four Seeed arms on a base."""

import math
import time
from dataclasses import replace

import pytest

from sx_embodiments.assembly import Pose
from sx_embodiments.checks import (
    BUDGET_S,
    MAX_FINDINGS,
    MEMBER_CAP,
    BaseField,
    ChangeBase,
    Check,
    ChecksTooLargeError,
    Collision,
    CollisionGeometryError,
    Finding,
    Move,
    Payload,
    Remedy,
    Severity,
    Stability,
    WorkspaceOverlap,
    WorkSurface,
    check_build,
    refusals,
    require_checkable,
)
from sx_embodiments.composition import compose_build
from sx_embodiments.known.b601 import B601_DM_MODULE
from sx_embodiments.mechanical import Box, Build, Mate, Placed, rigid_module
from sx_embodiments.mobile_base import DeckSocket, DriveKind, MobileBase
from test_mobile_base import B601_BASE, four_arm_base, four_arms

TABLE = WorkSurface(centre=(0.75, 0.0, 0.4), size=(0.6, 1.0))


def checked(build: Build, surface: WorkSurface | None = None) -> tuple[Finding, ...]:
    body = compose_build(build, name="checked", label="Checked")
    return check_build(build, body.urdf_bytes, work_surface=surface)


def of(findings: tuple[Finding, ...], check: Check) -> list[Finding]:
    return [finding for finding in findings if finding.check is check]


def applied(base: MobileBase, remedies: tuple[Remedy, ...]) -> MobileBase:
    """What a designer does with remedies on an unturned base: move a deck socket by a move's
    x and y, or set a base spec."""
    for remedy in remedies:
        match remedy:
            case Move():
                _, socket = remedy.mount.split(".")
                base = replace(
                    base,
                    deck=tuple(
                        replace(seat, xy=(seat.xy[0] + remedy.by[0], seat.xy[1] + remedy.by[1]))
                        if seat.name == socket
                        else seat
                        for seat in base.deck
                    ),
                )
            case ChangeBase(field=BaseField.TRACK_M | BaseField.WHEELBASE_M):
                base = replace(base, drive=replace(base.drive, **{remedy.field.value: remedy.to}))
            case ChangeBase():
                base = replace(base, **{remedy.field.value: remedy.to})
    return base


def test_four_arms_on_a_base_compose_with_warnings_and_no_refusal() -> None:
    findings = checked(four_arms(four_arm_base()), TABLE)
    assert refusals(findings) == ()
    assert of(findings, Check.BASE_NOT_QUALIFIED)[0].entry == "base"
    # Neighbouring arms share part of their workspace: a warning, never a refusal.
    overlaps = of(findings, Check.WORKSPACE_OVERLAP)
    assert {(finding.entry, finding.other) for finding in overlaps} >= {
        ("arm_front_left", "arm_front_right"),
        ("arm_rear_left", "arm_rear_right"),
    }
    for finding in overlaps:
        assert finding.severity is Severity.WARNING
        assert isinstance(finding.detail, WorkspaceOverlap)
        assert 0.0 < finding.detail.fraction_of_smaller < 1.0
    # The front arms reach the table ahead of the base; the rear ones are too far back.
    unreached = {finding.entry for finding in of(findings, Check.WORK_SURFACE_REACH)}
    assert "arm_front_left" not in unreached and "arm_rear_left" in unreached


def test_arms_placed_overlapping_are_refused_for_collision() -> None:
    base = replace(
        four_arm_base(),
        deck=(
            DeckSocket("left", B601_BASE, (0.0, 0.02)),
            DeckSocket("right", B601_BASE, (0.0, -0.02)),
        ),
    )
    build = Build(
        placed=(Placed("base", base.module),),
        mates=(
            Mate("left", B601_DM_MODULE, "base", "left"),
            Mate("right", B601_DM_MODULE, "base", "right"),
        ),
        robot="crowded",
        root="world",
    )
    (collision,) = refusals(checked(build))
    assert collision.check is Check.COLLISION_AT_REST
    assert (collision.entry, collision.other) == ("left", "right")
    assert isinstance(collision.detail, Collision) and collision.detail.fraction == 1.0
    # It says which member to move, on which mount, by how much; doing that clears it.
    (move,) = collision.remedies
    assert isinstance(move, Move) and (move.entry, move.mount) == ("right", "base.right")
    assert move.by[1] < 0.0 and "base.right" in collision.message
    fixed = replace(build, placed=(Placed("base", applied(base, (move,)).module),))
    assert of(checked(fixed), Check.COLLISION_AT_REST) == []


def test_an_overloaded_base_is_refused() -> None:
    findings = checked(four_arms(four_arm_base(payload_kg=3.0)))
    (payload,) = refusals(findings)
    assert payload.check is Check.PAYLOAD and payload.entry == "base"
    assert isinstance(payload.detail, Payload)
    assert payload.detail.capacity_kg == 3.0 < payload.detail.load_kg
    (rating,) = payload.remedies
    assert rating == ChangeBase("base", BaseField.PAYLOAD_KG, rating.to)
    assert refusals(checked(four_arms(applied(four_arm_base(payload_kg=3.0), (rating,))))) == ()


def test_arms_reaching_together_off_a_narrow_base_are_warned() -> None:
    light = four_arm_base(kind=DriveKind.DIFFERENTIAL)
    narrow = replace(light, mass_kg=4.0, drive=replace(light.drive, track_m=0.3, wheelbase_m=0.3))
    findings = checked(four_arms(narrow))
    (tips,) = of(findings, Check.STABILITY_EXTENDED)
    assert tips.severity is Severity.WARNING
    assert isinstance(tips.detail, Stability) and tips.detail.margin_m < 0.0
    assert {remedy.field for remedy in tips.remedies if isinstance(remedy, ChangeBase)} >= {
        BaseField.TRACK_M,
        BaseField.WHEELBASE_M,
    }
    wider = applied(narrow, tips.remedies)
    assert of(checked(four_arms(wider)), Check.STABILITY_EXTENDED) == []


def test_the_checks_fit_the_interactive_budget() -> None:
    build = four_arms(four_arm_base())
    body = compose_build(build, name="timed", label="Timed")
    check_build(build, body.urdf_bytes, work_surface=TABLE)  # the meshes' bounds, once
    started = time.process_time()
    first = check_build(build, body.urdf_bytes, work_surface=TABLE)
    elapsed = time.process_time() - started
    assert elapsed < BUDGET_S, f"{elapsed:.3f} s"
    assert check_build(build, body.urdf_bytes, work_surface=TABLE) == first


def _arms(count: int) -> Build:
    """``count`` B601 arms set down 0.8 m apart on a square grid."""
    side = math.ceil(math.sqrt(count))
    return Build(
        placed=tuple(
            Placed(
                f"arm_{index}",
                B601_DM_MODULE,
                Pose((0.8 * (index % side), 0.8 * (index // side), 0.0)),
            )
            for index in range(count)
        ),
        robot="arms",
        root="world",
    )


def test_a_composition_past_the_member_cap_is_refused_before_it_is_checked() -> None:
    build = _arms(64)  # the proto's most placements
    started = time.process_time()
    with pytest.raises(ChecksTooLargeError, match=f"at most {MEMBER_CAP} members"):
        require_checkable(build)
    with pytest.raises(ChecksTooLargeError):
        check_build(build, b"<robot/>")
    assert time.process_time() - started < BUDGET_S


def test_the_largest_checkable_composition_fits_the_budget() -> None:
    build = _arms(MEMBER_CAP)
    body = compose_build(build, name="arms", label="Arms")
    check_build(build, body.urdf_bytes)  # the meshes' bounds, once
    started = time.process_time()
    findings = check_build(build, body.urdf_bytes)
    elapsed = time.process_time() - started
    assert elapsed < BUDGET_S, f"{elapsed:.3f} s"
    assert len(findings) <= MAX_FINDINGS


def test_an_unreadable_collision_shape_is_a_typed_refusal() -> None:
    broken = rigid_module("block", "Block", shape=Box((0.1, 0.1, 0.1)), mass_kg=1.0, plug=None)
    description = broken.description
    assert isinstance(description, bytes)
    bad = replace(
        broken, description=description.replace(b'size="0.1 0.1 0.1"', b'size="0.1 0.1"')
    )
    build = Build(placed=(Placed("block", bad),), robot="block", root="world")
    with pytest.raises(CollisionGeometryError, match="3 positive size"):
        check_build(build, build.urdf())
