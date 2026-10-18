"""The physical checks of a composition, on the owner's example: four Seeed arms on a base."""

import time
from dataclasses import replace

from sx_embodiments.checks import (
    BUDGET_S,
    Check,
    Collision,
    Finding,
    Payload,
    Severity,
    Stability,
    WorkspaceOverlap,
    WorkSurface,
    check_build,
    refusals,
)
from sx_embodiments.composition import compose_build
from sx_embodiments.known.b601 import B601_DM_MODULE
from sx_embodiments.mechanical import Build, Mate, Placed
from sx_embodiments.mobile_base import DeckSocket, DriveKind
from test_mobile_base import B601_BASE, four_arm_base, four_arms

TABLE = WorkSurface(centre=(0.75, 0.0, 0.4), size=(0.6, 1.0))


def checked(build: Build, surface: WorkSurface | None = None) -> tuple[Finding, ...]:
    body = compose_build(build, name="checked", label="Checked")
    return check_build(build, body.urdf_bytes, work_surface=surface)


def of(findings: tuple[Finding, ...], check: Check) -> list[Finding]:
    return [finding for finding in findings if finding.check is check]


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


def test_an_overloaded_base_is_refused() -> None:
    findings = checked(four_arms(four_arm_base(payload_kg=3.0)))
    (payload,) = refusals(findings)
    assert payload.check is Check.PAYLOAD and payload.entry == "base"
    assert isinstance(payload.detail, Payload)
    assert payload.detail.capacity_kg == 3.0 < payload.detail.load_kg


def test_arms_reaching_together_off_a_narrow_base_are_warned() -> None:
    light = four_arm_base(kind=DriveKind.DIFFERENTIAL)
    narrow = replace(light, mass_kg=4.0, drive=replace(light.drive, track_m=0.3, wheelbase_m=0.3))
    findings = checked(four_arms(narrow))
    (tips,) = of(findings, Check.STABILITY_EXTENDED)
    assert tips.severity is Severity.WARNING
    assert isinstance(tips.detail, Stability) and tips.detail.margin_m < 0.0


def test_the_checks_fit_the_interactive_budget() -> None:
    build = four_arms(four_arm_base())
    body = compose_build(build, name="timed", label="Timed")
    check_build(build, body.urdf_bytes, work_surface=TABLE)  # the meshes' bounds, once
    started = time.perf_counter()
    first = check_build(build, body.urdf_bytes, work_surface=TABLE)
    elapsed = time.perf_counter() - started
    assert elapsed < BUDGET_S, f"{elapsed:.3f} s"
    assert check_build(build, body.urdf_bytes, work_surface=TABLE) == first
