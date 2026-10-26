#!/usr/bin/env python3
"""Render the registry bindings every driver crate composes its bodies from.

    python tools/render_driver_bindings.py           # rewrite every bindings file
    python tools/render_driver_bindings.py --check   # exit 1 when one disagrees

A driver crate under ``drivers/`` that composes a known body (``sx-damiao-can``'s B601-DM,
humanoid upper body and static upper body, ``sx-feetech-serial``'s SO-101) embeds that body's
document, rendered here from the direct-driven coordinates of ``robot.state`` in native state
order (a joint driven any other way, such as the humanoid's waist linkage, is outside every
driver's chain), and parses it with ``sx_embodiment_drivers::BodyBindings``: no bound,
address, port, model, sign or reduction is spelled in Rust. Every axis carries the port its
bus is commissioned under (``ActuatorBinding.bus_port``), so a body spanning several buses,
such as the static upper body's Damiao CAN bus and Feetech neck, is opened per port; each
crate driving one of its buses embeds the same document. A body is looked up in ``embodiments`` and
then in ``development_embodiments``, so a body still under construction (the static upper body)
is driven from the same registry facts its promotion will keep. ``vocabulary.json`` is the
registry's ``ActuatorModel`` and ``ActuatorBus``, which the driver crate's own lists are held
equal to, and each model's ``ACTUATOR_RATINGS`` entry (peak torque, supply range), which drivers
and commissioning read instead of spelling.
``tests/fixtures/drives.json`` in ``sx-damiao-can`` holds, for each Damiao body, the
``sx.instance-drive/v1`` documents :func:`sx_embodiments.instance.drive_document` renders for
its nominal instance and for one with a measured, reversed first joint, which the crate's
openers are tested against. ``tests/fixtures/media-plans.json`` in ``sx-embodiment-drivers``
holds the ``sx.media-plan/v3`` documents :func:`sx_embodiments.media.media_document`
renders, which the crate's ``MediaPlan`` is tested against, and ``media-poses.json`` beside it
holds, for each view of those plans that rides moving joints, its mount pose at seeded joint
positions as :func:`sx_embodiments.media.mount_pose_at` answers, which the crate's evaluation of
the rendered chain is held equal to. ``tests/fixtures/media-plans.json`` in ``sx-realsense``
holds the humanoid's head D435 planned with its imagers placed by unit, as colour and as the
infrared-pair body option, and ``tests/fixtures/media-plans.json`` in ``sx-uvc-stereo`` holds the
static upper body's ZED 2i placed by its serial with the factory calibration Stereolabs publishes
for it (``zed-2i-30371653.conf`` beside it), its inertial sensor placed by the same serial, which
``sx-stereolabs-sensors`` opens it by. ``tests/test_driver_bindings.py`` runs the same check.
"""

import argparse
import json
import sys
from pathlib import Path

from sx_embodiments import development_embodiments, embodiments
from sx_embodiments.calibration import (
    CalibrationMethod,
    JointUnitCalibration,
    Measurement,
    UnitSerial,
)
from sx_embodiments.composition import compose_build
from sx_embodiments.embodiment import Embodiment
from sx_embodiments.errors import UnknownEmbodimentError
from sx_embodiments.factory_intrinsics import zed_factory_calibration
from sx_embodiments.instance import (
    RobotInstance,
    Slot,
    SlotKind,
    drive_document,
    nominal_instance,
    resolve,
)
from sx_embodiments.known.humanoid import (
    HEAD_INFRARED_RIG,
    SENTIENT_HUMANOID_UPPER_BODY_INFRARED_BUILD,
)
from sx_embodiments.known.static_upper_body import STATIC_UPPER_BODY_RIG
from sx_embodiments.known.zed import EYES, ZED_2I_EYE_HEIGHT, ZED_2I_EYE_WIDTH, optical_frame
from sx_embodiments.layout import ACTUATOR_RATINGS, ActuatorBus, ActuatorModel
from sx_embodiments.parts import ChainSpec
from sx_embodiments.media import (
    NO_RIG,
    MediaRig,
    MicrophoneSpec,
    StereoPair,
    media_document,
    mount_pose_at,
)

ROOT = Path(__file__).resolve().parents[1]
#: Each rendered file and the registered body it carries.
BINDINGS: dict[Path, str] = {
    ROOT / "drivers/sx-damiao-can/bindings/b601-dm.json": "b601-dm",
    ROOT / "drivers/sx-damiao-can/bindings/sentient-humanoid-upper-body.json": (
        "sentient-humanoid-upper-body"
    ),
    ROOT / "drivers/sx-damiao-can/bindings/sentient-static-upper-body.json": (
        "sentient-static-upper-body"
    ),
    ROOT / "drivers/sx-feetech-serial/bindings/so101.json": "so101",
    # The static upper body's neck is a Feetech chain on its own port of the same body.
    ROOT / "drivers/sx-feetech-serial/bindings/sentient-static-upper-body.json": (
        "sentient-static-upper-body"
    ),
}
VOCABULARY = ROOT / "drivers/sx-embodiment-drivers/bindings/vocabulary.json"
DRIVES = ROOT / "drivers/sx-damiao-can/tests/fixtures/drives.json"
MEDIA = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/media-plans.json"
REALSENSE_MEDIA = ROOT / "drivers/sx-realsense/tests/fixtures/media-plans.json"
#: A placeholder device serial for the humanoid's head D435.
HEAD_D435 = "923322071108"
UVC_MEDIA = ROOT / "drivers/sx-uvc-stereo/tests/fixtures/media-plans.json"
#: The static upper body's ZED 2i, and the factory record Stereolabs publishes for its serial.
HEAD_ZED = "30371653"
ZED_FACTORY = ROOT / f"drivers/sx-uvc-stereo/tests/fixtures/zed-2i-{HEAD_ZED}.conf"
POSES = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/media-poses.json"
#: Joint positions per moving view the chain evaluation is checked at.
POSE_SAMPLES = 8
#: The Quest head's two cameras as one stereo track, with a microphone on the headset.
QUEST_RIG = MediaRig(
    stereo=(StereoPair("head", "head_left", "head_right"),),
    microphones=(MicrophoneSpec("head_mic", "quest3_microphone", 48_000, 1, "quest3s_head"),),
)
#: The Damiao bodies whose openers take an instance's drive.
DAMIAO_BODIES = ("b601-dm", "sentient-humanoid-upper-body", "sentient-static-upper-body")


def registered(name: str) -> Embodiment:
    """A registered body, executable or still in development."""
    try:
        return embodiments[name]
    except UnknownEmbodimentError:
        return development_embodiments[name]


def vocabulary() -> str:
    document = {
        "models": [model.value for model in ActuatorModel],
        "buses": [bus.value for bus in ActuatorBus],
        "ratings": {
            model.value: {
                "peak_torque_nm": rating.peak_torque_nm,
                "supply_volts": None
                if rating.supply is None
                else [rating.supply.min_volts, rating.supply.max_volts],
            }
            for model, rating in ((m, ACTUATOR_RATINGS[m]) for m in ActuatorModel)
        },
    }
    return json.dumps(document, indent=2) + "\n"


def drives() -> str:
    """Each Damiao body's nominal drive, and one with its first driven joint measured."""
    document: dict[str, object] = {}
    for name in DAMIAO_BODIES:
        body = registered(name)
        first = next(c for c in body.state.coordinates if c.axis.actuator is not None)
        slot = Slot(SlotKind.ACTUATOR, f"{first.instance}/{first.axis.name}")
        unit = UnitSerial(f"{name}-unit-1")
        measured = JointUnitCalibration(
            unit, 0.1, -1, None, Measurement(CalibrationMethod.HOMING, None)
        )
        instance = RobotInstance.of(body.id, {slot: unit}, (measured,))
        document[name] = {
            "nominal": drive_document(body, resolve(body, nominal_instance(body), ())),
            "measured": drive_document(body, resolve(body, instance, (measured,))),
        }
    return json.dumps(document, indent=2) + "\n"


def media() -> str:
    """The Quest head as one stereo track and as two, the humanoid, which declares no
    microphone, and the static upper body, whose ZED 2i is one stereo head track."""
    quest = embodiments["quest-ego"]
    humanoid = embodiments["sentient-humanoid-upper-body"]
    static = registered("sentient-static-upper-body")

    def plan(body: Embodiment, rig: MediaRig = NO_RIG) -> object:
        return media_document(body, resolve(body, nominal_instance(body), ()), rig)

    document = {
        "quest-ego-stereo": plan(quest, QUEST_RIG),
        "quest-ego-mono": plan(quest),
        "sentient-humanoid-upper-body": plan(humanoid),
        "sentient-static-upper-body": plan(static, STATIC_UPPER_BODY_RIG),
    }
    return json.dumps(document, indent=2) + "\n"


def realsense_media() -> str:
    """The humanoid's head D435 with its imagers placed: colour, and the infrared pair."""
    colour = embodiments["sentient-humanoid-upper-body"]
    infrared = compose_build(
        SENTIENT_HUMANOID_UPPER_BODY_INFRARED_BUILD,
        name="sentient-humanoid-upper-body-infrared",
        label="Sentient humanoid upper body, head infrared pair",
    )

    def placed(body: Embodiment, units: dict[str, str], rig: MediaRig = NO_RIG) -> object:
        instance = RobotInstance.of(
            body.id,
            {Slot(SlotKind.CAMERA, camera): UnitSerial(unit) for camera, unit in units.items()},
            (),
        )
        return media_document(body, resolve(body, instance, ()), rig)

    document = {
        "humanoid-head-color": placed(colour, {"head": f"{HEAD_D435}/color"}),
        "humanoid-head-infrared": placed(
            infrared,
            {"head_left": f"{HEAD_D435}/infrared_1", "head_right": f"{HEAD_D435}/infrared_2"},
            HEAD_INFRARED_RIG,
        ),
    }
    return json.dumps(document, indent=2) + "\n"


def uvc_media() -> str:
    """The static upper body's ZED 2i placed by serial, its eyes calibrated by the factory and
    its inertial sensor placed by the serial its sensor module reports."""
    body = registered("sentient-static-upper-body")
    factory = zed_factory_calibration(
        ZED_FACTORY.read_bytes(),
        HEAD_ZED,
        ZED_2I_EYE_WIDTH,
        ZED_2I_EYE_HEIGHT,
        (optical_frame("left"), optical_frame("right")),
    )
    calibrations = (factory.left, factory.right)
    instance = RobotInstance.of(
        body.id,
        {Slot(SlotKind.CAMERA, f"head_{eye}"): UnitSerial(f"{HEAD_ZED}/{eye}") for eye in EYES}
        | {Slot(SlotKind.COMPONENT, "head_imu"): UnitSerial(HEAD_ZED)},
        calibrations,
    )
    document = {
        "static-upper-body-head": media_document(
            body, resolve(body, instance, calibrations), STATIC_UPPER_BODY_RIG
        ),
    }
    return json.dumps(document, indent=2) + "\n"


def poses() -> str:
    """Each moving view's mount pose at seeded positions, from the description itself."""
    import random

    generator = random.Random(20261017)
    document: dict[str, object] = {}
    for key, body, rig in (
        ("sentient-humanoid-upper-body", embodiments["sentient-humanoid-upper-body"], NO_RIG),
        (
            "sentient-static-upper-body",
            registered("sentient-static-upper-body"),
            STATIC_UPPER_BODY_RIG,
        ),
    ):
        plan = media_document(body, resolve(body, nominal_instance(body), ()), rig)
        samples: list[object] = []
        for track in plan["tracks"]:  # type: ignore[union-attr]
            for view in track["views"]:  # type: ignore[index]
                if not view["moves_with"]:  # type: ignore[index]
                    continue
                for _ in range(POSE_SAMPLES):
                    positions = {
                        joint: round(generator.uniform(-1.2, 1.2), 6)
                        for joint in view["moves_with"]  # type: ignore[index]
                    }
                    translation, rotation = mount_pose_at(
                        body.urdf_bytes,
                        view["mount_frame"],  # type: ignore[index]
                        positions,
                    )
                    samples.append(
                        {
                            "track": track["track"],  # type: ignore[index]
                            "camera": view["camera"],  # type: ignore[index]
                            "positions": positions,
                            "translation_m": list(translation),
                            "rotation_xyzw": list(rotation),
                        }
                    )
        document[key] = samples
    return json.dumps(document, indent=2) + "\n"


def rendered() -> dict[Path, str]:
    files = {path: render(name) for path, name in BINDINGS.items()}
    files[VOCABULARY] = vocabulary()
    files[DRIVES] = drives()
    files[MEDIA] = media()
    files[REALSENSE_MEDIA] = realsense_media()
    files[UVC_MEDIA] = uvc_media()
    files[POSES] = poses()
    return files


def declared_homes(robot: Embodiment) -> tuple[dict[tuple[str, str], float], set[str]]:
    """Each chain joint's declared ``home``, by (component instance, joint name): where the
    driver returns it when an operation ends; and the instances that are chains. A part that
    declares none (a gripper) is absent, and the driver leaves that joint where it is."""
    homes: dict[tuple[str, str], float] = {}
    chains: set[str] = set()
    for component in robot.components:
        part = component.part
        if isinstance(part, ChainSpec):
            chains.add(str(component.instance))
            for joint, home in zip(part.joint_names, part.home, strict=True):
                homes[(str(component.instance), joint)] = home
    return homes, chains


def render(name: str) -> str:
    robot = registered(name)
    homes, chains = declared_homes(robot)
    axes: list[dict[str, object]] = []
    for coordinate in robot.state.coordinates:
        binding = coordinate.axis.actuator
        if binding is None:
            continue
        if coordinate.lower is None or coordinate.upper is None:
            raise ValueError(f"{name}/{coordinate.joint_name} is driven without bounds")
        home = homes.get((str(coordinate.instance), coordinate.axis.name))
        if home is None and str(coordinate.instance) in chains:
            raise ValueError(f"{name}/{coordinate.joint_name} is a chain joint with no home")
        axes.append(
            {
                "joint": coordinate.joint_name,
                "lower": coordinate.lower,
                "upper": coordinate.upper,
                "model": binding.model.value,
                "bus": binding.bus.value,
                "port": binding.bus_port,
                "provisional": binding.provisional,
                "bus_id": binding.bus_id,
                "sign": binding.sign,
                "reduction": binding.reduction,
            }
            | ({} if home is None else {"home": home})
        )
    document = {"embodiment": name, "id": str(robot.id), "axes": axes}
    return json.dumps(document, indent=2) + "\n"


def stale() -> list[Path]:
    return [
        path
        for path, text in rendered().items()
        if not path.is_file() or path.read_text(encoding="utf-8") != text
    ]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="compare instead of writing")
    arguments = parser.parse_args()
    if arguments.check:
        for path in stale():
            sys.stderr.write(f"stale: {path.relative_to(ROOT)}\n")
        return 1 if stale() else 0
    for path, text in rendered().items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
