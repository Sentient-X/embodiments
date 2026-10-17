#!/usr/bin/env python3
"""Render the registry bindings every driver crate composes its bodies from.

    python tools/render_driver_bindings.py           # rewrite every bindings file
    python tools/render_driver_bindings.py --check   # exit 1 when one disagrees

A driver crate under ``drivers/`` that composes a known body (``sx-damiao-can``'s B601-DM and
humanoid upper body, ``sx-feetech-serial``'s SO-101) embeds that body's document, rendered here
from the direct-driven coordinates of ``robot.state`` in native state order (a joint driven any
other way, such as the humanoid's waist linkage, is outside every driver's chain), and parses it
with ``sx_embodiment_drivers::BodyBindings``: no bound,
address, model, sign or reduction is spelled in Rust. ``vocabulary.json`` is the registry's
``ActuatorModel`` and ``ActuatorBus``, which the driver crate's own lists are held equal to.
``tests/fixtures/drives.json`` in ``sx-damiao-can`` holds, for each Damiao body, the
``sx.instance-drive/v1`` documents :func:`sx_embodiments.instance.drive_document` renders for
its nominal instance and for one with a measured, reversed first joint, which the crate's
openers are tested against. ``tests/fixtures/media-plans.json`` in ``sx-embodiment-drivers``
holds the ``sx.media-plan/v1`` documents :func:`sx_embodiments.media.media_document`
renders, which the crate's ``MediaPlan`` is tested against, and
``tests/fixtures/media-plans.json`` in ``sx-realsense`` the humanoid's head D435 planned with
its imagers placed by unit, as colour and as the infrared-pair body option.
``tests/test_driver_bindings.py`` runs the same check.
"""

import argparse
import json
import sys
from pathlib import Path

from sx_embodiments import embodiments
from sx_embodiments.calibration import (
    CalibrationMethod,
    JointUnitCalibration,
    Measurement,
    UnitSerial,
)
from sx_embodiments.composition import compose_build
from sx_embodiments.embodiment import Embodiment
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
from sx_embodiments.layout import ActuatorBus, ActuatorModel
from sx_embodiments.media import NO_RIG, MediaRig, MicrophoneSpec, StereoPair, media_document

ROOT = Path(__file__).resolve().parents[1]
#: Each rendered file and the registered body it carries.
BINDINGS: dict[Path, str] = {
    ROOT / "drivers/sx-damiao-can/bindings/b601-dm.json": "b601-dm",
    ROOT / "drivers/sx-damiao-can/bindings/sentient-humanoid-upper-body.json": (
        "sentient-humanoid-upper-body"
    ),
    ROOT / "drivers/sx-feetech-serial/bindings/so101.json": "so101",
}
VOCABULARY = ROOT / "drivers/sx-embodiment-drivers/bindings/vocabulary.json"
DRIVES = ROOT / "drivers/sx-damiao-can/tests/fixtures/drives.json"
MEDIA = ROOT / "drivers/sx-embodiment-drivers/tests/fixtures/media-plans.json"
REALSENSE_MEDIA = ROOT / "drivers/sx-realsense/tests/fixtures/media-plans.json"
#: A placeholder device serial for the humanoid's head D435.
HEAD_D435 = "923322071108"
#: The Quest head's two cameras as one stereo track, with a microphone on the headset.
QUEST_RIG = MediaRig(
    stereo=(StereoPair("head", "head_left", "head_right"),),
    microphones=(MicrophoneSpec("head_mic", "quest3_microphone", 48_000, 1, "quest3s_head"),),
)
#: The Damiao bodies whose openers take an instance's drive.
DAMIAO_BODIES = ("b601-dm", "sentient-humanoid-upper-body")


def vocabulary() -> str:
    document = {
        "models": [model.value for model in ActuatorModel],
        "buses": [bus.value for bus in ActuatorBus],
    }
    return json.dumps(document, indent=2) + "\n"


def drives() -> str:
    """Each Damiao body's nominal drive, and one with its first driven joint measured."""
    document: dict[str, object] = {}
    for name in DAMIAO_BODIES:
        body = embodiments[name]
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
    """The Quest head as one stereo track and as two, and the humanoid, which declares none."""
    quest = embodiments["quest-ego"]
    humanoid = embodiments["sentient-humanoid-upper-body"]

    def plan(body: Embodiment, rig: MediaRig = NO_RIG) -> object:
        return media_document(body, resolve(body, nominal_instance(body), ()), rig)

    document = {
        "quest-ego-stereo": plan(quest, QUEST_RIG),
        "quest-ego-mono": plan(quest),
        "sentient-humanoid-upper-body": plan(humanoid),
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


def rendered() -> dict[Path, str]:
    files = {path: render(name) for path, name in BINDINGS.items()}
    files[VOCABULARY] = vocabulary()
    files[DRIVES] = drives()
    files[MEDIA] = media()
    files[REALSENSE_MEDIA] = realsense_media()
    return files


def render(name: str) -> str:
    robot = embodiments[name]
    axes: list[dict[str, object]] = []
    for coordinate in robot.state.coordinates:
        binding = coordinate.axis.actuator
        if binding is None:
            continue
        if coordinate.lower is None or coordinate.upper is None:
            raise ValueError(f"{name}/{coordinate.joint_name} is driven without bounds")
        axes.append(
            {
                "joint": coordinate.joint_name,
                "lower": coordinate.lower,
                "upper": coordinate.upper,
                "model": binding.model.value,
                "bus": binding.bus.value,
                "bus_id": binding.bus_id,
                "sign": binding.sign,
                "reduction": binding.reduction,
            }
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
