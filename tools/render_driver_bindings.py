#!/usr/bin/env python3
"""Render the registry bindings every driver crate composes its bodies from.

    python tools/render_driver_bindings.py           # rewrite every bindings file
    python tools/render_driver_bindings.py --check   # exit 1 when one disagrees

A driver crate under ``drivers/`` that composes a known body (``sx-damiao-can``'s B601-DM,
``sx-feetech-serial``'s SO-101) embeds that body's document, rendered here from ``robot.state``
in native state order, and parses it with ``sx_embodiment_drivers::BodyBindings``: no bound,
address, model, sign or reduction is spelled in Rust. ``vocabulary.json`` is the registry's
``ActuatorModel`` and ``ActuatorBus``, which the driver crate's own lists are held equal to.
``tests/test_driver_bindings.py`` runs the same check.
"""

import argparse
import json
import sys
from pathlib import Path

from sx_embodiments import embodiments
from sx_embodiments.layout import ActuatorBus, ActuatorModel

ROOT = Path(__file__).resolve().parents[1]
#: Each rendered file and the registered body it carries.
BINDINGS: dict[Path, str] = {
    ROOT / "drivers/sx-damiao-can/bindings/b601-dm.json": "b601-dm",
    ROOT / "drivers/sx-feetech-serial/bindings/so101.json": "so101",
}
VOCABULARY = ROOT / "drivers/sx-embodiment-drivers/bindings/vocabulary.json"


def vocabulary() -> str:
    document = {
        "models": [model.value for model in ActuatorModel],
        "buses": [bus.value for bus in ActuatorBus],
    }
    return json.dumps(document, indent=2) + "\n"


def rendered() -> dict[Path, str]:
    files = {path: render(name) for path, name in BINDINGS.items()}
    files[VOCABULARY] = vocabulary()
    return files


def render(name: str) -> str:
    robot = embodiments[name]
    axes: list[dict[str, object]] = []
    for coordinate in robot.state.coordinates:
        binding = coordinate.axis.actuator
        if binding is None or coordinate.lower is None or coordinate.upper is None:
            raise ValueError(f"{name}/{coordinate.joint_name} has no bounded direct drive")
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
