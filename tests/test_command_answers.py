"""Every bus declares what its command answer reports, and the drivers read it rendered."""

import json
from pathlib import Path

from sx_embodiments.layout import COMMAND_ANSWERS, ActuatorBus

VOCABULARY = Path(__file__).resolve().parents[1] / (
    "drivers/sx-embodiment-drivers/bindings/vocabulary.json"
)


def test_every_bus_declares_its_command_answer_and_the_vocabulary_carries_it() -> None:
    assert set(COMMAND_ANSWERS) == set(ActuatorBus)
    rendered = json.loads(VOCABULARY.read_text())["command_answers"]
    for bus, answer in COMMAND_ANSWERS.items():
        assert rendered[bus.value] == {
            "command_answer_reports_motion": answer.reports_motion,
            "command_answer_reports_temperature": answer.reports_temperature,
        }
