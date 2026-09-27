"""The embodiment a recording carries, read at the current schema.

A recording's embodiment record is immutable: bytes written under schema 13 stay schema 13.
They are read through ``convert_v13_to_v14`` and keep the identity they were recorded
under. The canonical packages this registry published under an earlier schema are kept
under ``historical/`` as their exact documents, so a recording made against one of them
is still recognised after the canonical package changes.
"""

from __future__ import annotations

import json
from collections.abc import Mapping
from dataclasses import dataclass
from functools import cache
from importlib import resources

from .embodiment import Embodiment, convert_v13_to_v14
from .errors import EmbodimentSchemaError
from .identity import EmbodimentId, EmbodimentName


@dataclass(frozen=True)
class RecordedEmbodiment:
    """A recorded embodiment at the current schema, with the identity its bytes carry."""

    embodiment: Embodiment
    recorded_id: EmbodimentId


def read_recorded(document: Mapping[str, object]) -> RecordedEmbodiment:
    """Read one recorded embodiment document at the current schema.

    A schema-13 document is converted, and its id is verified against its content; a
    current document is parsed exactly. Raises ``EmbodimentSchemaError`` otherwise.
    """
    if document.get("schema_version") == 13:
        migration = convert_v13_to_v14(document)
        return RecordedEmbodiment(migration.embodiment, migration.source_id)
    embodiment = Embodiment.from_dict(document)
    return RecordedEmbodiment(embodiment, embodiment.id)


@cache
def _historical() -> dict[EmbodimentName, frozenset[EmbodimentId]]:
    identities: dict[EmbodimentName, set[EmbodimentId]] = {}
    for entry in (resources.files("sx_embodiments") / "historical").iterdir():
        if not entry.name.endswith(".json"):
            continue
        recorded = read_recorded(json.loads(entry.read_text()))
        name = recorded.embodiment.name
        if entry.name != f"{name}-{recorded.recorded_id}.json":
            raise EmbodimentSchemaError(f"historical document {entry.name} is misnamed")
        identities.setdefault(name, set()).add(recorded.recorded_id)
    return {name: frozenset(ids) for name, ids in identities.items()}


def historical_identities(name: EmbodimentName) -> frozenset[EmbodimentId]:
    """The identities ``name``'s canonical package carried under earlier schemas."""
    return _historical().get(name, frozenset())
