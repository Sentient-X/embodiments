"""The embodiment a recording carries, read at the current schema.

A recording's embodiment record is immutable: bytes written under an earlier schema stay
that schema. They are read through ``migrate`` and keep the identity they were recorded
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

from .embodiment import SCHEMA_VERSION, Embodiment, migrate
from .errors import EmbodimentSchemaError
from .identity import EmbodimentId, EmbodimentName


@dataclass(frozen=True)
class RecordedEmbodiment:
    """A recorded embodiment at the current schema, with the identity its bytes carry."""

    embodiment: Embodiment
    recorded_id: EmbodimentId


def read_recorded(document: Mapping[str, object]) -> RecordedEmbodiment:
    """Read one recorded embodiment document at the current schema.

    An earlier-schema document is converted, and its id is verified against its content; a
    current document is parsed exactly. Raises ``EmbodimentSchemaError`` otherwise.
    """
    if document.get("schema_version") != SCHEMA_VERSION:
        migration = migrate(document)
        return RecordedEmbodiment(migration.embodiment, migration.source_id)
    embodiment = Embodiment.from_dict(document)
    return RecordedEmbodiment(embodiment, embodiment.id)


@cache
def _historical() -> dict[EmbodimentName, dict[EmbodimentId, str]]:
    published: dict[EmbodimentName, dict[EmbodimentId, str]] = {}
    for entry in (resources.files("sx_embodiments") / "historical").iterdir():
        if not entry.name.endswith(".json"):
            continue
        text = entry.read_text()
        recorded = read_recorded(json.loads(text))
        name = recorded.embodiment.name
        if entry.name != f"{name}-{recorded.recorded_id}.json":
            raise EmbodimentSchemaError(f"historical document {entry.name} is misnamed")
        published.setdefault(name, {})[recorded.recorded_id] = text
    return published


def historical_identities(name: EmbodimentName) -> frozenset[EmbodimentId]:
    """The identities ``name``'s canonical package carried under earlier schemas."""
    return frozenset(_historical().get(name, {}))


def historical_document(name: EmbodimentName, identity: EmbodimentId) -> dict[str, object]:
    """The exact document ``name`` was published as under ``identity``, a fresh copy.

    What a previous deployment stored or a recording embedded; read it with
    ``read_recorded``. Raises ``KeyError`` for an identity this registry never published.
    """
    document: dict[str, object] = json.loads(_historical()[name][identity])
    return document
