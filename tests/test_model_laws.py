"""Registry-wide laws: every body is held to the model, none is left on an older form."""

import json
from importlib import resources

import pytest

from sx_embodiments import Limb, PlacedEmbodiment, compose_embodiments, migrate
from sx_embodiments.cameras import CAMERA_MODELS
from sx_embodiments.embodiment import SCHEMA_VERSION, Embodiment, embodiment_from_definition
from sx_embodiments.errors import EmbodimentSchemaError
from sx_embodiments.known import definitions
from sx_embodiments.parts import CameraSpec, ChainSpec


def _bodies() -> list[Embodiment]:
    bodies: list[Embodiment] = []
    for definition in definitions():
        try:
            bodies.append(embodiment_from_definition(definition))
        except EmbodimentSchemaError:
            continue  # a development body without a description has no frames to check
    return bodies


BODIES = _bodies()


@pytest.mark.parametrize("body", BODIES, ids=lambda body: str(body.name))
def test_every_body_composes_so_its_mount_frames_are_true(body: Embodiment) -> None:
    """A mount frame is a link of the description and each chain's joints lie below it.

    The registry's own joint lookup accepts an exact name without scoping, so only
    composition proves the frames; a body that cannot be composed has a false graph.
    """
    composed = compose_embodiments(
        "probe", {"x": PlacedEmbodiment(body)}, label="Probe", kind=body.kind
    )
    assert composed.state.width == body.state.width


@pytest.mark.parametrize("body", BODIES, ids=lambda body: str(body.name))
def test_every_authored_chain_says_which_limb_it_serves(body: Embodiment) -> None:
    for component in body.components:
        if isinstance(component.part, ChainSpec):
            assert component.limb is not None and component.limb is not Limb.UNDOCUMENTED, (
                component.instance
            )


def test_every_registry_camera_names_a_listed_model() -> None:
    for body in BODIES:
        for component in body.components:
            if isinstance(component.part, CameraSpec):
                assert component.part.model in CAMERA_MODELS, (body.name, component.instance)


def test_every_published_earlier_document_still_reads() -> None:
    """A recording made against any package this registry ever published converts today."""
    published = [
        json.loads(entry.read_text())
        for entry in (resources.files("sx_embodiments") / "historical").iterdir()
        if entry.name.endswith(".json")
    ]
    assert published
    for document in published:
        assert document["schema_version"] < SCHEMA_VERSION
        migration = migrate(document)
        assert str(migration.source_id) == document["id"]
