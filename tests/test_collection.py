"""An incomplete physical model must not erase documented collection equipment."""

from dataclasses import replace

import pytest

from sx_embodiments.assets import PackagedAsset
from sx_embodiments.collection import CollectionDeviceKind, CollectionMethod
from sx_embodiments.compose import SensorAttachment
from sx_embodiments.embodiment import Embodiment
from sx_embodiments.errors import PartValidationError
from sx_embodiments.identity import EmbodimentKind
from sx_embodiments.known import DEVELOPMENT_EMBODIMENTS, EmbodimentRegistry, collection_definitions
from sx_embodiments.known.das import QUEST_EGO_SPEC
from sx_embodiments.known.piper import PIPER_SPEC
from sx_embodiments.known.yubi import YUBI_SPEC
from sx_embodiments.parts import CameraSpec, SensorModel


def test_collection_catalog_does_not_materialize_or_promote_geometry(monkeypatch):
    def unavailable(*args, **kwargs):
        raise AssertionError("collection inventory must not materialize geometry")

    monkeypatch.setattr(EmbodimentRegistry, "__getitem__", unavailable)
    monkeypatch.setattr(Embodiment, "__post_init__", unavailable)
    monkeypatch.setattr(PackagedAsset, "provenanced_asset", unavailable)
    monkeypatch.setattr(PackagedAsset, "path", unavailable)
    definitions = collection_definitions()
    assert {row.collection.method for row in definitions} == set(CollectionMethod)
    assert {row.name for row in definitions} == {
        "yubi",
        "das-umi-v4",
        "insta360-umi",
        "quest-ego",
        "piperx-station",
        "b601-dm-station",
    }
    assert all(row.kind is not EmbodimentKind.ROBOT for row in definitions)
    assert YUBI_SPEC.name in DEVELOPMENT_EMBODIMENTS
    assert len(YUBI_SPEC.collection_devices) == 3
    assert PIPER_SPEC.collection is None
    assert PIPER_SPEC.collection_devices == ()


def test_declared_camera_inventory_tracks_component_changes():
    left = QUEST_EGO_SPEC.attachments[0]
    assert isinstance(left.part, CameraSpec)
    updated = replace(left.part, model=SensorModel.REALSENSE_D405)
    definition = replace(
        QUEST_EGO_SPEC,
        attachments=(replace(left, attachment=SensorAttachment(updated)),),
    )
    cameras = definition.collection_devices
    assert len(cameras) == 1
    assert cameras[0].quantity == 1
    assert cameras[0].kind is CollectionDeviceKind.CAMERA
    assert cameras[0].label == updated.model.value.replace("_", " ")
    assert cameras[0].source == updated.optics.source
    assert definition.collection.devices == ()


@pytest.mark.parametrize("quantity", [0, -1, True])
def test_inventory_rejects_invalid_device_counts(quantity):
    with pytest.raises(PartValidationError):
        replace(YUBI_SPEC.collection_devices[0], quantity=quantity)


@pytest.mark.parametrize("field", ["label", "placement", "details"])
def test_inventory_requires_device_descriptions(field):
    with pytest.raises(PartValidationError):
        replace(YUBI_SPEC.collection_devices[0], **{field: " "})


def test_catalog_copy_does_not_change_executable_content_identity():
    from sx_embodiments.embodiment import embodiment_from_definition

    original = embodiment_from_definition(QUEST_EGO_SPEC)
    updated = embodiment_from_definition(
        replace(
            QUEST_EGO_SPEC,
            collection=replace(
                QUEST_EGO_SPEC.collection, description="Updated catalog description."
            ),
        )
    )
    assert updated.id == original.id
    assert updated.to_dict() == original.to_dict()
