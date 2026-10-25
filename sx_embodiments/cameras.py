"""The open camera-model registry: one record per camera product, keyed by its model id.

A camera part names its product by id and carries its own nominal optics, so an embodiment
document stays complete without this table. The table is what a person reads: which product
an id is. Adding a camera adds a record here; it never changes the embodiment schema, and an
id this table does not list is still a valid part (a tenant's own camera).
"""

from dataclasses import dataclass
from typing import Final

from .parts import SensorModel


@dataclass(frozen=True, slots=True)
class CameraModel:
    model: SensorModel
    label: str


CAMERA_MODELS: Final = {
    record.model: record
    for record in (
        CameraModel(SensorModel("uvc_mono"), "UVC monochrome camera"),
        CameraModel(SensorModel("realsense_d405"), "Intel RealSense D405"),
        CameraModel(SensorModel("realsense_d435"), "Intel RealSense D435"),
        CameraModel(SensorModel("realsense_d435i"), "Intel RealSense D435i"),
        CameraModel(SensorModel("quest3_rgb"), "Meta Quest 3 passthrough RGB"),
        CameraModel(SensorModel("insta360_x5"), "Insta360 X5"),
        CameraModel(SensorModel("zed_2i"), "Stereolabs ZED 2i"),
    )
}


def camera_label(model: SensorModel) -> str:
    """The product's name, or its id when the registry does not list it."""
    record = CAMERA_MODELS.get(model)
    return record.label if record is not None else model.replace("_", " ")
