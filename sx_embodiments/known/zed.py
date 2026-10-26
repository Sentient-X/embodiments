"""The Stereolabs ZED 2i as a stereo pair of cameras, from Stereolabs' own descriptions.

A ZED 2i is two 1/3" rolling-shutter imagers behind 2.1 mm lenses (the wide option), 120 mm
apart, exposed together and delivered over USB Video Class as one side-by-side frame, left eye
first. Each eye is one declared camera on its optical frame; a body pairs them as one stereo
track (:class:`~sx_embodiments.media.StereoPair`).

* **Modes.** Stereolabs' video-mode table (``docs.stereolabs.com/docs/video/camera-controls``):
  HD2K 4416x1242 at 15 fps, HD1080 3840x1080 at 30 or 15, HD720 2560x720 at 60, 30 or 15, and VGA
  1344x376 at 100, 60, 30 or 15, side by side; through a USB 2 link the camera offers VGA alone.
  An eye is declared at HD720 and 30 fps, the SDK's default mode.
* **Placement.** The eyes' optical centres sit 60 mm either side of the device centre along its
  baseline. A description's ZED frame follows Stereolabs' ROS convention (x out of the lenses,
  y towards the left eye, z up), so the left eye's optical frame is at +60 mm on y and the right
  eye's at -60 mm, each turned to the optical convention (z along the view, x right, y down).
  The static upper body's ``ZED_camera`` frame is that convention: its meshes put the body
  175 mm along y and the lens barrels at y = +/-60 mm, standing out along +x.
* **Optics.** The nominal intrinsics are the datasheet's maximum field of view, 110 x 70 degrees,
  over the HD720 raster, rounded to a micro-pixel as the RealSense imagers' are. A unit's own
  intrinsics are the factory calibration Stereolabs publishes per serial
  (:func:`sx_embodiments.factory_intrinsics.zed_factory_calibration`).

* **Inertial unit.** The ZED 2i's sensor module (its own USB HID interface, ``2b03:f881``) reports
  an accelerometer and gyroscope at 400 Hz, a magnetometer, a barometer and temperatures, on what
  Stereolabs' ``zed-open-capture`` calls the RAW axes (``README.md``, "Coordinates system", at
  ``93739154``): x down, y towards the left eye, z out of the lenses, about the device centre.
  Read that way, it is the ROS ZED frame turned a quarter turn about its y axis; but on the static
  upper body (2026-10-26) the accelerometer put the camera 8 deg nose up when the ZED was upright,
  facing forward and visibly 8 deg nose down, and a camera-tilt step the model turns nose up read
  as nose down. Only the axes half a turn about the camera's vertical fit both: as mounted, the
  module's x is down, y towards the right eye and z out of the back. :func:`zed_2i_imu` mounts on
  the body's ZED frame at that rotation. The SDK reads a per-unit camera-to-IMU transform from the
  device which ``zed-open-capture`` does not expose; the nominal offset is zero, which a gravity
  direction does not depend on.

Depth (the ZED SDK's stereo matching and its neural modes) is not modelled: the camera is two image
streams and an inertial unit.
"""

import math
from typing import Final, Literal

from ..identity import PartId
from ..parts import (
    CameraModality,
    CameraOptics,
    CameraOpticsAuthority,
    CameraSpec,
    FactSource,
    InertialQuantity,
    InertialSpec,
    SensorModel,
)

ZED_2I: Final = SensorModel("zed_2i")
Eye = Literal["left", "right"]
EYES: Final[tuple[Eye, Eye]] = ("left", "right")
#: Each eye's optical centre from the device centre along the baseline, metres.
ZED_2I_EYE_OFFSET_M: Final = 0.060
#: From a ROS body frame (x forward, y left, z up) to the optical convention.
OPTICAL_RPY: Final = (-math.pi / 2, 0.0, -math.pi / 2)
#: The declared eye raster and rate (HD720 at 30 fps).
ZED_2I_EYE_WIDTH: Final = 1280
ZED_2I_EYE_HEIGHT: Final = 720
ZED_2I_FPS: Final = 30.0

_DATASHEET: Final = FactSource(
    repository="https://cdn.sanity.io/files/s18ewfw4/staging/"
    "c059860f8fe49f3856f6b8da770eb13cc543ac2c.pdf/ZED%202i%20Datasheet%20v1.2.pdf",
    revision="ZED 2i Datasheet v1.2",
    path="Lens: field of view 110 (H) x 70 (V) x 120 (D) max., 2.1 mm",
)


#: The description frame a body names its ZED by, Stereolabs' ROS convention.
ZED_FRAME: Final = "ZED_camera"
#: The sensor module's accelerometer and gyroscope rate.
ZED_2I_IMU_HZ: Final = 400.0
_RAW_AXES: Final = FactSource(
    repository="https://github.com/stereolabs/zed-open-capture",
    revision="93739154903a66c112ee52c3c6fbfd83b7218617",
    path="README.md#coordinates-system (images/imu_axis.jpg): x down, y left eye, z forward",
)


def optical_frame(eye: Eye, prefix: str = "zed") -> str:
    """The name of an eye's optical frame on a body's ZED."""
    return f"{prefix}_{eye}_optical"


def eye_offset(eye: Eye) -> float:
    """The eye's optical centre on the ZED frame's y axis, metres."""
    return ZED_2I_EYE_OFFSET_M if eye == "left" else -ZED_2I_EYE_OFFSET_M


def zed_2i_eye(eye: Eye) -> CameraSpec:
    """One ZED 2i eye at HD720 and 30 fps with the datasheet's nominal optics."""
    width, height = ZED_2I_EYE_WIDTH, ZED_2I_EYE_HEIGHT
    return CameraSpec(
        part_id=PartId(f"zed-2i-{eye}"),
        model=ZED_2I,
        modality=CameraModality.RGB,
        fps=ZED_2I_FPS,
        optics=CameraOptics(
            width=width,
            height=height,
            image_from_camera=(
                round(width / 2 / math.tan(math.radians(110.0) / 2), 6),
                0.0,
                width / 2,
                0.0,
                round(height / 2 / math.tan(math.radians(70.0) / 2), 6),
                height / 2,
                0.0,
                0.0,
                1.0,
            ),
            distortion_model="none",
            distortion_coefficients=(),
            authority=CameraOpticsAuthority.MODEL_NOMINAL,
            source=_DATASHEET,
        ),
    )


def zed_2i_imu() -> InertialSpec:
    """The ZED 2i's sensor module, mounted on the body's ZED frame (:data:`ZED_FRAME`)."""
    half = math.sqrt(0.5)
    return InertialSpec(
        part_id=PartId("zed-2i-imu"),
        model=ZED_2I,
        rate_hz=ZED_2I_IMU_HZ,
        measures=tuple(InertialQuantity),
        mount_from_sensor=((0.0, 0.0, 0.0), (-half, 0.0, half, 0.0)),
        source=_RAW_AXES,
    )
