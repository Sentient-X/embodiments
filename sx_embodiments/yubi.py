"""Standard YUBI encoder interpretation for the existing four-bar hardware.

These facts do not apply to Parallel YUBI. A new geometry/encoder revision needs
its own measured profile before it can share the capture or control path.
"""

from __future__ import annotations

import math
from functools import cache

from .parts import GripperSpec


class YubiEncoderError(Exception):
    """The standard YUBI encoder violates its range or per-connection identity."""


@cache
def yubi_jaw() -> GripperSpec:
    """The single canonical YUBI jaw definition shared by both hands."""
    from sx_embodiments import development_embodiments
    from sx_embodiments.parts import GripperSpec

    jaw = next(
        component.part
        for component in development_embodiments["yubi"].components
        if component.instance == "left_jaw"
    )
    if not isinstance(jaw, GripperSpec):
        raise YubiEncoderError("canonical YUBI left_jaw is not a gripper")
    return jaw


# The finger four-bar over-travels the CAD open stop: the left unit's encoder read
# up to 0.876 rad against the URDF box of [0, 0.785398] on 2026-08-18 (the right
# unit tops out at 0.759 rad). A raw sample is refused only beyond the joint box
# plus this envelope; the aperture curve clamps at the CAD open gap for FK.
YUBI_ENCODER_OVERTRAVEL_RAD = math.radians(10.0)


def encoder_envelope_rad() -> tuple[float, float]:
    """The raw-encoder envelope: the canonical jaw's joint box plus the over-travel."""
    jaw = yubi_jaw()
    return (jaw.joint_lower[0], jaw.joint_upper[0] + YUBI_ENCODER_OVERTRAVEL_RAD)


def jaw_gap_m(joint_angle_rad: float) -> float:
    """Jaw opening in metres from the canonical YUBI embodiment."""
    return yubi_jaw().aperture_from_drive(joint_angle_rad)


class YubiEncoder:
    """Interpret one standard YUBI UART stream and fence firmware identity changes."""

    def __init__(self) -> None:
        self._firmware_device_id: str | None = None

    def parse(self, line: str) -> float | None:
        """Return a normalized joint angle, or no sample for diagnostic text."""
        try:
            if "," in line:
                device_id, _, value = line.partition(",")
                raw = float(value)
            else:
                device_id, raw = None, float(line)
        except ValueError:
            return None
        # Firmware sends the signed zero difference without modulo. Real units
        # jitter slightly negative at rest; fold one signed revolution to [0, pi].
        if not math.isfinite(raw) or not -2.0 * math.pi <= raw <= 2.0 * math.pi:
            raise YubiEncoderError(f"encoder emitted a value outside the AS5601 range: {raw!r}")
        angle = abs((raw + math.pi) % (2.0 * math.pi) - math.pi)
        maximum = encoder_envelope_rad()[1]
        if angle > maximum:
            raise YubiEncoderError(
                f"encoder joint angle {angle!r} exceeds the YUBI limit {maximum}"
            )
        if device_id is not None:
            if not device_id.strip():
                raise YubiEncoderError("encoder firmware emitted an empty device ID")
            if self._firmware_device_id is None:
                self._firmware_device_id = device_id
            elif device_id != self._firmware_device_id:
                raise YubiEncoderError(
                    "encoder firmware device ID changed from "
                    f"{self._firmware_device_id!r} to {device_id!r}"
                )
        return angle
