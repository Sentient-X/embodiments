"""Every declared camera's default optics describe pixels its imager has: square ones.

An imager's pixels are square unless its product says otherwise, so a default intrinsic matrix
whose fx and fy differ is a derivation error, not a fact about the camera: the ZED 2i's was the
datasheet's maximum field of view taken per axis, fx/fy = 0.87, which QC then flagged on every
recorded frame as calibration that does not describe its video. No camera declared here has
anamorphic pixels; one that does states it beside its optics and is the one exception.
"""

import pytest

from sx_embodiments import Embodiment, EmbodimentError, development_embodiments, embodiments

#: How far a nominal fx/fy may stray from 1: a factory calibration's own spread (the ZED 2i's
#: is 0.9999) is far inside it, and a per-axis field-of-view derivation (0.87) far outside.
SQUARE_TOLERANCE = 0.01
#: Cameras whose product declares non-square pixels, by model; none so far.
ANAMORPHIC_MODELS: frozenset[str] = frozenset()


def _bodies() -> list[Embodiment]:
    bodies: list[Embodiment] = []
    for registry in (embodiments, development_embodiments):
        for name in registry:
            try:
                bodies.append(registry[name])
            except EmbodimentError:
                continue  # a body that does not build is another test's failure
    return bodies


@pytest.mark.parametrize("body", _bodies(), ids=lambda body: str(body.name))
def test_every_cameras_default_pixels_are_square(body: Embodiment) -> None:
    for binding in body.cameras:
        if str(binding.camera.model) in ANAMORPHIC_MODELS:
            continue
        optics = binding.camera.optics
        fx, fy = optics.image_from_camera[0], optics.image_from_camera[4]
        assert abs(fx / fy - 1.0) <= SQUARE_TOLERANCE, (
            f"{body.name}/{binding.name}: fx {fx} and fy {fy} describe non-square pixels"
        )
