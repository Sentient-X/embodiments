"""The calibration library: solvers that measure a built robot into the calibration kinds.

Install with the ``calibrate`` extra (NumPy, SciPy and OpenCV). :mod:`.intrinsics` fits one
camera unit's lens from target views; :mod:`.mounts` places cameras on the links that carry
them from views of a target another link holds; :mod:`.robot` binds both to a robot instance
and its body, so what they return is a sealed :class:`~sx_embodiments.calibration.
CameraUnitCalibration` or :class:`~sx_embodiments.calibration.RigidCalibration` for that
instance to select.
"""

from .intrinsics import IntrinsicsLimits, IntrinsicsSolve, View, solve_intrinsics
from .lens import Lens
from .mounts import (
    CameraViews,
    MountSolve,
    MountView,
    SolverStatus,
    hand_eye,
    orient_cameras,
    predict,
    solve_mounts,
    solve_target,
)
from .robot import JointView, MountMeasurement, carrier_of, measure_intrinsics, measure_mount
from .targets import CalibrationSolveError, CharucoBoard, Checkerboard, Detection, Target

__all__ = [
    "CalibrationSolveError",
    "CameraViews",
    "CharucoBoard",
    "Checkerboard",
    "Detection",
    "IntrinsicsLimits",
    "IntrinsicsSolve",
    "JointView",
    "Lens",
    "MountMeasurement",
    "MountSolve",
    "MountView",
    "SolverStatus",
    "Target",
    "View",
    "carrier_of",
    "hand_eye",
    "measure_intrinsics",
    "measure_mount",
    "orient_cameras",
    "predict",
    "solve_intrinsics",
    "solve_mounts",
    "solve_target",
]
