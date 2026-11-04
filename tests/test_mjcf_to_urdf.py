"""mjcf_to_urdf carries every MJCF orientation into the URDF as the same rotation.

MuJoCo's ``euler`` follows ``<compiler eulerseq>`` (default ``xyz``, lower case: axes that
rotate with the frame, so ``R = Rx(a) Ry(b) Rz(c)``) in ``<compiler angle>`` units (default
degrees). URDF ``rpy`` is fixed-axis ``R = Rz(yaw) Ry(pitch) Rx(roll)``. Copying the three
numbers across is right only for a single-axis rotation.
"""

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from sx_embodiments.known.yor import YOR_MJCF, YOR_URDF

ROOT = Path(__file__).resolve().parents[1]
_SPEC = importlib.util.spec_from_file_location("mjcf_to_urdf", ROOT / "tools/mjcf_to_urdf.py")
assert _SPEC is not None and _SPEC.loader is not None
converter = importlib.util.module_from_spec(_SPEC)
sys.modules["mjcf_to_urdf"] = converter
_SPEC.loader.exec_module(converter)


def _rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _urdf_matrix(rpy: str) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy.split())
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)


def _converted_rpy(tmp_path: Path, euler: str, compiler: str = 'angle="radian"') -> str:
    mjcf = tmp_path / "robot.mjcf"
    mjcf.write_text(
        f"""<mujoco model="t">
  <compiler {compiler}/>
  <asset/>
  <worldbody>
    <body name="root">
      <body name="child" pos="0 0 0.1" euler="{euler}"/>
    </body>
  </worldbody>
</mujoco>"""
    )
    joint = converter.convert(mjcf).find("joint[@name='root_to_child']")
    assert joint is not None
    origin = joint.find("origin")
    assert origin is not None
    return origin.attrib["rpy"]


@pytest.mark.parametrize("euler", ["3.14 -1.57 0", "-0.7854 0 3.14", "0.3 -0.2 1.1"])
def test_multi_axis_euler_becomes_the_same_rotation(tmp_path: Path, euler: str) -> None:
    a, b, c = (float(value) for value in euler.split())
    mujoco_rotation = _rx(a) @ _ry(b) @ _rz(c)
    rpy = _converted_rpy(tmp_path, euler)
    np.testing.assert_allclose(_urdf_matrix(rpy), mujoco_rotation, atol=1e-9)
    # The old projection copied the numbers; for these that is another rotation.
    assert not np.allclose(_urdf_matrix(euler), mujoco_rotation, atol=1e-3)


def test_compiler_degrees_and_eulerseq_are_honoured(tmp_path: Path) -> None:
    # Default units are degrees; an upper-case sequence is fixed-axis (extrinsic) Z-Y-X.
    rpy = _converted_rpy(tmp_path, "30 -20 45", compiler='eulerseq="ZYX"')
    expected = _rx(math.radians(45)) @ _ry(math.radians(-20)) @ _rz(math.radians(30))
    np.testing.assert_allclose(_urdf_matrix(rpy), expected, atol=1e-9)


def test_euler_matches_mujoco_itself() -> None:
    mujoco = pytest.importorskip("mujoco")
    rng = np.random.default_rng(7)
    for sequence in ("xyz", "XYZ", "zyx", "ZYX", "yzx"):
        for index in range(50):
            angles = rng.uniform(-math.pi, math.pi, 3)
            if index % 10 == 0:
                angles[1] = math.pi / 2  # the gimbal-locked pitch
            quat = np.zeros(4)
            mujoco.mju_euler2Quat(quat, angles, sequence)
            matrix = np.zeros(9)
            mujoco.mju_quat2Mat(matrix, quat)
            ours = np.array(converter.euler_matrix(tuple(angles), sequence))
            np.testing.assert_allclose(ours, matrix.reshape(3, 3), atol=1e-12)
            rpy = " ".join(str(value) for value in converter.urdf_rpy(tuple(map(tuple, ours))))
            np.testing.assert_allclose(_urdf_matrix(rpy), ours, atol=1e-9)


def test_registered_yor_urdf_is_the_current_projection() -> None:
    generated = ET.tostring(
        converter.convert(YOR_MJCF.path()), encoding="utf-8", xml_declaration=True
    )
    assert YOR_URDF.path().read_bytes() == generated + b"\n"
