#!/usr/bin/env python3
"""Translate the supplied Allisa gravity baseline, offline; never open a robot.

The output is a Teach-only profile, not a replacement for the platform's dynamics
or position gains. The reference's lumped wrist payload is included exactly once.
Run with sx-geometry available on PYTHONPATH; NumPy is the reference's dependency.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import socket
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

import numpy as np

from sx_geometry.frames import URDF_RPY, rotation_vector_matrix


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(values: object) -> str:
    return " ".join(format(float(v), ".17g") for v in values)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--drive", type=Path, required=True)
    parser.add_argument("--bindings", type=Path, required=True)
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--oracle", type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    # The reference creates unopened bus objects during offline construction.
    # Refuse every socket creation for the whole import, including the oracle.
    with patch.object(socket, "socket", side_effect=RuntimeError("offline import: no sockets")):
        sys.path.insert(0, str(source))
        reference = importlib.import_module("dual_arm_gravity")
        controller = reference.DualArmGravity(left_j4_ratio=0.70)
        drive = json.loads(args.drive.read_text())
        bindings = json.loads(args.bindings.read_text())
        # Generated bindings explicitly accept same-wiring historical revisions,
        # exactly as BodyBindings::is does for the hardware opener.
        if drive["embodiment"] not in (bindings["id"], *bindings.get("supersedes", [])):
            raise ValueError("Drive and binding embodiment identities differ")
        bound = {item["joint"]: item for item in bindings["axes"]}
        units = {item["joint"]: item for item in drive["joints"]}
        robot = ET.Element("robot", name="allisa_guidance_only")
        ET.SubElement(robot, "link", name="chest_link")
        settings = []
        motor_gains = []
        platform_zeros = []
        names = []
        for arm, motors in zip(controller.arms, reference.ARM_MOTORS, strict=True):
            if arm.friction_comp != 0:
                raise ValueError("The dual baseline must disable friction assist")
            friction = importlib.import_module(type(arm).__module__).FRICTION_TORQUE
            for index, (joint, link, motor, tracker) in enumerate(
                zip(arm.dynamics.joints, arm.dynamics.links, motors, arm.trackers, strict=True)
            ):
                name = joint.name
                axis = bound[name]
                unit = units[name]
                if axis["bus_id"] != motor.tx_id:
                    raise ValueError(f"Bus identity differs for {name}")
                gain = axis["sign"] * unit["direction"] * axis["reduction"]
                source_sign = arm.signs[motor.rx_id]
                if not math.isfinite(gain) or gain == 0 or source_sign not in (-1, 1):
                    raise ValueError(f"Invalid coordinate mapping for {name}")
                coordinate_gain = source_sign * gain
                coordinate_offset = source_sign * (-gain * unit["zero"] - arm.offsets[motor.rx_id])
                child = ET.SubElement(robot, "link", name=joint.child_link)
                inertial = ET.SubElement(child, "inertial")
                ET.SubElement(inertial, "origin", xyz=vector(link.com), rpy="0 0 0")
                ET.SubElement(inertial, "mass", value=str(link.mass))
                # This gravity-only model has no measured rotational inertias. A tiny
                # positive diagonal avoids cancellation when validating a point mass;
                # it never contributes to gravity and must never be used for M(q).
                ET.SubElement(inertial, "inertia", ixx="1e-9", ixy="0", ixz="0", iyy="1e-9", iyz="0", izz="1e-9")
                moving = ET.SubElement(robot, "joint", name=name, type="continuous")
                ET.SubElement(moving, "parent", link=joint.parent_link)
                ET.SubElement(moving, "child", link=joint.child_link)
                rotation = URDF_RPY.matrix(joint.rpy)
                if index == 3:
                    # Reference FK adds pi/2 to elbow q, not to the displayed CAD q.
                    rotation = rotation @ rotation_vector_matrix(
                        np.asarray(joint.axis) / np.linalg.norm(joint.axis) * (math.pi / 2)
                    )
                ET.SubElement(moving, "origin", xyz=vector(joint.xyz), rpy=vector(URDF_RPY.angles(rotation)))
                ET.SubElement(moving, "axis", xyz=vector(joint.axis))
                if name.startswith("left_"):
                    multiplier = arm.gravity_multipliers[name]
                else:
                    multiplier = (arm.j1_ratio, arm.j2_ratio, arm.j3_ratio, arm.j4_ratio,
                                  arm.j5_ratio, arm.j6_ratio, 0.0)[index]
                cap = 0.40 * min(motor.t_max, motor.effort_limit)
                if index == 4 or (name.startswith("left_") and index == 6):
                    cap = min(cap, 0.40)
                elif index == 5:
                    cap = min(cap, 0.75 if name.startswith("left_") else 0.80)
                settings.append({
                    "joint": name,
                    "coordinate_gain": coordinate_gain,
                    "coordinate_offset": coordinate_offset,
                    "gravity_scale": arm.gravity_ratio * multiplier,
                    "damping": tracker.kd_base * gain * gain,
                    "friction_coulomb": friction[name] * abs(gain),
                    "friction_scale": 0.0,
                    "torque_limit": cap * abs(gain),
                    "torque_slew_rate": reference.TORQUE_SLEW_RATE_NM_S * abs(gain),
                })
                names.append(name)
                motor_gains.append(gain)
                platform_zeros.append(unit["zero"])
        provenance = ["HANDOFF.md", "dual_arm_gravity.py", "can_bus.py",
                      "left_arm_freedrive.py", "right_arm_freedrive.py",
                      "left_arm_dynamics.py", "right_arm_dynamics.py",
                      "left_arm_calibrated_config.json", "right_arm_calibrated_config.json",
                      "left_arm_calibrated_inertials.json", "right_arm_gripper_inertials.json"]
        profile = {
            "schema": "sx.guidance/v1",
            "description_sha256": digest(args.urdf),
            "drive_sha256": digest(args.drive),
            "source_sha256": {name: digest(source / name) for name in provenance},
            "gravity": [0.0, 0.0, -9.80665],
            "model_urdf": ET.tostring(robot, encoding="unicode"),
            "joints": settings,
        }
        rng = np.random.default_rng(20261123)
        samples = []
        for case in range(256):
            if case == 0:
                position = np.zeros(14)
            else:
                position = np.array([rng.uniform(bound[n]["lower"], bound[n]["upper"]) for n in names])
            raw = np.array(motor_gains) * (position - np.array(platform_zeros))
            # A long dt bypasses transient slew, testing the full reference steady law
            # including payload, motor signs, gravity multipliers and effort clips.
            for arm in controller.arms:
                arm.prev_t_ff[:] = [0.0] * 7
            motor_torque = controller.torques(raw.tolist(), dt=1000.0)
            samples.append({"position": position.tolist(),
                            "motor_position": raw.tolist(),
                            "torque": (np.array(motor_torque) * motor_gains).tolist()})
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.oracle.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(json.dumps(profile, indent=2) + "\n")
        args.oracle.write_text(json.dumps({"joints": names, "tolerance_nm": 1e-5, "samples": samples}, indent=2) + "\n")
        print(f"Wrote separate Teach profile and {len(samples)} offline reference poses; no socket opened.")


if __name__ == "__main__":
    main()
