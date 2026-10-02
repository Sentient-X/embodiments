"""Mint the transcripts sx-damiao-can is held to, by running Damiao's own DM_CAN.py.

The reference, run here unmodified:

- Damiao's motor SDK, https://gitee.com/kit-miao/motor-sdk at
  fb0e9fc5455ecb02ed13cc1f43de58078be61b07, ``Python例程/u2can/DM_CAN.py`` (the ``SDK/电机SDK``
  submodule of https://gitee.com/kit-miao/damiao), Damiao's modified copy of cmjang's
  DM_Control_Python (https://github.com/cmjang/DM_Control_Python at
  7da93877ba844d9587149d6f3a6385453aa8379f; MIT, Copyright (c) 2024 cmjang).
  ``MotorControl`` talks to a fake serial device that records every byte it writes and
  answers as the motors would; ``DM_CAN.sleep`` is replaced by a recorder, so each wait the
  vendor's sequencing takes is in the transcript.

``transcripts.json`` holds:

- ``vectors``: ``float_to_uint`` and ``uint_to_float`` over edge values, ``controlMIT`` frames
  for free MIT vectors (clamped gains, positions past ``Q_MAX``, nonzero velocity and torque),
  and feedback frames decoded into ``Motor`` state, each produced by DM_CAN.py itself.
- ``sessions``: the B601-DM chain's bus traffic, every write with the motors' answers and every
  wait, for a whole session (open, three commands, stop), a stop with a faulted and a silent
  motor, and an open whose motor reports a VMAX register its limit row does not carry. The
  chain's own sequencing (``src/chain.rs``) is mirrored below by calling DM_CAN.py's functions
  in the same order; the waits the chain adds between polls are recorded beside the vendor's.
  Axes and bindings are the registry's (``../b601_bindings.json``); master ids, gains and the
  gripper motor's range are Seeed's (``src/b601.rs`` cites them).
- ``cross_checks``: the same MIT vectors and decodes through two other implementations, and
  every place they diverge from DM_CAN.py, stated rather than averaged away:
  ``Sentient-X/sentient_can`` at 3837d224327d89ed30b6fdee3577769d8dae5f94 through its own
  encoder and decoder (``sentient_can_harness.cpp`` beside this file), and
  ``huggingface/lerobot`` at ff71cae1ae2d09fd035553c35da65888ed6c8304, whose
  ``DamiaoMotorsBus`` methods ``_float_to_uint``, ``_uint_to_float``, ``_encode_mit_packet`` and
  ``_decode_motor_state`` are executed from the pinned file with their degree conversions made
  identity (the station commands radians).

Run with every source checked out at its pinned commit and the harness built:

    DAMIAO_MOTOR_SDK=<motor-sdk> LEROBOT=<lerobot> SENTIENT_CAN_HARNESS=<binary> \\
        python3 drivers/sx-damiao-can/tests/fixtures/dm_can/mint.py

It needs numpy, which DM_CAN.py imports.
"""

from __future__ import annotations

import ast
import copy
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import types
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
OUT = HERE / "transcripts.json"
BINDINGS = HERE.parent / "b601_bindings.json"

MOTOR_SDK_COMMIT = "fb0e9fc5455ecb02ed13cc1f43de58078be61b07"
LEROBOT_COMMIT = "ff71cae1ae2d09fd035553c35da65888ed6c8304"
SENTIENT_CAN_COMMIT = "3837d224327d89ed30b6fdee3577769d8dae5f94"


def checkout(variable: str, commit: str) -> Path:
    root = Path(os.environ[variable])
    head = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, check=True
    ).stdout.strip()
    if head != commit:
        raise SystemExit(f"{variable} is at {head}, not {commit}")
    return root


SDK = checkout("DAMIAO_MOTOR_SDK", MOTOR_SDK_COMMIT)
sys.path.insert(0, str(SDK / "Python例程" / "u2can"))
import DM_CAN  # noqa: E402  (Damiao's own module, from the pinned checkout)
import numpy as np  # noqa: E402

# --- the recording world ---------------------------------------------------------------------

EVENTS: list[dict[str, Any]] = []


def record_sleep(seconds: float) -> None:
    EVENTS.append({"sleep": seconds})


DM_CAN.sleep = record_sleep

FEEDBACK_COUNTS: dict[int, int] = {}
_vendor_recv_data = DM_CAN.Motor.recv_data


def counting_recv_data(self: Any, q: float, dq: float, tau: float, err: int) -> None:
    FEEDBACK_COUNTS[self.SlaveID] = FEEDBACK_COUNTS.get(self.SlaveID, 0) + 1
    _vendor_recv_data(self, q, dq, tau, err)


DM_CAN.Motor.recv_data = counting_recv_data


class FakeMotor:
    """What the bus answers for one motor: its registers, enable state and last position."""

    def __init__(self, slave: int, master: int, motor_type: int) -> None:
        self.slave = slave
        self.master = master
        q_max, dq_max, tau_max = DM_CAN.MotorControl.Limit_Param[motor_type]
        self.registers: dict[int, float | int] = {
            int(DM_CAN.DM_variable.PMAX): float(q_max),
            int(DM_CAN.DM_variable.VMAX): float(dq_max),
            int(DM_CAN.DM_variable.TMAX): float(tau_max),
            int(DM_CAN.DM_variable.CTRL_MODE): 2,
        }
        self.enabled = False
        self.q: tuple[int, int] = (0x7F, 0xFF)
        self.fault: int | None = None
        self.silent = False


class FakeSerial:
    """A USB2CAN bridge: records each transmit packet and queues the motors' answers."""

    T_MOS = 30
    T_ROTOR = 31

    def __init__(self, motors: list[FakeMotor]) -> None:
        self.is_open = False
        self.motors = {motor.slave: motor for motor in motors}
        self.pending = b""

    def open(self) -> None:
        self.is_open = True

    def close(self) -> None:
        self.is_open = False

    def read_all(self) -> bytes:
        data, self.pending = self.pending, b""
        return data

    def write(self, packet: bytes) -> None:
        can_id = packet[13] | (packet[14] << 8)
        data = bytes(packet[21:29])
        answers = self.answer(can_id, data)
        EVENTS.append({"write": packet.hex(), "respond": [answer.hex() for answer in answers]})
        self.pending += b"".join(answers)

    def answer(self, can_id: int, data: bytes) -> list[bytes]:
        if can_id == 0x7FF:
            motor = self.motors.get(data[0] | (data[1] << 8))
            if motor is None or motor.silent:
                return []
            if data[2] == 0xCC:
                return [self.feedback(motor)]
            rid = data[3]
            if data[2] == 0x55:
                motor.registers[rid] = (
                    struct.unpack("<I", data[4:8])[0]
                    if DM_CAN.is_in_ranges(rid)
                    else struct.unpack("<f", data[4:8])[0]
                )
            value = motor.registers.get(rid, 0)
            payload = (
                struct.pack("<I", int(value))
                if DM_CAN.is_in_ranges(rid)
                else struct.pack("<f", float(value))
            )
            return [self.packet(motor.master, bytes([data[0], data[1], data[2], rid]) + payload)]
        motor = self.motors.get(can_id)
        if motor is None or motor.silent:
            return []
        if data[:7] == b"\xff" * 7 and data[7] in (0xFC, 0xFD, 0xFE):
            motor.enabled = data[7] == 0xFC if data[7] != 0xFE else motor.enabled
        else:
            motor.q = (data[0], data[1])
        return [self.feedback(motor)]

    def feedback(self, motor: FakeMotor) -> bytes:
        state = motor.fault if motor.fault is not None else int(motor.enabled)
        data = bytes(
            [
                (state << 4) | (motor.slave & 0x0F),
                *motor.q,
                0x7F,
                0xF7,
                0xFF,
                self.T_MOS,
                self.T_ROTOR,
            ]
        )
        return self.packet(motor.master, data)

    @staticmethod
    def packet(can_id: int, data: bytes) -> bytes:
        return bytes([0xAA, 0x11, 0x08]) + struct.pack("<I", can_id) + data + bytes([0x55])


# --- the B601 chain, as src/chain.rs sequences DM_CAN.py's calls ------------------------------

ANSWER_POLLS = 20
ANSWER_POLL_INTERVAL = 0.001
DM_TYPE = {
    "damiao_dm4310": DM_CAN.DM_Motor_Type.DM4310,
    "damiao_dm4340": DM_CAN.DM_Motor_Type.DM4340,
}
# src/b601.rs: master = id + 0x10; kp/kd from reBotArm_control_py's rebotarm_dm.yaml (the 4340P's
# kd 8 declared as 5, the MIT maximum); motor ranges are the URDF limits intersected with
# LeRobot's DM_PROFILE.joint_limits soft box (degrees), and the gripper motor's position_limits.
CONTROLLER = {
    1: (120.0, 5.0, (-150.0, 150.0)),
    2: (120.0, 5.0, (-200.0, 1.0)),
    3: (120.0, 5.0, (-200.0, 1.0)),
    4: (18.0, 2.0, (-80.0, 90.0)),
    5: (18.0, 2.0, (-90.0, 90.0)),
    6: (18.0, 2.0, (-90.0, 90.0)),
    7: (8.0, 1.0, None),
}
GRIPPER_MOTOR_RANGE = (-5.0, 0.0)
# The DM4340 velocity ranges Damiao publishes: DM_CAN.py's row, then C++例程 damiao.h's.
ACCEPTED_VMAX = {int(DM_CAN.DM_Motor_Type.DM4340): (10.0, 8.0)}
# The fake motors' answers to the enable/disable/MIT frames carry state nibble 1 when enabled
# and 0 when disabled, the codes in motorbridge/motorbridge@e8b3ac66
# motor_vendors/damiao/src/protocol.rs:29-41 (status_name).
VENDOR_LIMIT_PARAM = copy.deepcopy(DM_CAN.MotorControl.Limit_Param)


def restore_vendor_limits() -> None:
    """DM_CAN.py's change_limit_param mutates the class's shared table; undo it per session."""
    for row, original in zip(DM_CAN.MotorControl.Limit_Param, VENDOR_LIMIT_PARAM, strict=True):
        row[:] = original


def motor_range(axis: dict[str, Any]) -> tuple[float, float]:
    _kp, _kd, soft = CONTROLLER[axis["bus_id"]]
    if soft is None:
        return GRIPPER_MOTOR_RANGE
    return (max(axis["lower"], math.radians(soft[0])), min(axis["upper"], math.radians(soft[1])))


def commandable(axis: dict[str, Any]) -> tuple[float, float]:
    lower, upper = motor_range(axis)
    ends = [
        value / (axis["sign"] * axis["reduction"]) + axis["zero_offset"] for value in (lower, upper)
    ]
    return (max(axis["lower"], min(ends)), min(axis["upper"], max(ends)))


class ChainRefusalError(Exception):
    def __init__(self, error: dict[str, Any]) -> None:
        super().__init__(error)
        self.error = error


class Chain:
    def __init__(self, axes: list[dict[str, Any]], serial: FakeSerial) -> None:
        self.axes = axes
        self.serial = serial
        self.control = DM_CAN.MotorControl(serial)
        self.motors = []
        for axis in axes:
            motor = DM_CAN.Motor(DM_TYPE[axis["model"]], axis["bus_id"], axis["bus_id"] + 0x10)
            self.control.addMotor(motor)
            self.motors.append(motor)
        self.enabled = False
        self.stop_pending = False

    def counts(self) -> list[int]:
        return [FEEDBACK_COUNTS.get(motor.SlaveID, 0) for motor in self.motors]

    def fresh(self, before: list[int]) -> list[bool]:
        return [now > then for now, then in zip(self.counts(), before, strict=True)]

    def await_answers(self, before: list[int]) -> None:
        for attempt in range(ANSWER_POLLS):
            if all(self.fresh(before)):
                return
            if attempt > 0:
                record_sleep(ANSWER_POLL_INTERVAL)
            self.control.recv()

    def expect_all(self, before: list[int], expected: int) -> None:
        for motor, fresh in zip(self.motors, self.fresh(before), strict=True):
            if not fresh:
                raise ChainRefusalError({"silent": motor.SlaveID})
            if motor.getError() != expected:
                raise ChainRefusalError(
                    {"unexpected_state": motor.SlaveID, "state": motor.getError()}
                )

    def feedback(self) -> list[dict[str, Any]]:
        # DM_CAN.py does not decode the temperatures; they are bytes 6 and 7 of the fake
        # motors' frames.
        return [
            {
                "bus_id": motor.SlaveID,
                "state": int(motor.getError()),
                "position": float(motor.getPosition()),
                "velocity": float(motor.getVelocity()),
                "torque": float(motor.getTorque()),
                "mos_celsius": FakeSerial.T_MOS,
                "rotor_celsius": FakeSerial.T_ROTOR,
            }
            for motor in self.motors
        ]

    def open(self) -> list[list[float]]:
        limits: list[list[float]] = []
        for motor in self.motors:
            if not self.control.switchControlMode(motor, DM_CAN.Control_Type.MIT):
                raise ChainRefusalError({"mode_not_confirmed": motor.SlaveID})
            row = VENDOR_LIMIT_PARAM[motor.MotorType]
            found = []
            for index, register in enumerate(("PMAX", "VMAX", "TMAX")):
                accepted = (
                    ACCEPTED_VMAX.get(int(motor.MotorType), (row[1],))
                    if register == "VMAX"
                    else (row[index],)
                )
                value = self.control.read_motor_param(motor, DM_CAN.DM_variable[register])
                if value is None or float(np.float32(value)) not in [
                    float(np.float32(table)) for table in accepted
                ]:
                    raise ChainRefusalError(
                        {"limit_mismatch": motor.SlaveID, "register": register, "motor": value}
                    )
                found.append(float(np.float32(value)))
            if found != [float(np.float32(table)) for table in row]:
                self.control.change_limit_param(motor.MotorType, *found)
            limits.append([motor.SlaveID, *found])
        before = self.counts()
        for motor in self.motors:
            self.control.disable(motor)
        self.await_answers(before)
        self.expect_all(before, 0)
        return limits

    def command(self, joints: list[float]) -> dict[str, Any]:
        if self.stop_pending:
            raise ChainRefusalError({"stop_pending": True})
        targets = []
        for axis, joint in zip(self.axes, joints, strict=True):
            lower, upper = commandable(axis)
            assert lower <= joint <= upper, (axis["joint"], joint)
            targets.append(axis["sign"] * (joint - axis["zero_offset"]) * axis["reduction"])
        if not self.enabled:
            before = self.counts()
            for motor in self.motors:
                self.control.enable(motor)
            self.await_answers(before)
            self.expect_all(before, 1)
            self.enabled = True
        before = self.counts()
        for motor, target in zip(self.motors, targets, strict=True):
            kp, kd, _soft = CONTROLLER[motor.SlaveID]
            self.control.controlMIT(motor, kp, kd, target, 0.0, 0.0)
        self.await_answers(before)
        self.expect_all(before, 1)
        observed = [
            float(motor.getPosition()) / (axis["sign"] * axis["reduction"]) + axis["zero_offset"]
            for motor, axis in zip(self.motors, self.axes, strict=True)
        ]
        return {"observed": observed, "feedback": self.feedback()}

    def stop(self) -> dict[str, Any]:
        self.enabled = False
        before = self.counts()
        for motor in self.motors:
            self.control.disable(motor)
        self.await_answers(before)
        silent = [
            motor for motor, fresh in zip(self.motors, self.fresh(before), strict=True) if not fresh
        ]
        if silent:
            for motor in silent:
                self.control.refresh_motor_status(motor)
            self.await_answers(before)
        unproven: list[int] = []
        faults: list[list[int]] = []
        for motor, fresh in zip(self.motors, self.fresh(before), strict=True):
            state = motor.getError()
            if fresh and state == 0:
                continue
            if fresh and state not in (0, 1):
                faults.append([motor.SlaveID, state])
            unproven.append(motor.SlaveID)
        self.stop_pending = bool(unproven)
        return {"unproven": unproven, "faults": faults}


def fake_motors(axes: list[dict[str, Any]]) -> list[FakeMotor]:
    return [
        FakeMotor(axis["bus_id"], axis["bus_id"] + 0x10, DM_TYPE[axis["model"]]) for axis in axes
    ]


ACTIONS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.5, -1.0, -1.5, 0.25, -0.25, 1.0, 0.03],
    # The commandable edges: soft box on joint1, joint4's lower, joint6; URDF elsewhere; the
    # gripper at the motor's open angle.
    [math.radians(-150.0), -3.14, 0.0, 1.57, 1.57, math.radians(-90.0), 0.045],
]
SEQUENCING = (
    "chain sequencing over vendor primitives: the order of calls is src/chain.rs's, mirrored "
    "here; every frame, wait and decode inside a call is DM_CAN.py's own"
)


def step(entry: dict[str, Any]) -> dict[str, Any]:
    """A step result with how many events the session had recorded when it ended."""
    return {**entry, "event_count": len(EVENTS)}


def run_session(axes: list[dict[str, Any]], script: str) -> dict[str, Any]:
    EVENTS.clear()
    FEEDBACK_COUNTS.clear()
    restore_vendor_limits()
    motors = fake_motors(axes)
    if script == "vmax_8":
        for motor in motors[:3]:
            motor.registers[int(DM_CAN.DM_variable.VMAX)] = 8.0
    if script == "limit_mismatch":
        motors[1].registers[int(DM_CAN.DM_variable.VMAX)] = 9.0
    serial = FakeSerial(motors)
    chain = Chain(axes, serial)
    steps: list[dict[str, Any]] = []
    try:
        limits = chain.open()
        steps.append(step({"step": "open", "limits": limits}))
        if script in ("session", "vmax_8"):
            for action in ACTIONS if script == "session" else ACTIONS[1:2]:
                steps.append(step({"step": "command", "joints": action, **chain.command(action)}))
            steps.append(step({"step": "stop", "report": chain.stop()}))
        elif script == "stop_fault":
            steps.append(
                step({"step": "command", "joints": ACTIONS[1], **chain.command(ACTIONS[1])})
            )
            serial.motors[3].fault = 0xA
            serial.motors[5].silent = True
            steps.append(step({"step": "stop", "report": chain.stop()}))
            try:
                chain.command(ACTIONS[0])
            except ChainRefusalError as failure:
                steps.append(
                    step({"step": "command", "joints": ACTIONS[0], "error": failure.error})
                )
        elif script == "limit_mismatch":
            raise SystemExit("an open with a VMAX no Damiao table carries must fail")
    except ChainRefusalError as failure:
        steps.append(step({"step": "open", "error": failure.error}))
    restore_vendor_limits()
    return {"name": script, "sequencing": SEQUENCING, "events": list(EVENTS), "steps": steps}


# --- vectors ---------------------------------------------------------------------------------

DM4310 = int(DM_CAN.DM_Motor_Type.DM4310)
DM4340 = int(DM_CAN.DM_Motor_Type.DM4340)
MIT_VECTORS = [
    # motor_type, kp, kd, q, dq, tau
    (DM4310, 10.0, 0.5, 1.234, -2.5, 1.5),
    (DM4340, 500.0, 5.0, -12.5, 8.0, -28.0),
    (DM4340, 120.0, 8.0, 0.25, 9.5, 3.0),  # kd past its range, dq inside DM4340's 10 not 8
    (DM4310, 501.0, -1.0, 13.0, 31.0, -11.0),  # every field past its range
    (DM4310, 0.0, 0.0, 0.0, 0.0, 0.0),
    (DM4310, 8.0, 1.0, -5.0, 0.0, 0.0),
]
FLOAT_TO_UINT = [
    (0.0, -12.5, 12.5, 16),
    (12.5, -12.5, 12.5, 16),
    (-12.5, -12.5, 12.5, 16),
    (12.6, -12.5, 12.5, 16),
    (-3.14, -12.5, 12.5, 16),
    (0.1, -10.0, 10.0, 12),
    (499.9, 0.0, 500.0, 12),
    (math.nan, -12.5, 12.5, 16),
]
UINT_TO_FLOAT = [
    (0, -12.5, 12.5, 16),
    (65535, -12.5, 12.5, 16),
    (32767, -12.5, 12.5, 16),
    (2047, -30.0, 30.0, 12),
    (4095, -28.0, 28.0, 12),
    (1234, -10.0, 10.0, 12),
]
DECODES = [
    # slave id, motor type, feedback data
    (0x01, DM4340, "11851e7ff7ff1f21"),
    (0x04, DM4310, "04ffff000fff2a2b"),
    (0x07, DM4310, "a7000080080000ff"),
    (0x02, DM4340, "e2123456789abcde"),
]


def mit_frame(motor_type: int, kp: float, kd: float, q: float, dq: float, tau: float) -> str:
    EVENTS.clear()
    serial = FakeSerial([])
    control = DM_CAN.MotorControl(serial)
    motor = DM_CAN.Motor(motor_type, 0x01, 0x11)
    control.addMotor(motor)
    control.controlMIT(motor, kp, kd, q, dq, tau)
    packet = bytes.fromhex(EVENTS[0]["write"])
    return packet[21:29].hex()


def decode(slave: int, motor_type: int, data: str) -> dict[str, Any]:
    serial = FakeSerial([])
    control = DM_CAN.MotorControl(serial)
    motor = DM_CAN.Motor(motor_type, slave, slave + 0x10)
    control.addMotor(motor)
    serial.pending = FakeSerial.packet(slave + 0x10, bytes.fromhex(data))
    control.recv()
    return {
        "slave_id": slave,
        "motor_type": motor_type,
        "data": data,
        "q": float(motor.getPosition()),
        "dq": float(motor.getVelocity()),
        "tau": float(motor.getTorque()),
        "err": int(motor.getError()),
    }


def float_to_uint(x: float, lo: float, hi: float, bits: int) -> dict[str, Any]:
    row: dict[str, Any] = {"x": None if math.isnan(x) else x, "min": lo, "max": hi, "bits": bits}
    try:
        row["result"] = int(DM_CAN.float_to_uint(x, lo, hi, bits))
    except ValueError as error:
        # numpy refuses to convert a NaN; DM_CAN.py raises.
        row["error"] = f"ValueError: {error}"
    return row


def vectors() -> dict[str, Any]:
    return {
        "float_to_uint": [float_to_uint(x, lo, hi, bits) for x, lo, hi, bits in FLOAT_TO_UINT],
        "uint_to_float": [
            {
                "x": x,
                "min": lo,
                "max": hi,
                "bits": bits,
                "result": float(DM_CAN.uint_to_float(np.uint16(x), lo, hi, bits)),
            }
            for x, lo, hi, bits in UINT_TO_FLOAT
        ],
        "mit": [
            {
                "motor_type": t,
                "kp": kp,
                "kd": kd,
                "q": q,
                "dq": dq,
                "tau": tau,
                "data": mit_frame(t, kp, kd, q, dq, tau),
            }
            for t, kp, kd, q, dq, tau in MIT_VECTORS
        ],
        "decode": [decode(slave, t, data) for slave, t, data in DECODES],
    }


# --- cross-checks ----------------------------------------------------------------------------

SENTIENT_CAN_TYPE = {DM4310: 1, DM4340: 3}  # sentient_can MotorType numbering


def sentient_can(requests: list[str]) -> list[str]:
    binary = os.environ["SENTIENT_CAN_HARNESS"]
    result = subprocess.run(
        [binary], input="\n".join(requests) + "\n", capture_output=True, text=True, check=True
    )
    return result.stdout.splitlines()


def lerobot_methods() -> types.SimpleNamespace:
    root = checkout("LEROBOT", LEROBOT_COMMIT) / "src/lerobot/motors/damiao"
    tables: dict[str, Any] = {}
    exec(compile((root / "tables.py").read_text(), "tables.py", "exec"), tables)
    source = (root / "damiao.py").read_text()
    wanted = {"_float_to_uint", "_uint_to_float", "_encode_mit_packet", "_decode_motor_state"}
    namespace: dict[str, Any] = {
        **tables,
        "np": types.SimpleNamespace(radians=lambda x: x, degrees=lambda x: x),
    }
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.ClassDef) and node.name == "DamiaoMotorsBus":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name in wanted:
                    module = ast.Module(body=[item], type_ignores=[])
                    exec(compile(module, "damiao.py", "exec"), namespace)
    bus = types.SimpleNamespace()
    for name in wanted:
        setattr(bus, name, types.MethodType(namespace[name], bus))
    bus.MotorType = tables["MotorType"]
    bus.MOTOR_LIMIT_PARAMS = tables["MOTOR_LIMIT_PARAMS"]
    return bus


def cross_checks(vendor: dict[str, Any]) -> dict[str, Any]:
    lerobot = lerobot_methods()
    lerobot_type = {DM4310: lerobot.MotorType.DM4310, DM4340: lerobot.MotorType.DM4340}
    divergences: list[str] = []

    sc_frames = sentient_can(
        [
            f"mit {SENTIENT_CAN_TYPE[v['motor_type']]} 1 17 {v['kp']!r} {v['kd']!r} "
            f"{v['q']!r} {v['dq']!r} {v['tau']!r}"
            for v in vendor["mit"]
        ]
    )
    mit_rows = []
    for vector, sc in zip(vendor["mit"], sc_frames, strict=True):
        sc_data = sc.split()[1]
        lr_data = bytes(
            lerobot._encode_mit_packet(
                lerobot_type[vector["motor_type"]],
                vector["kp"],
                vector["kd"],
                vector["q"],
                vector["dq"],
                vector["tau"],
            )
        ).hex()
        mit_rows.append({**vector, "sentient_can": sc_data, "lerobot": lr_data})
        for name, data in (("sentient_can", sc_data), ("lerobot", lr_data)):
            if data != vector["data"]:
                divergences.append(
                    f"{name} MIT frame for {DM_CAN.DM_Motor_Type(vector['motor_type']).name} "
                    f"(kp {vector['kp']}, kd {vector['kd']}, q {vector['q']}, dq {vector['dq']}, "
                    f"tau {vector['tau']}) is {data}; DM_CAN.py packs {vector['data']}"
                )

    sc_decodes = sentient_can(
        [
            f"decode {SENTIENT_CAN_TYPE[d['motor_type']]} {d['slave_id']} "
            f"{d['slave_id'] + 0x10} {d['data']}"
            for d in vendor["decode"]
        ]
    )
    decode_rows = []
    for row, sc in zip(vendor["decode"], sc_decodes, strict=True):
        q, dq, tau, _t_mos, _t_rotor = sc.split()
        lr = lerobot._decode_motor_state(
            bytes.fromhex(row["data"]), lerobot_type[row["motor_type"]]
        )
        sc_values = [float(q), float(dq), float(tau)]
        lr_values = [float(lr[0]), float(lr[1]), float(lr[2])]
        decode_rows.append({**row, "sentient_can": sc_values, "lerobot": lr_values})
        vendor_values = [row["q"], row["dq"], row["tau"]]
        for name, values in (("sentient_can", sc_values), ("lerobot", lr_values)):
            # DM_CAN.py returns float32; the others keep double precision.
            for field, ours, theirs in zip(("q", "dq", "tau"), vendor_values, values, strict=True):
                if float(np.float32(theirs)) != ours:
                    divergences.append(
                        f"{name} decodes {field} of {row['data']} "
                        f"({DM_CAN.DM_Motor_Type(row['motor_type']).name}) as {theirs!r}; "
                        f"DM_CAN.py as {ours!r}"
                    )

    vendor_row = DM_CAN.MotorControl.Limit_Param[DM4340]
    lr_row = lerobot.MOTOR_LIMIT_PARAMS[lerobot.MotorType.DM4340]
    if [float(x) for x in vendor_row] != [float(x) for x in lr_row]:
        divergences.append(
            f"DM4340 limit row: DM_CAN.py {list(map(float, vendor_row))}, LeRobot and "
            f"sentient_can {list(map(float, lr_row))} (Damiao's C++ SDK damiao.h also carries 8 "
            "rad/s; Seeed's motorbridge carries 10). The chain reads the motor's VMAX register "
            "at open and refuses a mismatch."
        )
    divergences.append(
        "DM_CAN.py returns decoded state as float32 (np.float32); sentient_can and LeRobot "
        "return double precision. The crate returns f32, as DM_CAN.py does."
    )
    return {
        "sources": {
            "sentient_can": f"Sentient-X/sentient_can@{SENTIENT_CAN_COMMIT}",
            "lerobot": f"huggingface/lerobot@{LEROBOT_COMMIT}",
        },
        "mit": mit_rows,
        "decode": decode_rows,
        "divergences": divergences,
    }


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    bindings = json.loads(BINDINGS.read_text())
    axes = bindings["axes"]
    vendor = vectors()
    document = {
        "source": f"https://gitee.com/kit-miao/motor-sdk@{MOTOR_SDK_COMMIT} "
        "Python例程/u2can/DM_CAN.py",
        "bindings": {"embodiment": bindings["embodiment"], "id": bindings["id"]},
        "vectors": vendor,
        "sessions": [
            run_session(axes, "session"),
            run_session(axes, "vmax_8"),
            run_session(axes, "stop_fault"),
            run_session(axes, "limit_mismatch"),
        ],
        "cross_checks": cross_checks(vendor),
    }
    OUT.write_text(json.dumps(document, indent=1, ensure_ascii=False) + "\n")
    lerobot = checkout("LEROBOT", LEROBOT_COMMIT)
    provenance = {
        "sources": {
            "DM_CAN.py": {
                "repository": "https://gitee.com/kit-miao/motor-sdk",
                "revision": MOTOR_SDK_COMMIT,
                "path": "Python例程/u2can/DM_CAN.py",
                "sha256": sha256(SDK / "Python例程" / "u2can" / "DM_CAN.py"),
            },
            "damiao.py": {
                "repository": "https://github.com/huggingface/lerobot",
                "revision": LEROBOT_COMMIT,
                "path": "src/lerobot/motors/damiao/damiao.py",
                "sha256": sha256(lerobot / "src/lerobot/motors/damiao/damiao.py"),
            },
            "tables.py": {
                "repository": "https://github.com/huggingface/lerobot",
                "revision": LEROBOT_COMMIT,
                "path": "src/lerobot/motors/damiao/tables.py",
                "sha256": sha256(lerobot / "src/lerobot/motors/damiao/tables.py"),
            },
            "sentient_can_harness.cpp": {
                "repository": "Sentient-X/sentient_can",
                "revision": SENTIENT_CAN_COMMIT,
                "path": "drivers/sx-damiao-can/tests/fixtures/dm_can/sentient_can_harness.cpp",
                "sha256": sha256(HERE / "sentient_can_harness.cpp"),
            },
            "mint.py": {"sha256": sha256(Path(__file__))},
        },
        "transcripts.json": {"sha256": sha256(OUT)},
    }
    (HERE / "provenance.json").write_text(
        json.dumps(provenance, indent=1, ensure_ascii=False) + "\n"
    )


if __name__ == "__main__":
    main()
