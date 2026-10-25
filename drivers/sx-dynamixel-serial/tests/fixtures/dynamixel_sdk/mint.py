"""Mint the transcript sx-dynamixel-serial is held to, by running ROBOTIS's own DynamixelSDK.

The references, run here unmodified:

- ROBOTIS's DynamixelSDK, https://github.com/ROBOTIS-GIT/DynamixelSDK at
  a8d35d51c87e7d253ece1baa963f498a8d933fa0, ``python/src/dynamixel_sdk/`` (Apache-2.0,
  Copyright 2017 ROBOTIS CO., LTD.). Its ``Protocol2PacketHandler``, ``GroupSyncWrite`` and
  ``GroupSyncRead`` talk to a capturing port in the shape of its ``PortHandler``, which records
  every packet written and hands back the status packet a servo would answer, built with the
  SDK's own ``updateCRC``.
- LeRobot, https://github.com/huggingface/lerobot at v0.6.0
  (30da8e687a6dfc617fcd94afc367ac7071c376ce): ``MotorsBus._unnormalize`` and
  ``MotorNormMode`` are executed from the pinned ``src/lerobot/motors/motors_bus.py`` with
  ``DynamixelMotorsBus.apply_drive_mode`` (False, checked below), and the control-table
  addresses and model numbers are checked against the pinned ``dynamixel/tables.py``.

The chain is ``bindings.json`` beside this file: a test pair, not a registered body (no
registered body binds a Dynamixel yet), an XL330-M288 finger at id 1 in percent under
current-based position, and an XM430-W350 wrist at id 2 in degrees under position mode, its
drive map reversed. ``xl330_xm430.json`` holds:

- ``calibration``: each servo's LeRobot calibration, and the registers it is read back from.
- ``identify``: per servo, ``ping`` answered with its model number, then
  ``read1ByteTxRx(Hardware_Error_Status)`` answered 0.
- ``read_calibration``: per servo, ``read4ByteTxRx`` of ``Homing_Offset``,
  ``Min_Position_Limit`` and ``Max_Position_Limit`` and ``read1ByteTxRx(Drive_Mode)``,
  answered with the calibration (the homing offset in two's complement).
- ``configure``: ``Torque_Enable`` 0 on every servo, ``Return_Delay_Time`` 0 on every servo,
  then each servo's ``Operating_Mode``, each ``write1ByteTxRx`` acknowledged.
- ``torque_enable``: ``Torque_Enable`` 1 per servo, acknowledged.
- ``goal_positions``: each target vector's ticks and the ``GroupSyncWrite`` packet
  (``Goal_Position``, four bytes) ``txPacket`` built for it.
- ``safe_stop``: ``Torque_Enable`` 0 per servo, acknowledged, then per servo
  ``readTxRx(Torque_Enable, 7)`` answered with torque 0 and no hardware error.
- ``sync_read``: one ``GroupSyncRead`` of ``Present_Position`` and both answers.

Run with both sources checked out at their pinned commits:

    DYNAMIXEL_SDK=<DynamixelSDK> LEROBOT=<lerobot> \\
        python3 drivers/sx-dynamixel-serial/tests/fixtures/dynamixel_sdk/mint.py

It needs pyserial (which ``dynamixel_sdk`` imports).
"""

from __future__ import annotations

import ast
import enum
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
OUT = HERE / "xl330_xm430.json"
BINDINGS = HERE / "bindings.json"

SDK_COMMIT = "a8d35d51c87e7d253ece1baa963f498a8d933fa0"
LEROBOT_COMMIT = "30da8e687a6dfc617fcd94afc367ac7071c376ce"
SDK_FILES = (
    "python/src/dynamixel_sdk/robotis_def.py",
    "python/src/dynamixel_sdk/protocol2_packet_handler.py",
    "python/src/dynamixel_sdk/group_sync_write.py",
    "python/src/dynamixel_sdk/group_sync_read.py",
    "python/src/dynamixel_sdk/port_handler.py",
)
LEROBOT_FILES = (
    "src/lerobot/motors/motors_bus.py",
    "src/lerobot/motors/dynamixel/dynamixel.py",
    "src/lerobot/motors/dynamixel/tables.py",
)


def checkout(variable: str, commit: str) -> Path:
    root = Path(os.environ[variable])
    head = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, check=True
    ).stdout.strip()
    if head != commit:
        raise SystemExit(f"{variable} is at {head}, not {commit}")
    return root


SDK = checkout("DYNAMIXEL_SDK", SDK_COMMIT)
LEROBOT = checkout("LEROBOT", LEROBOT_COMMIT)
sys.path.insert(0, str(SDK / "python/src"))
import dynamixel_sdk as dxl  # noqa: E402  (ROBOTIS's own package, from the pinned checkout)

UNCALIBRATED_ZERO = 0.0

# --- LeRobot's unnormalization, executed from the pinned file --------------------------------

MOTORS_BUS = (LEROBOT / LEROBOT_FILES[0]).read_text(encoding="utf-8")


def lerobot_definition(name: str) -> ast.stmt:
    for node in ast.walk(ast.parse(MOTORS_BUS)):
        if isinstance(node, ast.ClassDef | ast.FunctionDef) and node.name == name:
            return node
    raise SystemExit(f"{name} is not in {LEROBOT_FILES[0]}")


NAMESPACE: dict[str, Any] = {"Enum": enum.Enum}
exec(  # noqa: S102  (LeRobot's own definitions, from the pinned checkout)
    compile(
        ast.Module(
            body=[lerobot_definition("MotorNormMode"), lerobot_definition("_unnormalize")],
            type_ignores=[],
        ),
        LEROBOT_FILES[0],
        "exec",
    ),
    NAMESPACE,
)
MotorNormMode = NAMESPACE["MotorNormMode"]
_unnormalize = NAMESPACE["_unnormalize"]

DYNAMIXEL = (LEROBOT / LEROBOT_FILES[1]).read_text(encoding="utf-8")
TABLES = (LEROBOT / LEROBOT_FILES[2]).read_text(encoding="utf-8")
if "    apply_drive_mode = False\n" not in DYNAMIXEL:
    raise SystemExit("DynamixelMotorsBus.apply_drive_mode is no longer False")
REGISTERS = {
    "Model_Number": (0, 2),
    "Return_Delay_Time": (9, 1),
    "Drive_Mode": (10, 1),
    "Operating_Mode": (11, 1),
    "Homing_Offset": (20, 4),
    "Max_Position_Limit": (48, 4),
    "Min_Position_Limit": (52, 4),
    "Torque_Enable": (64, 1),
    "Hardware_Error_Status": (70, 1),
    "Goal_Position": (116, 4),
    "Present_Position": (132, 4),
}
for name, (address, size) in REGISTERS.items():
    if f'"{name}": ({address}, {size}),' not in TABLES:
        raise SystemExit(f"tables.py no longer puts {name} at {address}")
MODELS = {"dynamixel_xl330_m288": ("xl330-m288", 1200), "dynamixel_xm430_w350": ("xm430-w350", 1020)}
for model, number in MODELS.values():
    if f'"{model}": {number},' not in TABLES:
        raise SystemExit(f"tables.py no longer numbers {model} {number}")
    if f'"{model}": 4096,' not in TABLES:
        raise SystemExit(f"tables.py no longer gives {model} 4096 steps")
NORM_MODES = {"finger": MotorNormMode.RANGE_0_100, "wrist": MotorNormMode.DEGREES}
OPERATING_MODES = {"finger": 5, "wrist": 3}  # CURRENT_POSITION, POSITION

# (id, drive_mode, homing_offset, range_min, range_max)
CALIBRATION = [(1, 0, -1024, 1200, 2900), (2, 1, 300, 600, 3500)]


class Calibration:
    def __init__(self, drive_mode: int, range_min: int, range_max: int) -> None:
        self.drive_mode = drive_mode
        self.range_min = range_min
        self.range_max = range_max


class Bus:
    """The attributes ``_unnormalize`` reads from a ``DynamixelMotorsBus``."""

    apply_drive_mode = False

    def __init__(self, axes: list[dict[str, Any]]) -> None:
        self.names = {axis["bus_id"]: axis["joint"] for axis in axes}
        self.models = {axis["bus_id"]: MODELS[axis["model"]][0] for axis in axes}
        self.model_resolution_table = {model: 4096 for model, _ in MODELS.values()}
        self.motors = {
            axis["joint"]: type("Motor", (), {"norm_mode": NORM_MODES[axis["joint"]]})
            for axis in axes
        }
        self.calibration = {
            self.names[motor_id]: Calibration(drive_mode, lo, hi)
            for motor_id, drive_mode, _, lo, hi in CALIBRATION
        }

    def _id_to_name(self, motor_id: int) -> str:
        return self.names[motor_id]

    def _id_to_model(self, motor_id: int) -> str:
        return self.models[motor_id]

    _unnormalize = _unnormalize


# --- the recording port ----------------------------------------------------------------------


class CapturePort:
    """A ``PortHandler`` that records every written packet and replays status packets."""

    def __init__(self) -> None:
        self.is_using = False
        self.written: list[list[int]] = []
        self.pending: list[int] = []

    def clearPort(self) -> None:  # noqa: N802 - DynamixelSDK's port protocol
        pass

    def writePort(self, packet: list[int]) -> int:  # noqa: N802
        self.written.append(list(packet))
        return len(packet)

    def readPort(self, length: int) -> list[int]:  # noqa: N802
        taken, self.pending = self.pending[:length], self.pending[length:]
        return taken

    def setPacketTimeout(self, packet_length: int) -> None:  # noqa: N802
        pass

    def isPacketTimeout(self) -> bool:  # noqa: N802
        return True


HANDLER = dxl.PacketHandler(2.0)


def status(dxl_id: int, params: list[int], error: int = 0) -> list[int]:
    """One status packet: header, id, length, 0x55, error, params, the SDK's CRC."""
    length = len(params) + 4
    packet = [0xFF, 0xFF, 0xFD, 0x00, dxl_id, length & 0xFF, length >> 8, 0x55, error, *params]
    crc = HANDLER.updateCRC(0, packet, len(packet))
    return [*packet, crc & 0xFF, crc >> 8]


def le(value: int, size: int) -> list[int]:
    return list((value & ((1 << (8 * size)) - 1)).to_bytes(size, "little"))


def exchange(port: CapturePort, response: list[int], call: Any) -> dict[str, str]:
    port.pending = list(response)
    before = len(port.written)
    outcome = call()
    result = outcome[1] if isinstance(outcome, tuple) and len(outcome) == 3 else (
        outcome[0] if isinstance(outcome, tuple) else outcome
    )
    if result != dxl.COMM_SUCCESS:
        raise SystemExit(f"DynamixelSDK answered {result} to {port.written[before:]}")
    (request,) = port.written[before:]
    if port.pending:
        raise SystemExit(f"the SDK left {port.pending} unread")
    return {"request": bytes(request).hex(), "response": bytes(response).hex()}


ACTIONS = [
    [0.0, 0.0],
    [0.45, 1.0],
    [-0.8, -1.5],
    [0.9, 1.5],
]


def actuator(axis: dict[str, Any], joint: float) -> float:
    """The registry's drive map: ``sign * (joint - zero) * reduction``."""
    return axis["sign"] * (joint - UNCALIBRATED_ZERO) * axis["reduction"]


def wire(axis: dict[str, Any], target: float) -> float:
    """LeRobot's wire value: degrees, or percent of the declared range."""
    value = actuator(axis, target)
    if NORM_MODES[axis["joint"]] is MotorNormMode.RANGE_0_100:
        ends = (actuator(axis, axis["lower"]), actuator(axis, axis["upper"]))
        low, high = min(ends), max(ends)
        return (value - low) * 100.0 / (high - low)
    return math.degrees(value)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    axes: list[dict[str, Any]] = json.loads(BINDINGS.read_text(encoding="utf-8"))["axes"]
    port = CapturePort()
    ids = [axis["bus_id"] for axis in axes]
    model_of = {axis["bus_id"]: MODELS[axis["model"]][1] for axis in axes}
    calibration = {row[0]: row for row in CALIBRATION}

    identify = []
    for dxl_id in ids:
        identify.append(
            exchange(
                port,
                status(dxl_id, [*le(model_of[dxl_id], 2), 0x33]),
                lambda i=dxl_id: HANDLER.ping(port, i),
            )
        )
        identify.append(
            exchange(port, status(dxl_id, [0]), lambda i=dxl_id: HANDLER.read1ByteTxRx(port, i, 70))
        )

    read_calibration = []
    for dxl_id in ids:
        _, drive_mode, homing, low, high = calibration[dxl_id]
        for address, value in ((20, homing), (52, low), (48, high)):
            read_calibration.append(
                exchange(
                    port,
                    status(dxl_id, le(value, 4)),
                    lambda i=dxl_id, a=address: HANDLER.read4ByteTxRx(port, i, a),
                )
            )
        read_calibration.append(
            exchange(
                port, status(dxl_id, [drive_mode]), lambda i=dxl_id: HANDLER.read1ByteTxRx(port, i, 10)
            )
        )

    def writes(address: int, values: dict[int, int]) -> list[dict[str, str]]:
        return [
            exchange(
                port,
                status(dxl_id, []),
                lambda i=dxl_id: HANDLER.write1ByteTxRx(port, i, address, values[i]),
            )
            for dxl_id in ids
        ]

    modes = {axis["bus_id"]: OPERATING_MODES[axis["joint"]] for axis in axes}
    configure = [
        *writes(64, dict.fromkeys(ids, 0)),
        *writes(9, dict.fromkeys(ids, 0)),
        *writes(11, modes),
    ]
    torque_enable = writes(64, dict.fromkeys(ids, 1))

    bus = Bus(axes)
    goal_positions = []
    for action in ACTIONS:
        raw = bus._unnormalize(
            {axis["bus_id"]: wire(axis, target) for axis, target in zip(axes, action, strict=True)}
        )
        port.written.clear()
        writer = dxl.GroupSyncWrite(port, HANDLER, 116, 4)
        for dxl_id in ids:
            if not writer.addParam(dxl_id, le(raw[dxl_id], 4)):
                raise SystemExit(f"GroupSyncWrite refused servo {dxl_id}")
        if writer.txPacket() != dxl.COMM_SUCCESS:
            raise SystemExit("GroupSyncWrite.txPacket failed")
        (packet,) = port.written
        goal_positions.append(
            {"radians": action, "ticks": [raw[i] for i in ids], "packet": bytes(packet).hex()}
        )

    safe_stop = [
        *writes(64, dict.fromkeys(ids, 0)),
        *[
            exchange(
                port,
                status(dxl_id, [0, 0, 0, 0, 2, 0, 0]),
                lambda i=dxl_id: HANDLER.readTxRx(port, i, 64, 7),
            )
            for dxl_id in ids
        ],
    ]

    reader = dxl.GroupSyncRead(port, HANDLER, 132, 4)
    for dxl_id in ids:
        reader.addParam(dxl_id)
    positions = {1: 2048, 2: 0xFFFFFF38}  # the second, -200, in two's complement
    answers = [status(dxl_id, le(positions[dxl_id], 4)) for dxl_id in ids]
    port.written.clear()
    port.pending = [byte for answer in answers for byte in answer]
    if reader.txRxPacket() != dxl.COMM_SUCCESS:
        raise SystemExit("GroupSyncRead.txRxPacket failed")
    (request,) = port.written
    sync_read = {
        "request": bytes(request).hex(),
        "responses": [bytes(answer).hex() for answer in answers],
        "positions": [reader.getData(i, 132, 4) for i in ids],
    }

    document = {
        "calibration": [
            {"id": i, "drive_mode": d, "homing_offset": h, "range_min": lo, "range_max": hi}
            for i, d, h, lo, hi in CALIBRATION
        ],
        "identify": identify,
        "read_calibration": read_calibration,
        "configure": configure,
        "torque_enable": torque_enable,
        "goal_positions": goal_positions,
        "safe_stop": safe_stop,
        "sync_read": sync_read,
    }
    OUT.write_text(json.dumps(document, indent=1) + "\n", encoding="utf-8")
    sources = {
        **{
            f"dynamixel_sdk/{Path(name).name}": {
                "repository": "https://github.com/ROBOTIS-GIT/DynamixelSDK",
                "revision": SDK_COMMIT,
                "path": name,
                "sha256": sha256(SDK / name),
            }
            for name in SDK_FILES
        },
        **{
            name: {
                "repository": "https://github.com/huggingface/lerobot",
                "revision": LEROBOT_COMMIT,
                "path": name,
                "sha256": sha256(LEROBOT / name),
            }
            for name in LEROBOT_FILES
        },
        "bindings.json": {"sha256": sha256(BINDINGS)},
        "mint.py": {"sha256": sha256(Path(__file__))},
    }
    (HERE / "sources.json").write_text(
        json.dumps({"sources": sources, OUT.name: {"sha256": sha256(OUT)}}, indent=1) + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
