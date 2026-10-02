"""Mint the transcript sx-feetech-serial is held to, by running Feetech's own scservo_sdk.

The references, run here unmodified:

- Feetech's Python servo SDK, https://github.com/ftservo/FTServo_Python at
  cbcfa64674d592f7e5028ae72af42580f60500b4, ``scservo_sdk/`` (MIT, Copyright (c) 2024
  ftservo). Its ``sms_sts`` packet handler and ``GroupSyncWrite`` talk to a capturing port in
  the shape of its ``PortHandler``, which records every packet written and hands back the
  status packet the servo would answer.
- LeRobot, https://github.com/huggingface/lerobot at v0.6.0
  (30da8e687a6dfc617fcd94afc367ac7071c376ce): ``MotorsBus._unnormalize`` and
  ``MotorNormMode`` are executed from the pinned ``src/lerobot/motors/motors_bus.py``, since
  the SO-101's calibrated raw range is LeRobot's. The servos' normalization modes are
  ``SOFollower``'s (``src/lerobot/robots/so_follower/so_follower.py:50-59``, checked below),
  and so are the sequences: ``FeetechMotorsBus.enable_torque`` and ``disable_torque``
  (``src/lerobot/motors/feetech/feetech.py:291-305``), ``read_calibration`` for
  ``is_calibrated`` (``feetech.py:228-266``), and ``SOFollower.configure``
  (``so_follower.py:159-173``) with ``configure_motors`` (``feetech.py:209-226``), whose
  register values are checked against the pinned files below.

Wire values come from the registry's bindings (``../so101_bindings.json``, rendered by
``tools/render_driver_bindings.py``) through the drive map ``sign * (joint - zero_offset) *
reduction``: degrees for the arm, percent of the declared range for the gripper. Targets are
float32 and bounds-checked at float32, as the station's earlier Python follower did
(``sx_drivers.feetech.radians_to_wire``, the source the station's golden was first minted
from).

``sts3215_so101.json`` holds:

- ``motors`` and ``calibrations``: the chain and two per-unit calibrations (one with the
  gripper's drive inverted).
- ``goal_positions``: each target vector's encoded ticks (``scs_toscs(raw, 15)``) and the SYNC
  WRITE packet ``GroupSyncWrite.txPacket`` built for it.
- ``identify``: each servo's model-number read (``read2ByteTxRx``) answered as an STS3215.
- ``safe_stop``: per servo, ``Torque_Enable`` 0 and ``unLockEprom``, each acknowledged, then
  every servo's ``Torque_Enable`` read back as 0.
- ``torque_enable``: per servo, ``Torque_Enable`` 1 and ``LockEprom``, each acknowledged.
- ``homing_offsets``: each servo's ``Homing_Offset``, the calibration's fourth field.
- ``read_calibration``: per servo, ``Min_Position_Limit``, ``Max_Position_Limit`` and
  ``Homing_Offset`` read (``read2ByteTxRx``), answered with the ``so101`` calibration, the
  offset in sign-magnitude at bit 11 (the vendor's ``scs_toscs``).
- ``configure``: ``disable_torque``, then ``configure_motors`` per servo (``Return_Delay_Time``
  0, ``Maximum_Acceleration`` 254, ``Acceleration`` 254, ``Phase`` read, and written back with
  bit 4 cleared where the answer had it, as servo 6's does), then per servo ``Operating_Mode``
  0, ``P_Coefficient`` 16, ``I_Coefficient`` 0, ``D_Coefficient`` 32, and on the gripper
  ``Max_Torque_Limit`` 500, ``Protection_Current`` 250 and ``Overload_Torque`` 25.

Run with both sources checked out at their pinned commits:

    FTSERVO_PYTHON=<FTServo_Python> LEROBOT=<lerobot> \\
        python3 drivers/sx-feetech-serial/tests/fixtures/scservo/mint.py

It needs numpy and pyserial (which ``scservo_sdk`` imports).
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

import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "sts3215_so101.json"
BINDINGS = HERE.parent / "so101_bindings.json"

FTSERVO_COMMIT = "cbcfa64674d592f7e5028ae72af42580f60500b4"
LEROBOT_COMMIT = "30da8e687a6dfc617fcd94afc367ac7071c376ce"
VENDOR_FILES = (
    "scservo_sdk/scservo_def.py",
    "scservo_sdk/protocol_packet_handler.py",
    "scservo_sdk/group_sync_write.py",
    "scservo_sdk/sms_sts.py",
    "scservo_sdk/port_handler.py",
)
LEROBOT_FILES = (
    "src/lerobot/motors/motors_bus.py",
    "src/lerobot/motors/feetech/feetech.py",
    "src/lerobot/motors/feetech/tables.py",
    "src/lerobot/robots/so_follower/so_follower.py",
    "src/lerobot/robots/so_follower/config_so_follower.py",
)


def checkout(variable: str, commit: str) -> Path:
    root = Path(os.environ[variable])
    head = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "HEAD"], capture_output=True, text=True, check=True
    ).stdout.strip()
    if head != commit:
        raise SystemExit(f"{variable} is at {head}, not {commit}")
    return root


FTSERVO = checkout("FTSERVO_PYTHON", FTSERVO_COMMIT)
LEROBOT = checkout("LEROBOT", LEROBOT_COMMIT)
sys.path.insert(0, str(FTSERVO))
import scservo_sdk as scs  # noqa: E402  (Feetech's own package, from the pinned checkout)

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

# SOFollower's motors with use_degrees=True, its default; checked against the pinned file.
NORM_MODES = {
    "shoulder_pan": MotorNormMode.DEGREES,
    "shoulder_lift": MotorNormMode.DEGREES,
    "elbow_flex": MotorNormMode.DEGREES,
    "wrist_flex": MotorNormMode.DEGREES,
    "wrist_roll": MotorNormMode.DEGREES,
    "gripper": MotorNormMode.RANGE_0_100,
}
SO_FOLLOWER = (LEROBOT / LEROBOT_FILES[3]).read_text(encoding="utf-8")
for index, (name, mode) in enumerate(NORM_MODES.items(), start=1):
    spelled = "MotorNormMode.RANGE_0_100" if mode is MotorNormMode.RANGE_0_100 else "norm_mode_body"
    if f'"{name}": Motor({index}, "sts3215", {spelled}),' not in SO_FOLLOWER:
        raise SystemExit(f"so_follower.py no longer declares {name} as motor {index}")
if "use_degrees: bool = True" not in (LEROBOT / LEROBOT_FILES[4]).read_text(encoding="utf-8"):
    raise SystemExit("SOFollowerConfig.use_degrees no longer defaults to True")

FEETECH = (LEROBOT / LEROBOT_FILES[1]).read_text(encoding="utf-8")
TABLES = (LEROBOT / LEROBOT_FILES[2]).read_text(encoding="utf-8")
# Register addresses from STS_SMS_SERIES_CONTROL_TABLE, and the values configure writes.
REGISTERS = {
    "Return_Delay_Time": 7,
    "Min_Position_Limit": 9,
    "Max_Position_Limit": 11,
    "Max_Torque_Limit": 16,
    "Phase": 18,
    "P_Coefficient": 21,
    "D_Coefficient": 22,
    "I_Coefficient": 23,
    "Protection_Current": 28,
    "Homing_Offset": 31,
    "Operating_Mode": 33,
    "Overload_Torque": 36,
    "Acceleration": 41,
    "Maximum_Acceleration": 85,
}
for name, address in REGISTERS.items():
    if not any(f'"{name}": ({address}, ' in line for line in TABLES.splitlines()[40:100]):
        raise SystemExit(f"tables.py no longer puts {name} at {address}")
for spelled in (
    'self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)',
    'self.bus.write("P_Coefficient", motor, 16)',
    'self.bus.write("I_Coefficient", motor, 0)',
    'self.bus.write("D_Coefficient", motor, 32)',
    'self.bus.write("Max_Torque_Limit", motor, 500)',
    'self.bus.write("Protection_Current", motor, 250)',
    'self.bus.write("Overload_Torque", motor, 25)',
):
    if spelled not in SO_FOLLOWER:
        raise SystemExit(f"SOFollower.configure no longer runs {spelled}")
if "def configure_motors(self, return_delay_time=0, maximum_acceleration=254, acceleration=254)" not in FEETECH:
    raise SystemExit("configure_motors' defaults changed")
HOMING_OFFSET_SIGN_BIT = 11  # tables.py STS_SMS_SERIES_ENCODINGS_TABLE["Homing_Offset"]
HOMING_OFFSETS = [-31, 1240, -2047, 0, 512, -873]
# What each servo's Phase register answers; servo 6's has bit 4 set, so configure clears it.
PHASES = [0x00, 0x00, 0x00, 0x00, 0x00, 0x1C]

STS3215_RESOLUTION = 4096  # tables.py MODEL_RESOLUTION["sts3215"]
STS3215_MODEL = 777  # tables.py MODEL_NUMBER_TABLE["sts3215"]
SIGN_BIT = 15  # sms_sts.py scs_toscs(position, 15)


class Calibration:
    def __init__(self, drive_mode: int, range_min: int, range_max: int) -> None:
        self.drive_mode = drive_mode
        self.range_min = range_min
        self.range_max = range_max


class Bus:
    """The attributes ``_unnormalize`` reads from a ``FeetechMotorsBus``."""

    apply_drive_mode = True  # FeetechMotorsBus.apply_drive_mode
    model_resolution_table = {"sts3215": STS3215_RESOLUTION}

    def __init__(self, axes: list[dict[str, Any]], rows: list[tuple[int, int, int, int]]) -> None:
        self.names = {axis["bus_id"]: axis["joint"] for axis in axes}
        self.motors = {
            axis["joint"]: type("Motor", (), {"norm_mode": NORM_MODES[axis["joint"]]})
            for axis in axes
        }
        self.calibration = {
            self.names[motor_id]: Calibration(drive_mode, lo, hi)
            for motor_id, drive_mode, lo, hi in rows
        }

    def _id_to_name(self, motor_id: int) -> str:
        return self.names[motor_id]

    def _id_to_model(self, motor_id: int) -> str:
        return "sts3215"

    _unnormalize = _unnormalize


# --- the recording port ----------------------------------------------------------------------


class CapturePort:
    """A ``PortHandler`` that records every written packet and replays status packets."""

    def __init__(self) -> None:
        self.is_using = False
        self.written: list[list[int]] = []
        self.pending: list[int] = []

    def clearPort(self) -> None:  # noqa: N802 - scservo_sdk's port protocol
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


def status(scs_id: int, params: list[int]) -> list[int]:
    """One servo status packet: header, id, length, error 0, params, checksum."""
    body = [scs_id, len(params) + 2, 0, *params]
    return [0xFF, 0xFF, *body, ~sum(body) & 0xFF]


def exchange(port: CapturePort, response: list[int], call: Any) -> dict[str, str]:
    port.pending = list(response)
    before = len(port.written)
    outcome = call()
    result = outcome[1] if len(outcome) == 3 else outcome[0]
    if result != scs.COMM_SUCCESS:
        raise SystemExit(f"scservo_sdk answered {result} to {port.written[before:]}")
    (request,) = port.written[before:]
    return {"request": bytes(request).hex(), "response": bytes(response).hex()}


CALIBRATIONS = {
    "so101": [
        (1, 0, 750, 3350),
        (2, 0, 820, 3280),
        (3, 0, 900, 3150),
        (4, 0, 860, 3230),
        (5, 0, 40, 4050),
        (6, 0, 2030, 3470),
    ],
    "so101-inverted-gripper": [
        (1, 0, 750, 3350),
        (2, 0, 820, 3280),
        (3, 0, 900, 3150),
        (4, 0, 860, 3230),
        (5, 0, 40, 4050),
        (6, 1, 2030, 3470),
    ],
}

ACTIONS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    [0.5, -0.75, 1.25, -1.5, 2.5, 1.0],
    [-1.91986, 1.74533, -1.69, 1.65806, -2.74385, -0.174533],
    [1.9, -1.7, 1.6, -1.6, 2.8, 2.0944],
]


def actuator(axis: dict[str, Any], joint: float) -> float:
    """The registry's drive map: ``sign * (joint - zero_offset) * reduction``."""
    return axis["sign"] * (joint - axis["zero_offset"]) * axis["reduction"]


def wire(axis: dict[str, Any], target: np.float32) -> float:
    """LeRobot's wire value for one float32 target: degrees, or percent of the declared range."""
    if not np.float32(axis["lower"]) <= target <= np.float32(axis["upper"]):
        raise SystemExit(f"{axis['joint']} target {target} is outside its bounds")
    value = actuator(axis, float(target))
    if NORM_MODES[axis["joint"]] is MotorNormMode.RANGE_0_100:
        ends = (actuator(axis, axis["lower"]), actuator(axis, axis["upper"]))
        low, high = min(ends), max(ends)
        return (value - low) * 100.0 / (high - low)
    return math.degrees(value)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    bindings = json.loads(BINDINGS.read_text(encoding="utf-8"))
    axes: list[dict[str, Any]] = bindings["axes"]
    port = CapturePort()
    handler = scs.sms_sts(port)
    motors = [
        {
            "name": axis["joint"],
            "id": axis["bus_id"],
            "gripper": NORM_MODES[axis["joint"]] is MotorNormMode.RANGE_0_100,
            "lower": axis["lower"],
            "upper": axis["upper"],
        }
        for axis in axes
    ]
    goal_positions = []
    for calibration_name, calibration in CALIBRATIONS.items():
        bus = Bus(axes, calibration)
        for action in ACTIONS:
            targets = np.asarray(action, dtype=np.float32)
            raw = bus._unnormalize(
                {axis["bus_id"]: wire(axis, target) for axis, target in zip(axes, targets, strict=True)}
            )
            ticks = [handler.scs_toscs(raw[axis["bus_id"]], SIGN_BIT) for axis in axes]
            if any(abs(raw[axis["bus_id"]]) >= 1 << SIGN_BIT for axis in axes):
                raise SystemExit(f"{action} does not fit the sign-magnitude register")
            port.written.clear()
            writer = scs.GroupSyncWrite(handler, scs.SMS_STS_GOAL_POSITION_L, 2)
            for axis, value in zip(axes, ticks, strict=True):
                if not writer.addParam(
                    axis["bus_id"], [handler.scs_lobyte(value), handler.scs_hibyte(value)]
                ):
                    raise SystemExit(f"GroupSyncWrite refused servo {axis['bus_id']}")
            if writer.txPacket() != scs.COMM_SUCCESS:
                raise SystemExit("GroupSyncWrite.txPacket failed")
            (packet,) = port.written
            goal_positions.append(
                {
                    "calibration": calibration_name,
                    "radians": [float(value) for value in targets],
                    "ticks": ticks,
                    "packet": bytes(packet).hex(),
                }
            )

    ids = [axis["bus_id"] for axis in axes]
    safe_stop = []
    # FeetechMotorsBus.disable_torque: per servo, Torque_Enable 0 then Lock 0, each acknowledged.
    for scs_id in ids:
        safe_stop.append(
            exchange(
                port,
                status(scs_id, []),
                lambda i=scs_id: handler.write1ByteTxRx(i, scs.SMS_STS_TORQUE_ENABLE, 0),
            )
        )
        safe_stop.append(
            exchange(port, status(scs_id, []), lambda i=scs_id: handler.unLockEprom(i))
        )
    # The chain's read-back: every servo reports Torque_Enable 0.
    for scs_id in ids:
        safe_stop.append(
            exchange(
                port,
                status(scs_id, [0]),
                lambda i=scs_id: handler.read1ByteTxRx(i, scs.SMS_STS_TORQUE_ENABLE),
            )
        )
    identify = [
        exchange(
            port,
            status(scs_id, [handler.scs_lobyte(STS3215_MODEL), handler.scs_hibyte(STS3215_MODEL)]),
            lambda i=scs_id: handler.read2ByteTxRx(i, scs.SMS_STS_MODEL_L),
        )
        for scs_id in ids
    ]
    # FeetechMotorsBus.enable_torque: per servo, Torque_Enable 1 then Lock 1, each acknowledged.
    torque_enable = []
    for scs_id in ids:
        torque_enable.append(
            exchange(
                port,
                status(scs_id, []),
                lambda i=scs_id: handler.write1ByteTxRx(i, scs.SMS_STS_TORQUE_ENABLE, 1),
            )
        )
        torque_enable.append(
            exchange(port, status(scs_id, []), lambda i=scs_id: handler.LockEprom(i))
        )
    so101 = {row[0]: row for row in CALIBRATIONS["so101"]}
    read_calibration = []
    for scs_id, offset in zip(ids, HOMING_OFFSETS, strict=True):
        _, _, range_min, range_max = so101[scs_id]
        encoded = handler.scs_toscs(offset, HOMING_OFFSET_SIGN_BIT)
        if handler.scs_tohost(encoded, HOMING_OFFSET_SIGN_BIT) != offset:
            raise SystemExit(f"homing offset {offset} does not round-trip")
        for address, value in (
            (REGISTERS["Min_Position_Limit"], range_min),
            (REGISTERS["Max_Position_Limit"], range_max),
            (REGISTERS["Homing_Offset"], encoded),
        ):
            read_calibration.append(
                exchange(
                    port,
                    status(scs_id, [handler.scs_lobyte(value), handler.scs_hibyte(value)]),
                    lambda i=scs_id, a=address: handler.read2ByteTxRx(i, a),
                )
            )

    def write(scs_id: int, name: str, value: int, width: int = 1) -> dict[str, str]:
        address = REGISTERS[name] if name in REGISTERS else getattr(scs, name)
        call = handler.write2ByteTxRx if width == 2 else handler.write1ByteTxRx
        return exchange(port, status(scs_id, []), lambda: call(scs_id, address, value))

    # SOFollower.configure inside torque_disabled: disable_torque first.
    configure = []
    for scs_id in ids:
        configure.append(write(scs_id, "SMS_STS_TORQUE_ENABLE", 0))
        configure.append(exchange(port, status(scs_id, []), lambda i=scs_id: handler.unLockEprom(i)))
    # FeetechMotorsBus.configure_motors, protocol 0, on an STS3215.
    for scs_id, phase in zip(ids, PHASES, strict=True):
        configure.append(write(scs_id, "Return_Delay_Time", 0))
        configure.append(write(scs_id, "Maximum_Acceleration", 254))
        configure.append(write(scs_id, "Acceleration", 254))
        configure.append(
            exchange(
                port,
                status(scs_id, [phase]),
                lambda i=scs_id: handler.read1ByteTxRx(i, REGISTERS["Phase"]),
            )
        )
        if phase & 0x10:
            configure.append(write(scs_id, "Phase", phase & ~0x10))
    for axis in axes:
        scs_id = axis["bus_id"]
        configure.append(write(scs_id, "Operating_Mode", 0))  # OperatingMode.POSITION
        configure.append(write(scs_id, "P_Coefficient", 16))
        configure.append(write(scs_id, "I_Coefficient", 0))
        configure.append(write(scs_id, "D_Coefficient", 32))
        if axis["joint"] == "gripper":
            configure.append(write(scs_id, "Max_Torque_Limit", 500, width=2))
            configure.append(write(scs_id, "Protection_Current", 250, width=2))
            configure.append(write(scs_id, "Overload_Torque", 25))
    document = {
        "motors": motors,
        "calibrations": {
            name: [
                {"id": i, "drive_mode": d, "range_min": lo, "range_max": hi}
                for i, d, lo, hi in rows
            ]
            for name, rows in CALIBRATIONS.items()
        },
        "goal_positions": goal_positions,
        "identify": identify,
        "safe_stop": safe_stop,
        "torque_enable": torque_enable,
        "homing_offsets": [
            {"id": scs_id, "homing_offset": offset}
            for scs_id, offset in zip(ids, HOMING_OFFSETS, strict=True)
        ],
        "read_calibration": read_calibration,
        "configure": configure,
    }
    OUT.write_text(json.dumps(document, indent=1) + "\n", encoding="utf-8")
    sources: dict[str, dict[str, str]] = {}
    for path in VENDOR_FILES:
        sources[path] = {
            "repository": "https://github.com/ftservo/FTServo_Python",
            "revision": FTSERVO_COMMIT,
            "path": path,
            "sha256": sha256(FTSERVO / path),
        }
    for path in LEROBOT_FILES:
        sources[path] = {
            "repository": "https://github.com/huggingface/lerobot",
            "revision": LEROBOT_COMMIT,
            "path": path,
            "sha256": sha256(LEROBOT / path),
        }
    sources["so101_bindings.json"] = {"sha256": sha256(BINDINGS)}
    sources["mint.py"] = {"sha256": sha256(Path(__file__))}
    provenance = {"sources": sources, "sts3215_so101.json": {"sha256": sha256(OUT)}}
    (HERE / "provenance.json").write_text(json.dumps(provenance, indent=1) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
