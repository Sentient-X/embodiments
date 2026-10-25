# sx-embodiment-commission

The way a robot is commissioned: find every bridge on the host and what answers on each, then
match each port a body declares to its one bridge. It only reads. Every scan speaks through its
vendor crate's read-only port, which refuses any other packet or frame before the link sees a
byte. Nothing here can write a register, enable torque or move a joint.

```bash
sx-commission probe [--json] [--feetech-ids 1-20] [--damiao-ids 1-32]
sx-commission commission --bindings <file> [--bindings <file>...] \
  [--instance <id> --pod <id> --revision <n>] [--json] [--write <dir>]
```

`probe` lists every USB device with its vendor and product id, serial, tty or `SocketCAN`
interfaces, and whether this host can open it. It then scans each Feetech-capable serial bridge
(WCH, FTDI and Silicon Labs chips, at 1 Mbaud) and each CAN interface. `commission` also
matches the body's ports. By default it prints the `sx.station-commissioning/v1` document and
each Feetech port's `LeRobot` calibration. It writes `commissioning.json` and
`<port>.calibration.json` only with `--write`. It exits 1 when any port is refused.

| Item | Path |
|---|---|
| Host seam | `host::Host` (USB devices, ttys, CAN interfaces, open serial and CAN); `host::System` is this machine: `nusb`, `serialport`, sysfs and `sx-socketcan` on Linux |
| Enumeration | `bridges::enumerate(host)` gives one `Bridge` per non-hub USB device, with a `BridgeKind` (serial bus, CAN bus, camera, input, other) and an `Access` (`Tty`, `SocketCan`, `NoDriver { reason }`, `None`). A ZLG USBCANFD-200U on macOS is `NoDriver` |
| Kernel view | `sysfs::can_interfaces(net)` (type 280 links with their USB adapter's serial) and `sysfs::by_id(dir)` (stable tty aliases) |
| Probe | `probe::probe(host, &Ids)` returns a `Probe` with each `FeetechBus` (`sx_feetech_serial::scan`) and `DamiaoBus` (`sx_damiao_can::scan`) |
| Declared ports | `declared::DeclaredBody::parse(&[documents])` reads the rendered bindings' `port`. Without one, a Damiao axis is on `can` and other axes on a port named for their bus |
| Matching | `matching::commission(&body, &probe)` gives one `PortMatch` per port, or every `PortError` (`Missing`, `Ambiguous`, `WrongModel`, `Unqualified`) naming the candidates |
| Output | `output::commissioning(&Station, &matches)`, `output::calibration(&match)` (`LeRobot` JSON keyed by joint), `output::warnings` (a USB serial carried by several CAN channels), `probe_report`/`probe_json` |

A bus fits a port when every unit the port declares answers as its model: Feetech by model
number, Damiao by the `PMAX`/`TMAX` row. A bus whose answering ids are exactly the port's wins.
A bus that also carries other ids fits only when no exact bus does, and never when it is
exactly another port's bus. That is how a gripper on ids 1 and 2 is never taken from the neck's
bus, which also answers 1 and 2.

## Known gap

The ZLG USBCANFD-200U shows both of its channels under one USB serial. Commissioning matches
the right channel (`can1`) and records the serial, but `sx-embodiment-bodies`' `SocketCanPorts`
resolves that serial to both interfaces and refuses it as ambiguous. `output::warnings` says so
until a commissioning identity can name a channel.
