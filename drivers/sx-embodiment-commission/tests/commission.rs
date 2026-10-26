//! Commissioning the static upper body on a host replaying the robot as a read-only probe
//! found it on 2026-10-25: three WCH CH343 bridges (the neck's six STS3215s, and two grippers
//! of two each), a ZLG USBCANFD-200U whose second channel carries the fifteen Damiao motors,
//! a ZED 2i and a foot switch.

use std::collections::{BTreeMap, VecDeque};
use std::io;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use sx_damiao_can::dm_can::DmMotorType;
use sx_embodiment_bodies::Commissioning;
use sx_embodiment_commission::bridges::{Access, BridgeKind, enumerate};
use sx_embodiment_commission::declared::{BusKind, DeclaredBody, DeclaredError};
use sx_embodiment_commission::host::{CanInterface, Host, Tty, UsbDevice};
use sx_embodiment_commission::matching::{PortError, commission};
use sx_embodiment_commission::output::{Station, calibration, commissioning, warnings};
use sx_embodiment_commission::probe::{Ids, probe};
use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_embodiment_drivers::serial::SerialPort;

/// The registry's own rendering of the static upper body, with its ports.
const BINDINGS: &str =
    include_str!("../../sx-feetech-serial/bindings/sentient-static-upper-body.json");

const NECK_SERIAL: &str = "5A7C118709";
const LEFT_SERIAL: &str = "5AB0158173";
const RIGHT_SERIAL: &str = "5B91028463";
const ZLG_SERIAL: &str = "A30F115DD0120A949B80";

/// Each bus's servos as the probe read them: id, `range_min`, `range_max`, `homing_offset`.
const NECK: [(u8, u16, u16, i16); 6] = [
    (1, 979, 3052, -791),
    (2, 932, 3039, 526),
    (3, 1057, 3071, -58),
    (4, 980, 3129, -315),
    (5, 0, 4095, 1605),
    (6, 1760, 2909, -42),
];
const LEFT: [(u8, u16, u16, i16); 2] = [(3, 1712, 2322, -466), (4, 1756, 2342, 482)];
const RIGHT: [(u8, u16, u16, i16); 2] = [(1, 1720, 2302, 480), (2, 1813, 2358, -273)];

const DAMIAO: [(u16, DmMotorType); 15] = [
    (0x1F, DmMotorType::DM8009),
    (0x18, DmMotorType::DM8009),
    (0x19, DmMotorType::DM8009),
    (0x1A, DmMotorType::DM4340),
    (0x1B, DmMotorType::DM4340),
    (0x1C, DmMotorType::DM4340),
    (0x1D, DmMotorType::DM4310),
    (0x1E, DmMotorType::DM4310),
    (0x09, DmMotorType::DM8009),
    (0x10, DmMotorType::DM8009),
    (0x11, DmMotorType::DM4340),
    (0x12, DmMotorType::DM4340),
    (0x13, DmMotorType::DM4340),
    (0x14, DmMotorType::DM4310),
    (0x15, DmMotorType::DM4310),
];

type Registers = [u8; 70];

fn image(model: u16, range_min: u16, range_max: u16, homing_offset: i16) -> Registers {
    let mut registers = [0_u8; 70];
    let mut put2 = |address: usize, value: u16| {
        registers[address..address + 2].copy_from_slice(&value.to_le_bytes());
    };
    put2(3, model);
    put2(9, range_min);
    put2(11, range_max);
    let magnitude = homing_offset.unsigned_abs();
    put2(
        31,
        if homing_offset < 0 {
            magnitude | 0x800
        } else {
            magnitude
        },
    );
    put2(16, 1000);
    registers[0] = 3;
    registers[1] = 10;
    registers
}

fn servos(table: &[(u8, u16, u16, i16)]) -> BTreeMap<u8, Registers> {
    table
        .iter()
        .map(|&(id, low, high, offset)| (id, image(777, low, high, offset)))
        .collect()
}

fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte))
}

/// One serial bus answering from its servos' register images.
struct FakeSerial {
    servos: BTreeMap<u8, Registers>,
    pending: VecDeque<u8>,
    log: Arc<Mutex<Vec<Vec<u8>>>>,
}

impl SerialPort for FakeSerial {
    fn clear_port(&mut self) -> io::Result<()> {
        self.pending.clear();
        Ok(())
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        self.log.lock().unwrap().push(packet.to_vec());
        let id = packet[2];
        if let Some(registers) = self.servos.get(&id) {
            let data: &[u8] = if packet[4] == 2 {
                let address = usize::from(packet[5]);
                &registers[address..address + usize::from(packet[6])]
            } else {
                &[]
            };
            let mut answer = vec![0xFF, 0xFF, id, u8::try_from(data.len() + 2).unwrap(), 0];
            answer.extend_from_slice(data);
            answer.push(checksum(&answer[2..]));
            self.pending.extend(answer);
        }
        Ok(packet.len())
    }

    fn read_port(&mut self, buffer: &mut [u8]) -> io::Result<usize> {
        let take = buffer.len().min(self.pending.len());
        for (slot, byte) in buffer.iter_mut().zip(self.pending.drain(..take)) {
            *slot = byte;
        }
        Ok(take)
    }

    fn timeout(&self) -> Duration {
        Duration::from_millis(20)
    }

    fn set_packet_timeout(&mut self, _packet_length: usize) {}

    fn is_packet_timeout(&mut self) -> bool {
        self.pending.is_empty()
    }
}

/// One CAN bus whose motors answer register reads on their id plus `0x20`.
struct FakeCan {
    motors: BTreeMap<u16, DmMotorType>,
    inbox: VecDeque<CanFrame>,
    log: Arc<Mutex<Vec<CanFrame>>>,
}

impl CanPort for FakeCan {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.log.lock().unwrap().push(*frame);
        let data = frame.data;
        let id = u16::from_le_bytes([data[0], data[1]]);
        if let Some(&model) = self.motors.get(&id) {
            let [p, v, t] = model.limit_param();
            #[allow(clippy::cast_possible_truncation)]
            let value = match data[3] {
                7 => u32::from(id + 0x20).to_le_bytes(),
                21 => (p as f32).to_le_bytes(),
                22 => (v as f32).to_le_bytes(),
                23 => (t as f32).to_le_bytes(),
                _ => [0; 4],
            };
            self.inbox.push_back(CanFrame {
                id: u32::from(id + 0x20),
                data: [
                    data[0], data[1], data[2], data[3], value[0], value[1], value[2], value[3],
                ],
            });
        }
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        Ok(self.inbox.pop_front())
    }

    fn sleep(&mut self, _duration: Duration) {}
}

/// A host carrying the robot; every packet and frame any scan sends is logged.
#[derive(Default)]
struct Robot {
    usb: Vec<UsbDevice>,
    ttys: Vec<Tty>,
    can: Vec<CanInterface>,
    serial: BTreeMap<String, BTreeMap<u8, Registers>>,
    buses: BTreeMap<String, BTreeMap<u16, DmMotorType>>,
    serial_log: Arc<Mutex<Vec<Vec<u8>>>>,
    can_log: Arc<Mutex<Vec<CanFrame>>>,
}

impl Robot {
    fn usb(&mut self, vendor_id: u16, product_id: u16, serial: Option<&str>, classes: &[u8]) {
        self.usb.push(UsbDevice {
            vendor_id,
            product_id,
            serial: serial.map(str::to_owned),
            interface_classes: classes.to_vec(),
            ..UsbDevice::default()
        });
    }

    fn servo_bus(&mut self, serial: &str, servos: BTreeMap<u8, Registers>) {
        self.usb(0x1a86, 0x55d3, Some(serial), &[0x02, 0x0A]);
        let path = format!("/dev/cu.usbmodem{serial}1");
        self.ttys.push(Tty {
            path: path.clone(),
            vendor_id: 0x1a86,
            product_id: 0x55d3,
            serial: Some(serial.to_owned()),
        });
        self.serial.insert(path, servos);
    }

    /// The robot as found, with the ZLG's channels up under its Linux driver when `linux`.
    fn found(linux: bool) -> Self {
        let mut robot = Self::default();
        robot.servo_bus(NECK_SERIAL, servos(&NECK));
        robot.servo_bus(LEFT_SERIAL, servos(&LEFT));
        robot.servo_bus(RIGHT_SERIAL, servos(&RIGHT));
        robot.usb(0x3068, 0x0009, Some(ZLG_SERIAL), &[0xFF]);
        robot.usb(0x2b03, 0xf880, Some("OV0001"), &[0x0E, 0x0E]);
        robot.usb(0x2b03, 0xf881, Some("30371653"), &[0x03]);
        robot.usb(0x3553, 0xb001, None, &[0x03, 0x03]);
        robot.usb.push(UsbDevice {
            vendor_id: 0x05e3,
            product_id: 0x0610,
            device_class: 0x09,
            ..UsbDevice::default()
        });
        if linux {
            for name in ["can0", "can1"] {
                robot.can.push(CanInterface {
                    name: name.to_owned(),
                    usb_serial: Some(ZLG_SERIAL.to_owned()),
                });
            }
            robot.buses.insert("can0".to_owned(), BTreeMap::new());
            robot
                .buses
                .insert("can1".to_owned(), DAMIAO.into_iter().collect());
        }
        robot
    }
}

impl Host for Robot {
    type Serial = FakeSerial;
    type Can = FakeCan;

    fn usb_devices(&self) -> io::Result<Vec<UsbDevice>> {
        Ok(self.usb.clone())
    }

    fn ttys(&self) -> io::Result<Vec<Tty>> {
        Ok(self.ttys.clone())
    }

    fn can_interfaces(&self) -> io::Result<Vec<CanInterface>> {
        Ok(self.can.clone())
    }

    fn open_serial(&mut self, tty: &str, baud: u32) -> io::Result<FakeSerial> {
        assert_eq!(baud, 1_000_000);
        Ok(FakeSerial {
            servos: self.serial.get(tty).cloned().unwrap_or_default(),
            pending: VecDeque::new(),
            log: Arc::clone(&self.serial_log),
        })
    }

    fn open_can(&mut self, name: &str) -> io::Result<FakeCan> {
        Ok(FakeCan {
            motors: self.buses.get(name).cloned().unwrap_or_default(),
            inbox: VecDeque::new(),
            log: Arc::clone(&self.can_log),
        })
    }
}

fn body() -> DeclaredBody {
    DeclaredBody::parse(&[BINDINGS]).unwrap()
}

fn station() -> Station {
    Station {
        instance: "instance-content-id".to_owned(),
        pod_id: "pod-1".to_owned(),
        pod_revision: 3,
    }
}

#[test]
fn every_port_finds_its_one_bridge() {
    let mut robot = Robot::found(true);
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let matched = commission(&body(), &found).unwrap();
    let ports: Vec<(&str, &str, &str)> = matched
        .iter()
        .map(|port| {
            (
                port.port.as_str(),
                port.identity.as_str(),
                port.channel.as_str(),
            )
        })
        .collect();
    assert_eq!(
        ports,
        vec![
            ("can", ZLG_SERIAL, "can1"),
            ("left_gripper", LEFT_SERIAL, "/dev/cu.usbmodem5AB01581731"),
            ("right_gripper", RIGHT_SERIAL, "/dev/cu.usbmodem5B910284631"),
            ("neck", NECK_SERIAL, "/dev/cu.usbmodem5A7C1187091"),
        ]
    );

    // The document is the one the bodies crate opens a station from.
    let document = commissioning(&station(), &matched).to_string();
    let parsed = Commissioning::parse(&document, "instance-content-id").unwrap();
    assert_eq!(parsed.ports["neck"], NECK_SERIAL);
    assert_eq!(parsed.ports["can"], ZLG_SERIAL);
    assert_eq!(parsed.pod_revision, 3);

    // The ZLG's two channels share its serial: the station cannot yet open by it.
    let notes = warnings(&found, &matched);
    assert_eq!(notes.len(), 1);
    assert!(notes[0].contains("can0, can1"), "{}", notes[0]);
}

#[test]
fn a_feetech_port_carries_lerobots_calibration() {
    let mut robot = Robot::found(true);
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let matched = commission(&body(), &found).unwrap();
    let neck = calibration(&matched[3]).unwrap();
    assert_eq!(
        neck["so101_shoulder_pan_joint"],
        serde_json::json!({"id": 1, "drive_mode": 0, "homing_offset": -791, "range_min": 979, "range_max": 3052})
    );
    assert_eq!(neck["so101_camera_tilt_joint"]["range_min"], 1760);
    assert_eq!(neck.as_object().unwrap().len(), 6);
    let right = calibration(&matched[2]).unwrap();
    assert_eq!(
        right["right_gripper_left_finger_joint"]["homing_offset"],
        480
    );
    assert_eq!(right["right_gripper_right_finger_joint"]["id"], 2);
    assert!(calibration(&matched[0]).is_none());
}

#[test]
fn nothing_but_pings_and_reads_reach_any_bus() {
    let mut robot = Robot::found(true);
    let serial_log = Arc::clone(&robot.serial_log);
    let can_log = Arc::clone(&robot.can_log);
    let found = probe(&mut robot, &Ids::default()).unwrap();
    commission(&body(), &found).unwrap();
    let serial_log = serial_log.lock().unwrap();
    let can_log = can_log.lock().unwrap();
    assert!(!serial_log.is_empty() && !can_log.is_empty());
    assert!(
        serial_log
            .iter()
            .all(|packet| sx_feetech_serial::scan::is_read_only(packet))
    );
    assert!(can_log.iter().all(sx_damiao_can::scan::is_read_only));
}

#[test]
fn on_macos_the_zlg_is_present_and_the_can_port_is_missing() {
    let mut robot = Robot::found(false);
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let zlg = found
        .bridges
        .iter()
        .find(|bridge| bridge.usb.vendor_id == 0x3068)
        .unwrap();
    assert_eq!(zlg.kind, BridgeKind::CanBus);
    assert!(matches!(zlg.access, Access::NoDriver { .. }));
    let errors = commission(&body(), &found).unwrap_err();
    assert_eq!(errors.len(), 1);
    assert!(
        matches!(&errors[0], PortError::Missing { port, partial, .. }
        if port == "can" && partial.is_empty())
    );
}

#[test]
fn two_buses_answering_alike_are_refused_naming_both() {
    let mut robot = Robot::found(true);
    robot.servo_bus("5A7C000000", servos(&NECK));
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let errors = commission(&body(), &found).unwrap_err();
    assert_eq!(errors.len(), 1);
    let PortError::Ambiguous { port, candidates } = &errors[0] else {
        panic!("{:?}", errors[0]);
    };
    assert_eq!(port, "neck");
    let identities: Vec<&str> = candidates.iter().map(|c| c.identity.as_str()).collect();
    assert_eq!(identities, vec!["5A7C000000", NECK_SERIAL]);
}

#[test]
fn a_missing_gripper_is_never_taken_from_the_necks_bus() {
    let mut robot = Robot::found(true);
    robot
        .serial
        .insert("/dev/cu.usbmodem5B910284631".to_owned(), BTreeMap::new());
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let errors = commission(&body(), &found).unwrap_err();
    assert_eq!(errors.len(), 1);
    let PortError::Missing { port, ids, partial } = &errors[0] else {
        panic!("{:?}", errors[0]);
    };
    assert_eq!(
        (port.as_str(), ids.as_slice()),
        ("right_gripper", [1, 2].as_slice())
    );
    // The neck's bus also answers 1 and 2; it is named, not taken.
    assert_eq!(partial.len(), 1);
    assert_eq!(partial[0].identity, NECK_SERIAL);
}

#[test]
fn a_servo_of_another_model_is_refused_as_that() {
    let mut robot = Robot::found(true);
    let neck = robot.serial.get_mut("/dev/cu.usbmodem5A7C1187091").unwrap();
    neck.insert(3, image(1030, 1057, 3071, -58));
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let errors = commission(&body(), &found).unwrap_err();
    assert!(
        matches!(&errors[..], [PortError::WrongModel { port, bus_id: 3, found, .. }]
        if port == "neck" && found == "model number 1030")
    );
}

#[test]
fn a_damiao_motor_of_another_model_is_refused_as_that() {
    let mut robot = Robot::found(true);
    robot
        .buses
        .get_mut("can1")
        .unwrap()
        .insert(0x1F, DmMotorType::DM4310);
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let errors = commission(&body(), &found).unwrap_err();
    assert!(
        matches!(&errors[..], [PortError::WrongModel { port, bus_id: 0x1F, .. }]
        if port == "can")
    );
}

#[test]
fn extra_devices_and_extra_ids_are_ignored() {
    let mut robot = Robot::found(true);
    // An unrelated servo bus, an empty adapter, and a stray id on the left gripper's bus.
    robot.servo_bus("ABCDEF0001", servos(&[(10, 0, 4095, 0), (11, 0, 4095, 0)]));
    robot.servo_bus("ABCDEF0002", BTreeMap::new());
    robot
        .serial
        .get_mut("/dev/cu.usbmodem5AB01581731")
        .unwrap()
        .insert(12, image(777, 0, 4095, 0));
    let found = probe(&mut robot, &Ids::default()).unwrap();
    let matched = commission(&body(), &found).unwrap();
    assert_eq!(matched[1].identity, LEFT_SERIAL);
    assert_eq!(matched[1].servos.len(), 2);
}

#[test]
fn enumeration_classes_every_device() {
    let robot = Robot::found(true);
    let bridges = enumerate(&robot).unwrap();
    // The hub is not listed.
    assert_eq!(bridges.len(), 7);
    let kind = |vendor: u16, product: u16| {
        bridges
            .iter()
            .find(|b| (b.usb.vendor_id, b.usb.product_id) == (vendor, product))
            .unwrap()
    };
    assert_eq!(kind(0x2b03, 0xf880).kind, BridgeKind::Camera);
    assert_eq!(kind(0x3553, 0xb001).kind, BridgeKind::Input);
    assert_eq!(
        kind(0x3068, 0x0009).access,
        Access::SocketCan {
            interfaces: vec!["can0".to_owned(), "can1".to_owned()]
        }
    );
    let neck = bridges
        .iter()
        .find(|b| b.identity() == Some(NECK_SERIAL))
        .unwrap();
    assert!(neck.carries_servo_bus());
    // An unknown camera is found by its video interface.
    let mut robot = Robot::default();
    robot.usb(0x1234, 0x5678, Some("cam"), &[0x0E]);
    assert_eq!(enumerate(&robot).unwrap()[0].kind, BridgeKind::Camera);
}

#[test]
fn bindings_without_ports_put_damiao_on_can() {
    let document = r#"{"embodiment": "b", "id": "x", "axes": [
        {"joint": "a", "model": "damiao_dm4310", "bus": "damiao_can", "bus_id": 1},
        {"joint": "b", "model": "feetech_sts3215", "bus": "feetech_serial", "bus_id": 1}
    ]}"#;
    let body = DeclaredBody::parse(&[document]).unwrap();
    let ports: Vec<(&str, BusKind)> = body
        .ports
        .iter()
        .map(|port| (port.name.as_str(), port.bus))
        .collect();
    assert_eq!(
        ports,
        vec![
            ("can", BusKind::Damiao),
            ("feetech_serial", BusKind::Feetech)
        ]
    );
    let mixed = r#"{"embodiment": "b", "id": "x", "axes": [
        {"joint": "a", "model": "damiao_dm4310", "bus": "damiao_can", "bus_id": 1, "port": "p"},
        {"joint": "b", "model": "feetech_sts3215", "bus": "feetech_serial", "bus_id": 2, "port": "p"}
    ]}"#;
    assert!(matches!(
        DeclaredBody::parse(&[mixed]),
        Err(DeclaredError::MixedPort { .. })
    ));
}
