//! The read-only scan against servos answering as the static upper body's neck does.
//!
//! The register images are what a read-only probe of the robot read (2026-10-25): six STS3215
//! servos, model 777, firmware 3.10, at 1 Mbaud, calibrated by `LeRobot`. The first exchange of
//! each kind is pinned byte for byte; the rest are answered by the same images.

use std::collections::{BTreeMap, VecDeque};
use std::io;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use sx_embodiment_drivers::serial::SerialPort;
use sx_feetech_serial::scan::{ReadOnlyPort, is_read_only, scan};
use sx_feetech_serial::scservo_sdk::{CommResult, ProtocolPacketHandler};
use sx_feetech_serial::{MotorCalibration, STS3215_MODEL_NUMBER};

/// The neck's calibration as the probe read it: id, `range_min`, `range_max`, `homing_offset`.
const NECK: [(u8, u16, u16, i16); 6] = [
    (1, 979, 3052, -791),
    (2, 932, 3039, 526),
    (3, 1057, 3071, -58),
    (4, 980, 3129, -315),
    (5, 0, 4095, 1605),
    (6, 1760, 2909, -42),
];

fn image(range_min: u16, range_max: u16, homing_offset: i16) -> [u8; 70] {
    let mut registers = [0_u8; 70];
    let mut put2 = |address: usize, value: u16| {
        registers[address..address + 2].copy_from_slice(&value.to_le_bytes());
    };
    put2(3, STS3215_MODEL_NUMBER);
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
    put2(56, 2048);
    registers[0] = 3;
    registers[1] = 10;
    registers[21] = 16;
    registers[22] = 32;
    registers[62] = 121;
    registers[63] = 31;
    registers
}

/// Servos on a bus answering from their register images; every packet written is logged.
struct Servos {
    images: BTreeMap<u8, [u8; 70]>,
    pending: VecDeque<u8>,
    /// Ids that answer a ping and nothing else.
    ping_only: Vec<u8>,
    log: Arc<Mutex<Vec<Vec<u8>>>>,
}

fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte))
}

fn status(id: u8, data: &[u8]) -> Vec<u8> {
    let mut packet = vec![0xFF, 0xFF, id, u8::try_from(data.len() + 2).unwrap(), 0];
    packet.extend_from_slice(data);
    packet.push(checksum(&packet[2..]));
    packet
}

impl SerialPort for Servos {
    fn clear_port(&mut self) -> io::Result<()> {
        self.pending.clear();
        Ok(())
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        self.log.lock().unwrap().push(packet.to_vec());
        let id = packet[2];
        if let Some(registers) = self.images.get(&id) {
            let answer = match packet[4] {
                2 if self.ping_only.contains(&id) => Vec::new(),
                2 => {
                    let address = usize::from(packet[5]);
                    status(id, &registers[address..address + usize::from(packet[6])])
                }
                _ => status(id, &[]),
            };
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

fn neck() -> (Servos, Arc<Mutex<Vec<Vec<u8>>>>) {
    let log = Arc::new(Mutex::new(Vec::new()));
    let images = NECK
        .iter()
        .map(|&(id, low, high, offset)| (id, image(low, high, offset)))
        .collect();
    (
        Servos {
            images,
            pending: VecDeque::new(),
            ping_only: Vec::new(),
            log: Arc::clone(&log),
        },
        log,
    )
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut text, byte| {
        use std::fmt::Write as _;
        write!(text, "{byte:02x}").unwrap();
        text
    })
}

#[test]
fn the_neck_answers_with_its_calibration() {
    let (port, _) = neck();
    let found = scan(port, 1..=8);
    assert_eq!(found.silent, vec![7, 8]);
    assert!(found.failed.is_empty());
    let ids: Vec<u8> = found.servos.iter().map(|servo| servo.id).collect();
    assert_eq!(ids, vec![1, 2, 3, 4, 5, 6]);
    for (id, range_min, range_max, homing_offset) in NECK {
        let servo = found.servo(id).unwrap();
        assert_eq!(servo.model, STS3215_MODEL_NUMBER);
        assert_eq!((servo.firmware_major, servo.firmware_minor), (3, 10));
        assert_eq!(
            servo.calibration,
            MotorCalibration {
                drive_mode: false,
                homing_offset,
                range_min,
                range_max,
            }
        );
        assert_eq!(
            (
                servo.p_coefficient,
                servo.d_coefficient,
                servo.i_coefficient
            ),
            (16, 32, 0)
        );
        assert_eq!(servo.max_torque_limit, 1000);
        assert!(!servo.torque_enabled);
        assert_eq!(servo.present_position, 2048);
        assert_eq!(
            (servo.voltage_decivolts, servo.temperature_celsius),
            (121, 31)
        );
    }
}

#[test]
fn every_packet_is_a_ping_or_a_read_byte_for_byte() {
    let (port, log) = neck();
    scan(port, 1..=7);
    let log = log.lock().unwrap();
    assert!(log.iter().all(|packet| is_read_only(packet)));
    // id 1: PING, then READ Model_Number (address 3, two bytes), then the firmware's major.
    assert_eq!(hex(&log[0]), "ffff010201fb");
    assert_eq!(hex(&log[1]), "ffff0104020302f3");
    assert_eq!(hex(&log[2]), "ffff0104020001f7");
    // Then the limits, the offset at 31, and the rest; never an instruction but 1 or 2.
    assert_eq!(hex(&log[4]), "ffff0104020902ed");
    assert_eq!(hex(&log[6]), "ffff0104021f02d7");
    // 16 packets per servo that answers, one ping for the silent id 7.
    assert_eq!(log.len(), 6 * 16 + 1);
    assert_eq!(hex(&log[96]), "ffff070201f5");
}

#[test]
fn the_read_only_port_refuses_a_write_before_the_link() {
    let (port, log) = neck();
    let mut ph = ProtocolPacketHandler::new(ReadOnlyPort::new(port), 0);
    assert_eq!(ph.write1_byte_tx_rx(1, 40, 1).0, CommResult::TxFail);
    assert_eq!(ph.write2_byte_tx_rx(1, 42, 2048).0, CommResult::TxFail);
    assert_eq!(ph.sync_write_tx_only(40, 1, &[1, 1]), CommResult::TxFail);
    assert!(log.lock().unwrap().is_empty());
}

#[test]
fn a_servo_failing_a_read_is_reported_not_dropped() {
    let (mut port, _) = neck();
    port.ping_only.push(2);
    let found = scan(port, [1, 2]);
    assert_eq!(found.servos.len(), 1);
    assert_eq!(found.failed.len(), 1);
    let failure = found.failed[0];
    assert_eq!((failure.id, failure.address), (2, 3));
    assert_eq!(failure.result, CommResult::RxTimeout);
}
