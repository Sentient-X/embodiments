//! The crate replays what Feetech's own `scservo_sdk` did, byte for byte.
//!
//! `fixtures/scservo/sts3215_so101.json` is minted by `fixtures/scservo/mint.py`, which runs
//! the vendor SDK itself; see its header for the sources.

use std::collections::VecDeque;
use std::io;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use serde::Deserialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use sx_embodiment_drivers::serial::SerialPort;
use sx_embodiment_drivers::{ActuatorChain, BindingError, Evidence, StopReport, TargetError};
use sx_feetech_serial::scservo_sdk::CommResult;
use sx_feetech_serial::so101::so101;
use sx_feetech_serial::{FeetechChain, FeetechChainError, FeetechServo, MotorCalibration};

const GOLDEN: &str = include_str!("fixtures/scservo/sts3215_so101.json");

#[derive(Deserialize)]
struct Golden {
    motors: Vec<Motor>,
    calibrations: std::collections::BTreeMap<String, Vec<Calibration>>,
    goal_positions: Vec<GoalPosition>,
    identify: Vec<Exchange>,
    safe_stop: Vec<Exchange>,
    torque_enable: Vec<Exchange>,
}

#[derive(Deserialize)]
struct Motor {
    id: u16,
}

#[derive(Deserialize)]
struct Calibration {
    id: u16,
    drive_mode: u8,
    range_min: u16,
    range_max: u16,
}

#[derive(Deserialize)]
struct GoalPosition {
    calibration: String,
    radians: Vec<f64>,
    ticks: Vec<u16>,
    packet: String,
}

#[derive(Clone, Deserialize)]
struct Exchange {
    request: String,
    response: String,
}

fn golden() -> Golden {
    serde_json::from_str(GOLDEN).expect("golden transcript")
}

fn bytes(hex: &str) -> Vec<u8> {
    (0..hex.len())
        .step_by(2)
        .map(|index| u8::from_str_radix(&hex[index..index + 2], 16).expect("hex"))
        .collect()
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().fold(String::new(), |mut text, byte| {
        use std::fmt::Write as _;
        write!(text, "{byte:02x}").expect("hex");
        text
    })
}

/// A serial link that accepts only the expected requests, in order, and answers each with its
/// recorded status packet. Broadcast writes expect no answer. Like the mint's port, its packet
/// timeout runs out as soon as it has nothing more to hand out.
#[derive(Default)]
struct FakePort {
    expected: VecDeque<(Vec<u8>, Vec<u8>)>,
    pending: VecDeque<u8>,
    written: Vec<Vec<u8>>,
    /// Shared with the test, which flips it to make the input buffer undiscardable.
    clear_fails: Arc<AtomicBool>,
}

impl FakePort {
    fn expecting(exchanges: &[Exchange]) -> Self {
        Self {
            expected: exchanges
                .iter()
                .map(|exchange| (bytes(&exchange.request), bytes(&exchange.response)))
                .collect(),
            ..Self::default()
        }
    }
}

impl SerialPort for FakePort {
    fn clear_port(&mut self) -> io::Result<()> {
        if self.clear_fails.load(Ordering::SeqCst) {
            return Err(io::Error::other("the input buffer cannot be flushed"));
        }
        self.pending.clear();
        Ok(())
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        self.written.push(packet.to_vec());
        if packet.get(2) == Some(&0xFE) {
            return Ok(packet.len());
        }
        if let Some((request, response)) = self.expected.pop_front() {
            assert_eq!(hex(packet), hex(&request), "request bytes");
            self.pending.extend(response);
        }
        Ok(packet.len())
    }

    fn read_port(&mut self, length: usize) -> io::Result<Vec<u8>> {
        let available = length.min(self.pending.len());
        Ok(self.pending.drain(..available).collect())
    }

    fn timeout(&self) -> Duration {
        Duration::from_millis(10)
    }

    fn set_packet_timeout(&mut self, _packet_length: usize) {}

    fn is_packet_timeout(&mut self) -> bool {
        true
    }
}

fn servos(golden: &Golden, calibration: &str) -> Vec<FeetechServo> {
    let rows = &golden.calibrations[calibration];
    for (motor, row) in golden.motors.iter().zip(rows) {
        assert_eq!(motor.id, row.id);
    }
    let calibration: [MotorCalibration; 6] = rows
        .iter()
        .map(|row| MotorCalibration {
            drive_mode: row.drive_mode != 0,
            range_min: row.range_min,
            range_max: row.range_max,
        })
        .collect::<Vec<_>>()
        .try_into()
        .expect("six servos");
    so101(calibration)
}

fn open(exchanges: &[Exchange], calibration: &str) -> FeetechChain<FakePort> {
    FeetechChain::open(
        FakePort::expecting(exchanges),
        servos(&golden(), calibration),
    )
    .expect("open")
}

fn with(parts: &[&[Exchange]]) -> Vec<Exchange> {
    parts.iter().flat_map(|part| part.iter().cloned()).collect()
}

/// Rewrite the status packet of `exchange` to carry `error` and `params`.
fn answer(exchange: &mut Exchange, error: u8, params: &[u8]) {
    let id = bytes(&exchange.request)[2];
    let mut body = vec![id, u8::try_from(params.len() + 2).expect("length"), error];
    body.extend_from_slice(params);
    let sum = !body
        .iter()
        .fold(0_u8, |total, byte| total.wrapping_add(*byte));
    let mut packet = vec![0xFF, 0xFF];
    packet.extend(body);
    packet.push(sum);
    exchange.response = hex(&packet);
}

/// The safe-stop transcript's exchanges for `id`: torque off, unlock, torque read-back.
fn stop_exchanges(golden: &Golden, id: u8) -> Vec<usize> {
    golden
        .safe_stop
        .iter()
        .enumerate()
        .filter(|(_, exchange)| bytes(&exchange.request)[2] == id)
        .map(|(index, _)| index)
        .collect()
}

#[test]
fn the_transcript_is_the_bytes_the_provenance_names() {
    let provenance: Value =
        serde_json::from_str(include_str!("fixtures/scservo/provenance.json")).expect("provenance");
    assert_eq!(
        hex(&Sha256::digest(GOLDEN.as_bytes())),
        provenance["sts3215_so101.json"]["sha256"].as_str().unwrap()
    );
    assert_eq!(
        hex(&Sha256::digest(include_bytes!(
            "fixtures/so101_bindings.json"
        ))),
        provenance["sources"]["so101_bindings.json"]["sha256"]
            .as_str()
            .unwrap()
    );
}

#[test]
fn goal_positions_are_the_vendors_packets_byte_for_byte() {
    let golden = golden();
    assert!(golden.goal_positions.len() >= 8);
    for case in &golden.goal_positions {
        let mut chain = open(
            &with(&[&golden.identify, &golden.torque_enable]),
            &case.calibration,
        );
        let ticks: Vec<u16> = chain
            .servos()
            .iter()
            .zip(&case.radians)
            .map(|(servo, joint)| servo.goal_position(*joint).expect("ticks"))
            .collect();
        assert_eq!(ticks, case.ticks, "{} {:?}", case.calibration, case.radians);
        let receipt = chain.command(&case.radians).expect("command");
        assert_eq!(receipt.evidence, Evidence::DispatchAttempted);
        assert!(receipt.observed.is_empty() && receipt.feedback.is_empty());
        assert!(chain.port().expected.is_empty(), "torque was enabled first");
        assert_eq!(
            chain.port().written.last().map(|packet| hex(packet)),
            Some(case.packet.clone()),
            "{} {:?}",
            case.calibration,
            case.radians
        );
        // An enabled chain sends the SYNC WRITE alone.
        let before = chain.port().written.len();
        chain.command(&case.radians).expect("second command");
        assert_eq!(chain.port().written.len(), before + 1);
    }
}

#[test]
fn a_stop_disables_torque_proves_it_and_the_next_command_re_enables() {
    let golden = golden();
    let mut chain = open(
        &with(&[
            &golden.identify,
            &golden.torque_enable,
            &golden.safe_stop,
            &golden.torque_enable,
        ]),
        "so101",
    );
    // Three exchanges per servo (torque off, unlock, read-back), each within the link's
    // 10 ms timeout.
    assert_eq!(
        chain.stop_budget(),
        Duration::from_millis(10) * 3 * u32::try_from(golden.motors.len()).unwrap()
    );
    let radians = &golden.goal_positions[1].radians;
    chain.command(radians).expect("command");
    let report = chain.stop();
    assert_eq!(report, StopReport::default());
    assert!(report.prove().is_ok());
    chain.command(radians).expect("command after a proven stop");
    assert!(chain.port().expected.is_empty(), "torque was enabled again");
}

#[test]
fn a_servo_still_holding_torque_leaves_the_stop_unproven() {
    let golden = golden();
    let mut stop = golden.safe_stop.clone();
    let last = stop.last_mut().expect("read-back");
    answer(last, 0, &[1]);
    let mut chain = open(&with(&[&golden.identify, &stop]), "so101");
    let report = chain.stop();
    assert_eq!(report.unproven, vec![6]);
    assert!(matches!(
        chain.command(&golden.goal_positions[0].radians),
        Err(FeetechChainError::StopPending)
    ));
}

#[test]
fn an_unqualified_silent_or_misdeclared_servo_is_refused_at_open() {
    let golden = golden();
    let mut identify = golden.identify.clone();
    answer(&mut identify[2], 0, &[0x0A, 0x03]);
    assert!(matches!(
        FeetechChain::open(FakePort::expecting(&identify), servos(&golden, "so101")),
        Err(FeetechChainError::UnqualifiedServo {
            bus_id: 3,
            model: 778
        })
    ));
    assert!(matches!(
        FeetechChain::open(
            FakePort::expecting(&golden.identify[..2]),
            servos(&golden, "so101")
        ),
        Err(FeetechChainError::Comm {
            bus_id: 3,
            result: CommResult::RxTimeout
        })
    ));
    let mut flagged = golden.identify.clone();
    answer(&mut flagged[0], 0x04, &[0x09, 0x03]);
    assert!(matches!(
        FeetechChain::open(FakePort::expecting(&flagged), servos(&golden, "so101")),
        Err(FeetechChainError::ServoError {
            bus_id: 1,
            error: 0x04
        })
    ));
    let mut duplicated = servos(&golden, "so101");
    duplicated[1].axis.binding.bus_id = 1;
    assert!(matches!(
        FeetechChain::open(FakePort::default(), duplicated),
        Err(FeetechChainError::Binding(BindingError::DuplicateAddress(
            1
        )))
    ));
    let mut broadcast = servos(&golden, "so101");
    broadcast[0].axis.binding.bus_id = 0xFE;
    assert!(matches!(
        FeetechChain::open(FakePort::default(), broadcast),
        Err(FeetechChainError::Ids(0xFE))
    ));
    let mut uncalibrated = servos(&golden, "so101");
    uncalibrated[4].calibration.range_max = 4096;
    assert!(matches!(
        FeetechChain::open(FakePort::default(), uncalibrated),
        Err(FeetechChainError::Calibration(5))
    ));
    let mut other_model = servos(&golden, "so101");
    other_model[0].axis.binding.model = sx_embodiment_drivers::ActuatorModel::new("damiao_dm4310");
    assert!(matches!(
        FeetechChain::open(FakePort::default(), other_model),
        Err(FeetechChainError::Unqualified(1))
    ));
}

#[test]
fn out_of_range_or_wrong_width_targets_are_refused_before_the_bus() {
    let golden = golden();
    let mut chain = open(&golden.identify, "so101");
    let written = chain.port().written.len();
    assert!(matches!(
        chain.command(&[0.0; 5]),
        Err(FeetechChainError::Target(TargetError::Width {
            expected: 6,
            actual: 5
        }))
    ));
    for values in [vec![0.0, 0.0, 0.0, 0.0, 0.0, 3.0], vec![f64::NAN; 6]] {
        assert!(matches!(
            chain.command(&values),
            Err(FeetechChainError::Target(TargetError::OutOfBounds { .. }))
        ));
    }
    assert_eq!(
        chain.port().written.len(),
        written,
        "nothing reached the bus"
    );
}

#[test]
fn a_stalled_servo_mid_chain_still_lets_every_later_servo_torque_off() {
    let golden = golden();
    let mut stop = golden.safe_stop.clone();
    // Servo 2 is stalled: its torque-off acknowledgement carries the overload flag.
    let torque_off = stop_exchanges(&golden, 2)[0];
    answer(&mut stop[torque_off], 0x20, &[]);
    let mut chain = open(&with(&[&golden.identify, &stop]), "so101");
    let report = chain.stop();
    assert!(
        chain.port().expected.is_empty(),
        "every servo was commanded"
    );
    // The overload is not hidden behind the proven stop.
    assert_eq!(
        report,
        StopReport {
            unproven: vec![],
            faults: vec![(2, 0x20)]
        }
    );
    assert_eq!(report.prove().expect("proven").faults(), &[(2, 0x20)]);
}

#[test]
fn a_link_that_cannot_flush_its_input_still_sends_every_torque_off() {
    let golden = golden();
    let port = FakePort::expecting(&with(&[&golden.identify, &golden.safe_stop]));
    let clear_fails = Arc::clone(&port.clear_fails);
    let mut chain = FeetechChain::open(port, servos(&golden, "so101")).expect("open");
    clear_fails.store(true, Ordering::SeqCst);
    assert!(chain.stop().is_proven());
    assert!(
        chain.port().expected.is_empty(),
        "every servo was commanded"
    );
}

#[test]
fn a_silent_servo_is_reported_after_every_other_servo_is_stopped() {
    let golden = golden();
    let mut stop = golden.safe_stop.clone();
    for index in stop_exchanges(&golden, 2) {
        stop[index].response = String::new();
    }
    let mut chain = open(&with(&[&golden.identify, &stop]), "so101");
    let report = chain.stop();
    assert_eq!(report.unproven, vec![2]);
    assert!(
        chain.port().expected.is_empty(),
        "servos 3 to 6 were still commanded"
    );
    assert!(matches!(
        chain.command(&golden.goal_positions[0].radians),
        Err(FeetechChainError::StopPending)
    ));
}

#[test]
fn a_reply_from_another_servo_is_skipped_as_the_vendor_skips_it() {
    let golden = golden();
    let mut identify = golden.identify.clone();
    // Servo 1's answer arrives behind a stray status packet from servo 9.
    let stray = {
        let mut stray = identify[0].clone();
        stray.request = "ffff0904020302ed".to_owned();
        answer(&mut stray, 0, &[0x09, 0x03]);
        stray.response
    };
    identify[0].response = format!("{stray}{}", identify[0].response);
    FeetechChain::open(FakePort::expecting(&identify), servos(&golden, "so101"))
        .expect("the stray packet is skipped");
}
