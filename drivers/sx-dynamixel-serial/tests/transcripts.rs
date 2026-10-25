//! The chain replays what ROBOTIS's own `DynamixelSDK` did, byte for byte.
//!
//! `fixtures/dynamixel_sdk/xl330_xm430.json` is minted by `fixtures/dynamixel_sdk/mint.py`,
//! which runs the vendor SDK itself on the test pair in `fixtures/dynamixel_sdk/bindings.json`;
//! see its header for the sources.

mod common;

use common::{FakePort, bytes, hex};
use serde::Deserialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use sx_dynamixel_serial::dynamixel_sdk::{GroupSyncRead, Protocol2PacketHandler, update_crc};
use sx_dynamixel_serial::{
    DynamixelChain, DynamixelChainError, DynamixelServo, MotorCalibration, MotorNormMode,
    OperatingMode,
};
use sx_embodiment_drivers::{ActuatorChain, BodyBindings, ChainError, ChainFailure, Evidence};

const GOLDEN: &str = include_str!("fixtures/dynamixel_sdk/xl330_xm430.json");
const BINDINGS: &str = include_str!("fixtures/dynamixel_sdk/bindings.json");

#[derive(Deserialize)]
struct Golden {
    calibration: Vec<Calibration>,
    identify: Vec<Exchange>,
    read_calibration: Vec<Exchange>,
    configure: Vec<Exchange>,
    torque_enable: Vec<Exchange>,
    goal_positions: Vec<GoalPosition>,
    safe_stop: Vec<Exchange>,
    sync_read: SyncRead,
}

#[derive(Deserialize)]
struct Calibration {
    drive_mode: u8,
    homing_offset: i16,
    range_min: u16,
    range_max: u16,
}

#[derive(Clone, Deserialize)]
struct Exchange {
    request: String,
    response: String,
}

#[derive(Deserialize)]
struct GoalPosition {
    radians: Vec<f64>,
    ticks: Vec<u32>,
    packet: String,
}

#[derive(Deserialize)]
struct SyncRead {
    request: String,
    responses: Vec<String>,
    positions: Vec<u32>,
}

fn golden() -> Golden {
    serde_json::from_str(GOLDEN).expect("golden transcript")
}

fn servos(golden: &Golden) -> Vec<DynamixelServo> {
    let body = BodyBindings::parse(BINDINGS).expect("bindings");
    let setups = [
        (MotorNormMode::Range0100, OperatingMode::CurrentPosition),
        (MotorNormMode::Degrees, OperatingMode::Position),
    ];
    body.axes
        .into_iter()
        .zip(setups)
        .zip(&golden.calibration)
        .map(
            |((axis, (norm_mode, operating_mode)), row)| DynamixelServo {
                axis,
                norm_mode,
                calibration: MotorCalibration {
                    drive_mode: row.drive_mode != 0,
                    homing_offset: row.homing_offset,
                    range_min: row.range_min,
                    range_max: row.range_max,
                },
                operating_mode,
            },
        )
        .collect()
}

fn port(parts: &[&[Exchange]]) -> FakePort {
    FakePort::expecting(
        parts
            .iter()
            .flat_map(|part| part.iter())
            .map(|exchange| (exchange.request.as_str(), exchange.response.as_str())),
    )
}

fn opening(golden: &Golden) -> Vec<Exchange> {
    [
        golden.identify.as_slice(),
        &golden.read_calibration,
        &golden.configure,
    ]
    .concat()
}

fn open(golden: &Golden, after: &[&[Exchange]]) -> DynamixelChain<FakePort> {
    let opening = opening(golden);
    let mut parts: Vec<&[Exchange]> = vec![&opening];
    parts.extend_from_slice(after);
    DynamixelChain::open(port(&parts), servos(golden)).expect("open")
}

/// Rewrite `exchange`'s status packet to carry `error` and `params`.
fn answer(exchange: &mut Exchange, error: u8, params: &[u8]) {
    let id = bytes(&exchange.request)[4];
    let length = u16::try_from(params.len() + 4).expect("length");
    let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, id];
    packet.extend(length.to_le_bytes());
    packet.extend([0x55, error]);
    packet.extend_from_slice(params);
    let crc = update_crc(0, &packet);
    packet.extend(crc.to_le_bytes());
    exchange.response = hex(&packet);
}

#[test]
fn the_transcript_is_the_bytes_its_sources_record_names() {
    let sources: Value =
        serde_json::from_str(include_str!("fixtures/dynamixel_sdk/sources.json")).expect("sources");
    let digest = |data: &[u8]| hex(&Sha256::digest(data));
    assert_eq!(
        digest(GOLDEN.as_bytes()),
        sources["xl330_xm430.json"]["sha256"].as_str().expect("sha")
    );
    assert_eq!(
        digest(include_bytes!("fixtures/dynamixel_sdk/mint.py")),
        sources["sources"]["mint.py"]["sha256"]
            .as_str()
            .expect("sha")
    );
    assert_eq!(
        digest(BINDINGS.as_bytes()),
        sources["sources"]["bindings.json"]["sha256"]
            .as_str()
            .expect("sha")
    );
}

#[test]
fn open_identifies_checks_the_calibration_and_configures_as_the_vendor_sequence_does() {
    let golden = golden();
    let chain = open(&golden, &[]);
    assert!(chain.port().expected.is_empty(), "every exchange was made");
    assert_eq!(
        chain.port().written.len(),
        opening(&golden).len(),
        "and nothing else"
    );
}

#[test]
fn goal_positions_are_the_vendors_packets_byte_for_byte() {
    let golden = golden();
    let mut chain = open(&golden, &[&golden.torque_enable]);
    for (index, goal) in golden.goal_positions.iter().enumerate() {
        let servos = chain.servos().to_vec();
        for ((servo, radians), ticks) in servos.iter().zip(&goal.radians).zip(&goal.ticks) {
            assert_eq!(servo.goal_position(*radians).expect("encodes"), *ticks);
        }
        let receipt = chain.command(&goal.radians).expect("commanded");
        assert_eq!(receipt.evidence, Evidence::DispatchAttempted);
        let written = chain.port().written.last().expect("a packet");
        assert_eq!(hex(written), goal.packet, "action {index}");
    }
    assert!(chain.port().expected.is_empty(), "torque was enabled once");
}

#[test]
fn a_stop_disables_torque_proves_it_and_the_next_command_re_enables() {
    let golden = golden();
    let mut chain = open(
        &golden,
        &[
            &golden.torque_enable,
            &golden.safe_stop,
            &golden.torque_enable,
        ],
    );
    chain.command(&[0.0, 0.0]).expect("commanded");
    let report = chain.stop();
    assert!(report.is_proven(), "{report:?}");
    assert!(report.faults.is_empty());
    chain
        .command(&[0.0, 0.0])
        .expect("re-enabled and commanded");
    assert!(chain.port().expected.is_empty());
}

#[test]
fn a_servo_still_holding_torque_or_silent_leaves_the_stop_unproven() {
    let golden = golden();
    let mut stop = golden.safe_stop.clone();
    answer(&mut stop[2], 0, &[1, 0, 0, 0, 2, 0, 0]);
    let mut chain = open(&golden, &[&stop]);
    let report = chain.stop();
    assert_eq!(report.unproven, [1]);
    let error = chain.command(&[0.0, 0.0]).expect_err("a stop is pending");
    assert!(matches!(error, DynamixelChainError::StopPending));
    // A servo that does not answer its torque-off is still read back, and every other servo
    // is still stopped.
    let mut stop = golden.safe_stop.clone();
    stop[0].response = String::new();
    let mut chain = open(&golden, &[&stop]);
    let report = chain.stop();
    assert_eq!(report.unproven, [1]);
    assert_eq!(chain.port().written.len(), opening(&golden).len() + 4);
}

#[test]
fn a_hardware_error_read_while_stopping_is_latched_until_cleared() {
    let golden = golden();
    let mut stop = golden.safe_stop.clone();
    // The wrist reports an overload (bit 5) with its alert bit set.
    answer(&mut stop[3], 0x80, &[0, 0, 0, 0, 2, 0, 0x20]);
    let mut chain = open(&golden, &[&stop, &golden.torque_enable]);
    let proven = chain.stop().prove().expect("torque reads off");
    assert_eq!(proven.faults(), &[(2, 0x20)]);
    let error = chain.command(&[0.0, 0.0]).expect_err("faulted");
    assert_eq!(error.failure(), ChainFailure::Refused);
    chain.clear_faults();
    chain.command(&[0.0, 0.0]).expect("cleared");
}

#[test]
fn an_unqualified_faulted_or_miscalibrated_servo_is_refused_at_open() {
    let golden = golden();
    // An XL330-M077 (1190) where the binding names an XL330-M288.
    let mut identify = golden.identify.clone();
    answer(&mut identify[0], 0, &[0xA6, 0x04, 0x33]);
    let error = DynamixelChain::open(port(&[&identify]), servos(&golden)).err();
    assert!(matches!(
        error,
        Some(DynamixelChainError::UnqualifiedServo {
            bus_id: 1,
            model: 1190,
            expected: 1200
        })
    ));
    // A servo reporting an input-voltage error.
    let mut identify = golden.identify.clone();
    answer(&mut identify[3], 0x80, &[0x01]);
    let error = DynamixelChain::open(port(&[&identify]), servos(&golden)).err();
    assert!(matches!(
        error,
        Some(DynamixelChainError::HardwareError {
            bus_id: 2,
            status: 1
        })
    ));
    // A servo whose minimum position limit is not its calibration's.
    let mut read = golden.read_calibration.clone();
    answer(&mut read[1], 0, &1201_u32.to_le_bytes());
    let error = DynamixelChain::open(port(&[&golden.identify, &read]), servos(&golden)).err();
    assert!(matches!(
        error,
        Some(DynamixelChainError::Uncalibrated {
            bus_id: 1,
            range_min: 1201,
            ..
        })
    ));
    // A servo that does not answer.
    let mut identify = golden.identify.clone();
    identify[0].response = String::new();
    let error = DynamixelChain::open(port(&[&identify]), servos(&golden)).err();
    assert_eq!(error.map(|e| e.failure()), Some(ChainFailure::Timeout));
    // A servo bound on another bus never reaches the wire.
    let mut foreign = servos(&golden);
    foreign[0].axis.binding.bus = sx_embodiment_drivers::ActuatorBus::new("feetech_serial");
    let error = DynamixelChain::open(FakePort::default(), foreign).err();
    assert!(matches!(error, Some(DynamixelChainError::Unqualified(1))));
}

#[test]
fn out_of_range_or_wrong_width_targets_are_refused_before_the_bus() {
    let golden = golden();
    let mut chain = open(&golden, &[]);
    let before = chain.port().written.len();
    for bad in [vec![0.0], vec![0.0, 1.6], vec![f64::NAN, 0.0]] {
        let error = chain.command(&bad).expect_err("refused");
        assert_eq!(error.failure(), ChainFailure::Refused);
    }
    assert_eq!(chain.port().written.len(), before);
    // A wrist calibrated narrower than its declared bounds refuses what it cannot reach.
    let mut narrow = servos(&golden);
    narrow[1].calibration.range_min = 1500;
    narrow[1].calibration.range_max = 2100;
    assert!(narrow[1].goal_position(0.0).is_ok());
    assert!(matches!(
        narrow[1].goal_position(1.0),
        Err(DynamixelChainError::OutsideCalibration { bus_id: 2, .. })
    ));
}

#[test]
fn the_stop_budget_is_two_exchanges_per_servo() {
    let golden = golden();
    let chain = open(&golden, &[]);
    assert_eq!(chain.stop_budget(), std::time::Duration::from_millis(40));
}

#[test]
fn a_sync_read_is_the_vendors() {
    let golden = golden();
    let read = &golden.sync_read;
    let port = FakePort::expecting([(
        read.request.as_str(),
        read.responses.concat().leak() as &str,
    )]);
    let mut ph = Protocol2PacketHandler::new(port);
    let mut reader = GroupSyncRead::new(132, 4);
    reader.add_param(1);
    reader.add_param(2);
    assert_eq!(
        reader.tx_rx_packet(&mut ph),
        sx_dynamixel_serial::dynamixel_sdk::CommResult::Success
    );
    assert_eq!(reader.get_data(1, 132, 4), Some(read.positions[0]));
    assert_eq!(reader.get_data(2, 132, 4), Some(read.positions[1]));
}
