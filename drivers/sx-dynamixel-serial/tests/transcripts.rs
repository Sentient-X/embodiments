//! The chain replays what ROBOTIS's own `DynamixelSDK` did, byte for byte.
//!
//! `fixtures/dynamixel_sdk/xl330_xm430.json` is minted by `fixtures/dynamixel_sdk/mint.py`,
//! which runs the vendor SDK itself on the test pair in `fixtures/dynamixel_sdk/bindings.json`;
//! see its header for the sources. What the chain adds over the vendor's sequence (its own
//! limits, the hold before enabling, the observations) is checked on packets built here with
//! the SDK's CRC, beside the vendor's.

mod common;

use common::{FakePort, bytes, hex};
use serde::Deserialize;
use serde_json::Value;
use sha2::{Digest, Sha256};
use sx_dynamixel_serial::chain::{
    GOAL_CURRENT, GOAL_POSITION, LIMITS_READ_LENGTH, PRESENT_POSITION, PROFILE_ACCELERATION,
    PROFILE_VELOCITY,
};
use sx_dynamixel_serial::dynamixel_sdk::{GroupSyncRead, Protocol2PacketHandler, update_crc};
use sx_dynamixel_serial::{
    DynamixelChain, DynamixelChainError, DynamixelServo, MotorCalibration, MotorNormMode,
    OperatingMode,
};
use sx_embodiment_drivers::{
    ActuatorChain, ApproachLimit, BRING_UP_APPROACH, BodyBindings, ChainError, ChainFailure,
    Evidence,
};

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

/// The finger's declared `Goal Current`, in the XL330's own steps.
const FINGER_CURRENT: u16 = 100;
/// Where both servos rest: the middle of each one's calibrated range.
const REST: u32 = 2050;

fn servos(golden: &Golden) -> Vec<DynamixelServo> {
    let body = BodyBindings::parse(BINDINGS).expect("bindings");
    let setups = [
        (
            MotorNormMode::Range0100,
            OperatingMode::CurrentPosition,
            Some(FINGER_CURRENT),
        ),
        (MotorNormMode::Degrees, OperatingMode::Position, None),
    ];
    body.axes
        .into_iter()
        .zip(setups)
        .zip(&golden.calibration)
        .map(
            |((axis, (norm_mode, operating_mode, goal_current)), row)| DynamixelServo {
                axis,
                norm_mode,
                calibration: MotorCalibration {
                    drive_mode: row.drive_mode != 0,
                    homing_offset: row.homing_offset,
                    range_min: row.range_min,
                    range_max: row.range_max,
                },
                operating_mode,
                approach: BRING_UP_APPROACH,
                goal_current,
                multi_turn: operating_mode != OperatingMode::Position,
            },
        )
        .collect()
}

/// A Protocol 2.0 instruction packet (no stuffing: nothing built here carries the header).
fn instruction(id: u8, instruction: u8, params: &[u8]) -> String {
    let length = u16::try_from(params.len() + 3).expect("length");
    let mut packet = vec![0xFF, 0xFF, 0xFD, 0x00, id];
    packet.extend(length.to_le_bytes());
    packet.push(instruction);
    packet.extend_from_slice(params);
    let crc = update_crc(0, &packet);
    packet.extend(crc.to_le_bytes());
    hex(&packet)
}

fn status(id: u8, error: u8, params: &[u8]) -> String {
    let mut exchange = Exchange {
        request: instruction(id, 1, &[]),
        response: String::new(),
    };
    answer(&mut exchange, error, params);
    exchange.response
}

fn write(id: u8, address: u16, data: &[u8]) -> Exchange {
    let mut params = address.to_le_bytes().to_vec();
    params.extend_from_slice(data);
    Exchange {
        request: instruction(id, 3, &params),
        response: status(id, 0, &[]),
    }
}

fn read(id: u8, address: u16, data: &[u8]) -> Exchange {
    let length = u16::try_from(data.len()).expect("length");
    let mut params = address.to_le_bytes().to_vec();
    params.extend(length.to_le_bytes());
    Exchange {
        request: instruction(id, 2, &params),
        response: status(id, 0, data),
    }
}

/// One servo's registers 102..=135: `Goal Current`, the two profile registers and
/// `Present Position`, the rest zero.
fn limits_image(goal_current: u16, (velocity, acceleration): (u32, u32), present: u32) -> Vec<u8> {
    let mut image = vec![0; usize::from(LIMITS_READ_LENGTH)];
    image[0..2].copy_from_slice(&goal_current.to_le_bytes());
    image[6..10].copy_from_slice(&acceleration.to_le_bytes());
    image[10..14].copy_from_slice(&velocity.to_le_bytes());
    image[30..34].copy_from_slice(&present.to_le_bytes());
    image
}

/// Servo `index`'s registers as the chain leaves them, resting at `present`.
fn held(golden: &Golden, index: usize, present: u32) -> Vec<u8> {
    let servo = &servos(golden)[index];
    limits_image(
        servo.goal_current.unwrap_or(0),
        servo.profile().expect("a profile"),
        present,
    )
}

fn id(index: usize) -> u8 {
    u8::try_from(index + 1).expect("id")
}

/// Writing servo `index`'s limits and reading them back.
fn writes_limits(golden: &Golden, index: usize) -> Vec<Exchange> {
    let servo = &servos(golden)[index];
    let (velocity, acceleration) = servo.profile().expect("a profile");
    let mut exchanges = Vec::new();
    if let Some(current) = servo.goal_current {
        exchanges.push(write(id(index), GOAL_CURRENT, &current.to_le_bytes()));
    }
    exchanges.push(write(
        id(index),
        PROFILE_ACCELERATION,
        &acceleration.to_le_bytes(),
    ));
    exchanges.push(write(id(index), PROFILE_VELOCITY, &velocity.to_le_bytes()));
    exchanges.push(read(id(index), GOAL_CURRENT, &held(golden, index, REST)));
    exchanges
}

/// Every servo's `Present_Position`, one READ each: open's and each stop's observation.
fn positions() -> Vec<Exchange> {
    (0..2)
        .map(|index| read(id(index), PRESENT_POSITION, &REST.to_le_bytes()))
        .collect()
}

/// What open adds after the vendor's sequence: every limit, then where every servo rests.
fn limits_at_open(golden: &Golden) -> Vec<Exchange> {
    let mut exchanges: Vec<Exchange> = (0..2)
        .flat_map(|index| writes_limits(golden, index))
        .collect();
    exchanges.extend(positions());
    exchanges
}

/// An enable whose limits all still hold: the read-back, the hold where each servo rests,
/// then the vendor's torque enable.
fn enabling(golden: &Golden) -> Vec<Exchange> {
    let mut exchanges: Vec<Exchange> = (0..2)
        .map(|index| read(id(index), GOAL_CURRENT, &held(golden, index, REST)))
        .collect();
    for index in 0..2 {
        exchanges.push(read(id(index), PRESENT_POSITION, &REST.to_le_bytes()));
        exchanges.push(write(id(index), GOAL_POSITION, &REST.to_le_bytes()));
    }
    exchanges.extend(golden.torque_enable.iter().cloned());
    exchanges
}

/// A command's SYNC READ, each servo answering `images[i]`.
fn sync_read(images: &[Vec<u8>]) -> Exchange {
    let mut params = GOAL_CURRENT.to_le_bytes().to_vec();
    params.extend(LIMITS_READ_LENGTH.to_le_bytes());
    params.extend([1, 2]);
    Exchange {
        request: instruction(0xFE, 0x82, &params),
        response: images
            .iter()
            .enumerate()
            .map(|(index, image)| status(id(index), 0, image))
            .collect(),
    }
}

fn sync_read_held(golden: &Golden) -> Exchange {
    sync_read(&[held(golden, 0, REST), held(golden, 1, REST)])
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
        &limits_at_open(golden),
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
    let reads = vec![sync_read_held(&golden); golden.goal_positions.len()];
    let mut chain = open(&golden, &[&enabling(&golden), &reads]);
    for (index, goal) in golden.goal_positions.iter().enumerate() {
        let servos = chain.servos().to_vec();
        for ((servo, radians), ticks) in servos.iter().zip(&goal.radians).zip(&goal.ticks) {
            assert_eq!(servo.goal_position(*radians).expect("encodes"), *ticks);
        }
        let receipt = chain.command(&goal.radians).expect("commanded");
        assert_eq!(receipt.evidence, Evidence::DispatchAttempted);
        let written = &chain.port().written;
        // The SYNC WRITE, then the chain's own SYNC READ.
        assert_eq!(
            hex(&written[written.len() - 2]),
            goal.packet,
            "action {index}"
        );
    }
    assert!(chain.port().expected.is_empty(), "torque was enabled once");
}

#[test]
fn a_stop_disables_torque_proves_it_and_the_next_command_re_enables() {
    let golden = golden();
    let read = [sync_read_held(&golden)];
    let mut chain = open(
        &golden,
        &[
            &enabling(&golden),
            &read,
            &golden.safe_stop,
            &positions(),
            &enabling(&golden),
            &read,
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
    let mut chain = open(&golden, &[&stop, &positions()]);
    let report = chain.stop();
    assert_eq!(report.unproven, [1]);
    let error = chain.command(&[0.0, 0.0]).expect_err("a stop is pending");
    assert!(matches!(error, DynamixelChainError::StopPending));
    // A servo that does not answer its torque-off is still read back, and every other servo
    // is still stopped.
    let mut stop = golden.safe_stop.clone();
    stop[0].response = String::new();
    let mut chain = open(&golden, &[&stop, &positions()]);
    let report = chain.stop();
    assert_eq!(report.unproven, [1]);
    assert_eq!(chain.port().written.len(), opening(&golden).len() + 6);
}

#[test]
fn a_hardware_error_read_while_stopping_is_latched_until_cleared() {
    let golden = golden();
    let mut stop = golden.safe_stop.clone();
    // The wrist reports an overload (bit 5) with its alert bit set.
    answer(&mut stop[3], 0x80, &[0, 0, 0, 0, 2, 0, 0x20]);
    let read = [sync_read_held(&golden)];
    let mut chain = open(&golden, &[&stop, &positions(), &enabling(&golden), &read]);
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
fn the_stop_budget_is_three_exchanges_per_servo() {
    let golden = golden();
    let chain = open(&golden, &[]);
    assert_eq!(chain.stop_budget(), std::time::Duration::from_millis(60));
}

#[test]
fn open_confirms_every_limit_and_observes_where_the_body_rests() {
    let golden = golden();
    let chain = open(&golden, &[]);
    assert!(chain.port().expected.is_empty());
    // 0.25 rad/s is 10 steps of 0.229 rev/min, 0.5 rad/s² one of 214.577 rev/min².
    assert_eq!(chain.servos()[0].profile(), Some((10, 1)));
    assert!(chain.approach_speed_limited(0) && chain.approach_speed_limited(1));
    let rest: Vec<f64> = chain
        .servos()
        .iter()
        .map(|servo| servo.joint_from_raw(2050))
        .collect();
    assert_eq!(chain.last_observed(), Some(rest));
}

#[test]
fn an_unlimited_profile_or_an_undeclared_mode_is_refused_before_the_bus() {
    let golden = golden();
    let refused = |servos: Vec<DynamixelServo>| {
        let error = DynamixelChain::open(FakePort::default(), servos).err();
        assert_eq!(
            error.as_ref().map(ChainError::failure),
            Some(ChainFailure::Refused)
        );
        error
    };
    // A limit below one step would be written as 0, which the servo reads as no limit.
    let mut slow = servos(&golden);
    slow[1].approach = ApproachLimit {
        speed: 0.01,
        acceleration: 0.5,
    };
    assert!(matches!(
        refused(slow),
        Some(DynamixelChainError::ApproachLimit(2))
    ));
    let mut undeclared = servos(&golden);
    undeclared[0].multi_turn = false;
    assert!(matches!(
        refused(undeclared),
        Some(DynamixelChainError::MultiTurn(1))
    ));
    let mut extended = servos(&golden);
    extended[1].operating_mode = OperatingMode::ExtendedPosition;
    assert!(matches!(
        refused(extended),
        Some(DynamixelChainError::MultiTurn(2))
    ));
    let mut uncurrent = servos(&golden);
    uncurrent[0].goal_current = None;
    assert!(matches!(
        refused(uncurrent),
        Some(DynamixelChainError::GoalCurrent(1))
    ));
    let mut stray = servos(&golden);
    stray[1].goal_current = Some(50);
    assert!(matches!(
        refused(stray),
        Some(DynamixelChainError::GoalCurrent(2))
    ));
}

#[test]
fn a_limit_that_does_not_read_back_is_refused_at_open() {
    let golden = golden();
    let mut limits = limits_at_open(&golden);
    // The finger reads its Profile Velocity back as 0: no limit at all.
    limits[3].response = status(1, 0, &limits_image(FINGER_CURRENT, (0, 1), REST));
    let exchanges = [
        golden.identify.as_slice(),
        &golden.read_calibration,
        &golden.configure,
        &limits,
    ]
    .concat();
    let error = DynamixelChain::open(port(&[&exchanges]), servos(&golden)).err();
    assert!(matches!(
        error,
        Some(DynamixelChainError::Unconfirmed {
            bus_id: 1,
            register: "Profile Velocity",
            written: 10,
            read: 0
        })
    ));
}

#[test]
fn enabling_holds_every_servo_where_it_rests_before_torque_comes_on() {
    let golden = golden();
    let read = [sync_read_held(&golden)];
    let mut chain = open(&golden, &[&enabling(&golden), &read]);
    let before = chain.port().written.len();
    let receipt = chain.command(&[0.0, 0.0]).expect("commanded");
    assert!(chain.port().expected.is_empty(), "every exchange in order");
    let written: Vec<String> = chain.port().written[before..]
        .iter()
        .map(|packet| hex(packet))
        .collect();
    let at = |exchange: &Exchange| {
        written
            .iter()
            .position(|packet| *packet == exchange.request)
            .expect("written")
    };
    let torque_on = at(&golden.torque_enable[0]);
    for index in 0..2 {
        assert!(at(&write(id(index), GOAL_POSITION, &REST.to_le_bytes())) < torque_on);
    }
    // The command observes both servos; what open observed no longer holds.
    assert!(receipt.observed.iter().all(|joint| joint.is_finite()));
    assert_eq!(chain.last_observed(), None);
}

#[test]
fn a_servo_that_lost_its_profile_is_nan_and_the_next_enable_restores_it() {
    let golden = golden();
    // The wrist power-cycled while the link stayed up: its profile reads 0 again.
    let lost = limits_image(0, (0, 0), REST);
    let mut reenable = vec![
        read(1, GOAL_CURRENT, &held(&golden, 0, REST)),
        read(2, GOAL_CURRENT, &lost),
    ];
    reenable.extend(writes_limits(&golden, 1));
    reenable.extend(enabling(&golden).into_iter().skip(2));
    let mut chain = open(
        &golden,
        &[
            &enabling(&golden),
            &[sync_read(&[held(&golden, 0, REST), lost.clone()])],
            &reenable,
            &[sync_read_held(&golden)],
        ],
    );
    let receipt = chain.command(&[0.0, 0.0]).expect("commanded");
    assert!(receipt.observed[0].is_finite());
    assert!(receipt.observed[1].is_nan());
    assert!(chain.approach_speed_limited(0));
    assert!(!chain.approach_speed_limited(1));
    let receipt = chain.command(&[0.0, 0.0]).expect("re-enabled");
    assert!(receipt.observed.iter().all(|joint| joint.is_finite()));
    assert!(chain.approach_speed_limited(1));
    assert!(chain.port().expected.is_empty());
}

#[test]
fn a_servo_missing_from_the_sync_read_is_nan() {
    let golden = golden();
    // Only the finger answers. The vendor's getData answers nothing once any answer of the
    // read failed, so every axis is NaN, and the next command enables afresh.
    let mut partial = sync_read_held(&golden);
    partial.response = status(1, 0, &held(&golden, 0, REST));
    let mut chain = open(&golden, &[&enabling(&golden), &[partial]]);
    let receipt = chain.command(&[0.0, 0.0]).expect("commanded");
    assert!(receipt.observed.iter().all(|joint| joint.is_nan()));
}

#[test]
fn a_stop_observes_where_every_servo_rests() {
    let golden = golden();
    let mut chain = open(
        &golden,
        &[
            &enabling(&golden),
            &[sync_read_held(&golden)],
            &golden.safe_stop,
            &positions(),
        ],
    );
    chain.command(&[0.0, 0.0]).expect("commanded");
    assert_eq!(chain.last_observed(), None);
    assert!(chain.stop().is_proven());
    assert_eq!(chain.last_observed().map(|joints| joints.len()), Some(2));
    // A stop whose position reads go unanswered observes nothing.
    let mut chain = open(&golden, &[&golden.safe_stop]);
    assert!(chain.stop().is_proven());
    assert_eq!(chain.last_observed(), None);
}

#[test]
fn a_sync_read_is_the_vendors() {
    let golden = golden();
    let read = &golden.sync_read;
    // The packets built here for the chain's own exchanges are the vendor's.
    assert_eq!(instruction(0xFE, 0x82, &[132, 0, 4, 0, 1, 2]), read.request);
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

#[test]
fn a_chain_of_servos_on_two_ports_is_refused_before_the_bus() {
    let golden = golden();
    let mut split = servos(&golden);
    split[1].axis.port = "elsewhere".to_owned();
    split[1].axis.binding.bus_id = 1;
    assert!(matches!(
        DynamixelChain::open(FakePort::default(), split),
        Err(DynamixelChainError::Ports(ports)) if ports == ["dynamixel", "elsewhere"]
    ));
}
