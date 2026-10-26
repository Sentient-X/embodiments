//! What keeps the static upper body from moving where or how it should not, on the simulated
//! Damiao bus: each motor's communication timeout, armed right before a phased enable so none
//! trips; a joint resting outside its range, held or brought in and never sent further out; the
//! governor's slew from where the body rests; and what a stop and a cleared fault prove.

use std::time::Duration;

use serde_json::Value;
use sx_damiao_can::chain::{COMMUNICATION_LOST, DamiaoChain, DamiaoChainError};
use sx_damiao_can::sim::{Sim, feedback};
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::can::{CanFrame, FRAME_ALLOWANCE};
use sx_embodiment_drivers::{ActuatorChain, AxisLimits, Governor, InstanceDrive};

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

fn index_of(joint: &str) -> usize {
    STATIC_UPPER_BODY
        .joints
        .iter()
        .position(|gains| gains.joint == joint)
        .expect("a joint of the body")
}

/// The TIMEOUT write (0x55 to register 0x09) and read frames for `id`.
fn timeout_frames(id: u16, counts: [u8; 4]) -> (CanFrame, CanFrame) {
    let id = id.to_le_bytes();
    (
        CanFrame {
            id: 0x7FF,
            data: [
                id[0], id[1], 0x55, 0x09, counts[0], counts[1], counts[2], counts[3],
            ],
        },
        CanFrame {
            id: 0x7FF,
            data: [id[0], id[1], 0x33, 0x09, 0, 0, 0, 0],
        },
    )
}

#[test]
fn the_first_enable_writes_every_motors_timeout_to_ram_and_reads_it_back() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    // Open arms no watchdog: it is armed right before the enable.
    assert!(
        bus.state()
            .motors
            .values()
            .all(|motor| !motor.registers.contains_key(&0x09))
    );
    let rest = chain.last_observed().expect("observed at open");
    chain.command(&rest).expect("a command at rest");
    // 0.75 s in the register's 50 µs counts.
    let counts = 15_000_u32.to_le_bytes();
    let state = bus.state();
    let first_enable = state
        .frames
        .iter()
        .position(|frame| frame.data == [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
        .expect("an enable");
    for motor in &axes {
        let (write, read) = timeout_frames(motor.axis.binding.bus_id, counts);
        let write_at = state
            .frames
            .iter()
            .position(|frame| *frame == write)
            .expect("the TIMEOUT write");
        let read_at = state
            .frames
            .iter()
            .position(|frame| *frame == read)
            .expect("the TIMEOUT read");
        assert!(
            write_at < read_at && read_at < first_enable,
            "written, read, then enabled"
        );
        assert_eq!(
            state.motors[&motor.axis.binding.bus_id].registers[&0x09],
            counts
        );
    }
    // Never saved to flash: no frame carries the save command (0xAA).
    assert!(
        state
            .frames
            .iter()
            .all(|frame| !(frame.id == 0x7FF && frame.data[2] == 0xAA))
    );
}

#[test]
fn a_motor_that_does_not_hold_its_timeout_is_never_enabled() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let first = axes[0].axis.binding.bus_id;
    // A timeout below one count is refused before any frame.
    let bus = Sim::for_axes(&axes);
    let mut zero = axes.clone();
    zero[0].can_timeout = Some(Duration::from_micros(10));
    assert!(matches!(
        DamiaoChain::open(bus.clone(), zero),
        Err(DamiaoChainError::TimeoutRange(id)) if id == first
    ));
    assert_eq!(bus.sent(), 0, "refused before any frame");
    // Firmware that echoes the write but does not hold it fails the read-back.
    bus.state()
        .motors
        .get_mut(&first)
        .expect("first")
        .ignored
        .insert(0x09);
    let mut chain = DamiaoChain::open(bus.clone(), axes).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    assert!(matches!(
        chain.command(&rest),
        Err(DamiaoChainError::TimeoutNotConfirmed { bus_id, counts: 15_000 }) if bus_id == first
    ));
    assert!(bus.state().motors.values().all(|motor| motor.state == 0));
}

#[test]
fn bringing_fifteen_motors_up_and_enabling_them_trips_no_watchdog() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    // Four round trips: disable, MIT mode, every limit register, disable and settle; one frame
    // per motor in each, three in the limit reads, and no fixed wait.
    assert_eq!(bus.sent(), 15 * 6);
    assert_eq!(bus.take_waited(), Duration::ZERO);
    let rest = chain.last_observed().expect("observed at open");
    let opened = bus.sent();
    for _ in 0..50 {
        chain.command(&rest).expect("a command at rest");
        // A 50 Hz control period between commands, well inside the 0.75 s watchdog.
        bus.pass(Duration::from_millis(20));
    }
    // The first command's four round trips: TIMEOUT written, read back, enable, MIT.
    assert!(bus.state().frames.len() >= opened + 15 * 4);
    assert!(
        bus.state().motors.values().all(|motor| motor.state == 1),
        "no motor latched communication lost"
    );
    // The simulated watchdog is real: a motor left silent past it while enabled trips.
    bus.pass(Duration::from_millis(800));
    assert!(
        bus.state()
            .motors
            .values()
            .all(|motor| motor.state == COMMUNICATION_LOST)
    );
}

/// The left wrist roll as found on the static upper body (2026-10-26): resting at -2.937 rad
/// against a range of -2.097..2.096.
const WRIST_REST: f64 = -2.937;

#[test]
fn a_joint_resting_outside_its_range_opens_and_is_held_or_brought_in_never_further_out() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let wrist = index_of("left_wrist_roll_joint");
    let bus = Sim::for_axes(&axes);
    let id = axes[wrist].axis.binding.bus_id;
    let actuator = axes[wrist].axis.actuator_from_joint(WRIST_REST);
    bus.state().motors.get_mut(&id).expect("wrist").q = actuator;
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens where it rests");
    let rest = chain.last_observed().expect("observed at open");
    assert!((rest[wrist] - WRIST_REST).abs() < 1e-3, "{}", rest[wrist]);
    // Held where it rests.
    chain.command(&rest).expect("held outside its range");
    // Further out is refused before any frame.
    let sent = bus.sent();
    let mut out = rest;
    out[wrist] = rest[wrist] - 0.01;
    assert!(matches!(
        chain.command(&out),
        Err(DamiaoChainError::Target(_))
    ));
    assert_eq!(bus.sent(), sent, "refused before any frame");
    // Toward the range is taken, and the interval shrinks behind it.
    let mut inward = rest;
    inward[wrist] = rest[wrist] + 0.1;
    chain.command(&inward).expect("moved toward its range");
    let mut back = rest;
    back[wrist] = inward[wrist] - 0.05;
    assert!(matches!(
        chain.command(&back),
        Err(DamiaoChainError::Target(_))
    ));
}

#[test]
fn a_governed_joint_outside_its_range_comes_in_at_the_speed_bound() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let wrist = index_of("left_wrist_roll_joint");
    let elbow = index_of("right_elbow_pitch_joint");
    let bus = Sim::for_axes(&axes);
    let id = axes[wrist].axis.binding.bus_id;
    bus.state().motors.get_mut(&id).expect("wrist").q =
        axes[wrist].axis.actuator_from_joint(WRIST_REST);
    let chain = DamiaoChain::open(bus.clone(), axes).expect("opens");
    let limits = vec![
        AxisLimits {
            max_speed: 0.25,
            max_tracking_error: 0.2,
        };
        15
    ];
    let mut governed = Governor::new(chain, limits).expect("limits");
    let rest = governed.last_observed().expect("rest");
    // Asked to the wrist's range centre and the elbow (in range) up: both move, each by its
    // own interval, the wrist only ever inward.
    let mut target = rest;
    target[wrist] = 0.0;
    target[elbow] = 2.5;
    let mut last = rest[wrist];
    for _ in 0..20 {
        std::thread::sleep(Duration::from_millis(2));
        let receipt = governed.command(&target).expect("a governed command");
        let now = receipt.observed[wrist];
        assert!(now >= last - 1e-3, "the wrist moved out: {now} from {last}");
        assert!(
            now - last <= 0.005 + 1e-3,
            "faster than the bound: {now} from {last}"
        );
        last = now;
    }
    assert!(last > rest[wrist], "the wrist came in");
    assert!(governed.last_observed().expect("observed")[elbow] > rest[elbow]);
    assert!(governed.stop().is_proven());
}

#[test]
fn the_governor_starts_from_where_the_body_rests_and_slews() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let elbow = index_of("right_elbow_pitch_joint");
    bus.state()
        .motors
        .get_mut(&axes[elbow].axis.binding.bus_id)
        .expect("elbow")
        .q = 1.0;
    let chain = DamiaoChain::open(bus.clone(), axes).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    assert!((rest[elbow] - 1.0).abs() < 1e-3);
    let limits = vec![
        AxisLimits {
            max_speed: 0.25,
            max_tracking_error: 0.2,
        };
        15
    ];
    let mut governed = Governor::new(chain, limits).expect("limits");
    let mut target = rest;
    target[elbow] = 2.5;
    let receipt = governed.command(&target).expect("a governed command");
    // The elbow moved at most one 20 ms step at 0.25 rad/s from where it rested.
    assert!((receipt.observed[elbow] - rest[elbow]).abs() <= 0.005 + 1e-3);
    assert!(governed.stop().is_proven());
    // Fifteen motors on a USB bridge: each disable and refresh budgets the default allowance.
    assert!(governed.stop_budget() >= FRAME_ALLOWANCE * 30);
}

#[test]
fn firmware_that_always_answers_state_1_still_proves_a_stop_and_a_silent_motor_does_not() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    assert!(axes.iter().all(|motor| !motor.reports_enable_state));
    // Every motor's feedback reads state 1, enabled or not, as the bench firmware does.
    let bus = Sim::for_axes(&axes);
    assert!(
        bus.state()
            .motors
            .values()
            .all(|motor| !motor.reports_enable_state)
    );
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    chain.command(&rest).expect("a command at rest");
    let report = chain.stop();
    assert!(report.is_proven(), "{report:?}");
    // Proven only by the answers to the disable: every motor is named as unobserved.
    let ids: Vec<u16> = axes.iter().map(|motor| motor.axis.binding.bus_id).collect();
    assert_eq!(report.unobserved, ids);
    let proven = report.prove().expect("proven");
    assert!(!proven.observed_disabled());
    assert!(bus.state().motors.values().all(|motor| motor.state == 0));
    // A motor that stops answering leaves the stop unproven and motion refused.
    chain.command(&rest).expect("a command at rest");
    let silent = axes[0].axis.binding.bus_id;
    bus.kill(silent);
    bus.take_waited();
    let report = chain.stop();
    assert_eq!(report.unproven, vec![silent]);
    // Waiting out the silent motor twice still ends inside the stop's budget.
    let waited = bus.take_waited();
    assert!(
        waited > Duration::ZERO && waited < chain.stop_budget(),
        "{waited:?}"
    );
    assert!(chain.command(&rest).is_err());
    // The same firmware read as reporting its state refuses the open: state 1 where disabled.
    let mut reporting = axes;
    for motor in &mut reporting {
        motor.reports_enable_state = true;
    }
    let bus = Sim::for_axes(&reporting);
    for motor in bus.state().motors.values_mut() {
        motor.reports_enable_state = false;
    }
    assert!(matches!(
        DamiaoChain::open(bus.clone(), reporting),
        Err(DamiaoChainError::UnexpectedState { .. })
    ));
    assert!(bus.state().motors.values().all(|motor| motor.state == 0));
}

/// Every TIMEOUT write frame (0x55 to register 0x09) sent to `id`, by index into the frames.
fn timeout_writes(bus: &Sim, id: u16) -> Vec<usize> {
    let id = id.to_le_bytes();
    bus.state()
        .frames
        .iter()
        .enumerate()
        .filter(|(_, frame)| frame.id == 0x7FF && frame.data[..4] == [id[0], id[1], 0x55, 0x09])
        .map(|(index, _)| index)
        .collect()
}

/// Every frame index carrying `command` (`FF..command`) to `id`.
fn commands_to(bus: &Sim, id: u16, command: u8) -> Vec<usize> {
    let mut data = [0xFF; 8];
    data[7] = command;
    bus.state()
        .frames
        .iter()
        .enumerate()
        .filter(|(_, frame)| frame.id == u32::from(id) && frame.data == data)
        .map(|(index, _)| index)
        .collect()
}

#[test]
fn a_motor_power_cycled_while_stopped_has_its_timeout_restored_before_any_enable() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    chain.command(&rest).expect("a command at rest");
    assert!(chain.stop().is_proven());
    // The first motor loses power: its RAM TIMEOUT is gone and it reads its flash value, 0.
    let first = axes[0].axis.binding.bus_id;
    bus.state()
        .motors
        .get_mut(&first)
        .expect("first")
        .registers
        .remove(&0x09);
    let stopped_at = bus.state().frames.len();
    chain.command(&rest).expect("a command at rest");
    assert_eq!(
        bus.state().motors[&first].registers[&0x09],
        15_000_u32.to_le_bytes()
    );
    // Every motor's TIMEOUT was rewritten after the stop, and before the first enable frame.
    let first_enable = axes
        .iter()
        .filter_map(|motor| {
            commands_to(&bus, motor.axis.binding.bus_id, 0xFC)
                .into_iter()
                .find(|index| *index >= stopped_at)
        })
        .min()
        .expect("an enable");
    for motor in &axes {
        assert!(
            timeout_writes(&bus, motor.axis.binding.bus_id)
                .iter()
                .any(|index| (stopped_at..first_enable).contains(index)),
            "bus_id {} rewritten before the enable",
            motor.axis.binding.bus_id
        );
    }
}

#[test]
fn a_motor_that_does_not_confirm_its_timeout_after_a_power_cycle_is_never_enabled() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    chain.command(&rest).expect("a command at rest");
    assert!(chain.stop().is_proven());
    let last = axes[axes.len() - 1].axis.binding.bus_id;
    {
        let mut state = bus.state();
        let motor = state.motors.get_mut(&last).expect("last");
        motor.registers.remove(&0x09);
        motor.ignored.insert(0x09);
    }
    let stopped_at = bus.state().frames.len();
    assert!(matches!(
        chain.command(&rest),
        Err(DamiaoChainError::TimeoutNotConfirmed { bus_id, counts: 15_000 }) if bus_id == last
    ));
    // Not one motor was enabled: no enable frame since the stop, every motor still disabled.
    for motor in &axes {
        assert!(
            commands_to(&bus, motor.axis.binding.bus_id, 0xFC)
                .iter()
                .all(|index| *index < stopped_at)
        );
    }
    assert!(bus.state().motors.values().all(|motor| motor.state == 0));
}

#[test]
fn a_late_answer_to_a_command_is_not_taken_as_the_disable_acknowledgement() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    chain.command(&rest).expect("a command at rest");
    // The motor's answer to the last command arrives late, then the motor goes silent.
    let id = axes[3].axis.binding.bus_id;
    let late = feedback(id, &bus.state().motors[&id]);
    bus.deliver_late(late);
    bus.kill(id);
    let report = chain.stop();
    assert_eq!(report.unproven, vec![id], "{report:?}");
    assert!(!report.unobserved.contains(&id));
    assert!(matches!(
        chain.command(&rest),
        Err(DamiaoChainError::StopPending)
    ));
}

#[test]
fn firmware_that_reports_its_state_proves_a_stop_by_observation() {
    let mut axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    for motor in &mut axes {
        motor.reports_enable_state = true;
    }
    let bus = Sim::for_axes(&axes);
    let mut chain = DamiaoChain::open(bus, axes).expect("opens");
    let rest = chain.last_observed().expect("observed at open");
    chain.command(&rest).expect("a command at rest");
    let proven = chain.stop().prove().expect("proven");
    assert!(proven.observed_disabled());
}

#[test]
fn a_motor_latched_in_communication_lost_is_cleared_once_at_open() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let id = axes[2].axis.binding.bus_id;
    let bus = Sim::for_axes(&axes);
    bus.state().motors.get_mut(&id).expect("motor").state = COMMUNICATION_LOST;
    let mut chain = DamiaoChain::open(bus.clone(), axes.clone()).expect("opens once cleared");
    assert_eq!(commands_to(&bus, id, 0xFB).len(), 1, "cleared exactly once");
    assert!(bus.state().motors.values().all(|motor| motor.state == 0));
    // No other motor was sent the clear frame.
    for motor in axes.iter().filter(|motor| motor.axis.binding.bus_id != id) {
        assert!(commands_to(&bus, motor.axis.binding.bus_id, 0xFB).is_empty());
    }
    // The clear is refused while the chain is enabled.
    let rest = chain.last_observed().expect("observed at open");
    chain.command(&rest).expect("a command at rest");
    let sent = bus.sent();
    assert!(matches!(
        chain.clear_fault(id),
        Err(DamiaoChainError::ClearWhileEnabled)
    ));
    assert_eq!(bus.sent(), sent, "refused before any frame");
    assert!(chain.stop().is_proven());
    chain.clear_fault(id).expect("cleared while disabled");
}

#[test]
fn a_fault_that_persists_after_its_clear_still_refuses_the_open() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let id = axes[2].axis.binding.bus_id;
    let bus = Sim::for_axes(&axes);
    {
        let mut state = bus.state();
        let motor = state.motors.get_mut(&id).expect("motor");
        motor.state = COMMUNICATION_LOST;
        motor.persistent_fault = true;
    }
    assert!(matches!(
        DamiaoChain::open(bus.clone(), axes.clone()),
        Err(DamiaoChainError::FaultPersists { bus_id, code }) if bus_id == id && code == COMMUNICATION_LOST
    ));
    assert_eq!(commands_to(&bus, id, 0xFB).len(), 1, "cleared at most once");
    // Any other fault is not cleared at open: an over-current motor refuses it untouched.
    let bus = Sim::for_axes(&axes);
    bus.state().motors.get_mut(&id).expect("motor").state = 0xA;
    assert!(matches!(
        DamiaoChain::open(bus.clone(), axes),
        Err(DamiaoChainError::UnexpectedState { bus_id, .. }) if bus_id == id
    ));
    assert!(commands_to(&bus, id, 0xFB).is_empty());
}
