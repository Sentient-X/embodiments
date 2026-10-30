//! A gripper's fingers, percent servos, governed at their rated motion on a simulated bus of
//! STS3215s that move at their `Goal_Speed`. A finger's joint moves at its shaft's speed times
//! its declared range over its calibrated span, which is not the binding's reduction; the chain
//! reports that as its tracking ceiling, so the governor plans no faster than the finger moves
//! and opens and closes it at every share without a tracking stop. On 2026-10-30 act-sim's
//! fingers (calibrated 1024..3072 over 0.788 rad, four times slower than their shafts) were
//! ramped at the rated 4 rad/s and the station logged "joint `right_gripper_right_finger_joint`
//! was sent 0.153 but observed at 0.533, more than 0.357 away".

use std::cell::Cell;
use std::rc::Rc;
use std::time::{Duration, Instant};

use sx_embodiment_drivers::{ActuatorChain, AxisLimits, BodyBindings, Clock, Governor};
use sx_feetech_serial::sim::SimBus;
use sx_feetech_serial::static_upper_body::{BINDINGS, RIGHT_GRIPPER};
use sx_feetech_serial::{FeetechChain, MotorCalibration};

#[derive(Clone)]
struct Manual(Rc<Cell<Instant>>);

impl Clock for Manual {
    fn now(&self) -> Instant {
        self.0.get()
    }
}

const fn calibration(range_min: u16, range_max: u16) -> MotorCalibration {
    MotorCalibration {
        drive_mode: false,
        homing_offset: 0,
        range_min,
        range_max,
    }
}

/// act-sim's fixture: both fingers over half a turn.
const ACT_SIM: [MotorCalibration; 2] = [calibration(1024, 3072), calibration(1024, 3072)];
/// The Spark's fingers as the read-only probe of 2026-10-25 found them.
const SPARK: [MotorCalibration; 2] = [calibration(1720, 2302), calibration(1813, 2358)];

fn open(calibration: [MotorCalibration; 2]) -> (SimBus, FeetechChain<SimBus>) {
    let axes = RIGHT_GRIPPER
        .axes(BodyBindings::parse(BINDINGS).expect("bindings"))
        .expect("the gripper's axes");
    let servos = RIGHT_GRIPPER.servos(axes, &calibration).expect("servos");
    let bus = SimBus::sts3215(&[(1, calibration[0]), (2, calibration[1])]);
    let chain = FeetechChain::open(bus.clone(), servos).expect("the gripper opens");
    (bus, chain)
}

#[test]
fn a_fingers_ceiling_is_its_goal_speed_through_its_calibrated_span() {
    let (_, chain) = open(ACT_SIM);
    let ceilings = chain.tracking_ceilings();
    assert_eq!(ceilings.len(), 2);
    for (index, ceiling) in ceilings {
        let axis = &chain.axes()[index];
        // Goal_Speed 2607 steps/s at the shaft; the joint spans its declared range over 2048.
        let expected = 2607.0 / 2048.0 * (axis.upper - axis.lower);
        assert!((ceiling.speed - expected).abs() < 1e-9, "{ceiling:?}");
        assert!(ceiling.speed < axis.motion.speed / 3.0, "{ceiling:?}");
    }
    // The Spark's fingers span about their declared range: near the rated 4 rad/s.
    let (_, chain) = open(SPARK);
    for (_, ceiling) in chain.tracking_ceilings() {
        assert!((3.4..4.3).contains(&ceiling.speed), "{ceiling:?}");
    }
}

#[test]
fn a_gripper_opens_and_closes_at_every_share_without_a_tracking_stop() {
    for calibration in [ACT_SIM, SPARK] {
        for share in [0.25, 0.5, 1.0] {
            let (bus, chain) = open(calibration);
            let limits = AxisLimits::rated_all(chain.axes(), share).expect("rated");
            let ends: Vec<(f64, f64)> = chain.axes().iter().map(|a| (a.lower, a.upper)).collect();
            let clock = Manual(Rc::new(Cell::new(Instant::now())));
            let mut governor = Governor::with_clock(chain, limits, clock.clone()).expect("limits");
            // Open fully, close fully, open again: each the whole declared range.
            for goal in [
                [ends[0].0, ends[1].1],
                [ends[0].1, ends[1].0],
                [ends[0].0, ends[1].1],
            ] {
                let mut reached = false;
                for _ in 0..1000 {
                    let step = Duration::from_millis(20);
                    clock.0.set(clock.0.get() + step);
                    bus.advance(step);
                    let receipt = governor.command(&goal).unwrap_or_else(|error| {
                        panic!("{calibration:?} at share {share}: {error}")
                    });
                    if receipt
                        .observed
                        .iter()
                        .zip(goal)
                        .all(|(observed, goal)| (observed - goal).abs() < 0.01)
                    {
                        reached = true;
                        break;
                    }
                }
                assert!(
                    reached,
                    "{calibration:?} at share {share} never reached {goal:?}"
                );
            }
        }
    }
}
