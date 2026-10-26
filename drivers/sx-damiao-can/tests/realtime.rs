//! A steady-state control tick allocates nothing: the static upper body's fifteen Damiao
//! motors, governed, commanded tick after tick on a bus that itself allocates nothing.

use std::time::Duration;

use serde_json::Value;
use sx_damiao_can::chain::DamiaoChain;
use sx_damiao_can::sim::{Sim, Steady};
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::{ActuatorChain, AxisLimits, Feedforward, Governor, InstanceDrive};

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

#[test]
fn a_governed_command_tick_allocates_nothing() {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let width = axes.len();
    let port = Steady::from(&Sim::for_axes(&axes));
    let chain = DamiaoChain::open(port, axes).expect("opens");
    let mut governor = Governor::new(
        chain,
        vec![
            AxisLimits {
                max_speed: 0.25,
                max_tracking_error: 0.2,
            };
            width
        ],
    )
    .expect("limits");
    let mut target = governor.last_observed().expect("rest").to_vec();
    target[3] += 0.05;
    let feedforward = vec![Feedforward::default(); width];
    // The first command arms every watchdog and enables every motor: not a steady tick.
    governor.command(&target).expect("the first command");
    let ticks = allocation_counter::measure(|| {
        for tick in 0..200 {
            let receipt = if tick % 2 == 0 {
                governor.command(&target)
            } else {
                governor.command_with(&target, &feedforward)
            }
            .expect("a steady tick");
            assert_eq!(receipt.observed.len(), width);
            std::thread::sleep(Duration::ZERO);
        }
    });
    // The counter does see an allocation on this path: a read returns its own vector.
    let read = allocation_counter::measure(|| {
        assert_eq!(governor.read_positions().len(), width);
    });
    assert!(read.count_total > 0);
    assert_eq!(
        ticks.count_total, 0,
        "200 command ticks allocated {} times ({} bytes)",
        ticks.count_total, ticks.bytes_total
    );
}
