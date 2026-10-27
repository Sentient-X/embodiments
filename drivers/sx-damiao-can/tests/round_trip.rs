//! The chain pairs each command with its motor's answer and tells the link how long the
//! answer took (`CanPort::answered`): measured from the moment the link took the frame, and
//! recorded without allocating on a steady control tick.

use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;
use sx_damiao_can::chain::DamiaoChain;
use sx_damiao_can::sim::{Sim, Steady};
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::can::{CanFrame, CanPort, RoundTrip, RoundTripWindow};
use sx_embodiment_drivers::{ActuatorChain, InstanceDrive};

fn drive() -> InstanceDrive {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("an instance"),
    )
    .expect("a drive")
}

/// A bus whose motors answer no sooner than `delay` after the last frame sent, recording the
/// answer times the chain reports into a window the test reads.
struct Measured {
    bus: Steady,
    delay: Duration,
    last_sent: Option<Instant>,
    window: Arc<Mutex<RoundTripWindow>>,
}

impl Measured {
    fn hold(&self) {
        if let Some(sent) = self.last_sent {
            let until = sent + self.delay;
            while Instant::now() < until {
                std::hint::spin_loop();
            }
        }
    }
}

impl CanPort for Measured {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.last_sent = Some(Instant::now());
        self.bus.send(frame)
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        self.hold();
        self.bus.receive()
    }

    fn receive_until(&mut self, _deadline: Instant) -> io::Result<Option<CanFrame>> {
        self.receive()
    }

    fn sleep(&mut self, _duration: Duration) {}

    fn answered(&mut self, round_trip: Duration) {
        self.window.lock().expect("window").record(round_trip);
    }

    fn round_trip(&mut self) -> Option<RoundTrip> {
        self.window.lock().expect("window").take()
    }
}

fn measured(delay: Duration) -> (DamiaoChain<Measured>, Arc<Mutex<RoundTripWindow>>, usize) {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let width = axes.len();
    let window = Arc::new(Mutex::new(RoundTripWindow::default()));
    let port = Measured {
        bus: Steady::from(&Sim::for_axes(&axes)),
        delay,
        last_sent: None,
        window: Arc::clone(&window),
    };
    (DamiaoChain::open(port, axes).expect("opens"), window, width)
}

#[test]
fn every_answer_is_timed_from_its_own_command() {
    let delay = Duration::from_millis(3);
    let (_chain, window, width) = measured(delay);
    // Bring-up asks every motor's three limit registers: each answer comes `delay` after the
    // phase's last frame, so no earlier than that after its own.
    let taken = window.lock().expect("window").take().expect("answered");
    assert!(taken.answers >= 3 * width as u64, "{taken:?}");
    assert!(taken.p50 >= delay, "{taken:?}");
    assert!(
        taken.max >= taken.p99 && taken.p99 >= taken.p50,
        "{taken:?}"
    );
    assert!(taken.max < Duration::from_secs(1), "{taken:?}");
}

#[test]
fn a_steady_tick_times_every_answer_without_allocating() {
    let (mut chain, window, width) = measured(Duration::ZERO);
    let target = chain.last_observed().expect("rest").to_vec();
    chain.command(&target).expect("the first command");
    window.lock().expect("window").take();
    let ticks = allocation_counter::measure(|| {
        for _ in 0..200 {
            chain.command(&target).expect("a steady tick");
        }
    });
    assert_eq!(
        ticks.count_total, 0,
        "200 timed ticks allocated {} times ({} bytes)",
        ticks.count_total, ticks.bytes_total
    );
    let taken = window.lock().expect("window").take().expect("answered");
    assert_eq!(
        taken.answers,
        200 * width as u64,
        "one answer per motor per tick"
    );
}
