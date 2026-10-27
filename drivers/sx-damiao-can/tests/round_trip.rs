//! The chain pairs each command with its motor's answer and tells the link how long the
//! answer took (`CanPort::answered`): from the moment the link took the command's frame to the
//! moment the answer came off the link, whatever else the phase still had to send, and
//! recorded without allocating on a steady control tick.

use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;
use sx_damiao_can::chain::DamiaoChain;
use sx_damiao_can::sim::{Sim, Steady};
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::can::{CanFrame, CanPort};
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

/// Room for every answer a test sees, so recording one never allocates.
const CAPACITY: usize = 1 << 16;

/// What a test reads back: each answer's own delay on the bus, in the order the answers came
/// off it, and the round trips the chain reported, in the order it reported them.
#[derive(Default)]
struct Log {
    delays: Vec<Duration>,
    reported: Vec<Duration>,
}

/// A bus with a transmit queue one exchange deep: a frame is refused (`WouldBlock`) while the
/// previous command's answer has not been read, so the chain reads answers between its own
/// sends, as a full queue makes it do. Each command's answer comes off the link `delay(n)`
/// after the link took it, `n` counting commands.
struct Measured {
    bus: Steady,
    delay: fn(usize) -> Duration,
    commands: usize,
    pending: Option<(CanFrame, Instant, Duration)>,
    log: Arc<Mutex<Log>>,
}

impl CanPort for Measured {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        if self.pending.is_some() {
            return Err(io::Error::from(io::ErrorKind::WouldBlock));
        }
        let taken = Instant::now();
        self.bus.send(frame)?;
        let delay = (self.delay)(self.commands);
        self.commands += 1;
        if let Some(answer) = self.bus.receive()? {
            self.pending = Some((answer, taken + delay, delay));
        }
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        self.receive_until(Instant::now())
    }

    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        let Some((answer, due, delay)) = self.pending else {
            return Ok(None);
        };
        let until = due.min(deadline);
        while Instant::now() < until {
            std::hint::spin_loop();
        }
        if due > deadline {
            return Ok(None);
        }
        self.pending = None;
        self.log.lock().expect("log").delays.push(delay);
        Ok(Some(answer))
    }

    fn sleep(&mut self, _duration: Duration) {}

    /// Long enough that a loaded test machine never makes a refused frame give up: every
    /// answer comes, so no phase waits this out.
    fn frame_allowance(&self) -> Duration {
        Duration::from_secs(1)
    }

    fn answered(&mut self, round_trip: Duration) {
        self.log.lock().expect("log").reported.push(round_trip);
    }
}

fn measured(delay: fn(usize) -> Duration) -> (DamiaoChain<Measured>, Arc<Mutex<Log>>, usize) {
    let axes = STATIC_UPPER_BODY.axes(&drive()).expect("axes");
    let width = axes.len();
    let log = Arc::new(Mutex::new(Log {
        delays: Vec::with_capacity(CAPACITY),
        reported: Vec::with_capacity(CAPACITY),
    }));
    let port = Measured {
        bus: Steady::from(&Sim::for_axes(&axes)),
        delay,
        commands: 0,
        pending: None,
        log: Arc::clone(&log),
    };
    (DamiaoChain::open(port, axes).expect("opens"), log, width)
}

/// Staggered: consecutive commands wait 1.0, 1.3, 1.6, 1.9 and 2.2 ms for their answers.
fn staggered(command: usize) -> Duration {
    Duration::from_micros(1_000 + 300 * (command % 5) as u64)
}

#[test]
fn every_answer_is_timed_from_its_own_command_to_its_own_arrival() {
    let (_chain, log, width) = measured(staggered);
    let log = log.lock().expect("log");
    // Bring-up asks every motor's limit registers, among others; every answer is read while
    // later frames of its phase still wait to be sent, so a round trip stamped once the phase
    // was sent would carry those sends (a millisecond or more each) on top of its own delay.
    assert!(log.reported.len() >= 3 * width, "{}", log.reported.len());
    assert_eq!(
        log.reported.len(),
        log.delays.len(),
        "every answer off the link was paired with its command"
    );
    // Every answer comes no sooner than its own delay. Host scheduling on a loaded machine
    // makes some late (on a busy machine a third); a phase-stamped round trip would make all but each phase's last late
    // by at least one more command's delay (1 ms or more), far past the tolerance.
    let tolerance = Duration::from_micros(700);
    let mut on_time = 0;
    for (index, (reported, delay)) in log.reported.iter().zip(&log.delays).enumerate() {
        assert!(
            reported >= delay,
            "answer {index}: reported {reported:?} for a {delay:?} delay"
        );
        if *reported < *delay + tolerance {
            on_time += 1;
        }
    }
    assert!(width >= 4, "a phase of {width} cannot tell the two apart");
    assert!(
        on_time * 2 >= log.reported.len(),
        "{on_time} of {} answers within {tolerance:?} of their own delay",
        log.reported.len()
    );
}

#[test]
fn a_steady_tick_times_every_answer_without_allocating() {
    let (mut chain, log, width) = measured(|_| Duration::ZERO);
    let target = chain.last_observed().expect("rest").to_vec();
    chain.command(&target).expect("the first command");
    log.lock().expect("log").reported.clear();
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
    assert_eq!(
        log.lock().expect("log").reported.len(),
        200 * width,
        "one answer per motor per tick"
    );
}
