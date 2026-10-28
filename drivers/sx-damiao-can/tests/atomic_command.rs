//! A motion phase submits each bounded group once; a full queue admits no prefix, and a
//! missing feedback frame stays missing even when the whole phase was admitted together.
use std::io;
use std::num::NonZeroUsize;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde_json::Value;
use sx_damiao_can::chain::DamiaoChain;
use sx_damiao_can::sim::{Sim, Steady};
use sx_damiao_can::static_upper_body::STATIC_UPPER_BODY;
use sx_embodiment_drivers::can::{CanFrame, CanPort, SharedCanPort};
use sx_embodiment_drivers::{ActuatorChain, AxisReading, InstanceDrive};

#[derive(Default)]
struct Log {
    attempts: usize,
    accepted: Vec<Vec<u32>>,
    omit: Option<u32>,
}

struct Batches {
    bus: Steady,
    capacity: NonZeroUsize,
    refuse_attempt: usize,
    log: Arc<Mutex<Log>>,
}

impl CanPort for Batches {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.bus.send(frame)
    }

    fn batch_capacity(&self) -> NonZeroUsize {
        self.capacity
    }

    fn send_batch(&mut self, frames: &[CanFrame]) -> io::Result<()> {
        assert!(frames.len() <= self.capacity.get());
        let mut log = self.log.lock().expect("log");
        log.attempts += 1;
        if log.attempts == self.refuse_attempt {
            return Err(io::ErrorKind::WouldBlock.into());
        }
        log.accepted
            .push(frames.iter().map(|frame| frame.id).collect());
        for frame in frames {
            self.bus
                .send(frame)
                .expect("the simulated bus accepts each frame");
        }
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        while let Some(frame) = self.bus.receive()? {
            if Some(frame.id) != self.log.lock().expect("log").omit {
                return Ok(Some(frame));
            }
        }
        Ok(None)
    }

    fn receive_until(&mut self, _deadline: Instant) -> io::Result<Option<CanFrame>> {
        self.receive()
    }

    fn sleep(&mut self, _duration: Duration) {}
}

#[test]
fn command_groups_are_bounded_and_refusals_never_replay_an_accepted_prefix() {
    let drives: Value = serde_json::from_str(include_str!("fixtures/drives.json")).expect("drives");
    let nominal = &drives["sentient-static-upper-body"]["nominal"];
    let drive = InstanceDrive::parse(
        &nominal.to_string(),
        nominal["instance"].as_str().expect("id"),
    )
    .expect("drive");
    let axes = STATIC_UPPER_BODY.axes(&drive).expect("axes");
    let ids: Vec<_> = axes
        .iter()
        .map(|axis| u32::from(axis.axis.binding.bus_id))
        .collect();
    let bus = Steady::from(&Sim::for_axes(&axes));
    let log = Arc::new(Mutex::new(Log::default()));
    // The same Box + SharedCanPort layers the runtime uses must preserve batch capability.
    let port: Box<dyn CanPort> = Box::new(SharedCanPort::new(Batches {
        bus,
        capacity: NonZeroUsize::new(4).expect("positive"),
        refuse_attempt: 2,
        log: Arc::clone(&log),
    }));
    let mut chain = DamiaoChain::open(port, axes).expect("opens");
    let target = chain.last_observed().expect("rest").to_vec();
    chain.command(&target).expect("command");
    let mut recorded = log.lock().expect("log");
    assert_eq!(
        recorded.accepted.iter().map(Vec::len).collect::<Vec<_>>(),
        [4, 4, 4, 3]
    );
    assert_eq!(recorded.attempts, 5, "only the refused group is retried");
    assert_eq!(
        recorded
            .accepted
            .iter()
            .flatten()
            .copied()
            .collect::<Vec<_>>(),
        ids
    );
    assert!(
        chain
            .command_readings()
            .expect("readings")
            .iter()
            .all(AxisReading::is_fresh)
    );
    recorded.omit = Some(ids[ids.len() - 1] + 0x20);
    drop(recorded);
    let receipt = chain.command(&target).expect("a partly answered phase");
    assert!(receipt.observed[ids.len() - 1].is_nan());
    assert!(!chain.command_readings().expect("readings")[ids.len() - 1].is_fresh());
}
