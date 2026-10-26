//! Jog a body through the embodiment layer: relative moves and pauses, each command slewed by
//! the [`Governor`], every tick recorded to a deferred log.
//!
//! A run is a list of [`Step`]s. A move adds its amount to one joint's target, relative to
//! where the run has put it (where the body rested, at the start), and lasts as long as the
//! governor's speed takes to cover it plus a settling margin; a pause holds every target. Every
//! period the target vector goes to the governor, which steps each axis toward it and checks
//! that the body follows. When the steps are done the run ends as its [`AtEnd`] says
//! ([`Governor::end`]: a return under the governor, then the stop); `stop` set during the
//! return stops it where it is. On a refusal or fault, and as soon as `stop` is set during the
//! steps, it stops the body where it is, with no motion.
//!
//! The tick path does no I/O and no formatting: each tick is one fixed-size [`TickRecord`]
//! copied into a preallocated bounded queue ([`TickLog`]), which never blocks (a full queue
//! drops the record and counts it), and a writer thread formats and writes JSON lines. A record
//! holds the target asked for, the command the governor actually sent (each axis slewed toward
//! the target), what the body observed, and each motor's own answer.

use std::io::{self, BufWriter, Write};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::thread::JoinHandle;
use std::time::Duration;

use serde_json::{Map, Value, json};
use sx_damiao_can::chain::{MotorFeedback, MotorState};
use sx_embodiment_drivers::{ActuatorChain, AtEnd, Clock, Ending, Governor};

/// The most axes a [`TickRecord`] holds.
pub const MAX_AXES: usize = 32;

/// How long a move settles after the governor has covered it.
const SETTLE: Duration = Duration::from_millis(400);

/// One step of a run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Step {
    /// Move axis `axis` by `delta` (its unit, rad or m) from where the run has put it.
    Move { axis: usize, delta: f64 },
    /// Move axis `axis` by `delta` together with the next step: it takes no time of its own,
    /// so the governor slews it while the next move runs (written `&joint:degrees`).
    With { axis: usize, delta: f64 },
    /// Hold every target for this long.
    Pause(Duration),
}

/// Parse `joint:degrees` and `pause:seconds` steps, comma-separated, against the body's joint
/// names in native state order; `&joint:degrees` moves with the step after it.
///
/// # Errors
///
/// Names the step that is not one of those or names no joint of the body.
pub fn parse_sequence(sequence: &str, joints: &[String]) -> Result<Vec<Step>, String> {
    sequence
        .split(',')
        .map(|step| {
            let (with, step) = step
                .strip_prefix('&')
                .map_or((false, step), |rest| (true, rest));
            let (name, amount) = step.split_once(':').ok_or(format!("bad step {step}"))?;
            let amount: f64 = amount.parse().map_err(|_| format!("bad step {step}"))?;
            if !amount.is_finite() {
                return Err(format!("bad step {step}"));
            }
            if name == "pause" && !with {
                return Duration::try_from_secs_f64(amount)
                    .map(Step::Pause)
                    .map_err(|_| format!("bad pause {step}"));
            }
            let axis = joints
                .iter()
                .position(|joint| joint == name)
                .ok_or(format!("{name} is no joint of this body"))?;
            let delta = amount.to_radians();
            Ok(if with {
                Step::With { axis, delta }
            } else {
                Step::Move { axis, delta }
            })
        })
        .collect()
}

/// One motor's answer in a tick, as the bus reported it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotorRecord {
    pub bus_id: u16,
    pub state: MotorState,
    pub position: f32,
    pub velocity: f32,
    pub torque: f32,
    pub mos_celsius: u8,
    pub rotor_celsius: u8,
}

const NO_MOTOR: MotorRecord = MotorRecord {
    bus_id: 0,
    state: MotorState::Unavailable,
    position: f32::NAN,
    velocity: f32::NAN,
    torque: f32::NAN,
    mos_celsius: 0,
    rotor_celsius: 0,
};

/// How a tick's command ended.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Outcome {
    Commanded,
    /// Refused or failed; the run stops.
    Failed,
}

/// One tick, fixed-size so the tick path copies it and allocates nothing.
#[derive(Clone, Copy, Debug)]
pub struct TickRecord {
    /// Seconds since the run began.
    pub t: f64,
    /// Axes in use, at most [`MAX_AXES`].
    pub width: u8,
    /// The target asked for.
    pub requested: [f64; MAX_AXES],
    /// What the governor sent, each axis slewed toward its target; `NaN` when nothing went out.
    pub commanded: [f64; MAX_AXES],
    /// Where the body's answers observed each axis; `NaN` where none did.
    pub observed: [f64; MAX_AXES],
    /// Motors that answered, at most [`MAX_AXES`].
    pub motors: u8,
    pub feedback: [MotorRecord; MAX_AXES],
    pub outcome: Outcome,
}

impl TickRecord {
    /// An empty record for `width` axes.
    #[must_use]
    pub fn new(width: usize) -> Self {
        Self {
            t: 0.0,
            width: u8::try_from(width.min(MAX_AXES)).unwrap_or(u8::MAX),
            requested: [f64::NAN; MAX_AXES],
            commanded: [f64::NAN; MAX_AXES],
            observed: [f64::NAN; MAX_AXES],
            motors: 0,
            feedback: [NO_MOTOR; MAX_AXES],
            outcome: Outcome::Commanded,
        }
    }

    fn fill(target: &mut [f64; MAX_AXES], values: Option<&[f64]>) {
        for (slot, value) in target.iter_mut().zip(
            values
                .into_iter()
                .flatten()
                .chain(std::iter::repeat(&f64::NAN)),
        ) {
            *slot = *value;
        }
    }

    fn answers(&mut self, feedback: &[MotorFeedback]) {
        self.motors = u8::try_from(feedback.len().min(MAX_AXES)).unwrap_or(u8::MAX);
        for (slot, answer) in self.feedback.iter_mut().zip(feedback) {
            *slot = MotorRecord {
                bus_id: answer.bus_id,
                state: answer.state,
                position: answer.position,
                velocity: answer.velocity,
                torque: answer.torque,
                mos_celsius: answer.mos_celsius,
                rotor_celsius: answer.rotor_celsius,
            };
        }
    }
}

/// A deferred, never-blocking JSON-lines log of [`TickRecord`]s, written on its own thread.
pub struct TickLog {
    sender: Option<SyncSender<TickRecord>>,
    dropped: Arc<AtomicU64>,
    writer: Option<JoinHandle<io::Result<u64>>>,
}

impl TickLog {
    /// Start the writer thread over `out`, naming each axis by `joints`, behind a queue of
    /// `capacity` records allocated now.
    #[must_use]
    pub fn spawn(out: Box<dyn Write + Send>, joints: Vec<String>, capacity: usize) -> Self {
        let (sender, receiver) = sync_channel(capacity.max(1));
        let writer = std::thread::spawn(move || write_lines(out, &joints, &receiver));
        Self {
            sender: Some(sender),
            dropped: Arc::new(AtomicU64::new(0)),
            writer: Some(writer),
        }
    }

    /// Queue one record: a copy into a preallocated slot, never a wait. A full queue drops it.
    pub fn record(&self, record: &TickRecord) {
        if let Some(sender) = &self.sender
            && let Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) =
                sender.try_send(*record)
        {
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Close the queue, wait for the writer to drain it, and return the lines written and the
    /// records dropped.
    ///
    /// # Errors
    ///
    /// Returns the writer's I/O failure.
    pub fn finish(mut self) -> io::Result<(u64, u64)> {
        self.sender = None;
        let written = match self.writer.take() {
            Some(writer) => writer
                .join()
                .map_err(|_| io::Error::other("the log writer panicked"))??,
            None => 0,
        };
        Ok((written, self.dropped.load(Ordering::Relaxed)))
    }
}

/// Every record as one JSON line, `null` where a value is `NaN`.
fn write_lines(
    out: Box<dyn Write + Send>,
    joints: &[String],
    receiver: &Receiver<TickRecord>,
) -> io::Result<u64> {
    let mut out = BufWriter::new(out);
    let mut written = 0;
    let by_joint = |values: &[f64; MAX_AXES], width: usize| -> Value {
        Value::Object(
            joints
                .iter()
                .zip(values.iter().take(width))
                .map(|(joint, value)| (joint.clone(), json!(value)))
                .collect::<Map<_, _>>(),
        )
    };
    for record in receiver {
        let width = usize::from(record.width);
        let motors: Vec<Value> = record
            .feedback
            .iter()
            .take(usize::from(record.motors))
            .map(|motor| {
                json!({
                    "bus_id": motor.bus_id,
                    "state": state_name(motor.state),
                    "q": motor.position,
                    "dq": motor.velocity,
                    "tau": motor.torque,
                    "mos_c": motor.mos_celsius,
                    "rotor_c": motor.rotor_celsius,
                })
            })
            .collect();
        let line = json!({
            "t": record.t,
            "requested": by_joint(&record.requested, width),
            "commanded": by_joint(&record.commanded, width),
            "observed": by_joint(&record.observed, width),
            "motors": motors,
            "outcome": match record.outcome {
                Outcome::Commanded => "commanded",
                Outcome::Failed => "failed",
            },
        });
        serde_json::to_writer(&mut out, &line)?;
        out.write_all(b"\n")?;
        written += 1;
    }
    out.flush()?;
    Ok(written)
}

fn state_name(state: MotorState) -> String {
    match state {
        MotorState::Disabled => "disabled".to_owned(),
        MotorState::Enabled => "enabled".to_owned(),
        MotorState::Fault(code) => format!("fault {code:#x}"),
        MotorState::Unavailable => "unavailable".to_owned(),
    }
}

/// How a run ended.
#[derive(Debug)]
pub struct Ended {
    /// Why it stopped early, if it did.
    pub failure: Option<String>,
    /// What the body did at the end: the run's [`AtEnd`] when the steps finished, else
    /// [`Ending::Released`], stopped where it was.
    pub ending: Ending,
}

/// Run `steps` on `governor` every `period`, `pace` waiting out each period (a test advances
/// its clock instead), until the steps end or `stop` is set. Steps that finish end as `at_end`
/// says ([`Governor::end`], `stop` aborting a return); a failure or `stop` during the steps
/// stops the body where it is.
///
/// # Errors
///
/// Refuses before any command a body wider than [`MAX_AXES`], a body that reports no rest,
/// and a move of an axis the body does not have.
pub fn run<C, K>(
    governor: &mut Governor<C, K>,
    steps: &[Step],
    period: Duration,
    stop: &AtomicBool,
    log: Option<&TickLog>,
    at_end: AtEnd,
    pace: &mut dyn FnMut(),
) -> Result<Ended, String>
where
    C: ActuatorChain<Feedback = MotorFeedback>,
    K: Clock,
{
    let width = governor.axes().len();
    if width > MAX_AXES {
        return Err(format!("a body of {width} axes is wider than {MAX_AXES}"));
    }
    let mut target = governor
        .last_observed()
        .ok_or("the body reports no rest position")?;
    if let Some(axis) = steps.iter().find_map(|step| match step {
        Step::Move { axis, .. } | Step::With { axis, .. } if *axis >= width => Some(*axis),
        _ => None,
    }) {
        return Err(format!("axis {axis} is not in this body of {width}"));
    }
    let mut record = TickRecord::new(width);
    let mut tick: u32 = 0;
    let mut failure = None;
    let mut interrupted = false;
    'steps: for step in steps {
        let lasts = match *step {
            Step::Move { axis, delta } => {
                target[axis] += delta;
                // The axis's speed in force: the one given, tightened by any derate.
                let speed = governor.limits()[axis].max_speed;
                Duration::from_secs_f64(delta.abs() / speed.max(1e-3)) + SETTLE
            }
            Step::With { axis, delta } => {
                target[axis] += delta;
                Duration::ZERO
            }
            Step::Pause(lasts) => lasts,
        };
        // A move a person typed past where its joint may go is refused, not cut short: the
        // governor would bring it in to the edge and the run would wait out a move it never
        // makes. The rule is the governor's own (`CommandRange::admissible`).
        if let Step::Move { axis, .. } | Step::With { axis, .. } = *step {
            let at = governor.last_observed().map(|at| at[axis]);
            let admissible = governor.commandable()[axis].admissible(at);
            if !admissible.admits(target[axis]) {
                failure = Some(format!(
                    "{} to {:.4} is outside where it may go [{:.4}, {:.4}]",
                    governor.axes()[axis].joint,
                    target[axis],
                    admissible.lower,
                    admissible.upper
                ));
                break 'steps;
            }
        }
        let ticks = lasts.as_nanos().div_ceil(period.as_nanos().max(1));
        for _ in 0..ticks {
            if stop.load(Ordering::SeqCst) {
                interrupted = true;
                break 'steps;
            }
            record.t = period.as_secs_f64() * f64::from(tick);
            tick = tick.saturating_add(1);
            TickRecord::fill(&mut record.requested, Some(&target));
            let result = governor.command(&target);
            TickRecord::fill(&mut record.commanded, governor.sent());
            let receipt = result.as_ref().ok();
            TickRecord::fill(
                &mut record.observed,
                receipt.map(|receipt| receipt.observed.as_slice()),
            );
            record.answers(receipt.map_or(&[], |receipt| receipt.feedback.as_slice()));
            record.outcome = if receipt.is_some() {
                Outcome::Commanded
            } else {
                Outcome::Failed
            };
            if let Some(log) = log {
                log.record(&record);
            }
            if let Err(error) = result {
                failure = Some(format!("command refused or failed: {error}"));
                break 'steps;
            }
            pace();
        }
    }
    let ending = if failure.is_some() || interrupted {
        Ending::Released {
            stop: governor.stop(),
        }
    } else {
        // The moves carried no feed-forward, so neither does the return.
        governor.end(at_end, &[], None, pace, &|| stop.load(Ordering::SeqCst))
    };
    Ok(Ended { failure, ending })
}
