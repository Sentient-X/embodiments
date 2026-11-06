//! Any number of chains driven as one: a body whose native state order spans several buses.
//!
//! [`Composite`](crate::Composite) is two chains of known types; [`Chains`] is the same rule
//! over a list, for a body whose buses are read from its bindings, each chain usually boxed
//! behind one error and feedback type. The vector is each chain's axes in list order. Every
//! target is checked against every chain's commandable ranges, each widened to where that
//! chain observes the axis now ([`CommandRange::reaching`]), before any chain is written;
//! then **every chain is commanded at once**, each on its own bus: the first on the caller's
//! thread, each other on a thread of its own that lives as long as the body, so a
//! command takes as long as its slowest bus, not the sum of them. On the DGX Spark the static
//! upper body's four buses (the Damiao CAN at 4.0 ms, the neck's Feetech line at 2.0 ms, each
//! gripper's at 1.0 ms, p50) took 8 ms one after the other, and with the solve the 10 ms tick
//! overran to 11.05-11.24 ms, its ticks waking 5.6-5.7 ms late at median (2026-10-28..30
//! sessions). A bus thread takes the caller's real-time policy (`sx_realtime`) the first time
//! it is handed a command from a thread that runs under it, and a steady command allocates
//! nothing.
//!
//! A chain that fails while another may have applied its frames is a [`ChainFailure::Fault`]
//! whatever it reported, since that other device already holds its new target; one that fails
//! alone keeps its own failure (a refusal from every failing chain with nothing applied
//! anywhere stays a refusal). The error names the lowest failing chain. A chain that panics
//! is waited out with the others and its panic resumed on the caller. A stop stops every
//! chain, one after the other, whatever an earlier one answered, and is proven only when every
//! chain's is. Its budget is the sum of theirs.
//!
//! [`ActuatorChain::last_observed`] is every chain's in the same order, or none when any chain
//! has none, so a [`Governor`](crate::Governor) around the whole starts every axis from where it
//! rests or refuses. A receipt's evidence is the weakest any chain established. Its observed
//! joints are every chain's in order, a chain that did not report one for each of its axes
//! contributing `NaN` for each, so a tracking check fails closed on exactly those axes; when
//! no chain observed anything the vector is empty, as for a bus that answers nothing. Its
//! feedback is the chains' in order only when every chain reported one per axis, and empty
//! otherwise.

use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use thiserror::Error;

use crate::axes::Axes;
use crate::binding::BoundAxis;
use crate::chain::{
    ActuatorChain, AxisReading, ChainError, ChainFailure, CommandRange, Evidence, Feedforward,
    FeedforwardLimit, Receipt, StopReport, TargetError, check_reach,
};
use crate::rail::AxisDerate;

/// One chain of the body, shared with the thread that commands it.
type Member<C> = Arc<Mutex<C>>;

/// A member's lock. A thread that panicked inside its chain resumed that panic on the caller;
/// the chain itself is still the one the body holds, so it is used as it stands.
fn lock<C>(member: &Mutex<C>) -> MutexGuard<'_, C> {
    member.lock().unwrap_or_else(PoisonError::into_inner)
}

/// Chains in native state order, driven as one.
pub struct Chains<C: ActuatorChain> {
    members: Vec<Member<C>>,
    /// Per member after the first, the thread its commands run on, started at its first
    /// command.
    crew: Vec<Option<Crew<C>>>,
    axes: Vec<BoundAxis>,
    commandable: Vec<CommandRange>,
    /// Per member, how many axes it carries.
    widths: Vec<usize>,
    /// Per member, its command times on its bus.
    timing: Vec<crate::BusTiming>,
    /// The last hold's readings, every member's in its place (a member whose link failed reads
    /// unavailable); cleared by a command or a stop.
    kept: Option<Axes<AxisReading>>,
    /// When the last command started: the period a bus thread's real-time policy is asked for.
    last_command: Option<Instant>,
}

#[derive(Debug, Error)]
pub enum ChainsError<E> {
    #[error(transparent)]
    Target(#[from] TargetError),
    /// Chain `index` failed; `applied` when another chain may hold its new target.
    #[error("chain {index}: {error}{}", if *applied { " (while another chain applied)" } else { "" })]
    Chain {
        index: usize,
        applied: bool,
        error: E,
    },
}

impl<E: ChainError> ChainError for ChainsError<E> {
    fn failure(&self) -> ChainFailure {
        match self {
            Self::Target(_) => ChainFailure::Refused,
            Self::Chain { applied: true, .. } => ChainFailure::Fault,
            Self::Chain { error, .. } => error.failure(),
        }
    }

    fn motor_faults(&self) -> Vec<(u16, u8)> {
        match self {
            Self::Target(_) => Vec::new(),
            Self::Chain { error, .. } => error.motor_faults(),
        }
    }
}

/// One command for a member's thread: its slice of the targets and feed-forward, and whether
/// the caller runs under the host's real-time policy (with the period it ticks at).
struct Job {
    joints: Axes<f64>,
    feedforward: Option<Axes<Feedforward>>,
    realtime: Option<Duration>,
}

/// What a member's thread answers: its chain's result (or its panic) and how long its bus took.
type Done<C> = (
    std::thread::Result<
        Result<Receipt<<C as ActuatorChain>::Feedback>, <C as ActuatorChain>::Error>,
    >,
    Duration,
);

/// The thread one member's commands run on, for as long as the body lives, and the one slot
/// a command and its answer pass through (a lock and a condition variable: after its first
/// use nothing allocates, where a channel's first waits under contention allocate their
/// waiting contexts). Dropped, it is told to end and joined (it is never mid-command then:
/// every command waits for its answer).
struct Crew<C: ActuatorChain> {
    slot: Arc<Slot<C>>,
    thread: Option<JoinHandle<()>>,
}

/// What passes between the caller and a member's thread.
struct Slot<C: ActuatorChain> {
    state: Mutex<Handoff<C>>,
    changed: Condvar,
}

struct Handoff<C: ActuatorChain> {
    job: Option<Job>,
    done: Option<Done<C>>,
    /// The caller has dropped the body: end.
    closed: bool,
    /// The thread has ended: no answer will come.
    gone: bool,
}

impl<C: ActuatorChain> Slot<C> {
    fn state(&self) -> MutexGuard<'_, Handoff<C>> {
        self.state.lock().unwrap_or_else(PoisonError::into_inner)
    }

    fn wait<'a>(&self, guard: MutexGuard<'a, Handoff<C>>) -> MutexGuard<'a, Handoff<C>> {
        self.changed
            .wait(guard)
            .unwrap_or_else(PoisonError::into_inner)
    }

    /// Hand `job` over; false when the thread has ended.
    fn give(&self, job: Job) -> bool {
        let mut state = self.state();
        if state.gone {
            return false;
        }
        state.job = Some(job);
        drop(state);
        self.changed.notify_all();
        true
    }

    /// The answer to the job handed over, or `None` when the thread ended without one.
    fn answer(&self) -> Option<Done<C>> {
        let mut state = self.state();
        loop {
            if let Some(done) = state.done.take() {
                return Some(done);
            }
            if state.gone {
                return None;
            }
            state = self.wait(state);
        }
    }

    /// The next job, or `None` once the body is dropped.
    fn next(&self) -> Option<Job> {
        let mut state = self.state();
        loop {
            if state.closed {
                return None;
            }
            if let Some(job) = state.job.take() {
                return Some(job);
            }
            state = self.wait(state);
        }
    }

    fn answered(&self, done: Done<C>) {
        self.state().done = Some(done);
        self.changed.notify_all();
    }
}

/// Marks the slot `gone` however the thread ends.
struct Gone<C: ActuatorChain>(Arc<Slot<C>>);

impl<C: ActuatorChain> Drop for Gone<C> {
    fn drop(&mut self) {
        self.0.state().gone = true;
        self.0.changed.notify_all();
    }
}

impl<C: ActuatorChain> Drop for Crew<C> {
    fn drop(&mut self) {
        self.slot.state().closed = true;
        self.slot.changed.notify_all();
        if let Some(thread) = self.thread.take() {
            // A thread that panicked already resumed its panic on a caller.
            let _ = thread.join();
        }
    }
}

/// Command `chain` with `joints` and, when given, `feedforward`, timing its bus.
fn command<C: ActuatorChain>(
    chain: &mut C,
    joints: &[f64],
    feedforward: Option<&[Feedforward]>,
) -> (Result<Receipt<C::Feedback>, C::Error>, Duration) {
    let started = Instant::now();
    let receipt = match feedforward {
        Some(wanted) => chain.command_with(joints, wanted),
        None => chain.command(joints),
    };
    (receipt, started.elapsed())
}

impl<C> Crew<C>
where
    C: ActuatorChain + Send + 'static,
    C::Error: Send,
    C::Feedback: Send,
{
    /// The thread for `member`, named for its first joint.
    fn start(member: Member<C>, name: &str) -> std::io::Result<Self> {
        let slot = Arc::new(Slot {
            state: Mutex::new(Handoff {
                job: None,
                done: None,
                closed: false,
                gone: false,
            }),
            changed: Condvar::new(),
        });
        let gone = Gone(Arc::clone(&slot));
        let thread = std::thread::Builder::new()
            .name(format!("sx-bus-{name}"))
            .spawn(move || {
                let gone = gone;
                let mut elevated = false;
                while let Some(job) = gone.0.next() {
                    if let Some(period) = job.realtime
                        && !elevated
                    {
                        // The caller's policy; refused, the bus runs as the caller's would
                        // have without it, and the caller reports its own refusal.
                        let _ = sx_realtime::elevate(sx_realtime::Role::ControlLoop { period });
                        elevated = true;
                    }
                    let answer = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        let mut chain = lock(&member);
                        command(&mut *chain, &job.joints, job.feedforward.as_deref())
                    }));
                    gone.0.answered(match answer {
                        Ok((receipt, took)) => (Ok(receipt), took),
                        Err(panic) => (Err(panic), Duration::ZERO),
                    });
                }
            })?;
        Ok(Self {
            slot,
            thread: Some(thread),
        })
    }
}

impl<C: ActuatorChain> Chains<C> {
    #[must_use]
    pub fn new(chains: Vec<C>) -> Self {
        let axes = chains
            .iter()
            .flat_map(|chain| chain.axes().iter().cloned())
            .collect();
        let commandable = chains
            .iter()
            .flat_map(|chain| chain.commandable().iter().copied())
            .collect();
        let mut first = 0;
        let timing = chains
            .iter()
            .map(|chain| {
                let timing = crate::BusTiming::new(first);
                first += chain.axes().len();
                timing
            })
            .collect();
        let crew = chains.iter().map(|_| None).collect();
        let widths = chains.iter().map(|chain| chain.axes().len()).collect();
        Self {
            members: chains
                .into_iter()
                .map(|chain| Arc::new(Mutex::new(chain)))
                .collect(),
            crew,
            axes,
            commandable,
            widths,
            timing,
            kept: None,
            last_command: None,
        }
    }

    /// How many chains.
    #[must_use]
    pub fn len(&self) -> usize {
        self.members.len()
    }

    /// Whether there are none.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.members.is_empty()
    }

    /// The chain at `index`, in native state order, held while the guard lives.
    ///
    /// # Panics
    ///
    /// Panics when `index` is not a chain of these.
    pub fn chain(&self, index: usize) -> MutexGuard<'_, C> {
        lock(&self.members[index])
    }

    /// Put `chain` in place of the chain at `index`, which is given back; the axes and their
    /// commanded ranges are composed again.
    ///
    /// # Panics
    ///
    /// Panics when `index` is not a chain of these.
    pub fn replace(&mut self, index: usize, chain: C) -> C {
        let old = std::mem::replace(&mut *lock(&self.members[index]), chain);
        let mut axes = Vec::new();
        let mut commandable = Vec::new();
        let mut widths = Vec::new();
        for member in &self.members {
            let chain = lock(member);
            axes.extend(chain.axes().iter().cloned());
            commandable.extend(chain.commandable().iter().copied());
            widths.push(chain.axes().len());
        }
        self.axes = axes;
        self.commandable = commandable;
        self.widths = widths;
        old
    }

    /// The chains themselves, in native state order, to compose again ([`Chains::new`]) after
    /// some are replaced. Each bus thread is ended first.
    ///
    /// # Panics
    ///
    /// Never: once its thread is joined, nothing else holds a member.
    #[must_use]
    pub fn into_chains(self) -> Vec<C> {
        let Self { members, crew, .. } = self;
        drop(crew);
        members
            .into_iter()
            .map(|member| match Arc::try_unwrap(member) {
                Ok(member) => member.into_inner().unwrap_or_else(PoisonError::into_inner),
                Err(_) => unreachable!("a member is shared only with its joined thread"),
            })
            .collect()
    }
}

impl<C> Chains<C>
where
    C: ActuatorChain + Send + 'static,
    C::Error: Send,
    C::Feedback: Send,
{
    /// Every chain its slice of `joints`, and of `feedforward` when given, all at once.
    fn drive(
        &mut self,
        joints: &[f64],
        feedforward: Option<&[Feedforward]>,
    ) -> Result<Receipt<C::Feedback>, ChainsError<C::Error>> {
        self.kept = None;
        if joints.len() != self.axes.len() {
            return Err(TargetError::Width {
                expected: self.axes.len(),
                actual: joints.len(),
            }
            .into());
        }
        self.check_every_reach(joints)?;
        let handed = self.hand_out(joints, feedforward);
        let mut evidence = Evidence::BusObservedComplete;
        let mut observed = Axes::new();
        let mut feedback = Axes::new();
        let (mut observed_any, mut feedback_whole) = (false, true);
        // The lowest failing chain, and whether any chain may have applied its frames.
        let mut failed: Option<(usize, C::Error)> = None;
        let mut applied = 0_u64;
        let mut panicked = None;
        let mut first = 0;
        for index in 0..self.members.len() {
            let width = self.width(index);
            let span = first..first + width;
            first += width;
            let wanted = feedforward.map(|wanted| &wanted[span.clone()]);
            let (answer, took) = if handed & (1 << index) == 0 {
                let mut chain = lock(&self.members[index]);
                let (receipt, took) = command(&mut *chain, &joints[span], wanted);
                (Ok(receipt), took)
            } else {
                let answer = self.crew[index]
                    .as_ref()
                    .and_then(|crew| crew.slot.answer());
                if let Some(done) = answer {
                    done
                } else {
                    // The thread is gone without an answer; its panic, if any, is in its join.
                    self.crew[index] = None;
                    (
                        Err(Box::new("a bus thread ended mid-command") as _),
                        Duration::ZERO,
                    )
                }
            };
            if let Some(timing) = self.timing.get_mut(index) {
                timing.record(took);
            }
            let receipt = match answer {
                Ok(Ok(receipt)) => receipt,
                Ok(Err(error)) => {
                    if error.failure() != ChainFailure::Refused {
                        applied |= 1 << index;
                    }
                    if failed.is_none() {
                        failed = Some((index, error));
                    }
                    observed.extend(std::iter::repeat_n(f64::NAN, width));
                    continue;
                }
                Err(panic) => {
                    applied |= 1 << index;
                    panicked.get_or_insert(panic);
                    observed.extend(std::iter::repeat_n(f64::NAN, width));
                    continue;
                }
            };
            applied |= 1 << index;
            if receipt.evidence == Evidence::DispatchAttempted {
                evidence = Evidence::DispatchAttempted;
            }
            feedback_whole &= receipt.feedback.len() == width;
            if receipt.observed.len() == width {
                observed_any = true;
                observed.extend(receipt.observed);
            } else {
                // This chain did not observe each of its axes: say so per axis.
                observed.extend(std::iter::repeat_n(f64::NAN, width));
            }
            feedback.extend(receipt.feedback);
        }
        if let Some(panic) = panicked {
            std::panic::resume_unwind(panic);
        }
        if let Some((index, error)) = failed {
            return Err(ChainsError::Chain {
                index,
                applied: applied & !(1 << index) != 0,
                error,
            });
        }
        if !observed_any {
            observed.clear();
        }
        if !feedback_whole {
            feedback.clear();
        }
        Ok(Receipt {
            evidence,
            observed,
            feedback,
        })
    }

    /// Every chain after the first its slice, on its own thread (started at its first
    /// command); which chains were handed theirs, by bit. A chain whose thread cannot be
    /// started or has ended is commanded on the caller's thread instead.
    fn hand_out(&mut self, joints: &[f64], feedforward: Option<&[Feedforward]>) -> u64 {
        let now = Instant::now();
        let realtime = sx_realtime::is_elevated().then(|| {
            self.last_command
                .map_or(Duration::from_millis(10), |last| {
                    now.saturating_duration_since(last)
                })
                .clamp(Duration::from_millis(1), Duration::from_millis(100))
        });
        self.last_command = Some(now);
        // Every chain after the first to its own thread, then the first here.
        let mut first = self.widths.first().copied().unwrap_or(0);
        let mut handed = 0_u64;
        for index in 1..self.members.len() {
            let width = self.width(index);
            let span = first..first + width;
            first += width;
            if self.crew[index].is_none() {
                let name = self
                    .axes
                    .get(span.start)
                    .map_or("chain", |axis| axis.joint.as_str());
                match Crew::start(Arc::clone(&self.members[index]), name) {
                    Ok(crew) => self.crew[index] = Some(crew),
                    // No thread: this chain is commanded here, after the first.
                    Err(_) => continue,
                }
            }
            let job = Job {
                joints: Axes::from_slice(&joints[span.clone()]),
                feedforward: feedforward.map(|wanted| Axes::from_slice(&wanted[span])),
                realtime,
            };
            let sent = self.crew[index]
                .as_ref()
                .is_some_and(|crew| crew.slot.give(job));
            if sent {
                handed |= 1 << index;
            } else {
                self.crew[index] = None;
            }
        }
        handed
    }

    /// Each chain's slice against its ranges widened to where it observes its axes now,
    /// before any chain is written.
    fn check_every_reach(&self, joints: &[f64]) -> Result<(), TargetError> {
        let mut first = 0;
        for member in &self.members {
            let chain = lock(member);
            let width = chain.axes().len();
            let span = first..first + width;
            check_reach(
                chain.axes(),
                &self.commandable[span.clone()],
                chain.last_observed().as_deref(),
                &joints[span],
            )?;
            first += width;
        }
        Ok(())
    }

    /// How many axes the chain at `index` carries.
    fn width(&self, index: usize) -> usize {
        self.widths[index]
    }
}

impl<C> ActuatorChain for Chains<C>
where
    C: ActuatorChain + Send + 'static,
    C::Error: Send,
    C::Feedback: Send,
{
    type Error = ChainsError<C::Error>;
    type Feedback = C::Feedback;

    fn axes(&self) -> &[BoundAxis] {
        &self.axes
    }

    fn commandable(&self) -> &[CommandRange] {
        &self.commandable
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error> {
        self.drive(joints, None)
    }

    fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<Self::Feedback>, Self::Error> {
        if feedforward.len() != self.axes.len() {
            return Err(TargetError::Width {
                expected: self.axes.len(),
                actual: feedforward.len(),
            }
            .into());
        }
        self.drive(joints, Some(feedforward))
    }

    /// Each chain its own slice of `limits`.
    fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), Self::Error> {
        if limits.len() != self.axes.len() {
            return Err(TargetError::Width {
                expected: self.axes.len(),
                actual: limits.len(),
            }
            .into());
        }
        let mut rest = limits;
        for (index, member) in self.members.iter().enumerate() {
            let mut chain = lock(member);
            let (head, tail) = rest.split_at(chain.axes().len());
            rest = tail;
            chain
                .set_feedforward_limits(head)
                .map_err(|error| ChainsError::Chain {
                    index,
                    applied: false,
                    error,
                })?;
        }
        Ok(())
    }

    fn stop(&mut self) -> StopReport {
        self.kept = None;
        let mut report = StopReport::default();
        for member in &self.members {
            report.absorb(lock(member).stop());
        }
        report
    }

    /// Each chain its own slice of `rests`, whatever an earlier one answered.
    fn stop_holding(&mut self, rests: &[crate::Rest]) -> StopReport {
        self.kept = None;
        let mut report = StopReport::default();
        let mut rest = rests;
        for member in &self.members {
            let mut chain = lock(member);
            let (head, tail) = rest.split_at(chain.axes().len().min(rest.len()));
            rest = tail;
            report.absorb(chain.stop_holding(head));
        }
        report
    }

    /// Every chain's holds, one after the other, as one receipt in native state order; a
    /// chain that observes nothing reads `NaN` with default feedback at its axes. A chain whose
    /// link fails never stops the others' holds (the refute of 2026-11-03: one bus's error left
    /// every other bus unheld): its axes read unavailable in [`ActuatorChain::command_readings`],
    /// for the caller's silence rule to release them, and the exchange fails only when every
    /// chain failed.
    fn keep_holding(&mut self) -> Result<Receipt<Self::Feedback>, Self::Error> {
        let mut evidence = Evidence::BusObservedComplete;
        let mut observed = Axes::new();
        let mut feedback = Axes::new();
        let mut readings: Axes<AxisReading> = Axes::new();
        let mut first_failure = None;
        let mut answered = false;
        for (index, member) in self.members.iter().enumerate() {
            let mut chain = lock(member);
            let width = chain.axes().len();
            let receipt = match chain.keep_holding() {
                Ok(receipt) => {
                    answered = true;
                    receipt
                }
                Err(error) => {
                    first_failure.get_or_insert(ChainsError::Chain {
                        index,
                        applied: true,
                        error,
                    });
                    observed.extend((0..width).map(|_| f64::NAN));
                    feedback.extend((0..width).map(|_| C::Feedback::default()));
                    readings.extend(
                        (0..width).map(|_| AxisReading::Missing(crate::MissingReason::Unavailable)),
                    );
                    continue;
                }
            };
            match chain.command_readings() {
                Some(own) if own.len() == width => readings.extend_from_slice(&own),
                _ => readings.extend(
                    (0..width).map(|_| AxisReading::Missing(crate::MissingReason::Unavailable)),
                ),
            }
            if receipt.evidence != Evidence::BusObservedComplete {
                evidence = receipt.evidence;
            }
            if receipt.observed.len() == width {
                observed.extend_from_slice(&receipt.observed);
            } else {
                observed.extend((0..width).map(|_| f64::NAN));
            }
            if receipt.feedback.len() == width {
                feedback.extend_from_slice(&receipt.feedback);
            } else {
                feedback.extend((0..width).map(|_| C::Feedback::default()));
            }
        }
        self.kept = Some(readings);
        if !answered && let Some(failure) = first_failure {
            return Err(failure);
        }
        Ok(Receipt {
            evidence,
            observed,
            feedback,
        })
    }

    /// Every chain's read, in native state order: each chain's axes whatever another answered.
    fn read_positions(&mut self) -> Axes<AxisReading> {
        let mut readings = Axes::new();
        for member in &self.members {
            readings.extend(lock(member).read_positions());
        }
        readings
    }

    /// Every chain's, in native state order, or `None` when any chain has none.
    fn command_readings(&self) -> Option<Axes<AxisReading>> {
        if let Some(kept) = self.kept {
            return Some(kept);
        }
        let mut readings = Axes::new();
        for member in &self.members {
            readings.extend(lock(member).command_readings()?);
        }
        Some(readings)
    }

    /// Every chain's, in order.
    fn read_only_disables(&self) -> Vec<u16> {
        let mut ids = Vec::new();
        for member in &self.members {
            ids.extend(lock(member).read_only_disables());
        }
        ids
    }

    /// Every chain's, shifted to the chain's place in the vector.
    fn unavailable(&self) -> Vec<usize> {
        let mut first = 0;
        let mut unavailable = Vec::new();
        for member in &self.members {
            let chain = lock(member);
            unavailable.extend(chain.unavailable().into_iter().map(|index| first + index));
            first += chain.axes().len();
        }
        unavailable
    }

    /// Every chain's, shifted to the chain's place in the vector.
    fn tracking_ceilings(&self) -> Vec<(usize, crate::chain::TrackingCeiling)> {
        let mut first = 0;
        let mut ceilings = Vec::new();
        for member in &self.members {
            let chain = lock(member);
            ceilings.extend(
                chain
                    .tracking_ceilings()
                    .into_iter()
                    .map(|(axis, speed)| (first + axis, speed)),
            );
            first += chain.axes().len();
        }
        ceilings
    }

    /// Every chain's, shifted to the chain's place in the vector.
    fn derated(&self) -> Vec<AxisDerate> {
        let mut first = 0;
        let mut derated = Vec::new();
        for member in &self.members {
            let chain = lock(member);
            derated.extend(chain.derated().into_iter().map(|derate| AxisDerate {
                axis: first + derate.axis,
                ..derate
            }));
            first += chain.axes().len();
        }
        derated
    }

    /// Every chain's, in native state order, or `None` when any chain has none.
    fn last_observed(&self) -> Option<Axes<f64>> {
        let mut observed = Axes::new();
        for member in &self.members {
            observed.extend(lock(member).last_observed()?);
        }
        Some(observed)
    }

    /// One stop after the other.
    fn stop_budget(&self) -> Duration {
        self.members
            .iter()
            .map(|member| lock(member).stop_budget())
            .fold(Duration::ZERO, Duration::saturating_add)
    }

    fn clear_faults(&mut self) {
        for member in &self.members {
            lock(member).clear_faults();
        }
    }

    fn bus_timing(&self) -> Vec<crate::BusTiming> {
        self.timing.clone()
    }

    fn retried(&self) -> u64 {
        self.members
            .iter()
            .fold(0, |sum, member| sum.saturating_add(lock(member).retried()))
    }
}

/// A boxed chain is its chain, so a list of differently built chains can share one type.
impl<C: ActuatorChain + ?Sized> ActuatorChain for Box<C> {
    type Error = C::Error;
    type Feedback = C::Feedback;

    fn axes(&self) -> &[BoundAxis] {
        (**self).axes()
    }

    fn commandable(&self) -> &[CommandRange] {
        (**self).commandable()
    }

    fn command(&mut self, joints: &[f64]) -> Result<Receipt<Self::Feedback>, Self::Error> {
        (**self).command(joints)
    }

    fn command_with(
        &mut self,
        joints: &[f64],
        feedforward: &[Feedforward],
    ) -> Result<Receipt<Self::Feedback>, Self::Error> {
        (**self).command_with(joints, feedforward)
    }

    fn set_feedforward_limits(&mut self, limits: &[FeedforwardLimit]) -> Result<(), Self::Error> {
        (**self).set_feedforward_limits(limits)
    }

    fn stop(&mut self) -> StopReport {
        (**self).stop()
    }

    fn stop_holding(&mut self, rests: &[crate::Rest]) -> StopReport {
        (**self).stop_holding(rests)
    }

    fn keep_holding(&mut self) -> Result<Receipt<Self::Feedback>, Self::Error> {
        (**self).keep_holding()
    }

    fn stop_budget(&self) -> Duration {
        (**self).stop_budget()
    }

    fn read_positions(&mut self) -> Axes<AxisReading> {
        (**self).read_positions()
    }

    fn last_observed(&self) -> Option<Axes<f64>> {
        (**self).last_observed()
    }

    fn command_readings(&self) -> Option<Axes<AxisReading>> {
        (**self).command_readings()
    }

    fn unavailable(&self) -> Vec<usize> {
        (**self).unavailable()
    }

    fn read_only_disables(&self) -> Vec<u16> {
        (**self).read_only_disables()
    }

    fn derated(&self) -> Vec<AxisDerate> {
        (**self).derated()
    }

    fn tracking_ceilings(&self) -> Vec<(usize, crate::chain::TrackingCeiling)> {
        (**self).tracking_ceilings()
    }

    fn bus_timing(&self) -> Vec<crate::BusTiming> {
        (**self).bus_timing()
    }

    fn retried(&self) -> u64 {
        (**self).retried()
    }

    fn clear_faults(&mut self) {
        (**self).clear_faults();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::binding::{
        ActuatorBinding, ActuatorBus, ActuatorModel, UNCALIBRATED_DIRECTION, UNCALIBRATED_ZERO,
    };

    #[derive(Debug, Error)]
    #[error("{0:?}")]
    struct Failed(ChainFailure);

    impl ChainError for Failed {
        fn failure(&self) -> ChainFailure {
            self.0
        }
    }

    /// A chain that records what it was sent and answers as configured.
    struct Fake {
        axes: Vec<BoundAxis>,
        commandable: Vec<CommandRange>,
        answers: bool,
        fails: Option<ChainFailure>,
        sent: Vec<Vec<f64>>,
        stops: usize,
        unproven: Vec<u16>,
        budget: Duration,
        /// How long a command holds its bus.
        delay: Duration,
        /// A command panics.
        panics: bool,
        /// Whether a command records what it was sent (which allocates).
        records: bool,
    }

    fn fake(port: &str, ids: &[u16], answers: bool) -> Fake {
        let axes: Vec<BoundAxis> = ids
            .iter()
            .map(|id| BoundAxis {
                joint: format!("{port}{id}"),
                port: port.to_owned(),
                lower: -1.0,
                upper: 1.0,
                binding: ActuatorBinding {
                    model: ActuatorModel::new("m"),
                    bus: ActuatorBus::new("b"),
                    bus_id: *id,
                    sign: 1,
                    reduction: 1.0,
                },
                zero: UNCALIBRATED_ZERO,
                direction: UNCALIBRATED_DIRECTION,
                home: None,
                gripper: false,
                motion: crate::binding::AxisMotion {
                    speed: 1.0,
                    acceleration: 10.0,
                    inertia: 0.0,
                },
            })
            .collect();
        let commandable = axes
            .iter()
            .map(|_| CommandRange {
                lower: -0.5,
                upper: 0.5,
            })
            .collect();
        Fake {
            axes,
            commandable,
            answers,
            fails: None,
            sent: Vec::new(),
            stops: 0,
            unproven: Vec::new(),
            budget: Duration::from_millis(10),
            delay: Duration::ZERO,
            panics: false,
            records: true,
        }
    }

    /// A fake answer reports nothing beyond position.
    impl crate::chain::Reports for u16 {
        fn motion(&self) -> Option<(f64, f64)> {
            None
        }

        fn temperature(&self) -> Option<(u16, crate::chain::MotorTemperature)> {
            None
        }
    }

    impl ActuatorChain for Fake {
        type Error = Failed;
        type Feedback = u16;

        fn axes(&self) -> &[BoundAxis] {
            &self.axes
        }

        fn commandable(&self) -> &[CommandRange] {
            &self.commandable
        }

        fn command(&mut self, joints: &[f64]) -> Result<Receipt<u16>, Failed> {
            std::thread::sleep(self.delay);
            assert!(!self.panics, "the bus driver panicked");
            if let Some(failure) = self.fails {
                return Err(Failed(failure));
            }
            if self.records {
                self.sent.push(joints.to_vec());
            }
            Ok(if self.answers {
                Receipt {
                    evidence: Evidence::BusObservedComplete,
                    observed: Axes::from_slice(joints),
                    feedback: self.axes.iter().map(|axis| axis.binding.bus_id).collect(),
                }
            } else {
                Receipt {
                    evidence: Evidence::DispatchAttempted,
                    observed: Axes::new(),
                    feedback: Axes::new(),
                }
            })
        }

        fn stop(&mut self) -> StopReport {
            self.stops += 1;
            StopReport {
                unproven: self.unproven.clone(),
                ..StopReport::default()
            }
        }

        fn stop_budget(&self) -> Duration {
            self.budget
        }

        fn last_observed(&self) -> Option<Axes<f64>> {
            self.answers.then(|| Axes::filled(0.25, self.axes.len()))
        }

        fn read_positions(&mut self) -> Axes<AxisReading> {
            let reading = if self.answers {
                AxisReading::Fresh {
                    position: 0.5,
                    received: std::time::Instant::now(),
                }
            } else {
                AxisReading::Missing(crate::chain::MissingReason::Timeout)
            };
            Axes::filled(reading, self.axes.len())
        }
    }

    #[test]
    fn a_read_keeps_every_chains_axes_and_commands_nothing() {
        let mut chains = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], true)]);
        let read = chains.read_positions();
        assert_eq!(crate::chain::fresh_positions(&read), Some(vec![0.5; 3]));
        assert!((0..chains.len()).all(|index| chains.chain(index).sent.is_empty()));
        // A silent chain is missing on its own axes; every other chain's readings stand.
        let mut partial = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], false)]);
        let read = partial.read_positions();
        assert_eq!(read.len(), 3);
        assert!(read[0].is_fresh() && read[1].is_fresh());
        assert_eq!(
            read[2],
            AxisReading::Missing(crate::chain::MissingReason::Timeout)
        );
        assert_eq!(crate::chain::fresh_positions(&read), None);
    }

    #[test]
    fn targets_split_across_chains_in_order_and_receipts_stay_aligned() {
        let mut chains = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], true)]);
        assert_eq!(chains.axes().len(), 3);
        let receipt = chains.command(&[0.1, 0.2, 0.3]).expect("commanded");
        assert_eq!(receipt.evidence, Evidence::BusObservedComplete);
        assert_eq!(receipt.observed, [0.1, 0.2, 0.3]);
        assert_eq!(receipt.feedback, [1, 2, 1]);
        assert_eq!(chains.chain(1).sent, [[0.3]]);
        // A chain that answers nothing is unobserved on its own axes, NaN, and every other
        // chain's observations stand; feedback, not per axis, is dropped.
        let mut mixed = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], false)]);
        let receipt = mixed.command(&[0.1, 0.2, 0.3]).expect("commanded");
        assert_eq!(receipt.evidence, Evidence::DispatchAttempted);
        assert_eq!(receipt.observed[..2], [0.1, 0.2]);
        assert!(receipt.observed[2].is_nan());
        assert!(receipt.feedback.is_empty());
        let mut silent = Chains::new(vec![fake("can", &[1], false), fake("neck", &[1], false)]);
        assert!(
            silent
                .command(&[0.1, 0.3])
                .expect("commanded")
                .observed
                .is_empty()
        );
    }

    #[test]
    fn a_refused_target_reaches_no_chain_and_a_late_failure_is_a_fault() {
        let mut chains = Chains::new(vec![fake("can", &[1], true), fake("neck", &[1], true)]);
        let refused = chains.command(&[0.1, 0.9]).expect_err("out of range");
        assert_eq!(refused.failure(), ChainFailure::Refused);
        assert!(matches!(
            chains.command(&[0.1]),
            Err(ChainsError::Target(TargetError::Width { .. }))
        ));
        assert!((0..chains.len()).all(|index| chains.chain(index).sent.is_empty()));
        // Every chain is commanded at once: the first failing leaves the second applied, so
        // the body holds a new target somewhere and the failure is a fault.
        let mut first = fake("can", &[1], true);
        first.fails = Some(ChainFailure::Timeout);
        let mut chains = Chains::new(vec![first, fake("neck", &[1], true)]);
        let error = chains.command(&[0.0, 0.0]).expect_err("first fails");
        assert_eq!(error.failure(), ChainFailure::Fault);
        assert!(matches!(
            error,
            ChainsError::Chain {
                index: 0,
                applied: true,
                ..
            }
        ));
        assert_eq!(chains.chain(1).sent, [[0.0]]);
        // Failing alone, a chain keeps its own failure; refused everywhere, nothing applied.
        let mut alone = fake("can", &[1], true);
        alone.fails = Some(ChainFailure::Timeout);
        let mut chains = Chains::new(vec![alone]);
        assert_eq!(
            chains.command(&[0.0]).expect_err("fails").failure(),
            ChainFailure::Timeout
        );
        let (mut a, mut b) = (fake("can", &[1], true), fake("neck", &[1], true));
        a.fails = Some(ChainFailure::Refused);
        b.fails = Some(ChainFailure::Refused);
        let mut chains = Chains::new(vec![a, b]);
        assert_eq!(
            chains.command(&[0.0, 0.0]).expect_err("refused").failure(),
            ChainFailure::Refused
        );
        let mut second = fake("neck", &[1], true);
        second.fails = Some(ChainFailure::Refused);
        let mut chains = Chains::new(vec![fake("can", &[1], true), second]);
        let error = chains.command(&[0.0, 0.0]).expect_err("second fails");
        assert_eq!(error.failure(), ChainFailure::Fault);
    }

    #[test]
    fn a_stop_reaches_every_chain_and_is_proven_only_by_all() {
        let mut stuck = fake("can", &[1, 2], true);
        stuck.unproven = vec![2];
        let mut neck = fake("neck", &[1], false);
        neck.budget = Duration::from_millis(30);
        let mut chains = Chains::new(vec![stuck, neck]);
        assert_eq!(chains.stop_budget(), Duration::from_millis(40));
        let report = chains.stop();
        assert_eq!(report.unproven, [2]);
        assert!((0..chains.len()).all(|index| chains.chain(index).stops == 1));
        // The neck observes nothing here, so the whole has no resting position to start from.
        assert_eq!(chains.last_observed(), None);
        let both = Chains::new(vec![fake("can", &[1, 2], true), fake("neck", &[1], true)]);
        assert_eq!(both.last_observed(), Some(Axes::filled(0.25, 3)));
        let mut boxed: Vec<Box<dyn ActuatorChain<Error = Failed, Feedback = u16> + Send>> = vec![
            Box::new(fake("can", &[1], true)),
            Box::new(fake("neck", &[1], true)),
        ];
        assert!(boxed.iter_mut().all(|chain| chain.stop().is_proven()));
        assert!(Chains::new(boxed).stop().is_proven());
    }

    /// Four buses as the Spark's static upper body has them (CAN 4 ms, neck 2 ms, grippers
    /// 1 ms each): one command takes the slowest bus's time, not the 8 ms sum.
    #[test]
    fn every_bus_is_commanded_at_once() {
        let delays = [4, 2, 1, 1];
        let members: Vec<Fake> = delays
            .iter()
            .enumerate()
            .map(|(index, ms)| {
                let mut chain = fake(&format!("bus{index}"), &[1, 2], true);
                chain.delay = Duration::from_millis(*ms);
                chain
            })
            .collect();
        let mut chains = Chains::new(members);
        let targets = [0.1; 8];
        chains
            .command(&targets)
            .expect("warm: each bus thread starts");
        let started = std::time::Instant::now();
        for _ in 0..20 {
            let receipt = chains.command(&targets).expect("commanded");
            assert_eq!(receipt.observed, targets);
        }
        let each = started.elapsed() / 20;
        assert!(
            each < Duration::from_micros(6500),
            "a command took {each:?}, the buses one after the other"
        );
        assert!(each >= Duration::from_millis(4), "{each:?}");
        // Each bus's own time is recorded against it.
        let timing = chains.bus_timing();
        assert_eq!(timing.len(), 4);
        assert!(timing.iter().all(|t| t.commands == 21));
        assert!(timing[0].quantile(0.5) >= Duration::from_millis(4));
        assert!(timing[3].quantile(0.5) < Duration::from_millis(4));
        // Handed back whole, every bus thread ended.
        let back = chains.into_chains();
        assert_eq!(back.len(), 4);
        assert!(back.iter().all(|chain| chain.sent.len() == 21));
    }

    /// A bus driver that panics on its own thread is waited out with the others, and its panic
    /// reaches the caller, as it did when every bus ran on the caller's thread.
    #[test]
    fn a_panicking_bus_is_the_callers_panic() {
        let mut second = fake("neck", &[1], true);
        second.panics = true;
        let mut chains = Chains::new(vec![fake("can", &[1], true), second]);
        let caught =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| chains.command(&[0.0, 0.0])));
        assert!(caught.is_err(), "the panic reached the caller");
        assert_eq!(chains.chain(0).sent, [[0.0]], "the other bus was commanded");
        // The body still stops every chain.
        assert!(chains.stop().is_proven());
    }

    /// A steady command allocates nothing on the caller's thread: each bus thread is started
    /// once and handed fixed-size slices.
    #[test]
    fn a_steady_command_allocates_nothing() {
        let members: Vec<Fake> = (0..3)
            .map(|index| {
                let mut chain = fake(&format!("bus{index}"), &[1, 2], true);
                chain.records = false;
                // Slower than the first bus, so the caller waits on each answer.
                chain.delay = Duration::from_micros(if index == 0 { 0 } else { 300 });
                chain
            })
            .collect();
        let mut chains = Chains::new(members);
        let targets = [0.1; 6];
        let feedforward = [Feedforward::default(); 6];
        // Warm: each bus thread starts, and its lock and condition variable are first used.
        for tick in 0..20 {
            if tick % 2 == 0 {
                chains.command(&targets).expect("warm");
            } else {
                chains.command_with(&targets, &feedforward).expect("warm");
            }
        }
        let counted = allocation_counter::measure(|| {
            for tick in 0..200 {
                let receipt = if tick % 2 == 0 {
                    chains.command(&targets)
                } else {
                    chains.command_with(&targets, &feedforward)
                }
                .expect("steady");
                assert_eq!(receipt.observed.len(), 6);
            }
        });
        assert_eq!(counted.count_total, 0, "{counted:?}");
    }
}
