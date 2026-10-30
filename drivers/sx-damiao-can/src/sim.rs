//! A simulated Damiao CAN bus answering as the vendor's firmware does (feature `sim`).
//!
//! Each motor answers on its master id: a register write is echoed and held, a register read
//! answers what was written, else its model's limit table, a refresh or command answers feedback at the position last
//! commanded, moving at the velocity that command planned (its MIT `v_des`), and enable and
//! disable set the state its feedback reports. A bus built [`Sim::with_mit_dynamics`] instead
//! runs each motor's MIT law on a loaded joint in wall time: `J a = clamp(kp (p_des - q) +
//! kd (v_des - v) + tau_ff, +-peak) - b v`, integrated at 1 ms between frames, with no torque
//! once disabled or after its `TIMEOUT` lapses, so lag, coast, overshoot and ringing show up. A killed motor never
//! answers. A fault code is latched: enable and disable leave it, and only the clear frame
//! (`FF..FB`) clears it, to disabled, unless it is persistent. Waits are recorded rather than
//! slept unless `real` is set, as on the bench, and they are the bus's clock: a wait for an
//! answer that never comes advances it to the deadline. An enabled motor whose `TIMEOUT`
//! register holds a count and that hears nothing for longer than it latches fault 0xD,
//! communication lost, as Damiao's manual has the firmware do.
//! Shared by every suite that drives a Damiao body: this crate's, the bodies crate's and the
//! station's.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::io;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use sx_embodiment_drivers::can::{CanFrame, CanPort};

use crate::chain::{COMMUNICATION_LOST, DamiaoAxis, motor_type};
use crate::dm_can::{DmMotorType, float_to_uint, uint_to_float};

/// One simulated motor.
pub struct SimMotor {
    pub master: u16,
    pub limits: [f64; 3],
    pub state: u8,
    pub q: f64,
    /// The velocity the last MIT command planned (`v_des`), which feedback reports.
    pub v: f64,
    /// Register values written to the motor, by register id, as their four data bytes.
    pub registers: BTreeMap<u8, [u8; 4]>,
    /// Registers whose writes are echoed but not held, as firmware without them answers.
    pub ignored: BTreeSet<u8>,
    /// Whether feedback carries the enable state; firmware that does not answers state 1
    /// enabled or disabled ([`DamiaoAxis::reports_enable_state`]). A fault code is reported
    /// either way.
    pub reports_enable_state: bool,
    /// A fault the clear frame (`FF..FB`) does not clear, as a fault whose cause persists.
    pub persistent_fault: bool,
    /// The bus clock when the motor last heard a frame addressed to it.
    pub heard: Duration,
    /// The motor's model, which its loaded dynamics are chosen by.
    pub model: DmMotorType,
    /// The MIT law on a loaded joint ([`Sim::with_mit_dynamics`]); `None` jumps to each target.
    pub dynamics: Option<MitDynamics>,
    /// The joint's hard stops in the motor's coordinates, where its axis declares a range
    /// ([`Sim::for_axes`]): the dynamics never pass them.
    pub stops: Option<[f64; 2]>,
}

/// One motor's MIT law on the joint it drives, in the motor's own coordinates.
#[derive(Clone, Copy, Debug)]
pub struct MitDynamics {
    /// The joint's inertia at the motor and its viscous friction, kg m^2 and N m s/rad.
    pub inertia: f64,
    pub damping: f64,
    /// Its Coulomb (static and sliding) friction, N m: a joint at rest sticks until the torque
    /// on it exceeds this.
    pub coulomb: f64,
    /// A constant load along the motor's positive direction, N m (gravity at a pose).
    pub load: f64,
    /// Held where it rests, as a body on its stand or in a person's hands, until the motor is
    /// first enabled: a simulated body placed at a pose (`act-sim`'s commissioned Ready) is
    /// found there. From the first enable it carries its load, and a disabled motor lets it
    /// fall as a real one does.
    pub supported: bool,
    /// The most torque the motor gives, N m.
    pub peak: f64,
    /// The last MIT frame: gains, position, planned velocity and feed-forward torque.
    pub kp: f64,
    pub kd: f64,
    pub p_des: f64,
    pub v_des: f64,
    pub tau_ff: f64,
    /// The joint's velocity.
    pub velocity: f64,
    /// Integrated up to here, and the last frame heard, on the dynamics clock
    /// ([`SimState::dynamics_now`]).
    pub at: Option<Duration>,
    pub heard: Option<Duration>,
}

/// How far past its commandable window a simulated joint's hard stop is, rad at the motor.
const STOP_MARGIN: f64 = 0.15;

/// The integration step of [`MitDynamics`].
const MIT_STEP: f64 = 0.001;

impl MitDynamics {
    /// The static upper body's worst loaded joint per motor model, at 2.61 times the
    /// description's mass (`sx_embodiments.calibrate.motion`), its viscous friction and peak
    /// torque: J8009P shoulders 0.43 kg m^2, DM4340 elbows 0.124, DM4310 wrists 0.0175 plus
    /// rotor. An estimate, as the acceleration model is.
    #[must_use]
    pub fn loaded(model: DmMotorType) -> Self {
        // Coulomb friction per model as `tools/sim_joint_tracking.py`'s BAM drive models.
        let (inertia, damping, coulomb, peak) = match model {
            DmMotorType::DM8009 | DmMotorType::DM8006 | DmMotorType::DM10010 => {
                (0.43, 0.4, 1.2, 40.0)
            }
            DmMotorType::DM4340 | DmMotorType::DM4340_48V => (0.124, 0.3, 1.4, 27.0),
            _ => (0.02, 0.05, 0.15, 7.0),
        };
        Self {
            inertia,
            damping,
            coulomb,
            load: 0.0,
            supported: true,
            peak,
            kp: 0.0,
            kd: 0.0,
            p_des: 0.0,
            v_des: 0.0,
            tau_ff: 0.0,
            velocity: 0.0,
            at: None,
            heard: None,
        }
    }
}

impl SimMotor {
    /// Integrate the motor's MIT law up to `now`; past its armed `TIMEOUT` without a frame it
    /// latches communication lost and gives no more torque.
    fn integrate(&mut self, now: Duration) {
        let timeout = self
            .registers
            .get(&TIMEOUT)
            .map(|bytes| u32::from_le_bytes(*bytes))
            .filter(|counts| *counts > 0)
            .map(|counts| TIMEOUT_COUNT * counts);
        let Some(dynamics) = self.dynamics.as_mut() else {
            return;
        };
        let Some(mut at) = dynamics.at else {
            dynamics.at = Some(now);
            return;
        };
        let step = Duration::from_secs_f64(MIT_STEP);
        if dynamics.supported && self.state != 1 {
            dynamics.at = Some(now);
            return;
        }
        dynamics.supported = false;
        while at + step <= now {
            if self.state == 1
                && let (Some(timeout), Some(heard)) = (timeout, dynamics.heard)
                && at.saturating_sub(heard) > timeout
            {
                self.state = COMMUNICATION_LOST;
            }
            let torque = if self.state == 1 {
                (dynamics.kp * (dynamics.p_des - self.q)
                    + dynamics.kd * (dynamics.v_des - dynamics.velocity)
                    + dynamics.tau_ff)
                    .clamp(-dynamics.peak, dynamics.peak)
            } else {
                0.0
            };
            let drive = torque + dynamics.load - dynamics.damping * dynamics.velocity;
            if dynamics.velocity.abs() < 1e-4 && drive.abs() <= dynamics.coulomb {
                // Stuck: static friction holds it.
                dynamics.velocity = 0.0;
            } else {
                let direction = if dynamics.velocity.abs() < 1e-4 {
                    drive.signum()
                } else {
                    dynamics.velocity.signum()
                };
                let before = dynamics.velocity;
                dynamics.velocity +=
                    (drive - dynamics.coulomb * direction) / dynamics.inertia * MIT_STEP;
                // Friction stops a joint; it never reverses it.
                if before != 0.0
                    && before.signum() != dynamics.velocity.signum()
                    && drive.abs() <= dynamics.coulomb
                {
                    dynamics.velocity = 0.0;
                }
            }
            self.q += dynamics.velocity * MIT_STEP;
            if let Some([lower, upper]) = self.stops
                && !(lower..=upper).contains(&self.q)
            {
                // A hard stop: the joint stays at it.
                self.q = self.q.clamp(lower, upper);
                dynamics.velocity = 0.0;
            }
            at += step;
        }
        dynamics.at = Some(at);
    }
}

/// The `TIMEOUT` register (0x09) and its 50 µs count.
const TIMEOUT: u8 = 0x09;
const TIMEOUT_COUNT: Duration = Duration::from_micros(50);

impl SimMotor {
    /// Latch communication lost if the motor is enabled, its watchdog armed, and it has heard
    /// nothing for longer than the watchdog by `clock`.
    fn watch(&mut self, clock: Duration) {
        let Some(counts) = self
            .registers
            .get(&TIMEOUT)
            .map(|bytes| u32::from_le_bytes(*bytes))
        else {
            return;
        };
        if self.state == 1
            && counts > 0
            && clock.saturating_sub(self.heard) > TIMEOUT_COUNT * counts
        {
            self.state = COMMUNICATION_LOST;
        }
    }
}

/// The bus's motors and what crossed it.
/// What loads each simulated joint (gravity at the body's pose, from a dynamics model): given
/// every motor's position by bus id, writes each motor's load (N m, along its positive
/// direction, at the motor) by bus id. [`Sim::with_load_model`].
pub type LoadModel = Box<dyn FnMut(&BTreeMap<u16, f64>, &mut BTreeMap<u16, f64>) + Send>;

#[derive(Default)]
pub struct SimState {
    /// The model each motor's load is taken from at every frame, when one is given.
    pub load_model: Option<LoadModel>,
    /// The clock the motors' MIT dynamics run on: wall time since the bus was made, or, once
    /// [`Sim::with_manual_time`] is set, only what [`Sim::advance_time`] adds.
    pub dynamics_epoch: Option<Instant>,
    pub manual_time: Option<Duration>,
    pub motors: BTreeMap<u16, SimMotor>,
    pub dead: BTreeSet<u16>,
    pub inbox: VecDeque<CanFrame>,
    pub sent: usize,
    /// Every frame sent, in order.
    pub frames: Vec<CanFrame>,
    pub waited: Duration,
    /// Waits really sleep, as on the bench.
    pub real: bool,
    /// Link failures still to come: each send or receive takes the next and fails with it,
    /// as a failing USB adapter does, sending or reading nothing.
    pub failures: VecDeque<io::ErrorKind>,
}

impl SimState {
    /// Every motor's load from the load model at the body's present pose.
    fn refresh_loads(&mut self) {
        let Some(model) = self.load_model.as_mut() else {
            return;
        };
        let positions: BTreeMap<u16, f64> = self
            .motors
            .iter()
            .map(|(id, motor)| (*id, motor.q))
            .collect();
        let mut loads = BTreeMap::new();
        model(&positions, &mut loads);
        for (id, load) in loads {
            if let Some(dynamics) = self.motors.get_mut(&id).and_then(|m| m.dynamics.as_mut()) {
                dynamics.load = load;
            }
        }
    }

    /// Now on the dynamics clock: the manual time, or wall time since the bus was first asked.
    pub fn dynamics_now(&mut self) -> Duration {
        match self.manual_time {
            Some(now) => now,
            None => self
                .dynamics_epoch
                .get_or_insert_with(Instant::now)
                .elapsed(),
        }
    }

    /// Advance the bus clock by `duration`, tripping every watchdog it passes.
    fn advance(&mut self, duration: Duration) {
        self.waited += duration;
        let clock = self.waited;
        for motor in self.motors.values_mut() {
            motor.watch(clock);
        }
    }
}

/// A simulated bus, shared with the test through its handle.
#[derive(Clone)]
pub struct Sim(pub Arc<Mutex<SimState>>);

impl Sim {
    /// A bus of `(bus id, master id, model)` motors.
    #[must_use]
    pub fn of(motors: &[(u16, u16, DmMotorType)]) -> Self {
        let motors = motors
            .iter()
            .map(|&(id, master, model)| {
                (
                    id,
                    SimMotor {
                        master,
                        limits: model.limit_param(),
                        state: 0,
                        q: 0.0,
                        v: 0.0,
                        registers: BTreeMap::new(),
                        ignored: BTreeSet::new(),
                        reports_enable_state: true,
                        persistent_fault: false,
                        heard: Duration::ZERO,
                        model,
                        dynamics: None,
                        stops: None,
                    },
                )
            })
            .collect();
        Self(Arc::new(Mutex::new(SimState {
            motors,
            ..SimState::default()
        })))
    }

    /// The bus a chain of `axes` drives, each motor of its bound model and reporting its enable
    /// state as the axis says its firmware does.
    ///
    /// # Panics
    ///
    /// When an axis binds a model this crate does not drive.
    #[must_use]
    pub fn for_axes(axes: &[DamiaoAxis]) -> Self {
        let sim = Self::of(
            &axes
                .iter()
                .map(|axis| {
                    (
                        axis.axis.binding.bus_id,
                        axis.master_id,
                        motor_type(axis.axis.binding.model).expect("a Damiao model"),
                    )
                })
                .collect::<Vec<_>>(),
        );
        for axis in axes {
            if let Some(motor) = sim.state().motors.get_mut(&axis.axis.binding.bus_id) {
                motor.reports_enable_state = axis.reports_enable_state;
                // The hard stops a little past the commandable window, where the joint's
                // mechanical stop is.
                motor.stops = Some([
                    axis.motor_lower - STOP_MARGIN,
                    axis.motor_upper + STOP_MARGIN,
                ]);
            }
        }
        sim
    }

    /// The reBot B601-DM's motors among `ids`: DM4340P on 0x01..0x03 and DM4310 on 0x04..0x07,
    /// each answering on its id + [`crate::b601::MASTER_ID_OFFSET`].
    #[must_use]
    pub fn b601(ids: &[u16]) -> Self {
        Self::of(
            &ids.iter()
                .map(|id| {
                    let model = if *id <= 3 {
                        DmMotorType::DM4340
                    } else {
                        DmMotorType::DM4310
                    };
                    (*id, id + crate::b601::MASTER_ID_OFFSET, model)
                })
                .collect::<Vec<_>>(),
        )
    }

    /// Every DM4340 answering the older firmware's `VMAX` row, `v_max` rad/s.
    #[must_use]
    // A model's table row is matched exactly: the simulated register holds its literal.
    #[allow(clippy::float_cmp)]
    pub fn with_dm4340_vmax(self, v_max: f64) -> Self {
        for motor in self.state().motors.values_mut() {
            if motor.limits == DmMotorType::DM4340.limit_param() {
                motor.limits[1] = v_max;
            }
        }
        self
    }

    /// Every motor running its MIT law on its model's loaded joint ([`MitDynamics::loaded`])
    /// instead of jumping to each target.
    #[must_use]
    pub fn with_mit_dynamics(self) -> Self {
        for motor in self.state().motors.values_mut() {
            motor.dynamics = Some(MitDynamics::loaded(motor.model));
        }
        self
    }

    /// Every motor running its MIT law on the joint its body's own model describes: the
    /// inertia the bindings carry for its axis (the registry's estimate, at the motor through
    /// the drive's gain squared), no friction (a description carries none; a load model adds
    /// gravity), and its model's peak torque. The body a station believes it drives, so its
    /// feed-forward cancels what it models; friction is the bench simulation's
    /// ([`Sim::with_mit_dynamics`], `tools/sim_speed_ceiling.py`).
    #[must_use]
    pub fn with_model_dynamics(self, axes: &[DamiaoAxis]) -> Self {
        {
            let mut state = self.state();
            for axis in axes {
                let Some(motor) = state.motors.get_mut(&axis.axis.binding.bus_id) else {
                    continue;
                };
                let gain = axis.axis.actuator_from_joint(axis.axis.zero + 1.0);
                let mut dynamics = MitDynamics::loaded(motor.model);
                if axis.axis.motion.inertia > 0.0 {
                    dynamics.inertia = axis.axis.motion.inertia / (gain * gain);
                }
                dynamics.coulomb = 0.0;
                dynamics.damping = 0.0;
                motor.dynamics = Some(dynamics);
            }
        }
        self
    }

    /// Each motor loaded by `model` at the body's pose, refreshed at every frame: gravity from
    /// the same dynamics model a station's feed-forward uses, so the simulated body is the one
    /// the station believes it drives.
    #[must_use]
    pub fn with_load_model(self, model: LoadModel) -> Self {
        self.state().load_model = Some(model);
        self
    }

    /// Integrate every motor's MIT law up to now, with no frame sent: what a stalled host
    /// leaves the motors doing.
    pub fn settle(&self) {
        let mut state = self.state();
        state.refresh_loads();
        let now = state.dynamics_now();
        for motor in state.motors.values_mut() {
            motor.integrate(now);
        }
    }

    /// The motors' dynamics run only on time [`Sim::advance_time`] adds, not the wall clock:
    /// a suite moves time as its governor's clock moves, with no sleeps.
    #[must_use]
    pub fn with_manual_time(self) -> Self {
        self.state().manual_time = Some(Duration::ZERO);
        self
    }

    /// Move the manual dynamics clock on by `by` and integrate every motor to it.
    ///
    /// # Panics
    ///
    /// When the bus is not on manual time ([`Sim::with_manual_time`]).
    pub fn advance_time(&self, by: Duration) {
        let mut state = self.state();
        let now = state.manual_time.expect("a bus on manual time") + by;
        state.manual_time = Some(now);
        state.refresh_loads();
        for motor in state.motors.values_mut() {
            motor.integrate(now);
        }
    }

    /// The bus state.
    ///
    /// # Panics
    ///
    /// When a test panicked while holding it.
    pub fn state(&self) -> std::sync::MutexGuard<'_, SimState> {
        self.0.lock().expect("sim")
    }

    /// A frame arriving on the link before anything asks for it, as a late answer does.
    pub fn deliver_late(&self, frame: CanFrame) {
        self.state().inbox.push_back(frame);
    }

    /// Fail the link's next sends and receives with `kinds`, one each, in order.
    pub fn fail_next(&self, kinds: &[io::ErrorKind]) {
        self.state().failures.extend(kinds.iter().copied());
    }

    pub fn kill(&self, id: u16) {
        self.state().dead.insert(id);
    }

    #[must_use]
    pub fn sent(&self) -> usize {
        self.state().sent
    }

    #[must_use]
    pub fn waited(&self) -> Duration {
        self.state().waited
    }

    /// Let `duration` pass on the bus clock with nothing sent, as a stalled host does.
    pub fn pass(&self, duration: Duration) {
        self.state().advance(duration);
    }

    /// The waits recorded since the last take.
    #[allow(clippy::must_use_candidate)]
    pub fn take_waited(&self) -> Duration {
        std::mem::take(&mut self.state().waited)
    }
}

/// A motor's feedback frame at its current position, zero velocity and torque.
///
/// # Panics
///
/// When the motor's position is outside its table or its id exceeds the frame's nibble.
#[must_use]
pub fn feedback(id: u16, motor: &SimMotor) -> CanFrame {
    let [q_max, dq_max, tau_max] = motor.limits;
    let q = float_to_uint(motor.q, -q_max, q_max, 16).expect("q");
    let dq = float_to_uint(motor.v.clamp(-dq_max, dq_max), -dq_max, dq_max, 12).expect("dq");
    let tau = float_to_uint(0.0, -tau_max, tau_max, 12).expect("tau");
    let low = u8::try_from(id & 0x0f).expect("id");
    let state = if motor.reports_enable_state || motor.state > 1 {
        motor.state
    } else {
        1
    };
    CanFrame {
        id: u32::from(motor.master),
        data: [
            (state << 4) | low,
            (q >> 8).to_le_bytes()[0],
            q.to_le_bytes()[0],
            (dq >> 4).to_le_bytes()[0],
            (((dq & 0xf) << 4) | (tau >> 8)).to_le_bytes()[0],
            tau.to_le_bytes()[0],
            31,
            33,
        ],
    }
}

/// The motor a frame addresses.
///
/// # Panics
///
/// When a non-register frame carries an extended id.
#[must_use]
pub fn address(frame: &CanFrame) -> u16 {
    if frame.id == 0x7FF {
        u16::from_le_bytes([frame.data[0], frame.data[1]])
    } else {
        u16::try_from(frame.id).expect("standard id")
    }
}

/// A motor's answer to `frame`, as the vendor's firmware gives it.
pub fn answer(id: u16, motor: &mut SimMotor, frame: &CanFrame) -> Option<CanFrame> {
    let data = frame.data;
    if let Some(dynamics) = motor.dynamics.as_ref() {
        motor.v = dynamics.velocity;
    }
    if frame.id == 0x7FF {
        return match data[2] {
            0x55 => {
                if !motor.ignored.contains(&data[3]) {
                    motor
                        .registers
                        .insert(data[3], [data[4], data[5], data[6], data[7]]);
                }
                Some(CanFrame {
                    id: u32::from(motor.master),
                    data,
                })
            }
            0x33 => {
                let table = match data[3] {
                    21 => motor.limits[0],
                    22 => motor.limits[1],
                    23 => motor.limits[2],
                    _ => 0.0,
                };
                #[allow(clippy::cast_possible_truncation)]
                let value = motor
                    .registers
                    .get(&data[3])
                    .copied()
                    .unwrap_or_else(|| (table as f32).to_le_bytes());
                Some(CanFrame {
                    id: u32::from(motor.master),
                    data: [
                        data[0], data[1], 0x33, data[3], value[0], value[1], value[2], value[3],
                    ],
                })
            }
            0xCC => Some(feedback(id, motor)),
            _ => None,
        };
    }
    match data {
        [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC] if motor.state <= 1 => motor.state = 1,
        [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD] if motor.state <= 1 => motor.state = 0,
        [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB] if !motor.persistent_fault => {
            motor.state = 0;
        }
        [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB..=0xFD] => {}
        _ => {
            let q = (u16::from(data[0]) << 8) | u16::from(data[1]);
            let p_des = f64::from(q) / 65_535.0 * 2.0 * motor.limits[0] - motor.limits[0];
            let v = (u16::from(data[2]) << 4) | (u16::from(data[3]) >> 4);
            let v_des = f64::from(uint_to_float(v, -motor.limits[1], motor.limits[1], 12));
            match motor.dynamics.as_mut() {
                None => {
                    motor.q = p_des;
                    motor.v = v_des;
                }
                Some(dynamics) => {
                    let kp = (u16::from(data[3] & 0x0f) << 8) | u16::from(data[4]);
                    let kd = (u16::from(data[5]) << 4) | (u16::from(data[6]) >> 4);
                    let tau = (u16::from(data[6] & 0x0f) << 8) | u16::from(data[7]);
                    dynamics.kp = f64::from(uint_to_float(kp, 0.0, 500.0, 12));
                    dynamics.kd = f64::from(uint_to_float(kd, 0.0, 5.0, 12));
                    dynamics.tau_ff =
                        f64::from(uint_to_float(tau, -motor.limits[2], motor.limits[2], 12));
                    dynamics.p_des = p_des;
                    dynamics.v_des = v_des;
                    motor.v = dynamics.velocity;
                }
            }
        }
    }
    Some(feedback(id, motor))
}

impl CanPort for Sim {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        let mut state = self.state();
        if let Some(kind) = state.failures.pop_front() {
            return Err(io::Error::from(kind));
        }
        state.sent += 1;
        state.frames.push(*frame);
        let id = address(frame);
        if state.dead.contains(&id) {
            return Ok(());
        }
        let clock = state.waited;
        state.refresh_loads();
        let now = state.dynamics_now();
        if let Some(reply) = state.motors.get_mut(&id).and_then(|motor| {
            motor.watch(clock);
            motor.heard = clock;
            motor.integrate(now);
            if let Some(dynamics) = motor.dynamics.as_mut() {
                dynamics.heard = Some(now);
            }
            answer(id, motor, frame)
        }) {
            state.inbox.push_back(reply);
        }
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        let mut state = self.state();
        if let Some(kind) = state.failures.pop_front() {
            return Err(io::Error::from(kind));
        }
        Ok(state.inbox.pop_front())
    }

    /// An answer if one is pending; else the bus clock runs to `deadline` and nothing came.
    fn receive_until(&mut self, deadline: Instant) -> io::Result<Option<CanFrame>> {
        if let Some(frame) = self.state().inbox.pop_front() {
            return Ok(Some(frame));
        }
        self.sleep(deadline.saturating_duration_since(Instant::now()));
        Ok(self.state().inbox.pop_front())
    }

    fn sleep(&mut self, duration: Duration) {
        let real = {
            let mut state = self.state();
            state.advance(duration);
            // On manual time the motors' dynamics move with the bus's own waits too.
            if let Some(now) = state.manual_time.map(|now| now + duration) {
                state.manual_time = Some(now);
                state.refresh_loads();
                for motor in state.motors.values_mut() {
                    motor.integrate(now);
                }
            }
            state.real
        };
        if real {
            std::thread::sleep(duration);
        }
    }
}

/// The answers a [`Steady`] bus holds before they are read.
const STEADY_INBOX: usize = 256;

/// A simulated bus that allocates nothing once built: the same firmware answers as [`Sim`],
/// its motors owned and its pending answers in a fixed ring, recording no frames and waiting
/// no time. It is what a test measures a control tick's allocations on.
pub struct Steady {
    motors: BTreeMap<u16, SimMotor>,
    inbox: [Option<CanFrame>; STEADY_INBOX],
    head: usize,
    len: usize,
}

impl Steady {
    /// The motors of `sim`, taken as they stand.
    ///
    /// # Panics
    ///
    /// When a test panicked while holding `sim`.
    #[must_use]
    pub fn from(sim: &Sim) -> Self {
        let motors = std::mem::take(&mut sim.state().motors);
        Self {
            motors,
            inbox: [None; STEADY_INBOX],
            head: 0,
            len: 0,
        }
    }
}

impl CanPort for Steady {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        let id = address(frame);
        if let Some(reply) = self
            .motors
            .get_mut(&id)
            .and_then(|motor| answer(id, motor, frame))
        {
            if self.len == STEADY_INBOX {
                return Err(io::Error::from(io::ErrorKind::WouldBlock));
            }
            self.inbox[(self.head + self.len) % STEADY_INBOX] = Some(reply);
            self.len += 1;
        }
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        if self.len == 0 {
            return Ok(None);
        }
        let frame = self.inbox[self.head].take();
        self.head = (self.head + 1) % STEADY_INBOX;
        self.len -= 1;
        Ok(frame)
    }

    fn receive_until(&mut self, _deadline: Instant) -> io::Result<Option<CanFrame>> {
        self.receive()
    }

    fn sleep(&mut self, _duration: Duration) {}
}
