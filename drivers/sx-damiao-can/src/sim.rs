//! A simulated Damiao CAN bus answering as the vendor's firmware does (feature `sim`).
//!
//! Each motor answers on its master id: a register write is echoed and held, a register read
//! answers what was written, else its model's limit table, a refresh or command answers feedback at the position last
//! commanded, and enable and disable set the state its feedback reports. A killed motor never
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
use crate::dm_can::{DmMotorType, float_to_uint};

/// One simulated motor.
pub struct SimMotor {
    pub master: u16,
    pub limits: [f64; 3],
    pub state: u8,
    pub q: f64,
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
#[derive(Default)]
pub struct SimState {
    pub motors: BTreeMap<u16, SimMotor>,
    pub dead: BTreeSet<u16>,
    pub inbox: VecDeque<CanFrame>,
    pub sent: usize,
    /// Every frame sent, in order.
    pub frames: Vec<CanFrame>,
    pub waited: Duration,
    /// Waits really sleep, as on the bench.
    pub real: bool,
}

impl SimState {
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
                        registers: BTreeMap::new(),
                        ignored: BTreeSet::new(),
                        reports_enable_state: true,
                        persistent_fault: false,
                        heard: Duration::ZERO,
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
    let dq = float_to_uint(0.0, -dq_max, dq_max, 12).expect("dq");
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
            motor.q = f64::from(q) / 65_535.0 * 2.0 * motor.limits[0] - motor.limits[0];
        }
    }
    Some(feedback(id, motor))
}

impl CanPort for Sim {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        let mut state = self.state();
        state.sent += 1;
        state.frames.push(*frame);
        let id = address(frame);
        if state.dead.contains(&id) {
            return Ok(());
        }
        let clock = state.waited;
        if let Some(reply) = state.motors.get_mut(&id).and_then(|motor| {
            motor.watch(clock);
            motor.heard = clock;
            answer(id, motor, frame)
        }) {
            state.inbox.push_back(reply);
        }
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        Ok(self.state().inbox.pop_front())
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
            state.real
        };
        if real {
            std::thread::sleep(duration);
        }
    }
}
