//! A simulated Feetech bus answering as STS3215 firmware does, for the suites of every crate
//! that drives a Feetech body (`sim`).
//!
//! Each servo is a 256-byte register table, its model number the STS3215's and its position
//! limits and homing offset a supplied calibration. An instruction packet (`FF FF id len inst
//! params checksum`) is answered with a status packet: READ with the registers asked for,
//! WRITE by storing and acknowledging, SYNC WRITE (broadcast) by storing each servo's bytes
//! with no answer, SYNC READ by each live servo's status packet in the order asked. A servo with torque on moves toward its goal as simulated time passes
//! ([`SimBus::advance`]), no faster than its `Goal_Speed` (in the vendor's steps of 0.732 rpm;
//! 0, as the firmware reads it, is no limit: at once), the goal clamped into its
//! `Min_Angle_Limit`..`Max_Angle_Limit` as the firmware clamps it (the register keeps what was
//! written; limits of 0..0 clamp nothing); one with torque off rests where it was
//! put (mid-range at first, or [`SimBus::rest_at`]). A servo marked dead answers
//! nothing. The link's packet timeout runs out as
//! soon as nothing more is pending, as a recording port's does. Clones share one bus.
//!
//! A servo answers PING with an empty status packet. A servo may be given hard stops
//! ([`SimBus::stops`]): it never moves past them, and while its goal lies beyond the stop it
//! rests against, its `Present_Load` (60) reads the push, sign-magnitude at bit 10 as
//! Feetech's `SMS_STS::ReadLoad` decodes it (`FTServo_Arduino` `src/SMS_STS.cpp`), in tenths
//! of a percent and never above its `Torque_Limit` (48, 1000 at power-on). Moving freely or
//! limp, it reads 0. [`RealTime`] advances the bus by the wall-clock time between packets, for
//! a caller that waits on real time.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::io;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::serial::SerialPort;

use crate::chain::STS3215_MODEL_NUMBER;
use crate::scservo_sdk::{
    BROADCAST_ID, INST_PING, INST_READ, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE,
    SMS_STS_GOAL_POSITION_L, SMS_STS_GOAL_SPEED_L, SMS_STS_MAX_ANGLE_LIMIT_L,
    SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_MODEL_L, SMS_STS_OFS_L, SMS_STS_PRESENT_POSITION_L,
    SMS_STS_TORQUE_ENABLE,
};

/// Every servo's registers, the servos that answer nothing, and what is waiting to be read.
#[derive(Default)]
pub struct SimState {
    pub servos: BTreeMap<u8, [u8; 256]>,
    pub dead: BTreeSet<u8>,
    pub inbox: VecDeque<u8>,
    /// Every packet written, in order.
    pub written: Vec<Vec<u8>>,
    /// Every byte the host has read.
    pub read: usize,
    /// (servo, address) pairs whose writes the firmware does not take.
    pub ignored: BTreeSet<(u8, u8)>,
    /// Each servo's hard stops, in ticks: it never moves below the first or above the second.
    pub stops: BTreeMap<u8, (i32, i32)>,
    /// The part of a step each moving servo has travelled and not yet shown, so time advanced
    /// in slices shorter than a step still moves it.
    pub carry: BTreeMap<u8, f64>,
}

/// `Torque_Limit`, RAM, in tenths of a percent; the firmware loads it from `Max_Torque_Limit`
/// (16) at power-on (`SMS_STS_TORQUE_LIMIT_L` in `FTServo_Arduino`'s `src/SMS_STS.h`).
const TORQUE_LIMIT: u8 = 48;
/// `Present_Load`, sign-magnitude at bit 10.
const PRESENT_LOAD: u8 = 60;
/// Load, in tenths of a percent, per tick a goal lies past the stop the servo rests against.
const LOAD_PER_TICK: i32 = 10;

impl SimState {
    /// A servo's `Torque_Enable`.
    #[must_use]
    pub fn torque(&self, id: u8) -> u8 {
        self.servos[&id][usize::from(SMS_STS_TORQUE_ENABLE)]
    }

    /// A servo's last `Goal_Position`, as its register holds it.
    #[must_use]
    pub fn goal(&self, id: u8) -> u16 {
        let registers = &self.servos[&id];
        let low = usize::from(SMS_STS_GOAL_POSITION_L);
        u16::from_le_bytes([registers[low], registers[low + 1]])
    }
}

/// A simulated Feetech bus; clones share it.
#[derive(Clone, Default)]
pub struct SimBus(pub Arc<Mutex<SimState>>);

/// The STS register encoding of a homing offset: sign-magnitude at bit 11.
fn homing_register(offset: i16) -> u16 {
    let magnitude = offset.unsigned_abs() & 0x07FF;
    if offset < 0 {
        magnitude | (1 << 11)
    } else {
        magnitude
    }
}

impl SimBus {
    /// STS3215s at each id, holding each calibration, torque off.
    #[must_use]
    pub fn sts3215(servos: &[(u8, MotorCalibration)]) -> Self {
        let mut state = SimState::default();
        for (id, calibration) in servos {
            let mut registers = [0_u8; 256];
            let mut put = |address: u8, value: u16| {
                let [low, high] = value.to_le_bytes();
                registers[usize::from(address)] = low;
                registers[usize::from(address) + 1] = high;
            };
            put(SMS_STS_MODEL_L, STS3215_MODEL_NUMBER);
            put(TORQUE_LIMIT, 1000);
            put(SMS_STS_MIN_ANGLE_LIMIT_L, calibration.range_min);
            put(SMS_STS_MAX_ANGLE_LIMIT_L, calibration.range_max);
            put(SMS_STS_OFS_L, homing_register(calibration.homing_offset));
            put(
                SMS_STS_PRESENT_POSITION_L,
                calibration.range_min.midpoint(calibration.range_max),
            );
            state.servos.insert(*id, registers);
        }
        Self(Arc::new(Mutex::new(state)))
    }

    /// # Panics
    ///
    /// Panics when a holder of the lock panicked.
    pub fn state(&self) -> MutexGuard<'_, SimState> {
        self.0.lock().expect("the simulated bus")
    }

    /// Leave servo `id` resting at `raw` ticks.
    ///
    /// # Panics
    ///
    /// Panics when the bus has no servo `id`.
    pub fn rest_at(&self, id: u8, raw: u16) {
        let mut state = self.state();
        let registers = state.servos.get_mut(&id).expect("a simulated servo");
        let at = usize::from(SMS_STS_PRESENT_POSITION_L);
        registers[at..at + 2].copy_from_slice(&raw.to_le_bytes());
    }

    /// Give servo `id` hard stops at `low` and `high` ticks.
    pub fn stops(&self, id: u8, low: i32, high: i32) {
        self.state().stops.insert(id, (low, high));
    }

    /// A servo's `Present_Load`, decoded: positive pushing toward higher ticks.
    #[must_use]
    pub fn load(&self, id: u8) -> i32 {
        let state = self.state();
        let at = usize::from(PRESENT_LOAD);
        let raw = u16::from_le_bytes([state.servos[&id][at], state.servos[&id][at + 1]]);
        crate::scservo_sdk::ProtocolPacketHandler::<()>::scs_tohost(raw, 10)
    }

    /// Let `elapsed` pass: every servo holding torque moves toward its goal, at most its
    /// `Goal_Speed` times `elapsed`, and no further than its stops.
    pub fn advance(&self, elapsed: Duration) {
        let mut state = self.state();
        let SimState {
            servos,
            stops,
            carry,
            ..
        } = &mut *state;
        for (id, registers) in servos.iter_mut() {
            let load_at = usize::from(PRESENT_LOAD);
            if registers[usize::from(SMS_STS_TORQUE_ENABLE)] != 1 {
                registers[load_at..load_at + 2].copy_from_slice(&[0, 0]);
                continue;
            }
            let read = |registers: &[u8; 256], address: u8| {
                let at = usize::from(address);
                i32::from(u16::from_le_bytes([registers[at], registers[at + 1]]) & 0x7FFF)
            };
            let (min, max) = (
                read(registers, SMS_STS_MIN_ANGLE_LIMIT_L),
                read(registers, SMS_STS_MAX_ANGLE_LIMIT_L),
            );
            let goal = read(registers, SMS_STS_GOAL_POSITION_L);
            let goal = if min < max {
                goal.clamp(min, max)
            } else {
                goal
            };
            let present = read(registers, SMS_STS_PRESENT_POSITION_L);
            let speed = read(registers, SMS_STS_GOAL_SPEED_L);
            let next = if speed == 0 {
                goal
            } else {
                // Steps per second at `speed` steps of 0.732 rpm: 4096 * 0.732 / 60 each.
                let travel = f64::from(speed) * 4096.0 * 0.732 / 60.0 * elapsed.as_secs_f64()
                    + carry.get(id).copied().unwrap_or_default();
                // A sub-step is the servo's whole resolution: it shows a step or nothing, and
                // keeps the rest for the next slice of time.
                #[allow(clippy::cast_possible_truncation)]
                let reach = travel.floor() as i32;
                let fraction = if (goal - present).abs() > reach {
                    travel - travel.floor()
                } else {
                    0.0
                };
                carry.insert(*id, fraction);
                present + (goal - present).clamp(-reach, reach)
            };
            let (next, push) = match stops.get(id) {
                Some(&(low, _)) if next < low => (low, goal - low),
                Some(&(_, high)) if next > high => (high, goal - high),
                _ => (next, 0),
            };
            let limit = read(registers, TORQUE_LIMIT);
            let load = (push.abs() * LOAD_PER_TICK).min(limit);
            let load = u16::try_from(if push < 0 { load | (1 << 10) } else { load }).unwrap_or(0);
            registers[load_at..load_at + 2].copy_from_slice(&load.to_le_bytes());
            let at = usize::from(SMS_STS_PRESENT_POSITION_L);
            let next = u16::try_from(next).unwrap_or(0);
            registers[at..at + 2].copy_from_slice(&next.to_le_bytes());
        }
    }

    /// A servo's `Present_Position`, as its register holds it.
    #[must_use]
    pub fn present(&self, id: u8) -> u16 {
        let state = self.state();
        let at = usize::from(SMS_STS_PRESENT_POSITION_L);
        u16::from_le_bytes([state.servos[&id][at], state.servos[&id][at + 1]])
    }

    /// Power-cycle servo `id` while its link stays up: its RAM registers come back as the
    /// firmware's defaults, torque off, `Acceleration` and `Goal_Speed` 0.
    ///
    /// # Panics
    ///
    /// Panics when the bus has no servo `id`.
    pub fn power_cycle(&self, id: u8) {
        let mut state = self.state();
        let registers = state.servos.get_mut(&id).expect("a simulated servo");
        registers[usize::from(SMS_STS_TORQUE_ENABLE)] = 0;
        registers[usize::from(crate::scservo_sdk::SMS_STS_ACC)] = 0;
        let speed = usize::from(SMS_STS_GOAL_SPEED_L);
        registers[speed..speed + 2].copy_from_slice(&[0, 0]);
    }

    /// Make servo `id` answer nothing from now on.
    pub fn kill(&self, id: u8) {
        self.state().dead.insert(id);
    }
}

fn checksum(bytes: &[u8]) -> u8 {
    !bytes
        .iter()
        .fold(0_u8, |total, byte| total.wrapping_add(*byte))
}

fn status(id: u8, params: &[u8]) -> Vec<u8> {
    let mut body = vec![id, u8::try_from(params.len() + 2).unwrap_or(u8::MAX), 0];
    body.extend_from_slice(params);
    let mut packet = vec![0xFF, 0xFF];
    packet.extend_from_slice(&body);
    packet.push(checksum(&body));
    packet
}

impl SimState {
    fn answer(&mut self, packet: &[u8]) {
        let [0xFF, 0xFF, id, length, instruction, rest @ ..] = packet else {
            return;
        };
        let params = &rest[..usize::from(*length).saturating_sub(2).min(rest.len())];
        if *id == BROADCAST_ID && *instruction == INST_SYNC_READ {
            if let [address, count, servos @ ..] = params {
                for servo in servos {
                    if self.dead.contains(servo) {
                        continue;
                    }
                    if let Some(registers) = self.servos.get(servo) {
                        let start = usize::from(*address);
                        let reply = status(*servo, &registers[start..start + usize::from(*count)]);
                        self.inbox.extend(reply);
                    }
                }
            }
            return;
        }
        if *id == BROADCAST_ID {
            if *instruction == INST_SYNC_WRITE
                && let [address, width, servos @ ..] = params
            {
                for chunk in servos.chunks(usize::from(*width) + 1) {
                    if let [servo, data @ ..] = chunk
                        && let Some(registers) = self.servos.get_mut(servo)
                    {
                        let start = usize::from(*address);
                        registers[start..start + data.len()].copy_from_slice(data);
                    }
                }
            }
            return;
        }
        if self.dead.contains(id) {
            return;
        }
        let Some(registers) = self.servos.get_mut(id) else {
            return;
        };
        let reply = match (*instruction, params) {
            (INST_PING, []) => status(*id, &[]),
            (INST_READ, [address, count]) => {
                let start = usize::from(*address);
                status(*id, &registers[start..start + usize::from(*count)])
            }
            (INST_WRITE, [address, data @ ..]) => {
                if !self.ignored.contains(&(*id, *address)) {
                    let start = usize::from(*address);
                    registers[start..start + data.len()].copy_from_slice(data);
                }
                status(*id, &[])
            }
            _ => return,
        };
        self.inbox.extend(reply);
    }
}

impl SerialPort for SimBus {
    fn clear_port(&mut self) -> io::Result<()> {
        self.state().inbox.clear();
        Ok(())
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        let mut state = self.state();
        state.written.push(packet.to_vec());
        state.answer(packet);
        Ok(packet.len())
    }

    fn read_port(&mut self, length: usize) -> io::Result<Vec<u8>> {
        let mut state = self.state();
        let available = length.min(state.inbox.len());
        state.read += available;
        Ok(state.inbox.drain(..available).collect())
    }

    fn timeout(&self) -> Duration {
        Duration::from_millis(10)
    }

    fn set_packet_timeout(&mut self, _packet_length: usize) {}

    fn is_packet_timeout(&mut self) -> bool {
        self.state().inbox.is_empty()
    }
}

/// A [`SimBus`] that lets the wall-clock time since the last packet pass
/// ([`SimBus::advance`]) before each packet it is written, for a caller that waits in real time.
pub struct RealTime {
    pub bus: SimBus,
    last: Instant,
}

impl RealTime {
    #[must_use]
    pub fn new(bus: SimBus) -> Self {
        Self {
            bus,
            last: Instant::now(),
        }
    }
}

impl SerialPort for RealTime {
    fn clear_port(&mut self) -> io::Result<()> {
        self.bus.clear_port()
    }

    fn write_port(&mut self, packet: &[u8]) -> io::Result<usize> {
        let now = Instant::now();
        self.bus.advance(now - self.last);
        self.last = now;
        self.bus.write_port(packet)
    }

    fn read_port(&mut self, length: usize) -> io::Result<Vec<u8>> {
        self.bus.read_port(length)
    }

    fn timeout(&self) -> Duration {
        self.bus.timeout()
    }

    fn set_packet_timeout(&mut self, packet_length: usize) {
        self.bus.set_packet_timeout(packet_length);
    }

    fn is_packet_timeout(&mut self) -> bool {
        self.bus.is_packet_timeout()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::so101::{SO101, so101};
    use sx_embodiment_drivers::ActuatorChain;

    #[test]
    fn an_so101_opens_commands_and_stops_on_the_simulated_bus() {
        let calibration = MotorCalibration {
            drive_mode: false,
            homing_offset: -12,
            range_min: 800,
            range_max: 3200,
        };
        let bus = SimBus::sts3215(&(1..=6).map(|id| (id, calibration)).collect::<Vec<_>>());
        let mut chain =
            crate::FeetechChain::open(bus.clone(), so101([calibration; 6]).expect("composes"))
                .expect("opens");
        assert_eq!(SO101.servos.len(), 6);
        let rest = chain.last_observed().expect("read at open");
        assert!(
            rest.iter().take(5).all(|joint| joint.abs() < 1e-3),
            "{rest:?}"
        );
        chain.command(&[0.0; 6]).expect("commanded");
        assert_eq!(chain.last_observed(), None);
        assert_eq!(bus.state().torque(1), 1);
        assert_eq!(bus.state().goal(1), 2000);
        assert!(chain.stop().is_proven());
        assert!((1..=6).all(|id| bus.state().torque(id) == 0));
        bus.kill(3);
        assert_eq!(chain.stop().unproven, [3]);
    }

    /// Six servos set up as the static upper body's neck is: with the bring-up approach.
    fn neck_like() -> Vec<crate::FeetechServo> {
        let calibration = MotorCalibration {
            drive_mode: false,
            homing_offset: 0,
            range_min: 800,
            range_max: 3200,
        };
        let mut servos = so101([calibration; 6]).expect("composes");
        for servo in &mut servos {
            servo.configuration.approach = Some(crate::BRING_UP_APPROACH);
        }
        servos
    }

    fn calibrated_bus() -> SimBus {
        let calibration = MotorCalibration {
            drive_mode: false,
            homing_offset: 0,
            range_min: 800,
            range_max: 3200,
        };
        SimBus::sts3215(&(1..=6).map(|id| (id, calibration)).collect::<Vec<_>>())
    }

    #[test]
    fn a_declared_approach_limit_is_written_read_back_and_claimed() {
        let bus = calibrated_bus();
        let chain = crate::FeetechChain::open(bus.clone(), neck_like()).expect("opens");
        assert!((0..6).all(|axis| chain.approach_speed_limited(axis)));
        let state = bus.state();
        assert!(
            state
                .servos
                .values()
                .all(|r| (r[41], r[46], r[47]) == (3, 3, 0))
        );
    }

    #[test]
    fn an_approach_limit_the_servo_does_not_hold_or_cannot_carry_is_refused() {
        let bus = calibrated_bus();
        bus.state()
            .ignored
            .insert((4, crate::scservo_sdk::SMS_STS_GOAL_SPEED_L));
        assert!(matches!(
            crate::FeetechChain::open(bus.clone(), neck_like()),
            Err(crate::FeetechChainError::ApproachUnconfirmed {
                bus_id: 4,
                speed: 0,
                acceleration: 3
            })
        ));
        let mut crawling = neck_like();
        crawling[0].configuration.approach = Some(crate::ApproachLimit {
            speed: 0.01,
            acceleration: 0.5,
        });
        let written = bus.state().written.len();
        assert!(matches!(
            crate::FeetechChain::open(bus.clone(), crawling),
            Err(crate::FeetechChainError::ApproachLimit(1))
        ));
        assert_eq!(bus.state().written.len(), written, "refused before the bus");
    }

    #[test]
    fn a_servo_that_power_cycles_is_unobserved_until_its_limit_and_hold_are_proven_again() {
        let bus = calibrated_bus();
        let mut chain = crate::FeetechChain::open(bus.clone(), neck_like()).expect("opens");
        let zeros = [0.0; 6];
        assert!(chain.command(&zeros).expect("commanded").observed[2].is_finite());
        bus.power_cycle(3);
        // The next command's read finds servo 3 limp and unlimited: NaN, no longer limited.
        let observed = chain.command(&zeros).expect("commanded").observed;
        assert!(observed[2].is_nan());
        assert!(
            observed
                .iter()
                .enumerate()
                .all(|(i, joint)| i == 2 || joint.is_finite())
        );
        assert!(!chain.approach_speed_limited(2));
        // The one after enables afresh: the limit is written again and read back, the servo is
        // held where it rests, and torque comes back on.
        let written = bus.state().written.len();
        let observed = chain.command(&zeros).expect("commanded").observed;
        assert!(observed.iter().all(|joint| joint.is_finite()));
        assert!(chain.approach_speed_limited(2));
        let state = bus.state();
        let registers = &state.servos[&3];
        assert_eq!((registers[40], registers[41], registers[46]), (1, 3, 3));
        let rewrote = state.written[written..]
            .iter()
            .any(|packet| packet[2] == 3 && packet[4..7] == [0x03, 46, 3]);
        assert!(rewrote);
    }

    /// SO-101 servos calibrated narrower than their declared bounds: 1500..2500, about
    /// ±0.77 rad around mid-range, inside every arm joint's declared ±1.65 rad or more.
    fn narrow() -> MotorCalibration {
        MotorCalibration {
            drive_mode: false,
            homing_offset: 0,
            range_min: 1500,
            range_max: 2500,
        }
    }

    #[test]
    fn a_degrees_axis_is_commanded_only_where_its_unit_reaches() {
        let bus = SimBus::sts3215(&(1..=6).map(|id| (id, narrow())).collect::<Vec<_>>());
        let mut chain =
            crate::FeetechChain::open(bus.clone(), so101([narrow(); 6]).expect("composes"))
                .expect("opens");
        let reach = (2500.0 - 2000.0) * std::f64::consts::TAU / 4095.0;
        for (range, axis) in chain.commandable().iter().zip(chain.axes()).take(5) {
            assert!((range.upper - reach).abs() < 1e-9, "{}", axis.joint);
            assert!((range.lower + reach).abs() < 1e-9, "{}", axis.joint);
            assert!(axis.upper > range.upper && axis.lower < range.lower);
        }
        // The gripper's percent spans its calibrated range onto its declared bounds.
        let gripper = &chain.axes()[5];
        assert_eq!(chain.servos()[5].calibrated_reach(), None);
        assert!(chain.commandable()[5].admits(gripper.lower));
        assert!(chain.commandable()[5].admits(gripper.upper));
        // A target in the declared-only band is refused before the bus.
        let written = bus.state().written.len();
        let mut target = [0.0; 6];
        target[1] = 1.0;
        assert!(matches!(
            chain.command(&target),
            Err(crate::FeetechChainError::Target(_))
        ));
        assert_eq!(bus.state().written.len(), written);
    }

    #[test]
    fn an_axis_whose_declared_and_calibrated_ranges_do_not_overlap_is_refused_at_open() {
        let bus = calibrated_bus();
        let mut servos = so101([narrow(); 6]).expect("composes");
        servos[1].axis.lower = 1.0;
        servos[1].axis.upper = 1.5;
        let error = crate::FeetechChain::open(bus.clone(), servos).err();
        assert!(
            matches!(
                &error,
                Some(crate::FeetechChainError::RangeMismatch {
                    joint,
                    declared_lower,
                    calibrated_upper,
                    ..
                }) if joint == "shoulder_lift" && (*declared_lower - 1.0).abs() < 1e-12 && *calibrated_upper < 1.0
            ),
            "{error:?}"
        );
        assert!(bus.state().written.is_empty(), "refused before the bus");
    }

    #[test]
    fn a_servo_resting_outside_its_limits_is_given_the_nearest_one_at_its_confirmed_approach() {
        let bus = calibrated_bus();
        // The shoulder lift rests at tick 3303, 2.0 rad above mid-range and past its 3200
        // limit, as a servo pushed past its stop can read.
        bus.rest_at(2, 3303);
        let mut chain = crate::FeetechChain::open(bus.clone(), neck_like()).expect("opens");
        let rest = chain.last_observed().expect("read at open");
        assert!((rest[1] - 2.0).abs() < 1e-3, "{rest:?}");
        assert!(rest[1] > chain.commandable()[1].upper);
        let before = bus.state().written.len();
        chain.command(&[0.0; 6]).expect("enabled and commanded");
        assert_eq!(
            chain.held_inside(),
            [crate::HeldInside {
                bus_id: 2,
                present: 3303,
                goal: 3200
            }]
        );
        let state = bus.state();
        let written = &state.written[before..];
        let enable = written
            .iter()
            .position(|packet| packet[2] == 2 && packet[4..7] == [0x03, 40, 1])
            .expect("servo 2 enabled");
        let [low, high] = 3200_u16.to_le_bytes();
        assert!(
            written[..enable]
                .iter()
                .any(|packet| packet[2] == 2 && packet[4..8] == [0x03, 42, low, high]),
            "the clamped goal was written before torque came on"
        );
    }

    #[test]
    fn a_servo_resting_outside_its_limits_without_an_approach_limit_refuses_the_enable() {
        let calibration = MotorCalibration {
            drive_mode: false,
            homing_offset: 0,
            range_min: 800,
            range_max: 3200,
        };
        let bus = calibrated_bus();
        bus.rest_at(2, 3303);
        let mut chain =
            crate::FeetechChain::open(bus.clone(), so101([calibration; 6]).expect("composes"))
                .expect("opens");
        let error = chain.command(&[0.0; 6]).expect_err("refused");
        assert!(matches!(
            error,
            crate::FeetechChainError::RestsOutside {
                bus_id: 2,
                present: 3303,
                range_min: 800,
                range_max: 3200
            }
        ));
        assert_eq!(
            sx_embodiment_drivers::ChainError::failure(&error),
            sx_embodiment_drivers::ChainFailure::Refused
        );
        assert!(
            (1..=6).all(|id| bus.state().torque(id) == 0),
            "no torque came on"
        );
    }

    #[test]
    fn a_servo_against_a_stop_stays_there_and_reads_its_push_up_to_its_torque_limit() {
        let bus = calibrated_bus();
        bus.stops(1, 1500, 2100);
        {
            let mut state = bus.state();
            let registers = state.servos.get_mut(&1).expect("servo 1");
            registers[42..44].copy_from_slice(&2150_u16.to_le_bytes());
            registers[40] = 1;
        }
        bus.advance(Duration::from_secs(1));
        assert_eq!((bus.present(1), bus.load(1)), (2100, 500));
        bus.state().servos.get_mut(&1).expect("servo 1")[48..50]
            .copy_from_slice(&300_u16.to_le_bytes());
        bus.state().servos.get_mut(&1).expect("servo 1")[42..44]
            .copy_from_slice(&1400_u16.to_le_bytes());
        bus.advance(Duration::from_secs(1));
        assert_eq!((bus.present(1), bus.load(1)), (1500, -300));
        bus.state().servos.get_mut(&1).expect("servo 1")[40] = 0;
        bus.advance(Duration::from_secs(1));
        assert_eq!(bus.load(1), 0);
    }

    #[test]
    fn the_simulated_firmware_clamps_a_goal_to_its_limit_registers() {
        let bus = calibrated_bus();
        let mut chain = crate::FeetechChain::open(bus.clone(), neck_like()).expect("opens");
        chain.command(&[0.0; 6]).expect("commanded");
        // A goal written straight to the register, past the 3200 limit.
        bus.state().servos.get_mut(&1).expect("servo 1")[42..44]
            .copy_from_slice(&3500_u16.to_le_bytes());
        bus.advance(Duration::from_secs(60));
        assert_eq!(bus.present(1), 3200);
        assert_eq!(
            bus.state().goal(1),
            3500,
            "the register keeps what was written"
        );
    }

    /// The bytes one command puts on the line once torque is on: the `Goal_Position` SYNC
    /// WRITE, the SYNC READ, and every servo's answer to it.
    fn command_bytes(servos: u8) -> usize {
        let calibration = MotorCalibration {
            drive_mode: false,
            homing_offset: 0,
            range_min: 800,
            range_max: 3200,
        };
        let bus = SimBus::sts3215(&(1..=servos).map(|id| (id, calibration)).collect::<Vec<_>>());
        let mut setups = so101([calibration; 6]).expect("composes");
        setups.truncate(usize::from(servos));
        let mut chain = crate::FeetechChain::open(bus.clone(), setups).expect("opens");
        let zeros = vec![0.0; usize::from(servos)];
        chain.command(&zeros).expect("enabled and commanded");
        let (written, read) = {
            let state = bus.state();
            (state.written.len(), state.read)
        };
        let receipt = chain.command(&zeros).expect("commanded");
        assert!(receipt.observed.iter().all(|joint| joint.is_finite()));
        let state = bus.state();
        let sent: usize = state.written[written..].iter().map(Vec::len).sum();
        sent + state.read - read
    }

    #[test]
    fn one_command_costs_ten_bits_a_byte_of_its_write_read_and_answers() {
        // Six servos: a 26-byte SYNC WRITE, a 14-byte SYNC READ of Torque_Enable through
        // Present_Position and six 24-byte answers: 184 bytes, 1.84 ms at 1 Mbaud before any
        // adapter latency. Two servos: 14 + 10 + 48 = 72 bytes, 720 us.
        assert_eq!(command_bytes(6), 184);
        assert_eq!(command_bytes(2), 72);
    }
}
