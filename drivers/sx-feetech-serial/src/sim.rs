//! A simulated Feetech bus answering as STS3215 firmware does, for the suites of every crate
//! that drives a Feetech body (`sim`).
//!
//! Each servo is a 256-byte register table, its model number the STS3215's and its position
//! limits and homing offset a supplied calibration. An instruction packet (`FF FF id len inst
//! params checksum`) is answered with a status packet: READ with the registers asked for,
//! WRITE by storing and acknowledging, SYNC WRITE (broadcast) by storing each servo's bytes
//! with no answer. A servo marked dead answers nothing. The link's packet timeout runs out as
//! soon as nothing more is pending, as a recording port's does. Clones share one bus.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
use std::io;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::Duration;

use sx_embodiment_drivers::lerobot::MotorCalibration;
use sx_embodiment_drivers::serial::SerialPort;

use crate::chain::STS3215_MODEL_NUMBER;
use crate::scservo_sdk::{
    BROADCAST_ID, INST_READ, INST_SYNC_WRITE, INST_WRITE, SMS_STS_GOAL_POSITION_L,
    SMS_STS_MAX_ANGLE_LIMIT_L, SMS_STS_MIN_ANGLE_LIMIT_L, SMS_STS_MODEL_L, SMS_STS_OFS_L,
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
}

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
            put(SMS_STS_MIN_ANGLE_LIMIT_L, calibration.range_min);
            put(SMS_STS_MAX_ANGLE_LIMIT_L, calibration.range_max);
            put(SMS_STS_OFS_L, homing_register(calibration.homing_offset));
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
            (INST_READ, [address, count]) => {
                let start = usize::from(*address);
                status(*id, &registers[start..start + usize::from(*count)])
            }
            (INST_WRITE, [address, data @ ..]) => {
                let start = usize::from(*address);
                registers[start..start + data.len()].copy_from_slice(data);
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
        chain.command(&[0.0; 6]).expect("commanded");
        assert_eq!(bus.state().torque(1), 1);
        assert_eq!(bus.state().goal(1), 2000);
        assert!(chain.stop().is_proven());
        assert!((1..=6).all(|id| bus.state().torque(id) == 0));
        bus.kill(3);
        assert_eq!(chain.stop().unproven, [3]);
    }
}
