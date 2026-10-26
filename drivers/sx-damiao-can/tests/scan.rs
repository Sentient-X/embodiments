//! The read-only scan against motors answering as the static upper body's fifteen do.
//!
//! Each motor answers a register read on its master id (its id plus `0x20`) with its model's
//! limit row, as Damiao firmware does; the first exchange is pinned byte for byte.

use std::collections::{BTreeMap, VecDeque};
use std::io;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use sx_damiao_can::dm_can::{DmCanError, DmMotorType, MotorControl};
use sx_damiao_can::scan::{ReadOnlyCanPort, is_read_only, scan};
use sx_embodiment_drivers::can::{CanFrame, CanPort};

const BODY: [(u16, DmMotorType); 15] = [
    (0x1F, DmMotorType::DM8009),
    (0x18, DmMotorType::DM8009),
    (0x19, DmMotorType::DM8009),
    (0x1A, DmMotorType::DM4340),
    (0x1B, DmMotorType::DM4340),
    (0x1C, DmMotorType::DM4340),
    (0x1D, DmMotorType::DM4310),
    (0x1E, DmMotorType::DM4310),
    (0x09, DmMotorType::DM8009),
    (0x10, DmMotorType::DM8009),
    (0x11, DmMotorType::DM4340),
    (0x12, DmMotorType::DM4340),
    (0x13, DmMotorType::DM4340),
    (0x14, DmMotorType::DM4310),
    (0x15, DmMotorType::DM4310),
];

/// Motors answering register reads; every frame sent is logged.
struct Bus {
    motors: BTreeMap<u16, DmMotorType>,
    inbox: VecDeque<CanFrame>,
    log: Arc<Mutex<Vec<CanFrame>>>,
}

#[allow(clippy::cast_possible_truncation)]
fn register(id: u16, model: DmMotorType, rid: u8) -> [u8; 4] {
    let [p, v, t] = model.limit_param();
    match rid {
        7 => u32::from(id + 0x20).to_le_bytes(),
        8 => u32::from(id).to_le_bytes(),
        10 => 1_u32.to_le_bytes(),
        21 => (p as f32).to_le_bytes(),
        22 => (v as f32).to_le_bytes(),
        23 => (t as f32).to_le_bytes(),
        _ => [0; 4],
    }
}

impl CanPort for Bus {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.log.lock().unwrap().push(*frame);
        let data = frame.data;
        let id = u16::from_le_bytes([data[0], data[1]]);
        if frame.id == 0x7FF
            && data[2] == 0x33
            && let Some(&model) = self.motors.get(&id)
        {
            let value = register(id, model, data[3]);
            self.inbox.push_back(CanFrame {
                id: u32::from(id + 0x20),
                data: [
                    data[0], data[1], 0x33, data[3], value[0], value[1], value[2], value[3],
                ],
            });
        }
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        Ok(self.inbox.pop_front())
    }

    fn sleep(&mut self, _duration: Duration) {}
}

fn body() -> (Bus, Arc<Mutex<Vec<CanFrame>>>) {
    let log = Arc::new(Mutex::new(Vec::new()));
    (
        Bus {
            motors: BODY.into_iter().collect(),
            inbox: VecDeque::new(),
            log: Arc::clone(&log),
        },
        log,
    )
}

#[test]
fn the_fifteen_answer_on_their_master_ids() {
    let (port, _) = body();
    let found = scan(port, 0x01..=0x20).unwrap();
    assert_eq!(found.motors.len(), 15);
    assert_eq!(found.silent.len(), 32 - 15);
    for (id, model) in BODY {
        let motor = found.motor(id).unwrap();
        assert_eq!(motor.reply_id, u32::from(id + 0x20));
        assert_eq!(motor.master_id, u32::from(id + 0x20));
        assert_eq!(motor.esc_id, Some(u32::from(id)));
        assert_eq!(motor.control_mode, Some(1));
        assert!(motor.fits(model), "{id:#x} fits {model:?}");
        for other in [
            DmMotorType::DM4310,
            DmMotorType::DM4340,
            DmMotorType::DM8009,
        ] {
            if other != model {
                assert!(!motor.fits(other));
            }
        }
    }
}

#[test]
fn every_frame_is_a_register_read_byte_for_byte() {
    let (port, log) = body();
    scan(port, [0x09, 0x0A]).unwrap();
    let log = log.lock().unwrap();
    assert!(log.iter().all(is_read_only));
    // Id 9: MST_ID (7), ESC_ID (8), CTRL_MODE (10), PMAX, VMAX, TMAX, UV (0), OV (29), KT (1),
    // Gr (20); id 10: MST_ID alone.
    let registers: Vec<(u32, [u8; 8])> = log.iter().map(|frame| (frame.id, frame.data)).collect();
    assert_eq!(
        registers,
        vec![
            (0x7FF, [0x09, 0, 0x33, 7, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 8, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 10, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 21, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 22, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 23, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 0, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 29, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 1, 0, 0, 0, 0]),
            (0x7FF, [0x09, 0, 0x33, 20, 0, 0, 0, 0]),
            (0x7FF, [0x0A, 0, 0x33, 7, 0, 0, 0, 0]),
        ]
    );
}

#[test]
fn the_read_only_port_refuses_enable_and_writes_before_the_link() {
    let (port, log) = body();
    let mut control = MotorControl::new(ReadOnlyCanPort::new(port));
    control.add_motor(sx_damiao_can::dm_can::Motor::new(
        DmMotorType::DM8009,
        0x09,
        0x29,
    ));
    assert!(matches!(control.enable(0x09), Err(DmCanError::Io(_))));
    assert!(matches!(control.disable(0x09), Err(DmCanError::Io(_))));
    assert!(matches!(
        control.set_zero_position(0x09),
        Err(DmCanError::Io(_))
    ));
    assert!(matches!(
        control.switch_control_mode(0x09, sx_damiao_can::dm_can::ControlType::MIT),
        Err(DmCanError::Io(_))
    ));
    assert!(matches!(
        control.control_mit(0x09, 1.0, 0.1, 0.0, 0.0, 0.0),
        Err(DmCanError::Io(_))
    ));
    assert!(log.lock().unwrap().is_empty());
}

#[test]
fn a_reply_for_another_id_is_not_taken() {
    let (mut port, _) = body();
    // Every reply names id 0x09, whatever was asked.
    port.motors.clear();
    port.inbox.push_back(CanFrame {
        id: 0x29,
        data: [0x09, 0, 0x33, 7, 0x29, 0, 0, 0],
    });
    let found = scan(port, [0x10]).unwrap();
    assert_eq!(found.silent, vec![0x10]);
}
