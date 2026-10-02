//! The crate replays what Damiao's own `DM_CAN.py` did, byte for byte and wait for wait.
//!
//! `fixtures/dm_can/transcripts.json` is minted by `fixtures/dm_can/mint.py`, which runs the
//! vendor driver itself; see its header for the sources and the cross-checks.

// Parity is bit-for-bit: every float is compared exactly against the vendor's.
#![allow(clippy::float_cmp)]

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use serde_json::Value;
use sha2::{Digest, Sha256};
use sx_damiao_can::b601::b601;
use sx_damiao_can::chain::{DamiaoChain, DamiaoChainError, MotorFeedback, MotorLimits, MotorState};
use sx_damiao_can::dm_can::{
    DmCanError, DmMotorType, DmVariable, Motor, MotorControl, ParamValue, float_to_uint,
    uint_to_float,
};
use sx_damiao_can::usb2can::Usb2CanPort;
use sx_embodiment_drivers::can::{CanFrame, CanPort};
use sx_embodiment_drivers::{ActuatorChain, Evidence, StopReport};

fn transcripts() -> Value {
    serde_json::from_str(include_str!("fixtures/dm_can/transcripts.json")).expect("transcripts")
}

fn hex(bytes: &[u8]) -> String {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    bytes
        .iter()
        .flat_map(|byte| {
            [
                DIGITS[usize::from(byte >> 4)],
                DIGITS[usize::from(byte & 0xf)],
            ]
        })
        .map(char::from)
        .collect()
}

fn unhex(text: &str) -> Vec<u8> {
    (0..text.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&text[i..i + 2], 16).expect("hex"))
        .collect()
}

fn f64_of(value: &Value) -> f64 {
    value.as_f64().expect("number")
}

fn motor_type(index: &Value) -> DmMotorType {
    match index.as_u64().expect("motor type") {
        0 => DmMotorType::DM4310,
        2 => DmMotorType::DM4340,
        other => panic!("motor type {other} is not in the transcript's vocabulary"),
    }
}

#[test]
fn float_to_uint_is_the_vendors() {
    for row in transcripts()["vectors"]["float_to_uint"]
        .as_array()
        .unwrap()
    {
        let x = row["x"].as_f64().unwrap_or(f64::NAN);
        let bits = u32::try_from(row["bits"].as_u64().unwrap()).unwrap();
        let ours = float_to_uint(x, f64_of(&row["min"]), f64_of(&row["max"]), bits);
        match row.get("result") {
            Some(result) => assert_eq!(u64::from(ours.unwrap()), result.as_u64().unwrap(), "{row}"),
            None => assert!(matches!(ours, Err(DmCanError::NotANumber)), "{row}"),
        }
    }
}

#[test]
fn uint_to_float_is_the_vendors() {
    for row in transcripts()["vectors"]["uint_to_float"]
        .as_array()
        .unwrap()
    {
        let x = u16::try_from(row["x"].as_u64().unwrap()).unwrap();
        let bits = u32::try_from(row["bits"].as_u64().unwrap()).unwrap();
        let ours = uint_to_float(x, f64_of(&row["min"]), f64_of(&row["max"]), bits);
        assert_eq!(f64::from(ours), f64_of(&row["result"]), "{row}");
    }
}

/// A CAN link that keeps what is sent and hands out queued frames.
#[derive(Default)]
struct Capture {
    sent: Vec<CanFrame>,
    inbox: VecDeque<CanFrame>,
}

impl CanPort for Capture {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        self.sent.push(*frame);
        Ok(())
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        Ok(self.inbox.pop_front())
    }

    fn sleep(&mut self, _: Duration) {}
}

#[test]
fn control_mit_frames_are_the_vendors() {
    for row in transcripts()["vectors"]["mit"].as_array().unwrap() {
        let mut control = MotorControl::new(Capture::default());
        control.add_motor(Motor::new(motor_type(&row["motor_type"]), 0x01, 0x11));
        control
            .control_mit(
                0x01,
                f64_of(&row["kp"]),
                f64_of(&row["kd"]),
                f64_of(&row["q"]),
                f64_of(&row["dq"]),
                f64_of(&row["tau"]),
            )
            .expect("frame");
        let sent = &control.port().sent;
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].id, 0x01);
        assert_eq!(hex(&sent[0].data), row["data"].as_str().unwrap(), "{row}");
    }
}

#[test]
fn feedback_decodes_as_the_vendors() {
    for row in transcripts()["vectors"]["decode"].as_array().unwrap() {
        let slave = u16::try_from(row["slave_id"].as_u64().unwrap()).unwrap();
        let mut port = Capture::default();
        let mut data = [0; 8];
        data.copy_from_slice(&unhex(row["data"].as_str().unwrap()));
        port.inbox.push_back(CanFrame {
            id: u32::from(slave + 0x10),
            data,
        });
        let mut control = MotorControl::new(port);
        control.add_motor(Motor::new(
            motor_type(&row["motor_type"]),
            slave,
            slave + 0x10,
        ));
        control.recv().expect("recv");
        let motor = control.motor(slave).unwrap();
        assert_eq!(f64::from(motor.get_position()), f64_of(&row["q"]), "{row}");
        assert_eq!(f64::from(motor.get_velocity()), f64_of(&row["dq"]), "{row}");
        assert_eq!(f64::from(motor.get_torque()), f64_of(&row["tau"]), "{row}");
        assert_eq!(
            u64::from(motor.get_error()),
            row["err"].as_u64().unwrap(),
            "{row}"
        );
        // DM_CAN.py does not decode the temperatures: bytes 6 and 7, as motorbridge reads them.
        assert_eq!((motor.state_t_mos, motor.state_t_rotor), (data[6], data[7]));
    }
}

#[test]
fn feedback_on_another_id_or_naming_another_motor_is_dropped() {
    let mut port = Capture::default();
    let answer = [0x11, 0x80, 0x00, 0x80, 0x08, 0x00, 30, 31];
    // On the motor's command id rather than its feedback id.
    port.inbox.push_back(CanFrame {
        id: 0x01,
        data: answer,
    });
    // On its feedback id but naming motor 2 in the low nibble.
    let mut foreign = answer;
    foreign[0] = 0x12;
    port.inbox.push_back(CanFrame {
        id: 0x11,
        data: foreign,
    });
    let mut control = MotorControl::new(port);
    control.add_motor(Motor::new(DmMotorType::DM4310, 0x01, 0x11));
    control.recv().expect("recv");
    assert_eq!(control.motor(0x01).unwrap().feedback_count, 0);
}

/// The recorded bus: each write must be the next recorded one, and makes its answers readable;
/// each wait must be the next recorded wait.
struct Replay {
    events: VecDeque<Value>,
    pending: Vec<u8>,
}

#[derive(Clone)]
struct ReplayStream(Arc<Mutex<Replay>>);

impl ReplayStream {
    fn new(events: &[Value]) -> Self {
        Self(Arc::new(Mutex::new(Replay {
            events: events.iter().cloned().collect(),
            pending: Vec::new(),
        })))
    }

    fn port(&self) -> Usb2CanPort<Self> {
        let waits = self.clone();
        Usb2CanPort::with_pause(
            self.clone(),
            Box::new(move |duration: Duration| {
                let next = waits.0.lock().unwrap().events.pop_front();
                let seconds = next
                    .as_ref()
                    .and_then(|event| event.get("sleep"))
                    .and_then(Value::as_f64)
                    .unwrap_or_else(|| panic!("waited {duration:?}; the vendor did {next:?}"));
                assert!(
                    (duration.as_secs_f64() - seconds).abs() < 1e-9,
                    "waited {duration:?}; the vendor waited {seconds} s"
                );
            }),
        )
    }

    fn remaining(&self) -> usize {
        self.0.lock().unwrap().events.len()
    }
}

impl Read for ReplayStream {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut replay = self.0.lock().unwrap();
        let count = replay.pending.len().min(buf.len());
        buf[..count].copy_from_slice(&replay.pending[..count]);
        replay.pending.drain(..count);
        Ok(count)
    }
}

impl Write for ReplayStream {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let mut replay = self.0.lock().unwrap();
        let next = replay.events.pop_front();
        let expected = next
            .as_ref()
            .and_then(|event| event.get("write"))
            .and_then(Value::as_str)
            .unwrap_or_else(|| panic!("wrote {}; the vendor did {next:?}", hex(buf)));
        assert_eq!(
            hex(buf),
            expected,
            "a written packet differs from the vendor's"
        );
        for answer in next.as_ref().unwrap()["respond"].as_array().unwrap() {
            let bytes = unhex(answer.as_str().unwrap());
            replay.pending.extend(bytes);
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

fn session(name: &str) -> Value {
    transcripts()["sessions"]
        .as_array()
        .unwrap()
        .iter()
        .find(|session| session["name"] == name)
        .cloned()
        .unwrap()
}

fn joints(step: &Value) -> Vec<f64> {
    step["joints"]
        .as_array()
        .unwrap()
        .iter()
        .map(f64_of)
        .collect()
}

fn report(step: &Value) -> StopReport {
    let ids = |value: &Value| -> Vec<u16> {
        value
            .as_array()
            .unwrap()
            .iter()
            .map(|id| u16::try_from(id.as_u64().unwrap()).unwrap())
            .collect()
    };
    StopReport {
        unproven: ids(&step["report"]["unproven"]),
        faults: step["report"]["faults"]
            .as_array()
            .unwrap()
            .iter()
            .map(|pair| {
                let pair = ids(pair);
                (pair[0], u8::try_from(pair[1]).unwrap())
            })
            .collect(),
    }
}

fn limits(step: &Value) -> Vec<MotorLimits> {
    step["limits"]
        .as_array()
        .unwrap()
        .iter()
        .map(|row| {
            let row = row.as_array().unwrap();
            #[allow(clippy::cast_possible_truncation)]
            let narrow = |value: &Value| f64_of(value) as f32;
            MotorLimits {
                bus_id: u16::try_from(row[0].as_u64().unwrap()).unwrap(),
                p_max: narrow(&row[1]),
                v_max: narrow(&row[2]),
                t_max: narrow(&row[3]),
            }
        })
        .collect()
}

fn feedback(step: &Value) -> Vec<MotorFeedback> {
    #[allow(clippy::cast_possible_truncation)]
    let narrow = |value: &Value| f64_of(value) as f32;
    let byte = |value: &Value| u8::try_from(value.as_u64().unwrap()).unwrap();
    step["feedback"]
        .as_array()
        .unwrap()
        .iter()
        .map(|answer| MotorFeedback {
            bus_id: u16::try_from(answer["bus_id"].as_u64().unwrap()).unwrap(),
            state: MotorState::from_err(byte(&answer["state"])),
            position: narrow(&answer["position"]),
            velocity: narrow(&answer["velocity"]),
            torque: narrow(&answer["torque"]),
            mos_celsius: byte(&answer["mos_celsius"]),
            rotor_celsius: byte(&answer["rotor_celsius"]),
        })
        .collect()
}

/// Run the chain through a recorded session and hold every step to the recorded outcome.
fn replay(name: &str) {
    let session = session(name);
    let stream = ReplayStream::new(session["events"].as_array().unwrap());
    let steps = session["steps"].as_array().unwrap();
    let mut chain = DamiaoChain::open(stream.port(), b601()).expect("open");
    assert_eq!(steps[0]["step"], "open");
    assert_eq!(chain.limits(), limits(&steps[0]).as_slice());
    for step in &steps[1..] {
        match step["step"].as_str().unwrap() {
            "command" => match chain.command(&joints(step)) {
                Ok(receipt) => {
                    assert_eq!(receipt.evidence, Evidence::BusObservedComplete);
                    let observed: Vec<f64> = step["observed"]
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(f64_of)
                        .collect();
                    assert_eq!(receipt.observed, observed);
                    assert_eq!(receipt.feedback, feedback(step));
                }
                Err(DamiaoChainError::StopPending) => {
                    assert_eq!(step["error"]["stop_pending"], true);
                }
                Err(error) => panic!("{error}"),
            },
            "stop" => assert_eq!(chain.stop(), report(step)),
            other => panic!("unknown step {other}"),
        }
    }
    assert_eq!(stream.remaining(), 0, "the vendor's session went further");
}

#[test]
fn a_whole_b601_session_is_the_vendors_bus_traffic() {
    replay("session");
}

#[test]
fn a_dm4340_reporting_damiaos_other_velocity_range_is_scaled_by_it() {
    replay("vmax_8");
}

#[test]
fn a_faulted_and_a_silent_motor_leave_the_stop_unproven_and_motion_refused() {
    replay("stop_fault");
}

#[test]
fn a_vmax_no_damiao_table_carries_is_refused_at_open() {
    let session = session("limit_mismatch");
    let stream = ReplayStream::new(session["events"].as_array().unwrap());
    match DamiaoChain::open(stream.port(), b601()) {
        Err(DamiaoChainError::LimitMismatch {
            bus_id,
            register,
            motor,
        }) => {
            let error = &session["steps"][0]["error"];
            assert_eq!(u64::from(bus_id), error["limit_mismatch"].as_u64().unwrap());
            assert_eq!(register, DmVariable::VMAX);
            #[allow(clippy::cast_possible_truncation)]
            let reported = f64_of(&error["motor"]) as f32;
            assert_eq!(motor, Some(ParamValue::Float(reported)));
        }
        Err(error) => panic!("{error}"),
        Ok(_) => panic!("opened a motor with a VMAX no table carries"),
    }
    assert_eq!(stream.remaining(), 0);
}

#[test]
fn a_target_outside_the_commandable_range_sends_nothing() {
    let session = session("session");
    let events = session["events"].as_array().unwrap();
    let opened = usize::try_from(session["steps"][0]["event_count"].as_u64().unwrap()).unwrap();
    let stream = ReplayStream::new(&events[..opened]);
    let mut chain = DamiaoChain::open(stream.port(), b601()).expect("open");
    let mut targets = vec![0.0; 7];
    targets[1] = 0.1; // joint2 stops at 0
    assert!(matches!(
        chain.command(&targets),
        Err(DamiaoChainError::Target(_))
    ));
    targets[1] = 0.0;
    targets[0] = 2.7; // inside the URDF's 2.8 rad, past the deployed soft box's 150 deg
    assert!(matches!(
        chain.command(&targets),
        Err(DamiaoChainError::Uncommandable { .. })
    ));
    targets[0] = 0.0;
    targets[6] = 0.05; // inside the finger's stroke, past the gripper motor's open angle
    assert!(matches!(
        chain.command(&targets),
        Err(DamiaoChainError::Uncommandable { .. })
    ));
    assert!(matches!(
        chain.command(&[0.0; 6]),
        Err(DamiaoChainError::Target(_))
    ));
    assert_eq!(stream.remaining(), 0);
}

#[test]
// 3.14 is the URDF's own limit, not an approximation of pi.
#[allow(clippy::approx_constant)]
fn the_commandable_ranges_are_the_urdf_inside_the_soft_box_and_the_reachable_gripper() {
    let ranges: Vec<(f64, f64)> = b601()
        .iter()
        .map(|motor| {
            let range = motor.commandable();
            (range.lower, range.upper)
        })
        .collect();
    let degrees = f64::to_radians;
    assert_eq!(
        ranges,
        vec![
            (degrees(-150.0), degrees(150.0)),
            (-3.14, 0.0),
            (-3.14, 0.0),
            (degrees(-80.0), 1.57),
            (-1.57, 1.57),
            (degrees(-90.0), degrees(90.0)),
            (0.0, 0.045),
        ]
    );
}

#[test]
fn the_transcripts_are_the_bytes_the_provenance_names() {
    let provenance: Value =
        serde_json::from_str(include_str!("fixtures/dm_can/provenance.json")).expect("provenance");
    let digest = Sha256::digest(include_bytes!("fixtures/dm_can/transcripts.json"));
    assert_eq!(
        hex(&digest),
        provenance["transcripts.json"]["sha256"].as_str().unwrap()
    );
}
