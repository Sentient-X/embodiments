//! The `sx-commission` binary killed by a signal while it holds a servo powered: it catches the
//! signal, turns torque off, restores what it changed, and exits non-zero.
//!
//! The bus is a pseudo-terminal: the binary opens its slave as it opens a USB serial adapter,
//! and the master side answers as the simulated STS3215 does.
#![cfg(unix)]

use std::fs::File;
use std::io::{Read as _, Write as _};
use std::os::fd::OwnedFd;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

use nix::sys::signal::{Signal, kill};
use nix::unistd::Pid;
use sx_embodiment_drivers::serial::SerialPort as _;
use sx_feetech_serial::MotorCalibration;
use sx_feetech_serial::sim::{RealTime, SimBus};

/// Answer every whole packet the binary writes on `master` from `bus`.
fn serve(master: OwnedFd, bus: SimBus) {
    let mut reader = File::from(master);
    let mut writer = reader.try_clone().expect("a second handle");
    let mut port = RealTime::new(bus);
    std::thread::spawn(move || {
        let mut pending: Vec<u8> = Vec::new();
        let mut buffer = [0_u8; 256];
        loop {
            let Ok(read) = reader.read(&mut buffer) else {
                return;
            };
            if read == 0 {
                return;
            }
            pending.extend_from_slice(&buffer[..read]);
            loop {
                let Some(start) = pending.windows(2).position(|w| w == [0xFF, 0xFF]) else {
                    pending.clear();
                    break;
                };
                pending.drain(..start);
                if pending.len() < 4 {
                    break;
                }
                let total = usize::from(pending[3]) + 4;
                if pending.len() < total {
                    break;
                }
                let packet: Vec<u8> = pending.drain(..total).collect();
                let _ = port.write_port(&packet);
                let answer = port.read_port(1024).unwrap_or_default();
                if !answer.is_empty() && writer.write_all(&answer).is_err() {
                    return;
                }
            }
        }
    });
}

fn torque(bus: &SimBus) -> u8 {
    bus.state().servos[&1][40]
}

#[test]
fn a_terminating_signal_while_powered_disables_restores_and_exits_non_zero() {
    for signal in [Signal::SIGTERM, Signal::SIGINT, Signal::SIGHUP] {
        let bus = SimBus::sts3215(&[(
            1,
            MotorCalibration {
                drive_mode: false,
                homing_offset: 0,
                range_min: 1000,
                range_max: 3000,
            },
        )]);
        let pty = nix::pty::openpty(None, None).expect("a pseudo-terminal");
        let path = nix::unistd::ttyname(&pty.slave).expect("its slave's name");
        serve(pty.master, bus.clone());
        let dir = std::env::temp_dir().join(format!(
            "sx-calibrate-signal-{}-{signal}",
            std::process::id()
        ));
        let _ = std::fs::remove_dir_all(&dir);
        // Goal_Speed 1 makes the 300-tick nudge take seconds: the signal lands mid-move.
        let mut command = Command::new(env!("CARGO_BIN_EXE_sx-commission"));
        command
            .args(["calibrate", "jog", "--bus"])
            .arg(&path)
            .args(["--id", "1", "--delta", "300", "--move", "--goal-speed", "1"])
            .arg("--journal-dir")
            .arg(&dir);
        // macOS sets a serial line's rate by an ioctl a pseudo-terminal refuses (ENOTTY);
        // serialport skips it at rate 0, which a pseudo-terminal does not need.
        if cfg!(target_os = "macos") {
            command.args(["--baud", "0"]);
        }
        let child = command
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("the binary starts");
        let mut child = child;
        let pid = Pid::from_raw(i32::try_from(child.id()).expect("a pid"));
        let deadline = Instant::now() + Duration::from_secs(10);
        while torque(&bus) != 1 {
            if Instant::now() >= deadline || child.try_wait().ok().flatten().is_some() {
                let _ = child.kill();
                let output = child.wait_with_output().expect("it exits");
                panic!(
                    "the binary never powered the servo: {}",
                    String::from_utf8_lossy(&output.stderr)
                );
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        std::thread::sleep(Duration::from_millis(200));
        assert_eq!(torque(&bus), 1, "still mid-move when signalled");
        kill(pid, signal).expect("signalled");
        let output = child.wait_with_output().expect("it exits");
        let stderr = String::from_utf8_lossy(&output.stderr);
        assert_eq!(output.status.code(), Some(1), "{signal}: {stderr}");
        assert!(stderr.contains("stopped by signal"), "{stderr}");
        assert_eq!(torque(&bus), 0, "{signal}: torque off");
        let state = bus.state();
        let r = &state.servos[&1];
        assert_eq!(
            (
                r[41],
                u16::from_le_bytes([r[46], r[47]]),
                u16::from_le_bytes([r[48], r[49]])
            ),
            (0, 0, 1000),
            "{signal}: acceleration, speed and torque limit restored"
        );
        drop(state);
        let journals: Vec<_> = std::fs::read_dir(&dir)
            .map(|entries| entries.flatten().collect())
            .unwrap_or_default();
        assert!(
            journals.is_empty(),
            "{signal}: journal removed: {journals:?}"
        );
        drop(pty.slave);
    }
}
