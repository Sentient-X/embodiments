//! The switch firmware's status lines and its start/stop holds.
//!
//! Beside the 100 Hz angle lines `encoder_line` parses, the YUBI encoder firmware with a
//! start/stop switch (XIAO ESP32-C6, AS5601, a KW10 dry contact on D9) prints a status line
//! at 50 Hz: `# ` then one JSON object (up to about 540 bytes). It repeats the latest switch
//! event on every frame: `event_id` counts accepted holds (2 s closed, one per hold, taps do
//! nothing) since the boot named by `session`, and `command` is the last one's START or STOP.
//! A host acts on each `(session, event_id)` once. Measured on the right encoder of rig
//! yubi-pod1-20261114 (2026-11-15), whose captures are this crate's test fixtures.
//!
//! This module owns the wire shape and the once-per-hold law ([`Holds`]); what a hold does —
//! which gripper may start an episode, and against which recorder state — is the host's.

use serde::Deserialize;

/// Which gripper the firmware says it is built into.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    Left,
    Right,
}

impl Side {
    pub fn as_str(self) -> &'static str {
        match self {
            Side::Left => "left",
            Side::Right => "right",
        }
    }
}

/// The firmware's last requested command: `None` before its first hold. The firmware keeps
/// its own requested state; a host that has its own (a recorder) never reads it as truth.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "UPPERCASE")]
pub enum Command {
    None,
    Start,
    Stop,
}

/// What `type` a status line names: the firmware prints only `state` frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
enum Kind {
    State,
}

#[derive(Deserialize)]
struct Wire {
    #[serde(rename = "type")]
    kind: Kind,
    session: String,
    ms: u64,
    gripper: Side,
    switch_enabled: bool,
    device_id: String,
    encoder_ok: bool,
    zero_set: bool,
    zero_error: bool,
    pressed: bool,
    event_id: u64,
    event_ms: u64,
    command: Command,
}

/// One status frame, as much of it as a host decides on. The angle fields it also carries
/// are not read here: the angle line (`encoder_line`) stays the one sample source, so a
/// recording's timing law is unchanged.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Status {
    /// The boot this frame belongs to: random per ESP32 start (a DTR edge resets it).
    pub session: String,
    /// Device uptime when sampled, ms.
    pub ms: u64,
    pub gripper: Side,
    /// Whether this unit's switch is wired and armed by its firmware.
    pub switch_enabled: bool,
    /// The ESP32-C6 factory MAC the firmware reads (the USB serial, without colons).
    pub device_id: String,
    pub encoder_ok: bool,
    pub zero_set: bool,
    pub zero_error: bool,
    /// The switch is closed now.
    pub pressed: bool,
    /// Accepted holds since `session` began; 0 is none.
    pub event_id: u64,
    /// Device uptime at the last accepted hold, ms (0 before the first).
    pub event_ms: u64,
    pub command: Command,
}

/// The prefix that marks a status line.
pub const PREFIX: &[u8] = b"# ";

/// Whether a line is the firmware's status line (never an angle sample, however it parses).
pub fn is_status(line: &[u8]) -> bool {
    line.starts_with(PREFIX)
}

/// A whole status line (without its newline), or `None` for any other line and for a status
/// line that does not parse: one cut by a reset or a transfer error, or one past the line
/// bound. A bad status line is dropped, never fatal — the next frame repeats its state.
pub fn parse_status(line: &[u8]) -> Option<Status> {
    let body = line.strip_prefix(PREFIX)?;
    let wire: Wire = serde_json::from_slice(body).ok()?;
    let Kind::State = wire.kind;
    Some(Status {
        session: wire.session,
        ms: wire.ms,
        gripper: wire.gripper,
        switch_enabled: wire.switch_enabled,
        device_id: wire.device_id,
        encoder_ok: wire.encoder_ok,
        zero_set: wire.zero_set,
        zero_error: wire.zero_error,
        pressed: wire.pressed,
        event_id: wire.event_id,
        event_ms: wire.event_ms,
        command: wire.command,
    })
}

/// A hold first seen by this reader.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Hold {
    pub gripper: Side,
    pub switch_enabled: bool,
    pub device_id: String,
    pub session: String,
    pub event_id: u64,
    pub event_ms: u64,
    /// What the firmware asked for; a recorder decides by its own state, not this.
    pub command: Command,
    /// Holds the firmware counted between the last one seen and this one, never seen: the
    /// stream is a latest-state stream, so a slow or absent reader collapses holds.
    pub missed: u64,
}

/// The once-per-hold law for one reader of one encoder. The first frame a reader sees, and
/// the first of each new session, is a baseline: an event already counted when the reader
/// came is never acted on, so a reconnect or a reset never replays an old hold.
#[derive(Debug, Default)]
pub struct Holds {
    seen: Option<(String, u64)>,
}

impl Holds {
    /// The hold this frame newly reports, if any.
    pub fn observe(&mut self, status: &Status) -> Option<Hold> {
        let last = match &self.seen {
            Some((session, last)) if *session == status.session => *last,
            _ => {
                self.seen = Some((status.session.clone(), status.event_id));
                return None;
            }
        };
        self.seen = Some((status.session.clone(), status.event_id));
        // A count that went back within one session is no hold: it is a new baseline.
        (status.event_id > last).then(|| Hold {
            gripper: status.gripper,
            switch_enabled: status.switch_enabled,
            device_id: status.device_id.clone(),
            session: status.session.clone(),
            event_id: status.event_id,
            event_ms: status.event_ms,
            command: status.command,
            missed: status.event_id - last - 1,
        })
    }

    /// Parse one line and observe it: the hold it newly reports, if it is a status line.
    pub fn line(&mut self, line: &[u8]) -> Option<Hold> {
        self.observe(&parse_status(line)?)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encoder_line::{self, Lines};

    const HOLDS: &[u8] =
        include_bytes!("../tests/fixtures/right-switch-three-holds-2026-11-15.txt");
    const TAPS: &[u8] = include_bytes!("../tests/fixtures/right-switch-taps-2026-11-15.txt");

    fn frame(session: &str, event_id: u64, command: &str) -> Vec<u8> {
        format!(
            "# {{\"type\":\"state\",\"session\":\"{session}\",\"ms\":121,\"gripper\":\"right\",\
             \"switch_enabled\":true,\"device_id\":\"f02eabfeff9b6490\",\"raw\":1715,\
             \"encoder_ok\":true,\"zero_set\":true,\"zero_error\":false,\"pressed\":false,\
             \"event_id\":{event_id},\"event_ms\":0,\"command\":\"{command}\",\
             \"recording_requested\":false}}"
        )
        .into_bytes()
    }

    /// Replays a captured byte stream through the framer, as a host reads it, in reads of
    /// `chunk` bytes: (angle samples, status frames, holds).
    fn replay(stream: &[u8], chunk: usize) -> (usize, usize, Vec<Hold>) {
        let mut lines = Lines::default();
        let mut holds = Holds::default();
        let (mut samples, mut frames, mut seen) = (0, 0, Vec::new());
        for bytes in stream.chunks(chunk) {
            lines
                .feed::<String>(bytes, |line| {
                    if encoder_line::parse(line)?.is_some() {
                        samples += 1;
                    }
                    if parse_status(line).is_some() {
                        frames += 1;
                    }
                    seen.extend(holds.line(line));
                    Ok(())
                })
                .unwrap();
        }
        (samples, frames, seen)
    }

    #[test]
    fn a_real_status_line_parses_whole() {
        let line = HOLDS
            .split(|b| *b == b'\n')
            .find(|line| line.starts_with(b"# {"))
            .unwrap();
        assert!(
            line.len() > 512,
            "the firmware's status line is {} bytes",
            line.len()
        );
        let status = parse_status(line).unwrap();
        assert_eq!(status.session, "4b7a646a4b3f5968");
        assert_eq!(status.gripper, Side::Right);
        assert!(status.switch_enabled && status.encoder_ok && status.zero_set);
        assert!(!status.zero_error && !status.pressed);
        assert_eq!(status.device_id, "f02eabfeff9b6490");
        assert_eq!((status.event_id, status.command), (0, Command::None));
        // Never an angle sample, however the angle parser splits it.
        assert_eq!(encoder_line::parse(line).unwrap(), None);
    }

    /// Bench, 2026-11-15, right encoder of yubi-pod1-20261114: three 2 s holds after a DTR
    /// reset (ROM banner and a status line cut by the reset first). Exactly START, STOP,
    /// START, once each, in any read size, and the angle lines are untouched.
    #[test]
    fn three_real_holds_replay_as_three_holds_once_each() {
        for chunk in [1, 7, 64, 512, 4096, 1 << 16] {
            let (samples, frames, holds) = replay(HOLDS, chunk);
            let got: Vec<_> = holds.iter().map(|h| (h.event_id, h.command)).collect();
            assert_eq!(
                got,
                [(1, Command::Start), (2, Command::Stop), (3, Command::Start)],
                "read size {chunk}"
            );
            assert!(
                holds
                    .iter()
                    .all(|h| h.gripper == Side::Right && h.switch_enabled)
            );
            assert!(holds.iter().all(|h| h.missed == 0));
            assert_eq!(
                holds.iter().map(|h| h.event_ms).collect::<Vec<_>>(),
                [22_421, 29_469, 36_274]
            );
            // 100.2 Hz of angles and 50 Hz of status over the 45 s capture.
            assert!(samples > 4_400, "{samples} samples");
            assert_eq!(frames, 2_255, "every whole status frame parses");
        }
    }

    /// Bench, same unit: a reset that cut the previous boot's status line mid-object, then
    /// taps under 2 s. No hold, and the cut line is dropped rather than refused.
    #[test]
    fn real_taps_and_a_line_cut_by_a_reset_are_no_hold() {
        let (samples, frames, holds) = replay(TAPS, 512);
        assert!(holds.is_empty(), "{holds:?}");
        assert!(
            samples > 1_000 && frames > 800,
            "{samples} samples, {frames} frames"
        );
        let cut = TAPS.split(|b| *b == b'\n').nth(1).unwrap();
        assert!(cut.starts_with(b"# {") && parse_status(cut).is_none());
    }

    #[test]
    fn the_first_frame_and_a_new_session_are_baselines_and_repeats_act_once() {
        let mut holds = Holds::default();
        // A reader that comes while event 3 stands never acts on it.
        assert_eq!(holds.line(&frame("a", 3, "START")), None);
        assert_eq!(holds.line(&frame("a", 3, "START")), None);
        let hold = holds.line(&frame("a", 4, "STOP")).unwrap();
        assert_eq!(
            (hold.event_id, hold.command, hold.missed),
            (4, Command::Stop, 0)
        );
        assert_eq!(holds.line(&frame("a", 4, "STOP")), None);
        // A jump counts what was never seen.
        assert_eq!(holds.line(&frame("a", 7, "START")).unwrap().missed, 2);
        // The device reset: its first frame is a baseline, then its holds count.
        assert_eq!(holds.line(&frame("b", 0, "NONE")), None);
        assert_eq!(holds.line(&frame("b", 1, "START")).unwrap().event_id, 1);
        // A count that went back in one session re-baselines, never acts.
        assert_eq!(holds.line(&frame("b", 0, "NONE")), None);
        assert_eq!(holds.line(&frame("b", 1, "START")).unwrap().event_id, 1);
        // A new reader (a reconnect) baselines again.
        let mut again = Holds::default();
        assert_eq!(again.line(&frame("b", 1, "START")), None);
    }

    #[test]
    fn malformed_partial_and_foreign_lines_are_dropped() {
        let whole = frame("a", 1, "START");
        assert!(parse_status(&whole).is_some());
        assert!(parse_status(&whole[..whole.len() - 1]).is_none());
        assert!(parse_status(&whole[2..]).is_none(), "no prefix, no status");
        let poisoned = [b"# \0".as_slice(), &whole[2..]].concat();
        assert!(
            parse_status(&poisoned).is_none(),
            "a link cut marker poisons it"
        );
        assert!(parse_status(b"# boot").is_none());
        assert!(parse_status(b"0.25").is_none());
        let other = String::from_utf8(whole.clone())
            .unwrap()
            .replace("\"state\"", "\"log\"");
        assert!(parse_status(other.as_bytes()).is_none());
        let unknown = String::from_utf8(whole).unwrap().replace("START", "PAUSE");
        assert!(parse_status(unknown.as_bytes()).is_none());
    }
}
