//! Entirely local keyboard teaching. Hardware is reached only through the embodiment layer.
use std::collections::{BTreeMap, BTreeSet};
use std::io::{self, IsTerminal, Read, Write};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use nix::sys::signal::{self, SigSet, SigmaskHow, Signal};
use nix::sys::termios::{self, SetArg, Termios};
use sx_embodiment_bodies::system::SystemPorts;
use sx_embodiment_bodies::{
    Body, Commissioning, Observer, OpenOptions, ServoCalibrations, observe, open_with,
};
use sx_embodiment_commission::bench::BodyFlags;
use sx_embodiment_commission::teach::{Action, Config, Controller, JointDiagnostic, Mode};
use sx_embodiment_commission::teach_episode::{Episode, EpisodeWriter, Header};
use sx_embodiment_drivers::dynamics::{
    Dynamics, DynamicsCalibration, certified_together, sha256_hex,
};
use sx_embodiment_drivers::guidance::GuidanceProfile;
use sx_embodiment_drivers::{ActuatorChain, BoundAxis, InstanceDrive};
use sx_embodiment_ik::teleop::Gripper;

const PERIOD: Duration = Duration::from_millis(10);
const INPUT_CAPACITY: usize = 16;
const HELP: &str = "sx-teach --drive FILE --commissioning FILE --urdf FILE --dynamics FILE\n\
  --joints joint1,joint2 --out-dir DIR [--replay FILE] [--move] [--check-config]\n\
  [--held joint=rad] [--speed 0.2] [--share 0.25] [--damping 0.5] [--guidance FILE]\n\
  [--status-file FILE] [--max-seconds 3600] [--calibration port=file] [--exclude joint,...]\n\
--check-config checks local files only and never opens hardware.\n\
Without --move, checks files and observes hardware without enabling motors.\n\
With --move, requires an attached local terminal. Keys:\n\
  e engage hold; t teach (grippers free by hand); r start/save recording; h hold (recording continues); SPACE stop\n\
  p approach recording start, then p again plays; - close / + open active grippers in hold\n\
  x release torque and exit from hold/fault (arms must be supported).\n\
Ctrl-C stops, releases torque and exits; support both arms before pressing it.\n\
Space/q and terminal loss request hold; existing motor-fault safety handling remains in effect.\n\
--guidance selects independently calibrated Teach gravity/damping; Hold/replay retain their original model and gains.\n\
--status-file publishes optional local read-only telemetry; keyboard controls remain authoritative.\n\
Teach releases active grippers for hand guidance; Hold/replay restore powered control.\n\
This version records joint/gripper trajectories only, without camera video.";

struct Args {
    body: BodyFlags,
    urdf: PathBuf,
    dynamics: Vec<PathBuf>,
    guidance: Option<PathBuf>,
    joints: Vec<String>,
    held: BTreeMap<String, f64>,
    out_dir: PathBuf,
    replay: Option<PathBuf>,
    moving: bool,
    check_config: bool,
    status_file: Option<PathBuf>,
    speed: f64,
    share: f64,
    damping: f64,
    max_seconds: f64,
}

fn parse() -> Result<Option<Args>, String> {
    let mut parsed = Args {
        body: BodyFlags::default(),
        urdf: PathBuf::new(),
        dynamics: Vec::new(),
        guidance: None,
        joints: Vec::new(),
        held: BTreeMap::new(),
        out_dir: PathBuf::new(),
        replay: None,
        moving: false,
        check_config: false,
        status_file: None,
        speed: 0.2,
        share: 0.25,
        damping: 0.5,
        max_seconds: 3600.0,
    };
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().ok_or_else(|| format!("{arg} needs a value"));
        if parsed.body.take(&arg, &mut value)? {
            continue;
        }
        match arg.as_str() {
            "--help" | "-h" => {
                println!("{HELP}");
                return Ok(None);
            }
            "--urdf" => parsed.urdf = value()?.into(),
            "--dynamics" => parsed.dynamics.push(value()?.into()),
            "--guidance" => parsed.guidance = Some(value()?.into()),
            "--joints" => parsed.joints = value()?.split(',').map(str::to_owned).collect(),
            "--held" => {
                let entry = value()?;
                let (joint, number) = entry.split_once('=').ok_or("--held is joint=rad")?;
                let number: f64 = number.parse().map_err(|_| "bad --held angle")?;
                if joint.is_empty()
                    || !number.is_finite()
                    || parsed.held.insert(joint.to_owned(), number).is_some()
                {
                    return Err("--held joints must be unique, named and finite".into());
                }
            }
            "--out-dir" => parsed.out_dir = value()?.into(),
            "--replay" => parsed.replay = Some(value()?.into()),
            "--move" => parsed.moving = true,
            "--check-config" => parsed.check_config = true,
            "--status-file" => parsed.status_file = Some(value()?.into()),
            "--speed" => parsed.speed = value()?.parse().map_err(|_| "bad --speed")?,
            "--share" => parsed.share = value()?.parse().map_err(|_| "bad --share")?,
            "--damping" => parsed.damping = value()?.parse().map_err(|_| "bad --damping")?,
            "--max-seconds" => {
                parsed.max_seconds = value()?.parse().map_err(|_| "bad --max-seconds")?;
            }
            _ => return Err(format!("unknown argument {arg}")),
        }
    }
    if parsed.urdf.as_os_str().is_empty()
        || parsed.dynamics.is_empty()
        || parsed.joints.is_empty()
        || parsed.out_dir.as_os_str().is_empty()
    {
        return Err("--urdf, --dynamics, --joints and --out-dir are required".into());
    }
    if !(parsed.speed.is_finite()
        && 0.0 < parsed.speed
        && parsed.speed <= 1.0
        && parsed.share.is_finite()
        && 0.0 < parsed.share
        && parsed.share <= 1.0
        && parsed.damping.is_finite()
        && parsed.damping > 0.0
        && parsed.max_seconds.is_finite()
        && 0.0 < parsed.max_seconds
        && parsed.max_seconds <= 3600.0)
    {
        return Err("speed/share in (0,1]; damping positive; max-seconds in (0,3600]".into());
    }
    Ok(Some(parsed))
}

fn read(path: &Path) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|error| format!("{}: {error}", path.display()))
}

/// One immutable snapshot: the exact parsed values opened and the raw bytes hashed.
struct Inputs {
    model: Dynamics,
    guidance: Option<GuidanceProfile>,
    config_sha: String,
    drive: InstanceDrive,
    commissioning: Commissioning,
    calibrations: ServoCalibrations,
    provisional: Vec<String>,
    grippers: Vec<Gripper>,
}

impl Inputs {
    fn open(&self) -> Result<Body, String> {
        let provisional: Vec<_> = self.provisional.iter().map(String::as_str).collect();
        open_with(
            &self.drive,
            &self.commissioning,
            &self.calibrations,
            OpenOptions {
                allow_provisional: &provisional,
                ..OpenOptions::default()
            },
            &mut SystemPorts::system(),
        )
        .map_err(|error| error.to_string())
    }
    fn observe(&self) -> Result<Observer, String> {
        let provisional: Vec<_> = self.provisional.iter().map(String::as_str).collect();
        observe(
            &self.drive,
            &self.commissioning,
            &self.calibrations,
            OpenOptions {
                allow_provisional: &provisional,
                ..OpenOptions::default()
            },
            &mut SystemPorts::system(),
        )
        .map_err(|error| error.to_string())
    }
}

/// Reuse the canonical geometric finger endpoints; drive order is the hardware order.
fn canonical_grippers(
    description_sha: &str,
    drive: &InstanceDrive,
    out_of_service: &BTreeSet<String>,
    documents: &mut BTreeMap<String, String>,
) -> Result<Vec<Gripper>, String> {
    let Some(files) = sx_embodiment_ik::every_registered()
        .iter()
        .find(|files| sha256_hex(files.urdf) == description_sha)
    else {
        return Ok(Vec::new());
    };
    let body = files.body().map_err(|error| error.to_string())?;
    let mut grippers = Vec::new();
    for limb in 0..body.limbs().len() {
        let Some(mut gripper) = Gripper::of(&body, limb) else {
            continue;
        };
        for (axis, _, _) in &mut gripper.fingers {
            let name = &body
                .model()
                .axes()
                .get(*axis)
                .ok_or_else(|| format!("canonical gripper model axis {axis} is invalid"))?
                .name;
            *axis = drive
                .joints
                .iter()
                .position(|joint| &joint.joint == name)
                .ok_or_else(|| {
                    format!("canonical gripper joint {name} is absent from the drive")
                })?;
        }
        let mut excluded = 0;
        for (axis, _, _) in &gripper.fingers {
            let joint = drive
                .joints
                .get(*axis)
                .ok_or_else(|| format!("canonical gripper drive axis {axis} is invalid"))?;
            if out_of_service.contains(&joint.joint) {
                excluded += 1;
            }
        }
        if excluded == gripper.fingers.len() {
            continue;
        }
        if excluded > 0 {
            return Err(
                "canonical gripper is partially excluded; exclude all its fingers or none".into(),
            );
        }
        grippers.push(gripper);
    }
    documents.insert("gripper-render-pack".to_owned(), files.pack.to_owned());
    documents.insert("gripper-collision".to_owned(), files.collision.to_owned());
    documents.insert("gripper-urdf".to_owned(), files.urdf.to_owned());
    Ok(grippers)
}

/// Whole excluded ports disappear from the opened chain. Resolve names again at this
/// boundary rather than carrying drive-file indices into a shorter native command vector.
fn opened_grippers(
    grippers: &[Gripper],
    drive: &InstanceDrive,
    axes: &[BoundAxis],
) -> Result<Vec<Gripper>, String> {
    let mut mapped = grippers.to_vec();
    for gripper in &mut mapped {
        for (index, _, _) in &mut gripper.fingers {
            let joint = drive
                .joints
                .get(*index)
                .ok_or_else(|| format!("canonical gripper drive axis {index} is invalid"))?;
            *index = axes
                .iter()
                .position(|axis| axis.joint == joint.joint)
                .ok_or_else(|| {
                    format!(
                        "required gripper finger {} is absent from the opened body",
                        joint.joint
                    )
                })?;
            if !axes[*index].gripper {
                return Err(format!(
                    "opened joint {} is not a gripper finger",
                    joint.joint
                ));
            }
        }
    }
    Ok(mapped)
}

fn snapshot_read(path: &Path, cache: &mut BTreeMap<PathBuf, String>) -> Result<String, String> {
    let identity = path
        .canonicalize()
        .map_err(|error| format!("{}: {error}", path.display()))?;
    if let Some(raw) = cache.get(&identity) {
        return Ok(raw.clone());
    }
    let raw = read(&identity)?;
    cache.insert(identity, raw.clone());
    Ok(raw)
}

#[allow(clippy::too_many_lines)] // Read/parse/hash one snapshot before any hardware opener runs.
fn inputs(args: &Args) -> Result<Inputs, String> {
    let mut cache = BTreeMap::new();
    let urdf = snapshot_read(&args.urdf, &mut cache)?;
    let mut model = Dynamics::of(&urdf, [0.0, 0.0, -9.81]).map_err(|error| error.to_string())?;
    let description_sha = sha256_hex(&urdf);
    let drive_raw = snapshot_read(Path::new(&args.body.drive), &mut cache)?;
    let instance = serde_json::from_str::<serde_json::Value>(&drive_raw)
        .map_err(|error| error.to_string())?["instance"]
        .as_str()
        .ok_or("drive names no instance")?
        .to_owned();
    let drive = InstanceDrive::parse(&drive_raw, &instance).map_err(|error| error.to_string())?;
    let drive_sha = sha256_hex(&drive_raw);
    let commissioning_raw = snapshot_read(Path::new(&args.body.commissioning), &mut cache)?;
    let mut commissioning =
        Commissioning::parse(&commissioning_raw, &instance).map_err(|error| error.to_string())?;
    commissioning
        .out_of_service
        .extend(args.body.exclude.iter().cloned());
    for joint in &args.joints {
        if commissioning.unidentified.contains(joint) {
            return Err(format!(
                "{joint} is commissioned as unidentified; qualify its dynamics before teaching"
            ));
        }
    }
    let mut documents = BTreeMap::from([
        ("urdf".to_owned(), urdf),
        ("drive".to_owned(), drive_raw),
        ("commissioning".to_owned(), commissioning_raw),
    ]);
    let mut parsed_dynamics = Vec::new();
    let mut calibrated_links = BTreeSet::new();
    let mut calibrated_joints = BTreeSet::new();
    let mut historical = BTreeMap::new();
    for (index, path) in args.dynamics.iter().enumerate() {
        let text = snapshot_read(path, &mut cache)?;
        let calibration = DynamicsCalibration::parse(&text).map_err(|error| error.to_string())?;
        if calibration
            .links
            .iter()
            .any(|(name, _)| !calibrated_links.insert(name.clone()))
            || calibration
                .joints
                .iter()
                .any(|(name, _)| !calibrated_joints.insert(name.clone()))
        {
            return Err("dynamics files overlap".into());
        }
        let digest = &calibration.description_sha256;
        if digest != &description_sha && !historical.contains_key(digest) {
            if digest.len() != 64 || !digest.bytes().all(|byte| byte.is_ascii_hexdigit()) {
                return Err("dynamics description digest must be 64 hexadecimal characters".into());
            }
            let history = args
                .urdf
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join("_by_digest")
                .join(digest)
                .join(args.urdf.file_name().ok_or("URDF filename missing")?);
            let raw = snapshot_read(&history, &mut cache)?;
            if sha256_hex(&raw) != *digest {
                return Err(format!(
                    "historical URDF {} does not match {digest}",
                    history.display()
                ));
            }
            documents.insert(format!("historical-{digest}"), raw.clone());
            historical.insert(digest.clone(), raw);
        }
        parsed_dynamics.push(calibration);
        documents.insert(format!("dynamics-{index}"), text);
    }
    let calibration_names: Vec<_> = args
        .dynamics
        .iter()
        .map(|path| path.display().to_string())
        .collect();
    let calibration_set: Vec<_> = calibration_names
        .iter()
        .zip(&parsed_dynamics)
        .map(|(name, calibration)| (name.as_str(), calibration))
        .collect();
    certified_together(&calibration_set).map_err(|error| error.to_string())?;
    let earlier: Vec<_> = historical.values().map(String::as_str).collect();
    model = model
        .superseding(&earlier)
        .map_err(|error| error.to_string())?;
    for calibration in &parsed_dynamics {
        model = model
            .calibrated(calibration)
            .map_err(|error| error.to_string())?;
    }
    let guidance = args
        .guidance
        .as_ref()
        .map(|path| {
            let text = snapshot_read(path, &mut cache)?;
            let profile = GuidanceProfile::parse(&text, &description_sha, &drive_sha)
                .map_err(|error| error.to_string())?;
            for setting in profile.settings() {
                if !model
                    .tree()
                    .joints()
                    .iter()
                    .any(|joint| joint.name == setting.joint)
                {
                    return Err(format!(
                        "guidance names unknown physical joint {}",
                        setting.joint
                    ));
                }
            }
            for joint in &args.joints {
                if profile.joint_settings(joint).is_none() {
                    return Err(format!("guidance does not cover selected joint {joint}"));
                }
            }
            documents.insert("guidance".to_owned(), text);
            Ok(profile)
        })
        .transpose()?;
    let mut calibrations = ServoCalibrations::new();
    let mut explicit = BTreeSet::new();
    for (port, path) in &args.body.calibrations {
        if !explicit.insert(port.clone()) {
            return Err(format!("duplicate explicit calibration for port {port}"));
        }
        let raw = snapshot_read(Path::new(path), &mut cache)?;
        calibrations = calibrations
            .with_lerobot(port, &raw)
            .map_err(|error| error.to_string())?;
        documents.insert(format!("calibration-{port}"), raw);
    }
    let directory = Path::new(&args.body.commissioning)
        .parent()
        .unwrap_or_else(|| Path::new("."));
    for port in commissioning.ports.keys() {
        if explicit.contains(port) {
            continue;
        }
        let path = directory.join(sx_embodiment_bodies::calibration_file(port));
        match path.try_exists() {
            Ok(false) => continue,
            Err(error) => return Err(format!("{}: {error}", path.display())),
            Ok(true) => {}
        }
        let raw = snapshot_read(&path, &mut cache)?;
        calibrations = calibrations
            .with_lerobot(port, &raw)
            .map_err(|error| error.to_string())?;
        documents.insert(format!("automatic-{port}"), raw);
    }
    let grippers = canonical_grippers(
        &description_sha,
        &drive,
        &commissioning.out_of_service,
        &mut documents,
    )?;
    let description = serde_json::json!({"documents":documents,"joints":args.joints,"held":args.held,"exclude":args.body.exclude,"provisional":args.body.provisional,"speed":args.speed,"share":args.share,"damping":args.damping,"period_s":PERIOD.as_secs_f64()});
    Ok(Inputs {
        model,
        guidance,
        config_sha: sha256_hex(&description.to_string()),
        drive,
        commissioning,
        calibrations,
        provisional: args.body.provisional.clone(),
        grippers,
    })
}

/// Optional local file telemetry. This queue cannot block motor control; a slow viewer may
/// miss updates. Freshness follows the snapshot's host measurement time, not the time the disk flushed.
/// Diagnostic ages use host observations/receipts, never a motor's native clock. Completed
/// gravity is the retained requested command, not measured applied torque; fault/hold updates
/// may replace motion feedback. Release retains the initiating terminal error for review.
struct StatusFrame {
    mode: Mode,
    position: [f64; sx_embodiment_commission::teach_episode::MAX_AXES],
    width: usize,
    updated_at_unix_ms: u64,
    error: Option<String>,
    diagnostics: Option<[JointDiagnostic; sx_embodiment_commission::teach_episode::MAX_AXES]>,
}

#[derive(Clone, Copy, serde::Serialize)]
struct StatusRange {
    lower: f64,
    upper: f64,
}

struct StatusPublisher {
    sender: Option<SyncSender<StatusFrame>>,
    worker: Option<thread::JoinHandle<()>>,
    last: Option<Instant>,
}

impl StatusPublisher {
    fn spawn(
        path: PathBuf,
        joints: Vec<String>,
        ranges: BTreeMap<String, StatusRange>,
    ) -> Result<Self, String> {
        let (sender, receiver) = sync_channel::<StatusFrame>(2);
        let worker = thread::Builder::new()
            .name("sx-teach-status".to_owned())
            .spawn(move || {
                for frame in receiver {
                    if let Err(error) = write_status(&path, &joints, &ranges, &frame) {
                        eprintln!("sx-teach status file: {error}");
                        break;
                    }
                }
            })
            .map_err(|error| error.to_string())?;
        Ok(Self {
            sender: Some(sender),
            worker: Some(worker),
            last: None,
        })
    }

    #[allow(clippy::too_many_arguments)] // One bounded status frame carries these observations together.
    fn publish(
        &mut self,
        mode: Mode,
        position: &[f64],
        measured_at: Instant,
        error: Option<&str>,
        force: bool,
        diagnostics: Option<&[JointDiagnostic; sx_embodiment_commission::teach_episode::MAX_AXES]>,
    ) {
        let now = Instant::now();
        if !force
            && self
                .last
                .is_some_and(|last| now.duration_since(last) < Duration::from_millis(100))
        {
            return;
        }
        if position.len() > sx_embodiment_commission::teach_episode::MAX_AXES {
            return;
        }
        let received_at = SystemTime::now().checked_sub(now.saturating_duration_since(measured_at));
        let updated_at_unix_ms = received_at
            .and_then(|at| at.duration_since(UNIX_EPOCH).ok())
            .and_then(|elapsed| u64::try_from(elapsed.as_millis()).ok())
            .unwrap_or(0);
        let mut frame = StatusFrame {
            mode,
            position: [0.0; sx_embodiment_commission::teach_episode::MAX_AXES],
            width: position.len(),
            updated_at_unix_ms,
            error: error.map(str::to_owned),
            diagnostics: diagnostics.copied(),
        };
        frame.position[..position.len()].copy_from_slice(position);
        if let Some(sender) = &self.sender {
            let _ = sender.try_send(frame);
        }
        self.last = Some(now);
    }

    fn finish(mut self) {
        self.sender = None;
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn status_json(
    joints: &[String],
    ranges: &BTreeMap<String, StatusRange>,
    frame: &StatusFrame,
) -> serde_json::Value {
    let positions: BTreeMap<_, _> = joints
        .iter()
        .zip(frame.position.iter().take(frame.width))
        .filter(|(_, position)| position.is_finite())
        .map(|(joint, value)| (joint, *value))
        .collect();
    let mode = match frame.mode {
        Mode::Disarmed => "disarmed",
        Mode::Hold => "hold",
        Mode::Teach => "teach",
        Mode::Approach => "approach",
        Mode::ReadyReplay => "ready_replay",
        Mode::Replay => "replay",
        Mode::Fault => "fault",
        Mode::Released => "released",
    };
    let now = Instant::now();
    let finite = |value: Option<f64>| value.filter(|value| value.is_finite());
    let age_ms =
        |at: Option<Instant>| at.map(|at| now.saturating_duration_since(at).as_secs_f64() * 1000.0);
    let diagnostics: BTreeMap<_, _> =
        frame
            .diagnostics
            .as_ref()
            .map_or_else(BTreeMap::new, |diagnostics| {
                joints.iter().zip(diagnostics).map(|(joint, diagnostic)| (joint, serde_json::json!({
            "calculated_gravity_nm":finite(diagnostic.calculated_gravity_nm),
            "support_offset_nm":finite(diagnostic.support_offset_nm),
            "completed_gravity_command_nm":finite(diagnostic.completed_gravity_command_nm),
            "position_target_rad":finite(diagnostic.position_target_rad),
            "feedback_velocity_rad_s":finite(diagnostic.feedback_velocity_rad_s),
            "feedback_torque_nm":finite(diagnostic.feedback_torque_nm),
            "model_age_ms":age_ms(diagnostic.model_observed_at),
            "gravity_command_age_ms":age_ms(diagnostic.gravity_command_completed_at),
            "feedback_age_ms":age_ms(diagnostic.feedback_received_at),
        }))).collect()
            });
    serde_json::json!({"mode":mode,"positions":positions,"ranges":ranges,"updated_at_unix_ms":frame.updated_at_unix_ms,"error":frame.error,"diagnostics":diagnostics})
}

fn write_status(
    path: &Path,
    joints: &[String],
    ranges: &BTreeMap<String, StatusRange>,
    frame: &StatusFrame,
) -> Result<(), String> {
    let filename = path
        .file_name()
        .ok_or("status path needs a filename")?
        .to_string_lossy();
    let temporary = path.with_file_name(format!(".{filename}.{}.tmp", std::process::id()));
    let file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&temporary)
        .map_err(|error| error.to_string())?;
    let outcome = (|| {
        let mut writer = io::BufWriter::new(file);
        serde_json::to_writer(&mut writer, &status_json(joints, ranges, frame))
            .map_err(|error| error.to_string())?;
        writer.flush().map_err(|error| error.to_string())?;
        std::fs::rename(&temporary, path).map_err(|error| error.to_string())
    })();
    if outcome.is_err() {
        let _ = std::fs::remove_file(temporary);
    }
    outcome
}

struct Terminal(Termios);
impl Terminal {
    fn raw() -> Result<Self, String> {
        if !io::stdin().is_terminal() || !io::stdout().is_terminal() {
            return Err("--move requires a local interactive terminal".into());
        }
        let original = termios::tcgetattr(io::stdin()).map_err(|error| error.to_string())?;
        let mut raw = original.clone();
        termios::cfmakeraw(&mut raw);
        termios::tcsetattr(io::stdin(), SetArg::TCSANOW, &raw)
            .map_err(|error| error.to_string())?;
        Ok(Self(original))
    }
}
impl Drop for Terminal {
    fn drop(&mut self) {
        let _ = termios::tcsetattr(io::stdin(), SetArg::TCSANOW, &self.0);
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Input {
    Action(Action),
    Record,
}
/// Discard terminal escape/Alt sequences; arrow keys must never become motor controls.
fn key(byte: u8, escape: &mut u8) -> Option<Input> {
    if matches!(byte, b' ' | 3 | b'q') {
        *escape = 0;
        return Some(Input::Action(if byte == 3 {
            Action::Exit
        } else {
            Action::Stop
        }));
    }
    match *escape {
        1 => {
            *escape = if matches!(byte, b'[' | b'O') { 2 } else { 0 };
            return None;
        }
        2 => {
            if (0x40..=0x7e).contains(&byte) {
                *escape = 0;
            }
            return None;
        }
        _ => {}
    }
    Some(match byte {
        27 => {
            *escape = 1;
            return None;
        }
        b'e' => Input::Action(Action::Engage),
        b't' => Input::Action(Action::Teach),
        b'h' => Input::Action(Action::Hold),
        b'p' => Input::Action(Action::Replay),
        b'x' => Input::Action(Action::Release),
        b'-' => Input::Action(Action::GripperStep(-0.05)),
        b'+' => Input::Action(Action::GripperStep(0.05)),
        b'r' => Input::Record,
        _ => return None,
    })
}

/// Ctrl-C cannot be lost when the ordinary keyboard queue is full.
fn queue_input(
    event: Input,
    sender: &SyncSender<Input>,
    exit_requested: &AtomicBool,
    lost: &AtomicBool,
) -> bool {
    if event == Input::Action(Action::Exit) {
        exit_requested.store(true, Ordering::Release);
        return true;
    }
    match sender.try_send(event) {
        Ok(()) => true,
        Err(TrySendError::Full(_)) => {
            // Hold on overflow, but keep reading so a later Ctrl-C can still exit.
            lost.store(true, Ordering::Release);
            true
        }
        Err(TrySendError::Disconnected(_)) => {
            lost.store(true, Ordering::Release);
            false
        }
    }
}

fn controls(input: &Receiver<Input>, exit_requested: &AtomicBool) -> (Option<Action>, bool) {
    let mut action = None;
    let mut recording_key = false;
    for _ in 0..INPUT_CAPACITY {
        if exit_requested.swap(false, Ordering::AcqRel) {
            return (Some(Action::Exit), false);
        }
        let Ok(event) = input.try_recv() else { break };
        match event {
            Input::Action(Action::Exit) => return (Some(Action::Exit), false),
            Input::Action(Action::Stop) if action != Some(Action::Exit) => {
                action = Some(Action::Stop);
            }
            Input::Action(command) if !matches!(action, Some(Action::Stop | Action::Exit)) => {
                action = Some(command);
            }
            Input::Record => recording_key = true,
            Input::Action(_) => {}
        }
    }
    if exit_requested.swap(false, Ordering::AcqRel) {
        action = Some(Action::Exit);
    }
    if action == Some(Action::Exit) {
        recording_key = false;
    }
    (action, recording_key)
}

fn request_signal(signal: Signal, exit_requested: &AtomicBool, interrupted: &AtomicBool) {
    if signal == Signal::SIGINT {
        exit_requested.store(true, Ordering::Release);
    } else {
        interrupted.store(true, Ordering::Release);
    }
}

enum DiskResult {
    Created(PathBuf, Result<EpisodeWriter, String>),
    Finished(PathBuf, Result<Episode, String>),
}
struct Capture {
    path: PathBuf,
    writer: EpisodeWriter,
    began: Option<Instant>,
    samples: usize,
}

fn notice(sender: &SyncSender<String>, message: impl Into<String>) {
    let _ = sender.try_send(message.into());
}

fn finalize(
    capture: Capture,
    header: Header,
    sender: SyncSender<DiskResult>,
) -> Result<(), String> {
    thread::Builder::new()
        .name("sx-teach-finalize".to_owned())
        .spawn(move || {
            let result = capture
                .writer
                .finish()
                .map_err(|error| error.to_string())
                .and_then(|_| {
                    Episode::load(&capture.path, &header).map_err(|error| error.to_string())
                });
            let _ = sender.send(DiskResult::Finished(capture.path, result));
        })
        .map_err(|error| error.to_string())?;
    Ok(())
}

#[allow(
    clippy::too_many_arguments,
    clippy::too_many_lines,
    clippy::needless_pass_by_value
)]
// One worker owns these channels and the entire control/recording lifecycle until release.
fn control(
    mut controller: Controller<Body>,
    header: Header,
    out_dir: PathBuf,
    input: Receiver<Input>,
    notices: SyncSender<String>,
    lost: Arc<AtomicBool>,
    interrupted: Arc<AtomicBool>,
    exit_requested: Arc<AtomicBool>,
    mut telemetry: Option<StatusPublisher>,
) -> Result<(), String> {
    if let Err(error) = sx_realtime::elevate(sx_realtime::Role::ControlLoop { period: PERIOD }) {
        notice(
            &notices,
            format!("Real-time scheduling unavailable: {error}. Loop timing is still monitored."),
        );
    }
    let (disk_tx, disk_rx) = sync_channel(2);
    let mut capture: Option<Capture> = None;
    let mut last_error: Option<String> = None;
    let mut reported_faults = BTreeSet::new();
    let mut pending = false;
    let mut lost_handled = false;
    let mut sequence = 0_u64;
    let mut last_mode = controller.mode();
    let mut last_replay_ready = controller.replay_ready();
    let mut next = Instant::now();
    loop {
        if interrupted.swap(false, Ordering::AcqRel)
            || lost.load(Ordering::Acquire) && !lost_handled
        {
            lost_handled |= lost.load(Ordering::Acquire);
            controller.fault("terminal/input lost or interrupted");
            last_error = Some("terminal/input lost or interrupted".to_owned());
            notice(
                &notices,
                "Stopped: holding after terminal loss or interrupt.",
            );
        }
        while let Ok(result) = disk_rx.try_recv() {
            pending = false;
            match result {
                DiskResult::Created(path, Ok(writer)) if controller.mode() == Mode::Teach => {
                    capture = Some(Capture {
                        path,
                        writer,
                        began: None,
                        samples: 0,
                    });
                    notice(&notices, "Recording started. Press r to save and hold.");
                }
                DiskResult::Created(_, Ok(_)) => notice(
                    &notices,
                    "Recording cancelled before first sample; incomplete file retained.",
                ),
                DiskResult::Created(_, Err(error)) | DiskResult::Finished(_, Err(error)) => {
                    controller.fault(&error);
                    last_error = Some(error.clone());
                    notice(&notices, format!("Recording failed: {error}"));
                }
                DiskResult::Finished(path, Ok(episode)) => {
                    if let Err(error) = controller.set_episode(episode) {
                        controller.fault(&error);
                        last_error = Some(error.clone());
                        notice(&notices, error);
                    } else {
                        notice(
                            &notices,
                            format!("Saved {}. Checking recording for replay...", path.display()),
                        );
                    }
                }
            }
        }
        if let Some(take) = capture.as_ref()
            && let Err(error) = take.writer.check()
        {
            controller.fault(&error.to_string());
            last_error = Some(error.to_string());
            notice(&notices, error.to_string());
        }
        let (mut action, mut recording_key) = controls(&input, &exit_requested);
        let mut finish_requested = false;
        if lost.load(Ordering::Acquire) && !matches!(action, Some(Action::Release | Action::Exit)) {
            action = Some(Action::Stop);
            recording_key = false;
        }
        if pending
            && !matches!(
                action,
                Some(Action::Stop | Action::Hold | Action::Release | Action::Exit)
            )
        {
            action = None;
        }
        if recording_key
            && !pending
            && !matches!(action, Some(Action::Stop | Action::Release | Action::Exit))
        {
            if capture.is_some() {
                // Finish only after this tick has established a successful hold.
                action = Some(Action::Hold);
                finish_requested = true;
            } else if controller.mode() == Mode::Teach {
                sequence += 1;
                let timestamp = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map_or(0, |duration| duration.as_nanos());
                let path = out_dir.join(format!("episode-{timestamp}-{sequence}.jsonl"));
                let (sender, header) = (disk_tx.clone(), header.clone());
                pending = true;
                match thread::Builder::new()
                    .name("sx-teach-create".to_owned())
                    .spawn(move || {
                        let result = EpisodeWriter::create(&path, header, 1000)
                            .map_err(|error| error.to_string());
                        let _ = sender.send(DiskResult::Created(path, result));
                    }) {
                    Ok(_) => notice(&notices, "Preparing recording..."),
                    Err(error) => {
                        pending = false;
                        controller.fault(&error.to_string());
                        last_error = Some(error.to_string());
                        notice(&notices, error.to_string());
                    }
                }
            } else {
                notice(&notices, "Press t to enter teaching before recording.");
            }
        }
        if action.is_some_and(|action| matches!(action, Action::Release))
            && (capture.is_some() || pending)
        {
            notice(&notices, "Finish the recording before releasing torque.");
            action = Some(Action::Hold);
        }
        if capture.is_some() && matches!(action, Some(Action::Replay)) {
            notice(&notices, "Finish the recording before replaying.");
            action = None;
        }
        if matches!(action, Some(Action::Replay)) && !controller.replay_ready() {
            notice(
                &notices,
                "Replay is unavailable or still being checked. Wait for the ready message.",
            );
            action = None;
        }
        if action == Some(Action::Replay)
            && controller.mode() == Mode::Hold
            && !controller.engagement_ready()
        {
            notice(
                &notices,
                "Keep supporting the arm during the gravity ramp, then press p again.",
            );
        }
        let mut held_successfully = false;
        match controller.tick(action) {
            Ok(snapshot) => {
                held_successfully = snapshot.mode == Mode::Hold;
                if !matches!(snapshot.mode, Mode::Fault | Mode::Released) {
                    last_error = None;
                    reported_faults.clear();
                }
                if let Some(take) = capture.as_mut()
                    && matches!(snapshot.mode, Mode::Teach | Mode::Hold)
                {
                    let now = snapshot.measured_at;
                    let elapsed = take
                        .began
                        .map_or(0.0, |began| now.duration_since(began).as_secs_f64());
                    take.began.get_or_insert(now);
                    if let Err(error) =
                        take.writer
                            .record(elapsed, &snapshot.position, &snapshot.velocity)
                    {
                        controller.fault(&error.to_string());
                        last_error = Some(error.to_string());
                        notice(&notices, error.to_string());
                    } else {
                        take.samples += 1;
                    }
                }
                if let Some(status) = telemetry.as_mut() {
                    status.publish(
                        controller.mode(),
                        &snapshot.position,
                        snapshot.measured_at,
                        last_error.as_deref(),
                        controller.mode() == Mode::Released,
                        Some(&controller.diagnostics()),
                    );
                }
            }
            Err(error) => {
                // Keep the initiating fault visible while secondary hold diagnostics evolve.
                last_error.get_or_insert_with(|| error.clone());
                if let Some(status) = telemetry.as_mut() {
                    // No new observation is vouched for on a failed control tick.
                    status.publish(
                        controller.mode(),
                        &[],
                        Instant::now(),
                        last_error.as_deref(),
                        false,
                        Some(&controller.diagnostics()),
                    );
                }
                let category = error
                    .split_once(':')
                    .map_or(error.as_str(), |(prefix, _)| prefix);
                if !reported_faults.contains(category) {
                    reported_faults.insert(category.to_owned());
                    notice(&notices, format!("Stopped: {error}"));
                }
            }
        }
        if capture.is_some()
            && (!matches!(controller.mode(), Mode::Teach | Mode::Hold) || finish_requested)
        {
            let take = capture.take().expect("capture exists");
            // A stop/fault is not a successfully completed demonstration.
            if !held_successfully || take.samples < 2 {
                drop(take);
                notice(&notices, "Recording interrupted; incomplete take retained.");
            } else {
                pending = true;
                if let Err(error) = finalize(take, header.clone(), disk_tx.clone()) {
                    pending = false;
                    controller.fault(&error);
                    last_error = Some(error.clone());
                    notice(&notices, error);
                }
            }
        }
        let ready = controller.replay_ready();
        if ready && !last_replay_ready {
            notice(
                &notices,
                format!(
                    "Recording checked. Replay {:.3}x. Press p to approach its starting pose.",
                    controller.playback_rate()
                ),
            );
        }
        last_replay_ready = ready;
        let mode = controller.mode();
        if mode != last_mode {
            notice(&notices, format!("Mode: {mode:?}"));
            last_mode = mode;
        }
        if mode == Mode::Released {
            notice(&notices, "Torque released. Exiting.");
            if let Some(status) = telemetry {
                status.finish();
            }
            return Ok(());
        }
        next += PERIOD;
        if let Some(wait) = next.checked_duration_since(Instant::now()) {
            thread::sleep(wait);
        } else {
            next = Instant::now();
        }
    }
}

fn run() -> Result<(), String> {
    let Some(args) = parse()? else {
        return Ok(());
    };
    run_args(args)
}

#[allow(clippy::too_many_lines)] // Startup ordering keeps signal blocking before every hardware thread.
fn run_args(args: Args) -> Result<(), String> {
    let snapshot = inputs(&args)?;
    if args.check_config {
        println!(
            "Local configuration parsed. Configuration {}. No hardware opened; live control preflight has not run.",
            snapshot.config_sha
        );
        return Ok(());
    }
    if !args.moving {
        let _observer = snapshot.observe()?;
        let configuration_sha256 = &snapshot.config_sha;
        println!(
            "Local files parsed and read-only observer opened. Configuration {configuration_sha256}. No motors enabled.\nLive motion preflight runs with --move before engagement. Use supported arms and an attending operator."
        );
        return Ok(());
    }
    if !io::stdin().is_terminal() || !io::stdout().is_terminal() {
        return Err("--move requires a local interactive terminal".into());
    }
    std::fs::create_dir_all(&args.out_dir).map_err(|error| error.to_string())?;
    // Block handled signals before any hardware/control threads are created.
    let mut signals = SigSet::empty();
    for signal in [
        Signal::SIGINT,
        Signal::SIGTERM,
        Signal::SIGHUP,
        Signal::SIGQUIT,
    ] {
        signals.add(signal);
    }
    signal::pthread_sigmask(SigmaskHow::SIG_BLOCK, Some(&signals), None)
        .map_err(|error| error.to_string())?;
    let interrupted = Arc::new(AtomicBool::new(false));
    let exit_requested = Arc::new(AtomicBool::new(false));
    {
        let interrupted = Arc::clone(&interrupted);
        let exit_requested = Arc::clone(&exit_requested);
        thread::Builder::new()
            .name("sx-teach-signals".to_owned())
            .spawn(move || {
                while let Ok(signal) = signals.wait() {
                    request_signal(signal, &exit_requested, &interrupted);
                }
                interrupted.store(true, Ordering::Release);
            })
            .map_err(|error| error.to_string())?;
    }
    let mut controller_config = Config {
        joints: args.joints,
        held: args.held,
        speed: args.speed,
        share: args.share,
        damping: args.damping,
        period: PERIOD,
        max_seconds: args.max_seconds,
        grippers: snapshot.grippers.clone(),
    };
    let body = snapshot.open()?;
    controller_config.grippers = opened_grippers(&snapshot.grippers, &snapshot.drive, body.axes())?;
    let ranges: BTreeMap<_, _> = body
        .axes()
        .iter()
        .zip(body.commandable())
        .filter(|(_, range)| range.lower.is_finite() && range.upper.is_finite())
        .map(|(axis, range)| {
            (
                axis.joint.clone(),
                StatusRange {
                    lower: range.lower,
                    upper: range.upper,
                },
            )
        })
        .collect();
    let unavailable = body.unavailable();
    let status_joints = body
        .axes()
        .iter()
        .enumerate()
        .filter(|(index, _)| !unavailable.contains(index))
        .map(|(_, axis)| axis.joint.clone())
        .collect();
    let mut telemetry = args
        .status_file
        .map(|path| StatusPublisher::spawn(path, status_joints, ranges))
        .transpose()?;
    let mut controller =
        match Controller::with_guidance(body, snapshot.model, controller_config, snapshot.guidance)
        {
            Ok(controller) => controller,
            Err(error) => {
                if let Some(status) = telemetry.as_mut() {
                    status.publish(Mode::Fault, &[], Instant::now(), Some(&error), true, None);
                }
                if let Some(status) = telemetry {
                    status.finish();
                }
                return Err(error);
            }
        };
    let header = Header::new(
        snapshot.config_sha,
        controller.joints(),
        PERIOD.as_secs_f64(),
    )
    .map_err(|error| error.to_string())?;
    if let Some(path) = &args.replay {
        controller.set_episode(Episode::load(path, &header).map_err(|error| error.to_string())?)?;
    }
    println!(
        "{HELP}\n\nArms must be supported before enabling or releasing torque. Starting disarmed."
    );
    let _terminal = Terminal::raw()?;
    let lost = Arc::new(AtomicBool::new(false));
    let (input_tx, input_rx) = sync_channel(INPUT_CAPACITY);
    let (notices_tx, notices_rx) = sync_channel(32);
    {
        let lost = Arc::clone(&lost);
        let exit_requested = Arc::clone(&exit_requested);
        thread::spawn(move || {
            let mut byte = [0_u8];
            let mut escape = 0;
            loop {
                if io::stdin().read_exact(&mut byte).is_err() {
                    lost.store(true, Ordering::Release);
                    break;
                }
                let Some(event) = key(byte[0], &mut escape) else {
                    continue;
                };
                if !queue_input(event, &input_tx, &exit_requested, &lost) {
                    break;
                }
            }
        });
    }
    let status_lost = Arc::clone(&lost);
    let worker = thread::spawn(move || {
        control(
            controller,
            header,
            args.out_dir,
            input_rx,
            notices_tx,
            lost,
            interrupted,
            exit_requested,
            telemetry,
        )
    });
    let mut output = io::stdout().lock();
    while let Ok(message) = notices_rx.recv() {
        if write!(output, "{message}\r\n")
            .and_then(|()| output.flush())
            .is_err()
        {
            status_lost.store(true, Ordering::Release);
        }
    }
    worker
        .join()
        .map_err(|_| "motor control thread panicked".to_owned())?
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-teach: {error}");
            ExitCode::FAILURE
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn terminal_arrows_and_alt_sequences_never_issue_motor_actions() {
        let mut escape = 0;
        for byte in b"\x1b[A\x1b[B\x1b[C\x1b[D\x1b[1;5A\x1bOp\x1bp\x1br\x1b+" {
            assert!(key(*byte, &mut escape).is_none());
        }
        assert!(
            matches!(key(b'+', &mut escape), Some(Input::Action(Action::GripperStep(step))) if (step-0.05).abs()<1e-12)
        );
        assert_eq!(key(b'r', &mut escape), Some(Input::Record));
    }
    #[test]
    fn escape_then_space_always_stops_and_resets_decoder() {
        let mut escape = 0;
        assert!(key(27, &mut escape).is_none());
        assert_eq!(key(b' ', &mut escape), Some(Input::Action(Action::Stop)));
        assert_eq!(escape, 0);
        assert_eq!(key(b'e', &mut escape), Some(Input::Action(Action::Engage)));
    }

    #[test]
    fn unfinished_csi_cannot_swallow_repeated_stop_keys() {
        let mut escape = 0;
        assert!(key(27, &mut escape).is_none());
        assert!(key(b'[', &mut escape).is_none());
        for byte in [b' ', b' ', b'q'] {
            assert_eq!(key(byte, &mut escape), Some(Input::Action(Action::Stop)));
            assert_eq!(escape, 0);
        }
    }

    #[test]
    fn ctrl_c_preempts_every_escape_state_and_queued_control() {
        for mut escape in 0..=2 {
            assert_eq!(key(3, &mut escape), Some(Input::Action(Action::Exit)));
            assert_eq!(escape, 0);
        }
        for exit_position in 0..=4 {
            let (sender, receiver) = sync_channel(8);
            let mut events = vec![
                Input::Action(Action::Stop),
                Input::Record,
                Input::Action(Action::Replay),
                Input::Action(Action::Engage),
            ];
            events.insert(exit_position, Input::Action(Action::Exit));
            for event in events {
                sender.send(event).unwrap();
            }
            assert_eq!(
                controls(&receiver, &AtomicBool::new(false)),
                (Some(Action::Exit), false)
            );
        }
    }

    #[test]
    fn ctrl_c_survives_a_full_queue_without_starting_or_finishing_a_recording() {
        let (sender, receiver) = sync_channel(1);
        let exit_requested = AtomicBool::new(false);
        let lost = AtomicBool::new(false);
        assert!(queue_input(Input::Record, &sender, &exit_requested, &lost));
        assert!(queue_input(
            Input::Action(Action::Exit),
            &sender,
            &exit_requested,
            &lost
        ));
        assert!(!lost.load(Ordering::Acquire));
        assert!(queue_input(Input::Record, &sender, &exit_requested, &lost));
        assert!(lost.load(Ordering::Acquire));
        assert_eq!(
            controls(&receiver, &exit_requested),
            (Some(Action::Exit), false)
        );
    }

    #[test]
    fn controls_checks_exit_before_draining_and_bounds_work_per_tick() {
        for exiting in [true, false] {
            let (sender, receiver) = sync_channel(INPUT_CAPACITY * 2);
            for _ in 0..INPUT_CAPACITY * 2 {
                sender.send(Input::Record).unwrap();
            }
            let requested = AtomicBool::new(exiting);
            let result = controls(&receiver, &requested);
            if exiting {
                assert_eq!(result, (Some(Action::Exit), false));
                assert_eq!(receiver.try_iter().count(), INPUT_CAPACITY * 2);
            } else {
                assert_eq!(result, (None, true));
                assert_eq!(receiver.try_iter().count(), INPUT_CAPACITY);
            }
        }
    }

    #[test]
    fn only_sigint_requests_release_while_other_signals_preserve_hold() {
        for signal in [Signal::SIGTERM, Signal::SIGHUP, Signal::SIGQUIT] {
            let exit_requested = AtomicBool::new(false);
            let interrupted = AtomicBool::new(false);
            request_signal(signal, &exit_requested, &interrupted);
            assert!(!exit_requested.load(Ordering::Acquire));
            assert!(interrupted.load(Ordering::Acquire));
        }
        let exit_requested = AtomicBool::new(false);
        let interrupted = AtomicBool::new(false);
        request_signal(Signal::SIGINT, &exit_requested, &interrupted);
        assert!(exit_requested.load(Ordering::Acquire));
        assert!(!interrupted.load(Ordering::Acquire));
    }

    fn snapshot_fixture(directory: &Path) -> Args {
        std::fs::create_dir_all(directory).unwrap();
        let drives: serde_json::Value = serde_json::from_str(include_str!(
            "../../../sx-damiao-can/tests/fixtures/drives.json"
        ))
        .unwrap();
        let mut drive = drives["sentient-static-upper-body"]["nominal"].clone();
        drive["joints"][0]["zero"] = serde_json::json!(0.125);
        let commissioning = serde_json::json!({"schema":"sx.station-commissioning/v1", "instance":drive["instance"], "pod_id":"snapshot-test", "pod_revision":1, "ports":{"can":"CAN", "gripper":"SERVO", "neck":"NECK"}});
        std::fs::write(directory.join("drive.json"), drive.to_string()).unwrap();
        std::fs::write(
            directory.join("commissioning.json"),
            commissioning.to_string(),
        )
        .unwrap();
        let urdf = "<robot name='snapshot'><link name='base'/></robot>";
        std::fs::write(directory.join("body.urdf"), urdf).unwrap();
        let calibration = serde_json::json!({"schema":"sx.calibration/v1", "kind":"dynamics", "description_sha256":sha256_hex(urdf), "stiction_velocity":0.01, "links":[], "joints":[], "units":[], "measurement":{"method":"synthetic"}});
        std::fs::write(directory.join("dynamics.json"), calibration.to_string()).unwrap();
        let servo = |offset| {
            serde_json::json!({"finger":{"id":1,"drive_mode":0,"homing_offset":offset,"range_min":10,"range_max":200}}).to_string()
        };
        std::fs::write(directory.join("explicit.json"), servo(13)).unwrap();
        std::fs::write(directory.join("neck.calibration.json"), servo(31)).unwrap();
        // An explicit calibration replaces automatic data for its port completely.
        std::fs::write(
            directory.join("gripper.calibration.json"),
            "unusable old calibration",
        )
        .unwrap();
        Args {
            body: BodyFlags {
                drive: directory.join("drive.json").to_str().unwrap().to_owned(),
                commissioning: directory
                    .join("commissioning.json")
                    .to_str()
                    .unwrap()
                    .to_owned(),
                calibrations: vec![(
                    "gripper".to_owned(),
                    directory.join("explicit.json").to_str().unwrap().to_owned(),
                )],
                ..BodyFlags::default()
            },
            urdf: directory.join("body.urdf"),
            dynamics: vec![directory.join("dynamics.json")],
            guidance: None,
            joints: vec!["left_shoulder_pitch_joint".to_owned()],
            held: BTreeMap::new(),
            out_dir: directory.join("episodes"),
            replay: None,
            moving: false,
            check_config: false,
            status_file: None,
            speed: 0.2,
            share: 0.25,
            damping: 0.5,
            max_seconds: 60.0,
        }
    }

    #[test]
    fn snapshot_checks_reference_fit_with_its_certified_companion() {
        let directory = temp_directory();
        let mut args = snapshot_fixture(&directory);
        let file = directory.join("dynamics.json");
        let mut reference: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(&file).unwrap()).unwrap();
        let source = "a".repeat(64);
        reference["measurement"] = serde_json::json!({
            "method":"reference_model_fit",
            "residual":{"value":0.0,"unit":"newton_metre"},
            "evidence":[{"kind":"certified_beside","reference":source}]
        });
        std::fs::write(&file, reference.to_string()).unwrap();
        let Err(error) = inputs(&args) else {
            panic!("missing companion accepted")
        };
        assert!(error.contains("certified"), "{error}");
        let mut companion = reference.clone();
        companion["measurement"] = serde_json::json!({
            "method":"synthetic",
            "evidence":[{"kind":"sealed_document","reference":source}]
        });
        let companion_file = directory.join("companion.json");
        std::fs::write(&companion_file, companion.to_string()).unwrap();
        args.dynamics.push(companion_file.clone());
        assert!(inputs(&args).is_ok(), "the certified pair must load");
        companion["measurement"]["evidence"][0]["reference"] = serde_json::json!("b".repeat(64));
        std::fs::write(&companion_file, companion.to_string()).unwrap();
        let Err(error) = inputs(&args) else {
            panic!("wrong companion accepted")
        };
        assert!(error.contains("certified"), "{error}");
        std::fs::remove_dir_all(directory).unwrap();
    }

    fn temp_directory() -> PathBuf {
        static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        std::env::temp_dir().join(format!(
            "sx-teach-snapshot-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ))
    }

    #[test]
    fn guidance_snapshot_is_bound_to_exact_inputs_and_profile_bytes() {
        let directory = temp_directory();
        let mut args = snapshot_fixture(&directory);
        let urdf = "<robot name='guidance-snapshot'><link name='base'/><link name='arm'/><joint name='left_shoulder_pitch_joint' type='revolute'><parent link='base'/><child link='arm'/><axis xyz='0 1 0'/><limit lower='-3' upper='3' effort='100' velocity='5'/></joint></robot>";
        std::fs::write(&args.urdf, urdf).unwrap();
        let mut calibration: serde_json::Value =
            serde_json::from_str(&read(&args.dynamics[0]).unwrap()).unwrap();
        calibration["description_sha256"] = serde_json::json!(sha256_hex(urdf));
        std::fs::write(&args.dynamics[0], calibration.to_string()).unwrap();
        let mut profile = serde_json::json!({"schema":"sx.guidance/v1", "description_sha256":sha256_hex(urdf),
            "drive_sha256":sha256_hex(&read(Path::new(&args.body.drive)).unwrap()),
            "gravity":[0.0,0.0,-9.81], "model_urdf":urdf, "joints":[{
                "joint":"left_shoulder_pitch_joint", "coordinate_gain":1.0, "coordinate_offset":0.0,
                "gravity_scale":0.95,"damping":0.45,"friction_coulomb":0.2,"friction_scale":0.0,
                "torque_limit":null,"torque_slew_rate":20.0}]});
        let path = directory.join("guidance.json");
        args.guidance = Some(path.clone());
        std::fs::write(&path, profile.to_string()).unwrap();
        let snapshot = inputs(&args).unwrap();
        profile["joints"][0]["damping"] = serde_json::json!(0.30);
        std::fs::write(&path, profile.to_string()).unwrap();
        assert_ne!(snapshot.config_sha, inputs(&args).unwrap().config_sha);
        assert!(
            (snapshot
                .guidance
                .unwrap()
                .joint_settings("left_shoulder_pitch_joint")
                .unwrap()
                .damping
                - 0.45)
                .abs()
                < 1e-12
        );
        profile["drive_sha256"] = serde_json::json!("f".repeat(64));
        std::fs::write(&path, profile.to_string()).unwrap();
        assert!(inputs(&args).is_err_and(|error| error.contains("drive_sha256")));
        profile["drive_sha256"] =
            serde_json::json!(sha256_hex(&read(Path::new(&args.body.drive)).unwrap()));
        profile["description_sha256"] = serde_json::json!("f".repeat(64));
        std::fs::write(&path, profile.to_string()).unwrap();
        assert!(inputs(&args).is_err_and(|error| error.contains("description_sha256")));
        std::fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn changing_files_cannot_change_the_hashed_hardware_snapshot() {
        let directory = temp_directory();
        let args = snapshot_fixture(&directory);
        let snapshot = inputs(&args).unwrap();
        let identity = snapshot.config_sha.clone();
        let mut drive: serde_json::Value =
            serde_json::from_str(&read(Path::new(&args.body.drive)).unwrap()).unwrap();
        drive["joints"][0]["zero"] = serde_json::json!(1.25);
        std::fs::write(&args.body.drive, drive.to_string()).unwrap();
        let mut commissioning: serde_json::Value =
            serde_json::from_str(&read(Path::new(&args.body.commissioning)).unwrap()).unwrap();
        commissioning["unidentified"] = serde_json::json!(["left_shoulder_pitch_joint"]);
        std::fs::write(&args.body.commissioning, commissioning.to_string()).unwrap();
        std::fs::write(
            directory.join("explicit.json"),
            "replaced while operator reads instructions",
        )
        .unwrap();
        std::fs::write(directory.join("neck.calibration.json"), "also replaced").unwrap();
        assert!((snapshot.drive.joints[0].zero - 0.125).abs() < 1e-12);
        assert!(snapshot.commissioning.unidentified.is_empty());
        assert_eq!(
            snapshot
                .calibrations
                .of("gripper", 1)
                .unwrap()
                .homing_offset,
            13
        );
        assert_eq!(
            snapshot.calibrations.of("neck", 1).unwrap().homing_offset,
            31
        );
        assert_eq!(snapshot.config_sha, identity);
        assert!(inputs(&args).is_err_and(|error| error.contains("unidentified")));
        // The opener accepts these already-parsed members; no path is retained in Inputs.
        std::fs::remove_dir_all(directory).unwrap();
    }

    #[test]
    fn historical_urdf_is_digest_checked_before_calibration_compatibility() {
        let directory = temp_directory();
        let args = snapshot_fixture(&directory);
        let old = "<robot name='snapshot-old'><link name='base'/></robot>";
        let digest = sha256_hex(old);
        let historical = directory.join("_by_digest").join(&digest);
        std::fs::create_dir_all(&historical).unwrap();
        std::fs::write(historical.join("body.urdf"), old).unwrap();
        let mut calibration: serde_json::Value =
            serde_json::from_str(&read(&args.dynamics[0]).unwrap()).unwrap();
        calibration["description_sha256"] = serde_json::json!(digest);
        std::fs::write(&args.dynamics[0], calibration.to_string()).unwrap();
        assert!(inputs(&args).is_ok());
        std::fs::write(
            historical.join("body.urdf"),
            "<robot name='wrong'><link name='base'/></robot>",
        )
        .unwrap();
        assert!(inputs(&args).is_err_and(|error| error.contains("does not match")));
        calibration["description_sha256"] = serde_json::json!("../../another-file");
        std::fs::write(&args.dynamics[0], calibration.to_string()).unwrap();
        assert!(inputs(&args).is_err_and(|error| error.contains("64 hexadecimal")));
        std::fs::remove_dir_all(directory).unwrap();
    }
    #[test]
    fn canonical_static_body_grippers_open_mirrored_fingers_in_opposite_directions() {
        let files =
            sx_embodiment_ik::registered("sentient-static-upper-body").expect("static body pack");
        let drives: serde_json::Value = serde_json::from_str(include_str!(
            "../../../sx-damiao-can/tests/fixtures/drives.json"
        ))
        .unwrap();
        let raw = &drives["sentient-static-upper-body"]["nominal"];
        let drive =
            InstanceDrive::parse(&raw.to_string(), raw["instance"].as_str().unwrap()).unwrap();
        let mut documents = BTreeMap::new();
        let grippers = canonical_grippers(
            &sha256_hex(files.urdf),
            &drive,
            &BTreeSet::new(),
            &mut documents,
        )
        .unwrap();
        assert_eq!(grippers.len(), 2);
        for gripper in grippers {
            assert_eq!(gripper.fingers.len(), 2);
            assert!(
                (gripper.fingers[0].2 - gripper.fingers[0].1)
                    * (gripper.fingers[1].2 - gripper.fingers[1].1)
                    < 0.0
            );
            let mut closed = vec![0.0; drive.joints.len()];
            let mut opened = closed.clone();
            gripper.write(gripper.closed_width_m, &mut closed);
            gripper.write(gripper.open_width_m, &mut opened);
            for (axis, _, _) in &gripper.fingers {
                assert!(drive.joints[*axis].joint.contains("finger"));
                assert!((opened[*axis] - closed[*axis]).abs() > 0.1);
            }
        }
        assert_eq!(documents["gripper-render-pack"], files.pack);
        assert_eq!(documents["gripper-collision"], files.collision);
        assert!(
            canonical_grippers(
                &"00".repeat(32),
                &drive,
                &BTreeSet::new(),
                &mut BTreeMap::new()
            )
            .unwrap()
            .is_empty()
        );
    }
    #[test]
    fn file_only_dispatch_accepts_valid_config_with_no_physical_bus() {
        let directory = temp_directory();
        let mut args = snapshot_fixture(&directory);
        let mut commissioning: serde_json::Value =
            serde_json::from_str(&read(Path::new(&args.body.commissioning)).unwrap()).unwrap();
        for value in commissioning["ports"].as_object_mut().unwrap().values_mut() {
            *value =
                serde_json::json!(format!("NO-PHYSICAL-TEACH-TEST-BUS-{}", std::process::id()));
        }
        std::fs::write(&args.body.commissioning, commissioning.to_string()).unwrap();
        args.check_config = true;
        let status_file = directory.join("must-not-be-written.json");
        args.status_file = Some(status_file.clone());
        // Exercise the production startup dispatch. Opening the observer would fail against
        // these nonexistent bus identities; check-config must return before that operation.
        assert!(run_args(args).is_ok());
        assert!(!status_file.exists());
        std::fs::remove_dir_all(directory).unwrap();
    }
    #[test]
    fn disjoint_fits_allow_different_stiction_velocities_but_still_refuse_overlap() {
        let directory = temp_directory();
        let mut args = snapshot_fixture(&directory);
        let urdf = "<robot name='two-arm-friction'><link name='base'/><link name='left'/><link name='right'/><joint name='left_shoulder_pitch_joint' type='revolute'><parent link='base'/><child link='left'/><axis xyz='0 1 0'/><limit lower='-1' upper='1' effort='10' velocity='1'/></joint><joint name='right_shoulder_pitch_joint' type='revolute'><parent link='base'/><child link='right'/><axis xyz='0 1 0'/><limit lower='-1' upper='1' effort='10' velocity='1'/></joint></robot>";
        std::fs::write(&args.urdf, urdf).unwrap();
        let fit = |joint, velocity| {
            serde_json::json!({
                "schema":"sx.calibration/v1", "kind":"dynamics",
                "description_sha256":sha256_hex(urdf), "stiction_velocity":velocity,
                "links":[], "joints":[{"joint":joint, "coulomb":0.1,
                    "viscous":0.01,"armature":0.0,"stribeck":0.0,
                    "stribeck_velocity":0.1,"stribeck_exponent":2.0,
                    "load_motor":0.0,"load_external":0.0,
                    "load_motor_stribeck":0.0,"load_external_stribeck":0.0}],
                "units":[], "measurement":{"method":"synthetic"}
            })
            .to_string()
        };
        std::fs::write(&args.dynamics[0], fit("left_shoulder_pitch_joint", 0.02)).unwrap();
        let right = directory.join("right-arm.dynamics.json");
        std::fs::write(&right, fit("right_shoulder_pitch_joint", 0.04)).unwrap();
        args.dynamics.push(right.clone());
        assert!(inputs(&args).is_ok());
        std::fs::write(right, fit("left_shoulder_pitch_joint", 0.04)).unwrap();
        assert!(inputs(&args).is_err_and(|error| error.contains("dynamics files overlap")));
        std::fs::remove_dir_all(directory).unwrap();
    }
    #[test]
    fn status_file_publishes_only_finite_observations_with_hardware_ranges() {
        let directory = temp_directory();
        std::fs::create_dir_all(&directory).unwrap();
        let path = directory.join("status.json");
        let mut args = snapshot_fixture(&directory);
        let before = inputs(&args).unwrap().config_sha;
        args.status_file = Some(path.clone());
        assert_eq!(inputs(&args).unwrap().config_sha, before);
        let joints = vec!["elbow".to_owned(), "unavailable".to_owned()];
        let ranges = BTreeMap::from([(
            "elbow".to_owned(),
            StatusRange {
                lower: -1.0,
                upper: 1.0,
            },
        )]);
        let mut frame = StatusFrame {
            mode: Mode::Teach,
            position: [0.0; sx_embodiment_commission::teach_episode::MAX_AXES],
            width: 2,
            updated_at_unix_ms: 1234,
            error: None,
            diagnostics: Some(
                [JointDiagnostic::default(); sx_embodiment_commission::teach_episode::MAX_AXES],
            ),
        };
        frame.position[0] = 0.25;
        frame.position[1] = f64::NAN;
        let received = Instant::now()
            .checked_sub(Duration::from_millis(20))
            .expect("20 ms clock history");
        frame.diagnostics.as_mut().unwrap()[0] = JointDiagnostic {
            calculated_gravity_nm: Some(-0.4),
            support_offset_nm: Some(0.1),
            completed_gravity_command_nm: Some(-0.3),
            position_target_rad: Some(0.25),
            feedback_velocity_rad_s: Some(-0.75),
            feedback_torque_nm: Some(6.0),
            model_observed_at: Some(received),
            gravity_command_completed_at: Some(received),
            feedback_received_at: Some(received),
        };
        frame.diagnostics.as_mut().unwrap()[1].feedback_torque_nm = Some(f64::NAN);
        write_status(&path, &joints, &ranges, &frame).unwrap();
        let observed: serde_json::Value = serde_json::from_str(&read(&path).unwrap()).unwrap();
        assert_eq!(observed["mode"], "teach");
        assert_eq!(observed["positions"].as_object().unwrap().len(), 1);
        assert!((observed["positions"]["elbow"].as_f64().unwrap() - 0.25).abs() < 1e-12);
        assert_eq!(observed["ranges"]["elbow"]["lower"], -1.0);
        assert_eq!(observed["updated_at_unix_ms"], 1234);
        assert_eq!(observed["diagnostics"]["elbow"]["feedback_torque_nm"], 6.0);
        assert_eq!(observed["diagnostics"]["elbow"]["support_offset_nm"], 0.1);
        assert_eq!(
            observed["diagnostics"]["elbow"]["feedback_velocity_rad_s"],
            -0.75
        );
        assert!(
            observed["diagnostics"]["elbow"]["feedback_age_ms"]
                .as_f64()
                .unwrap()
                >= 20.0
        );
        assert!(observed["diagnostics"]["unavailable"]["feedback_torque_nm"].is_null());
        assert!(observed["diagnostics"]["unavailable"]["feedback_age_ms"].is_null());
        frame.mode = Mode::Fault;
        frame.error = Some("feedback expired".to_owned());
        frame.width = 0;
        write_status(&path, &joints, &ranges, &frame).unwrap();
        let fault: serde_json::Value = serde_json::from_str(&read(&path).unwrap()).unwrap();
        assert_eq!(fault["mode"], "fault");
        assert_eq!(fault["error"], "feedback expired");
        assert!(fault["positions"].as_object().unwrap().is_empty());
        assert!(!std::fs::read_dir(&directory).unwrap().any(|entry| {
            entry
                .unwrap()
                .file_name()
                .to_string_lossy()
                .ends_with(".tmp")
        }));
        std::fs::remove_dir_all(directory).unwrap();
    }
    #[test]
    fn file_snapshot_isolates_grippers_with_the_commissioned_exclusions() {
        let directory = temp_directory();
        let mut args = snapshot_fixture(&directory);
        let files = sx_embodiment_ik::registered("sentient-static-upper-body").unwrap();
        std::fs::write(&args.urdf, files.urdf).unwrap();
        let mut calibration: serde_json::Value =
            serde_json::from_str(&read(&args.dynamics[0]).unwrap()).unwrap();
        calibration["description_sha256"] = serde_json::json!(sha256_hex(files.urdf));
        std::fs::write(&args.dynamics[0], calibration.to_string()).unwrap();
        let raw_drive: serde_json::Value =
            serde_json::from_str(&read(Path::new(&args.body.drive)).unwrap()).unwrap();
        let joints: Vec<String> = raw_drive["joints"]
            .as_array()
            .unwrap()
            .iter()
            .map(|joint| joint["joint"].as_str().unwrap().to_owned())
            .collect();
        for (unselected, selected) in [("right_", "left_"), ("left_", "right_")] {
            args.body.exclude = joints
                .iter()
                .filter(|joint| {
                    joint.starts_with(unselected)
                        || joint.starts_with("so101_")
                        || joint.as_str() == "torso_joint"
                })
                .cloned()
                .collect();
            args.joints = vec![format!("{selected}shoulder_pitch_joint")];
            let snapshot = inputs(&args).unwrap();
            assert_eq!(snapshot.grippers.len(), 1);
            for (axis, _, _) in &snapshot.grippers[0].fingers {
                assert!(snapshot.drive.joints[*axis].joint.starts_with(selected));
                assert!(
                    !snapshot
                        .commissioning
                        .out_of_service
                        .contains(&snapshot.drive.joints[*axis].joint)
                );
            }
        }
        args.body.exclude = joints
            .iter()
            .filter(|joint| joint.starts_with("so101_") || joint.as_str() == "torso_joint")
            .cloned()
            .collect();
        assert_eq!(inputs(&args).unwrap().grippers.len(), 2);
        args.body
            .exclude
            .push("left_gripper_left_finger_joint".to_owned());
        assert!(inputs(&args).is_err_and(|error| error.contains("partially excluded")));
        std::fs::remove_dir_all(directory).unwrap();
    }
    #[test]
    fn grippers_follow_opened_native_order_when_the_opposite_serial_port_is_closed() {
        let registry: Vec<serde_json::Value> = serde_json::from_str(include_str!(
            "../../../sx-embodiment-bodies/bindings/registry.json"
        ))
        .unwrap();
        let raw = registry
            .iter()
            .find(|body| body["embodiment"] == "sentient-static-upper-body")
            .unwrap();
        let bindings = sx_embodiment_drivers::BodyBindings::parse(&raw.to_string()).unwrap();
        let drives: serde_json::Value = serde_json::from_str(include_str!(
            "../../../sx-damiao-can/tests/fixtures/drives.json"
        ))
        .unwrap();
        let raw_drive = &drives["sentient-static-upper-body"]["nominal"];
        let drive = InstanceDrive::parse(
            &raw_drive.to_string(),
            raw_drive["instance"].as_str().unwrap(),
        )
        .unwrap();
        let files = sx_embodiment_ik::registered("sentient-static-upper-body").unwrap();
        for side in ["left", "right", "both"] {
            let excluded: BTreeSet<_> = drive
                .joints
                .iter()
                .filter(|joint| {
                    (side == "left" && joint.joint.starts_with("right_gripper_"))
                        || (side == "right" && joint.joint.starts_with("left_gripper_"))
                })
                .map(|joint| joint.joint.clone())
                .collect();
            let grippers = canonical_grippers(
                &sha256_hex(files.urdf),
                &drive,
                &excluded,
                &mut BTreeMap::new(),
            )
            .unwrap();
            // The actual opener omits all axes on closed serial ports, retaining the CAN
            // chain (including masked axes). Use its registry order, without inventing axes.
            let axes: Vec<_> = bindings
                .axes
                .iter()
                .filter(|axis| {
                    axis.port == "can"
                        || side == "both" && axis.port.ends_with("_gripper")
                        || axis.port == format!("{side}_gripper")
                })
                .cloned()
                .collect();
            let mapped = opened_grippers(&grippers, &drive, &axes).unwrap();
            assert_eq!(axes.len(), if side == "both" { 19 } else { 17 });
            assert_eq!(mapped.len(), if side == "both" { 2 } else { 1 });
            if side != "both" {
                assert_eq!(
                    mapped[0]
                        .fingers
                        .iter()
                        .map(|finger| finger.0)
                        .collect::<Vec<_>>(),
                    vec![15, 16]
                );
            }
            let mut commanded = vec![0.123_456; axes.len()];
            for gripper in &mapped {
                gripper.write(gripper.open_width_m, &mut commanded);
            }
            for (axis, value) in axes.iter().zip(&commanded) {
                if axis.gripper {
                    assert!((value - 0.123_456).abs() > 1e-6);
                } else {
                    assert!((value - 0.123_456).abs() < 1e-12);
                }
            }
            assert!(
                opened_grippers(&grippers, &drive, &axes[..15])
                    .is_err_and(|error| error.contains("absent from the opened body"))
            );
        }
    }
}
