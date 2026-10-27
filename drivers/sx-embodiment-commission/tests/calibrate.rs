//! `calibrate` against the simulated STS3215 bus: every way out of a run leaves each servo
//! limp and each register as it was, a sweep stops at a stall and backs off, and a dry run
//! sends nothing but reads.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use sx_embodiment_commission::calibrate::CalibrateError;
use sx_embodiment_commission::calibrate::cli::{self, parse};
use sx_embodiment_commission::calibrate::journal::Journal;
use sx_embodiment_commission::calibrate::motion::{
    Direction, End, JogEvent, JogPlan, Sweep, Watch, drive_to, jog, sweep, sweep_finger,
};
use sx_embodiment_commission::calibrate::session::{Session, Slow};
use sx_feetech_serial::MotorCalibration;
use sx_feetech_serial::scan::is_read_only;
use sx_feetech_serial::sim::{RealTime, SimBus};

const BUS: &str = "/dev/cu.sim";

/// Two fingers with limits 1000..3000, resting at 2000, against hard stops at 1500 and 2600.
fn fingers() -> SimBus {
    let calibration = MotorCalibration {
        drive_mode: false,
        homing_offset: -40,
        range_min: 1000,
        range_max: 3000,
    };
    let bus = SimBus::sts3215(&[(1, calibration), (2, calibration)]);
    for id in [1, 2] {
        bus.stops(id, 1500, 2600);
    }
    bus
}

fn journal_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("sx-calibrate-test-{}-{name}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    dir
}

fn settings() -> Sweep {
    Sweep {
        slow: Slow {
            // Steps per second: about 0.24 turn/s, so a simulated sweep takes a few seconds.
            goal_speed: 1000,
            acceleration: 5,
            torque_limit: 250,
        },
        watch: Watch {
            load_threshold: 150,
            stall_window: Duration::from_millis(200),
            min_progress: 3,
            period: Duration::from_millis(5),
            backoff: 30,
            tolerance: 4,
            settle: Duration::from_millis(50),
        },
        lead: 20,
        max_travel: 1500,
        limit_margin: 3,
    }
}

fn session(bus: &SimBus, dir: &Path, budget: Duration) -> Session<RealTime> {
    let journal = Journal::create(dir, BUS, 1_000_000).expect("a fresh journal");
    Session::new(
        RealTime::new(bus.clone()),
        journal,
        budget,
        Arc::new(AtomicUsize::new(0)),
    )
}

fn register(bus: &SimBus, id: u8, address: usize, width: usize) -> u16 {
    let state = bus.state();
    let r = &state.servos[&id];
    if width == 1 {
        u16::from(r[address])
    } else {
        u16::from_le_bytes([r[address], r[address + 1]])
    }
}

/// Torque off, and every register a run changes as the simulated servo started.
fn untouched(bus: &SimBus, id: u8, limits: (u16, u16)) {
    assert_eq!(register(bus, id, 40, 1), 0, "servo {id} torque");
    assert_eq!(register(bus, id, 41, 1), 0, "servo {id} acceleration");
    assert_eq!(register(bus, id, 46, 2), 0, "servo {id} goal speed");
    assert_eq!(register(bus, id, 48, 2), 1000, "servo {id} torque limit");
    assert_eq!(register(bus, id, 55, 1), 0, "servo {id} lock");
    assert_eq!(
        (register(bus, id, 9, 2), register(bus, id, 11, 2)),
        limits,
        "servo {id} limits"
    );
}

/// Every `Goal_Position` written to `id`, in order.
fn goals(bus: &SimBus, id: u8) -> Vec<i32> {
    bus.state()
        .written
        .iter()
        .filter(|p| p[2] == id && p[4] == 3 && p[5] == 42)
        .map(|p| i32::from(u16::from_le_bytes([p[6], p[7]])))
        .collect()
}

#[test]
fn a_finger_is_swept_to_each_stop_backed_off_returned_and_undone() {
    let bus = fingers();
    let dir = journal_dir("sweep");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    let finger = sweep_finger(&mut session, 1, &settings(), None).expect("swept");
    assert_eq!(finger.start, 2000);
    assert_eq!((finger.low.reached, finger.high.reached), (1500, 2600));
    assert!(matches!(finger.low.end, End::Load(load) if load.abs() >= 150));
    assert!(matches!(finger.high.end, End::Load(load) if load.abs() >= 150));
    // It backed off the stop at once, and never commanded past it by more than its lead.
    assert!(
        (1525..=1535).contains(&finger.low.backed_off_to),
        "{:?}",
        finger.low
    );
    assert!(
        (2565..=2575).contains(&finger.high.backed_off_to),
        "{:?}",
        finger.high
    );
    let written = goals(&bus, 1);
    assert!(
        written.iter().all(|g| (1480..=2620).contains(g)),
        "{written:?}"
    );
    assert!((finger.returned.at - 2000).abs() <= 4);
    let calibration = finger
        .calibration(
            Some(sx_embodiment_commission::calibrate::motion::Closed::High),
            10,
        )
        .expect("both ends are stops");
    assert_eq!(
        (
            calibration.range_min,
            calibration.range_max,
            calibration.drive_mode
        ),
        (1510, 2590, true)
    );
    assert_eq!(calibration.homing_offset, -40);
    // Undone before the run moved on: limp, registers back.
    untouched(&bus, 1, (1000, 3000));
    assert!(session.close().is_clean());
    assert!(!Journal::path_for(&dir, BUS).exists(), "journal removed");
}

#[test]
fn a_sweep_inside_narrow_limits_ends_at_them_and_a_widened_one_finds_the_stops() {
    let bus = fingers();
    for id in [1, 2] {
        let mut state = bus.state();
        let r = state.servos.get_mut(&id).expect("servo");
        r[9..11].copy_from_slice(&1700_u16.to_le_bytes());
        r[11..13].copy_from_slice(&2300_u16.to_le_bytes());
    }
    let dir = journal_dir("limits");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    let narrow = sweep_finger(&mut session, 1, &settings(), None).expect("swept");
    assert_eq!(narrow.low.end, End::Limit(1700));
    assert_eq!(narrow.high.end, End::Limit(2300));
    assert_eq!(narrow.calibration(None, 10), None);
    let wide = sweep_finger(&mut session, 2, &settings(), Some(400)).expect("swept");
    assert_eq!(wide.limits, (1300, 2700));
    assert_eq!((wide.low.reached, wide.high.reached), (1500, 2600));
    // The widened limits and the Lock that kept them off the servo's flash are restored.
    untouched(&bus, 2, (1700, 2300));
    assert!(session.close().is_clean());
}

#[test]
fn an_error_mid_sweep_leaves_the_servo_limp_and_its_limits_restored() {
    let bus = fingers();
    let dir = journal_dir("deadline");
    // Slow enough (about 50 ticks a second, 500 ticks to the stop) that the deadline passes
    // mid-sweep, well after the servo is powered.
    let mut slow = settings();
    slow.slow.goal_speed = 1;
    let mut session = session(&bus, &dir, Duration::from_millis(1500));
    let result = sweep_finger(&mut session, 1, &slow, Some(200));
    assert!(
        matches!(result, Err(CalibrateError::Deadline(_))),
        "{result:?}"
    );
    assert_eq!(register(&bus, 1, 40, 1), 1, "still powered until closed");
    let report = session.close();
    assert!(report.is_clean(), "{report}");
    untouched(&bus, 1, (1000, 3000));
    assert!(!Journal::path_for(&dir, BUS).exists());
}

#[test]
fn a_panic_while_powered_unwinds_through_the_guard() {
    let bus = fingers();
    let dir = journal_dir("panic");
    let caught = catch_unwind(AssertUnwindSafe(|| {
        let mut session = session(&bus, &dir, Duration::from_secs(30));
        session.widen_limits(1, 900, 3100).expect("widened");
        session.enable(1, &settings().slow).expect("enabled");
        assert_eq!(register(&bus, 1, 40, 1), 1);
        panic!("a bug in the middle of a sweep");
    }));
    assert!(caught.is_err());
    untouched(&bus, 1, (1000, 3000));
    assert!(!Journal::path_for(&dir, BUS).exists());
}

#[test]
fn a_session_dropped_without_closing_undoes_itself() {
    let bus = fingers();
    let dir = journal_dir("drop");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    session.enable(2, &settings().slow).expect("enabled");
    drop(session);
    untouched(&bus, 2, (1000, 3000));
}

#[test]
fn a_signal_stops_the_sweep_at_its_next_step() {
    let bus = fingers();
    let dir = journal_dir("signal");
    let stop = Arc::new(AtomicUsize::new(0));
    let journal = Journal::create(&dir, BUS, 1_000_000).expect("journal");
    let mut session = Session::new(
        RealTime::new(bus.clone()),
        journal,
        Duration::from_secs(30),
        Arc::clone(&stop),
    );
    let mut slow = settings();
    slow.slow.goal_speed = 1;
    session.enable(1, &slow.slow).expect("enabled");
    let raise = std::thread::spawn({
        let stop = Arc::clone(&stop);
        move || {
            std::thread::sleep(Duration::from_millis(150));
            stop.store(15, Ordering::SeqCst);
        }
    });
    let result = sweep(&mut session, 1, Direction::Increasing, &slow);
    raise.join().expect("raised");
    assert!(
        matches!(result, Err(CalibrateError::Signal(15))),
        "{result:?}"
    );
    assert!(session.close().is_clean());
    untouched(&bus, 1, (1000, 3000));
}

#[test]
fn a_torque_off_that_does_not_read_back_is_named_and_its_journal_kept_for_restore() {
    let bus = fingers();
    let dir = journal_dir("stuck");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    session.enable(1, &settings().slow).expect("enabled");
    bus.state().ignored.insert((1, 40));
    let report = session.close();
    assert!(!report.is_clean());
    assert_eq!(report.torque_unproven.len(), 1);
    assert_eq!(report.torque_unproven[0].0, 1);
    assert!(
        report
            .to_string()
            .contains("DANGER: servo 1 may still hold torque")
    );
    // A servo that may still be powered keeps its low torque limit: restoring it would only
    // make it stronger.
    assert_eq!(register(&bus, 1, 48, 2), 250);
    let path = Journal::path_for(&dir, BUS);
    assert_eq!(report.journal_kept.as_deref(), Some(path.as_path()));
    // The next run refuses until the journal is undone.
    assert!(matches!(
        Journal::create(&dir, BUS, 1_000_000),
        Err(CalibrateError::Unrestored { .. })
    ));
    // Once the servo answers again, --restore undoes it and removes the journal.
    bus.state().ignored.clear();
    let args = parse(&["--restore".into(), path.display().to_string()]).expect("parsed");
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run(
        &args,
        |_, _| Ok(RealTime::new(bus.clone())),
        &Arc::new(AtomicUsize::new(0)),
        &mut stdout,
        &mut stderr,
    );
    assert_eq!(code, 0, "{}", String::from_utf8_lossy(&stdout));
    untouched(&bus, 1, (1000, 3000));
    assert!(!path.exists());
}

#[test]
fn an_enable_whose_speed_limit_does_not_read_back_is_refused_before_torque() {
    let bus = fingers();
    bus.state().ignored.insert((1, 46));
    let dir = journal_dir("speed");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    let result = session.enable(1, &settings().slow);
    assert!(
        matches!(
            result,
            Err(CalibrateError::ReadBack {
                id: 1,
                address: 46,
                wrote: 1000,
                read: 0
            })
        ),
        "{result:?}"
    );
    assert!(bus.state().written.iter().all(|p| p[4..7] != [3, 40, 1]));
    assert!(session.close().is_clean());
}

#[test]
fn one_servo_is_powered_at_a_time_and_a_servo_resting_outside_its_limits_is_refused() {
    let bus = fingers();
    let dir = journal_dir("one");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    session.enable(1, &settings().slow).expect("enabled");
    assert!(matches!(
        session.enable(2, &settings().slow),
        Err(CalibrateError::OnePowered { id: 2, powered: 1 })
    ));
    assert_eq!(register(&bus, 2, 40, 1), 0);
    assert!(session.close().is_clean());
    bus.rest_at(2, 900);
    let mut session = self::session(&bus, &dir, Duration::from_secs(30));
    assert!(matches!(
        session.enable(2, &settings().slow),
        Err(CalibrateError::Refused(_))
    ));
    assert_eq!(register(&bus, 2, 40, 1), 0);
    assert!(session.close().is_clean());
}

#[test]
fn a_jog_moves_out_and_back_and_times_the_move() {
    let bus = fingers();
    let dir = journal_dir("jog");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    let s = settings();
    let mut events = Vec::new();
    let plan = JogPlan {
        widen: None,
        hold: Duration::from_millis(200),
    };
    let jogged = jog(&mut session, 1, 120, &s.slow, &s.watch, &plan, &mut |e| {
        events.push(e);
    })
    .expect("jogged");
    let hold = jogged.hold.expect("held");
    assert!(hold.ended_unix - hold.started_unix >= 0.2);
    assert!((hold.min - 2120).abs() <= 4 && (hold.max - 2120).abs() <= 4);
    assert!(matches!(
        events[..],
        [
            JogEvent::Holding { id: 1, .. },
            JogEvent::Returned { id: 1, .. }
        ]
    ));
    assert_eq!((jogged.start, jogged.target), (2000, 2120));
    assert_eq!(jogged.out.end, End::Arrived);
    assert_eq!(jogged.back.end, End::Arrived);
    assert!(jogged.out.ticks_per_second().is_some());
    untouched(&bus, 1, (1000, 3000));
    // A jog into a stop backs off and is still brought back.
    let mut session = self::session_after(&bus, session);
    session.enable(1, &s.slow).expect("enabled");
    let into = drive_to(&mut session, 1, 2900, &s.watch).expect("driven");
    assert!(into.end.is_stop(), "{into:?}");
    assert_eq!(into.at, 2600);
    assert!(session.state(1).expect("read").position < 2600);
    assert!(session.close().is_clean());
}

fn session_after(bus: &SimBus, previous: Session<RealTime>) -> Session<RealTime> {
    let dir = previous
        .journal()
        .path()
        .parent()
        .expect("a directory")
        .to_owned();
    assert!(previous.close().is_clean());
    session(bus, &dir, Duration::from_secs(30))
}

#[test]
fn a_dry_run_sends_only_reads_and_starts_no_journal() {
    let bus = fingers();
    let dir = journal_dir("dry");
    let words: Vec<String> = [
        "gripper",
        "--bus",
        BUS,
        "--ids",
        "1,2",
        "--widen",
        "300",
        "--journal-dir",
        dir.to_str().expect("utf-8"),
    ]
    .map(String::from)
    .to_vec();
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run(
        &parse(&words).expect("parsed"),
        |_, _| Ok(bus.clone()),
        &Arc::new(AtomicUsize::new(0)),
        &mut stdout,
        &mut stderr,
    );
    assert_eq!(code, 0, "{}", String::from_utf8_lossy(&stderr));
    let state = bus.state();
    assert!(!state.written.is_empty());
    assert!(state.written.iter().all(|p| is_read_only(p)));
    assert!(!Journal::path_for(&dir, BUS).exists());
    assert!(String::from_utf8_lossy(&stdout).contains("widened for the sweep to 700..3300"));
}

#[test]
fn a_moving_run_refuses_to_start_over_an_earlier_runs_journal() {
    let bus = fingers();
    let dir = journal_dir("stale");
    let _earlier = Journal::create(&dir, BUS, 1_000_000).expect("an earlier run");
    let words: Vec<String> = [
        "jog",
        "--bus",
        BUS,
        "--id",
        "1",
        "--delta",
        "50",
        "--move",
        "--journal-dir",
        dir.to_str().expect("utf-8"),
    ]
    .map(String::from)
    .to_vec();
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run(
        &parse(&words).expect("parsed"),
        |_, _| Ok(bus.clone()),
        &Arc::new(AtomicUsize::new(0)),
        &mut stdout,
        &mut stderr,
    );
    assert_eq!(code, 1);
    assert!(String::from_utf8_lossy(&stderr).contains("--restore"));
    assert!(bus.state().written.is_empty());
}

#[test]
fn the_cli_refuses_a_large_jog_and_a_missing_bus() {
    let words = |w: &[&str]| w.iter().map(|s| (*s).to_owned()).collect::<Vec<_>>();
    assert!(
        parse(&words(&[
            "jog", "--bus", BUS, "--id", "1", "--delta", "900"
        ]))
        .is_err()
    );
    assert!(parse(&words(&["gripper", "--ids", "1"])).is_err());
    assert!(parse(&words(&["gripper", "--bus", BUS])).is_err());
}

#[test]
fn a_finger_the_lowered_torque_cannot_move_is_stuck_not_at_a_stop() {
    let bus = fingers();
    let dir = journal_dir("stuck-finger");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    let s = settings();
    session.enable(1, &s.slow).expect("enabled");
    // From here its goal writes are not taken: it never leaves 2000.
    bus.state().ignored.insert((1, 42));
    let end = sweep(&mut session, 1, Direction::Decreasing, &s).expect("ended");
    assert_eq!(end.end, End::Stuck);
    assert!(!end.end.is_stop());
    assert!(session.close().is_clean());
}

/// A port that drops the next `n` replies to a read of `Present_Position`, as a half-duplex
/// bus now and then loses a status packet.
struct Lossy {
    inner: RealTime,
    drop_next: Arc<AtomicUsize>,
}

impl sx_embodiment_drivers::serial::SerialPort for Lossy {
    fn clear_port(&mut self) -> std::io::Result<()> {
        self.inner.clear_port()
    }
    fn write_port(&mut self, packet: &[u8]) -> std::io::Result<usize> {
        let written = self.inner.write_port(packet)?;
        if packet[4] == 2 && packet[5] == 56 && self.drop_next.load(Ordering::SeqCst) > 0 {
            self.drop_next.fetch_sub(1, Ordering::SeqCst);
            self.inner.clear_port()?;
        }
        Ok(written)
    }
    fn read_port(&mut self, buffer: &mut [u8]) -> std::io::Result<usize> {
        self.inner.read_port(buffer)
    }
    fn timeout(&self) -> Duration {
        self.inner.timeout()
    }
    fn set_packet_timeout(&mut self, packet_length: usize) {
        self.inner.set_packet_timeout(packet_length);
    }
    fn is_packet_timeout(&mut self) -> bool {
        self.inner.is_packet_timeout()
    }
}

#[test]
fn a_lost_reply_is_read_again_and_three_in_a_row_stop_the_run() {
    let bus = fingers();
    let dir = journal_dir("lossy");
    let drop_next = Arc::new(AtomicUsize::new(0));
    let journal = Journal::create(&dir, BUS, 1_000_000).expect("journal");
    let mut session = Session::new(
        Lossy {
            inner: RealTime::new(bus.clone()),
            drop_next: Arc::clone(&drop_next),
        },
        journal,
        Duration::from_secs(30),
        Arc::new(AtomicUsize::new(0)),
    );
    session.enable(1, &settings().slow).expect("enabled");
    drop_next.store(2, Ordering::SeqCst);
    assert_eq!(
        session.state(1).expect("read on the third try").position,
        2000
    );
    drop_next.store(3, Ordering::SeqCst);
    assert!(matches!(
        session.state(1),
        Err(CalibrateError::Comm {
            id: 1,
            address: 56,
            ..
        })
    ));
    assert!(session.close().is_clean());
    untouched(&bus, 1, (1000, 3000));
}

#[test]
fn a_move_that_stops_short_without_load_holds_instead_of_backing_off() {
    let bus = fingers();
    let dir = journal_dir("short");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    let s = settings();
    session.enable(1, &s.slow).expect("enabled");
    // Friction a few ticks short of the target: goal writes are not taken any more.
    bus.state().ignored.insert((1, 42));
    let arrival = drive_to(&mut session, 1, 2010, &s.watch).expect("ended");
    assert_eq!(arrival.end, End::NoProgress);
    // Its last goal is where it stands, never backed off the way it came.
    assert_eq!(goals(&bus, 1).last(), Some(&2000));
    assert!(session.close().is_clean());
}

#[test]
fn a_stall_a_few_ticks_short_of_the_limit_is_the_limit_not_a_stop() {
    let bus = fingers();
    let dir = journal_dir("near-limit");
    let mut session = session(&bus, &dir, Duration::from_secs(30));
    let s = settings();
    session
        .widen_limits(1, 1000, 2100)
        .expect("narrowed for the test");
    session.enable(1, &s.slow).expect("enabled");
    // It moves until 2094, six ticks short of its limit, then a weak servo stops answering
    // its small error: its goal writes are no longer taken.
    let reached = drive_to(&mut session, 1, 2094, &s.watch).expect("moved");
    assert_eq!(reached.end, End::Arrived);
    bus.state().ignored.insert((1, 42));
    let end = sweep(&mut session, 1, Direction::Increasing, &s).expect("ended");
    assert_eq!(end.end, End::Limit(2100));
    assert!(session.close().is_clean());
}

#[test]
fn goto_leaves_a_servo_at_its_target_limp_even_from_outside_its_limits() {
    let bus = fingers();
    bus.rest_at(1, 3100);
    let dir = journal_dir("goto");
    let words: Vec<String> = [
        "goto",
        "--bus",
        BUS,
        "--id",
        "1",
        "--to",
        "2128",
        "--widen",
        "200",
        "--move",
        "--journal-dir",
        dir.to_str().expect("utf-8"),
    ]
    .map(String::from)
    .to_vec();
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run(
        &parse(&words).expect("parsed"),
        |_, _| Ok(RealTime::new(bus.clone())),
        &Arc::new(AtomicUsize::new(0)),
        &mut stdout,
        &mut stderr,
    );
    assert_eq!(code, 0, "{}", String::from_utf8_lossy(&stderr));
    assert!((i32::from(bus.present(1)) - 2128).abs() <= 4);
    untouched(&bus, 1, (1000, 3000));
    assert!(!Journal::path_for(&dir, BUS).exists());
}

/// Enter presses scripted against a simulated bus: the first press is the middle, as the bus
/// already rests; every poll of the range loop then puts the servos where the next step of
/// `script` says, as a person's hand would, and the press after the last step ends it.
struct HandOnBus {
    bus: SimBus,
    script: Vec<Vec<(u8, u16)>>,
    step: usize,
}

impl sx_embodiment_commission::calibrate::range::Prompt for HandOnBus {
    fn wait(&mut self) -> Result<(), CalibrateError> {
        Ok(())
    }

    fn pressed(&mut self) -> bool {
        let Some(positions) = self.script.get(self.step) else {
            return true;
        };
        for &(id, at) in positions {
            self.bus.rest_at(id, at);
        }
        self.step += 1;
        false
    }
}

/// Three neck-like servos with wide limits, each holding a different homing offset.
fn neck() -> SimBus {
    let calibration = |homing_offset| MotorCalibration {
        drive_mode: false,
        homing_offset,
        range_min: 0,
        range_max: 4095,
    };
    let bus = SimBus::sts3215(&[
        (1, calibration(0)),
        (2, calibration(-791)),
        (3, calibration(1605)),
    ]);
    bus.rest_at(1, 1000);
    bus.rest_at(2, 2109);
    bus.rest_at(3, 3000);
    bus
}

fn range_words(out: &Path, extra: &[&str]) -> Vec<String> {
    let mut words: Vec<String> = [
        "range",
        "--bus",
        BUS,
        "--ids",
        "1-3",
        "--joints",
        "pan,lift,roll",
        "--out",
        out.to_str().expect("utf-8"),
    ]
    .map(String::from)
    .to_vec();
    words.extend(extra.iter().map(|word| (*word).to_owned()));
    words
}

#[test]
fn a_range_shown_by_hand_is_recorded_in_the_middle_frame_and_nothing_is_written() {
    let bus = neck();
    let dir = journal_dir("range");
    std::fs::create_dir_all(&dir).expect("dir");
    let out = dir.join("neck.calibration.json");
    let mut hand = HandOnBus {
        bus: bus.clone(),
        script: vec![
            vec![(1, 600), (2, 1800), (3, 2800)],
            vec![(1, 1500), (2, 2600), (3, 3500)],
            vec![(1, 1200), (2, 2200), (3, 3100)],
        ],
        step: 0,
    };
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run_with(
        &parse(&range_words(&out, &["--full-turn", "3"])).expect("parsed"),
        |_, _| Ok(bus.clone()),
        &Arc::new(AtomicUsize::new(0)),
        &mut hand,
        &mut stdout,
        &mut stderr,
    );
    let printed = String::from_utf8_lossy(&stdout);
    assert_eq!(code, 0, "{printed}{}", String::from_utf8_lossy(&stderr));
    let calibration: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&out).expect("written")).expect("json");
    // pan: offset 0, middle at raw 1000, so homing_offset 1000 - 2047; its travel 600..1500
    // reads 1647..2547 in that frame.
    assert_eq!(
        calibration["pan"],
        serde_json::json!({"id": 1, "drive_mode": 0, "homing_offset": -1047,
                           "range_min": 1647, "range_max": 2547})
    );
    // lift: held offset -791, middle reading 2109 is raw 1318: homing_offset -729; 1800..2600
    // reads 1738..2538.
    assert_eq!(
        calibration["lift"],
        serde_json::json!({"id": 2, "drive_mode": 0, "homing_offset": -729,
                           "range_min": 1738, "range_max": 2538})
    );
    // roll: held offset 1605, middle reading 3000 is raw 509 (it wrapped): homing_offset
    // -1538; a full-turn joint gets 0..4095.
    assert_eq!(
        calibration["roll"],
        serde_json::json!({"id": 3, "drive_mode": 0, "homing_offset": -1538,
                           "range_min": 0, "range_max": 4095})
    );
    assert!(printed.contains("Step 1"), "{printed}");
    assert!(printed.contains("now"), "{printed}");
    let state = bus.state();
    assert!(
        state.written.iter().all(|p| is_read_only(p)),
        "nothing but reads"
    );
    assert!(!Journal::path_for(&dir, BUS).exists(), "no journal");
}

#[test]
fn a_range_refuses_while_any_listed_servo_holds_torque() {
    let bus = neck();
    bus.state().servos.get_mut(&2).expect("servo 2")[40] = 1;
    let dir = journal_dir("range-torque");
    std::fs::create_dir_all(&dir).expect("dir");
    let out = dir.join("neck.calibration.json");
    let mut hand = HandOnBus {
        bus: bus.clone(),
        script: Vec::new(),
        step: 0,
    };
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run_with(
        &parse(&range_words(&out, &[])).expect("parsed"),
        |_, _| Ok(bus.clone()),
        &Arc::new(AtomicUsize::new(0)),
        &mut hand,
        &mut stdout,
        &mut stderr,
    );
    assert_eq!(code, 1);
    assert!(String::from_utf8_lossy(&stderr).contains("[2] hold torque"));
    assert!(!out.exists());
    assert!(bus.state().written.iter().all(|p| is_read_only(p)));
    // And it is never a moving mode.
    assert!(parse(&range_words(&out, &["--move"])).is_err());
}

#[test]
fn apply_writes_the_offsets_limits_and_neck_protection_and_keeps_them() {
    let bus = neck();
    for (id, protection) in [(1_u8, (1000_u16, 310_u16, 80_u8)), (3, (500, 250, 25))] {
        let mut state = bus.state();
        let r = state.servos.get_mut(&id).expect("servo");
        r[16..18].copy_from_slice(&protection.0.to_le_bytes());
        r[28..30].copy_from_slice(&protection.1.to_le_bytes());
        r[36] = protection.2;
        r[48..50].copy_from_slice(&protection.0.to_le_bytes());
    }
    let dir = journal_dir("apply-offsets");
    std::fs::create_dir_all(&dir).expect("dir");
    let file = dir.join("measured.json");
    let robot = dir.join("robot");
    std::fs::write(
        &file,
        serde_json::json!({
            "pan": {"id": 1, "drive_mode": 0, "homing_offset": -1047, "range_min": 1647,
                    "range_max": 2547},
            "roll": {"id": 3, "drive_mode": 0, "homing_offset": 42, "range_min": 1760,
                     "range_max": 2909},
        })
        .to_string(),
    )
    .expect("file");
    let words: Vec<String> = [
        "apply",
        "--bus",
        BUS,
        "--calibration",
        file.to_str().expect("utf-8"),
        "--move",
        "--protection",
        "neck-defaults",
        "--robot",
        robot.to_str().expect("utf-8"),
        "--port",
        "neck",
        "--journal-dir",
        dir.to_str().expect("utf-8"),
    ]
    .map(String::from)
    .to_vec();
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run(
        &parse(&words).expect("parsed"),
        |_, _| Ok(bus.clone()),
        &Arc::new(AtomicUsize::new(0)),
        &mut stdout,
        &mut stderr,
    );
    let printed = String::from_utf8_lossy(&stdout);
    assert_eq!(code, 0, "{printed}{}", String::from_utf8_lossy(&stderr));
    // Homing offsets are sign-magnitude at bit 11.
    assert_eq!(register(&bus, 1, 31, 2), 1047 | 0x800);
    assert_eq!(register(&bus, 3, 31, 2), 42);
    assert_eq!(
        (register(&bus, 1, 9, 2), register(&bus, 1, 11, 2)),
        (1647, 2547)
    );
    assert_eq!(
        (register(&bus, 3, 9, 2), register(&bus, 3, 11, 2)),
        (1760, 2909)
    );
    assert!(printed.contains("homing_offset 0 -> -1047"), "{printed}");
    assert!(
        printed.contains("servo 1: protection already 1000/310/80"),
        "{printed}"
    );
    assert!(
        printed.contains("servo 3: protection Max_Torque 500 -> 1000"),
        "{printed}"
    );
    for (address, width, value) in [(16, 2, 1000), (28, 2, 310), (36, 1, 80), (48, 2, 1000)] {
        assert_eq!(
            register(&bus, 3, address, width),
            value,
            "register {address}"
        );
    }
    // Installed where the station loads the neck's calibration from, as the servos hold it.
    let installed = robot.join(sx_embodiment_bodies::calibration_file("neck"));
    let kept: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&installed).expect("installed"))
            .expect("json");
    let applied: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(&file).expect("file")).expect("json");
    assert_eq!(kept, applied);
    assert!(printed.contains("commissioned calibration"), "{printed}");
    // Kept: closing put only Lock back, and the journal is gone.
    assert_eq!(register(&bus, 1, 55, 1), 0);
    assert_eq!(register(&bus, 1, 40, 1), 0);
    assert!(!Journal::path_for(&dir, BUS).exists());
}

#[test]
fn apply_refuses_a_servo_holding_torque_and_changes_nothing() {
    let bus = neck();
    bus.state().servos.get_mut(&1).expect("servo 1")[40] = 1;
    let dir = journal_dir("apply-torque");
    std::fs::create_dir_all(&dir).expect("dir");
    let file = dir.join("c.json");
    std::fs::write(
        &file,
        r#"{"pan": {"id": 1, "drive_mode": 0, "homing_offset": -5, "range_min": 100, "range_max": 200}}"#,
    )
    .expect("file");
    let words: Vec<String> = [
        "apply",
        "--bus",
        BUS,
        "--calibration",
        file.to_str().expect("utf-8"),
        "--move",
        "--journal-dir",
        dir.to_str().expect("utf-8"),
    ]
    .map(String::from)
    .to_vec();
    let (mut stdout, mut stderr) = (Vec::new(), Vec::new());
    let code = cli::run(
        &parse(&words).expect("parsed"),
        |_, _| Ok(bus.clone()),
        &Arc::new(AtomicUsize::new(0)),
        &mut stdout,
        &mut stderr,
    );
    assert_ne!(code, 0);
    assert!(String::from_utf8_lossy(&stderr).contains("holds torque"));
    assert!(
        parse(
            &[
                "apply",
                "--bus",
                BUS,
                "--calibration",
                "x.json",
                "--move",
                "--robot",
                "r"
            ]
            .map(String::from)
        )
        .is_err(),
        "--robot needs --port"
    );
    let publish = |extra: &[&str]| {
        let mut words: Vec<String> = ["apply", "--bus", BUS, "--calibration", "x.json", "--move"]
            .map(String::from)
            .to_vec();
        words.extend(extra.iter().map(|word| (*word).to_owned()));
        parse(&words)
    };
    assert_eq!(
        publish(&["--publish", "--port", "neck"])
            .expect("publish")
            .install,
        Some((
            std::path::PathBuf::from(cli::STATION_CALIBRATION_INBOX),
            "neck".to_owned()
        ))
    );
    assert!(publish(&["--publish"]).is_err(), "--publish needs --port");
    let borrowed = publish(&["--from-station", "--publish", "--port", "neck"]).expect("borrow");
    assert_eq!(borrowed.borrow.as_deref(), Some("neck"));
    assert!(
        publish(&["--from-station"]).is_err(),
        "--from-station needs --port"
    );
    assert!(
        publish(&["--from-station", "--port", "neck"]).is_err(),
        "apply from the station publishes what it writes"
    );
    assert!(
        publish(&["--publish", "--robot", "r", "--port", "neck"]).is_err(),
        "--publish is the station's inbox"
    );
    assert_eq!(register(&bus, 1, 31, 2), 0);
    assert_eq!(
        (register(&bus, 1, 9, 2), register(&bus, 1, 11, 2)),
        (0, 4095)
    );
}

#[test]
fn a_bus_is_borrowed_from_the_station_and_returned_when_the_loan_drops() {
    use sx_embodiment_commission::station_loan::Loan;
    let root = journal_dir("loans");
    std::fs::create_dir_all(&root).expect("loans");
    let stop = AtomicUsize::new(0);
    // The station lends what is asked: here, a thread standing in for it.
    let lender = {
        let root = root.clone();
        std::thread::spawn(move || {
            while !root.join("neck.borrow").exists() {
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            let pid = std::fs::read_to_string(root.join("neck.borrow")).expect("request");
            std::fs::write(root.join("neck.lent"), pid).expect("lent");
        })
    };
    let loan = Loan::borrow(&root, "neck", std::time::Duration::from_secs(5), &stop).expect("lent");
    lender.join().expect("lender");
    assert_eq!(
        std::fs::read_to_string(root.join("neck.borrow")).expect("request"),
        std::process::id().to_string()
    );
    drop(loan);
    assert!(!root.join("neck.borrow").exists(), "returned");
    // A station that does not lend in time leaves nothing behind.
    let refused = Loan::borrow(&root, "tail", std::time::Duration::from_millis(200), &stop);
    assert!(refused.is_err());
    assert!(!root.join("tail.borrow").exists());
    assert!(
        Loan::borrow(
            &root.join("missing"),
            "neck",
            std::time::Duration::ZERO,
            &stop
        )
        .is_err()
    );
}
