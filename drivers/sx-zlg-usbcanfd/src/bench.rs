//! A CAN round-trip benchmark over any `CanPort`, sending only Damiao register reads.
//!
//! The same code measures this crate's userspace path and, on Linux, `SocketCAN` over ZLG's own
//! kernel module, so the two are compared frame for frame. Every frame goes through
//! [`ReadOnly`], which refuses anything but a register read of `PMAX`, `VMAX` or `TMAX`
//! (`0x7FF`, `[id_lo, id_hi, 0x33, rid, 0, 0, 0, 0]`, the registers `sx_damiao_can` reads at
//! open, the rule `sx_damiao_can::scan::ReadOnlyCanPort` enforces, narrowed further): an enable,
//! a disable, a zero, an MIT command or a register write cannot leave the host.
//!
//! - **Latency**: one read to one motor at a time, its reply awaited, `samples` times.
//! - **Cycles**: at each rate, every period sends one read to every motor and awaits all
//!   replies before the next period starts; reported are each cycle's time to its last reply,
//!   cycles that missed their deadline and replies that never came. The register asked rotates
//!   each cycle, so a late reply from an earlier cycle is not mistaken for a current one.
//! - **Throughput**: every motor kept with one read outstanding for a while; replies per
//!   second, and requests sent.
//!
//! Replies are awaited by polling the port without sleeping, as a control loop polls; the
//! process CPU time reported per phase includes that polling.

use std::io;
use std::time::{Duration, Instant};

use serde::Serialize;
use sx_embodiment_drivers::can::{CanFrame, CanPort};

/// The id Damiao register requests are sent to.
pub const REGISTER_ID: u32 = 0x7FF;
/// A register read's command byte.
pub const READ: u8 = 0x33;
/// The registers read: `PMAX`, `VMAX`, `TMAX`.
pub const REGISTERS: [u8; 3] = [21, 22, 23];

/// The read of register `rid` from motor `motor`.
#[must_use]
pub const fn read_request(motor: u16, rid: u8) -> CanFrame {
    let id = motor.to_le_bytes();
    CanFrame {
        id: REGISTER_ID,
        data: [id[0], id[1], READ, rid, 0, 0, 0, 0],
    }
}

/// Whether `frame` is a read of one of [`REGISTERS`] and nothing else.
#[must_use]
pub fn is_allowed(frame: &CanFrame) -> bool {
    frame.id == REGISTER_ID
        && frame.data[2] == READ
        && REGISTERS.contains(&frame.data[3])
        && frame.data[4..] == [0, 0, 0, 0]
}

/// Whether `frame` is motor `motor`'s reply, on `master`, to a read of `rid`.
#[must_use]
pub fn is_reply(frame: &CanFrame, motor: u16, master: u16, rid: u8) -> bool {
    frame.id == u32::from(master)
        && frame.data[..2] == motor.to_le_bytes()
        && frame.data[2] == READ
        && frame.data[3] == rid
}

/// A port that carries only [`is_allowed`] frames.
pub struct ReadOnly<P>(P);

impl<P: CanPort> ReadOnly<P> {
    pub const fn new(port: P) -> Self {
        Self(port)
    }

    pub const fn inner(&self) -> &P {
        &self.0
    }
}

impl<P: CanPort> CanPort for ReadOnly<P> {
    fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
        if !is_allowed(frame) {
            return Err(io::Error::new(
                io::ErrorKind::PermissionDenied,
                format!(
                    "the benchmark sends only PMAX/VMAX/TMAX register reads, not {:#x} {:02X?}",
                    frame.id, frame.data
                ),
            ));
        }
        self.0.send(frame)
    }

    fn receive(&mut self) -> io::Result<Option<CanFrame>> {
        self.0.receive()
    }

    fn sleep(&mut self, duration: Duration) {
        self.0.sleep(duration);
    }

    fn frame_allowance(&self) -> Duration {
        self.0.frame_allowance()
    }
}

/// What to measure.
#[derive(Clone, Debug, Serialize)]
pub struct Config {
    /// (send id, master id) of each motor.
    pub motors: Vec<(u16, u16)>,
    pub latency_samples: usize,
    pub rates_hz: Vec<u32>,
    pub cycle_seconds: f64,
    pub throughput_seconds: f64,
    /// How long one reply may take before it counts as lost.
    pub reply_timeout_ms: u64,
}

impl Config {
    /// The static upper body: send ids 0x09, 0x10–0x15, 0x18–0x1F, master = send + 0x20.
    #[must_use]
    pub fn static_upper_body() -> Self {
        let ids = [0x09].into_iter().chain(0x10..=0x15).chain(0x18..=0x1F);
        Self {
            motors: ids.map(|id| (id, id + 0x20)).collect(),
            latency_samples: 2000,
            rates_hz: vec![50, 100, 250, 500],
            cycle_seconds: 10.0,
            throughput_seconds: 10.0,
            reply_timeout_ms: 20,
        }
    }
}

/// Percentiles of a set of durations, microseconds.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize)]
pub struct Percentiles {
    pub count: usize,
    pub p50_us: f64,
    pub p90_us: f64,
    pub p99_us: f64,
    pub max_us: f64,
}

impl Percentiles {
    fn of(samples: &mut [Duration]) -> Self {
        if samples.is_empty() {
            return Self::default();
        }
        samples.sort_unstable();
        let at = |fraction: f64| {
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )]
            let index = ((samples.len() as f64 * fraction).ceil() as usize).clamp(1, samples.len());
            micros(samples[index - 1])
        };
        Self {
            count: samples.len(),
            p50_us: at(0.50),
            p90_us: at(0.90),
            p99_us: at(0.99),
            max_us: micros(samples[samples.len() - 1]),
        }
    }
}

fn micros(duration: Duration) -> f64 {
    duration.as_secs_f64() * 1e6
}

/// Single round trips.
#[derive(Clone, Debug, Serialize)]
pub struct Latency {
    pub motor: u16,
    pub round_trip: Percentiles,
    pub lost: usize,
    pub cpu_seconds_per_second: Option<f64>,
}

/// One rate's cycles.
#[derive(Clone, Debug, Serialize)]
pub struct Cycles {
    pub rate_hz: u32,
    pub motors: usize,
    pub cycles: usize,
    /// Each cycle's time from its first request to its last reply, complete cycles only.
    pub cycle: Percentiles,
    pub missed_deadlines: usize,
    pub lost_replies: usize,
    pub cpu_seconds_per_second: Option<f64>,
}

/// Saturated request and reply rate.
#[derive(Clone, Debug, Serialize)]
pub struct Throughput {
    pub seconds: f64,
    pub requests_sent: u64,
    pub replies: u64,
    pub replies_per_second: f64,
    /// Requests plus replies on the bus per second.
    pub frames_per_second: f64,
    pub lost_replies: u64,
    pub cpu_seconds_per_second: Option<f64>,
}

/// One benchmark run.
#[derive(Clone, Debug, Serialize)]
pub struct Report {
    pub transport: String,
    pub config: Config,
    pub frame_allowance_us: f64,
    pub latency: Latency,
    pub cycles: Vec<Cycles>,
    pub throughput: Throughput,
}

/// Why a run stopped.
#[derive(Debug, thiserror::Error)]
pub enum BenchError {
    #[error("the benchmark needs at least one motor")]
    NoMotors,
    #[error("the port failed: {0}")]
    Port(#[from] io::Error),
}

/// The process's CPU time so far, where this platform reports it.
pub type CpuClock = fn() -> Option<Duration>;

/// Measure `port`: latency, cycles at each rate, throughput.
///
/// # Errors
///
/// Stops at the first port failure, and refuses an empty motor list.
pub fn run<P: CanPort>(
    port: &mut ReadOnly<P>,
    transport: &str,
    config: &Config,
    cpu: CpuClock,
) -> Result<Report, BenchError> {
    let &(motor, master) = config.motors.first().ok_or(BenchError::NoMotors)?;
    let timeout = Duration::from_millis(config.reply_timeout_ms);
    drain(port)?;
    let latency = latency(port, motor, master, config.latency_samples, timeout, cpu)?;
    let mut cycles = Vec::with_capacity(config.rates_hz.len());
    for rate in &config.rates_hz {
        drain(port)?;
        cycles.push(cycles_at(
            port,
            &config.motors,
            *rate,
            Duration::from_secs_f64(config.cycle_seconds),
            cpu,
        )?);
    }
    drain(port)?;
    let throughput = throughput(
        port,
        &config.motors,
        Duration::from_secs_f64(config.throughput_seconds),
        timeout,
        cpu,
    )?;
    Ok(Report {
        transport: transport.to_owned(),
        config: config.clone(),
        frame_allowance_us: micros(port.frame_allowance()),
        latency,
        cycles,
        throughput,
    })
}

/// Read everything already waiting.
fn drain<P: CanPort>(port: &mut ReadOnly<P>) -> io::Result<()> {
    let settle = Instant::now() + Duration::from_millis(20);
    while Instant::now() < settle {
        while port.receive()?.is_some() {}
        std::hint::spin_loop();
    }
    Ok(())
}

fn cpu_rate(cpu: CpuClock, before: Option<Duration>, elapsed: Duration) -> Option<f64> {
    let used = cpu()?.checked_sub(before?)?;
    Some(used.as_secs_f64() / elapsed.as_secs_f64().max(1e-9))
}

fn latency<P: CanPort>(
    port: &mut ReadOnly<P>,
    motor: u16,
    master: u16,
    samples: usize,
    timeout: Duration,
    cpu: CpuClock,
) -> Result<Latency, BenchError> {
    let mut round_trips = Vec::with_capacity(samples);
    let mut lost = 0;
    let cpu_before = cpu();
    let started = Instant::now();
    for sample in 0..samples {
        let rid = REGISTERS[sample % REGISTERS.len()];
        let sent = Instant::now();
        port.send(&read_request(motor, rid))?;
        let deadline = sent + timeout;
        loop {
            if let Some(frame) = port.receive()? {
                if is_reply(&frame, motor, master, rid) {
                    round_trips.push(sent.elapsed());
                    break;
                }
                continue;
            }
            if Instant::now() >= deadline {
                lost += 1;
                break;
            }
            std::hint::spin_loop();
        }
    }
    Ok(Latency {
        motor,
        round_trip: Percentiles::of(&mut round_trips),
        lost,
        cpu_seconds_per_second: cpu_rate(cpu, cpu_before, started.elapsed()),
    })
}

fn cycles_at<P: CanPort>(
    port: &mut ReadOnly<P>,
    motors: &[(u16, u16)],
    rate_hz: u32,
    length: Duration,
    cpu: CpuClock,
) -> Result<Cycles, BenchError> {
    let period = Duration::from_secs_f64(1.0 / f64::from(rate_hz.max(1)));
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let count = (length.as_secs_f64() * f64::from(rate_hz)).round() as usize;
    let mut times = Vec::with_capacity(count);
    let mut answered = vec![false; motors.len()];
    let (mut missed, mut lost) = (0, 0);
    let cpu_before = cpu();
    let started = Instant::now();
    for cycle in 0..count {
        let start = started + period * u32::try_from(cycle).unwrap_or(u32::MAX);
        while Instant::now() < start {
            std::hint::spin_loop();
        }
        let deadline = start + period;
        let rid = REGISTERS[cycle % REGISTERS.len()];
        answered.fill(false);
        for (motor, _) in motors {
            port.send(&read_request(*motor, rid))?;
        }
        let mut remaining = motors.len();
        let mut last_reply = start;
        while remaining > 0 && Instant::now() < deadline {
            let Some(frame) = port.receive()? else {
                std::hint::spin_loop();
                continue;
            };
            if let Some(index) = motors
                .iter()
                .position(|(motor, master)| is_reply(&frame, *motor, *master, rid))
                && !answered[index]
            {
                answered[index] = true;
                remaining -= 1;
                last_reply = Instant::now();
            }
        }
        if remaining == 0 {
            times.push(last_reply - start);
        } else {
            missed += 1;
            lost += remaining;
        }
    }
    Ok(Cycles {
        rate_hz,
        motors: motors.len(),
        cycles: count,
        cycle: Percentiles::of(&mut times),
        missed_deadlines: missed,
        lost_replies: lost,
        cpu_seconds_per_second: cpu_rate(cpu, cpu_before, started.elapsed()),
    })
}

fn throughput<P: CanPort>(
    port: &mut ReadOnly<P>,
    motors: &[(u16, u16)],
    length: Duration,
    timeout: Duration,
    cpu: CpuClock,
) -> Result<Throughput, BenchError> {
    // Each motor's outstanding read: its register and when it was sent.
    let mut outstanding: Vec<(u8, Instant)> = Vec::with_capacity(motors.len());
    let (mut sent, mut replies, mut lost) = (0_u64, 0_u64, 0_u64);
    let cpu_before = cpu();
    let started = Instant::now();
    for (index, (motor, _)) in motors.iter().enumerate() {
        let rid = REGISTERS[index % REGISTERS.len()];
        port.send(&read_request(*motor, rid))?;
        outstanding.push((rid, Instant::now()));
        sent += 1;
    }
    let end = started + length;
    let mut scanned = started;
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        // Timeouts are looked for each millisecond, busy or not: a saturated link may never
        // leave the receive queue empty.
        if now.duration_since(scanned) >= Duration::from_millis(1) {
            scanned = now;
            for (index, (rid, at)) in outstanding.iter_mut().enumerate() {
                if now.duration_since(*at) > timeout {
                    lost += 1;
                    port.send(&read_request(motors[index].0, *rid))?;
                    *at = now;
                    sent += 1;
                }
            }
        }
        let Some(frame) = port.receive()? else {
            std::hint::spin_loop();
            continue;
        };
        let answered = motors
            .iter()
            .zip(&outstanding)
            .position(|((motor, master), (rid, _))| is_reply(&frame, *motor, *master, *rid));
        if let Some(index) = answered {
            replies += 1;
            let rid = next_register(outstanding[index].0);
            port.send(&read_request(motors[index].0, rid))?;
            outstanding[index] = (rid, Instant::now());
            sent += 1;
        }
    }
    let elapsed = started.elapsed();
    #[allow(clippy::cast_precision_loss)]
    let per_second = |count: u64| count as f64 / elapsed.as_secs_f64();
    Ok(Throughput {
        seconds: elapsed.as_secs_f64(),
        requests_sent: sent,
        replies,
        replies_per_second: per_second(replies),
        frames_per_second: per_second(sent + replies),
        lost_replies: lost,
        cpu_seconds_per_second: cpu_rate(cpu, cpu_before, elapsed),
    })
}

/// The register read after `rid`, in turn.
fn next_register(rid: u8) -> u8 {
    let at = REGISTERS
        .iter()
        .position(|register| *register == rid)
        .unwrap_or(0);
    REGISTERS[(at + 1) % REGISTERS.len()]
}

/// The process's CPU time: `/proc/self/stat` on Linux (at the usual 100 ticks per second),
/// `ps` elsewhere.
#[must_use]
pub fn process_cpu() -> Option<Duration> {
    if let Ok(stat) = std::fs::read_to_string("/proc/self/stat") {
        let fields: Vec<&str> = stat.rsplit(')').next()?.split_whitespace().collect();
        let ticks: u64 =
            fields.get(11)?.parse::<u64>().ok()? + fields.get(12)?.parse::<u64>().ok()?;
        return Some(Duration::from_millis(ticks * 10));
    }
    let output = std::process::Command::new("ps")
        .args(["-o", "time=", "-p", &std::process::id().to_string()])
        .output()
        .ok()?;
    parse_ps_time(String::from_utf8_lossy(&output.stdout).trim())
}

/// `ps -o time=` as `[[DD-]HH:]MM:SS[.ss]`.
fn parse_ps_time(text: &str) -> Option<Duration> {
    let mut seconds = 0.0;
    for part in text.split(['-', ':']) {
        seconds = seconds * 60.0 + part.parse::<f64>().ok()?;
    }
    Some(Duration::from_secs_f64(seconds))
}

/// The 1-minute load average, where this platform reports it.
#[must_use]
pub fn load_average() -> Option<f64> {
    if let Ok(text) = std::fs::read_to_string("/proc/loadavg") {
        return text.split_whitespace().next()?.parse().ok();
    }
    let output = std::process::Command::new("sysctl")
        .args(["-n", "vm.loadavg"])
        .output()
        .ok()?;
    String::from_utf8_lossy(&output.stdout)
        .trim_matches(|c: char| c == '{' || c == '}' || c.is_whitespace())
        .split_whitespace()
        .next()?
        .parse()
        .ok()
}

/// The report as a table.
#[must_use]
pub fn table(report: &Report) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    let cpu = |value: Option<f64>| value.map_or_else(|| "-".to_owned(), |v| format!("{v:.3}"));
    let _ = writeln!(out, "transport: {}", report.transport);
    let latency = &report.latency;
    let rtt = latency.round_trip;
    let _ = writeln!(
        out,
        "round trip (motor {:#04x}, {} samples): p50 {:.0} us  p90 {:.0} us  p99 {:.0} us  max {:.0} us  lost {}  cpu {} s/s",
        latency.motor,
        rtt.count,
        rtt.p50_us,
        rtt.p90_us,
        rtt.p99_us,
        rtt.max_us,
        latency.lost,
        cpu(latency.cpu_seconds_per_second)
    );
    let _ = writeln!(
        out,
        "{:>6} {:>6} {:>7} {:>10} {:>10} {:>10} {:>7} {:>6} {:>8}",
        "rate", "motors", "cycles", "p50 us", "p99 us", "max us", "missed", "lost", "cpu s/s"
    );
    for cycles in &report.cycles {
        let _ = writeln!(
            out,
            "{:>6} {:>6} {:>7} {:>10.0} {:>10.0} {:>10.0} {:>7} {:>6} {:>8}",
            cycles.rate_hz,
            cycles.motors,
            cycles.cycles,
            cycles.cycle.p50_us,
            cycles.cycle.p99_us,
            cycles.cycle.max_us,
            cycles.missed_deadlines,
            cycles.lost_replies,
            cpu(cycles.cpu_seconds_per_second)
        );
    }
    let throughput = &report.throughput;
    let _ = writeln!(
        out,
        "throughput: {:.0} replies/s, {:.0} frames/s, {} lost, cpu {} s/s",
        throughput.replies_per_second,
        throughput.frames_per_second,
        throughput.lost_replies,
        cpu(throughput.cpu_seconds_per_second)
    );
    out
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::*;

    /// Motors that answer every read on their master id, except those told to stay silent.
    struct Bus {
        motors: Vec<(u16, u16)>,
        silent: Vec<u16>,
        inbox: VecDeque<CanFrame>,
        sent: Vec<CanFrame>,
    }

    impl CanPort for Bus {
        fn send(&mut self, frame: &CanFrame) -> io::Result<()> {
            self.sent.push(*frame);
            let motor = u16::from_le_bytes([frame.data[0], frame.data[1]]);
            if let Some((_, master)) = self.motors.iter().find(|(id, _)| *id == motor)
                && !self.silent.contains(&motor)
            {
                let mut data = frame.data;
                data[4..].copy_from_slice(&12.5_f32.to_le_bytes());
                self.inbox.push_back(CanFrame {
                    id: u32::from(*master),
                    data,
                });
            }
            Ok(())
        }

        fn receive(&mut self) -> io::Result<Option<CanFrame>> {
            Ok(self.inbox.pop_front())
        }
    }

    fn bus(silent: Vec<u16>) -> ReadOnly<Bus> {
        ReadOnly::new(Bus {
            motors: Config::static_upper_body().motors,
            silent,
            inbox: VecDeque::new(),
            sent: Vec::new(),
        })
    }

    #[test]
    fn only_pmax_vmax_tmax_reads_leave_the_host() {
        let mut port = bus(Vec::new());
        for rid in REGISTERS {
            port.send(&read_request(0x09, rid)).expect("a read");
        }
        let refused = [
            // Enable, disable, zero, an MIT command to motor 0x09.
            CanFrame {
                id: 0x09,
                data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
            },
            CanFrame {
                id: 0x09,
                data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
            },
            CanFrame {
                id: 0x09,
                data: [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE],
            },
            CanFrame {
                id: 0x09,
                data: [0x80, 0, 0x80, 0, 0, 0, 0x80, 0],
            },
            // A register write, a flash save, a read of another register, a read with a tail.
            CanFrame {
                id: 0x7FF,
                data: [0x09, 0, 0x55, 9, 1, 0, 0, 0],
            },
            CanFrame {
                id: 0x7FF,
                data: [0x09, 0, 0xAA, 0, 0, 0, 0, 0],
            },
            read_request(0x09, 10),
            CanFrame {
                id: 0x7FF,
                data: [0x09, 0, 0x33, 21, 0, 0, 0, 1],
            },
        ];
        for frame in refused {
            let error = port.send(&frame).expect_err("refused");
            assert_eq!(error.kind(), io::ErrorKind::PermissionDenied);
        }
        assert_eq!(
            port.inner().sent.len(),
            3,
            "only the three reads reached the bus"
        );
    }

    fn quick() -> Config {
        Config {
            latency_samples: 50,
            rates_hz: vec![100, 500],
            cycle_seconds: 0.05,
            throughput_seconds: 0.05,
            reply_timeout_ms: 2,
            ..Config::static_upper_body()
        }
    }

    #[test]
    fn a_bus_that_answers_everything_loses_nothing() {
        let mut port = bus(Vec::new());
        let report = run(&mut port, "fake", &quick(), || None).expect("runs");
        assert_eq!(report.latency.round_trip.count, 50);
        assert_eq!(report.latency.lost, 0);
        for cycles in &report.cycles {
            assert_eq!(cycles.motors, 15);
            assert_eq!(cycles.lost_replies, 0, "{}", cycles.rate_hz);
            assert_eq!(cycles.missed_deadlines, 0);
            assert_eq!(cycles.cycle.count, cycles.cycles);
        }
        assert!(report.throughput.replies > 0);
        assert_eq!(report.throughput.lost_replies, 0);
        assert!(port.inner().sent.iter().all(is_allowed));
        assert!(table(&report).contains("throughput"));
        serde_json::to_string(&report).expect("serializes");
    }

    #[test]
    fn a_silent_motor_is_counted_lost_in_every_phase() {
        let mut config = quick();
        config.motors.swap(0, 1);
        let silent = config.motors[0].0;
        let mut port = bus(vec![silent]);
        let report = run(&mut port, "fake", &config, || None).expect("runs");
        assert_eq!(report.latency.lost, 50);
        for cycles in &report.cycles {
            assert_eq!(cycles.missed_deadlines, cycles.cycles);
            assert_eq!(cycles.lost_replies, cycles.cycles);
        }
        assert!(report.throughput.lost_replies > 0);
    }

    #[test]
    fn percentiles_and_ps_times_read_as_expected() {
        let mut samples: Vec<Duration> = (1..=100).map(Duration::from_micros).collect();
        let p = Percentiles::of(&mut samples);
        assert_eq!(
            (p.count, p.p50_us, p.p90_us, p.p99_us, p.max_us),
            (100, 50.0, 90.0, 99.0, 100.0)
        );
        assert_eq!(parse_ps_time("0:01.50"), Some(Duration::from_millis(1500)));
        assert_eq!(parse_ps_time("1:00:00"), Some(Duration::from_secs(3600)));
        assert_eq!(parse_ps_time("x"), None);
    }
}
