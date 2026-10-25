//! Read a planned inertial sensor held still, and say which way is up.
//!
//! ```text
//! sx-imu-still --plan <sx.media-plan/v3 json> --instance <instance id> [--sensor <name>] [--seconds N]
//! sx-imu-still --stream --plan ... --instance ... [--sensor <name>] [--seconds N]
//! ```
//!
//! With `--stream` it prints instead one JSON line per 50 ms window of host time, until
//! `--seconds` (if given) or until stopped: `{"t": <unix seconds>, "pitch_deg", "roll_deg", "g"}`,
//! the window's mean specific force carried onto the mount link (`ZED_camera` for a ZED) as its
//! pitch (nose down +) and roll (left side up +), and that force's magnitude in m/s². `t` is the
//! host wall clock midway between the window's first and last sample, for aligning against other
//! host-stamped logs.
//!
//! Prints the sample rate on the device's and the host's clocks, the mean specific force and the
//! gyroscope bias on the sensor's axes, and the up direction, roll and pitch of the link the
//! sensor is mounted on: the plan's `mount_from_sensor` carries the axes there. Read-only: it
//! streams the sensor and moves nothing.

use std::io::Write;
use std::process::ExitCode;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use sx_embodiment_drivers::{
    InertialSample, InertialSensor, InertialSource, MediaPlan, Stillness, Tilt,
};

/// One `--stream` line's window of host time.
const WINDOW: Duration = Duration::from_millis(50);

struct Options {
    plan: String,
    instance: String,
    sensor: Option<String>,
    seconds: Option<f64>,
    stream: bool,
}

fn options() -> Result<Options, String> {
    let mut arguments = std::env::args().skip(1);
    let (mut plan, mut instance, mut sensor, mut seconds, mut stream) =
        (None, None, None, None, false);
    while let Some(flag) = arguments.next() {
        if flag == "--stream" {
            stream = true;
            continue;
        }
        let value = arguments
            .next()
            .ok_or_else(|| format!("{flag} takes a value"))?;
        match flag.as_str() {
            "--plan" => plan = Some(value),
            "--instance" => instance = Some(value),
            "--sensor" => sensor = Some(value),
            "--seconds" => {
                seconds = Some(
                    value
                        .parse()
                        .ok()
                        .filter(|seconds: &f64| seconds.is_finite() && *seconds > 0.0)
                        .ok_or_else(|| format!("{value} is not a positive number of seconds"))?,
                );
            }
            _ => return Err(format!("unknown flag {flag}")),
        }
    }
    Ok(Options {
        plan: plan.ok_or("--plan is required")?,
        instance: instance.ok_or("--instance is required")?,
        sensor,
        seconds,
        stream,
    })
}

fn unix_seconds(at: SystemTime) -> f64 {
    at.duration_since(UNIX_EPOCH)
        .map_or(0.0, |since| since.as_secs_f64())
}

/// One JSON line for a window of samples, or `None` for an empty window.
#[allow(clippy::cast_precision_loss)] // a window holds a few dozen samples
fn window_line(sensor: &InertialSensor, window: &[InertialSample]) -> Option<String> {
    let (first, last) = (window.first()?, window.last()?);
    let mut sum = [0.0; 3];
    for sample in window {
        for (total, value) in sum.iter_mut().zip(sample.acceleration_mps2) {
            *total += value;
        }
    }
    let mean = sum.map(|total| total / window.len() as f64);
    let tilt = Tilt::of_specific_force(sensor.in_mount(mean))?;
    let g = mean.iter().map(|value| value * value).sum::<f64>().sqrt();
    let t = f64::midpoint(
        unix_seconds(first.captured.wall),
        unix_seconds(last.captured.wall),
    );
    Some(format!(
        "{{\"t\": {t:.4}, \"pitch_deg\": {:.3}, \"roll_deg\": {:.3}, \"g\": {g:.4}}}",
        tilt.pitch_rad.to_degrees(),
        tilt.roll_rad.to_degrees()
    ))
}

/// Print a line per [`WINDOW`] until `seconds`, if given, or until stdout closes.
fn stream(
    sensor: &InertialSensor,
    source: &mut impl InertialSource,
    seconds: Option<f64>,
) -> Result<(), String> {
    let started = Instant::now();
    let limit = seconds.map(Duration::from_secs_f64);
    let mut stdout = std::io::stdout().lock();
    let mut window = Vec::new();
    let mut opened = Instant::now();
    while limit.is_none_or(|limit| started.elapsed() < limit) {
        window.push(source.next().map_err(|error| error.to_string())?);
        if opened.elapsed() >= WINDOW {
            if let Some(line) = window_line(sensor, &window) {
                // A closed pipe (a reader that stopped) ends the stream, not an error.
                if writeln!(stdout, "{line}")
                    .and_then(|()| stdout.flush())
                    .is_err()
                {
                    return Ok(());
                }
            }
            window.clear();
            opened = Instant::now();
        }
    }
    Ok(())
}

fn degrees(v: [f64; 3]) -> [f64; 3] {
    v.map(f64::to_degrees)
}

#[allow(clippy::cast_precision_loss)] // sample counts are far below 2^52
fn run() -> Result<(), String> {
    let options = options()?;
    let document = std::fs::read_to_string(&options.plan)
        .map_err(|error| format!("{}: {error}", options.plan))?;
    let plan = MediaPlan::parse(&document, &options.instance).map_err(|error| error.to_string())?;
    let sensor = match &options.sensor {
        Some(name) => plan.inertial.iter().find(|sensor| &sensor.sensor == name),
        None => plan.inertial.first(),
    }
    .ok_or("the plan declares no such inertial sensor")?;
    let mut source = sx_stereolabs_sensors::hid::open(sensor).map_err(|error| error.to_string())?;
    if options.stream {
        return stream(sensor, &mut source, options.seconds);
    }
    let firmware = source.firmware();
    let started = Instant::now();
    let window = Duration::from_secs_f64(options.seconds.unwrap_or(5.0));
    let mut samples = Vec::new();
    let (mut magnetic, mut pressure, mut temperature) = (Vec::new(), Vec::new(), Vec::new());
    while started.elapsed() < window {
        let sample = source.next().map_err(|error| error.to_string())?;
        magnetic.extend(sample.magnetic_field_t);
        pressure.extend(sample.pressure_pa);
        temperature.extend(sample.temperature_c);
        samples.push(sample);
    }
    let host_seconds = started.elapsed().as_secs_f64();
    let still = Stillness::of(&samples).ok_or("fewer than two samples")?;
    let force = still.mean_acceleration_mps2;
    let magnitude = force.iter().map(|v| v * v).sum::<f64>().sqrt();
    println!(
        "sensor {} ({} unit {}, firmware {}.{}), frame {} on {}",
        sensor.sensor,
        sensor.model,
        sensor.unit.as_deref().unwrap_or("-"),
        firmware >> 8,
        firmware & 0xff,
        sensor.sensor,
        sensor.mount_frame
    );
    println!(
        "samples {} over {:.3} s device / {host_seconds:.3} s host: {:.1} Hz device, {:.1} Hz host",
        still.samples,
        still.span.as_secs_f64(),
        still.rate_hz,
        still.samples as f64 / host_seconds
    );
    println!(
        "specific force (sensor axes) [{:.4}, {:.4}, {:.4}] m/s^2, |f| {magnitude:.4}, spread {:.4}",
        force[0], force[1], force[2], still.acceleration_spread_mps2
    );
    let bias = still.gyro_bias_radps;
    let bias_deg = degrees(bias);
    println!(
        "gyro bias (sensor axes) [{:.5}, {:.5}, {:.5}] rad/s = [{:.3}, {:.3}, {:.3}] deg/s, spread {:.5} rad/s",
        bias[0], bias[1], bias[2], bias_deg[0], bias_deg[1], bias_deg[2], still.gyro_spread_radps
    );
    let in_mount = sensor.in_mount(force);
    let tilt =
        Tilt::of_specific_force(in_mount).ok_or("no specific force to take a direction from")?;
    println!(
        "up in {} [{:.4}, {:.4}, {:.4}]: roll {:.2} deg (left side up +), pitch {:.2} deg (nose down +)",
        sensor.mount_frame,
        tilt.up[0],
        tilt.up[1],
        tilt.up[2],
        tilt.roll_rad.to_degrees(),
        tilt.pitch_rad.to_degrees()
    );
    if let Some(last) = magnetic.last() {
        println!(
            "magnetic field {} readings, last [{:.2}, {:.2}, {:.2}] uT",
            magnetic.len(),
            last[0] * 1e6,
            last[1] * 1e6,
            last[2] * 1e6
        );
    }
    if !pressure.is_empty() {
        println!(
            "pressure {} readings, mean {:.1} Pa",
            pressure.len(),
            pressure.iter().sum::<f64>() / pressure.len() as f64
        );
    }
    if !temperature.is_empty() {
        println!(
            "imu temperature mean {:.2} C",
            temperature.iter().sum::<f64>() / temperature.len() as f64
        );
    }
    if still.gyro_spread_radps > 0.05 || (magnitude - 9.81).abs() > 0.3 {
        println!("warning: the sensor was not still; the tilt is not a level reading");
    }
    Ok(())
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("sx-imu-still: {error}");
            ExitCode::FAILURE
        }
    }
}
