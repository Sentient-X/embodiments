//! How fluid and how quick the solve is: every registry body with a tool frame
//! (`fixtures/registry.json`, `tools/render_ik_registry.py`) follows a scripted tremor trace,
//! and the station's own solve (`fixtures/static-teleop-config.json` from Ready) answers a
//! 10 cm jump, a 0.5 m/s sweep, 1 and 2 Hz sines and the tremor, each tick timed.
//!
//! Null-space jerk is the third difference of the joint positions, per tick, projected onto the
//! null space of the driven tools' Jacobian (rows `[Jv; 0.15 Jw]`, the solve's own metric): the
//! joint motion that moves no tool, which an operator sees as an elbow that shivers while the
//! hand is still.
//!
//! ```text
//! cargo test --release -p sx-embodiment-ik --test fluid                     # registry-wide
//! cargo test --release -p sx-embodiment-ik --test fluid -- --ignored --nocapture  # tables
//! ```

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::needless_range_loop,
    clippy::too_many_lines,
    clippy::many_single_char_names
)]

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use sx_embodiment_ik::teleop::{Targets, Teleop, TeleopConfig};
use sx_embodiment_ik::{
    Body, Caps, Ik, Limb, STATIC_UPPER_BODY_COLLISION, STATIC_UPPER_BODY_URDF, Settings, Target,
    Task,
};
use sx_frames::{Pose, QuaternionLayout};

const DT: f64 = 0.01;
const RHO: f64 = 0.15;
const REGISTRY: &str = include_str!("fixtures/registry.json");

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (self.0 >> 11) as f64 / (1_u64 << 53) as f64
    }
    fn uniform(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.next()
    }
}

fn pose(body: &Body, q: &[f64], limb: usize) -> Pose {
    body.frame(&body.model().poses(q), limb).pose()
}

/// `a * b` for quaternions `[x, y, z, w]`.
fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [ax, ay, az, aw] = a;
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

/// The rotation by `v` (axis times angle) as a quaternion.
fn quat_exp(v: [f64; 3]) -> [f64; 4] {
    let angle = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if angle < 1e-12 {
        return [0.0, 0.0, 0.0, 1.0];
    }
    let s = (angle / 2.0).sin() / angle;
    [v[0] * s, v[1] * s, v[2] * s, (angle / 2.0).cos()]
}

/// The scripted operator: a slow 3 cm circle at 0.25 Hz (4.7 cm/s), on it a physiological
/// tremor (7, 9.5 and 11 Hz, 1.0 / 0.7 / 0.5 mm per axis, and 0.5 degrees at 8 Hz) and a
/// tracker's 0.3 mm of noise.
struct Tremor {
    phases: [[f64; 3]; 3],
    turn_axis: [f64; 3],
    noise: Rng,
}

impl Tremor {
    fn new(seed: u64) -> Self {
        let mut rng = Rng(seed);
        let mut phases = [[0.0; 3]; 3];
        for row in &mut phases {
            for cell in row.iter_mut() {
                *cell = rng.uniform(0.0, std::f64::consts::TAU);
            }
        }
        let axis = [
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
        ];
        let length = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        Self {
            phases,
            turn_axis: axis.map(|v| v / length),
            noise: Rng(seed ^ 0x9e37_79b9),
        }
    }

    /// The smooth path's offset at `t`, and the target's (tremor and noise on it).
    fn at(&mut self, t: f64, from: &Pose) -> (Pose, Pose) {
        use std::f64::consts::TAU;
        let ramp = (t / 0.5).min(1.0);
        let circle = [
            0.03 * ((TAU * 0.25 * t).cos() - 1.0) * ramp,
            0.0,
            0.03 * (TAU * 0.25 * t).sin() * ramp,
        ];
        let mut smooth = *from;
        let mut target = *from;
        for k in 0..3 {
            smooth.translation_m[k] += circle[k];
            let shake = 0.001 * (TAU * 7.0 * t + self.phases[0][k]).sin()
                + 0.0007 * (TAU * 9.5 * t + self.phases[1][k]).sin()
                + 0.0005 * (TAU * 11.0 * t + self.phases[2][k]).sin()
                + self.noise.uniform(-0.0003, 0.0003);
            target.translation_m[k] = smooth.translation_m[k] + shake * ramp;
        }
        let angle = 0.5_f64.to_radians() * (TAU * 8.0 * t).sin() * ramp;
        let xyzw = QuaternionLayout::Xyzw;
        target.rotation = xyzw.read(quat_mul(
            quat_exp(self.turn_axis.map(|v| v * angle)),
            xyzw.write(from.rotation),
        ));
        (smooth, target)
    }
}

/// Rows `[Jv; RHO Jw]` of each listed limb over `columns` at `q`, by central differences.
fn jacobian(body: &Body, q: &[f64], limbs: &[usize], columns: &[usize]) -> Vec<Vec<f64>> {
    let h = 1e-6;
    let mut rows = vec![vec![0.0; columns.len()]; 6 * limbs.len()];
    let mut moved = q.to_vec();
    for (c, &axis) in columns.iter().enumerate() {
        moved[axis] = q[axis] + h;
        let plus = body.model().poses(&moved);
        moved[axis] = q[axis] - h;
        let minus = body.model().poses(&moved);
        moved[axis] = q[axis];
        for (l, &limb) in limbs.iter().enumerate() {
            let (a, b) = (body.frame(&plus, limb), body.frame(&minus, limb));
            for k in 0..3 {
                rows[6 * l + k][c] = (a.t[k] - b.t[k]) / (2.0 * h);
            }
            // R+ R-^T is a rotation by 2h w: its vee.
            let mut m = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] = (0..3).map(|k| a.r[i][k] * b.r[j][k]).sum();
                }
            }
            let w = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]];
            for k in 0..3 {
                rows[6 * l + 3 + k][c] = RHO * w[k] / 2.0 / (2.0 * h);
            }
        }
    }
    rows
}

/// `v` less its part in the row space of `rows`: `v - J' (J J' + eps)^-1 J v`.
fn null_part(rows: &[Vec<f64>], v: &[f64]) -> Vec<f64> {
    let m = rows.len();
    let mut a = vec![vec![0.0; m]; m];
    for i in 0..m {
        for j in 0..m {
            a[i][j] = rows[i].iter().zip(&rows[j]).map(|(x, y)| x * y).sum();
        }
        a[i][i] += 1e-10;
    }
    let mut b: Vec<f64> = rows
        .iter()
        .map(|r| r.iter().zip(v).map(|(x, y)| x * y).sum())
        .collect();
    // Cholesky in place.
    for j in 0..m {
        let mut d = a[j][j];
        for k in 0..j {
            d -= a[j][k] * a[j][k];
        }
        let d = d.max(1e-18).sqrt();
        a[j][j] = d;
        for i in j + 1..m {
            let mut s = a[i][j];
            for k in 0..j {
                s -= a[i][k] * a[j][k];
            }
            a[i][j] = s / d;
        }
    }
    for i in 0..m {
        for k in 0..i {
            b[i] -= a[i][k] * b[k];
        }
        b[i] /= a[i][i];
    }
    for i in (0..m).rev() {
        for k in i + 1..m {
            b[i] -= a[k][i] * b[k];
        }
        b[i] /= a[i][i];
    }
    let mut out = v.to_vec();
    for (i, row) in rows.iter().enumerate() {
        for (o, r) in out.iter_mut().zip(row) {
            *o -= r * b[i];
        }
    }
    out
}

fn norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn rms(values: &[f64]) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    (values.iter().map(|v| v * v).sum::<f64>() / values.len() as f64).sqrt()
}

/// One run's measures.
#[derive(Clone, Copy, Debug, Default)]
struct Measures {
    /// Tool to target after each step, RMS over the run past its first half second, metres.
    track_m: f64,
    /// Tool to the smooth path under the tremor, the same way.
    smooth_m: f64,
    /// Null-space jerk, RMS, rad/s^3.
    null_jerk: f64,
    /// Whole joint jerk over the driven columns, RMS, rad/s^3.
    jerk: f64,
}

/// Jerk measures of a run's positions (one per tick) over `columns`, for `limbs`.
fn jerks(body: &Body, history: &[Vec<f64>], limbs: &[usize], columns: &[usize]) -> (f64, f64) {
    let (mut null, mut whole) = (Vec::new(), Vec::new());
    for k in 3..history.len() {
        let j: Vec<f64> = columns
            .iter()
            .map(|&a| {
                (history[k][a] - 3.0 * history[k - 1][a] + 3.0 * history[k - 2][a]
                    - history[k - 3][a])
                    / (DT * DT * DT)
            })
            .collect();
        let rows = jacobian(body, &history[k - 1], limbs, columns);
        null.push(norm(&null_part(&rows, &j)));
        whole.push(norm(&j));
    }
    (rms(&null), rms(&whole))
}

/// A registry body ready to solve, with the posture it starts in.
struct Registered {
    name: String,
    body: Body,
    caps: Caps,
    /// Limbs whose tool some axis moves.
    driven: Vec<usize>,
}

fn registry() -> (Vec<Registered>, Vec<String>) {
    let documents: Vec<serde_json::Value> = serde_json::from_str(REGISTRY).expect("registry");
    let (mut bodies, mut skipped) = (Vec::new(), Vec::new());
    for document in documents {
        let name = document["embodiment"].as_str().expect("name").to_owned();
        let tools: Vec<String> = document["tools"]
            .as_array()
            .expect("tools")
            .iter()
            .map(|t| t.as_str().expect("frame").to_owned())
            .collect();
        let limbs: Vec<Limb> = tools
            .iter()
            .map(|frame| Limb {
                name: frame.clone(),
                frame: frame.clone(),
                task: Task::Pose,
            })
            .collect();
        let body = Body::new(
            document["urdf"].as_str().expect("urdf"),
            &document["collision"].to_string(),
            &limbs,
        )
        .unwrap_or_else(|e| panic!("{name}: {e}"));
        let driven: Vec<usize> = (0..tools.len())
            .filter(|&i| !body.limb_axes(i).is_empty())
            .collect();
        if driven.is_empty() {
            // A handheld tool (YUBI, a UMI): no joint carries it.
            skipped.push(name);
            continue;
        }
        let axes = body.model().axes();
        let speed: Vec<f64> = axes
            .iter()
            .map(|a| a.velocity.unwrap_or(2.0).min(4.0))
            .collect();
        let accel = speed.iter().map(|v| v / 0.1).collect();
        bodies.push(Registered {
            name,
            body,
            caps: Caps { speed, accel },
            driven,
        });
    }
    (bodies, skipped)
}

fn answers(settings: &Settings) -> Settings {
    Settings {
        // Answers, not a loaded host's clock.
        deadline: Duration::from_secs(3600),
        ..settings.clone()
    }
}

/// A clear, well-conditioned posture to start from: the best of 64 seeded draws inside the
/// middle half of every limit.
fn start(ik: &Ik, driven: &[usize]) -> Vec<f64> {
    let body = ik.body();
    let axes = body.model().axes();
    let mut rng = Rng(7);
    let mut best = (f64::NEG_INFINITY, ik.rest().to_vec());
    let columns = body.solved().to_vec();
    for _ in 0..64 {
        let q: Vec<f64> = axes
            .iter()
            .enumerate()
            .map(|(i, a)| {
                if !columns.contains(&i) {
                    return ik.rest()[i];
                }
                let [lo, hi] = a.limits.unwrap_or([-1.5, 1.5]);
                let (mid, half) = (f64::midpoint(lo, hi), (hi - lo) / 4.0);
                rng.uniform(mid - half, mid + half)
            })
            .collect();
        let clear = ik.clearance(&q).0;
        let rows = jacobian(body, &q, driven, &columns);
        // The smallest singular value's square, roughly: the least diagonal of J J'.
        let worst = (0..rows.len())
            .map(|i| rows[i].iter().map(|v| v * v).sum::<f64>())
            .fold(f64::INFINITY, f64::min);
        let score = clear.min(0.05) / 0.05 + worst.sqrt();
        if clear > 0.02 && score > best.0 {
            best = (score, q);
        }
    }
    best.1
}

/// The tremor trace through a bare [`Ik`] as a ghost (each step from the last), 3 s at 100 Hz.
fn tremor_run(registered: &Registered, settings: &Settings) -> Measures {
    tremor_timed(registered, settings, &mut Vec::new())
}

/// [`tremor_run`], each step's wall time pushed to `times`.
fn tremor_timed(registered: &Registered, settings: &Settings, times: &mut Vec<f64>) -> Measures {
    let body = registered.body.clone();
    let mut ik = Ik::new(
        body.clone(),
        registered.caps.clone(),
        answers(settings),
        Vec::new(),
    )
    .expect("solve");
    let mut q = start(&ik, &registered.driven);
    let limbs = body.limbs().len();
    let from: Vec<Pose> = (0..limbs).map(|i| pose(&body, &q, i)).collect();
    let mut tremors: Vec<Tremor> = (0..limbs).map(|i| Tremor::new(11 + i as u64)).collect();
    let available = vec![true; q.len()];
    let mut history = vec![q.clone()];
    let (mut track, mut smooth_error) = (Vec::new(), Vec::new());
    ik.reset();
    for tick in 1..=300 {
        let t = f64::from(tick) * DT;
        let mut smooth = Vec::with_capacity(limbs);
        let targets: Vec<Option<Target>> = (0..limbs)
            .map(|i| {
                let (path, target) = tremors[i].at(t, &from[i]);
                smooth.push(path);
                registered
                    .driven
                    .contains(&i)
                    .then_some(Target::Pose(target))
            })
            .collect();
        let began = Instant::now();
        let step = ik.step(&q, &available, &targets, DT).expect("a step");
        times.push(began.elapsed().as_secs_f64() * 1e6);
        q.copy_from_slice(&step.q);
        if t > 0.5 {
            for &i in &registered.driven {
                track.push(step.limbs[i].residual_m);
                let at = pose(&body, &q, i).translation_m;
                smooth_error.push(norm(&[0, 1, 2].map(|k| at[k] - smooth[i].translation_m[k])));
            }
        }
        history.push(q.clone());
    }
    let columns = body.solved().to_vec();
    let (null_jerk, jerk) = jerks(&body, &history[50..], &registered.driven, &columns);
    Measures {
        track_m: rms(&track),
        smooth_m: rms(&smooth_error),
        null_jerk,
        jerk,
    }
}

/// Every registry body with a tool frame some joint carries follows the scripted tremor, and
/// the solve's step-to-step
/// smoothness cost ([`Settings::smoothness`]) leaves that tracking as it was (within 5 %, or
/// 0.05 mm) while the joint motion that moves no tool shivers less wherever
/// the body has such motion.
#[test]
fn every_registry_tool_tracks_a_tremor_as_before_with_less_null_space_jerk() {
    let (bodies, skipped) = registry();
    assert!(bodies.len() >= 15, "only {} bodies solve", bodies.len());
    let before = before();
    let mut lines = Vec::new();
    let mut failures = Vec::new();
    for registered in &bodies {
        let old = tremor_run(registered, &before);
        let new = tremor_run(registered, &Settings::default());
        lines.push(format!(
            "{:40} track {:5.2} -> {:5.2} mm, smooth path {:5.2} -> {:5.2} mm, null jerk {:9.1} -> {:9.1} rad/s^3, jerk {:9.1} -> {:9.1}",
            registered.name,
            old.track_m * 1e3,
            new.track_m * 1e3,
            old.smooth_m * 1e3,
            new.smooth_m * 1e3,
            old.null_jerk,
            new.null_jerk,
            old.jerk,
            new.jerk,
        ));
        if new.track_m > (old.track_m * 1.05).max(old.track_m + 5e-5) {
            failures.push(format!("{}: tracking grew", registered.name));
        }
        // A body whose tools leave it joint motion to spare: a six-joint arm's whole pose
        // task leaves none, and a near-singular wrist's sliver of it (0.3 % of a UR5e's
        // jerk) is the measure's own noise.
        let redundant = old.null_jerk > 0.02 * old.jerk;
        if redundant && new.null_jerk >= 0.97 * old.null_jerk {
            failures.push(format!("{}: null-space jerk did not fall", registered.name));
        }
    }
    println!("{}", lines.join("\n"));
    println!("no joint carries a tool: {skipped:?}");
    assert!(failures.is_empty(), "{failures:#?}\n{}", lines.join("\n"));
}

/// The solve before [`Settings::smoothness`].
fn before() -> Settings {
    Settings {
        smoothness: 0.0,
        ..Settings::default()
    }
}

/// Median, 99th percentile and largest of `times`, microseconds.
fn spread(mut times: Vec<f64>) -> (f64, f64, f64) {
    times.sort_by(f64::total_cmp);
    let at = |p: f64| times[((times.len() - 1) as f64 * p) as usize];
    (at(0.5), at(0.99), times[times.len() - 1])
}

/// Each registry body's tick on this host, before and after [`Settings::smoothness`]: the
/// tremor trace three times over, every step timed. A tick must stay inside the station's
/// 4 ms budget ([`Settings::deadline`]) with room to spare.
#[test]
#[ignore = "a table: cargo test --release --test fluid table_tick_time -- --ignored --nocapture --test-threads 1"]
fn table_tick_time() {
    let (bodies, _) = registry();
    println!(
        "{:40} {:>4} {:>24} {:>24}",
        "body", "n", "before med/p99/max us", "after med/p99/max us"
    );
    for registered in &bodies {
        let (mut old, mut new) = (Vec::new(), Vec::new());
        for _ in 0..3 {
            tremor_timed(registered, &before(), &mut old);
            tremor_timed(registered, &Settings::default(), &mut new);
        }
        let (a, b) = (spread(old), spread(new));
        println!(
            "{:40} {:>4} {:>8.0} {:>7.0} {:>7.0} {:>8.0} {:>7.0} {:>7.0}",
            registered.name,
            registered.body.solved().len(),
            a.0,
            a.1,
            a.2,
            b.0,
            b.1,
            b.2
        );
    }
}

// --- The station's own solve ---------------------------------------------------------------

struct Station {
    solve: Teleop,
    body: Body,
    ready: Vec<f64>,
    times: Vec<Duration>,
    held: usize,
}

impl Station {
    fn new() -> Self {
        let config = TeleopConfig::parse(include_str!("fixtures/static-teleop-config.json"))
            .expect("the station's solve config");
        let ready: BTreeMap<String, f64> =
            serde_json::from_str(include_str!("fixtures/static-ready.json")).expect("ready");
        let mut solve = Teleop::new(STATIC_UPPER_BODY_URDF, STATIC_UPPER_BODY_COLLISION, &config)
            .expect("solve");
        // Answers, not a loaded host's clock: each tick is timed and counted against the
        // station's 4 ms budget below instead.
        solve.set_deadline(Duration::from_secs(3600));
        let body = solve.ik().body().clone();
        let ready = solve
            .model()
            .axes()
            .iter()
            .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
            .collect();
        Self {
            solve,
            body,
            ready,
            times: Vec::new(),
            held: 0,
        }
    }

    /// Run `ticks` from Ready with the left hand at `left(t, from)` and the right held at its
    /// Ready pose, the follower ideal (measured where commanded): each tick's target and tool
    /// pose (left), and the positions (Ready first).
    fn run(
        &mut self,
        ticks: usize,
        mut left: impl FnMut(f64, &Pose) -> Pose,
    ) -> (Vec<(Pose, Pose)>, Vec<Vec<f64>>) {
        let mut q = self.ready.clone();
        let (from, right) = (pose(&self.body, &q, 0), pose(&self.body, &q, 1));
        let available = vec![true; q.len()];
        let mut out = Vec::with_capacity(ticks);
        let mut history = vec![q.clone()];
        self.solve.reset();
        for tick in 1..=ticks {
            let t = tick as f64 * DT;
            let target = left(t, &from);
            let targets = Targets {
                tcp: [Some(target), Some(right)],
                ..Targets::default()
            };
            let measured: Vec<Option<f64>> = q.iter().map(|v| Some(*v)).collect();
            let began = Instant::now();
            let step = self
                .solve
                .step(&q, &measured, &available, &targets, DT)
                .expect("a step");
            self.times.push(began.elapsed());
            if self.times[self.times.len() - 1] > Settings::default().deadline {
                self.held += 1;
            }
            q.copy_from_slice(&step.q);
            out.push((target, pose(&self.body, &q, 0)));
            history.push(q.clone());
        }
        (out, history)
    }
}

fn distance(a: &Pose, b: &Pose) -> f64 {
    norm(&[0, 1, 2].map(|k| a.translation_m[k] - b.translation_m[k]))
}

fn shifted(from: &Pose, by: [f64; 3]) -> Pose {
    let mut p = *from;
    for k in 0..3 {
        p.translation_m[k] += by[k];
    }
    p
}

/// The station's solve, as numbers: run before and after a change to the solve.
#[test]
#[ignore = "a table: cargo test --release --test fluid table_station -- --ignored --nocapture"]
fn table_station() {
    let tuned = std::env::var("SX_ONE_EURO")
        .ok()
        .map(|v| {
            v.split(',')
                .filter_map(|x| x.trim().parse().ok())
                .collect::<Vec<f64>>()
        })
        .and_then(|v| match v[..] {
            [
                min_cutoff_hz,
                beta_per_m_s,
                beta_per_rad_s,
                derivative_cutoff_hz,
            ] => Some(sx_embodiment_ik::smoothing::OneEuro {
                min_cutoff_hz,
                beta_per_m_s,
                beta_per_rad_s,
                derivative_cutoff_hz,
            }),
            _ => None,
        })
        .unwrap_or(sx_embodiment_ik::smoothing::OneEuro::TOOL);
    let filters = if std::env::var("SX_ONE_EURO").is_ok() {
        vec![Some(tuned)]
    } else {
        vec![None, Some(tuned)]
    };
    for filter in filters {
        println!("\n== the operator's tremor filter: {filter:?}");
        let mut station = Station::new();
        station.solve.set_input_filter(filter);
        station_table(&mut station);
    }
}

fn station_table(station: &mut Station) {
    // A 10 cm jump each way: time to within 5 mm and 1 mm.
    let mut closing = Vec::new();
    for (k, sign) in [
        (0, 1.0),
        (0, -1.0),
        (1, 1.0),
        (1, -1.0),
        (2, 1.0),
        (2, -1.0),
    ] {
        let mut by = [0.0; 3];
        by[k] = 0.1 * sign;
        let (run, _) = station.run(200, |_, from| shifted(from, by));
        let first = |within: f64| {
            run.iter()
                .position(|(target, at)| distance(target, at) < within)
                .map_or(f64::NAN, |i| (i + 1) as f64 * DT)
        };
        closing.push((first(0.005), first(0.001)));
        // How far the tool went past the target along the jump, and where it ended.
        let past = run
            .iter()
            .map(|(target, at)| sign * (at.translation_m[k] - target.translation_m[k]))
            .fold(0.0_f64, f64::max);
        println!(
            "jump {:+.1} m on {}: within 5 mm at {:.2} s, 1 mm at {:.2} s; overshoot {:.1} mm, after 2 s {:.1} mm off",
            by[k],
            ["x", "y", "z"][k],
            first(0.005),
            first(0.001),
            past * 1e3,
            distance(&run[run.len() - 1].0, &run[run.len() - 1].1) * 1e3
        );
    }
    let mean = |f: fn(&(f64, f64)) -> f64| closing.iter().map(f).sum::<f64>() / 6.0;
    let worst = |f: fn(&(f64, f64)) -> f64| closing.iter().map(f).fold(0.0, f64::max);
    println!(
        "JUMP 10 cm: to 5 mm mean {:.3} s worst {:.3} s; to 1 mm mean {:.3} s worst {:.3} s",
        mean(|c| c.0),
        worst(|c| c.0),
        mean(|c| c.1),
        worst(|c| c.1)
    );
    // A 0.5 m/s sweep of 15 cm: the lag behind it, then the settle.
    for (k, axis) in [(1, "y"), (2, "z")] {
        let (run, _) = station.run(150, |t: f64, from: &Pose| {
            let mut by = [0.0; 3];
            by[k] = 0.5 * t.min(0.3);
            shifted(from, by)
        });
        let lags: Vec<f64> = run[15..30].iter().map(|(a, b)| distance(a, b)).collect();
        let settle = run[30..]
            .iter()
            .position(|(a, b)| distance(a, b) < 0.001)
            .map_or(f64::NAN, |i| (i + 1) as f64 * DT);
        println!(
            "SWEEP 0.5 m/s on {axis}: lag mean {:.1} mm, at the end {:.1} mm; settled to 1 mm {:.2} s after",
            lags.iter().sum::<f64>() / lags.len() as f64 * 1e3,
            lags[lags.len() - 1] * 1e3,
            settle
        );
    }
    // Sines of 3 cm at 1 and 2 Hz: RMS error and the delay that best explains the tool.
    for hz in [1.0, 2.0] {
        let (run, _) = station.run(300, |t: f64, from: &Pose| {
            shifted(
                from,
                [0.0, 0.03 * (std::f64::consts::TAU * hz * t).sin(), 0.0],
            )
        });
        let tail = &run[100..];
        let error = rms(&tail.iter().map(|(a, b)| distance(a, b)).collect::<Vec<_>>());
        let delay = (0..40)
            .min_by(|&x, &y| {
                let fit = |d: usize| {
                    rms(&(d..tail.len())
                        .map(|i| tail[i].1.translation_m[1] - tail[i - d].0.translation_m[1])
                        .collect::<Vec<_>>())
                };
                fit(x).total_cmp(&fit(y))
            })
            .unwrap_or(0);
        println!(
            "SINE {hz} Hz 3 cm: RMS error {:.1} mm, delay {:.0} ms",
            error * 1e3,
            delay as f64 * DT * 1e3
        );
    }
    // The tremor trace on the left hand.
    let mut tremor = Tremor::new(11);
    let mut smooth = Vec::new();
    let (run, history) = station.run(300, |t, from| {
        let (path, target) = tremor.at(t, from);
        smooth.push(path);
        target
    });
    let track = rms(&run[50..]
        .iter()
        .map(|(a, b)| distance(a, b))
        .collect::<Vec<_>>());
    let path = rms(&run[50..]
        .iter()
        .zip(&smooth[50..])
        .map(|((_, b), s)| distance(s, b))
        .collect::<Vec<_>>());
    let columns: Vec<usize> = station.body.limb_axes(0).to_vec();
    let (null_jerk, jerk) = jerks(&station.body, &history[50..], &[0], &columns);
    println!(
        "TREMOR: RMS to target {:.2} mm, to the smooth path {:.2} mm; left arm null-space jerk {:.0} rad/s^3, joint jerk {:.0} rad/s^3",
        track * 1e3,
        path * 1e3,
        null_jerk,
        jerk
    );
    let mut times: Vec<f64> = station
        .times
        .iter()
        .map(|d| d.as_secs_f64() * 1e6)
        .collect();
    times.sort_by(f64::total_cmp);
    let at = |p: f64| times[((times.len() - 1) as f64 * p) as usize];
    println!(
        "TICK over {} steps: median {:.0} us, p99 {:.0} us, max {:.0} us; past the 4 ms budget {}",
        times.len(),
        at(0.5),
        at(0.99),
        times[times.len() - 1],
        station.held
    );
}

/// How much of a lagging follower's tool lag the horizon's lag model wins back
/// ([`Ik::set_follower_lag`]): each joint follows its command as a first-order lag of 40 ms
/// (the Spark's 26-63 ms), measured where it is, and the station's solve is told that lag or not.
/// The tool as measured against the target on a 0.5 m/s sweep and 1 and 2 Hz sines.
#[test]
#[ignore = "a table: cargo test --release --test fluid table_follower_lag -- --ignored --nocapture"]
fn table_follower_lag() {
    const LAG_S: f64 = 0.04;
    for modelled in [0.0, LAG_S] {
        let mut station = Station::new();
        let names: Vec<String> = station
            .solve
            .model()
            .axes()
            .iter()
            .map(|a| a.name.clone())
            .collect();
        for name in &names {
            station.solve.set_follower_lag(name, modelled);
        }
        let runs: [(&str, Box<dyn Fn(f64, &Pose) -> Pose>); 3] = [
            (
                "sweep 0.5 m/s",
                Box::new(|t: f64, from: &Pose| shifted(from, [0.0, 0.5 * t.min(0.3), 0.0])),
            ),
            (
                "sine 1 Hz",
                Box::new(|t: f64, from: &Pose| {
                    shifted(from, [0.0, 0.03 * (std::f64::consts::TAU * t).sin(), 0.0])
                }),
            ),
            (
                "sine 2 Hz",
                Box::new(|t: f64, from: &Pose| {
                    shifted(
                        from,
                        [0.0, 0.03 * (std::f64::consts::TAU * 2.0 * t).sin(), 0.0],
                    )
                }),
            ),
        ];
        for (name, left) in runs {
            let mut command = station.ready.clone();
            let mut actual = station.ready.clone();
            let (from, right) = (
                pose(&station.body, &command, 0),
                pose(&station.body, &command, 1),
            );
            let available = vec![true; command.len()];
            station.solve.reset();
            let mut errors = Vec::new();
            for tick in 1..=300 {
                let t = f64::from(tick) * DT;
                let target = left(t, &from);
                let targets = Targets {
                    tcp: [Some(target), Some(right)],
                    ..Targets::default()
                };
                let measured: Vec<Option<f64>> = actual.iter().map(|v| Some(*v)).collect();
                let step = station
                    .solve
                    .step(&command, &measured, &available, &targets, DT)
                    .expect("a step");
                command.copy_from_slice(&step.q);
                let blend = 1.0 - (-DT / LAG_S).exp();
                for (a, c) in actual.iter_mut().zip(&command) {
                    *a += blend * (c - *a);
                }
                if t > 0.15 {
                    errors.push(distance(&target, &pose(&station.body, &actual, 0)));
                }
            }
            println!(
                "lag modelled {modelled:.2} s, {name}: measured tool to target RMS {:.1} mm",
                rms(&errors) * 1e3
            );
        }
    }
}
