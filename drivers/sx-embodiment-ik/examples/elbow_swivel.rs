//! Elbow swivel of each arm over a session export: the elbow's direction off the shoulder-wrist
//! line, as an angle from straight down (0 = hanging, 90 = out sideways, 180 = up), per 30-degree
//! bin. The arms are the body's own (`BodyFiles::limbs`, each hand's `arm`: shoulder, elbow and
//! wrist links), so any registry body with arms is measured the same way.
//!
//! Copy into an embodiments checkout and run on robot-bench export JSONs: commanded joints
//! (`{"names": [...], "commands": [[t_ns, [q...]], ...]}`, `control_journal_lag.py export`) or
//! measured ones (`{"names": [...], "q": [[q...], ...]}`, one row a sample). It prints the
//! table, then one JSON line: per arm, the share of rows per 30-degree bin and p50 / p95 degrees.
//!
//! ```bash
//! cp .agents/skills/robot-bench/scripts/elbow_swivel.rs \
//!    packages/sx-embodiments/drivers/sx-embodiment-ik/examples/
//! cargo run --release -p sx-embodiment-ik --example elbow_swivel -- /tmp/eve/json/41eb2d39_e40e9290.json
//! # another body: a registry name second
//! cargo run --release -p sx-embodiment-ik --example elbow_swivel -- export.json sentient-static-upper-body
//! ```

// A measurement tool run by hand (robot-bench `retarget_bench`), never shipped: its numeric
// casts and terse names are a script's, so clippy is off here.
#![allow(clippy::all, clippy::pedantic, clippy::nursery)]
use sx_embodiment_ik::{elbow_direction, registered};

fn main() {
    let path = std::env::args().nth(1).expect("an export json");
    let body_name = std::env::args()
        .nth(2)
        .unwrap_or_else(|| "sentient-static-upper-body".to_owned());
    let v: serde_json::Value =
        serde_json::from_slice(&std::fs::read(path).expect("read")).expect("json");
    let names: Vec<String> = v["names"]
        .as_array()
        .expect("names")
        .iter()
        .map(|n| n.as_str().expect("name").to_owned())
        .collect();
    let files = registered(&body_name).expect("a committed body");
    let body = files.body().expect("body");
    let model = body.model();
    let arms: Vec<(String, [usize; 3])> = files
        .limbs()
        .expect("limbs")
        .into_iter()
        .filter_map(|limb| {
            let arm = limb.arm?;
            let link = |n: &str| model.link(n).expect("arm link");
            Some((
                limb.name,
                [link(&arm.shoulder), link(&arm.elbow), link(&arm.wrist)],
            ))
        })
        .collect();
    let axes: Vec<String> = model.axes().iter().map(|a| a.name.clone()).collect();
    let mut hist = vec![[0_usize; 6]; arms.len()];
    let mut angles: Vec<Vec<f64>> = vec![Vec::new(); arms.len()];
    let mut rows = 0;
    // Commanded rows carry their time first; measured rows are the joints alone.
    let (series, of_row): (
        &Vec<serde_json::Value>,
        fn(&serde_json::Value) -> &serde_json::Value,
    ) = match v["commands"].as_array() {
        Some(commands) => (commands, |row| &row[1]),
        None => (v["q"].as_array().expect("commands or q"), |row| row),
    };
    for row in series {
        let given: Vec<f64> = of_row(row)
            .as_array()
            .expect("q")
            .iter()
            .map(|x| x.as_f64().unwrap_or(0.0))
            .collect();
        let q: Vec<f64> = axes
            .iter()
            .map(|a| names.iter().position(|n| n == a).map_or(0.0, |j| given[j]))
            .collect();
        let frames = model.poses(&q);
        rows += 1;
        for (index, (_, [s, e, w])) in arms.iter().enumerate() {
            let p = |link: usize| frames.links[link].t;
            if let Some(angle) = swivel_from_down(p(*s), p(*e), p(*w)) {
                hist[index][((angle / 30.0) as usize).min(5)] += 1;
                angles[index].push(angle);
            }
        }
    }
    println!("{rows} rows; elbow swivel from straight down, share of rows per 30-degree bin");
    for ((name, _), bins) in arms.iter().zip(&hist) {
        let total: usize = bins.iter().sum();
        let shares: Vec<String> = bins
            .iter()
            .enumerate()
            .map(|(i, c)| {
                format!(
                    "{}-{}: {:.0}%",
                    i * 30,
                    i * 30 + 30,
                    100.0 * *c as f64 / total.max(1) as f64
                )
            })
            .collect();
        println!("  {name:5} {}", shares.join("  "));
    }
    let summary: serde_json::Map<String, serde_json::Value> = arms
        .iter()
        .zip(&hist)
        .zip(&mut angles)
        .map(|(((name, _), bins), angles)| {
            angles.sort_by(f64::total_cmp);
            let total: usize = bins.iter().sum();
            let pick = |p: f64| {
                angles
                    .get(((p * angles.len() as f64) as usize).min(angles.len().saturating_sub(1)))
                    .copied()
            };
            (
                name.clone(),
                serde_json::json!({
                    "rows": total,
                    "share": bins.iter().map(|c| *c as f64 / total.max(1) as f64).collect::<Vec<_>>(),
                    "p50_deg": pick(0.5),
                    "p95_deg": pick(0.95),
                }),
            )
        })
        .collect();
    println!("{}", serde_json::Value::Object(summary));
}

/// The elbow's angle off the shoulder-wrist line from straight down (root -z projected off the
/// line), degrees; `None` for an arm within a centimetre of straight.
fn swivel_from_down(s: [f64; 3], e: [f64; 3], w: [f64; 3]) -> Option<f64> {
    let (off, radius) = elbow_direction(s, e, w)?;
    if radius < 0.01 {
        return None;
    }
    let axis = [0, 1, 2].map(|k| w[k] - s[k]);
    let length = axis.iter().map(|x| x * x).sum::<f64>().sqrt();
    let d = axis.map(|x| x / length);
    let down = [0.0, 0.0, -1.0];
    let along: f64 = (0..3).map(|k| d[k] * down[k]).sum();
    let across = [0, 1, 2].map(|k| down[k] - d[k] * along);
    let n = across.iter().map(|x| x * x).sum::<f64>().sqrt();
    let cos = (0..3).map(|k| off[k] * across[k]).sum::<f64>() / n.max(1e-12);
    Some(cos.clamp(-1.0, 1.0).acos().to_degrees())
}
