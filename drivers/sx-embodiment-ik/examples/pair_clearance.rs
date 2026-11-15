//! Per capsule pair clearance (m) at Ready or at given poses. usage: pair_clearance [poses.json]
//! poses.json: {"names": [...], "q": [[...], ...]} ; prints per pair min/median and share < margin.

// A measurement tool run by hand (robot-bench `retarget_bench`), never shipped: its numeric
// casts and terse names are a script's, so clippy is off here.
#![allow(clippy::all, clippy::pedantic, clippy::nursery)]
use sx_embodiment_ik::closest_segments;
fn main() {
    let files = sx_embodiment_ik::registered("sentient-static-upper-body").expect("pack");
    let body = files.body().expect("body");
    let model = body.model();
    let names: Vec<String> = model.axes().iter().map(|a| a.name.clone()).collect();
    let ready = files.ready().expect("ready");
    let arg = std::env::args().nth(1);
    let poses: Vec<Vec<f64>> = match &arg {
        None => vec![
            names
                .iter()
                .map(|n| ready.get(n).copied().unwrap_or(0.0))
                .collect(),
        ],
        Some(p) => {
            let v: serde_json::Value = serde_json::from_slice(&std::fs::read(p).unwrap()).unwrap();
            let given: Vec<String> = serde_json::from_value(v["names"].clone()).unwrap();
            let rows: Vec<Vec<Option<f64>>> = serde_json::from_value(v["q"].clone()).unwrap();
            rows.iter()
                .map(|r| {
                    names
                        .iter()
                        .map(|n| {
                            given
                                .iter()
                                .position(|g| g == n)
                                .and_then(|i| r[i])
                                .unwrap_or_else(|| ready.get(n).copied().unwrap_or(0.0))
                        })
                        .collect()
                })
                .collect()
        }
    };
    let proxies = body.proxies();
    let links = model.links();
    let mut per: Vec<Vec<f64>> = vec![Vec::new(); proxies.pairs.len()];
    for q in &poses {
        let frames = model.poses(q);
        for (k, &(i, j)) in proxies.pairs.iter().enumerate() {
            let (a, b) = (&proxies.capsules[i], &proxies.capsules[j]);
            let (pa, pb) = (&frames.links[a.link], &frames.links[b.link]);
            let (a0, a1, b0, b1) = (pa.apply(a.a), pa.apply(a.b), pb.apply(b.a), pb.apply(b.b));
            let (x, y) = closest_segments(a0, a1, b0, b1);
            let mut d = ((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2) + (x[2] - y[2]).powi(2))
                .sqrt()
                - a.radius
                - b.radius;
            if d < 0.02 {
                if let Some((h, _, _)) = sx_embodiment_ik::hull::link_distance(
                    &proxies.pieces,
                    &proxies.pieces_of[a.link],
                    pa,
                    &proxies.pieces_of[b.link],
                    pb,
                ) {
                    if h > d {
                        d = h;
                    }
                }
            }
            per[k].push(d);
        }
    }
    let mut rows: Vec<(f64, String)> = per
        .iter()
        .enumerate()
        .map(|(k, d)| {
            let mut s = d.clone();
            s.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let (i, j) = proxies.pairs[k];
            let (a, b) = (&proxies.capsules[i], &proxies.capsules[j]);
            let under = d.iter().filter(|x| **x < 0.01).count() as f64 / d.len() as f64;
            (
                s[0],
                format!(
                    "{:>8.1} mm min {:>8.1} p50 {:>5.1}% <10mm  {} (r {:.3}) / {} (r {:.3})",
                    s[0] * 1e3,
                    s[s.len() / 2] * 1e3,
                    under * 100.0,
                    links[a.link],
                    a.radius,
                    links[b.link],
                    b.radius
                ),
            )
        })
        .collect();
    rows.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    for (_, r) in rows.iter().take(12) {
        println!("{r}");
    }
}
