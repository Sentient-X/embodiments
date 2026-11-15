//! Mesh-accurate (hull) distance of every pair with a neck (so101_ / camera) link, over random
//! poses of the neck's whole joint range, the rest of the body at Ready. Prints per pair the
//! least hull distance, the share of poses under 10 mm, and the capsule distance there.

// A measurement tool run by hand (robot-bench `retarget_bench`), never shipped: its numeric
// casts and terse names are a script's, so clippy is off here.
#![allow(clippy::all, clippy::pedantic, clippy::nursery)]
use sx_embodiment_ik::closest_segments;
fn main() {
    let files = sx_embodiment_ik::registered("sentient-static-upper-body").expect("pack");
    let body = files.body().expect("body");
    let model = body.model();
    let axes = model.axes();
    let ready = files.ready().expect("ready");
    let base: Vec<f64> = axes
        .iter()
        .map(|a| ready.get(&a.name).copied().unwrap_or(0.0))
        .collect();
    let neck: Vec<usize> = axes
        .iter()
        .enumerate()
        .filter(|(_, a)| a.name.starts_with("so101_"))
        .map(|(i, _)| i)
        .collect();
    let proxies = body.proxies();
    let links = model.links();
    let is_neck = |l: usize| {
        links[l].starts_with("so101_") || links[l].contains("camera") || links[l].contains("ZED")
    };
    let pairs: Vec<(usize, usize)> = proxies
        .pairs
        .iter()
        .copied()
        .filter(|&(i, j)| is_neck(proxies.capsules[i].link) || is_neck(proxies.capsules[j].link))
        .collect();
    let n: usize = std::env::args()
        .nth(1)
        .and_then(|v| v.parse().ok())
        .unwrap_or(20000);
    let mut state = 7u64;
    let mut next = || {
        state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (state >> 11) as f64 / (1u64 << 53) as f64
    };
    let mut least = vec![(f64::INFINITY, f64::INFINITY, Vec::new()); pairs.len()];
    let mut under = vec![0usize; pairs.len()];
    for _ in 0..n {
        let mut q = base.clone();
        for &a in &neck {
            if let Some([lo, hi]) = axes[a].limits {
                q[a] = lo + (hi - lo) * next();
            }
        }
        let frames = model.poses(&q);
        for (k, &(i, j)) in pairs.iter().enumerate() {
            let (a, b) = (&proxies.capsules[i], &proxies.capsules[j]);
            let (pa, pb) = (&frames.links[a.link], &frames.links[b.link]);
            let (x, y) =
                closest_segments(pa.apply(a.a), pa.apply(a.b), pb.apply(b.a), pb.apply(b.b));
            let cap = ((x[0] - y[0]).powi(2) + (x[1] - y[1]).powi(2) + (x[2] - y[2]).powi(2))
                .sqrt()
                - a.radius
                - b.radius;
            let hull = if cap < 0.03 {
                sx_embodiment_ik::hull::link_distance(
                    &proxies.pieces,
                    &proxies.pieces_of[a.link],
                    pa,
                    &proxies.pieces_of[b.link],
                    pb,
                )
                .map_or(cap, |h| h.0.max(cap))
            } else {
                cap
            };
            if hull < 0.01 {
                under[k] += 1;
            }
            if hull < least[k].0 {
                least[k] = (hull, cap, neck.iter().map(|&a| q[a]).collect());
            }
        }
    }
    let mut order: Vec<usize> = (0..pairs.len()).collect();
    order.sort_by(|&a, &b| least[a].0.partial_cmp(&least[b].0).unwrap());
    let names: Vec<&str> = neck.iter().map(|&a| axes[a].name.as_str()).collect();
    println!("neck axes {names:?}, {n} poses");
    for k in order.into_iter().take(200) {
        let (i, j) = pairs[k];
        let (a, b) = (&proxies.capsules[i], &proxies.capsules[j]);
        println!(
            "{:>7.1} mm hull ({:>7.1} capsule) {:>5.1}% <10mm  {} r{:.3} / {} r{:.3}  at {:?}",
            least[k].0 * 1e3,
            least[k].1 * 1e3,
            100.0 * under[k] as f64 / n as f64,
            links[a.link],
            a.radius,
            links[b.link],
            b.radius,
            least[k]
                .2
                .iter()
                .map(|v| (v * 100.0).round() / 100.0)
                .collect::<Vec<_>>()
        );
    }
}
