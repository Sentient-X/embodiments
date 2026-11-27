//! A side's wrist tags as they were sighted, frame by frame: the one record between the
//! detector and the solve, whether the headset sighted them online from the frames it was
//! encoding or the Library sighted them later from the decoded clip.
//!
//! `gripper_{side}.jaw_tags.csv`:
//!
//! ```text
//! # sx.jaw-tags/v1 side=<left|right> ids=<a>,<b>
//! t_monotonic_ns,marker_id,x0,y0,x1,y1,x2,y2,x3,y3
//! 1200000000,,,,,,,,,
//! 1233333333,0,101.250,340.500,...
//! 1233333333,1,...
//! ```
//!
//! One row per marker sighted in an analysed frame, at the frame's exposure on the host's
//! `CLOCK_MONOTONIC`, its corners in full-frame pixels in OpenCV's order; an analysed frame
//! with no marker is one row with the ID and corners empty. A frame absent from the file was
//! not analysed (skipped by the cadence because no marker was in sight, or dropped because the
//! sighter was busy). Stamps never decrease. A sighter that saw the take through ends the file
//! with `# ended offered=<frames> dropped=<busy>`; one that stopped before the take ended (the
//! headset ran hot, its battery fell) with `# stopped <reason>`, so no solve mistakes the rest
//! for unseen. A file with neither was cut off.

use crate::detect::Marker;

pub const SCHEMA: &str = "sx.jaw-tags/v1";
pub const HEADER: &str = "t_monotonic_ns,marker_id,x0,y0,x1,y1,x2,y2,x3,y3";

/// The sightings file's name after `gripper_{side}`.
pub fn file_name(side: &str) -> String {
    format!("gripper_{side}.jaw_tags.csv")
}

/// One analysed frame.
#[derive(Clone, Debug, PartialEq)]
pub struct Sighting {
    pub t_ns: i64,
    pub markers: Vec<Marker>,
}

/// The file's first two lines.
pub fn preamble(side: &str, ids: [u32; 2]) -> String {
    format!(
        "# {SCHEMA} side={side} ids={},{}\n{HEADER}\n",
        ids[0], ids[1]
    )
}

/// One analysed frame's rows.
pub fn rows(sighting: &Sighting) -> String {
    use std::fmt::Write as _;
    let mut out = String::new();
    if sighting.markers.is_empty() {
        let _ = writeln!(out, "{},,,,,,,,,", sighting.t_ns);
    }
    for marker in &sighting.markers {
        let _ = write!(out, "{},{}", sighting.t_ns, marker.id);
        for corner in &marker.corners {
            let _ = write!(out, ",{:.3},{:.3}", corner[0], corner[1]);
        }
        out.push('\n');
    }
    out
}

/// The line a sighter that stopped early ends its file with, newline included.
pub fn stopped(reason: &str) -> String {
    format!("# stopped {reason}\n")
}

/// The line a sighter that saw the take through ends its file with: the frames it was offered
/// and how many of them it dropped busy.
pub fn ended(offered: u64, dropped: u64) -> String {
    format!("# ended offered={offered} dropped={dropped}\n")
}

/// How a sightings file ends.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Ending {
    /// Seen through: the frames offered and dropped busy.
    Ended { offered: u64, dropped: u64 },
    /// Stopped before the take ended, and why.
    Stopped(String),
    /// Neither line: the file was cut off.
    CutOff,
}

/// A sightings file read back.
#[derive(Clone, Debug, PartialEq)]
pub struct Sightings {
    pub sightings: Vec<Sighting>,
    pub ids: [u32; 2],
    pub ending: Ending,
}

/// The sightings in `text`, for `side`. A torn last line (a take cut off mid-write) is
/// dropped; anything else malformed refuses the file.
pub fn parse(text: &str, side: &str) -> Result<Sightings, String> {
    let mut lines = text.split_inclusive('\n');
    let first = lines.next().ok_or("the sightings file is empty")?;
    let body = first
        .trim_end()
        .strip_prefix("# ")
        .and_then(|rest| rest.strip_prefix(SCHEMA))
        .ok_or_else(|| format!("the sightings file does not open with # {SCHEMA}"))?;
    let mut named_side = None;
    let mut ids = None;
    for token in body.split_whitespace() {
        match token.split_once('=') {
            Some(("side", value)) => named_side = Some(value.to_owned()),
            Some(("ids", value)) => {
                let parsed: Vec<u32> =
                    value
                        .split(',')
                        .map(str::parse)
                        .collect::<Result<_, _>>()
                        .map_err(|_| format!("ids {value:?} are not two marker IDs"))?;
                if let [a, b] = parsed.as_slice() {
                    ids = Some([*a, *b]);
                } else {
                    return Err(format!("ids {value:?} are not two marker IDs"));
                }
            }
            _ => return Err(format!("the sightings file names an unknown {token:?}")),
        }
    }
    if named_side.as_deref() != Some(side) {
        return Err(format!("the sightings file is not the {side} side's"));
    }
    let ids = ids.ok_or("the sightings file names no marker IDs")?;
    if lines.next().map(str::trim_end) != Some(HEADER) {
        return Err("the sightings file has no header".to_owned());
    }
    let mut out: Vec<Sighting> = Vec::new();
    let mut ending = Ending::CutOff;
    for line in lines {
        if !line.ends_with('\n') {
            break;
        }
        if let Some(reason) = line.trim_end().strip_prefix("# stopped ") {
            ending = Ending::Stopped(reason.to_owned());
            break;
        }
        if let Some(counts) = line.trim_end().strip_prefix("# ended ") {
            let count = |key: &str| {
                counts
                    .split_whitespace()
                    .find_map(|kv| kv.strip_prefix(key)?.strip_prefix('='))
                    .and_then(|v| v.parse().ok())
                    .ok_or_else(|| format!("the sightings file's end {counts:?} has no {key}"))
            };
            ending = Ending::Ended {
                offered: count("offered")?,
                dropped: count("dropped")?,
            };
            break;
        }
        let fields: Vec<&str> = line.trim_end().split(',').collect();
        if fields.len() != 10 {
            return Err(format!(
                "sightings row {line:?} has {} fields, not 10",
                fields.len()
            ));
        }
        let t_ns: i64 = fields[0]
            .parse()
            .map_err(|_| format!("sightings row {line:?} has no stamp"))?;
        if out.last().is_some_and(|last| last.t_ns > t_ns) {
            return Err(format!("sightings stamp {t_ns} goes back"));
        }
        if out.last().is_none_or(|last| last.t_ns != t_ns) {
            out.push(Sighting {
                t_ns,
                markers: Vec::new(),
            });
        }
        if fields[1].is_empty() {
            continue;
        }
        let id: u32 = fields[1]
            .parse()
            .map_err(|_| format!("sightings row {line:?} has no marker ID"))?;
        let mut corners = [[0.0; 2]; 4];
        for (k, corner) in corners.iter_mut().enumerate() {
            for (axis, value) in corner.iter_mut().enumerate() {
                *value = fields[2 + 2 * k + axis]
                    .parse()
                    .ok()
                    .filter(|v: &f64| v.is_finite())
                    .ok_or_else(|| {
                        format!("sightings row {line:?} has a corner that is no number")
                    })?;
            }
        }
        if let Some(last) = out.last_mut() {
            last.markers.push(Marker { id, corners });
        }
    }
    Ok(Sightings {
        sightings: out,
        ids,
        ending,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sightings_round_trip_and_a_torn_line_is_dropped() {
        let sightings = vec![
            Sighting {
                t_ns: 100,
                markers: Vec::new(),
            },
            Sighting {
                t_ns: 133,
                markers: vec![
                    Marker {
                        id: 0,
                        corners: [[1.5, 2.0], [3.0, 4.0], [5.0, 6.0], [7.0, 8.25]],
                    },
                    Marker {
                        id: 1,
                        corners: [[11.0, 12.0], [13.0, 14.0], [15.0, 16.0], [17.0, 18.0]],
                    },
                ],
            },
        ];
        let mut text = preamble("right", [0, 1]);
        for s in &sightings {
            text.push_str(&rows(s));
        }
        let whole = text.clone();
        text.push_str("166,0,1.0");
        let back = parse(&text, "right").unwrap();
        assert_eq!(back.ids, [0, 1]);
        assert_eq!(back.sightings, sightings);
        assert_eq!(back.ending, Ending::CutOff);
        assert!(parse(&text, "left").is_err());
        let hot = parse(&(whole.clone() + &stopped("thermal_moderate")), "right").unwrap();
        assert_eq!(hot.sightings, sightings);
        assert_eq!(hot.ending, Ending::Stopped("thermal_moderate".into()));
        let seen = parse(&(whole + &ended(3, 1)), "right").unwrap();
        assert_eq!(seen.ending, Ending::Ended { offered: 3, dropped: 1 });
    }
}
