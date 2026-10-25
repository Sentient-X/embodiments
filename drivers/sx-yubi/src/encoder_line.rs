//! The ESP32-C6 gripper encoder's line protocol: `<angle>\n` or `<device id>,<angle>\n`, the
//! angle in radians from an AS5601. Shared by the laptop pod (tty) and the headset rig
//! (CDC-ACM over usbfs), so both refuse exactly the same lines.

/// The YUBI gripper's mechanical limit, radians.
pub const MAX_ANGLE_RAD: f64 = 0.959_931_1;
/// Bytes kept of one line; the remainder of a longer line is dropped.
pub const MAX_LINE: usize = 256;

/// Parse one line (without its newline). `Ok(None)` is a line that carries no sample (a
/// boot banner, a log line); `Err` is a sample the rig must refuse.
pub fn parse(line: &[u8]) -> Result<Option<(Option<String>, f64)>, String> {
    let text = String::from_utf8_lossy(line);
    let (reported_id, value) = match text.split_once(',') {
        Some((reported_id, value)) => {
            let reported_id = reported_id.trim();
            if reported_id.is_empty() {
                return Err("encoder firmware emitted an empty device identity".to_owned());
            }
            (Some(reported_id.to_owned()), value.trim())
        }
        None => (None, text.trim()),
    };
    let Ok(raw) = value.parse::<f64>() else {
        return Ok(None);
    };
    if !raw.is_finite() || !(-std::f64::consts::TAU..=std::f64::consts::TAU).contains(&raw) {
        return Err(format!(
            "encoder emitted a value outside the AS5601 range: {raw}"
        ));
    }
    let angle = ((raw + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI)
        .abs();
    if angle > MAX_ANGLE_RAD {
        return Err(format!(
            "encoder joint angle {angle} exceeds the YUBI limit {MAX_ANGLE_RAD}"
        ));
    }
    Ok(Some((reported_id, angle)))
}

/// Splits a serial byte stream into lines. The first line may be the tail of one cut when
/// the port opened, so it is discarded; a line longer than `MAX_LINE` keeps its first bytes.
pub struct Lines {
    line: Vec<u8>,
    discard_first: bool,
}

impl Default for Lines {
    fn default() -> Self {
        Self {
            line: Vec::with_capacity(MAX_LINE),
            discard_first: true,
        }
    }
}

impl Lines {
    /// Feed bytes; `each` sees every complete line (without its newline) in order and may
    /// stop the feed by returning an error.
    pub fn feed<E>(
        &mut self,
        bytes: &[u8],
        mut each: impl FnMut(&[u8]) -> Result<(), E>,
    ) -> Result<(), E> {
        for byte in bytes {
            if *byte == b'\n' {
                let result = if self.discard_first {
                    self.discard_first = false;
                    Ok(())
                } else {
                    each(&self.line)
                };
                self.line.clear();
                result?;
            } else if self.line.len() < MAX_LINE {
                self.line.push(*byte);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn samples_with_and_without_identity_parse_and_fold_into_the_joint_range() {
        assert_eq!(parse(b"0.25").unwrap(), Some((None, 0.25)));
        assert_eq!(parse(b" -0.25\r").unwrap(), Some((None, 0.25)));
        let (id, angle) = parse(b"enc-7,6.0").unwrap().unwrap();
        assert_eq!(id.as_deref(), Some("enc-7"));
        assert!((angle - (std::f64::consts::TAU - 6.0)).abs() < 1e-12);
        assert_eq!(parse(b"ESP-ROM:esp32c6-20220919").unwrap(), None);
    }

    #[test]
    fn out_of_range_and_anonymous_samples_are_refused() {
        assert!(
            parse(b"3.14")
                .unwrap_err()
                .contains("exceeds the YUBI limit")
        );
        assert!(
            parse(b"7.0")
                .unwrap_err()
                .contains("outside the AS5601 range")
        );
        assert!(
            parse(b"NaN")
                .unwrap_err()
                .contains("outside the AS5601 range")
        );
        assert!(
            parse(b" ,0.1")
                .unwrap_err()
                .contains("empty device identity")
        );
    }

    #[test]
    fn the_first_cut_line_is_discarded_and_long_lines_are_capped() {
        let mut lines = Lines::default();
        let mut seen = Vec::new();
        let long = vec![b'9'; 300];
        lines
            .feed::<()>(b"25\n0.10\n0.2", |l| {
                seen.push(l.to_vec());
                Ok(())
            })
            .unwrap();
        lines
            .feed::<()>(&[b"0\n".as_slice(), &long, b"\n"].concat(), |l| {
                seen.push(l.to_vec());
                Ok(())
            })
            .unwrap();
        assert_eq!(seen[0], b"0.10");
        assert_eq!(seen[1], b"0.20");
        assert_eq!(seen[2].len(), MAX_LINE);
        let error = lines.feed(b"x\n", |_| Err("stop")).unwrap_err();
        assert_eq!(error, "stop");
    }
}
