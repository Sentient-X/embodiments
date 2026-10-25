//! Bit timing for a channel's nominal and data phases (`docs/PROTOCOL.md` §3.3).

use std::time::Duration;

use thiserror::Error;

use crate::wire::Timing;

/// A channel's bit rates: classic CAN, or CAN FD with a faster data phase.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bitrate {
    /// Classic CAN at this many bit/s; the data-phase fields repeat the nominal ones.
    Classic(u32),
    /// CAN FD: arbitration at `nominal`, data phase at `data`, bit/s.
    Fd { nominal: u32, data: u32 },
}

impl Bitrate {
    /// Classic CAN at 1 Mbit/s, the Damiao bus.
    pub const CLASSIC_1M: Self = Self::Classic(1_000_000);

    /// The nominal and data-phase timing on a controller clocked at `clock_hz`.
    ///
    /// # Errors
    ///
    /// Refuses a rate the clock cannot divide exactly within the device's ranges.
    pub fn timings(self, clock_hz: u32) -> Result<(Timing, Timing), TimingError> {
        match self {
            Self::Classic(rate) => {
                let nominal = nominal(clock_hz, rate)?;
                Ok((nominal, nominal))
            }
            Self::Fd {
                nominal: rate,
                data,
            } => Ok((nominal(clock_hz, rate)?, data_phase(clock_hz, data)?)),
        }
    }
}

/// A rate the clock cannot reach.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[error(
    "{rate} bit/s cannot be timed exactly from a {clock} Hz clock within the {phase} phase's ranges"
)]
pub struct TimingError {
    pub clock: u32,
    pub rate: u32,
    pub phase: &'static str,
}

/// One phase's permitted ranges (§3.3), real values.
struct Limits {
    tseg1: u16,
    tseg2: u16,
    sjw: u16,
    prescaler: u16,
    phase: &'static str,
}

const NOMINAL: Limits = Limits {
    tseg1: 256,
    tseg2: 128,
    sjw: 128,
    prescaler: 512,
    phase: "nominal",
};

const DATA: Limits = Limits {
    tseg1: 16,
    tseg2: 16,
    sjw: 16,
    prescaler: 32,
    phase: "data",
};

/// The nominal timing for `rate`. At 1 Mbit/s it is the specification's worked choice
/// (§7.1): 80 MHz, prescaler 2, TSEG1 29, TSEG2 10, SJW 2; 60 MHz, prescaler 1, TSEG1 44,
/// TSEG2 15 (SJW 2 here). Other rates are searched.
///
/// # Errors
///
/// Refuses a rate the clock cannot reach.
pub fn nominal(clock_hz: u32, rate: u32) -> Result<Timing, TimingError> {
    match (clock_hz, rate) {
        (80_000_000, 1_000_000) => Ok(Timing {
            prescaler: 2,
            tseg1: 29,
            tseg2: 10,
            sjw: 2,
        }),
        (60_000_000, 1_000_000) => Ok(Timing {
            prescaler: 1,
            tseg1: 44,
            tseg2: 15,
            sjw: 2,
        }),
        _ => {
            let sample = if rate >= 800_000 { 750 } else { 875 };
            search(clock_hz, rate, sample, 40, &NOMINAL)
        }
    }
}

/// The CAN FD data-phase timing for `rate`, sampled at 75 %.
///
/// # Errors
///
/// Refuses a rate the clock cannot reach.
pub fn data_phase(clock_hz: u32, rate: u32) -> Result<Timing, TimingError> {
    search(clock_hz, rate, 750, 25, &DATA)
}

/// The smallest prescaler giving an exact rate with at most `prefer_tq` time quanta, else the
/// smallest that fits at all; TSEG1 placed for `sample_permille`, SJW as wide as TSEG2 allows.
fn search(
    clock_hz: u32,
    rate: u32,
    sample_permille: u32,
    prefer_tq: u32,
    limits: &Limits,
) -> Result<Timing, TimingError> {
    let refused = TimingError {
        clock: clock_hz,
        rate,
        phase: limits.phase,
    };
    if rate == 0 {
        return Err(refused);
    }
    let candidates = (1..=u32::from(limits.prescaler)).filter_map(|prescaler| {
        let divisor = prescaler * rate;
        if !clock_hz.is_multiple_of(divisor) {
            return None;
        }
        let total = clock_hz / divisor;
        let tseg1 = (total * sample_permille + 500) / 1000;
        let tseg1 = u16::try_from(tseg1.checked_sub(1)?).ok()?;
        let tseg2 = u16::try_from(total.checked_sub(1)?.checked_sub(u32::from(tseg1))?).ok()?;
        if tseg1 == 0 || tseg2 == 0 || tseg1 > limits.tseg1 || tseg2 > limits.tseg2 {
            return None;
        }
        Some((
            total,
            Timing {
                prescaler: u16::try_from(prescaler).ok()?,
                tseg1,
                tseg2,
                sjw: tseg2.min(limits.sjw),
            },
        ))
    });
    let mut first = None;
    for (total, timing) in candidates {
        if total <= prefer_tq {
            return Ok(timing);
        }
        first.get_or_insert(timing);
    }
    first.ok_or(refused)
}

/// The longest one frame can hold the bus, worst-case bit stuffing included: what the transmit
/// path budgets for it. Classic CAN: `34 + 8n` stuffable bits (standard id; `54 + 8n` extended),
/// at most one stuff bit per four after the first, then 13 fixed bits (CRC delimiter, ACK slot
/// and delimiter, end of frame, interframe space). An 8-byte standard frame is 135 bits, 135 µs
/// at 1 Mbit/s. CAN FD: the arbitration fields and the 13 tail bits at the nominal rate, the
/// control, data, stuff count and CRC at the data rate when the frame switches bit rate.
#[must_use]
pub fn frame_time(bitrate: Bitrate, extended: bool, fd: bool, brs: bool, len: u8) -> Duration {
    let data = 8 * u64::from(len);
    let (nominal_rate, data_rate) = match bitrate {
        Bitrate::Classic(rate) => (rate, rate),
        Bitrate::Fd { nominal, data } => (nominal, data),
    };
    let ns = |bits: u64, rate: u32| bits * 1_000_000_000 / u64::from(rate.max(1));
    if !fd {
        let stuffable = if extended { 54 } else { 34 } + data;
        let bits = stuffable + (stuffable - 1) / 4 + 13;
        return Duration::from_nanos(ns(bits, nominal_rate));
    }
    let arbitration = if extended { 36 } else { 17 };
    let crc = if len > 16 { 21 } else { 17 };
    // ESI, DLC and the data stuff dynamically; the stuff count and CRC carry fixed stuff bits.
    let dynamic = 5 + data;
    let phase = dynamic + (dynamic - 1) / 4 + 4 + crc + crc.div_ceil(4) + 1;
    let nominal_bits = arbitration + (arbitration - 1) / 4 + 13;
    let rate = if brs { data_rate } else { nominal_rate };
    Duration::from_nanos(ns(nominal_bits, nominal_rate) + ns(phase, rate))
}

/// The bus time of the frame [`frame_time`] budgets most for at this bit rate among classic
/// ones: an 8-byte standard-id data frame, the Damiao bus's every frame.
#[must_use]
pub fn classic_frame_time(bitrate: Bitrate) -> Duration {
    frame_time(bitrate, false, false, false, 8)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rate(clock: u32, timing: Timing) -> u32 {
        clock
            / (u32::from(timing.prescaler)
                * (1 + u32::from(timing.tseg1) + u32::from(timing.tseg2)))
    }

    #[test]
    fn one_megabit_is_the_specs_worked_timing_on_both_clocks() {
        let (nominal, data) = Bitrate::CLASSIC_1M.timings(80_000_000).expect("80 MHz");
        assert_eq!(nominal, data, "classic repeats the nominal fields");
        assert_eq!(
            (nominal.prescaler, nominal.tseg1, nominal.tseg2, nominal.sjw),
            (2, 29, 10, 2)
        );
        assert_eq!(rate(80_000_000, nominal), 1_000_000);
        let (nominal, _) = Bitrate::CLASSIC_1M.timings(60_000_000).expect("60 MHz");
        assert_eq!(rate(60_000_000, nominal), 1_000_000);
        assert_eq!(1 + nominal.tseg1, 45, "75 % of 60 tq");
    }

    #[test]
    fn searched_rates_are_exact_and_inside_the_ranges() {
        for clock in [80_000_000, 60_000_000] {
            for nominal_rate in [125_000, 250_000, 500_000, 1_000_000] {
                for data_rate in [2_000_000, 4_000_000, 5_000_000] {
                    let Ok((n, d)) = (Bitrate::Fd {
                        nominal: nominal_rate,
                        data: data_rate,
                    })
                    .timings(clock) else {
                        // 60 MHz cannot reach 4 Mbit/s exactly.
                        assert_eq!((clock, data_rate), (60_000_000, 4_000_000));
                        continue;
                    };
                    assert_eq!(rate(clock, n), nominal_rate);
                    assert_eq!(rate(clock, d), data_rate);
                    assert!(d.tseg1 <= 16 && d.tseg2 <= 16 && d.prescaler <= 32 && d.sjw <= 16);
                    assert!(n.tseg1 <= 256 && n.tseg2 <= 128 && n.prescaler <= 512);
                }
            }
        }
        assert!(nominal(80_000_000, 0).is_err());
        assert!(data_phase(80_000_000, 7_000_000).is_err());
    }

    #[test]
    fn bus_time_is_the_worst_case_stuffed_frame() {
        let one_m = Bitrate::CLASSIC_1M;
        assert_eq!(classic_frame_time(one_m), Duration::from_micros(135));
        assert_eq!(
            frame_time(one_m, false, false, false, 0),
            Duration::from_micros(34 + 8 + 13)
        );
        assert_eq!(
            frame_time(one_m, true, false, false, 8),
            Duration::from_micros(118 + 29 + 13)
        );
        let fd = Bitrate::Fd {
            nominal: 1_000_000,
            data: 5_000_000,
        };
        let switched = frame_time(fd, false, true, true, 64);
        let unswitched = frame_time(fd, false, true, false, 64);
        assert!(switched < unswitched / 3, "{switched:?} {unswitched:?}");
        assert!(switched > Duration::from_micros(100) && switched < Duration::from_micros(200));
    }
}
