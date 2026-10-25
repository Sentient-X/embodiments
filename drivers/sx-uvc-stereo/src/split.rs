//! One side-by-side YUYV frame as its left and right eyes in I420.
//!
//! YUYV packs two pixels in four bytes, `Y0 U Y1 V`, sharing one chroma sample. An eye starts on
//! a macropixel boundary, so each eye's luma is copied and its chroma is the rounded mean of the
//! row pair it covers in I420 (the last row alone when the height is odd).

use sx_embodiment_drivers::I420Image;
use thiserror::Error;

/// A frame that is not two eyes of the planned raster.
#[derive(Debug, Error, PartialEq, Eq)]
pub enum SplitError {
    #[error("an eye {eye_width} pixels wide does not start on a YUYV macropixel")]
    OddEye { eye_width: u32 },
    #[error(
        "a {width}x{height} YUYV frame with stride {stride} does not fit {len} bytes, or is not \
         two {eye_width}-pixel eyes"
    )]
    Raster {
        width: u32,
        height: u32,
        eye_width: u32,
        stride: usize,
        len: usize,
    },
}

/// Split one `2 * eye_width` x `height` YUYV frame, `stride` bytes a row, into its left and
/// right eyes.
///
/// # Errors
///
/// Returns [`SplitError`] for an odd eye width, or a buffer that cannot hold `height` rows of
/// `stride` bytes of two such eyes.
pub fn split(
    eye_width: u32,
    height: u32,
    stride: usize,
    data: &[u8],
) -> Result<[I420Image; 2], SplitError> {
    if !eye_width.is_multiple_of(2) {
        return Err(SplitError::OddEye { eye_width });
    }
    let width = eye_width.saturating_mul(2);
    let refuse = || SplitError::Raster {
        width,
        height,
        eye_width,
        stride,
        len: data.len(),
    };
    let (eye, lines) = (eye_width as usize, height as usize);
    // Both eyes' bytes in a row; each eye is half of it.
    let row = eye.checked_mul(4).ok_or_else(refuse)?;
    let needed = lines
        .checked_sub(1)
        .and_then(|n| stride.checked_mul(n))
        .and_then(|n| n.checked_add(row))
        .ok_or_else(refuse)?;
    if eye == 0 || stride < row || data.len() < needed {
        return Err(refuse());
    }
    let half = eye / 2;
    let eye_at = |offset: usize| {
        let mut image = I420Image::filled(eye_width, height, (0, 0, 0));
        let byte = |x: usize, y: usize| data[y * stride + offset + x];
        for y in 0..lines {
            for x in 0..eye {
                // Pixel x of the eye is byte 2x of its row: Y0 at 4m, Y1 at 4m + 2.
                image.y[y * eye + x] = byte(2 * x, y);
            }
        }
        for cy in 0..lines.div_ceil(2) {
            let (top, bottom) = (2 * cy, (2 * cy + 1).min(lines - 1));
            for cx in 0..half {
                let mean = |at: usize| {
                    let sum = u16::from(byte(at, top)) + u16::from(byte(at, bottom));
                    u8::try_from(sum.div_ceil(2)).unwrap_or(u8::MAX)
                };
                image.u[cy * half + cx] = mean(4 * cx + 1);
                image.v[cy * half + cx] = mean(4 * cx + 3);
            }
        }
        image
    };
    Ok([eye_at(0), eye_at(row / 2)])
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 2x2-pixel eye pair (4x2 frame) padded to a stride of 10 bytes a row.
    fn frame() -> Vec<u8> {
        let mut data = Vec::new();
        // Row 0: left macropixel Y 10 20 U 100 V 200, right Y 30 40 U 110 V 210, pad.
        data.extend([10, 100, 20, 200, 30, 110, 40, 210, 0, 0]);
        // Row 1: left Y 11 21 U 103 V 203, right Y 31 41 U 111 V 211, pad.
        data.extend([11, 103, 21, 203, 31, 111, 41, 211, 0, 0]);
        data
    }

    #[test]
    fn each_eye_keeps_its_luma_and_averages_its_chroma_over_the_row_pair() {
        let [left, right] = split(2, 2, 10, &frame()).expect("split");
        assert!(left.is_well_formed() && right.is_well_formed());
        assert_eq!(left.y, [10, 20, 11, 21]);
        assert_eq!(right.y, [30, 40, 31, 41]);
        assert_eq!((left.u[0], left.v[0]), (102, 202));
        assert_eq!((right.u[0], right.v[0]), (111, 211));
    }

    #[test]
    fn an_odd_last_row_is_its_own_chroma() {
        let data = &frame()[..10];
        let [left, _] = split(2, 1, 10, data).expect("split");
        assert_eq!((left.u, left.v), (vec![100], vec![200]));
    }

    #[test]
    fn a_frame_that_is_not_two_eyes_is_refused() {
        assert!(matches!(
            split(3, 2, 12, &[0; 24]),
            Err(SplitError::OddEye { .. })
        ));
        assert!(split(2, 2, 10, &frame()[..17]).is_err());
        assert!(split(2, 2, 6, &[0; 20]).is_err());
        assert!(split(0, 2, 0, &[]).is_err());
        assert!(split(2, 0, 8, &[]).is_err());
    }
}
