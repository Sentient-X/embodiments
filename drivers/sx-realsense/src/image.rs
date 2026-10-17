//! RGB8 and Y8 rasters as I420 in BT.601 limited range.
//!
//! The integer forms are the ones libyuv and the ITU tables use: luma
//! `16 + (66 R + 129 G + 25 B + 128) >> 8`, chroma from the mean of each 2x2 block. An infrared
//! intensity is luma scaled into 16..=235 with neutral chroma.

use sx_embodiment_drivers::I420Image;
use thiserror::Error;

/// A raster whose buffer is not the size its width, height and stride say.
#[derive(Debug, Error, PartialEq, Eq)]
#[error("a {width}x{height} {format} raster with stride {stride} does not fit {len} bytes")]
pub struct RasterError {
    pub format: &'static str,
    pub width: u32,
    pub height: u32,
    pub stride: usize,
    pub len: usize,
}

fn rows(
    format: &'static str,
    bytes_per_pixel: usize,
    width: u32,
    height: u32,
    stride: usize,
    data: &[u8],
) -> Result<(usize, usize), RasterError> {
    let refuse = || RasterError {
        format,
        width,
        height,
        stride,
        len: data.len(),
    };
    let (w, h) = (width as usize, height as usize);
    let row = w.checked_mul(bytes_per_pixel).ok_or_else(refuse)?;
    let needed = match h {
        0 => 0,
        _ => stride
            .checked_mul(h - 1)
            .and_then(|n| n.checked_add(row))
            .ok_or_else(refuse)?,
    };
    if w == 0 || h == 0 || stride < row || data.len() < needed {
        return Err(refuse());
    }
    Ok((w, h))
}

fn luma(r: i32, g: i32, b: i32) -> u8 {
    clamp_u8(((66 * r + 129 * g + 25 * b + 128) >> 8) + 16)
}

fn clamp_u8(value: i32) -> u8 {
    u8::try_from(value.clamp(0, 255)).unwrap_or(u8::MAX)
}

/// Convert packed RGB8 rows to I420.
///
/// # Errors
///
/// Returns [`RasterError`] when `data` cannot hold `height` rows of `stride` bytes.
pub fn rgb8_to_i420(
    width: u32,
    height: u32,
    stride: usize,
    data: &[u8],
) -> Result<I420Image, RasterError> {
    let (columns, lines) = rows("RGB8", 3, width, height, stride, data)?;
    let pixel = |x: usize, y: usize| {
        let at = y * stride + x * 3;
        (
            i32::from(data[at]),
            i32::from(data[at + 1]),
            i32::from(data[at + 2]),
        )
    };
    let mut image = I420Image::filled(width, height, (0, 0, 0));
    for y in 0..lines {
        for x in 0..columns {
            let (r, g, b) = pixel(x, y);
            image.y[y * columns + x] = luma(r, g, b);
        }
    }
    let chroma_width = columns.div_ceil(2);
    for cy in 0..lines.div_ceil(2) {
        for cx in 0..chroma_width {
            let (mut red, mut green, mut blue, mut count) = (0, 0, 0, 0);
            for y in (2 * cy)..(2 * cy + 2).min(lines) {
                for x in (2 * cx)..(2 * cx + 2).min(columns) {
                    let (pr, pg, pb) = pixel(x, y);
                    (red, green, blue, count) = (red + pr, green + pg, blue + pb, count + 1);
                }
            }
            let half = count / 2;
            let (r, g, b) = (
                (red + half) / count,
                (green + half) / count,
                (blue + half) / count,
            );
            image.u[cy * chroma_width + cx] =
                clamp_u8(((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128);
            image.v[cy * chroma_width + cx] =
                clamp_u8(((112 * r - 94 * g - 18 * b + 128) >> 8) + 128);
        }
    }
    Ok(image)
}

/// Convert Y8 intensity rows to I420 luma with neutral chroma.
///
/// # Errors
///
/// Returns [`RasterError`] when `data` cannot hold `height` rows of `stride` bytes.
pub fn y8_to_i420(
    width: u32,
    height: u32,
    stride: usize,
    data: &[u8],
) -> Result<I420Image, RasterError> {
    let (columns, lines) = rows("Y8", 1, width, height, stride, data)?;
    let mut image = I420Image::filled(width, height, (16, 128, 128));
    for y in 0..lines {
        for x in 0..columns {
            image.y[y * columns + x] =
                clamp_u8(16 + (i32::from(data[y * stride + x]) * 219 + 127) / 255);
        }
    }
    Ok(image)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn primaries_land_on_the_bt601_limited_range_values() {
        // White, black, red over a 3x1 raster padded to a stride of 10.
        let data = [255, 255, 255, 0, 0, 0, 255, 0, 0, 9];
        let image = rgb8_to_i420(3, 1, 10, &data).expect("raster");
        assert!(image.is_well_formed());
        assert_eq!(image.y, [235, 16, 82]);
        // The left block is white and black averaged to mid grey; the right is pure red.
        assert_eq!((image.u[0], image.v[0]), (128, 128));
        assert_eq!((image.u[1], image.v[1]), (90, 240));
    }

    #[test]
    fn infrared_intensity_spans_the_limited_luma_range() {
        let image = y8_to_i420(3, 1, 3, &[0, 128, 255]).expect("raster");
        assert_eq!(image.y, [16, 126, 235]);
        assert!(image.u.iter().chain(&image.v).all(|&c| c == 128));
    }

    #[test]
    fn a_buffer_shorter_than_its_raster_is_refused() {
        assert!(rgb8_to_i420(2, 2, 6, &[0; 11]).is_err());
        assert!(y8_to_i420(4, 1, 3, &[0; 4]).is_err());
        assert!(y8_to_i420(0, 1, 0, &[]).is_err());
    }
}
