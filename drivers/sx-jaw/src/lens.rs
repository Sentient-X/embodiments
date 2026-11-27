//! A camera's intrinsics as the tag solver uses them: OpenCV's projection and its point
//! undistortion (`sx_embodiments.calibrate.lens.Lens`), for the equidistant fisheye the wrist
//! cameras are calibrated with, Brown-Conrady, and none.

use crate::geometry::Vec3;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Distortion {
    None,
    /// OpenCV's four-coefficient fisheye (`equidistant`, `kannala_brandt`).
    Fisheye,
    /// `plumb_bob` (`k1 k2 p1 p2 k3`) and `rational_polynomial` (`... k4 k5 k6`).
    BrownConrady,
}

#[derive(Clone, Debug, PartialEq)]
pub struct Lens {
    pub width: u32,
    pub height: u32,
    /// Row-major 3x3.
    pub k: [f64; 9],
    pub distortion: Distortion,
    /// Eight slots; unused ones are zero.
    pub coefficients: [f64; 8],
}

impl Lens {
    /// A lens from its wire distortion model name and coefficients.
    pub fn new(
        width: u32,
        height: u32,
        k: [f64; 9],
        model: &str,
        coefficients: &[f64],
    ) -> Result<Self, String> {
        let (distortion, count) = match model {
            "none" => (Distortion::None, 0),
            "equidistant" | "kannala_brandt" => (Distortion::Fisheye, 4),
            "plumb_bob" => (Distortion::BrownConrady, 5),
            "rational_polynomial" => (Distortion::BrownConrady, 8),
            other => return Err(format!("unknown distortion model {other:?}")),
        };
        if coefficients.len() != count {
            return Err(format!(
                "a {model} lens has {count} coefficients, not {}",
                coefficients.len()
            ));
        }
        if !k.iter().chain(coefficients).all(|v| v.is_finite()) || k[0] <= 0.0 || k[4] <= 0.0 {
            return Err("a lens needs finite intrinsics and positive focal lengths".to_owned());
        }
        let mut slots = [0.0; 8];
        slots[..count].copy_from_slice(coefficients);
        Ok(Self {
            width,
            height,
            k,
            distortion,
            coefficients: slots,
        })
    }

    /// A camera-frame point to pixels; `None` behind the camera.
    pub fn project(&self, point: Vec3) -> Option<[f64; 2]> {
        let z = point[2];
        if z <= 1e-9 {
            return None;
        }
        let (x, y) = (point[0] / z, point[1] / z);
        let d = &self.coefficients;
        let (u, v) = match self.distortion {
            Distortion::Fisheye => {
                let radius = (x * x + y * y).sqrt();
                let theta = radius.atan();
                let t2 = theta * theta;
                let distorted = theta
                    * (1.0 + d[0] * t2 + d[1] * t2.powi(2) + d[2] * t2.powi(3) + d[3] * t2.powi(4));
                let s = if radius > 1e-12 {
                    distorted / radius.max(1e-12)
                } else {
                    1.0
                };
                (x * s, y * s)
            }
            Distortion::None => (x, y),
            Distortion::BrownConrady => {
                let r2 = x * x + y * y;
                let mut radial = 1.0 + d[0] * r2 + d[1] * r2.powi(2) + d[4] * r2.powi(3);
                radial /= 1.0 + d[5] * r2 + d[6] * r2.powi(2) + d[7] * r2.powi(3);
                (
                    x * radial + 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x),
                    y * radial + d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y,
                )
            }
        };
        let k = &self.k;
        Some([k[0] * u + k[1] * v + k[2], k[4] * v + k[5]])
    }

    /// Pixels to ideal (undistorted, unit-focal) image coordinates, as OpenCV's
    /// `fisheye.undistortPoints` / `undistortPoints` with their default criteria; a fisheye
    /// point whose Newton iteration does not converge is `None` (OpenCV writes -1e6).
    pub fn normalized(&self, pixel: [f64; 2]) -> Option<[f64; 2]> {
        let k = &self.k;
        let d = &self.coefficients;
        match self.distortion {
            Distortion::None => {
                let y = (pixel[1] - k[5]) / k[4];
                let x = (pixel[0] - k[2] - k[1] * y) / k[0];
                Some([x, y])
            }
            Distortion::Fisheye => {
                let pw = [(pixel[0] - k[2]) / k[0], (pixel[1] - k[5]) / k[4]];
                let theta_d = (pw[0] * pw[0] + pw[1] * pw[1])
                    .sqrt()
                    .clamp(-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2);
                const EPSILON: f64 = 1e-8;
                let mut theta = theta_d;
                let mut converged = false;
                let mut s = 0.0;
                if theta_d.abs() > EPSILON {
                    for _ in 0..10 {
                        let t2 = theta * theta;
                        let (t4, t6, t8) = (t2 * t2, t2 * t2 * t2, t2 * t2 * t2 * t2);
                        let fix = (theta * (1.0 + d[0] * t2 + d[1] * t4 + d[2] * t6 + d[3] * t8)
                            - theta_d)
                            / (1.0
                                + 3.0 * d[0] * t2
                                + 5.0 * d[1] * t4
                                + 7.0 * d[2] * t6
                                + 9.0 * d[3] * t8);
                        theta -= fix;
                        if fix.abs() < EPSILON {
                            converged = true;
                            break;
                        }
                    }
                    s = theta.tan() / theta_d;
                } else {
                    converged = true;
                }
                let flipped = (theta_d < 0.0 && theta > 0.0) || (theta_d > 0.0 && theta < 0.0);
                (converged && !flipped).then_some([pw[0] * s, pw[1] * s])
            }
            Distortion::BrownConrady => {
                let y0 = (pixel[1] - k[5]) / k[4];
                let x0 = (pixel[0] - k[2]) / k[0];
                let (mut x, mut y) = (x0, y0);
                for _ in 0..5 {
                    let r2 = x * x + y * y;
                    let icdist = (1.0 + ((d[7] * r2 + d[6]) * r2 + d[5]) * r2)
                        / (1.0 + ((d[4] * r2 + d[1]) * r2 + d[0]) * r2);
                    if icdist < 0.0 {
                        x = x0;
                        y = y0;
                        break;
                    }
                    let dx = 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x);
                    let dy = d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y;
                    x = (x0 - dx) * icdist;
                    y = (y0 - dy) * icdist;
                }
                Some([x, y])
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wrist() -> Lens {
        Lens::new(
            640,
            480,
            [271.15, 0.0, 310.57, 0.0, 270.43, 221.63, 0.0, 0.0, 1.0],
            "equidistant",
            &[-0.0413, -0.00224, -0.00366, 0.00155],
        )
        .unwrap()
    }

    #[test]
    fn a_fisheye_pixel_undistorts_back_onto_its_ray() {
        let lens = wrist();
        for point in [[0.01, 0.02, 0.05], [-0.03, 0.025, 0.04], [0.0, 0.0, 1.0]] {
            let pixel = lens.project(point).unwrap();
            let ideal = lens.normalized(pixel).unwrap();
            assert!((ideal[0] - point[0] / point[2]).abs() < 1e-7, "{ideal:?}");
            assert!((ideal[1] - point[1] / point[2]).abs() < 1e-7, "{ideal:?}");
        }
    }
}
