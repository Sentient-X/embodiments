//! ArUco `DICT_4X4_50` markers sighted on an 8-bit luma plane, OpenCV's `ArucoDetector` with
//! its default parameters and `CORNER_REFINE_SUBPIX`, the detector `sx_embodiments.jaw_tags`
//! was measured with, restricted to a region and to the wanted IDs:
//!
//! 1. Adaptive mean thresholds (windows 3, 13 and 23, constant 7) over the region, each pixel's
//!    mean taken over the real image around it (replicated only at the image's own edges), so a
//!    region reads exactly as the whole frame does.
//! 2. The outer border of every dark component, approximated to a polygon (3% of its length);
//!    convex quadrilaterals of plausible size, far enough from the image edge, are candidates.
//! 3. Each candidate's 6x6 cells read by a nearest-pixel warp and Otsu's threshold; a border
//!    with more than five light cells is not a marker, and the 4x4 inner code must equal a
//!    wanted marker's code exactly in one of its four rotations (OpenCV's correction for this
//!    dictionary, one bit at 0.6, rounds to none).
//! 4. Corners refined by OpenCV's `cornerSubPix` (a window of 0.3 module, at most 5 pixels; 30
//!    iterations to 0.1 pixel), in OpenCV's corner order: the marker's top-left first,
//!    clockwise.

use crate::dictionary;

/// An 8-bit luma plane (the Y of the NV12 the rig encodes, or a decoded grey frame), or a crop
/// of one: `width` and `height` are the whole frame's, `crop` the part `data` holds (rows of
/// `stride` bytes from the crop's top-left). Geometry (the edge margins, the perimeter bounds)
/// is the frame's, so a crop around a region reads exactly as the frame would there.
#[derive(Clone, Copy, Debug)]
pub struct Luma<'a> {
    pub data: &'a [u8],
    pub width: usize,
    pub height: usize,
    pub stride: usize,
    pub crop: Region,
}

impl<'a> Luma<'a> {
    /// A whole frame.
    pub fn new(data: &'a [u8], width: usize, height: usize, stride: usize) -> Result<Self, String> {
        Self::cropped(data, stride, Region::whole(width, height), width, height)
    }

    /// The part `crop` of a `width` x `height` frame.
    pub fn cropped(
        data: &'a [u8],
        stride: usize,
        crop: Region,
        width: usize,
        height: usize,
    ) -> Result<Self, String> {
        let (w, h) = (
            crop.x1.saturating_sub(crop.x0),
            crop.y1.saturating_sub(crop.y0),
        );
        if w == 0
            || h == 0
            || crop.x1 > width
            || crop.y1 > height
            || stride < w
            || data.len() < stride * (h - 1) + w
        {
            return Err(format!(
                "a {w}x{h} luma crop of a {width}x{height} frame, stride {stride}, needs more than {} bytes",
                data.len()
            ));
        }
        Ok(Self {
            data,
            width,
            height,
            stride,
            crop,
        })
    }

    /// The pixel at frame coordinates inside the crop.
    #[inline]
    fn at(&self, x: usize, y: usize) -> u8 {
        self.data[(y - self.crop.y0) * self.stride + (x - self.crop.x0)]
    }

    /// The pixel at frame coordinates, `None` outside the crop.
    #[inline]
    fn get(&self, x: isize, y: isize) -> Option<u8> {
        let c = &self.crop;
        (x >= c.x0 as isize && y >= c.y0 as isize && x < c.x1 as isize && y < c.y1 as isize)
            .then(|| self.at(x as usize, y as usize))
    }

    /// The pixel at frame coordinates, replicated past the frame's edges (and the crop's).
    #[inline]
    fn clamped(&self, x: isize, y: isize) -> u8 {
        let c = &self.crop;
        let x = x.clamp(c.x0 as isize, c.x1 as isize - 1) as usize;
        let y = y.clamp(c.y0 as isize, c.y1 as isize - 1) as usize;
        self.at(x, y)
    }
}

/// A half-open pixel rectangle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Region {
    pub x0: usize,
    pub y0: usize,
    pub x1: usize,
    pub y1: usize,
}

impl Region {
    pub fn whole(width: usize, height: usize) -> Self {
        Self {
            x0: 0,
            y0: 0,
            x1: width,
            y1: height,
        }
    }

    pub fn area(&self) -> usize {
        (self.x1 - self.x0) * (self.y1 - self.y0)
    }

    /// The bounding box of `corners`, grown by `pad` pixels and clipped to the image.
    pub fn around(corners: &[[f64; 2]], pad: f64, width: usize, height: usize) -> Self {
        let (mut x0, mut y0, mut x1, mut y1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for c in corners {
            x0 = x0.min(c[0]);
            y0 = y0.min(c[1]);
            x1 = x1.max(c[0]);
            y1 = y1.max(c[1]);
        }
        let clip = |v: f64, max: usize| v.clamp(0.0, max as f64) as usize;
        Self {
            x0: clip((x0 - pad).floor(), width),
            y0: clip((y0 - pad).floor(), height),
            x1: clip((x1 + pad).ceil() + 1.0, width),
            y1: clip((y1 + pad).ceil() + 1.0, height),
        }
    }

    pub fn union(self, other: Self) -> Self {
        Self {
            x0: self.x0.min(other.x0),
            y0: self.y0.min(other.y0),
            x1: self.x1.max(other.x1),
            y1: self.y1.max(other.y1),
        }
    }

    pub fn overlaps(&self, other: &Self) -> bool {
        self.x0 < other.x1 && other.x0 < self.x1 && self.y0 < other.y1 && other.y0 < self.y1
    }
}

/// One marker: its ID and its four corners in full-image pixels, OpenCV's order.
#[derive(Clone, Debug, PartialEq)]
pub struct Marker {
    pub id: u32,
    pub corners: [[f64; 2]; 4],
}

/// OpenCV's `DetectorParameters` defaults that this detector reads.
const THRESHOLD_WINDOWS: [usize; 2] = [13, 23];
const THRESHOLD_CONSTANT: i32 = 7;
const MIN_PERIMETER_RATE: f64 = 0.03;
const MAX_PERIMETER_RATE: f64 = 4.0;
const POLYGON_ACCURACY_RATE: f64 = 0.03;
const MIN_CORNER_DISTANCE_RATE: f64 = 0.05;
const MIN_DISTANCE_TO_BORDER: f64 = 3.0;
const MIN_MARKER_DISTANCE_RATE: f64 = 0.125;
const CELL_PIXELS: usize = 4;
const MAX_BORDER_ERRORS_RATE: f64 = 0.35;
const MIN_OTSU_STD_DEV: f64 = 5.0;
const REFINE_WINDOW_MAX: usize = 5;
const REFINE_WINDOW_RELATIVE: f64 = 0.3;
const REFINE_ITERATIONS: usize = 30;
const REFINE_ACCURACY_PX: f64 = 0.1;
const PAD: usize = 11;

#[derive(Clone, Debug)]
struct Candidate {
    corners: [[f64; 2]; 4],
    contour_len: usize,
}

/// The detector with its scratch buffers, reused frame to frame without allocating once warm.
#[derive(Default)]
pub struct Detector {
    integral: Vec<u32>,
    binary: Vec<u8>,
    labels: Vec<u32>,
    parents: Vec<u32>,
    starts: Vec<(u32, u32, u32, u32, u32)>,
    slots: Vec<u32>,
    contour: Vec<[i32; 2]>,
}

impl Detector {
    pub fn new() -> Self {
        Self::default()
    }

    /// The wanted markers inside `region` of `image`, each ID at most once (a marker whose ID
    /// is sighted twice is dropped, as `jaw_tags.sight_tags` drops it).
    pub fn detect(&mut self, image: &Luma, region: Region, wanted: &[u32]) -> Vec<Marker> {
        let region = Region {
            x0: region.x0.max(image.crop.x0),
            y0: region.y0.max(image.crop.y0),
            x1: region.x1.min(image.crop.x1),
            y1: region.y1.min(image.crop.y1),
        };
        if region.x1 <= region.x0 + 2 || region.y1 <= region.y0 + 2 {
            return Vec::new();
        }
        let codes: Vec<(u32, u16)> = wanted
            .iter()
            .filter_map(|id| dictionary::code(*id).map(|code| (*id, code)))
            .collect();
        self.integral_of(image, region);
        let mut candidates = Vec::new();
        for window in THRESHOLD_WINDOWS {
            self.threshold(image, region, window);
            self.candidates(image, region, &mut candidates);
        }
        let groups = group(&candidates);
        let mut found: Vec<Marker> = Vec::new();
        let mut seen: Vec<u32> = Vec::new();
        for mut members in groups {
            members.sort_by(|a, b| candidates[*b].contour_len.cmp(&candidates[*a].contour_len));
            for index in members {
                if let Some(marker) = identify(image, &candidates[index], &codes) {
                    seen.push(marker.id);
                    found.push(marker);
                    break;
                }
            }
        }
        found.retain(|marker| seen.iter().filter(|id| **id == marker.id).count() == 1);
        for marker in &mut found {
            refine(image, &mut marker.corners);
        }
        found.sort_by_key(|marker| marker.id);
        found
    }

    /// The summed-area table of the region grown by [`PAD`], its pixels replicated past the
    /// image's own edges as OpenCV's box filter replicates them.
    #[inline(never)]
    fn integral_of(&mut self, image: &Luma, region: Region) {
        let w = region.x1 - region.x0 + 2 * PAD;
        let h = region.y1 - region.y0 + 2 * PAD;
        self.integral.clear();
        self.integral.resize((w + 1) * (h + 1), 0);
        let x_origin = region.x0 as isize - PAD as isize;
        let y_origin = region.y0 as isize - PAD as isize;
        for y in 0..h {
            let mut row = 0u32;
            let sy = y as isize + y_origin;
            for x in 0..w {
                let value = image.clamped(x as isize + x_origin, sy);
                row += u32::from(value);
                self.integral[(y + 1) * (w + 1) + x + 1] = self.integral[y * (w + 1) + x + 1] + row;
            }
        }
    }

    /// OpenCV's `adaptiveThreshold(MEAN_C, THRESH_BINARY_INV, window, 7)` over the region:
    /// 1 where the pixel is at least 7 below its window's rounded mean.
    #[inline(never)]
    fn threshold(&mut self, image: &Luma, region: Region, window: usize) {
        let w = region.x1 - region.x0;
        let h = region.y1 - region.y0;
        let stride = w + 2 * PAD + 1;
        let half = window / 2;
        let area = (window * window) as u32;
        self.binary.clear();
        self.binary.resize(w * h, 0);
        for y in 0..h {
            let top = (y + PAD - half) * stride;
            let bottom = (y + PAD + half + 1) * stride;
            for x in 0..w {
                let left = x + PAD - half;
                let right = x + PAD + half + 1;
                let sum = self.integral[bottom + right] + self.integral[top + left]
                    - self.integral[top + right]
                    - self.integral[bottom + left];
                // The rounded mean (ties to even, as OpenCV's box filter rounds into 8 bits).
                let (q, r) = (sum / area, sum % area);
                let mean = (q + u32::from(2 * r > area || (2 * r == area && q % 2 == 1))) as i32;
                let pixel = i32::from(image.at(region.x0 + x, region.y0 + y));
                self.binary[y * w + x] = u8::from(pixel - mean <= -THRESHOLD_CONSTANT);
            }
        }
    }

    /// The quadrilateral candidates among the dark components' outer borders.
    #[inline(never)]
    fn candidates(&mut self, image: &Luma, region: Region, out: &mut Vec<Candidate>) {
        let w = region.x1 - region.x0;
        let h = region.y1 - region.y0;
        let longest = image.width.max(image.height) as f64;
        let min_perimeter = MIN_PERIMETER_RATE * longest;
        let max_perimeter = MAX_PERIMETER_RATE * longest;
        self.label(w, h);
        let starts = std::mem::take(&mut self.starts);
        for &(sx, sy, bw, bh, _) in &starts {
            // An outer border is at least the box's perimeter in 8-connected steps.
            if (2 * (bw + bh)) as f64 > 2.0 * max_perimeter
                || (2.0 * f64::from(bw.max(bh))) < min_perimeter * 0.5
            {
                continue;
            }
            // A convex quadrilateral's border is no longer than its box's perimeter: a border
            // that runs on is not one, and is not followed further.
            let limit = (2 * (bw + bh) + 16) as usize;
            if !self.trace(
                w,
                h,
                sx as usize,
                sy as usize,
                limit.min(max_perimeter as usize + 1),
            ) {
                continue;
            }
            let length = self.contour.len();
            if (length as f64) < min_perimeter || (length as f64) > max_perimeter {
                continue;
            }
            let Some(quad) = approximate_quad(&self.contour, POLYGON_ACCURACY_RATE * length as f64)
            else {
                continue;
            };
            let mut corners = quad.map(|[x, y]| {
                [
                    f64::from(x) + region.x0 as f64,
                    f64::from(y) + region.y0 as f64,
                ]
            });
            let mut min_side = longest * longest;
            for j in 0..4 {
                let a = corners[j];
                let b = corners[(j + 1) % 4];
                min_side = min_side.min((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2));
            }
            let min_corner = length as f64 * MIN_CORNER_DISTANCE_RATE;
            if min_side < min_corner * min_corner {
                continue;
            }
            let (cols, rows) = (image.width as f64, image.height as f64);
            if corners.iter().any(|c| {
                c[0] < MIN_DISTANCE_TO_BORDER
                    || c[1] < MIN_DISTANCE_TO_BORDER
                    || c[0] > cols - 1.0 - MIN_DISTANCE_TO_BORDER
                    || c[1] > rows - 1.0 - MIN_DISTANCE_TO_BORDER
            }) {
                continue;
            }
            let d1 = [corners[1][0] - corners[0][0], corners[1][1] - corners[0][1]];
            let d2 = [corners[2][0] - corners[0][0], corners[2][1] - corners[0][1]];
            if d1[0] * d2[1] - d1[1] * d2[0] < 0.0 {
                corners.swap(1, 3);
            }
            out.push(Candidate {
                corners,
                contour_len: length,
            });
        }
        self.starts = starts;
    }

    /// 8-connected components of the binary region, each with its first pixel in raster order
    /// (the top of its outer border) and its bounding box.
    #[inline(never)]
    fn label(&mut self, w: usize, h: usize) {
        self.labels.clear();
        self.labels.resize(w * h, 0);
        self.parents.clear();
        self.parents.push(0);
        fn find(parents: &mut [u32], mut a: u32) -> u32 {
            while parents[a as usize] != a {
                parents[a as usize] = parents[parents[a as usize] as usize];
                a = parents[a as usize];
            }
            a
        }
        for y in 0..h {
            let row = y * w;
            for x in 0..w {
                if self.binary[row + x] == 0 {
                    continue;
                }
                let left = if x > 0 { self.labels[row + x - 1] } else { 0 };
                let (up_left, up, up_right) = if y > 0 {
                    let above = row - w;
                    (
                        if x > 0 { self.labels[above + x - 1] } else { 0 },
                        self.labels[above + x],
                        if x + 1 < w {
                            self.labels[above + x + 1]
                        } else {
                            0
                        },
                    )
                } else {
                    (0, 0, 0)
                };
                // Above, a run's labels are already one component; only the left neighbour and
                // the up-right one can join two.
                let mut label = if up != 0 {
                    up
                } else if up_left != 0 {
                    up_left
                } else {
                    0
                };
                for other in [left, up_right] {
                    if other == 0 || other == label {
                        continue;
                    }
                    if label == 0 {
                        label = other;
                        continue;
                    }
                    let (a, b) = (
                        find(&mut self.parents, label),
                        find(&mut self.parents, other),
                    );
                    if a != b {
                        let (low, high) = (a.min(b), a.max(b));
                        self.parents[high as usize] = low;
                    }
                }
                if label == 0 {
                    label = self.parents.len() as u32;
                    self.parents.push(label);
                }
                self.labels[row + x] = label;
            }
        }
        // Every label's parent is no larger than itself: one pass upward flattens them all.
        for l in 1..self.parents.len() {
            let parent = self.parents[l] as usize;
            self.parents[l] = self.parents[parent];
        }
        self.starts.clear();
        self.slots.clear();
        self.slots.resize(self.parents.len(), u32::MAX);
        for y in 0..h {
            let row = y * w;
            for x in 0..w {
                let label = self.labels[row + x];
                if label == 0 {
                    continue;
                }
                let root = self.parents[label as usize] as usize;
                let (x, y) = (x as u32, y as u32);
                if self.slots[root] == u32::MAX {
                    self.slots[root] = self.starts.len() as u32;
                    self.starts.push((x, y, 1, 1, x));
                } else {
                    let entry = &mut self.starts[self.slots[root] as usize];
                    // (start x, start y, width, height, min x)
                    let min_x = entry.4.min(x);
                    let max_x = (entry.4 + entry.2 - 1).max(x);
                    entry.4 = min_x;
                    entry.2 = max_x - min_x + 1;
                    entry.3 = y - entry.1 + 1;
                }
            }
        }
    }

    /// The outer border of the component whose first raster pixel is `(sx, sy)`, by Suzuki
    /// and Abe's border following (8-connected), as `findContours` lists it; `false` once it
    /// runs past `limit` points.
    #[inline(never)]
    fn trace(&mut self, w: usize, h: usize, sx: usize, sy: usize, limit: usize) -> bool {
        // Neighbour offsets, counter-clockwise on the screen starting east.
        const DIRS: [(isize, isize); 8] = [
            (1, 0),
            (1, -1),
            (0, -1),
            (-1, -1),
            (-1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        ];
        // Every foreground pixel 8-connected to the start is its component's.
        let on = |x: isize, y: isize, binary: &[u8]| {
            x >= 0
                && y >= 0
                && (x as usize) < w
                && (y as usize) < h
                && binary[y as usize * w + x as usize] != 0
        };
        self.contour.clear();
        let start = (sx as isize, sy as isize);
        // The pixel west of the start is outside the component: search clockwise from it.
        let mut first = None;
        for k in 0..8 {
            let d = (4 + 8 - k) % 8;
            let (dx, dy) = DIRS[d];
            if on(start.0 + dx, start.1 + dy, &self.binary) {
                first = Some(d);
                break;
            }
        }
        let Some(first_dir) = first else {
            self.contour.push([start.0 as i32, start.1 as i32]);
            return true;
        };
        let p1 = (start.0 + DIRS[first_dir].0, start.1 + DIRS[first_dir].1);
        let mut previous = p1;
        let mut current = start;
        loop {
            // Counter-clockwise from the direction after `previous`, around `current`.
            let back = DIRS
                .iter()
                .position(|d| (current.0 + d.0, current.1 + d.1) == previous)
                .unwrap_or(0);
            let mut next = current;
            for k in 1..=8 {
                let d = (back + k) % 8;
                let candidate = (current.0 + DIRS[d].0, current.1 + DIRS[d].1);
                if on(candidate.0, candidate.1, &self.binary) {
                    next = candidate;
                    break;
                }
            }
            self.contour.push([current.0 as i32, current.1 as i32]);
            if next == start && current == p1 {
                break;
            }
            if self.contour.len() > limit {
                return false;
            }
            previous = current;
            current = next;
        }
        true
    }
}

/// Douglas-Peucker over a closed contour (OpenCV's `approxPolyDP`): split at a pair of
/// far-apart points, simplify each half, and drop nearly collinear vertices; `None` unless the
/// polygon is a convex quadrilateral.
#[inline(never)]
fn approximate_quad(contour: &[[i32; 2]], epsilon: f64) -> Option<[[i32; 2]; 4]> {
    let n = contour.len();
    if n < 4 {
        return None;
    }
    let dist2 = |a: [i32; 2], b: [i32; 2]| {
        let dx = f64::from(a[0] - b[0]);
        let dy = f64::from(a[1] - b[1]);
        dx * dx + dy * dy
    };
    let mut start = 0;
    let mut end = 0;
    for _ in 0..3 {
        let origin = contour[start];
        let mut best = 0.0;
        for (j, point) in contour.iter().enumerate() {
            let d = dist2(origin, *point);
            if d > best {
                best = d;
                end = j;
            }
        }
        std::mem::swap(&mut start, &mut end);
    }
    let (a, b) = (start.min(end), start.max(end));
    if a == b {
        return None;
    }
    let mut vertices = vec![a];
    simplify(contour, a, b, epsilon, &mut vertices);
    vertices.push(b);
    simplify(contour, b, a + n, epsilon, &mut vertices);
    // Drop nearly collinear vertices (OpenCV's second pass).
    let mut points: Vec<[i32; 2]> = vertices.iter().map(|i| contour[i % n]).collect();
    let mut i = 0;
    while points.len() > 2 && i < points.len() {
        let len = points.len();
        let prev = points[(i + len - 1) % len];
        let point = points[i];
        let next = points[(i + 1) % len];
        let dx = f64::from(next[0] - prev[0]);
        let dy = f64::from(next[1] - prev[1]);
        let cross = (f64::from(point[0] - prev[0]) * dy - f64::from(point[1] - prev[1]) * dx).abs();
        let inner = f64::from(point[0] - prev[0]) * f64::from(next[0] - point[0])
            + f64::from(point[1] - prev[1]) * f64::from(next[1] - point[1]);
        if cross * cross <= 0.5 * epsilon * epsilon * (dx * dx + dy * dy)
            && dx != 0.0
            && dy != 0.0
            && inner >= 0.0
        {
            points.remove(i);
        } else {
            i += 1;
        }
    }
    if points.len() != 4 {
        return None;
    }
    let mut sign = 0.0f64;
    for i in 0..4 {
        let a = points[i];
        let b = points[(i + 1) % 4];
        let c = points[(i + 2) % 4];
        let cross = f64::from(b[0] - a[0]) * f64::from(c[1] - b[1])
            - f64::from(b[1] - a[1]) * f64::from(c[0] - b[0]);
        if cross == 0.0 || (sign != 0.0 && cross.signum() != sign) {
            return None;
        }
        sign = cross.signum();
    }
    Some([points[0], points[1], points[2], points[3]])
}

fn simplify(contour: &[[i32; 2]], from: usize, to: usize, epsilon: f64, out: &mut Vec<usize>) {
    let n = contour.len();
    if to <= from + 1 {
        return;
    }
    let a = contour[from % n];
    let b = contour[to % n];
    let dx = f64::from(b[0] - a[0]);
    let dy = f64::from(b[1] - a[1]);
    let length2 = dx * dx + dy * dy;
    let mut best = 0.0;
    let mut at = from;
    for k in from + 1..to {
        let p = contour[k % n];
        let px = f64::from(p[0] - a[0]);
        let py = f64::from(p[1] - a[1]);
        let d = if length2 > 0.0 {
            let cross = px * dy - py * dx;
            cross * cross / length2
        } else {
            px * px + py * py
        };
        if d > best {
            best = d;
            at = k;
        }
    }
    if best > epsilon * epsilon {
        simplify(contour, from, at, epsilon, out);
        out.push(at);
        simplify(contour, at, to, epsilon, out);
    }
}

/// Candidates closer than an eighth of the smaller perimeter (mean corner distance, best
/// alignment) are one marker seen at several thresholds.
#[inline(never)]
fn group(candidates: &[Candidate]) -> Vec<Vec<usize>> {
    let mut owner: Vec<usize> = (0..candidates.len()).collect();
    fn root(owner: &mut [usize], mut a: usize) -> usize {
        while owner[a] != a {
            owner[a] = owner[owner[a]];
            a = owner[a];
        }
        a
    }
    for i in 0..candidates.len() {
        for j in i + 1..candidates.len() {
            let perimeter = candidates[i].contour_len.min(candidates[j].contour_len) as f64;
            let limit = perimeter * MIN_MARKER_DISTANCE_RATE;
            let near = (0..4).any(|first| {
                let mut sum = 0.0;
                for c in 0..4 {
                    let a = candidates[i].corners[(c + first) % 4];
                    let b = candidates[j].corners[c];
                    sum += (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2);
                }
                sum / 4.0 < limit * limit
            });
            if near {
                let (ri, rj) = (root(&mut owner, i), root(&mut owner, j));
                owner[ri.max(rj)] = ri.min(rj);
            }
        }
    }
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut slot = vec![usize::MAX; candidates.len()];
    for i in 0..candidates.len() {
        let r = root(&mut owner, i);
        if slot[r] == usize::MAX {
            slot[r] = groups.len();
            groups.push(Vec::new());
        }
        groups[slot[r]].push(i);
    }
    groups
}

/// The homography taking the unit square's corners `(0,0) (1,0) (1,1) (0,1)` onto `quad`.
pub(crate) fn square_homography(quad: &[[f64; 2]; 4]) -> Option<[[f64; 3]; 3]> {
    let [p0, p1, p2, p3] = *quad;
    let dx1 = p1[0] - p2[0];
    let dx2 = p3[0] - p2[0];
    let dx3 = p0[0] - p1[0] + p2[0] - p3[0];
    let dy1 = p1[1] - p2[1];
    let dy2 = p3[1] - p2[1];
    let dy3 = p0[1] - p1[1] + p2[1] - p3[1];
    let det = dx1 * dy2 - dx2 * dy1;
    if det.abs() < 1e-12 {
        return None;
    }
    let g = (dx3 * dy2 - dx2 * dy3) / det;
    let h = (dx1 * dy3 - dx3 * dy1) / det;
    Some([
        [p1[0] - p0[0] + g * p1[0], p3[0] - p0[0] + h * p3[0], p0[0]],
        [p1[1] - p0[1] + g * p1[1], p3[1] - p0[1] + h * p3[1], p0[1]],
        [g, h, 1.0],
    ])
}

/// The candidate's code read and matched, its corners rotated so the marker's top-left is
/// first; `None` when it is no wanted marker.
#[inline(never)]
fn identify(image: &Luma, candidate: &Candidate, codes: &[(u32, u16)]) -> Option<Marker> {
    let cells = dictionary::MARKER_SIZE + 2;
    let side = cells * CELL_PIXELS;
    let homography = square_homography(&candidate.corners)?;
    let last = (side - 1) as f64;
    let mut warped = [0u8; 24 * 24];
    for y in 0..side {
        for x in 0..side {
            let u = x as f64 / last;
            let v = y as f64 / last;
            let w = homography[2][0] * u + homography[2][1] * v + 1.0;
            let w = if w != 0.0 { 1.0 / w } else { 0.0 };
            let sx = ((homography[0][0] * u + homography[0][1] * v + homography[0][2]) * w)
                .round_ties_even();
            let sy = ((homography[1][0] * u + homography[1][1] * v + homography[1][2]) * w)
                .round_ties_even();
            warped[y * side + x] = image.get(sx as isize, sy as isize).unwrap_or(0);
        }
    }
    // Is there contrast enough for Otsu inside the border's middle?
    let inner = CELL_PIXELS / 2..side - CELL_PIXELS / 2;
    let (mut sum, mut sum2, mut count) = (0.0, 0.0, 0.0);
    for y in inner.clone() {
        for x in inner.clone() {
            let value = f64::from(warped[y * side + x]);
            sum += value;
            sum2 += value * value;
            count += 1.0;
        }
    }
    let mean = sum / count;
    let deviation = (sum2 / count - mean * mean).max(0.0).sqrt();
    let mut bits = [[false; 6]; 6];
    if deviation < MIN_OTSU_STD_DEV {
        let white = mean > 127.0;
        bits = [[white; 6]; 6];
    } else {
        let threshold = otsu(&warped[..side * side]);
        for (cy, row) in bits.iter_mut().enumerate() {
            for (cx, bit) in row.iter_mut().enumerate() {
                let mut white = 0;
                for y in cy * CELL_PIXELS..(cy + 1) * CELL_PIXELS {
                    for x in cx * CELL_PIXELS..(cx + 1) * CELL_PIXELS {
                        white += usize::from(warped[y * side + x] > threshold);
                    }
                }
                *bit = white > CELL_PIXELS * CELL_PIXELS / 2;
            }
        }
    }
    let mut border_errors = 0;
    for (y, row) in bits.iter().enumerate() {
        for (x, bit) in row.iter().enumerate() {
            if (y == 0 || y == cells - 1 || x == 0 || x == cells - 1) && *bit {
                border_errors += 1;
            }
        }
    }
    let marker_cells = dictionary::MARKER_SIZE * dictionary::MARKER_SIZE;
    if border_errors > (marker_cells as f64 * MAX_BORDER_ERRORS_RATE) as usize {
        return None;
    }
    let n = dictionary::MARKER_SIZE;
    let sampled = |r: usize, c: usize| bits[r + 1][c + 1];
    for k in 0..4 {
        // The marker's grid when its top-left sits at the candidate's corner `k`.
        let mut code = 0u16;
        for r in 0..n {
            for c in 0..n {
                let bit = match k {
                    0 => sampled(r, c),
                    1 => sampled(c, n - 1 - r),
                    2 => sampled(n - 1 - r, n - 1 - c),
                    _ => sampled(n - 1 - c, r),
                };
                code = (code << 1) | u16::from(bit);
            }
        }
        if let Some((id, _)) = codes.iter().find(|(_, wanted)| *wanted == code) {
            let c = candidate.corners;
            return Some(Marker {
                id: *id,
                corners: [c[k], c[(k + 1) % 4], c[(k + 2) % 4], c[(k + 3) % 4]],
            });
        }
    }
    None
}

/// OpenCV's Otsu threshold over 8-bit pixels.
fn otsu(pixels: &[u8]) -> u8 {
    let mut histogram = [0usize; 256];
    for p in pixels {
        histogram[usize::from(*p)] += 1;
    }
    let total = pixels.len() as f64;
    let mu: f64 = histogram
        .iter()
        .enumerate()
        .map(|(i, h)| i as f64 * *h as f64)
        .sum::<f64>()
        / total;
    let (mut q1, mut mu1, mut best_sigma, mut best) = (0.0f64, 0.0f64, 0.0f64, 0u8);
    for (i, h) in histogram.iter().enumerate() {
        let p = *h as f64 / total;
        mu1 *= q1;
        q1 += p;
        let q2 = 1.0 - q1;
        if q1.min(q2) < f64::from(f32::EPSILON) || q1.max(q2) > 1.0 - f64::from(f32::EPSILON) {
            continue;
        }
        mu1 = (mu1 + i as f64 * p) / q1;
        let mu2 = (mu - q1 * mu1) / q2;
        let sigma = q1 * q2 * (mu1 - mu2) * (mu1 - mu2);
        if sigma > best_sigma {
            best_sigma = sigma;
            best = i as u8;
        }
    }
    best
}

/// OpenCV's `cornerSubPix` with the ArUco detector's window for this marker's module size.
#[inline(never)]
fn refine(image: &Luma, corners: &mut [[f64; 2]; 4]) {
    let mut perimeter = 0.0;
    for i in 0..4 {
        let a = corners[i];
        let b = corners[(i + 1) % 4];
        perimeter += ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    }
    let module = perimeter / 4.0 / (dictionary::MARKER_SIZE + 2) as f64;
    let window =
        ((REFINE_WINDOW_RELATIVE * module).round_ties_even() as usize).clamp(1, REFINE_WINDOW_MAX);
    corner_sub_pix(image, corners, window);
}

pub(crate) fn corner_sub_pix(image: &Luma, corners: &mut [[f64; 2]], half: usize) {
    let win = 2 * half + 1;
    let mut mask = vec![0.0f64; win * win];
    for i in 0..win {
        let y = (i as f64 - half as f64) / half as f64;
        let vy = (-y * y).exp();
        for j in 0..win {
            let x = (j as f64 - half as f64) / half as f64;
            mask[i * win + j] = f64::from((vy * (-x * x).exp()) as f32);
        }
    }
    let patch_side = win + 2;
    let mut patch = vec![0.0f32; patch_side * patch_side];
    let eps2 = REFINE_ACCURACY_PX * REFINE_ACCURACY_PX;
    for corner in corners.iter_mut() {
        let initial = *corner;
        let mut ci = initial;
        for _ in 0..REFINE_ITERATIONS {
            rect_sub_pix(image, ci, patch_side, &mut patch);
            let (mut a, mut b, mut c, mut bb1, mut bb2) = (0.0, 0.0, 0.0, 0.0, 0.0);
            for i in 0..win {
                let py = i as f64 - half as f64;
                for j in 0..win {
                    let m = mask[i * win + j];
                    let px = j as f64 - half as f64;
                    let tgx = f64::from(
                        patch[(i + 1) * patch_side + j + 2] - patch[(i + 1) * patch_side + j],
                    );
                    let tgy = f64::from(
                        patch[(i + 2) * patch_side + j + 1] - patch[i * patch_side + j + 1],
                    );
                    let gxx = tgx * tgx * m;
                    let gxy = tgx * tgy * m;
                    let gyy = tgy * tgy * m;
                    a += gxx;
                    b += gxy;
                    c += gyy;
                    bb1 += gxx * px + gxy * py;
                    bb2 += gxy * px + gyy * py;
                }
            }
            let det = a * c - b * b;
            if det.abs() <= f64::EPSILON * f64::EPSILON {
                break;
            }
            let s = 1.0 / det;
            let next = [
                ci[0] + c * s * bb1 - b * s * bb2,
                ci[1] - b * s * bb1 + a * s * bb2,
            ];
            let err = (next[0] - ci[0]).powi(2) + (next[1] - ci[1]).powi(2);
            ci = next;
            if ci[0] < 0.0
                || ci[0] >= image.width as f64
                || ci[1] < 0.0
                || ci[1] >= image.height as f64
            {
                break;
            }
            if err <= eps2 {
                break;
            }
        }
        if (ci[0] - initial[0]).abs() > half as f64 || (ci[1] - initial[1]).abs() > half as f64 {
            ci = initial;
        }
        *corner = ci;
    }
}

/// OpenCV's `getRectSubPix` (float output): a `side x side` patch centred on `centre`,
/// bilinear, replicated past the image's edges.
fn rect_sub_pix(image: &Luma, centre: [f64; 2], side: usize, out: &mut [f32]) {
    let half = (side as f64 - 1.0) * 0.5;
    let x0 = centre[0] - half;
    let y0 = centre[1] - half;
    let ix = x0.floor();
    let iy = y0.floor();
    let fx = (x0 - ix) as f32;
    let fy = (y0 - iy) as f32;
    let (ix, iy) = (ix as isize, iy as isize);
    for y in 0..side {
        for x in 0..side {
            let px = ix + x as isize;
            let py = iy + y as isize;
            let p00 = f32::from(image.clamped(px, py));
            let p01 = f32::from(image.clamped(px + 1, py));
            let p10 = f32::from(image.clamped(px, py + 1));
            let p11 = f32::from(image.clamped(px + 1, py + 1));
            out[y * side + x] = p00 * (1.0 - fx) * (1.0 - fy)
                + p01 * fx * (1.0 - fy)
                + p10 * (1.0 - fx) * fy
                + p11 * fx * fy;
        }
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// A marker drawn axis-aligned: `cell` pixels per cell, its top-left at `(x0, y0)`, on a
    /// white page.
    pub(crate) fn drawn(
        id: u32,
        width: usize,
        height: usize,
        x0: usize,
        y0: usize,
        cell: usize,
    ) -> Vec<u8> {
        let mut image = vec![230u8; width * height];
        let code = dictionary::code(id).unwrap();
        for cy in 0..6 {
            for cx in 0..6 {
                let white = cy > 0
                    && cy < 5
                    && cx > 0
                    && cx < 5
                    && (code >> (15 - ((cy - 1) * 4 + (cx - 1)))) & 1 == 1;
                for y in 0..cell {
                    for x in 0..cell {
                        image[(y0 + cy * cell + y) * width + x0 + cx * cell + x] =
                            if white { 230 } else { 20 };
                    }
                }
            }
        }
        image
    }

    #[test]
    fn a_drawn_marker_is_found_with_its_corners_in_opencvs_order() {
        let (w, h) = (160, 120);
        let image = drawn(1, w, h, 40, 30, 8);
        let luma = Luma::new(&image, w, h, w).unwrap();
        let found = Detector::new().detect(&luma, Region::whole(w, h), &[0, 1]);
        assert_eq!(found.len(), 1, "{found:?}");
        assert_eq!(found[0].id, 1);
        let expected = [[39.5, 29.5], [87.5, 29.5], [87.5, 77.5], [39.5, 77.5]];
        for (got, want) in found[0].corners.iter().zip(expected) {
            assert!(
                (got[0] - want[0]).abs() < 0.8 && (got[1] - want[1]).abs() < 0.8,
                "{:?}",
                found[0].corners
            );
        }
        // Another marker's ID is not wanted, and a region away from it finds nothing.
        assert!(
            Detector::new()
                .detect(&luma, Region::whole(w, h), &[2])
                .is_empty()
        );
        let away = Region {
            x0: 100,
            y0: 0,
            x1: 160,
            y1: 120,
        };
        assert!(Detector::new().detect(&luma, away, &[1]).is_empty());
    }

    #[test]
    fn a_rotated_marker_names_its_own_top_left_first() {
        let (w, h) = (160, 120);
        let upright = drawn(0, w, h, 40, 30, 8);
        // Rotate the picture a quarter turn about the marker's centre (64, 54): the marker's
        // top-left moves to the picture's top-right of the square.
        let mut turned = vec![230u8; w * h];
        for y in 0..h {
            for x in 0..w {
                let (dx, dy) = (x as isize - 64, y as isize - 54);
                let (sx, sy) = (64 + dy, 54 - dx - 1);
                if sx >= 0 && sy >= 0 && (sx as usize) < w && (sy as usize) < h {
                    turned[y * w + x] = upright[sy as usize * w + sx as usize];
                }
            }
        }
        let luma = Luma::new(&turned, w, h, w).unwrap();
        let found = Detector::new().detect(&luma, Region::whole(w, h), &[0]);
        assert_eq!(found.len(), 1);
        let first = found[0].corners[0];
        assert!(first[0] > 80.0 && first[1] < 40.0, "{:?}", found[0].corners);
    }
}
