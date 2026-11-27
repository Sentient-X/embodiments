//! Where and how often the headset looks for the finger markers while it records, so the
//! sighting costs a bounded, small share of a core:
//!
//! - one frame in [`Cadence::every`] is analysed (the rest are not looked at);
//! - a marker seen recently is looked for only in a window around where it was (its box grown
//!   by its own size), so a frame costs two small regions, not the whole picture;
//! - the whole picture is searched only for a marker not seen recently, and then at most once
//!   in [`Cadence::search_every`] analysed frames;
//! - a take whose first [`Cadence::probe_frames`] frames show no marker stops looking at all
//!   (a gripper without tags costs nothing after its first seconds).

use crate::detect::{Detector, Luma, Marker, Region};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cadence {
    /// Analyse one frame in this many.
    pub every: u32,
    /// Search the whole picture at most once in this many analysed frames.
    pub search_every: u32,
    /// A marker is looked for around where it was while seen within this many analysed
    /// frames.
    pub tracked_for: u32,
    /// Stop when none of the first this many frames (recorded, not analysed) showed a marker.
    pub probe_frames: u32,
}

impl Default for Cadence {
    fn default() -> Self {
        Self {
            every: 1,
            search_every: 4,
            tracked_for: 15,
            probe_frames: 300,
        }
    }
}

/// What the tracker did with one recorded frame.
#[derive(Clone, Debug, PartialEq)]
pub enum Look {
    /// Not analysed (the cadence, or the take showed no marker in its opening).
    Skipped,
    /// Analysed: the markers found, and the pixels examined.
    Analysed { markers: Vec<Marker>, pixels: usize },
}

pub struct Tracker {
    ids: [u32; 2],
    cadence: Cadence,
    detector: Detector,
    last: [Option<[[f64; 2]; 4]>; 2],
    unseen_for: [u32; 2],
    frames: u32,
    since_search: u32,
    ever_seen: bool,
    /// Whether the whole picture may be searched (off while the headset runs warm: only the
    /// windows around markers already found are looked at).
    pub searching: bool,
}

/// Pixels kept around a planned region in the crop the caller copies: the threshold's widest
/// window and the corner refinement read this far past it.
pub const CROP_MARGIN: usize = 16;

impl Tracker {
    pub fn new(ids: [u32; 2], cadence: Cadence) -> Self {
        Self {
            ids,
            cadence,
            detector: Detector::new(),
            last: [None, None],
            unseen_for: [u32::MAX; 2],
            frames: 0,
            since_search: u32::MAX,
            ever_seen: false,
            searching: true,
        }
    }

    pub fn ids(&self) -> [u32; 2] {
        self.ids
    }

    /// Whether the take showed no marker in its opening and the tracker stopped looking.
    pub fn gave_up(&self) -> bool {
        !self.ever_seen && self.frames > self.cadence.probe_frames
    }

    /// Count the next recorded frame and plan it: the regions of the `width` x `height`
    /// picture to look in, or `None` when it is not analysed (the cadence, a take without
    /// markers, nothing to look for while searching is off). A caller that cannot analyse it
    /// after all (its worker is busy) just drops the plan.
    pub fn plan(&mut self, width: usize, height: usize) -> Option<Vec<Region>> {
        let index = self.frames;
        self.frames = self.frames.saturating_add(1);
        if self.gave_up() || !index.is_multiple_of(self.cadence.every.max(1)) {
            return None;
        }
        // A marker seen in the last analysed frame is looked for around where it was; one
        // lost for a few frames in a wider window around it; one long lost or never seen
        // only by the rate-limited whole-picture search.
        let mut windows: Vec<Region> = Vec::new();
        let mut lost = 0;
        for k in 0..2 {
            let Some(corners) = self.last[k] else {
                lost += 1;
                continue;
            };
            let grow = match self.unseen_for[k] {
                0 => 1.0,
                n if n < self.cadence.tracked_for => 2.5,
                _ => {
                    lost += 1;
                    continue;
                }
            };
            let side = side_px(&corners);
            windows.push(Region::around(
                &corners,
                (grow * side).max(16.0),
                width,
                height,
            ));
        }
        self.since_search = self.since_search.saturating_add(1);
        if self.searching && lost > 0 && self.since_search >= self.cadence.search_every {
            self.since_search = 0;
            return Some(vec![Region::whole(width, height)]);
        }
        let mut regions: Vec<Region> = Vec::new();
        for region in windows {
            match regions.iter_mut().find(|r| r.overlaps(&region)) {
                Some(existing) => *existing = existing.union(region),
                None => regions.push(region),
            }
        }
        (!regions.is_empty()).then_some(regions)
    }

    /// Look in the planned regions, each read from its crop (`crops[i]` holds `regions[i]`
    /// grown by [`CROP_MARGIN`], clipped to the picture), and remember what was found.
    pub fn look(&mut self, crops: &[(Luma, Region)]) -> Vec<Marker> {
        let mut markers: Vec<Marker> = Vec::new();
        for (image, region) in crops {
            for marker in self.detector.detect(image, *region, &self.ids) {
                if !markers.iter().any(|m| m.id == marker.id) {
                    markers.push(marker);
                }
            }
        }
        for k in 0..2 {
            match markers.iter().find(|m| m.id == self.ids[k]) {
                Some(marker) => {
                    self.last[k] = Some(marker.corners);
                    self.unseen_for[k] = 0;
                    self.ever_seen = true;
                }
                None => self.unseen_for[k] = self.unseen_for[k].saturating_add(1),
            }
        }
        markers.sort_by_key(|m| m.id);
        markers
    }

    /// Plan and look at one whole picture in memory (the Library, tests and benches).
    pub fn sight(&mut self, image: &Luma) -> Look {
        let Some(regions) = self.plan(image.width, image.height) else {
            return Look::Skipped;
        };
        let pixels = regions.iter().map(Region::area).sum();
        let crops: Vec<(Luma, Region)> = regions.into_iter().map(|r| (*image, r)).collect();
        Look::Analysed {
            markers: self.look(&crops),
            pixels,
        }
    }
}

/// `region` grown by [`CROP_MARGIN`] and clipped to a `width` x `height` picture: the part of
/// the frame a crop for it must hold.
pub fn crop_for(region: Region, width: usize, height: usize) -> Region {
    Region {
        x0: region.x0.saturating_sub(CROP_MARGIN),
        y0: region.y0.saturating_sub(CROP_MARGIN),
        x1: (region.x1 + CROP_MARGIN).min(width),
        y1: (region.y1 + CROP_MARGIN).min(height),
    }
}

fn side_px(corners: &[[f64; 2]; 4]) -> f64 {
    let mut total = 0.0;
    for i in 0..4 {
        let a = corners[i];
        let b = corners[(i + 1) % 4];
        total += ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    }
    total / 4.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::detect::Detector;
    use crate::detect::tests::drawn;

    #[test]
    fn a_tracked_marker_is_looked_for_in_its_window_and_a_tagless_take_stops() {
        let (w, h) = (640, 480);
        let image = drawn(0, w, h, 60, 60, 8);
        let luma = Luma::new(&image, w, h, w).unwrap();
        let mut tracker = Tracker::new([0, 1], Cadence::default());
        let mut pixels = Vec::new();
        for _ in 0..20 {
            if let Look::Analysed { markers, pixels: p } = tracker.sight(&luma) {
                assert_eq!(markers.len(), 1);
                pixels.push(p);
            }
        }
        assert_eq!(pixels.len(), 20);
        // Marker 1 is never seen: the whole picture is searched once in four frames, and
        // marker 0 only in its window otherwise.
        assert_eq!(pixels.iter().filter(|p| **p == w * h).count(), 5);
        assert!(
            pixels
                .iter()
                .filter(|p| **p != w * h)
                .all(|p| *p < w * h / 10)
        );
        // Warm: no whole-picture search, only the window.
        tracker.searching = false;
        for _ in 0..8 {
            if let Look::Analysed { pixels: p, .. } = tracker.sight(&luma) {
                assert!(p < w * h / 10);
            }
        }

        let blank = vec![200u8; w * h];
        let luma = Luma::new(&blank, w, h, w).unwrap();
        let mut tracker = Tracker::new([0, 1], Cadence::default());
        let mut looked = 0;
        for _ in 0..400 {
            if let Look::Analysed { .. } = tracker.sight(&luma) {
                looked += 1;
            }
        }
        assert_eq!(looked, 75);
        assert!(tracker.gave_up());
    }

    #[test]
    fn a_crop_around_a_region_reads_as_the_whole_frame_does() {
        let (w, h) = (640, 480);
        let image = drawn(1, w, h, 300, 200, 9);
        let whole = Luma::new(&image, w, h, w).unwrap();
        let region = Region {
            x0: 280,
            y0: 180,
            x1: 380,
            y1: 280,
        };
        let expected = Detector::new().detect(&whole, region, &[1]);
        assert_eq!(expected.len(), 1);
        let crop = crop_for(region, w, h);
        let mut bytes = Vec::new();
        for y in crop.y0..crop.y1 {
            bytes.extend_from_slice(&image[y * w + crop.x0..y * w + crop.x1]);
        }
        let cropped = Luma::cropped(&bytes, crop.x1 - crop.x0, crop, w, h).unwrap();
        assert_eq!(Detector::new().detect(&cropped, region, &[1]), expected);
    }
}
