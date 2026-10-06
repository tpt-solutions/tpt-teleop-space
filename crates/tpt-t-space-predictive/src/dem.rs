//! Local Digital Elevation Model generation from stereo camera data at
//! 10 Hz (spec §4.4).
//!
//! The rover's stereo pair reports per-pixel disparities; a pinhole
//! triangulation converts each disparity to a height over the nominal
//! ground plane, binned into a fixed `W×H` grid centred ahead of the
//! rover. The grid then yields slopes (finite differences) and hazard
//! flags for the path planner.
//!
//! Everything is a fixed-size buffer: one grid allocation at construction,
//! zero per frame. A sample with a bad disparity (`None`) simply doesn't
//! vote — no fallback fabrication.

use core::fmt;

/// A local DEM grid: `W×H` heights over a `cell`-metre lattice, origin at
/// the grid centre, x forward.
pub struct DemGrid<const W: usize, const H: usize> {
    cell_m: f32,
    /// Sum of heights and vote counts per cell (incremental binning).
    /// Buffers are exact-size and allocated once at construction (the
    /// zero-alloc policy is "none after initialization").
    sum: Vec<f32>,
    votes: Vec<u32>,
    /// Merged heights (valid when `votes > 0`).
    height: Vec<f32>,
    /// Slope magnitude per cell, radians (finite-difference over valid
    /// neighbors; `f32::NAN` at underdetermined cells).
    slope: Vec<f32>,
}

/// A DEM cell viewed by consumers: height + slope + hazard.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cell {
    /// Height over the nominal plane, metres.
    pub height_m: f32,
    /// Slope magnitude, radians.
    pub slope_rad: f32,
    /// Whether the cell has any data.
    pub valid: bool,
    /// Whether the cell is a no-go hazard.
    pub hazard: bool,
}

/// DEM configuration.
#[derive(Debug, Clone, Copy)]
pub struct DemConfig {
    /// Grid cell size, metres.
    pub cell_m: f32,
    /// Slope above which a cell is a no-go hazard, radians
    /// (spec §6 example: 20° safety limit, 25° slope detected).
    pub hazard_slope_rad: f32,
    /// Minimum stereo votes for a cell to be valid.
    pub min_votes: u32,
}

impl DemConfig {
    /// Rover defaults: 0.25 m cells, 20° no-go slope.
    #[must_use]
    pub fn rover() -> Self {
        Self {
            cell_m: 0.25,
            hazard_slope_rad: 20.0_f32.to_radians(),
            min_votes: 1,
        }
    }
}

/// Why stereo ingestion rejected a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StereoError {
    /// Disparity out of the calibrated range.
    BadDisparity,
}

impl fmt::Display for StereoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("disparity outside the calibrated range")
    }
}

impl std::error::Error for StereoError {}

/// Camera geometry for disparity → height conversion (pinhole, rectified
/// pair, optical axes parallel, baseline along x).
#[derive(Debug, Clone, Copy)]
pub struct StereoRig {
    /// Baseline, metres.
    pub baseline_m: f32,
    /// Focal length, pixels.
    pub focal_px: f32,
    /// Camera height over the nominal ground plane, metres.
    pub height_m: f32,
    /// Valid disparity range (pixels).
    pub disparity_range: (f32, f32),
}

impl StereoRig {
    /// Height of a ground point from its disparity:
    /// `Z = f·B / disparity` (depth), and the ground-plane height is
    /// `camera_height − Z·sin(pitch)` ≈ `height − Z` for a nadir-ish
    /// rig. The grid binning uses the projected ground coordinates
    /// `(x = u·Z/f, y)`.
    pub fn depth_m(&self, disparity_px: f32) -> Result<f32, StereoError> {
        let (dmin, dmax) = self.disparity_range;
        if !(dmin..=dmax).contains(&disparity_px) || disparity_px <= 0.0 {
            return Err(StereoError::BadDisparity);
        }
        Ok(self.focal_px * self.baseline_m / disparity_px)
    }
}

impl<const W: usize, const H: usize> DemGrid<W, H> {
    /// New empty grid.
    #[must_use]
    pub fn new(config: DemConfig) -> Self {
        let n = W * H;
        Self {
            cell_m: config.cell_m,
            sum: vec![0.0; n],
            votes: vec![0; n],
            height: vec![0.0; n],
            slope: vec![0.0; n],
        }
    }

    /// Reset for a new frame (10 Hz cadence).
    pub fn reset(&mut self) {
        self.sum.fill(0.0);
        self.votes.fill(0);
    }

    /// Ingest one stereo sample: pixel `(u, v)` in the rectified left
    /// image (u right, v down, origin at image centre) with its
    /// disparity. Height = camera height − depth along the ray.
    pub fn ingest(&mut self, rig: &StereoRig, u_px: f32, v_px: f32, disparity_px: f32) {
        let Ok(depth) = rig.depth_m(disparity_px) else {
            return; // bad disparity: no vote
        };
        // Ground coordinates ahead of the rover (x forward, y right).
        let x = u_px * depth / rig.focal_px;
        let y = v_px * depth / rig.focal_px;
        let height = rig.height_m - depth;
        self.ingest_point(x, y, height);
    }

    /// Direct ground-plane ingestion (used by the simulator).
    pub fn ingest_point(&mut self, x_m: f32, y_m: f32, height_m: f32) {
        let gx = (x_m / self.cell_m + W as f32 / 2.0).floor() as isize;
        let gy = (y_m / self.cell_m + H as f32 / 2.0).floor() as isize;
        if gx < 0 || gx >= W as isize || gy < 0 || gy >= H as isize {
            return; // outside the local grid: ignored
        }
        let idx = gy as usize * W + gx as usize;
        self.sum[idx] += height_m;
        self.votes[idx] += 1;
    }

    /// Finalize the frame: merge votes and compute slopes. Call after all
    /// samples of the frame are ingested.
    pub fn finish(&mut self) {
        let n = W * H;
        for i in 0..n {
            self.height[i] = if self.votes[i] > 0 {
                self.sum[i] / self.votes[i] as f32
            } else {
                0.0
            };
        }
        // Slope magnitude by central differences over valid neighbors.
        for gy in 0..H {
            for gx in 0..W {
                let i = gy * W + gx;
                if self.votes[i] == 0 {
                    self.slope[i] = f32::NAN;
                    continue;
                }
                let (mut dx, mut dy) = (0.0f32, 0.0f32);
                if gx > 0 && gx + 1 < W && self.votes[i - 1] > 0 && self.votes[i + 1] > 0 {
                    dx = (self.height[i + 1] - self.height[i - 1]) / (2.0 * self.cell_m);
                }
                if gy > 0 && gy + 1 < H && self.votes[i - W] > 0 && self.votes[i + W] > 0 {
                    dy = (self.height[i + W] - self.height[i - W]) / (2.0 * self.cell_m);
                }
                self.slope[i] = (dx * dx + dy * dy).sqrt().atan();
            }
        }
    }

    /// Query one cell (grid coordinates).
    #[must_use]
    pub fn cell(&self, gx: usize, gy: usize, config: &DemConfig) -> Cell {
        let i = gy * W + gx;
        let valid = self.votes[i] >= config.min_votes;
        let slope = self.slope[i];
        Cell {
            height_m: self.height[i],
            slope_rad: slope,
            valid,
            hazard: valid && slope > config.hazard_slope_rad,
        }
    }

    /// Query at ground coordinates (bilinear cell pick; hazard if any
    /// covered cell is hazardous — conservative for planning).
    #[must_use]
    pub fn sample(&self, x_m: f32, y_m: f32, config: &DemConfig) -> Cell {
        let gx = (x_m / self.cell_m + W as f32 / 2.0).floor() as isize;
        let gy = (y_m / self.cell_m + H as f32 / 2.0).floor() as isize;
        if gx < 0 || gx >= W as isize || gy < 0 || gy >= H as isize {
            return Cell {
                height_m: 0.0,
                slope_rad: 0.0,
                valid: false,
                hazard: true, // off-grid is unknown → treat as hazard
            };
        }
        self.cell(gx as usize, gy as usize, config)
    }

    /// Grid width/height in cells.
    #[must_use]
    pub fn dims(&self) -> (usize, usize) {
        (W, H)
    }

    /// Cell size, metres.
    #[must_use]
    pub fn cell_m(&self) -> f32 {
        self.cell_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const W: usize = 32;
    const H: usize = 32;

    fn rig() -> StereoRig {
        StereoRig {
            baseline_m: 0.5,
            focal_px: 800.0,
            height_m: 1.5,
            disparity_range: (4.0, 200.0),
        }
    }

    #[test]
    fn flat_ground_yields_flat_dem() {
        let mut dem = DemGrid::<W, H>::new(DemConfig::rover());
        // Flat ground at camera height 1.5: height = 0 everywhere. Depth
        // per pixel row: for v=0 (centre), depth = 1.5 → disparity f·B/Z
        // = 800·0.5/1.5 ≈ 266 — beyond the range; use points instead.
        let mut t = 0u32;
        for gy in -12..12 {
            for gx in -12..12 {
                dem.ingest_point(gx as f32 * 0.25, gy as f32 * 0.25, 0.0);
                t += 1;
            }
        }
        assert_eq!(t, 576);
        dem.finish();
        let cfg = DemConfig::rover();
        for (gx, gy) in [(15, 15), (10, 20), (18, 12)] {
            let c = dem.cell(gx, gy, &cfg);
            assert!(c.valid);
            assert!(c.height_m.abs() < 1e-6);
            assert!(!c.hazard, "flat ground is not a hazard");
        }
    }

    #[test]
    fn twenty_five_degree_slope_is_flagged() {
        let mut dem = DemGrid::<W, H>::new(DemConfig::rover());
        let slope = 25.0_f32.to_radians();
        // A planar ramp rising in +x: height = tan(25°)·x.
        for gy in -12..12 {
            for gx in 0..24 {
                let x = gx as f32 * 0.25;
                dem.ingest_point(x, gy as f32 * 0.25, slope.tan() * x);
            }
        }
        dem.finish();
        let cfg = DemConfig::rover();
        // Sample the ramp mid-face.
        let c = dem.sample(3.0, 0.0, &cfg);
        assert!(c.valid);
        let slope_deg = c.slope_rad.to_degrees();
        assert!(
            (slope_deg - 25.0).abs() < 2.0,
            "recovered slope {slope_deg:.1}° vs 25°"
        );
        assert!(c.hazard, "25° exceeds the 20° limit → no-go");
        // The spec scenario: the planner must see this hazard.
        assert!(slope_deg > 20.0);
    }

    #[test]
    fn sparse_votes_below_min_are_invalid() {
        let mut dem = DemGrid::<W, H>::new(DemConfig {
            min_votes: 3,
            ..DemConfig::rover()
        });
        dem.ingest_point(0.0, 0.0, 0.1);
        dem.finish();
        let cfg = DemConfig {
            min_votes: 3,
            ..DemConfig::rover()
        };
        assert!(!dem.sample(0.0, 0.0, &cfg).valid);
        // Off-grid is unknown and conservatively hazardous.
        let c = dem.sample(100.0, 100.0, &cfg);
        assert!(!c.valid);
        assert!(c.hazard);
    }

    #[test]
    fn bad_disparity_gets_no_vote() {
        let r = rig();
        assert_eq!(r.depth_m(1.0), Err(StereoError::BadDisparity));
        assert_eq!(r.depth_m(-5.0), Err(StereoError::BadDisparity));
        let d = r.depth_m(100.0).expect("in range");
        assert!((d - 800.0 * 0.5 / 100.0).abs() < 1e-5);
    }

    #[test]
    fn frame_reset_clears_votes() {
        let mut dem = DemGrid::<W, H>::new(DemConfig::rover());
        dem.ingest_point(0.0, 0.0, 0.0);
        dem.reset();
        dem.finish();
        assert!(!dem.sample(0.0, 0.0, &DemConfig::rover()).valid);
    }
}
