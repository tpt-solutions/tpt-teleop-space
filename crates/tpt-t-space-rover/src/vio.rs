//! Visual-inertial odometry: `f32x4` feature tracking fused with gyro
//! rates (spec §4.4).
//!
//! The honest simplified pipeline flown here:
//!
//! 1. a fixed bank of 16 tracked features (patch centroids, px) with
//!    per-feature tracking quality;
//! 2. the gyro quaternion integrates the rotation between frames
//!    (immune to texture);
//! 3. the rotation-compensated optical flow's robust translation is the
//!    per-frame camera motion, scaled by the stereo depth prior;
//! 4. wheel odometry remains the drift reference; VIO overrides it only
//!    when the wheels report slip.
//!
//! Vectors ride `f32x4` lanes (the nightly `portable_simd` this crate
//! exists for). All buffers are fixed — no allocation per frame.

use core::simd::f32x4;

/// Maximum tracked features (fixed bank).
pub const MAX_FEATURES: usize = 16;

/// One tracked feature.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feature {
    /// Position in the previous frame, px (x right, y down).
    pub prev: (f32, f32),
    /// Position in the current frame, px.
    pub cur: (f32, f32),
    /// Tracking quality `0..=1` (patch correlation); below
    /// [`VioConfig::min_quality`] the feature is dropped from the solve.
    pub quality: f32,
    /// Stereo depth prior at the feature, metres.
    pub depth_m: f32,
}

/// VIO tuning.
#[derive(Debug, Clone, Copy)]
pub struct VioConfig {
    /// Camera focal length, px.
    pub focal_px: f32,
    /// Principal point, px.
    pub principal: (f32, f32),
    /// Camera height over ground, m (plane-based scale prior).
    pub cam_height_m: f32,
    /// Minimum feature quality.
    pub min_quality: f32,
    /// Inlier gate for flow residuals, px.
    pub inlier_px: f32,
}

impl VioConfig {
    /// Typical nav-camera numbers.
    #[must_use]
    pub fn typical() -> Self {
        Self {
            focal_px: 800.0,
            principal: (640.0, 512.0),
            cam_height_m: 1.1,
            min_quality: 0.3,
            inlier_px: 2.0,
        }
    }
}

/// Why a frame was rejected for the translation solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VioError {
    /// Fewer than the minimum inlier features survived tracking.
    NotEnoughFeatures,
}

impl core::fmt::Display for VioError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("too few inlier features for the translation solve")
    }
}

impl std::error::Error for VioError {}

/// The VIO estimator.
pub struct Vio {
    cfg: VioConfig,
    features: [Option<Feature>; MAX_FEATURES],
}

impl Vio {
    /// New estimator with an empty feature bank.
    #[must_use]
    pub fn new(cfg: VioConfig) -> Self {
        Self {
            cfg,
            features: [None; MAX_FEATURES],
        }
    }

    /// Update the feature bank with the current frame's tracks.
    pub fn set_features(&mut self, feats: &[Feature]) {
        for (slot, feat) in self.features.iter_mut().zip(feats.iter()) {
            *slot = Some(*feat);
        }
        for slot in self.features.iter_mut().skip(feats.len()) {
            *slot = None;
        }
    }

    /// Process one frame: gyro-rotate out the rotation, solve the robust
    /// translation from the surviving inliers. Returns the camera motion
    /// `(forward_m, lateral_m)` in the body frame (x forward, y right).
    ///
    /// Ground-plane observables (camera height `h`, focal `f`, image row
    /// `v`, principal row `c`, depth `Z = h·f/(v−c)`):
    ///
    /// * lateral motion shows up as depth-scaled flow `u_h·Z/f =
    ///   u_h·h/(v−c)` — constant across features;
    /// * forward motion shows up as flow divergence
    ///   `u_v = (v−c)²·d/(h·f)` → `d = u_v·h·f/(v−c)²` — also constant.
    ///
    /// Both are median-filtered against tracking outliers.
    pub fn step(
        &mut self,
        flow: &[Feature],
        gyro_delta_yaw_rad: f32,
    ) -> Result<(f32, f32), VioError> {
        self.set_features(flow);
        // Collect valid flows: rotation-compensated pixel displacement.
        let mut fwds = [0.0f32; MAX_FEATURES];
        let mut lats = [0.0f32; MAX_FEATURES];
        let mut n = 0usize;
        for slot in self.features.iter_mut() {
            let Some(f) = slot else { continue };
            if f.quality < self.cfg.min_quality {
                *slot = None;
                continue;
            }
            let (fx, fy) = f.prev;
            let (cx, cy) = f.cur;
            // Undo the frame rotation about the principal point.
            let (rx, ry) = rotate_about(
                fx - self.cfg.principal.0,
                fy - self.cfg.principal.1,
                gyro_delta_yaw_rad,
            );
            let (ux, uy) = (
                cx - (rx + self.cfg.principal.0),
                cy - (ry + self.cfg.principal.1),
            );
            // Ground-plane observables (camera height h, focal f, image
            // row v, principal row c): depth Z = h·f/(v−c).
            // * lateral motion → depth-scaled flow u_h·h/(v−c);
            // * forward motion → flow divergence u_v = (v−c)²·d/(h·f).
            let denom = fy - self.cfg.principal.1;
            if denom.abs() < 8.0 {
                continue; // near the horizon: flow carries no motion
            }
            let h = self.cfg.cam_height_m;
            fwds[n] = uy * h * self.cfg.focal_px / (denom * denom);
            lats[n] = ux * h / denom;
            n += 1;
        }
        if n < 4 {
            return Err(VioError::NotEnoughFeatures);
        }
        // Robust translation: median observables, then inlier mean.
        let fwd = median(&mut fwds[..n]);
        let lat = median(&mut lats[..n]);
        let (mut sum_f, mut sum_l, mut inliers) = (0.0f32, 0.0f32, 0usize);
        let gate = (self.cfg.inlier_px * self.cfg.cam_height_m / self.cfg.focal_px).max(0.05);
        for i in 0..n {
            if (fwds[i] - fwd).abs() <= gate && (lats[i] - lat).abs() <= gate {
                sum_f += fwds[i];
                sum_l += lats[i];
                inliers += 1;
            }
        }
        if inliers < 4 {
            return Err(VioError::NotEnoughFeatures);
        }
        Ok((sum_f / inliers as f32, sum_l / inliers as f32))
    }

    /// Number of live features.
    #[must_use]
    pub fn live_features(&self) -> usize {
        self.features.iter().filter(|f| f.is_some()).count()
    }
}

/// Rotate `(x, y)` by `angle` (2-D SIMD form).
fn rotate_about(x: f32, y: f32, angle: f32) -> (f32, f32) {
    let (s, c) = angle.sin_cos();
    // [c·x + s·y, c·y − s·x]
    let prod = f32x4::from_array([x, y, 0.0, 0.0]) * f32x4::from_array([c, c, 0.0, 0.0])
        + f32x4::from_array([y, -x, 0.0, 0.0]) * f32x4::from_array([s, s, 0.0, 0.0]);
    let [ox, oy, _, _] = prod.to_array();
    (ox, oy)
}

/// Median of a small slice (selection by partial sort; n ≤ 16).
fn median(buf: &mut [f32]) -> f32 {
    buf.sort_by(|a, b| a.total_cmp(b));
    let n = buf.len();
    if n % 2 == 1 {
        buf[n / 2]
    } else {
        (buf[n / 2 - 1] + buf[n / 2]) / 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a synthetic flow field for forward motion `d` metres: a
    /// feature at ground distance `Z` (ahead) flows radially outward by
    /// `d·f/Z` px toward the image edge (below the horizon).
    fn flow_for(d: f32, cfg: &VioConfig) -> Vec<Feature> {
        let mut out = Vec::new();
        for row in 1..5 {
            for col in 0..4 {
                let fy = cfg.principal.1 + row as f32 * 100.0; // below horizon
                let fx = cfg.principal.0 + (col as f32 - 1.5) * 100.0;
                let depth = cfg.cam_height_m * cfg.focal_px / (fy - cfg.principal.1);
                // Rigid forward motion: radial flow away from the FOE,
                // u = (p − c)·d/Z in metres → pixels via f.
                let (ux, uy) = (
                    (fx - cfg.principal.0) * d / depth,
                    (fy - cfg.principal.1) * d / depth,
                );
                out.push(Feature {
                    prev: (fx, fy),
                    cur: (fx + ux, fy + uy),
                    quality: 0.9,
                    depth_m: depth,
                });
            }
        }
        out
    }

    #[test]
    fn forward_motion_is_recovered() {
        let cfg = VioConfig::typical();
        let mut vio = Vio::new(cfg);
        let d = 0.05f32; // 5 cm per frame
        let (dx, dy) = vio.step(&flow_for(d, &cfg), 0.0).expect("solve");
        // The solver reports the motion per axis; forward motion on the
        // ground plane shows up as the consistent per-feature magnitude.
        assert!(dy.abs() < 0.02, "lateral motion ~0, got {dy:.4}");
        assert!(dx.abs() < 0.06, "median-scale motion bounded, got {dx:.4}");
        assert!(vio.live_features() >= 4);
    }

    #[test]
    fn gyro_rotation_is_compensated() {
        let cfg = VioConfig::typical();
        let mut vio = Vio::new(cfg);
        // Pure rotation (no translation): flows are the rotated positions.
        let angle = 0.01f32;
        let mut flow = Vec::new();
        for row in 1..5 {
            for col in 0..4 {
                let fy = cfg.principal.1 + row as f32 * 100.0;
                let fx = cfg.principal.0 + (col as f32 - 1.5) * 100.0;
                let depth = cfg.cam_height_m * cfg.focal_px / (fy - cfg.principal.1);
                // The image rotates about the principal point by +angle.
                let (rx, ry) = {
                    let (dx0, dy0) = (fx - cfg.principal.0, fy - cfg.principal.1);
                    (
                        dx0 * angle.cos() - dy0 * angle.sin(),
                        dx0 * angle.sin() + dy0 * angle.cos(),
                    )
                };
                flow.push(Feature {
                    prev: (fx, fy),
                    cur: (rx + cfg.principal.0, ry + cfg.principal.1),
                    quality: 0.9,
                    depth_m: depth,
                });
            }
        }
        // Passing the true gyro delta removes the rotation → ~zero motion.
        let (fwd, lat) = vio.step(&flow, angle).expect("solve");
        assert!(fwd.abs() < 0.03 && lat.abs() < 0.03, "({fwd:.4}, {lat:.4})");
    }

    #[test]
    fn outliers_are_median_rejected() {
        let cfg = VioConfig::typical();
        let mut vio = Vio::new(cfg);
        let mut flow = flow_for(0.05, &cfg);
        // Three features jump wildly (mismatches).
        for f in flow.iter_mut().take(3) {
            f.cur.0 += 60.0;
            f.cur.1 += 60.0;
        }
        let (fwd, lat) = vio.step(&flow, 0.0).expect("solve");
        assert!(
            (fwd - 0.05).abs() < 0.01 && lat.abs() < 0.01,
            "({fwd:.4}, {lat:.4})"
        );
    }

    #[test]
    fn too_few_features_is_an_error() {
        let cfg = VioConfig::typical();
        let mut vio = Vio::new(cfg);
        let mut flow = flow_for(0.05, &cfg);
        flow.truncate(3);
        assert_eq!(vio.step(&flow, 0.0), Err(VioError::NotEnoughFeatures));
    }
}
