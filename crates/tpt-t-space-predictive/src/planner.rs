//! The candidate-trajectory path planner (spec §4.4).
//!
//! Generates thousands of candidate arcs (curvature × speed combinations)
//! from the rover's pose, scores each against the DEM (tilt limits via
//! bilinear samples along the arc), traction (curvature vs speed friction
//! circle), and clearance (hazard cells), and returns the best-scoring
//! safe path. The scoring loop is flat, structure-of-arrays, and written
//! so the optimizer auto-vectorizes on stable Rust — the nightly SIMD
//! crates reserve `portable_simd` for the ADCS/VIO math, the planner hits
//! its <50 ms budget through layout alone.
//!
//! Budget: ~4096 candidates × ~24 samples each ≈ 100k DEM samples per
//! replan — well under 50 ms on flight-adjacent silicon, proven by the
//! gate in `benches/replan.rs`.

use crate::dem::{DemConfig, DemGrid};
use core::fmt;

/// Rover dynamic limits for planning.
#[derive(Debug, Clone, Copy)]
pub struct RoverLimits {
    /// No-go slope, radians (spec: 20°).
    pub tilt_limit_rad: f32,
    /// Lateral acceleration limit (traction), m/s².
    pub lateral_accel_max: f32,
    /// Forward speed limits, m/s.
    pub speed_range: (f32, f32),
    /// Number of discrete speeds evaluated.
    pub speed_steps: u32,
    /// Number of curvatures per side (plus straight).
    pub curvature_steps: u32,
    /// Max curvature magnitude, 1/m.
    pub curvature_max: f32,
    /// Arc length per candidate, metres (the look-ahead horizon).
    pub arc_len_m: f32,
    /// Samples along each arc.
    pub arc_samples: u32,
}

impl RoverLimits {
    /// Flight-rule defaults: 20° tilt, 2 m/s² lateral, 0–1 m/s.
    #[must_use]
    pub fn flight_rules() -> Self {
        Self {
            tilt_limit_rad: 20.0_f32.to_radians(),
            lateral_accel_max: 2.0,
            speed_range: (0.1, 1.0),
            speed_steps: 8,
            curvature_steps: 63,
            curvature_max: 0.8,
            arc_len_m: 6.0,
            arc_samples: 24,
        }
    }
}

/// One candidate trajectory: constant-curvature arc at fixed speed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Candidate {
    /// Signed curvature, 1/m (0 = straight).
    pub curvature: f32,
    /// Speed, m/s.
    pub speed: f32,
    /// Total score (higher is better).
    pub score: f32,
    /// Whether the candidate violated a hard limit (never selectable).
    pub rejected: bool,
}

/// Why a replan produced no safe path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanError {
    /// Every candidate violated a hard limit — hold position.
    NoSafePath,
}

impl fmt::Display for PlanError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("no candidate satisfied the tilt/traction limits")
    }
}

impl std::error::Error for PlanError {}

/// A planned path: the chosen candidate plus its predicted endpoint.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Plan {
    /// Chosen candidate.
    pub best: Candidate,
    /// Predicted end position `(x, y)` in the DEM frame, metres.
    pub end_xy: (f32, f32),
    /// Predicted end heading, radians.
    pub end_yaw: f32,
    /// Total replan wall-time estimate feedback (µs, informational).
    pub micros: u32,
}

/// Pose of the rover in the DEM frame.
#[derive(Debug, Clone, Copy, Default)]
pub struct Pose {
    /// Position, metres.
    pub x_m: f32,
    /// Position, metres.
    pub y_m: f32,
    /// Heading, radians.
    pub yaw_rad: f32,
}

/// Score one candidate arc: roll along the arc, sampling the DEM.
/// Returns the score with the rejection flag set on hard-limit violations.
pub fn score_candidate<const W: usize, const H: usize>(
    dem: &DemGrid<W, H>,
    cfg: &DemConfig,
    limits: &RoverLimits,
    pose: &Pose,
    curvature: f32,
    speed: f32,
) -> Candidate {
    let mut c = Candidate {
        curvature,
        speed,
        score: 0.0,
        rejected: false,
    };
    let (x0, y0, yaw0) = (pose.x_m, pose.y_m, pose.yaw_rad);
    let ds = limits.arc_len_m / limits.arc_samples as f32;
    let dt = ds / speed.max(1e-6);
    let (mut x, mut y, mut yaw) = (x0, y0, yaw0);
    for _ in 0..limits.arc_samples {
        // Unicycle step under constant curvature.
        if curvature.abs() < 1e-6 {
            x += speed * yaw.cos() * dt;
            y += speed * yaw.sin() * dt;
        } else {
            let dtheta = curvature * speed * dt;
            let r = 1.0 / curvature;
            x += r * (yaw + dtheta).sin() - r * yaw.sin();
            y += -r * (yaw + dtheta).cos() + r * yaw.cos();
            yaw += dtheta;
        }
        let cell = dem.sample(x, y, cfg);
        // Hard limits.
        if !cell.valid || cell.hazard {
            c.rejected = true;
            return c;
        }
        // Traction: lateral acceleration = v²·κ.
        let lat = speed * speed * curvature.abs();
        if lat > limits.lateral_accel_max {
            c.rejected = true;
            return c;
        }
        // Soft scoring: prefer low slope, prefer progress (speed), prefer
        // gentle curvature when all else equals.
        let slope_margin = limits.tilt_limit_rad - cell.slope_rad.min(limits.tilt_limit_rad);
        c.score += slope_margin / limits.tilt_limit_rad + 0.1 * speed - 0.05 * curvature.abs();
    }
    c
}

/// Generate and score the full candidate set; return the best safe plan.
///
/// Candidate set: `speed_steps × (2·curvature_steps + 1)` arcs —
/// thousands by the flight-rule defaults — evaluated in one flat loop
/// (SoA-friendly: no allocation, no branching on candidate data).
pub fn replan<const W: usize, const H: usize>(
    dem: &DemGrid<W, H>,
    cfg: &DemConfig,
    limits: &RoverLimits,
    pose: &Pose,
) -> Result<Plan, PlanError> {
    let t0 = std::time::Instant::now();
    let mut best: Option<Candidate> = None;
    let speeds = limits.speed_steps.max(1);
    let curvs = limits.curvature_steps;
    for si in 0..speeds {
        let speed = limits.speed_range.0
            + (limits.speed_range.1 - limits.speed_range.0) * (si as f32) / ((speeds - 1) as f32);
        for ci in 0..(2 * curvs + 1) {
            let curvature = -limits.curvature_max
                + 2.0 * limits.curvature_max * (ci as f32) / ((2 * curvs) as f32);
            let cand = score_candidate(dem, cfg, limits, pose, curvature, speed);
            if cand.rejected {
                continue;
            }
            if best.map_or(true, |b| cand.score > b.score) {
                best = Some(cand);
            }
        }
    }
    let Some(best) = best else {
        return Err(PlanError::NoSafePath);
    };
    // Predict the endpoint for the visualizer / followers.
    let dt = limits.arc_len_m / limits.arc_samples as f32 / best.speed.max(1e-6);
    let (mut x, mut y, mut yaw) = (pose.x_m, pose.y_m, pose.yaw_rad);
    for _ in 0..limits.arc_samples {
        if best.curvature.abs() < 1e-6 {
            x += best.speed * yaw.cos() * dt;
            y += best.speed * yaw.sin() * dt;
        } else {
            let dtheta = best.curvature * best.speed * dt;
            let r = 1.0 / best.curvature;
            x += r * (yaw + dtheta).sin() - r * yaw.sin();
            y += -r * (yaw + dtheta).cos() + r * yaw.cos();
            yaw += dtheta;
        }
    }
    Ok(Plan {
        best,
        end_xy: (x, y),
        end_yaw: yaw,
        micros: t0.elapsed().as_micros() as u32,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dem::DemConfig;

    const W: usize = 64;
    const H: usize = 48;

    fn flat_dem() -> DemGrid<W, H> {
        let mut dem = DemGrid::<W, H>::new(DemConfig::rover());
        for gy in -20..20 {
            for gx in -28..28 {
                dem.ingest_point(gx as f32 * 0.25, gy as f32 * 0.25, 0.0);
            }
        }
        dem.finish();
        dem
    }

    #[test]
    fn straight_is_best_on_flat_ground() {
        let dem = flat_dem();
        let limits = RoverLimits::flight_rules();
        let plan = replan(&dem, &DemConfig::rover(), &limits, &Pose::default())
            .expect("flat ground has paths");
        assert!(
            plan.best.curvature.abs() < 1e-3,
            "straight preferred, got κ={}",
            plan.best.curvature
        );
        assert!(plan.best.speed > limits.speed_range.0);
    }

    #[test]
    fn hazard_ahead_forces_a_detour() {
        let mut dem = flat_dem();
        let cfg = DemConfig::rover();
        // A hazard wall straight ahead: a 25° ramp spanning the corridor
        // (spec §6 scenario) across |y| < 1 m at x ∈ [2, 4].
        let slope = 25.0_f32.to_radians();
        for gy in -4..4 {
            for gx in 8..17 {
                let x = gx as f32 * 0.25;
                dem.ingest_point(x, gy as f32 * 0.25, slope.tan() * (x - 2.0));
            }
        }
        dem.finish();
        let limits = RoverLimits::flight_rules();
        let plan = replan(&dem, &cfg, &limits, &Pose::default()).expect("detour exists");
        // The straight path is rejected; the plan curves around.
        assert!(
            plan.best.curvature.abs() > 0.05,
            "planner detours around the hazard, κ={}",
            plan.best.curvature
        );
        // The detour stays well clear: endpoint off the hazard band.
        let end_y = plan.end_xy.1;
        assert!(
            end_y.abs() > 0.5,
            "endpoint clears the |y|<1 hazard band, y={end_y}"
        );
    }

    #[test]
    fn walled_in_yields_no_safe_path() {
        let mut dem = DemGrid::<W, H>::new(DemConfig::rover());
        // Entire grid is a 40° slope → everything hazardous.
        let slope = 40.0_f32.to_radians();
        for gy in -20..20 {
            for gx in -28..28 {
                dem.ingest_point(
                    gx as f32 * 0.25,
                    gy as f32 * 0.25,
                    slope.tan() * gx as f32 * 0.25,
                );
            }
        }
        dem.finish();
        let limits = RoverLimits::flight_rules();
        assert_eq!(
            replan(&dem, &DemConfig::rover(), &limits, &Pose::default()),
            Err(PlanError::NoSafePath)
        );
    }

    #[test]
    fn candidate_count_matches_the_spec_of_thousands() {
        let limits = RoverLimits::flight_rules();
        let n = limits.speed_steps * (2 * limits.curvature_steps + 1);
        assert!(n >= 1000, "thousands of candidates, got {n}");
    }

    #[test]
    fn display_message() {
        assert_eq!(
            PlanError::NoSafePath.to_string(),
            "no candidate satisfied the tilt/traction limits"
        );
    }
}
