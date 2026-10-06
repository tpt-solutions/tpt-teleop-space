//! Autonomous hazard avoidance consuming the `tpt-t-space-predictive`
//! DEM and path planner (spec §4.4 / §6).
//!
//! The rover's traverse loop at 10 Hz: stereo → DEM → replan → follow.
//! [`HazardAvoidance::traverse_tick`] is one loop: it returns the body
//! command for the followers, or [`Avoidance::Replan`] bookkeeping the
//! mission loop can audit. The §6 scenario (25° slope detected, 15 m
//! detour replanned in 40 ms, no ground input) is the crate's integration
//! test (`tests/detour.rs`).

use tpt_t_space_predictive::dem::{DemConfig, DemGrid};
use tpt_t_space_predictive::planner::{PlanError, Pose, RoverLimits, replan};

/// What one traverse tick decided.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Avoidance {
    /// Following the current plan at the planned curvature/speed.
    Following {
        /// Planned curvature, 1/m.
        curvature: f32,
        /// Planned speed, m/s.
        speed: f32,
    },
    /// The plan was refreshed this tick (new DEM data or plan expiry).
    Replanned {
        /// Planned curvature, 1/m.
        curvature: f32,
        /// Planned speed, m/s.
        speed: f32,
        /// Replan duration, µs (telemetry for the §6 budget audit).
        replan_us: u32,
    },
    /// No safe path — hold position and raise the flag.
    Holding {
        /// Ticks held (for the ground report).
        ticks: u32,
    },
}

/// Traverse controller: owns the planner limits and the plan lifetime.
pub struct HazardAvoidance {
    limits: RoverLimits,
    dem_cfg: DemConfig,
    /// Replan cadence, ticks (10 Hz DEM → replan every N ticks or on
    /// hazard detection).
    replan_every: u32,
    ticks_since_replan: u32,
    hold_ticks: u32,
    last_plan: Option<(f32, f32)>, // (curvature, speed)
    /// Total replans (telemetry).
    pub replans: u32,
}

impl HazardAvoidance {
    /// New avoidance controller with flight-rule limits.
    #[must_use]
    pub fn new(replan_every: u32) -> Self {
        Self {
            limits: RoverLimits::flight_rules(),
            dem_cfg: DemConfig::rover(),
            replan_every: replan_every.max(1),
            ticks_since_replan: 0,
            hold_ticks: 0,
            last_plan: None,
            replans: 0,
        }
    }

    /// One traverse tick over the current DEM at the current pose.
    pub fn traverse_tick<const W: usize, const H: usize>(
        &mut self,
        dem: &DemGrid<W, H>,
        pose: &Pose,
    ) -> Avoidance {
        self.ticks_since_replan += 1;
        let due = self.ticks_since_replan >= self.replan_every || self.last_plan.is_none();
        if due {
            match replan(dem, &self.dem_cfg, &self.limits, pose) {
                Ok(plan) => {
                    self.ticks_since_replan = 0;
                    self.hold_ticks = 0;
                    self.replans += 1;
                    self.last_plan = Some((plan.best.curvature, plan.best.speed));
                    return Avoidance::Replanned {
                        curvature: plan.best.curvature,
                        speed: plan.best.speed,
                        replan_us: plan.micros,
                    };
                }
                Err(PlanError::NoSafePath) => {
                    self.hold_ticks += 1;
                    self.last_plan = None;
                    return Avoidance::Holding {
                        ticks: self.hold_ticks,
                    };
                }
            }
        }
        let (curvature, speed) = self.last_plan.expect("due path covers None");
        Avoidance::Following { curvature, speed }
    }

    /// Force a replan next tick (mission-loop nudge, e.g. waypoint change).
    pub fn invalidate_plan(&mut self) {
        self.ticks_since_replan = self.replan_every;
    }

    /// Planner limits view.
    #[must_use]
    pub fn limits(&self) -> &RoverLimits {
        &self.limits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const W: usize = 64;
    const H: usize = 48;

    fn flat() -> DemGrid<W, H> {
        let mut dem = DemGrid::new(DemConfig::rover());
        for gy in -20..20 {
            for gx in -28..28 {
                dem.ingest_point(gx as f32 * 0.25, gy as f32 * 0.25, 0.0);
            }
        }
        dem.finish();
        dem
    }

    #[test]
    fn follows_flat_ground_without_churn() {
        let dem = flat();
        let mut av = HazardAvoidance::new(5);
        let mut decisions = Vec::new();
        for _ in 0..12 {
            decisions.push(av.traverse_tick(&dem, &Pose::default()));
        }
        let replans = decisions
            .iter()
            .filter(|d| matches!(d, Avoidance::Replanned { .. }))
            .count();
        assert_eq!(replans, 3, "replan every 5 ticks over 12");
        assert!(
            decisions
                .iter()
                .all(|d| !matches!(d, Avoidance::Holding { .. }))
        );
    }

    #[test]
    fn holds_when_walled_in_and_resumes_when_clear() {
        let mut dem = DemGrid::<W, H>::new(DemConfig::rover());
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
        let mut av = HazardAvoidance::new(1);
        assert!(matches!(
            av.traverse_tick(&dem, &Pose::default()),
            Avoidance::Holding { .. }
        ));
        // Terrain clears: the controller resumes within one tick.
        let clear = flat();
        av.invalidate_plan();
        assert!(matches!(
            av.traverse_tick(&clear, &Pose::default()),
            Avoidance::Replanned { .. }
        ));
    }
}
