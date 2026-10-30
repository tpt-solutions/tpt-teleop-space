//! Simulated target headroom (Phase 16: performance on real silicon,
//! simulated).
//!
//! No target silicon is available. This times the hot paths on the host and
//! reports how much slower a target could be before each spec budget fails.
//! That is a bound on the target's speed, not a measurement of it.
//!
//! The ADCS control loop (budget 200 µs) lives in the nightly satellite
//! crate, which this crate cannot call, so it is listed as not measured.

use std::hint::black_box;
use std::time::Instant;

use tpt_t_space_predictive::dem::DemConfig;
use tpt_t_space_predictive::planner::{Pose, RoverLimits, replan};
use tpt_t_space_radiation::tmr::TmrVoter;
use tpt_t_space_sim::scenario;

/// One budget and what the host took to meet it.
#[derive(Debug, Clone, PartialEq)]
pub struct Headroom {
    /// The hot path.
    pub name: &'static str,
    /// Spec budget, microseconds.
    pub budget_us: f64,
    /// Median host time, microseconds. Zero when not measured.
    pub host_us: f64,
    /// Whether the host ran this path at all.
    pub measured: bool,
}

impl Headroom {
    /// How many times slower than this host a target could be and still meet
    /// the budget. `None` when the path was not measured.
    #[must_use]
    pub fn slowdown_limit(&self) -> Option<f64> {
        (self.measured && self.host_us > 0.0).then(|| self.budget_us / self.host_us)
    }
}

/// Median of `reps` timed runs of `f`, each run doing `inner` calls, in µs
/// per call.
fn median_us(reps: u32, inner: u32, mut f: impl FnMut()) -> f64 {
    let mut samples: Vec<f64> = (0..reps)
        .map(|_| {
            let start = Instant::now();
            for _ in 0..inner {
                f();
            }
            start.elapsed().as_secs_f64() * 1e6 / f64::from(inner)
        })
        .collect();
    samples.sort_by(f64::total_cmp);
    samples[samples.len() / 2]
}

/// Measure the budgeted hot paths, taking the median of `reps` runs each.
#[must_use]
pub fn measure(reps: u32) -> Vec<Headroom> {
    let reps = reps.max(1);

    let mut voter = TmrVoter::default();
    let tmr = median_us(reps, 10_000, || {
        black_box(voter.submit(black_box([7u32, 7, 7])));
    });

    let dem = scenario::terrain();
    let cfg = DemConfig::rover();
    let limits = RoverLimits::flight_rules();
    let pose = Pose {
        x_m: 0.0,
        y_m: 0.0,
        yaw_rad: 0.0,
    };
    let replan_us = median_us(reps, 5, || {
        black_box(replan(&dem, &cfg, &limits, &pose).is_ok());
    });

    vec![
        Headroom {
            name: "TMR vote and reset trigger",
            budget_us: 50.0,
            host_us: tmr,
            measured: true,
        },
        Headroom {
            name: "rover replan (spec §6)",
            budget_us: 50_000.0,
            host_us: replan_us,
            measured: true,
        },
        Headroom {
            name: "ADCS control loop",
            budget_us: 200.0,
            host_us: 0.0,
            measured: false,
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_report_covers_each_budget_and_marks_the_unmeasured_one() {
        let rows = measure(3);
        assert_eq!(rows.len(), 3);
        let adcs = rows
            .iter()
            .find(|r| r.name == "ADCS control loop")
            .expect("adcs row");
        assert!(!adcs.measured);
        assert_eq!(adcs.slowdown_limit(), None);
        for r in rows.iter().filter(|r| r.measured) {
            assert!(r.host_us > 0.0, "{r:?}");
            assert!(r.slowdown_limit().is_some_and(|s| s > 1.0), "{r:?}");
        }
    }
}
