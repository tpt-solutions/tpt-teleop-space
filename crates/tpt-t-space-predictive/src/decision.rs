//! The autonomous decision framework shared by rover hazard avoidance and
//! orbital docking approach (spec §3, `tpt-t-space-predictive`).
//!
//! Both domains reduce to the same loop: propose candidates, score them
//! against a domain evaluator, select the best safe one. [`Evaluator`]
//! is the contract; [`select_best`] is the fixed-top-1 selection with a
//! hard safety gate — a candidate that fails [`EvalOutcome::safe`] is
//! never selectable regardless of score. Deterministic, allocation-free.

/// Outcome of evaluating one candidate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EvalOutcome {
    /// Soft score (higher is better).
    pub score: f32,
    /// Hard safety gate: only safe candidates are selectable.
    pub safe: bool,
}

/// The domain-side evaluator contract.
pub trait Evaluator {
    /// The candidate type this evaluator scores.
    type Candidate: Copy;
    /// Score one candidate.
    fn evaluate(&mut self, candidate: &Self::Candidate) -> EvalOutcome;
}

/// Select the best safe candidate. Returns `None` when none are safe —
/// the caller's cue to hold/shadow (rover) or abort to a hold orbit
/// (servicer). `candidates` is any exact-size iterator; no allocation.
pub fn select_best<C: Copy, E: Evaluator<Candidate = C>>(
    evaluator: &mut E,
    candidates: impl Iterator<Item = C>,
) -> Option<C> {
    let mut best: Option<(C, f32)> = None;
    for c in candidates {
        let out = evaluator.evaluate(&c);
        if !out.safe {
            continue;
        }
        if best.map_or(true, |(_, s)| out.score > s) {
            best = Some((c, out.score));
        }
    }
    best.map(|(c, _)| c)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 1-D "approach corridor" evaluator: candidates are approach
    /// speeds; safety = stopping distance under the keep-out sphere,
    /// score = closes distance fast (shared by docking).
    struct Approach {
        distance_m: f32,
        max_decel: f32,
    }

    impl Evaluator for Approach {
        type Candidate = f32;

        fn evaluate(&mut self, &speed: &f32) -> EvalOutcome {
            let stopping = speed * speed / (2.0 * self.max_decel);
            EvalOutcome {
                // Reward closing speed, temper with a braking-effort tax.
                score: speed - 0.4 * stopping,
                safe: stopping < self.distance_m,
            }
        }
    }

    #[test]
    fn docking_approach_selects_the_fastest_safe_speed() {
        let mut e = Approach {
            distance_m: 2.0,
            max_decel: 0.5,
        };
        let speeds = [0.2f32, 0.5, 1.0, 1.4, 2.0]; // stopping: 0.04..4.0 m
        let best = select_best(&mut e, speeds.iter().copied()).expect("safe options exist");
        // 1.4 m/s stops in 1.96 m < 2.0 ✓; 2.0 m/s stops in 4.0 m ✗.
        assert!((best - 1.4).abs() < 1e-6);
    }

    #[test]
    fn all_unsafe_returns_none() {
        let mut e = Approach {
            distance_m: 0.01,
            max_decel: 0.1,
        };
        let speeds = [0.5f32, 1.0, 2.0];
        assert_eq!(select_best(&mut e, speeds.iter().copied()), None);
    }

    #[test]
    fn rover_flavored_evaluator_prefers_low_hazard() {
        /// Candidates are heading offsets; hazard rises off-corridor.
        struct Corridor;
        impl Evaluator for Corridor {
            type Candidate = f32;
            fn evaluate(&mut self, &offset: &f32) -> EvalOutcome {
                EvalOutcome {
                    score: -offset.abs(),
                    safe: offset.abs() < 1.0,
                }
            }
        }
        let mut e = Corridor;
        let offsets = [-2.0f32, -0.5, 0.0, 0.3, 1.5];
        let best = select_best(&mut e, offsets.iter().copied()).unwrap();
        assert!(best.abs() < 1e-6, "straight down the corridor wins");
    }
}
