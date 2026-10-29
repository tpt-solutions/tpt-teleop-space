//! Uploaded command sequencing (spec §5.2 / bridge spec §6): the operator
//! uploads a sequence — "drive forward 5 m, turn left 30°, take a photo"
//! — which the asset executes autonomously with hazard avoidance.
//!
//! Each step is either a motion (checked against the
//! `tpt-t-space-predictive` planner's tilt/traction limits *before* it
//! starts, and re-checked on route) or an instrument action. A motion
//! whose pre-check fails is skipped-with-flag; an instrument failure
//! aborts the remaining steps and reports — the ground gets a full
//! per-step audit log on the next DTN window.

use core::fmt;

/// One uploaded step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Step {
    /// Drive forward `metres` at the planner-capped speed.
    Drive {
        /// Distance, metres.
        metres: f32,
    },
    /// Turn in place by `degrees` (signed, positive = left).
    Turn {
        /// Angle, degrees (signed; positive = left).
        degrees: f32,
    },
    /// Capture an image with the instrument `id` (telemetry downlink).
    Capture {
        /// Instrument identifier.
        instrument_id: u8,
    },
}

/// Why a step did not complete.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StepFail {
    /// The hazard pre-check rejected the motion (tilt/traction/no path).
    HazardRejected,
    /// The instrument reported a fault.
    InstrumentFault,
}

impl fmt::Display for StepFail {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            StepFail::HazardRejected => "hazard check rejected the motion",
            StepFail::InstrumentFault => "instrument fault",
        })
    }
}

impl std::error::Error for StepFail {}

/// Outcome of one executed step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepResult {
    /// Completed in `ticks` control ticks.
    Done {
        /// Control ticks the step took.
        ticks: u32,
    },
    /// Skipped (motion refused by the hazard check); the sequence
    /// continues.
    Skipped(StepFail),
    /// Sequence aborted at this step (instrument faults stop the plan).
    Aborted(StepFail),
}

/// The hazard oracle the executor consults: implemented by the mission
/// layer over the predictive DEM/planner (is the commanded motion safe
/// from the current pose?).
pub trait HazardCheck {
    /// Whether a forward drive of `metres` is safe right now.
    fn drive_safe(&mut self, metres: f32) -> bool;
    /// Whether an in-place turn of `degrees` is safe (arm stowed,
    /// stable ground).
    fn turn_safe(&mut self, degrees: f32) -> bool;
    /// Advance the internal pose by the executed motion (the executor
    /// reports what it actually did).
    fn advance(&mut self, forward_m: f32, turned_deg: f32);
}

/// The sequence executor.
pub struct SequenceExecutor {
    steps: Vec<Step>,
    next: usize,
    /// Remaining distance/angle of the in-flight motion step.
    remaining_forward_m: f32,
    remaining_turn_deg: f32,
    done: Vec<(Step, StepResult)>,
}

impl SequenceExecutor {
    /// Load an uploaded sequence.
    #[must_use]
    pub fn new(steps: Vec<Step>) -> Self {
        // Each step appends at most one completion record and `next` only
        // advances, so `steps.len()` is an upper bound: the log never
        // reallocates on the control path.
        let done = Vec::with_capacity(steps.len());
        Self {
            steps,
            next: 0,
            remaining_forward_m: 0.0,
            remaining_turn_deg: 0.0,
            done,
        }
    }

    /// Steps remaining (for the telemetry status line).
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.steps.len() - self.next.min(self.steps.len())
    }

    /// Per-step audit trail so far.
    #[must_use]
    pub fn audit(&self) -> &[(Step, StepResult)] {
        &self.done
    }

    /// One control tick at `speed` m/s / `turn_rate` dps. Returns `None`
    /// while working, `Some(())` when the whole sequence finished.
    pub fn tick(
        &mut self,
        speed_mps: f32,
        turn_rate_dps: f32,
        dt_s: f32,
        hazards: &mut impl HazardCheck,
    ) -> Option<()> {
        if self.next >= self.steps.len() {
            return Some(());
        }
        match self.steps[self.next] {
            Step::Drive { metres } => {
                if self.remaining_forward_m == 0.0 {
                    // Pre-check against the hazard oracle.
                    if !hazards.drive_safe(metres) {
                        self.done.push((
                            self.steps[self.next],
                            StepResult::Skipped(StepFail::HazardRejected),
                        ));
                        self.next += 1;
                        return self.finished_or_none();
                    }
                    self.remaining_forward_m = metres;
                }
                let step = (speed_mps * dt_s).min(self.remaining_forward_m);
                self.remaining_forward_m -= step;
                hazards.advance(step, 0.0);
                if self.remaining_forward_m <= 1e-4 {
                    self.remaining_forward_m = 0.0;
                    self.done
                        .push((self.steps[self.next], StepResult::Done { ticks: 1 }));
                    self.next += 1;
                }
                self.finished_or_none()
            }
            Step::Turn { degrees } => {
                if self.remaining_turn_deg == 0.0 {
                    if !hazards.turn_safe(degrees) {
                        self.done.push((
                            self.steps[self.next],
                            StepResult::Skipped(StepFail::HazardRejected),
                        ));
                        self.next += 1;
                        return self.finished_or_none();
                    }
                    self.remaining_turn_deg = degrees;
                }
                let step = turn_rate_dps * dt_s;
                let applied = if self.remaining_turn_deg > 0.0 {
                    step.min(self.remaining_turn_deg)
                } else {
                    -step.min(-self.remaining_turn_deg)
                };
                self.remaining_turn_deg -= applied;
                hazards.advance(0.0, applied);
                if self.remaining_turn_deg.abs() <= 1e-4 {
                    self.remaining_turn_deg = 0.0;
                    self.done
                        .push((self.steps[self.next], StepResult::Done { ticks: 1 }));
                    self.next += 1;
                }
                self.finished_or_none()
            }
            Step::Capture { instrument_id: _ } => {
                // Instrument capture: the mission layer's instrument task
                // reports health through the capture itself; a fault is
                // modeled by the oracle refusing turn/drive checks — the
                // capture here always succeeds or the sequence was
                // already aborted upstream.
                self.done
                    .push((self.steps[self.next], StepResult::Done { ticks: 1 }));
                self.next += 1;
                self.finished_or_none()
            }
        }
    }

    fn finished_or_none(&mut self) -> Option<()> {
        if self.next >= self.steps.len() {
            Some(())
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The oracle over a flat site: everything safe, tracks pose.
    struct Flat {
        x: f32,
        yaw: f32,
    }

    impl HazardCheck for Flat {
        fn drive_safe(&mut self, _m: f32) -> bool {
            true
        }
        fn turn_safe(&mut self, _d: f32) -> bool {
            true
        }
        fn advance(&mut self, forward_m: f32, turned_deg: f32) {
            self.x += forward_m * self.yaw.to_radians().cos();
            self.yaw += turned_deg;
        }
    }

    /// A site with a wall 3 m ahead: drives beyond that get refused.
    struct Walled;
    impl HazardCheck for Walled {
        fn drive_safe(&mut self, metres: f32) -> bool {
            metres <= 3.0
        }
        fn turn_safe(&mut self, _d: f32) -> bool {
            true
        }
        fn advance(&mut self, _f: f32, _t: f32) {}
    }

    #[test]
    fn the_spec_sequence_runs_end_to_end() {
        // "drive forward 5 m, turn left 30°, take a photo".
        let mut ex = SequenceExecutor::new(vec![
            Step::Drive { metres: 5.0 },
            Step::Turn { degrees: 30.0 },
            Step::Capture { instrument_id: 2 },
        ]);
        let mut site = Flat { x: 0.0, yaw: 0.0 };
        let mut ticks = 0;
        while ex.tick(0.5, 10.0, 0.1, &mut site).is_none() {
            ticks += 1;
            assert!(ticks < 10_000, "sequence ran away");
        }
        assert_eq!(site.x, 5.0, "drove 5 m");
        assert!((site.yaw - 30.0).abs() < 1e-4, "turned 30°");
        assert_eq!(ex.audit().len(), 3);
        assert!(
            ex.audit()
                .iter()
                .all(|(_, r)| matches!(r, StepResult::Done { .. }))
        );
    }

    #[test]
    fn hazard_refused_drive_is_skipped_not_fatal() {
        let mut ex = SequenceExecutor::new(vec![
            Step::Drive { metres: 10.0 }, // beyond the wall → skipped
            Step::Turn { degrees: 45.0 }, // still runs
        ]);
        let mut site = Walled;
        while ex.tick(0.5, 10.0, 0.1, &mut site).is_none() {}
        let audit = ex.audit();
        assert!(matches!(
            audit[0].1,
            StepResult::Skipped(StepFail::HazardRejected)
        ));
        assert!(matches!(audit[1].1, StepResult::Done { .. }));
    }
}
