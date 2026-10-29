//! Sequence linter and dry run (Phase 20): check an uplink sequence on the
//! ground, before it is sent.
//!
//! * [`lint`] checks the sequence alone: step count, per-step limits, and
//!   non-finite values. It does not move anything.
//! * [`dry_run`] also replays the sequence through the same
//!   [`HazardCheck`] oracle the rover uses, advancing the oracle's pose as
//!   each step would. A step the oracle refuses is reported with its index,
//!   and the replay continues, as the executor does.

use crate::sequencing::{HazardCheck, Step};

/// Ground-side limits a sequence must respect.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SequenceLimits {
    /// Longest single drive, metres.
    pub max_drive_m: f32,
    /// Largest single turn, degrees (either direction).
    pub max_turn_deg: f32,
    /// Uplink budget: most steps a sequence may carry.
    pub max_steps: usize,
}

impl SequenceLimits {
    /// Defaults for a surface rover uplink: 10 m drives, half-turns, 64 steps.
    #[must_use]
    pub fn rover_default() -> Self {
        Self {
            max_drive_m: 10.0,
            max_turn_deg: 180.0,
            max_steps: 64,
        }
    }
}

/// One problem found in a sequence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Issue {
    /// The sequence has no steps.
    Empty,
    /// More steps than the uplink budget allows.
    TooManySteps {
        /// Steps in the sequence.
        count: usize,
        /// The budget.
        max: usize,
    },
    /// A drive is zero, negative, or not a number.
    BadDrive {
        /// Zero-based step index.
        index: usize,
        /// The commanded distance.
        metres: f32,
    },
    /// A drive exceeds the per-step limit.
    DriveTooLong {
        /// Zero-based step index.
        index: usize,
        /// The commanded distance.
        metres: f32,
        /// The limit.
        max: f32,
    },
    /// A turn is not a number or exceeds the per-step limit.
    BadTurn {
        /// Zero-based step index.
        index: usize,
        /// The commanded angle.
        degrees: f32,
        /// The limit, degrees.
        max: f32,
    },
    /// Dry run only: the hazard oracle refused this step from the pose it
    /// would have been reached at.
    HazardRefused {
        /// Zero-based step index.
        index: usize,
        /// The refused step.
        step: Step,
    },
}

/// The outcome of linting a sequence.
#[derive(Debug, Clone, PartialEq)]
pub struct Report {
    /// Every problem found, in step order.
    pub issues: Vec<Issue>,
    /// Steps examined.
    pub steps: usize,
    /// Total commanded drive, metres (finite drives only).
    pub drive_m: f32,
}

impl Report {
    /// Whether the sequence passed every check.
    #[must_use]
    pub fn is_clean(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Check a sequence against `limits` without moving anything.
#[must_use]
pub fn lint(steps: &[Step], limits: &SequenceLimits) -> Report {
    let mut issues = Vec::new();
    if steps.is_empty() {
        issues.push(Issue::Empty);
    }
    if steps.len() > limits.max_steps {
        issues.push(Issue::TooManySteps {
            count: steps.len(),
            max: limits.max_steps,
        });
    }
    let mut drive_m = 0.0_f32;
    for (index, step) in steps.iter().enumerate() {
        match *step {
            Step::Drive { metres } => {
                if !metres.is_finite() || metres <= 0.0 {
                    issues.push(Issue::BadDrive { index, metres });
                } else if metres > limits.max_drive_m {
                    issues.push(Issue::DriveTooLong {
                        index,
                        metres,
                        max: limits.max_drive_m,
                    });
                } else {
                    drive_m += metres;
                }
            }
            Step::Turn { degrees } => {
                if !degrees.is_finite() || degrees.abs() > limits.max_turn_deg {
                    issues.push(Issue::BadTurn {
                        index,
                        degrees,
                        max: limits.max_turn_deg,
                    });
                }
            }
            Step::Capture { .. } => {}
        }
    }
    Report {
        issues,
        steps: steps.len(),
        drive_m,
    }
}

/// [`lint`], then replay the sequence through `oracle`.
///
/// The oracle sees each accepted motion through `advance`, so its pose
/// follows the sequence exactly as the rover's would. Steps with static
/// problems are not sent to the oracle.
#[must_use]
pub fn dry_run(steps: &[Step], limits: &SequenceLimits, oracle: &mut impl HazardCheck) -> Report {
    let mut report = lint(steps, limits);
    for (index, step) in steps.iter().enumerate() {
        let static_ok = !report.issues.iter().any(|i| match *i {
            Issue::BadDrive { index: at, .. }
            | Issue::DriveTooLong { index: at, .. }
            | Issue::BadTurn { index: at, .. } => at == index,
            _ => false,
        });
        if !static_ok {
            continue;
        }
        match *step {
            Step::Drive { metres } => {
                if oracle.drive_safe(metres) {
                    oracle.advance(metres, 0.0);
                } else {
                    report
                        .issues
                        .push(Issue::HazardRefused { index, step: *step });
                }
            }
            Step::Turn { degrees } => {
                if oracle.turn_safe(degrees) {
                    oracle.advance(0.0, degrees);
                } else {
                    report
                        .issues
                        .push(Issue::HazardRefused { index, step: *step });
                }
            }
            Step::Capture { .. } => {}
        }
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Refuses drives past `limit_m` from the origin and any turn.
    struct Walls {
        limit_m: f32,
        pose_m: f32,
        yaw_deg: f32,
    }

    impl HazardCheck for Walls {
        fn drive_safe(&mut self, metres: f32) -> bool {
            self.pose_m + metres <= self.limit_m
        }
        fn turn_safe(&mut self, _degrees: f32) -> bool {
            true
        }
        fn advance(&mut self, forward_m: f32, turned_deg: f32) {
            self.pose_m += forward_m;
            self.yaw_deg += turned_deg;
        }
    }

    fn limits() -> SequenceLimits {
        SequenceLimits::rover_default()
    }

    #[test]
    fn a_clean_sequence_passes() {
        let seq = [
            Step::Drive { metres: 5.0 },
            Step::Turn { degrees: 30.0 },
            Step::Capture { instrument_id: 1 },
        ];
        let r = lint(&seq, &limits());
        assert!(r.is_clean(), "{:?}", r.issues);
        assert_eq!(r.steps, 3);
        assert!((r.drive_m - 5.0).abs() < 1e-6);
    }

    #[test]
    fn empty_and_oversized_sequences_are_flagged() {
        assert_eq!(lint(&[], &limits()).issues, vec![Issue::Empty]);
        let long = vec![Step::Capture { instrument_id: 0 }; 65];
        assert_eq!(
            lint(&long, &limits()).issues,
            vec![Issue::TooManySteps { count: 65, max: 64 }]
        );
    }

    #[test]
    fn bad_and_oversized_motion_is_flagged_with_its_index() {
        let seq = [
            Step::Drive { metres: 0.0 },
            Step::Drive { metres: 12.0 },
            Step::Turn { degrees: 270.0 },
            Step::Turn { degrees: f32::NAN },
        ];
        let issues = lint(&seq, &limits()).issues;
        assert_eq!(issues.len(), 4);
        assert!(matches!(issues[0], Issue::BadDrive { index: 0, .. }));
        assert!(matches!(issues[1], Issue::DriveTooLong { index: 1, .. }));
        assert!(matches!(issues[2], Issue::BadTurn { index: 2, .. }));
        assert!(matches!(issues[3], Issue::BadTurn { index: 3, .. }));
    }

    #[test]
    fn dry_run_reports_the_refused_step_and_keeps_going() {
        let seq = [
            Step::Drive { metres: 4.0 },
            Step::Drive { metres: 4.0 },
            Step::Drive { metres: 2.0 },
        ];
        let mut oracle = Walls {
            limit_m: 6.0,
            pose_m: 0.0,
            yaw_deg: 0.0,
        };
        let r = dry_run(&seq, &limits(), &mut oracle);
        assert_eq!(
            r.issues,
            vec![Issue::HazardRefused {
                index: 1,
                step: Step::Drive { metres: 4.0 }
            }]
        );
        // The refused drive did not move the pose; the 2 m drive did.
        assert!((oracle.pose_m - 6.0).abs() < 1e-6);
    }

    #[test]
    fn dry_run_does_not_consult_the_oracle_for_bad_steps() {
        struct Count(u32);
        impl HazardCheck for Count {
            fn drive_safe(&mut self, _m: f32) -> bool {
                self.0 += 1;
                true
            }
            fn turn_safe(&mut self, _d: f32) -> bool {
                self.0 += 1;
                true
            }
            fn advance(&mut self, _f: f32, _t: f32) {}
        }
        let mut count = Count(0);
        let r = dry_run(&[Step::Drive { metres: -1.0 }], &limits(), &mut count);
        assert_eq!(count.0, 0);
        assert_eq!(r.issues.len(), 1);
    }
}
