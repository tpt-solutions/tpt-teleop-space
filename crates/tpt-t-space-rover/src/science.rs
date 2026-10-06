//! Multi-sol science sequencing with per-step sensor verification and
//! automatic rollback (spec §5.1 / §6 "Sample Collection").
//!
//! A sequence is the pre-loaded chain from the spec's data flow: deploy
//! arm → press drill → extract core → seal tube → stow in carousel. Each
//! step runs a bounded actuation profile and is *verified* by its torque
//! and position sensors; a failed verification rolls back the completed
//! prefix in reverse (safe-stow each actuator) and reports — no ground
//! loop needed to decide.

/// One sequence step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StepKind {
    /// Unstow and deploy the arm to the target site.
    DeployArm,
    /// Press the drill to the rock surface (preload via torque).
    PressDrill,
    /// Drill and extract the core (position + torque profile).
    ExtractCore,
    /// Seal the sample tube.
    SealTube,
    /// Stow the sealed tube in the carousel.
    StowCarousel,
}

impl StepKind {
    /// All steps in execution order.
    #[must_use]
    pub fn sample_collection() -> [StepKind; 5] {
        [
            StepKind::DeployArm,
            StepKind::PressDrill,
            StepKind::ExtractCore,
            StepKind::SealTube,
            StepKind::StowCarousel,
        ]
    }

    /// The safe-stow counterpart used during rollback (None for steps
    /// whose failure requires no undo action beyond stopping).
    #[must_use]
    pub fn rollback_action(self) -> StepKind {
        match self {
            StepKind::DeployArm | StepKind::PressDrill | StepKind::ExtractCore => {
                StepKind::DeployArm // retract arm to stow
            }
            StepKind::SealTube | StepKind::StowCarousel => StepKind::StowCarousel,
        }
    }
}

/// Actuator sensor snapshot at a verification point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensors {
    /// Joint torque, N·m (norm over the loaded joints).
    pub torque_nm: f32,
    /// Achieved position along the step profile, `0..=1`.
    pub position: f32,
    /// End-effector contact flag (drill/press steps).
    pub contact: bool,
}

/// Verification thresholds per step.
#[derive(Debug, Clone, Copy)]
pub struct StepProfile {
    /// Torque window `(min, max)`, N·m.
    pub torque_window: (f32, f32),
    /// Required final position, `0..=1`.
    pub position_target: f32,
    /// Position tolerance.
    pub position_tol: f32,
    /// Whether end-effector contact is required.
    pub requires_contact: bool,
}

impl StepProfile {
    /// Verification profiles for the sample-collection chain.
    #[must_use]
    pub fn sample_collection() -> [StepProfile; 5] {
        [
            StepProfile {
                torque_window: (0.5, 12.0),
                position_target: 1.0,
                position_tol: 0.02,
                requires_contact: false,
            },
            StepProfile {
                torque_window: (8.0, 40.0), // preload band
                position_target: 1.0,
                position_tol: 0.05,
                requires_contact: true,
            },
            StepProfile {
                torque_window: (5.0, 30.0),
                position_target: 1.0,
                position_tol: 0.03,
                requires_contact: true,
            },
            StepProfile {
                torque_window: (2.0, 15.0),
                position_target: 1.0,
                position_tol: 0.02,
                requires_contact: false,
            },
            StepProfile {
                torque_window: (1.0, 10.0),
                position_target: 1.0,
                position_tol: 0.02,
                requires_contact: false,
            },
        ]
    }
}

/// How a step (or sequence) concluded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StepOutcome {
    /// Verification passed.
    Verified,
    /// Torque outside the window (stall, overload, or broken contact).
    TorqueViolation,
    /// Position missed the target tolerance.
    PositionMiss,
    /// Required contact never made.
    NoContact,
}

/// State of the sequencer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SeqState {
    /// No sequence loaded.
    Idle,
    /// Step `step` of the plan is actuating.
    Executing {
        /// Index into the plan.
        step: usize,
    },
    /// Rolling back `step` completed steps after a failure.
    RollingBack {
        /// Index of the step that failed.
        step: usize,
    },
    /// All steps verified.
    Complete,
    /// Rollback finished after a failure at `failed_at`.
    Failed {
        /// Index of the step that failed.
        failed_at: usize,
        /// Why it failed.
        outcome: StepOutcome,
    },
}

/// The science sequencer over the sample-collection chain.
pub struct Sequencer {
    kinds: [StepKind; 5],
    profiles: [StepProfile; 5],
    state: SeqState,
    /// Completed (verified) steps, in order.
    completed: [bool; 5],
    /// Rollback actions still to run.
    rollback: [bool; 5],
    /// Outcome of the step that failed (reported at rollback completion).
    failed_outcome: Option<StepOutcome>,
    /// Sol tags for the audit log (multi-sol sequences chain here).
    sol: u32,
    /// Verification points consumed (telemetry).
    pub steps_executed: u32,
}

impl Sequencer {
    /// Load the sample-collection sequence for `sol`.
    #[must_use]
    pub fn sample_collection(sol: u32) -> Self {
        Self {
            kinds: StepKind::sample_collection(),
            profiles: StepProfile::sample_collection(),
            state: SeqState::Idle,
            completed: [false; 5],
            rollback: [false; 5],
            failed_outcome: None,
            sol,
            steps_executed: 0,
        }
    }

    /// Begin execution.
    pub fn start(&mut self) -> SeqState {
        self.state = SeqState::Executing { step: 0 };
        self.state
    }

    /// Current state.
    #[must_use]
    pub fn state(&self) -> SeqState {
        self.state
    }

    /// Sol tag.
    #[must_use]
    pub fn sol(&self) -> u32 {
        self.sol
    }

    /// Step kinds in execution order (telemetry / audit log).
    #[must_use]
    pub fn kinds(&self) -> [StepKind; 5] {
        self.kinds
    }

    /// Feed one actuation-verification point for the current step. This
    /// is the sequencer's entire input surface: the actuator layer calls
    /// it when the step's profile completes.
    pub fn verify(&mut self, sensors: &Sensors) -> SeqState {
        let SeqState::Executing { step } = self.state else {
            return self.state;
        };
        let profile = &self.profiles[step];
        let outcome = if sensors.torque_nm < profile.torque_window.0
            || sensors.torque_nm > profile.torque_window.1
        {
            StepOutcome::TorqueViolation
        } else if (sensors.position - profile.position_target).abs() > profile.position_tol {
            StepOutcome::PositionMiss
        } else if profile.requires_contact && !sensors.contact {
            StepOutcome::NoContact
        } else {
            StepOutcome::Verified
        };
        self.steps_executed += 1;
        if outcome == StepOutcome::Verified {
            self.completed[step] = true;
            if step + 1 == 5 {
                self.state = SeqState::Complete;
            } else {
                self.state = SeqState::Executing { step: step + 1 };
            }
            return self.state;
        }
        // Failure: arm the rollback over the completed prefix.
        for i in 0..step {
            self.rollback[i] = self.completed[i];
        }
        self.rollback[step] = true; // the failed step itself safe-stows
        self.state = SeqState::RollingBack { step };
        self.failed_outcome = Some(outcome);
        self.state
    }

    /// Feed one rollback completion (each armed rollback action done).
    pub fn verify_rollback(&mut self) -> SeqState {
        let SeqState::RollingBack { step } = self.state else {
            return self.state;
        };
        // Clear the deepest armed action.
        for i in (0..=step).rev() {
            if self.rollback[i] {
                self.rollback[i] = false;
                break;
            }
        }
        if self.rollback.iter().any(|armed| *armed) {
            return self.state;
        }
        let outcome = self.failed_outcome.take().unwrap_or(StepOutcome::NoContact);
        self.state = SeqState::Failed {
            failed_at: step,
            outcome,
        };
        self.state
    }

    /// Whether step `i` verified (for the telemetry log).
    #[must_use]
    pub fn verified(&self, i: usize) -> bool {
        self.completed.get(i).copied().unwrap_or(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn good() -> Sensors {
        Sensors {
            torque_nm: 6.0,
            position: 1.0,
            contact: false,
        }
    }

    fn verify_ok(seq: &mut Sequencer, contact: bool, torque: f32) -> SeqState {
        let s = Sensors {
            torque_nm: torque,
            position: 1.0,
            contact,
        };
        seq.verify(&s)
    }

    #[test]
    fn full_sample_collection_verifies() {
        let mut seq = Sequencer::sample_collection(1047);
        seq.start();
        assert_eq!(
            verify_ok(&mut seq, false, 6.0),
            SeqState::Executing { step: 1 }
        );
        assert_eq!(
            verify_ok(&mut seq, true, 20.0),
            SeqState::Executing { step: 2 }
        );
        assert_eq!(
            verify_ok(&mut seq, true, 12.0),
            SeqState::Executing { step: 3 }
        );
        assert_eq!(
            verify_ok(&mut seq, false, 6.0),
            SeqState::Executing { step: 4 }
        );
        assert_eq!(verify_ok(&mut seq, false, 4.0), SeqState::Complete);
        for i in 0..5 {
            assert!(seq.verified(i));
        }
        assert_eq!(seq.sol(), 1047);
    }

    #[test]
    fn drill_stall_rolls_back_everything() {
        let mut seq = Sequencer::sample_collection(1048);
        seq.start();
        verify_ok(&mut seq, false, 6.0); // arm deployed
        verify_ok(&mut seq, true, 20.0); // pressed
        // Extract stalls: torque collapses (bit breaks, no load).
        let state = verify_ok(&mut seq, true, 0.5);
        let SeqState::RollingBack { step } = state else {
            panic!("expected rollback, got {state:?}");
        };
        assert_eq!(step, 2, "failed at extract-core");
        // Rollback actions complete in order (three armed: the two
        // verified steps plus the failed one's safe-stow). The third
        // completion latches the failure.
        assert!(matches!(
            seq.verify_rollback(),
            SeqState::RollingBack { .. }
        ));
        assert!(matches!(
            seq.verify_rollback(),
            SeqState::RollingBack { .. }
        ));
        assert!(matches!(
            seq.verify_rollback(),
            SeqState::Failed {
                failed_at: 2,
                outcome: StepOutcome::TorqueViolation
            }
        ));
        // Nothing reports verified after rollback.
        assert!(!seq.verified(2));
    }

    #[test]
    fn no_contact_at_press_fails_fast() {
        let mut seq = Sequencer::sample_collection(1049);
        seq.start();
        verify_ok(&mut seq, false, 6.0);
        let state = verify_ok(&mut seq, false, 20.0); // pressed thin air
        assert!(matches!(state, SeqState::RollingBack { step: 1 }));
    }

    #[test]
    fn position_miss_is_detected() {
        let mut seq = Sequencer::sample_collection(1050);
        seq.start();
        let state = seq.verify(&Sensors {
            torque_nm: 6.0,
            position: 0.9, // 2% tolerance missed
            contact: false,
        });
        assert!(matches!(state, SeqState::RollingBack { step: 0 }));
    }

    #[test]
    fn verify_outside_execution_is_inert() {
        let mut seq = Sequencer::sample_collection(1051);
        // Idle: verify is a no-op.
        assert_eq!(seq.verify(&good()), SeqState::Idle);
        seq.start();
        verify_ok(&mut seq, false, 6.0);
        verify_ok(&mut seq, true, 20.0);
        verify_ok(&mut seq, true, 12.0);
        verify_ok(&mut seq, false, 6.0);
        verify_ok(&mut seq, false, 4.0); // complete
        assert_eq!(seq.verify(&good()), SeqState::Complete);
    }
}
