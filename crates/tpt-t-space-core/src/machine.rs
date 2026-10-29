//! Mission state machine for spacecraft (spec §3, `tpt-t-space-core`).
//!
//! Models the high-level mission lifecycle of a single asset — rover,
//! orbiter, or deep-space probe — as the ordered Cruise → Approach →
//! Operations → Safe Mode spine from the design doc. Transition legality is
//! encoded by [`MissionState::next`]; illegal transitions are rejected rather
//! than panicked, so the caller can route them through fault handling.
//!
//! Safe Mode is the survival state (spec §4.5): entered atomically from any
//! state on a thermal or power limit breach, it exits only through a
//! deliberate recovery back to Cruise. The universal safety state machine
//! from `tpt-t-domain-bridge` (`AUTONOMOUS → REQUESTING_TELEOP →
//! TELEOP_ACTIVE → RETURNING_TO_AUTONOMY` plus `EMERGENCY_STOP`) composes
//! *on top* of this machine: this crate models the mission lifecycle, the
//! bridge machine gates who is commanding.

use core::fmt;

/// High-level mission state of a spacecraft.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MissionState {
    /// Long transfer between bodies: ultra-low-power hibernation with
    /// periodic wake-ups for trajectory correction maneuvers.
    Cruise,
    /// Closing on the target (orbit insertion, landing ellipse, or
    /// rendezvous approach corridor).
    Approach,
    /// Executing the payload mission (surface operations, science
    /// sequencing, station-keeping, servicing).
    Operations,
    /// Survival state: non-essential payloads powered down, sun-pointing
    /// attitude, low-rate DTN beacon (see [`crate::safe_mode`]).
    SafeMode,
    /// Fault / degraded: control handed to the fault-management reflexes
    /// pending triage into Safe Mode or recovery.
    Fault,
    /// Teleoperation engaged (operator commanding over the DTN link via
    /// `tpt-t-space-teleop`).
    Teleop,
}

impl MissionState {
    /// Returns `Some(target)` if a transition from `self` to `target` is
    /// legal, otherwise `None`.
    ///
    /// The legal graph encodes the spec's Cruise → Approach → Operations
    /// spine plus real-mission shortcuts: an approach abort returns to
    /// Cruise, Operations may cycle back through Approach for a new target,
    /// any state may enter Safe Mode (the survival trigger is unconditional),
    /// and Safe Mode recovers only through a deliberate return to Cruise.
    pub fn next(self, target: MissionState) -> Option<MissionState> {
        if self == target {
            return Some(self);
        }
        use MissionState::*;
        let ok = matches!(
            (self, target),
            (Cruise, Approach)
                | (Cruise, SafeMode) // hibernation anomaly
                | (Approach, Operations)
                | (Approach, Cruise) // aborted approach / next target
                | (Operations, Approach) // re-approach for a new site
                | (Operations, Cruise) // mission leg complete
                | (Operations, SafeMode) // dust storm, thermal limit, …
                | (Approach, SafeMode)
                | (_, Fault)          // any state may fault
                | (Fault, SafeMode)   // triage: protect the asset
                | (Fault, Cruise)     // transient: recover in place
                | (SafeMode, Cruise)  // storm cleared / fault cleared
                | (_, Teleop)         // operator assistance from any state
                | (Teleop, Cruise)
                | (Teleop, Approach)
                | (Teleop, Operations)
        );
        ok.then_some(target)
    }

    /// Whether the asset is in a survival-critical state (main mission
    /// compute shed to minimums).
    pub fn surviving(self) -> bool {
        matches!(self, MissionState::SafeMode | MissionState::Fault)
    }
}

impl fmt::Display for MissionState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            MissionState::Cruise => "CRUISE",
            MissionState::Approach => "APPROACH",
            MissionState::Operations => "OPERATIONS",
            MissionState::SafeMode => "SAFE_MODE",
            MissionState::Fault => "FAULT",
            MissionState::Teleop => "TELEOP",
        };
        f.write_str(name)
    }
}

/// Transition policy for a state type.
pub trait Transition {
    /// The state space this policy operates over.
    type State;
    /// Return the next state, or `None` if the transition is illegal.
    fn next(self, target: Self::State) -> Option<Self::State>;
}

impl Transition for MissionState {
    type State = MissionState;
    fn next(self, target: Self::State) -> Option<Self::State> {
        MissionState::next(self, target)
    }
}

/// A minimal generic state machine driven by an implementor of [`Transition`].
#[derive(Debug, Clone, Copy)]
pub struct StateMachine<S> {
    state: S,
}

impl<S: Copy + PartialEq + fmt::Debug> StateMachine<S> {
    /// Create a machine in `initial` state.
    pub fn new(initial: S) -> Self {
        Self { state: initial }
    }

    /// Current state.
    pub fn state(&self) -> S {
        self.state
    }

    /// Attempt a transition via [`Transition::next`].
    pub fn transition(&mut self, target: S) -> Result<(), TransitionError<S>>
    where
        S: Transition<State = S>,
    {
        match self.state.next(target) {
            Some(s) => {
                self.state = s;
                Ok(())
            }
            None => Err(TransitionError {
                from: self.state,
                to: target,
            }),
        }
    }
}

/// Error returned when a transition is rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionError<S> {
    /// State the machine was in.
    pub from: S,
    /// State that was requested.
    pub to: S,
}

impl<S: fmt::Debug> fmt::Display for TransitionError<S> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "illegal transition {:?} -> {:?}", self.from, self.to)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spec_spine_is_legal() {
        // The spec's canonical mission: Cruise → Approach → Operations,
        // then Safe Mode on the dust-storm trigger, recovery to Cruise.
        let mut m = StateMachine::new(MissionState::Cruise);
        m.transition(MissionState::Approach).unwrap();
        m.transition(MissionState::Operations).unwrap();
        m.transition(MissionState::SafeMode).unwrap();
        m.transition(MissionState::Cruise).unwrap();
        assert_eq!(m.state(), MissionState::Cruise);
    }

    #[test]
    fn approach_abort_and_reapproach() {
        let mut m = StateMachine::new(MissionState::Cruise);
        m.transition(MissionState::Approach).unwrap();
        // Abort back to cruise…
        m.transition(MissionState::Cruise).unwrap();
        // …then approach again and enter operations.
        m.transition(MissionState::Approach).unwrap();
        m.transition(MissionState::Operations).unwrap();
        assert_eq!(m.state(), MissionState::Operations);
        // Operations may also return to cruise between mission legs.
        m.transition(MissionState::Cruise).unwrap();
    }

    // REQ: FSM-001
    #[test]
    fn illegal_jumps_rejected() {
        let mut m = StateMachine::new(MissionState::Cruise);
        // Cannot operate without approaching.
        assert!(m.transition(MissionState::Operations).is_err());
        assert_eq!(m.state(), MissionState::Cruise);
        // Safe Mode cannot jump straight into work: recovery passes Cruise.
        m.transition(MissionState::SafeMode).unwrap();
        assert!(m.transition(MissionState::Operations).is_err());
        assert!(m.transition(MissionState::Approach).is_err());
        m.transition(MissionState::Cruise).unwrap();
    }

    #[test]
    fn fault_triages_to_safe_mode_or_recovers() {
        let mut m = StateMachine::new(MissionState::Approach);
        assert!(m.transition(MissionState::Fault).is_ok());
        // Fault cannot go straight to work.
        assert!(m.transition(MissionState::Operations).is_err());
        m.transition(MissionState::SafeMode).unwrap();
        m.transition(MissionState::Cruise).unwrap();

        // A transient fault recovers in place via Cruise.
        let mut m2 = StateMachine::new(MissionState::Cruise);
        m2.transition(MissionState::Fault).unwrap();
        m2.transition(MissionState::Cruise).unwrap();
        assert_eq!(m2.state(), MissionState::Cruise);
    }

    #[test]
    fn teleop_from_anywhere_and_back() {
        let mut m = StateMachine::new(MissionState::Operations);
        assert!(m.transition(MissionState::Teleop).is_ok());
        assert!(m.transition(MissionState::Operations).is_ok());
        assert!(m.transition(MissionState::Teleop).is_ok());
        assert!(m.transition(MissionState::Approach).is_ok());
    }

    #[test]
    fn same_state_is_idempotent() {
        let mut m = StateMachine::new(MissionState::Cruise);
        assert!(m.transition(MissionState::Cruise).is_ok());
        assert_eq!(m.state(), MissionState::Cruise);
    }

    #[test]
    fn surviving_flag() {
        assert!(MissionState::SafeMode.surviving());
        assert!(MissionState::Fault.surviving());
        assert!(!MissionState::Cruise.surviving());
        assert!(!MissionState::Operations.surviving());
    }

    #[test]
    fn display_uses_spec_names() {
        assert_eq!(MissionState::Cruise.to_string(), "CRUISE");
        assert_eq!(MissionState::SafeMode.to_string(), "SAFE_MODE");
    }
}
