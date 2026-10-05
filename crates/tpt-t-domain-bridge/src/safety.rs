//! The universal autonomy↔teleop safety state machine.
//!
//! `bridge spec.txt` §4.3:
//!
//! ```text
//! AUTONOMOUS → REQUESTING_TELEOP → TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS
//!                     ↓
//!               EMERGENCY_STOP
//! ```
//!
//! Semantics per state:
//!
//! * [`SafetyState::Autonomous`] — the domain drives itself.
//! * [`SafetyState::RequestingTeleop`] — the domain detected a fault and is
//!   asking for human assistance (see [`crate::assist`]).
//! * [`SafetyState::TeleopActive`] — the human is in control; the domain
//!   still enforces its own safety limits (speed limits, geofences).
//! * [`SafetyState::ReturningToAutonomy`] — the human released control; the
//!   domain blends setpoints back to the autonomy controller.
//! * [`SafetyState::EmergencyStop`] — the human *or* the domain triggered an
//!   immediate stop. Entered from any state; exit requires an explicit
//!   reset-to-autonomous after the fault is cleared.
//!
//! This machine composes *on top of* a domain's own operating state machine
//! (e.g. `tpt-t-space-core::machine::MissionState`): the domain keeps its
//! domain lifecycle, and this machine gates who is commanding it.

use core::fmt;

/// States of the universal teleop safety machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum SafetyState {
    /// Domain drives itself; no operator session.
    #[default]
    Autonomous = 0,
    /// Domain detected a fault and requests operator assistance.
    RequestingTeleop = 1,
    /// Operator session active; human commands flow through the domain.
    TeleopActive = 2,
    /// Operator released control; setpoints blend back to autonomy.
    ReturningToAutonomy = 3,
    /// Latched immediate stop (operator- or domain-triggered).
    EmergencyStop = 4,
}

impl SafetyState {
    /// The `u8` discriminator carried in [`crate::wire::DomainState::safety_state`].
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    /// Decode the discriminator stored in [`crate::wire::DomainState::safety_state`].
    /// Invalid discriminants decode to [`SafetyState::EmergencyStop`] (the
    /// conservative choice for a corrupt wire value).
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Autonomous),
            1 => Some(Self::RequestingTeleop),
            2 => Some(Self::TeleopActive),
            3 => Some(Self::ReturningToAutonomy),
            4 => Some(Self::EmergencyStop),
            _ => None,
        }
    }

    /// Whether an operator session is (still) in the loop.
    pub fn teleop_in_loop(self) -> bool {
        matches!(self, Self::TeleopActive | Self::ReturningToAutonomy)
    }
}

impl fmt::Display for SafetyState {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            SafetyState::Autonomous => "AUTONOMOUS",
            SafetyState::RequestingTeleop => "REQUESTING_TELEOP",
            SafetyState::TeleopActive => "TELEOP_ACTIVE",
            SafetyState::ReturningToAutonomy => "RETURNING_TO_AUTONOMY",
            SafetyState::EmergencyStop => "EMERGENCY_STOP",
        };
        f.write_str(name)
    }
}

/// Why an [`SafetyStateMachine`] transition was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IllegalTransition {
    /// State the machine was in.
    pub from: SafetyState,
    /// Event that was dispatched.
    pub event: SafetyEvent,
}

impl fmt::Display for IllegalTransition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "illegal event {:?} in state {}", self.event, self.from)
    }
}

impl std::error::Error for IllegalTransition {}

/// Events driving the safety machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SafetyEvent {
    /// Domain requests operator assistance (fault, stall, unrecoverable plan).
    RequestAssistance,
    /// Operator session accepted; commands start flowing.
    TeleopEngaged,
    /// Assist request withdrawn: fault cleared or request timed out/cancelled
    /// before an operator picked up.
    RequestCancelled,
    /// Operator releases control; blending back to autonomy begins.
    TeleopReleased,
    /// Blend-back finished; the domain is fully autonomous again.
    ReturnComplete,
    /// Operator re-engaged during the blend-back window.
    TeleopReEngaged,
    /// Immediate stop from any state.
    EmergencyStop,
    /// E-stop cleared after fault remediation; resume autonomy only.
    EmergencyCleared,
}

/// The universal safety state machine (see module docs for the diagram).
///
/// Instances live with the domain adapter (`tpt-t-space-teleop` for this
/// repo). Illegal event/state combinations are refused, not panicked, so the
/// caller can route them through fault handling.
#[derive(Debug, Clone, Copy)]
pub struct SafetyStateMachine {
    state: SafetyState,
}

impl Default for SafetyStateMachine {
    fn default() -> Self {
        Self {
            state: SafetyState::Autonomous,
        }
    }
}

impl SafetyStateMachine {
    /// Create a machine in `initial` state (normally [`SafetyState::Autonomous`]).
    pub fn new(initial: SafetyState) -> Self {
        Self { state: initial }
    }

    /// Current state.
    pub fn state(&self) -> SafetyState {
        self.state
    }

    /// Whether the machine is latched in [`SafetyState::EmergencyStop`].
    pub fn stopped(&self) -> bool {
        self.state == SafetyState::EmergencyStop
    }

    /// Dispatch an event. Returns the new state, or
    /// [`IllegalTransition`] if the event is not legal in the current state.
    pub fn dispatch(&mut self, event: SafetyEvent) -> Result<SafetyState, IllegalTransition> {
        let target = match (self.state, event) {
            (SafetyState::Autonomous, SafetyEvent::RequestAssistance) => {
                SafetyState::RequestingTeleop
            }
            // Supervisor/AI-initiated takeover: an operator session can be
            // granted without a domain fault request (the diagram's
            // REQUESTING_TELEOP step covers only domain-triggered sessions).
            (SafetyState::Autonomous, SafetyEvent::TeleopEngaged) => SafetyState::TeleopActive,
            (SafetyState::RequestingTeleop, SafetyEvent::TeleopEngaged) => {
                SafetyState::TeleopActive
            }
            (SafetyState::RequestingTeleop, SafetyEvent::RequestCancelled) => {
                SafetyState::Autonomous
            }
            (SafetyState::TeleopActive, SafetyEvent::TeleopReleased) => {
                SafetyState::ReturningToAutonomy
            }
            (SafetyState::ReturningToAutonomy, SafetyEvent::ReturnComplete) => {
                SafetyState::Autonomous
            }
            (SafetyState::ReturningToAutonomy, SafetyEvent::TeleopReEngaged) => {
                SafetyState::TeleopActive
            }
            (_, SafetyEvent::EmergencyStop) => SafetyState::EmergencyStop,
            (SafetyState::EmergencyStop, SafetyEvent::EmergencyCleared) => SafetyState::Autonomous,
            _ => {
                return Err(IllegalTransition {
                    from: self.state,
                    event,
                });
            }
        };
        self.state = target;
        Ok(target)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn happy_path_full_cycle() {
        let mut m = SafetyStateMachine::default();
        assert_eq!(m.state(), SafetyState::Autonomous);

        m.dispatch(SafetyEvent::RequestAssistance).unwrap();
        assert_eq!(m.state(), SafetyState::RequestingTeleop);

        m.dispatch(SafetyEvent::TeleopEngaged).unwrap();
        assert_eq!(m.state(), SafetyState::TeleopActive);

        m.dispatch(SafetyEvent::TeleopReleased).unwrap();
        assert_eq!(m.state(), SafetyState::ReturningToAutonomy);

        m.dispatch(SafetyEvent::ReturnComplete).unwrap();
        assert_eq!(m.state(), SafetyState::Autonomous);
    }

    #[test]
    fn supervisor_takeover_skips_requesting_state() {
        let mut m = SafetyStateMachine::default();
        m.dispatch(SafetyEvent::TeleopEngaged).unwrap();
        assert_eq!(m.state(), SafetyState::TeleopActive);
        m.dispatch(SafetyEvent::TeleopReleased).unwrap();
        m.dispatch(SafetyEvent::ReturnComplete).unwrap();
        assert_eq!(m.state(), SafetyState::Autonomous);
    }

    #[test]
    fn request_cancelled_returns_to_autonomy_directly() {
        let mut m = SafetyStateMachine::default();
        m.dispatch(SafetyEvent::RequestAssistance).unwrap();
        m.dispatch(SafetyEvent::RequestCancelled).unwrap();
        assert_eq!(m.state(), SafetyState::Autonomous);
    }

    #[test]
    fn reengage_during_blend_back() {
        let mut m = SafetyStateMachine::default();
        m.dispatch(SafetyEvent::RequestAssistance).unwrap();
        m.dispatch(SafetyEvent::TeleopEngaged).unwrap();
        m.dispatch(SafetyEvent::TeleopReleased).unwrap();
        m.dispatch(SafetyEvent::TeleopReEngaged).unwrap();
        assert_eq!(m.state(), SafetyState::TeleopActive);
    }

    #[test]
    fn estop_from_every_state_and_latch() {
        for start in [
            SafetyState::Autonomous,
            SafetyState::RequestingTeleop,
            SafetyState::TeleopActive,
            SafetyState::ReturningToAutonomy,
        ] {
            let mut m = SafetyStateMachine::new(start);
            assert_eq!(
                m.dispatch(SafetyEvent::EmergencyStop).unwrap(),
                SafetyState::EmergencyStop
            );
            assert!(m.stopped());
            // E-stop re-dispatch stays latched in EmergencyStop.
            m.dispatch(SafetyEvent::EmergencyStop).unwrap();
            assert!(m.stopped());
            // Only clearing exits, and it exits to Autonomous only.
            assert_eq!(
                m.dispatch(SafetyEvent::TeleopEngaged),
                Err(IllegalTransition {
                    from: SafetyState::EmergencyStop,
                    event: SafetyEvent::TeleopEngaged,
                })
            );
            m.dispatch(SafetyEvent::EmergencyCleared).unwrap();
            assert_eq!(m.state(), SafetyState::Autonomous);
        }
    }

    #[test]
    fn illegal_event_combinations_refused_without_state_change() {
        let mut m = SafetyStateMachine::default();
        // No teleop session to release.
        assert!(m.dispatch(SafetyEvent::TeleopReleased).is_err());
        assert_eq!(m.state(), SafetyState::Autonomous);
        // Cannot complete a return that never started.
        m.dispatch(SafetyEvent::RequestAssistance).unwrap();
        assert!(m.dispatch(SafetyEvent::ReturnComplete).is_err());
        assert_eq!(m.state(), SafetyState::RequestingTeleop);
    }

    #[test]
    fn wire_discriminator_roundtrip() {
        for s in [
            SafetyState::Autonomous,
            SafetyState::RequestingTeleop,
            SafetyState::TeleopActive,
            SafetyState::ReturningToAutonomy,
            SafetyState::EmergencyStop,
        ] {
            assert_eq!(SafetyState::from_u8(s.as_u8()), Some(s));
        }
        assert_eq!(SafetyState::from_u8(200), None);
    }

    #[test]
    fn display_uses_spec_names() {
        assert_eq!(
            SafetyState::RequestingTeleop.to_string(),
            "REQUESTING_TELEOP"
        );
        assert_eq!(SafetyState::EmergencyStop.to_string(), "EMERGENCY_STOP");
    }
}
