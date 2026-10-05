//! The single, atomic, zero-allocation Safe Mode transition (spec §4.5).
//!
//! When `tpt-t-space-thermal` or `tpt-t-space-power` detects a survival
//! limit breach — battery temperature below −20 °C, solar output under the
//! survival threshold in a dust storm — the asset must instantly enter the
//! survival configuration. Spec-mandated sequence, executed as *one*
//! indivisible operation:
//!
//! 1. power down non-essential payloads ([`SafeModeActions::shed_payloads`]),
//! 2. slew to a sun-pointing attitude ([`SafeModeActions::slew_to_sun_point`]),
//! 3. begin the low-rate DTN beacon ([`SafeModeActions::start_beacon`]).
//!
//! The controller owns the decision, the [`SafeModeActions`] implementor
//! owns the actuators: dependency inversion keeps `core` free of `dtn`/ADCS
//! dependencies while the transition stays atomic (no intermediate states —
//! if `enter` returns, the asset *is* in Safe Mode), allocation-free, and
//! idempotent (re-entry from an already-safe state performs nothing, so a
//! flapping sensor cannot re-trigger actuator transients).
//!
//! Traceability (Phase 15): every accepted entry records the [`cause`]
//! (SafeModeCause) and the entry timestamp; `transitions()` counts entries
//! for the audit log.

use core::fmt;

/// Why Safe Mode was entered.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum SafeModeCause {
    /// Thermal limit breach (e.g. battery temperature < −20 °C).
    ThermalLimit,
    /// Power dropped below the survival threshold (eclipse overrun, dust
    /// storm under the solar arrays).
    PowerSurvivalThreshold,
    /// Radiation/fault-management escalation (SEU storm, scrubber
    /// uncorrectable multi-bit failure).
    RadiationAnomaly,
    /// Watchdog triage: a core missed its deadline and was reset.
    Watchdog,
    /// Explicit operator command over the DTN uplink.
    OperatorCommand,
    /// Unclassified (carries a coarse domain code).
    Unknown(u8),
}

impl fmt::Display for SafeModeCause {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SafeModeCause::ThermalLimit => f.write_str("THERMAL_LIMIT"),
            SafeModeCause::PowerSurvivalThreshold => f.write_str("POWER_SURVIVAL"),
            SafeModeCause::RadiationAnomaly => f.write_str("RADIATION"),
            SafeModeCause::Watchdog => f.write_str("WATCHDOG"),
            SafeModeCause::OperatorCommand => f.write_str("OPERATOR"),
            SafeModeCause::Unknown(c) => write!(f, "UNKNOWN({c})"),
        }
    }
}

/// The survival actions the controller drives. Implemented by the platform
/// composition layer (power + ADCS + DTN glue); called only from
/// [`SafeModeController::enter`], in this exact order.
pub trait SafeModeActions {
    /// Power down all non-essential payloads (science stack, transceivers
    /// above beacon rate, heaters beyond survival set).
    fn shed_payloads(&mut self);
    /// Command the attitude-control system to the sun-pointing attitude
    /// (power + thermal positive).
    fn slew_to_sun_point(&mut self);
    /// Begin the low-rate DTN beacon so the orbiter pass finds us alive.
    fn start_beacon(&mut self, rate_bps: u32);
}

/// Low-rate beacon rate, bits per second (spec §4.5: "low-rate beacon").
pub const SAFE_MODE_BEACON_BPS: u32 = 8;

/// Lifecycle state of the Safe Mode controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
pub enum SafeModeState {
    /// Normal operations.
    #[default]
    Nominal = 0,
    /// Survival configuration active.
    SafeMode = 1,
}

/// The Safe Mode decision point. One instance per asset; not `Sync` — it
/// lives on the supervisor/fault-management loop thread.
#[derive(Debug, Clone, Copy)]
pub struct SafeModeController {
    state: SafeModeState,
    cause: Option<SafeModeCause>,
    entered_us: u64,
    transitions: u32,
}

impl Default for SafeModeController {
    fn default() -> Self {
        Self {
            state: SafeModeState::Nominal,
            cause: None,
            entered_us: 0,
            transitions: 0,
        }
    }
}

impl SafeModeController {
    /// Create a nominal controller.
    pub fn new() -> Self {
        Self::default()
    }

    /// Current lifecycle state.
    pub fn state(&self) -> SafeModeState {
        self.state
    }

    /// Whether the asset is in the survival configuration.
    pub fn is_safe(&self) -> bool {
        self.state == SafeModeState::SafeMode
    }

    /// Recorded entry cause (`None` while nominal).
    pub fn cause(&self) -> Option<SafeModeCause> {
        self.cause
    }

    /// Entry timestamp of the active/last Safe Mode episode, µs.
    pub fn entered_at_us(&self) -> u64 {
        self.entered_us
    }

    /// Number of entries performed since boot (audit counter).
    pub fn transitions(&self) -> u32 {
        self.transitions
    }

    /// The atomic transition. Enters Safe Mode by executing all three
    /// survival actions unconditionally, in spec order. Returns `true` if
    /// the transition was performed, `false` if the asset was already safe
    /// (idempotent — the actions are *not* re-run).
    pub fn enter(
        &mut self,
        now_us: u64,
        cause: SafeModeCause,
        actions: &mut impl SafeModeActions,
    ) -> bool {
        if self.state == SafeModeState::SafeMode {
            return false;
        }
        actions.shed_payloads();
        actions.slew_to_sun_point();
        actions.start_beacon(SAFE_MODE_BEACON_BPS);
        self.state = SafeModeState::SafeMode;
        self.cause = Some(cause);
        self.entered_us = now_us;
        self.transitions = self.transitions.wrapping_add(1);
        true
    }

    /// Exit Safe Mode back to nominal after the fault clears. The recovery
    /// sequence (DTN resume, payload restore, attitude re-acquisition) is
    /// mission-specific and driven by the caller; the controller only
    /// latches the state.
    pub fn recover(&mut self, now_us: u64) -> Result<(), NotSafe> {
        if self.state != SafeModeState::SafeMode {
            return Err(NotSafe { at_us: now_us });
        }
        self.state = SafeModeState::Nominal;
        Ok(())
    }
}

/// Recovery attempted while not in Safe Mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NotSafe {
    /// Timestamp of the rejected recovery attempt, µs.
    pub at_us: u64,
}

impl fmt::Display for NotSafe {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "recovery attempted while not in Safe Mode")
    }
}

impl std::error::Error for NotSafe {}

#[cfg(test)]
mod tests {
    use super::*;

    /// Records every action in order, with the payload-power set state.
    #[derive(Default)]
    struct MockPlatform {
        log: [&'static str; 8],
        log_len: usize,
        beacon_bps: u32,
        payloads_shed: bool,
    }

    impl MockPlatform {
        fn push(&mut self, s: &'static str) {
            if self.log_len < self.log.len() {
                self.log[self.log_len] = s;
                self.log_len += 1;
            }
        }
    }

    impl SafeModeActions for MockPlatform {
        fn shed_payloads(&mut self) {
            self.payloads_shed = true;
            self.push("shed");
        }
        fn slew_to_sun_point(&mut self) {
            self.push("slew");
        }
        fn start_beacon(&mut self, rate_bps: u32) {
            self.beacon_bps = rate_bps;
            self.push("beacon");
        }
    }

    #[test]
    fn atomic_entry_runs_all_actions_in_spec_order() {
        let mut c = SafeModeController::new();
        let mut p = MockPlatform::default();
        assert!(c.enter(1_000, SafeModeCause::ThermalLimit, &mut p));
        assert_eq!(&p.log[..p.log_len], &["shed", "slew", "beacon"]);
        assert!(p.payloads_shed);
        assert_eq!(p.beacon_bps, SAFE_MODE_BEACON_BPS);
        assert!(c.is_safe());
        assert_eq!(c.cause(), Some(SafeModeCause::ThermalLimit));
        assert_eq!(c.entered_at_us(), 1_000);
        assert_eq!(c.transitions(), 1);
    }

    #[test]
    fn reentry_is_idempotent_no_action_replay() {
        let mut c = SafeModeController::new();
        let mut p = MockPlatform::default();
        c.enter(1_000, SafeModeCause::PowerSurvivalThreshold, &mut p);
        // A second (flapping) trigger performs nothing.
        assert!(!c.enter(2_000, SafeModeCause::ThermalLimit, &mut p));
        assert_eq!(p.log_len, 3, "no action replay");
        assert_eq!(c.transitions(), 1);
        assert_eq!(
            c.cause(),
            Some(SafeModeCause::PowerSurvivalThreshold),
            "first cause preserved"
        );
    }

    #[test]
    fn recover_then_reenter_is_fresh_episode() {
        let mut c = SafeModeController::new();
        let mut p = MockPlatform::default();
        c.enter(1_000, SafeModeCause::RadiationAnomaly, &mut p);
        c.recover(60_000_000).expect("recover");
        assert_eq!(c.state(), SafeModeState::Nominal);
        assert!(!c.is_safe());
        // Storm returns: full episode again, counter advances.
        assert!(c.enter(90_000_000, SafeModeCause::ThermalLimit, &mut p));
        assert_eq!(c.transitions(), 2);
        assert_eq!(c.entered_at_us(), 90_000_000);
    }

    #[test]
    fn recover_only_from_safe_mode() {
        let mut c = SafeModeController::new();
        assert_eq!(c.recover(5), Err(NotSafe { at_us: 5 }));
        let mut p = MockPlatform::default();
        c.enter(10, SafeModeCause::OperatorCommand, &mut p);
        c.recover(20).expect("recover");
        assert_eq!(c.recover(30), Err(NotSafe { at_us: 30 }));
    }

    #[test]
    fn cause_display() {
        assert_eq!(
            SafeModeCause::PowerSurvivalThreshold.to_string(),
            "POWER_SURVIVAL"
        );
        assert_eq!(SafeModeCause::Unknown(9).to_string(), "UNKNOWN(9)");
    }
}
