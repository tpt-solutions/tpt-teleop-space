#![warn(missing_docs)]
//! `tpt-t-space-thermal` — thermal management (spec §3 / §4.5).
//!
//! Monitors heater states, radiator deployment, and battery temperatures;
//! a limit breach (e.g. battery below −20 °C) arms the shared Safe Mode
//! trigger ([`tpt_t_space_core::safe_mode`]) so the asset protects itself.
//!
//! All monitors are hysteresis-based finite-state watchers: a limit must
//! be crossed *and held* for a debounce interval before it reports, so a
//! single noisy sample cannot fire the survival transition. Fixed-size,
//! allocation-free, panic-free.

use core::time::Duration;

/// Battery survival limit, °C (spec §4.5 example: battery temp < −20 °C).
pub const BATTERY_SURVIVAL_MIN_C: f32 = -20.0;

/// Why a thermal monitor arms Safe Mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ThermalCause {
    /// Battery temperature below the survival minimum.
    BatteryUnderTemp,
    /// Battery temperature above the charge ceiling.
    BatteryOverTemp,
    /// Electronics below the cold-start floor.
    ElectronicsUnderTemp,
    /// Electronics above the redline with heaters already off.
    ElectronicsOverTemp,
}

/// Monitored heater circuits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Heater {
    /// Battery survival heaters.
    Battery,
    /// Propellant / fuel-line heaters.
    Propellant,
    /// Electronics plate.
    Electronics,
}

/// One heater circuit's commanded + sensed state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeaterState {
    /// Commanded on/off.
    pub commanded_on: bool,
    /// Sensed current draw > 0 (the heater actually heats).
    pub drawing_current: bool,
}

/// Radiator deployment state (latched mechanism position).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadiatorState {
    /// Stowed for launch / cruise.
    Stowed,
    /// Deployment in progress (steps counted).
    Deploying { step: u8, of: u8 },
    /// Deployed and latched.
    Deployed,
    /// Deployment stalled (steps not advancing) — a deploy fault.
    Stalled,
}

/// Why a radiator is unhealthy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadiatorFault {
    /// Deploy steps stopped advancing.
    DeployStalled,
    /// Telemetry from the radiator lost.
    TelemetryLost,
}

/// Hysteresis thresholds for one temperature channel.
#[derive(Debug, Clone, Copy)]
pub struct Limits {
    /// Below this → under-temp fault (Safe Mode for the battery channel).
    pub min_c: f32,
    /// Above this → over-temp fault.
    pub max_c: f32,
    /// Recovery margins: the fault clears only `recover_min_c` above `min_c`
    /// / `recover_max_c` below `max_c`.
    pub recover_margin_c: f32,
    /// Debounce: the limit must hold this long to report.
    pub debounce: Duration,
}

/// Battery channel defaults: −20 °C survival floor, +45 °C ceiling.
#[must_use]
pub fn battery_limits() -> Limits {
    Limits {
        min_c: BATTERY_SURVIVAL_MIN_C,
        max_c: 45.0,
        recover_margin_c: 3.0,
        debounce: Duration::from_secs(5),
    }
}

/// Electronics channel defaults.
#[must_use]
pub fn electronics_limits() -> Limits {
    Limits {
        min_c: -35.0,
        max_c: 70.0,
        recover_margin_c: 5.0,
        debounce: Duration::from_secs(2),
    }
}

/// One channel's watcher state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Watch {
    Nominal,
    UnderArm { since_s: u64 },
    OverArm { since_s: u64 },
    UnderFault,
    OverFault,
}

/// A hysteresis + debounce watcher for one temperature channel.
#[derive(Debug, Clone, Copy)]
pub struct TempChannel {
    limits: Limits,
    /// Cause classes reported on the under/over faults (this channel's
    /// identity in the event vocabulary).
    causes: (ThermalCause, ThermalCause),
    watch: Watch,
}

impl TempChannel {
    /// New watcher for a channel reporting `causes.0` on under-temp and
    /// `causes.1` on over-temp.
    #[must_use]
    pub fn new(limits: Limits, causes: (ThermalCause, ThermalCause)) -> Self {
        Self {
            limits,
            causes,
            watch: Watch::Nominal,
        }
    }

    /// Current fault state (`Some(cause)` when faulted).
    #[must_use]
    pub fn state(&self) -> Option<ThermalCause> {
        match self.watch {
            Watch::UnderFault => Some(self.causes.0),
            Watch::OverFault => Some(self.causes.1),
            _ => None,
        }
    }

    /// Step the watcher: `now_s` is the monotonic clock, `temp_c` the
    /// latest sample. Returns the (possibly new) fault state.
    pub fn step(&mut self, now_s: u64, temp_c: f32) -> Option<ThermalCause> {
        match self.watch {
            Watch::Nominal => {
                if temp_c < self.limits.min_c {
                    self.watch = Watch::UnderArm { since_s: now_s };
                } else if temp_c > self.limits.max_c {
                    self.watch = Watch::OverArm { since_s: now_s };
                }
            }
            Watch::UnderArm { since_s } => {
                if temp_c >= self.limits.min_c {
                    self.watch = Watch::Nominal;
                } else if now_s.saturating_sub(since_s) >= self.limits.debounce.as_secs() {
                    self.watch = Watch::UnderFault;
                }
            }
            Watch::OverArm { since_s } => {
                if temp_c <= self.limits.max_c {
                    self.watch = Watch::Nominal;
                } else if now_s.saturating_sub(since_s) >= self.limits.debounce.as_secs() {
                    self.watch = Watch::OverFault;
                }
            }
            Watch::UnderFault => {
                if temp_c >= self.limits.min_c + self.limits.recover_margin_c {
                    self.watch = Watch::Nominal;
                }
            }
            Watch::OverFault => {
                if temp_c <= self.limits.max_c - self.limits.recover_margin_c {
                    self.watch = Watch::Nominal;
                }
            }
        }
        self.state()
    }
}

/// The thermal subsystem: battery + electronics channels, heater feedback,
/// and radiator deployment tracking.
#[derive(Debug, Clone, Copy)]
pub struct ThermalSubsystem {
    battery: TempChannel,
    electronics: TempChannel,
    heaters: [HeaterState; 3],
    radiator: RadiatorState,
    last_deploy_step_s: u64,
}

impl ThermalSubsystem {
    /// New subsystem, heaters commanded on, radiator stowed.
    #[must_use]
    pub fn new() -> Self {
        Self {
            battery: TempChannel::new(
                battery_limits(),
                (
                    ThermalCause::BatteryUnderTemp,
                    ThermalCause::BatteryOverTemp,
                ),
            ),
            electronics: TempChannel::new(
                electronics_limits(),
                (
                    ThermalCause::ElectronicsUnderTemp,
                    ThermalCause::ElectronicsOverTemp,
                ),
            ),
            heaters: [
                HeaterState {
                    commanded_on: true,
                    drawing_current: true,
                },
                HeaterState {
                    commanded_on: true,
                    drawing_current: true,
                },
                HeaterState {
                    commanded_on: true,
                    drawing_current: true,
                },
            ],
            radiator: RadiatorState::Stowed,
            last_deploy_step_s: 0,
        }
    }

    /// Feed one battery temperature sample; returns the armed Safe Mode
    /// cause when the survival limit is breached and debounced.
    pub fn step_battery_temp(&mut self, now_s: u64, temp_c: f32) -> Option<ThermalCause> {
        self.battery.step(now_s, temp_c)
    }

    /// Feed one electronics temperature sample.
    pub fn step_electronics_temp(&mut self, now_s: u64, temp_c: f32) -> Option<ThermalCause> {
        self.electronics.step(now_s, temp_c)
    }

    /// Update heater feedback (sensor current per circuit). A commanded-on
    /// heater drawing no current is a heater fault — surfaced in
    /// [`health_bits`] but not by itself a Safe Mode trigger.
    pub fn set_heater_feedback(&mut self, heater: Heater, drawing_current: bool) {
        let idx = heater as usize;
        if idx < self.heaters.len() {
            self.heaters[idx].drawing_current = drawing_current;
        }
    }

    /// Advance radiator deployment; a stalled sequence (no step progress
    /// within `now_s - last_progress` seconds) latches [`RadiatorState::Stalled`].
    pub fn step_radiator(&mut self, now_s: u64, observed: RadiatorState) -> Option<RadiatorFault> {
        if let RadiatorState::Deploying { step, .. } = observed {
            if let RadiatorState::Deploying { step: prev, .. } = self.radiator {
                if step == prev && now_s.saturating_sub(self.last_deploy_step_s) > 30 {
                    self.radiator = RadiatorState::Stalled;
                    return Some(RadiatorFault::DeployStalled);
                }
            }
            self.last_deploy_step_s = now_s;
        }
        self.radiator = observed;
        None
    }

    /// Radiator state.
    #[must_use]
    pub fn radiator(&self) -> RadiatorState {
        self.radiator
    }

    /// Aggregate health bitfield for [`tpt_t_space_core::wire::SubsystemHealth`]:
    /// bit 0..3 — heater circuit commanded-on-but-cold; bit 4 — radiator
    /// stalled; bits 8.. — channel faults (8: battery under, 9: battery
    /// over, 10: electronics).
    #[must_use]
    pub fn health_bits(&self) -> u32 {
        let mut bits = 0u32;
        for (i, h) in self.heaters.iter().enumerate() {
            if h.commanded_on && !h.drawing_current {
                bits |= 1 << i;
            }
        }
        if self.radiator == RadiatorState::Stalled {
            bits |= 1 << 4;
        }
        if self.battery.state() == Some(ThermalCause::BatteryUnderTemp) {
            bits |= 1 << 8;
        }
        if self.battery.state() == Some(ThermalCause::BatteryOverTemp) {
            bits |= 1 << 9;
        }
        if self.electronics.state().is_some() {
            bits |= 1 << 10;
        }
        bits
    }

    /// The Safe Mode cause this subsystem currently arms, if any.
    #[must_use]
    pub fn safe_mode_cause(&self) -> Option<ThermalCause> {
        self.battery.state().or(self.electronics.state())
    }
}

impl Default for ThermalSubsystem {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn battery_survival_limit_triggers_after_debounce() {
        let mut t = ThermalSubsystem::new();
        // First sample arms, debounce is 5 s.
        assert_eq!(t.step_battery_temp(0, -22.0), None);
        assert_eq!(t.step_battery_temp(2, -22.5), None);
        // Spec example: battery temp < −20 °C held → Safe Mode trigger.
        assert_eq!(
            t.step_battery_temp(6, -22.0),
            Some(ThermalCause::BatteryUnderTemp)
        );
        assert_eq!(t.safe_mode_cause(), Some(ThermalCause::BatteryUnderTemp));
        assert_ne!(t.health_bits() & (1 << 8), 0);
    }

    #[test]
    fn single_noisy_sample_does_not_trigger() {
        let mut t = ThermalSubsystem::new();
        assert_eq!(t.step_battery_temp(0, 21.0), None);
        assert_eq!(t.step_battery_temp(1, -30.0), None, "armed only");
        assert_eq!(t.step_battery_temp(2, 21.0), None, "back to nominal");
        assert_eq!(t.step_battery_temp(10, -31.0), None, "re-armed, not held");
        assert!(t.safe_mode_cause().is_none());
    }

    #[test]
    fn over_temp_and_recovery_hysteresis() {
        let mut t = ThermalSubsystem::new();
        assert_eq!(t.step_battery_temp(0, 50.0), None);
        assert_eq!(
            t.step_battery_temp(6, 50.0),
            Some(ThermalCause::BatteryOverTemp)
        );
        // Recovery needs the margin (45 − 3 = 42).
        assert!(t.step_battery_temp(7, 43.0).is_some());
        assert_eq!(t.step_battery_temp(8, 41.0), None);
        assert!(t.safe_mode_cause().is_none());
    }

    #[test]
    fn electronics_channel_maps_to_its_cause() {
        let mut t = ThermalSubsystem::new();
        assert_eq!(t.step_electronics_temp(0, 75.0), None);
        assert_eq!(
            t.step_electronics_temp(3, 76.0),
            Some(ThermalCause::ElectronicsOverTemp)
        );
    }

    #[test]
    fn heater_feedback_fault_is_visible_not_fatal() {
        let mut t = ThermalSubsystem::new();
        t.set_heater_feedback(Heater::Battery, false);
        assert_ne!(t.health_bits() & 1, 0, "battery heater dead bit");
        assert!(
            t.safe_mode_cause().is_none(),
            "heater fault alone is not Safe Mode"
        );
    }

    #[test]
    fn stalled_radiator_latches_fault() {
        let mut t = ThermalSubsystem::new();
        assert_eq!(
            t.step_radiator(0, RadiatorState::Deploying { step: 2, of: 5 }),
            None
        );
        // Same step 31 s later → stalled.
        assert_eq!(
            t.step_radiator(31, RadiatorState::Deploying { step: 2, of: 5 }),
            Some(RadiatorFault::DeployStalled)
        );
        assert_eq!(t.radiator(), RadiatorState::Stalled);
        // Progress resumes → fault clears.
        assert_eq!(
            t.step_radiator(40, RadiatorState::Deploying { step: 3, of: 5 }),
            None
        );
        assert_eq!(t.step_radiator(41, RadiatorState::Deployed), None);
    }
}
