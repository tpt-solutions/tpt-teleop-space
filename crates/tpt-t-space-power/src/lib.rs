#![warn(missing_docs)]
//! `tpt-t-space-power` — power subsystem management (spec §3 / §4.5).
//!
//! * solar-array pointing: a coarse grid search over the two gimbal axes
//!   maximizes `cos(incidence)` against the sun vector — deterministic,
//!   bounded, allocation-free;
//! * battery charge/discharge cycle tracking (coulomb counting with a
//!   depth-of-discharge accumulator);
//! * eclipse-period load shedding over three priority tiers;
//! * the survival trigger: solar output below the survival threshold for a
//!   sustained interval (dust storm, spec §6) arms the shared Safe Mode
//!   transition.
//!
//! No locks, no allocation after construction, no panics: every decision
//! is a typed event the mission loop routes.

use tpt_t_space_core::safe_mode::SafeModeCause;

/// Survival threshold: sustained solar output below this fraction of the
/// clear-sky rating arms Safe Mode (spec §6 dust-storm scenario).
pub const SURVIVAL_OUTPUT_FRACTION: f32 = 0.08;

/// How long the output must stay below the threshold before Safe Mode
/// (debounce, seconds).
pub const SURVIVAL_DEBOUNCE_S: u64 = 60;

/// Load-shed priority tiers (shed highest number first).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum LoadTier {
    /// Never shed: survival heaters, avionics core.
    Critical = 0,
    /// Shed in eclipse when the discharge budget tightens.
    Mission = 1,
    /// Shed first: opportunistic payloads, backhaul radios.
    Opportunistic = 2,
}

/// One sheddable/restoreable load.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Load {
    /// Which tier it belongs to.
    pub tier: LoadTier,
    /// Whether it is currently powered.
    pub powered: bool,
}

/// Eclipse/insolation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SunState {
    /// Full sun.
    Sun,
    /// Shadowed (eclipse, or dust-darkened sky below the survival line).
    Shadowed,
}

/// Pointing solution for one solar array: gimbal angles + expected output.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Pointing {
    /// Gimbal azimuth, radians.
    pub azimuth_rad: f32,
    /// Gimbal elevation, radians.
    pub elevation_rad: f32,
    /// Expected output fraction `0.0..=1.0` at this pointing.
    pub output: f32,
}

/// The solar array: two-axis gimbal with bounded travel.
#[derive(Debug, Clone, Copy)]
pub struct SolarArray {
    /// Clear-sky output at normal incidence, watts.
    pub rated_w: f32,
    /// Gimbal travel limits `(az_min, az_max, el_min, el_max)`, radians.
    pub travel: (f32, f32, f32, f32),
    /// Grid search resolution per axis.
    pub steps: u32,
}

impl SolarArray {
    /// Best pointing for a sun vector in the array gimbal frame (sun side
    /// is +z): a deterministic grid search maximizing the z-component of
    /// the array normal rotated by the gimbal angles.
    #[must_use]
    pub fn best_pointing(&self, sun: [f32; 3]) -> Pointing {
        let (az_min, az_max, el_min, el_max) = self.travel;
        let steps = self.steps.max(2);
        let mut best = Pointing {
            azimuth_rad: az_min,
            elevation_rad: el_min,
            output: 0.0,
        };
        let sun_len = (sun[0] * sun[0] + sun[1] * sun[1] + sun[2] * sun[2]).sqrt();
        if sun_len < 1e-6 {
            return best;
        }
        let (sx, sy, sz) = (sun[0] / sun_len, sun[1] / sun_len, sun[2] / sun_len);
        for i in 0..steps {
            let az = az_min + (az_max - az_min) * (i as f32) / ((steps - 1) as f32);
            for j in 0..steps {
                let el = el_min + (el_max - el_min) * (j as f32) / ((steps - 1) as f32);
                // Array normal after (az about z, then el about y):
                // n = Rz(az)·Ry(el)·ẑ
                let (ca, sa, ce, se) = (az.cos(), az.sin(), el.cos(), el.sin());
                let nx = sa * ce;
                let ny = sa * se;
                let nz = ca;
                let dot = nx * sx + ny * sy + nz * sz;
                if dot > best.output {
                    best = Pointing {
                        azimuth_rad: az,
                        elevation_rad: el,
                        output: dot,
                    };
                }
            }
        }
        best
    }

    /// Expected watts at a pointing.
    #[must_use]
    pub fn watts(&self, p: &Pointing) -> f32 {
        p.output.max(0.0) * self.rated_w
    }
}

/// Battery state: coulomb-counted SoC plus cycle bookkeeping.
#[derive(Debug, Clone, Copy)]
pub struct Battery {
    /// Capacity, watt-hours.
    pub capacity_wh: f32,
    /// State of charge, watt-hours stored.
    soc_wh: f32,
    /// Accumulated discharge depth for the current cycle, Wh.
    cycle_depth_wh: f32,
    /// Completed charge/discharge cycles (full depth-of-discharge count).
    cycles: f32,
    discharging: bool,
}

impl Battery {
    /// New full battery.
    #[must_use]
    pub fn new(capacity_wh: f32) -> Self {
        Self {
            capacity_wh,
            soc_wh: capacity_wh,
            cycle_depth_wh: 0.0,
            cycles: 0.0,
            discharging: false,
        }
    }

    /// State of charge, percent.
    #[must_use]
    pub fn soc_pct(&self) -> f32 {
        100.0 * self.soc_wh / self.capacity_wh
    }

    /// Completed equivalent full cycles.
    #[must_use]
    pub fn cycles(&self) -> f32 {
        self.cycles
    }

    /// Integrate one interval: `watts` positive = charging, negative =
    /// discharging. Tracks depth-of-discharge for cycle counting.
    pub fn step(&mut self, watts: f32, dt_s: f32) {
        let delta_wh = watts * dt_s / 3600.0;
        let next = (self.soc_wh + delta_wh).clamp(0.0, self.capacity_wh);
        let discharged = self.soc_wh - next;
        if discharged > 0.0 {
            if !self.discharging {
                self.discharging = true;
            }
            self.cycle_depth_wh += discharged;
        } else if delta_wh > 0.0 && self.discharging && next >= self.capacity_wh * 0.95 {
            // Recharged near full: close the cycle.
            self.cycles += self.cycle_depth_wh / self.capacity_wh;
            self.cycle_depth_wh = 0.0;
            self.discharging = false;
        }
        self.soc_wh = next;
    }
}

/// The power subsystem: array, battery, load tiers, and the survival
/// trigger.
#[derive(Debug, Clone, Copy)]
pub struct PowerSubsystem {
    array: SolarArray,
    battery: Battery,
    loads: [Load; 8],
    /// Current pointing.
    pointing: Pointing,
    under_threshold_since_s: Option<u64>,
    shed_active: bool,
    last_output_w: f32,
}

impl PowerSubsystem {
    /// Assemble from parts; loads start powered, tier from index order
    /// (0-1 critical, 2-5 mission, 6-7 opportunistic — callers override
    /// via [`Self::set_load_tier`]).
    #[must_use]
    pub fn new(array: SolarArray, battery: Battery) -> Self {
        let mut loads = [Load {
            tier: LoadTier::Mission,
            powered: true,
        }; 8];
        for (i, load) in loads.iter_mut().enumerate() {
            load.tier = match i {
                0..=1 => LoadTier::Critical,
                2..=5 => LoadTier::Mission,
                _ => LoadTier::Opportunistic,
            };
        }
        let pointing = Pointing {
            azimuth_rad: 0.0,
            elevation_rad: 0.0,
            output: 0.0,
        };
        Self {
            array,
            battery,
            loads,
            pointing,
            under_threshold_since_s: None,
            shed_active: false,
            last_output_w: 0.0,
        }
    }

    /// Override one load's tier.
    pub fn set_load_tier(&mut self, index: usize, tier: LoadTier) {
        if index < self.loads.len() {
            self.loads[index].tier = tier;
        }
    }

    /// Re-optimize array pointing for the current sun vector.
    pub fn track_sun(&mut self, sun: [f32; 3]) -> Pointing {
        self.pointing = self.array.best_pointing(sun);
        self.pointing
    }

    /// Integrate one power interval at `now_s`: harvest from the pointing,
    /// consume the powered loads, manage eclipse shedding. Returns the
    /// Safe Mode cause when the survival trigger fires.
    pub fn step(&mut self, now_s: u64, dt_s: f32, shadowed: bool) -> Option<SafeModeCause> {
        let harvest = if shadowed {
            0.0
        } else {
            self.array.watts(&self.pointing)
        };
        self.last_output_w = harvest;
        let load_w: f32 = self
            .loads
            .iter()
            .filter(|l| l.powered)
            .enumerate()
            .map(|(i, l)| 8.0 + i as f32 * 4.0) // per-load nameplate, watts
            .sum();
        self.battery.step(harvest - load_w, dt_s);
        self.manage_shedding(shadowed);
        self.survival_trigger(now_s)
    }

    /// Eclipse load shedding: drop Opportunistic then Mission tiers while
    /// shadowed; restore in reverse order when back in sun.
    fn manage_shedding(&mut self, shadowed: bool) {
        if shadowed {
            for tier in [LoadTier::Opportunistic, LoadTier::Mission] {
                for load in &mut self.loads {
                    if load.tier == tier {
                        load.powered = false;
                    }
                }
            }
            self.shed_active = true;
        } else if self.shed_active {
            // Restore in reverse: Mission first, Opportunistic last.
            for tier in [LoadTier::Mission, LoadTier::Opportunistic] {
                for load in &mut self.loads {
                    if load.tier == tier {
                        load.powered = true;
                    }
                }
            }
            self.shed_active = false;
        }
    }

    /// The dust-storm / eclipse-overrun survival trigger: output under the
    /// survival fraction, sustained for the debounce window.
    fn survival_trigger(&mut self, now_s: u64) -> Option<SafeModeCause> {
        let fraction = self.last_output_w / self.array.rated_w.max(1.0);
        if fraction < SURVIVAL_OUTPUT_FRACTION {
            let since = *self.under_threshold_since_s.get_or_insert(now_s);
            if now_s.saturating_sub(since) >= SURVIVAL_DEBOUNCE_S {
                return Some(SafeModeCause::PowerSurvivalThreshold);
            }
        } else {
            self.under_threshold_since_s = None;
        }
        None
    }

    /// Whether shedding is active.
    #[must_use]
    pub fn shedding(&self) -> bool {
        self.shed_active
    }

    /// Last computed harvest, watts.
    #[must_use]
    pub fn output_w(&self) -> f32 {
        self.last_output_w
    }

    /// Battery view.
    #[must_use]
    pub fn battery(&self) -> &Battery {
        &self.battery
    }

    /// Current pointing.
    #[must_use]
    pub fn pointing(&self) -> &Pointing {
        &self.pointing
    }

    /// Whether a load is powered.
    #[must_use]
    pub fn load_powered(&self, index: usize) -> bool {
        self.loads.get(index).map_or(false, |l| l.powered)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn array() -> SolarArray {
        SolarArray {
            rated_w: 1000.0,
            travel: (-1.5, 1.5, -0.5, 1.5),
            steps: 32,
        }
    }

    #[test]
    fn pointing_tracks_the_sun() {
        let arr = array();
        // Sun at +z (straight up): zero gimbal is optimal.
        let p = arr.best_pointing([0.0, 0.0, 10.0]);
        assert!(p.output > 0.99, "cos incidence near 1, got {}", p.output);
        // Sun toward +x: azimuth swings toward it, output stays high.
        let p = arr.best_pointing([10.0, 0.0, 2.0]);
        assert!(p.output > 0.9, "tracking keeps incidence high");
        assert!(p.azimuth_rad > 0.5, "azimuth moved toward the sun");
        // Degenerate sun vector: don't NaN.
        let p = arr.best_pointing([0.0, 0.0, 0.0]);
        assert!(p.output == 0.0);
    }

    #[test]
    fn eclipse_sheds_and_restores() {
        let mut p = PowerSubsystem::new(array(), Battery::new(2000.0));
        p.track_sun([0.0, 0.0, 1.0]);
        // Enter eclipse: opportunistic and mission loads drop.
        p.step(0, 1.0, true);
        assert!(p.shedding());
        assert!(!p.load_powered(6), "opportunistic shed");
        assert!(!p.load_powered(2), "mission shed");
        assert!(p.load_powered(0), "critical kept");
        // Sun returns: restore.
        p.step(10, 1.0, false);
        assert!(!p.shedding());
        assert!(p.load_powered(6));
    }

    #[test]
    fn dust_storm_triggers_survival_safe_mode() {
        let mut p = PowerSubsystem::new(array(), Battery::new(2000.0));
        p.track_sun([0.0, 0.0, 1.0]);
        // Dust storm: sky dark (modeled as shadowed). The trigger debounces.
        assert_eq!(p.step(0, 1.0, true), None);
        assert_eq!(p.step(30, 1.0, true), None);
        assert_eq!(
            p.step(61, 1.0, true),
            Some(SafeModeCause::PowerSurvivalThreshold),
            "spec §6: solar output below survival threshold → Safe Mode"
        );
    }

    #[test]
    fn brief_shadow_does_not_trigger() {
        let mut p = PowerSubsystem::new(array(), Battery::new(2000.0));
        p.track_sun([0.0, 0.0, 1.0]);
        p.step(0, 1.0, true);
        p.step(30, 1.0, true);
        // Back in sun before the debounce: no trigger, timer resets.
        p.step(40, 1.0, false);
        p.step(100, 1.0, true);
        assert!(p.step(120, 1.0, true).is_none(), "fresh 20s < 60s debounce");
    }

    #[test]
    fn battery_cycles_count() {
        let mut b = Battery::new(100.0);
        b.step(-50.0, 3600.0); // 50 Wh out
        assert!((b.soc_pct() - 50.0).abs() < 1e-4);
        assert_eq!(b.cycles(), 0.0);
        b.step(100.0, 3600.0); // back to full → cycle closes (0.5 DoD)
        assert!((b.cycles() - 0.5).abs() < 1e-4);
        assert!(b.soc_pct() > 95.0);
    }
}
