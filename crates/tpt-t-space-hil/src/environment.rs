//! Simulated thermal-vacuum cycling (Phase 16: extreme-environment validation,
//! simulated: thermal only).
//!
//! A chamber cycles the part between hot and cold plateaus. This cycles a
//! model: a triangle-wave battery temperature, fed through the shipped
//! thermal subsystem one sample per second. It counts the Safe Mode trips
//! that the cycle causes.
//!
//! Vibration and vacuum outgassing are not modelled. Neither has a model
//! here, so neither is covered by these results.

use tpt_t_space_thermal::{ThermalCause, ThermalSubsystem};

/// A temperature cycle, applied to the battery channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cycle {
    /// Cold plateau, °C.
    pub low_c: f32,
    /// Hot plateau, °C.
    pub high_c: f32,
    /// Length of one full cycle, seconds.
    pub period_s: u64,
    /// Number of cycles.
    pub cycles: u32,
}

/// What the cycle tripped.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Report {
    /// Temperature samples applied.
    pub samples: u64,
    /// Entries into the under-temperature fault.
    pub low_trips: u32,
    /// Entries into the over-temperature fault.
    pub high_trips: u32,
    /// Seconds into the run of the first trip, if any.
    pub first_trip_s: Option<u64>,
}

/// Run the cycle through the thermal subsystem, one sample per second.
#[must_use]
pub fn run(c: Cycle) -> Report {
    let mut thermal = ThermalSubsystem::new();
    let mut report = Report::default();
    let mut previous: Option<ThermalCause> = None;
    let total = u64::from(c.cycles) * c.period_s;
    for t in 0..total {
        let phase = (t % c.period_s) as f32 / c.period_s as f32;
        let wave = if phase < 0.5 {
            phase * 2.0
        } else {
            2.0 - phase * 2.0
        };
        let temp = c.low_c + (c.high_c - c.low_c) * wave;
        report.samples += 1;
        let now = thermal.step_battery_temp(t, temp);
        if let (Some(cause), None) = (now, previous) {
            match cause {
                ThermalCause::BatteryUnderTemp => report.low_trips += 1,
                ThermalCause::BatteryOverTemp => report.high_trips += 1,
                _ => {}
            }
            report.first_trip_s.get_or_insert(t);
        }
        previous = now;
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_cycle_inside_the_battery_limits_trips_nothing() {
        let r = run(Cycle {
            low_c: 0.0,
            high_c: 30.0,
            period_s: 3_600,
            cycles: 3,
        });
        assert_eq!((r.low_trips, r.high_trips), (0, 0), "{r:?}");
        assert_eq!(r.first_trip_s, None);
    }

    #[test]
    fn a_cold_plateau_below_the_survival_floor_trips_the_low_fault() {
        let r = run(Cycle {
            low_c: -35.0,
            high_c: 30.0,
            period_s: 3_600,
            cycles: 2,
        });
        assert!(r.low_trips >= 1, "{r:?}");
        assert_eq!(r.high_trips, 0, "{r:?}");
        assert!(r.first_trip_s.is_some());
    }

    #[test]
    fn a_hot_plateau_above_the_ceiling_trips_the_high_fault() {
        let r = run(Cycle {
            low_c: 10.0,
            high_c: 55.0,
            period_s: 3_600,
            cycles: 2,
        });
        assert!(r.high_trips >= 1, "{r:?}");
        assert_eq!(r.low_trips, 0, "{r:?}");
    }
}
