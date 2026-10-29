//! Mission-profile presets (Phase 20): sane defaults for one vehicle class.
//!
//! A scenario starts from these instead of hand-picking limits. Each preset
//! reuses the workspace's own constructors and constants (the link geometry,
//! battery and electronics thermal limits, the rover flight rules, the power
//! survival fraction), so a change in one crate reaches every preset.

use tpt_t_space_power::SURVIVAL_OUTPUT_FRACTION;
use tpt_t_space_predictive::planner::RoverLimits;
use tpt_t_space_thermal::{Limits, battery_limits, electronics_limits};

use crate::dtn_link::LinkProfile;

/// One vehicle class's defaults.
#[derive(Debug, Clone, Copy)]
pub struct Profile {
    /// Short name used on the command line.
    pub name: &'static str,
    /// One-line description.
    pub summary: &'static str,
    /// Downlink geometry.
    pub link: LinkProfile,
    /// Battery capacity, watt-hours.
    pub battery_wh: f32,
    /// Clear-sky array rating, watts.
    pub array_w: f32,
    /// Survival floor, as a fraction of the array rating.
    pub survival_fraction: f32,
    /// Battery temperature limits.
    pub battery: Limits,
    /// Electronics temperature limits.
    pub electronics: Limits,
    /// Traverse limits, for vehicles that drive.
    pub rover: Option<RoverLimits>,
}

/// LEO cubesat: relay passes, an eclipse every orbit, no traverse.
#[must_use]
pub fn leo_cubesat() -> Profile {
    Profile {
        name: "leo-cubesat",
        summary: "LEO cubesat: relay passes, eclipse every orbit",
        link: LinkProfile::leo_relay(),
        battery_wh: 120.0,
        array_w: 350.0,
        survival_fraction: SURVIVAL_OUTPUT_FRACTION,
        battery: battery_limits(),
        electronics: electronics_limits(),
        rover: None,
    }
}

/// Mars surface rover: twelve light-minutes, long occultations, traverse limits.
#[must_use]
pub fn mars_rover() -> Profile {
    Profile {
        name: "mars-rover",
        summary: "Mars surface rover: 12 light-minutes, occultation every sol",
        link: LinkProfile::mars_surface(),
        battery_wh: 900.0,
        array_w: 600.0,
        survival_fraction: SURVIVAL_OUTPUT_FRACTION,
        battery: battery_limits(),
        electronics: electronics_limits(),
        rover: Some(RoverLimits::flight_rules()),
    }
}

/// GEO servicer: a short light time, a rare occultation, docking power.
#[must_use]
pub fn geo_servicer() -> Profile {
    Profile {
        name: "geo-servicer",
        summary: "GEO servicer: short light time, rare occultation",
        link: LinkProfile {
            one_way_delay_s: 0.13,
            blackout_period_s: 86_164.0,
            blackout_fraction: 0.0,
            phase_s: 0.0,
        },
        battery_wh: 2_000.0,
        array_w: 3_000.0,
        survival_fraction: SURVIVAL_OUTPUT_FRACTION,
        battery: battery_limits(),
        electronics: electronics_limits(),
        rover: None,
    }
}

/// Every preset, in display order.
#[must_use]
pub fn all() -> [Profile; 3] {
    [leo_cubesat(), mars_rover(), geo_servicer()]
}

/// The preset named `name`, if there is one.
#[must_use]
pub fn by_name(name: &str) -> Option<Profile> {
    all().into_iter().find(|p| p.name == name)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_preset_is_physically_sane() {
        for p in all() {
            assert!(p.battery_wh > 0.0 && p.array_w > 0.0, "{}", p.name);
            assert!(
                p.survival_fraction > 0.0 && p.survival_fraction < 1.0,
                "{}",
                p.name
            );
            assert!(p.battery.min_c < p.battery.max_c, "{}", p.name);
            assert!(
                p.link.one_way_delay_s >= 0.0 && p.link.blackout_period_s > 0.0,
                "{}",
                p.name
            );
        }
    }

    #[test]
    fn light_time_orders_the_geometries() {
        let leo = leo_cubesat().link.one_way_delay_s;
        let geo = geo_servicer().link.one_way_delay_s;
        let mars = mars_rover().link.one_way_delay_s;
        assert!(leo < geo && geo < mars, "leo {leo} geo {geo} mars {mars}");
    }

    #[test]
    fn only_the_rover_drives() {
        assert!(mars_rover().rover.is_some());
        assert!(leo_cubesat().rover.is_none());
        assert!(geo_servicer().rover.is_none());
    }

    #[test]
    fn names_resolve_and_are_unique() {
        for p in all() {
            assert_eq!(by_name(p.name).map(|q| q.name), Some(p.name));
        }
        assert!(by_name("venus-lander").is_none());
        let names: Vec<_> = all().iter().map(|p| p.name).collect();
        assert_eq!(
            names.len(),
            names
                .iter()
                .collect::<std::collections::BTreeSet<_>>()
                .len()
        );
    }
}
