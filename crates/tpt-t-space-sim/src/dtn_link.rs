//! DTN link profiles (spec §8): blackout windows (planetary occultation,
//! dust storms) and signal-path delay (Mars–Earth light time).
//!
//! The profile is deterministic: given a simulated time it answers "is
//! the link up?" — the DTN agent (Phase 5) consumes exactly that.

/// Delay + blackout profile for one relay geometry.
#[derive(Debug, Clone, Copy)]
pub struct LinkProfile {
    /// One-way light time, seconds (Mars: 180–1300 s; LEO relay: ~0.01).
    pub one_way_delay_s: f64,
    /// Occultation period, seconds (Mars rotation ≈ 88_775 s).
    pub blackout_period_s: f64,
    /// Fraction of the period the link is DOWN.
    pub blackout_fraction: f32,
    /// Phase offset of the blackout within the period, seconds.
    pub phase_s: f64,
}

impl LinkProfile {
    /// The Mars-surface → orbiter → Earth geometry from spec §6.
    #[must_use]
    pub fn mars_surface() -> Self {
        Self {
            one_way_delay_s: 720.0, // ~12 light-minutes
            blackout_period_s: 88_775.0,
            blackout_fraction: 0.45,
            phase_s: 0.0,
        }
    }

    /// A LEO relay pass schedule: 90-min orbit, contact ~12 min/pass.
    #[must_use]
    pub fn leo_relay() -> Self {
        Self {
            one_way_delay_s: 0.01,
            blackout_period_s: 5_400.0,
            blackout_fraction: 0.87,
            phase_s: 0.0,
        }
    }

    /// Whether the link is up at `t_s`.
    #[must_use]
    pub fn link_up(&self, t_s: f64) -> bool {
        if self.blackout_fraction <= 0.0 {
            return true;
        }
        let phase = (t_s - self.phase_s).rem_euclid(self.blackout_period_s);
        let down_len = self.blackout_period_s * f64::from(self.blackout_fraction);
        phase >= down_len
    }

    /// One-way delay at `t_s` (metre-accurate to the phase — constant in
    /// this model; the visualizer animates the variance).
    #[must_use]
    pub fn delay_s(&self, _t_s: f64) -> f64 {
        self.one_way_delay_s
    }

    /// Seconds until the link returns, given it is currently down.
    #[must_use]
    pub fn seconds_until_return(&self, t_s: f64) -> f64 {
        if self.link_up(t_s) {
            return 0.0;
        }
        let phase = (t_s - self.phase_s).rem_euclid(self.blackout_period_s);
        let down_len = self.blackout_period_s * f64::from(self.blackout_fraction);
        down_len - phase
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mars_geometry_blacks_out_on_schedule() {
        let p = LinkProfile::mars_surface();
        // Mid-blackout: phase 0..45 % of 88_775 s ≈ the first ~11 hours.
        assert!(!p.link_up(10_000.0), "in occultation");
        assert!(p.link_up(60_000.0), "window open");
        // Light time ~12 min.
        assert!((p.delay_s(0.0) - 720.0).abs() < 1.0);
        // Return time counts down to the window.
        let wait = p.seconds_until_return(10_000.0);
        assert!(wait > 0.0 && wait < p.blackout_period_s * 0.45);
        assert_eq!(p.seconds_until_return(60_000.0), 0.0);
    }

    #[test]
    fn leo_relay_is_mostly_dark_but_fast() {
        let p = LinkProfile::leo_relay();
        assert!((p.blackout_fraction - 0.87).abs() < 1e-6);
        assert!(!p.link_up(100.0), "87 % of the orbit is dark");
        assert!(p.link_up(4_800.0), "pass window");
        assert!(p.delay_s(0.0) < 0.1);
    }

    #[test]
    fn always_up_profile_never_blacks_out() {
        let p = LinkProfile {
            one_way_delay_s: 0.01,
            blackout_period_s: 5_400.0,
            blackout_fraction: 0.0,
            phase_s: 0.0,
        };
        for t in (0..1000).map(|k| k as f64 * 13.7) {
            assert!(p.link_up(t));
            assert_eq!(p.seconds_until_return(t), 0.0);
        }
    }
}
