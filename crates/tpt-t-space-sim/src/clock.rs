//! The variable-speed simulation clock (spec §8): multi-year cruise
//! phases or complex rendezvous maneuvers in minutes of wall time.

use core::time::Duration;

/// The sim clock: simulated time advances at a configurable multiple of
/// wall time, with manual steps for the batch tests.
#[derive(Debug, Clone, Copy)]
pub struct SimClock {
    /// Simulation time, seconds since epoch.
    sim_s: f64,
    /// Speed factor (1 = real time, 3_600 = 1 hour per second).
    speed: f64,
    /// Wall time consumed, seconds (for the throughput telemetry).
    wall_s: f64,
}

impl SimClock {
    /// New clock at epoch with the given speed.
    #[must_use]
    pub fn new(speed: f64) -> Self {
        Self {
            sim_s: 0.0,
            speed: speed.max(1e-9),
            wall_s: 0.0,
        }
    }

    /// Simulated time, seconds.
    #[must_use]
    pub fn sim_s(&self) -> f64 {
        self.sim_s
    }

    /// Simulated time in days (cruise phases read better in days).
    #[must_use]
    pub fn sim_days(&self) -> f64 {
        self.sim_s / 86_400.0
    }

    /// Speed factor.
    #[must_use]
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Change speed mid-scenario (rendezvous fine-trims drop to 1×).
    pub fn set_speed(&mut self, speed: f64) {
        self.speed = speed.max(1e-9);
    }

    /// Advance by a wall-time duration at the current speed.
    pub fn advance(&mut self, wall: Duration) {
        self.wall_s += wall.as_secs_f64();
        self.sim_s += wall.as_secs_f64() * self.speed;
    }

    /// Batch step: advance the simulation directly by `sim_s` seconds
    /// (headless tests don't wait for wall time).
    pub fn step_sim(&mut self, sim_s: f64) {
        self.sim_s += sim_s;
        self.wall_s += sim_s / self.speed;
    }

    /// Throughput: simulated seconds per wall second so far.
    #[must_use]
    pub fn throughput(&self) -> f64 {
        if self.wall_s <= 0.0 {
            return self.speed;
        }
        self.sim_s / self.wall_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn speed_factor_scales_time() {
        let mut c = SimClock::new(3_600.0); // 1 h/s
        c.advance(Duration::from_secs(60)); // 1 wall minute
        assert!((c.sim_s() - 3_600.0 * 60.0).abs() < 1e-6, "1 hour/min");
        assert!((c.throughput() - 3_600.0).abs() < 1e-6);
    }

    #[test]
    fn batch_steps_cover_multi_year_cruises() {
        let mut c = SimClock::new(1e7);
        // A 7-month cruise to Mars ≈ 212 days.
        while c.sim_days() < 212.0 {
            c.step_sim(3_600.0); // 1-hour steps
        }
        assert!(c.sim_days() >= 212.0);
        // Wall time at 1e7×: 212 days of sim = ~1.8 wall seconds.
        assert!(
            c.wall_s < 5.0,
            "years of cruise in seconds of wall: {} s",
            c.wall_s
        );
    }

    #[test]
    fn speed_changes_take_effect_immediately() {
        let mut c = SimClock::new(1.0);
        c.advance(Duration::from_secs(1));
        assert!((c.sim_s() - 1.0).abs() < 1e-9);
        c.set_speed(100.0);
        c.advance(Duration::from_secs(1));
        assert!((c.sim_s() - 101.0).abs() < 1e-6);
    }
}
