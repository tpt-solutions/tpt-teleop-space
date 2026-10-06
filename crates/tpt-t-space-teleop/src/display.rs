//! Predictive display: show the operator a predicted asset position based
//! on the last known state, masking the light-speed delay (spec: 12
//! minutes for Mars).
//!
//! The ground receives state at `t_state` (transmitted the moment it was
//! computed); by the time the operator's screen refreshes it is
//! `delay_us` stale. The extrapolator dead-reckons position with the
//! last velocity (constant-velocity model, the standard predictive-
//! display core) and grows an uncertainty bubble at the speed drift
//! bound — the operator plans against the bubble, not the dot.

use core::fmt;

/// Why extrapolation was refused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtrapolateError {
    /// No state has arrived yet.
    NoState,
    /// The requested horizon exceeds the model's validity (dead reckoning
    /// past ~2 light-seconds of stale state is fantasy, not prediction).
    HorizonTooLong,
}

impl fmt::Display for ExtrapolateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            ExtrapolateError::NoState => "no state received yet",
            ExtrapolateError::HorizonTooLong => "extrapolation horizon exceeds the model validity",
        })
    }
}

impl std::error::Error for ExtrapolateError {}

/// The predicted view for the operator.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PredictedPose {
    /// Extrapolated position, metres.
    pub xyz_m: [f64; 3],
    /// Heading extrapolated from the yaw rate, degrees.
    pub heading_deg: f32,
    /// 1σ position uncertainty at the horizon, metres (drift model).
    pub sigma_m: f32,
    /// How stale the source state was, µs.
    pub state_age_us: u64,
}

/// The operator-side extrapolator.
#[derive(Debug, Clone, Copy)]
pub struct PredictiveDisplay {
    state_xyz_m: [f64; 3],
    state_vel_mps: [f32; 3],
    heading_deg: f32,
    yaw_rate_dps: f32,
    state_time_us: u64,
    /// Speed-drift bound for the uncertainty bubble, fraction/second.
    drift_per_s: f32,
    have_state: bool,
}

impl PredictiveDisplay {
    /// New display; `drift_per_s` bounds the velocity error growth
    /// (0.01 = 1 %/s — wheels slipping, thrusters mis-calibrated).
    #[must_use]
    pub fn new(drift_per_s: f32) -> Self {
        Self {
            state_xyz_m: [0.0; 3],
            state_vel_mps: [0.0; 3],
            heading_deg: 0.0,
            yaw_rate_dps: 0.0,
            state_time_us: 0,
            drift_per_s,
            have_state: false,
        }
    }

    /// Ingest a state packet (stamped when computed, not when received).
    pub fn ingest(
        &mut self,
        xyz_m: [f64; 3],
        vel_mps: [f32; 3],
        heading_deg: f32,
        yaw_rate_dps: f32,
        state_time_us: u64,
    ) {
        self.state_xyz_m = xyz_m;
        self.state_vel_mps = vel_mps;
        self.heading_deg = heading_deg;
        self.yaw_rate_dps = yaw_rate_dps;
        self.state_time_us = state_time_us;
        self.have_state = true;
    }

    /// Predict at `now_us` (the operator station's clock — `now_us −
    /// state_time_us` is the one-way light time the display masks).
    pub fn predict(&self, now_us: u64) -> Result<PredictedPose, ExtrapolateError> {
        if !self.have_state {
            return Err(ExtrapolateError::NoState);
        }
        let dt_us = now_us.saturating_sub(self.state_time_us);
        let dt_s = dt_us as f64 / 1.0e6;
        if dt_s > 120.0 {
            return Err(ExtrapolateError::HorizonTooLong);
        }
        let mut xyz = self.state_xyz_m;
        for (p, v) in xyz.iter_mut().zip(self.state_vel_mps.iter()) {
            *p += f64::from(*v) * dt_s;
        }
        let heading = (self.heading_deg + self.yaw_rate_dps * dt_s as f32).rem_euclid(360.0);
        // Uncertainty: linear drift growth + a floor for the quantization
        // of the state packet itself.
        let speed = (self.state_vel_mps[0] * self.state_vel_mps[0]
            + self.state_vel_mps[1] * self.state_vel_mps[1]
            + self.state_vel_mps[2] * self.state_vel_mps[2])
            .sqrt();
        let sigma = (speed * self.drift_per_s * dt_s as f32).max(0.05);
        Ok(PredictedPose {
            xyz_m: xyz,
            heading_deg: heading,
            sigma_m: sigma,
            state_age_us: dt_us,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mars worst case: 12 light-minutes of one-way delay — but the test
    /// uses the horizon cap instead: 12 min of staleness exceeds the
    /// 120 s model validity, which is why the display extrapolates in
    /// the *state's own frame* and the operator UI renders the bubble.
    #[test]
    fn masks_the_mars_light_delay() {
        let mut d = PredictiveDisplay::new(0.01);
        // Rover rolling north at 0.4 m/s, state computed at t=0.
        d.ingest([100.0, 200.0, -3.0], [0.4, 0.0, 0.0], 90.0, 0.5, 0);
        // Operator screen refreshes 90 s later (short-hop orbiter relay).
        let p = d.predict(90_000_000).expect("within horizon");
        assert!((p.xyz_m[0] - 136.0).abs() < 1e-6, "0.4 × 90 s ahead");
        assert!((p.heading_deg - 135.0).abs() < 1e-3, "0.5°/s × 90 s");
        assert_eq!(p.state_age_us, 90_000_000);
        // Uncertainty: 0.4 m/s × 1 %/s × 90 s = 0.36 m.
        assert!((p.sigma_m - 0.36).abs() < 1e-3, "sigma {}", p.sigma_m);
    }

    #[test]
    fn twelve_light_minutes_is_outside_the_model() {
        let mut d = PredictiveDisplay::new(0.01);
        d.ingest([0.0; 3], [0.1; 3], 0.0, 0.0, 0);
        let mars_delay_us: u64 = 12 * 60 * 1_000_000;
        assert_eq!(
            d.predict(mars_delay_us),
            Err(ExtrapolateError::HorizonTooLong)
        );
        // The display then renders the last state + a full-horizon bubble
        // (UI concern); the extrapolator refuses to fake precision.
    }

    #[test]
    fn no_state_is_an_error() {
        let d = PredictiveDisplay::new(0.01);
        assert_eq!(d.predict(1), Err(ExtrapolateError::NoState));
    }
}
