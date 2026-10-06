//! Multi-satellite formation flying: relative orbit maintenance over
//! inter-satellite links (spec §5.2).
//!
//! Relative motion uses the Clohessy–Wiltshire linearization about a
//! circular reference orbit; each follower fuses ISL range measurements
//! (and inter-satellite bearing when available) into its relative state,
//! and a bounded PD controller maintains the desired offset for
//! interferometry / SAR formation geometry.

/// CW frame: x along-track, y cross-track, z radial; reference orbit
/// angular rate `n`.
#[derive(Debug, Clone, Copy)]
pub struct CwModel {
    /// Reference orbit mean motion, rad/s.
    pub n: f32,
}

impl CwModel {
    /// Step the relative state `(pos[3], vel[3])` by `dt` (RK-free
    /// first-order is adequate at control rates for the drift terms).
    #[must_use]
    pub fn step(&self, pos: &[f32; 3], vel: &[f32; 3], dt_s: f32) -> ([f32; 3], [f32; 3]) {
        let (x, _y, z) = (pos[0], pos[1], pos[2]);
        let n = self.n;
        // CW accelerations (unforced).
        let ax = 3.0 * n * n * x + 2.0 * n * vel[2];
        let ay = 0.0;
        let az = -2.0 * n * vel[0] + n * n * z;
        let mut nvel = *vel;
        nvel[0] += ax * dt_s;
        nvel[1] += ay * dt_s;
        nvel[2] += az * dt_s;
        let mut npos = *pos;
        for i in 0..3 {
            npos[i] += nvel[i] * dt_s;
        }
        (npos, nvel)
    }
}

/// One follower satellite's relative-nav + control state.
pub struct Follower {
    /// Desired offset from the chief, metres (formation geometry).
    pub offset_cmd_m: [f32; 3],
    /// Estimated relative position (chief ← follower), metres.
    rel_pos: [f32; 3],
    /// Estimated relative velocity, m/s.
    rel_vel: [f32; 3],
    /// Control gains (PD).
    kp: f32,
    kd: f32,
    /// Max ΔV per correction, m/s.
    dv_max: f32,
    /// ISL updates received.
    isl_updates: u32,
}

impl Follower {
    /// New follower at some initial relative state.
    #[must_use]
    pub fn new(offset_cmd_m: [f32; 3], init_pos: [f32; 3], init_vel: [f32; 3]) -> Self {
        Self {
            offset_cmd_m,
            rel_pos: init_pos,
            rel_vel: init_vel,
            kp: 2.0e-3,
            kd: 2.0e-2,
            dv_max: 0.02,
            isl_updates: 0,
        }
    }

    /// Fuse one ISL range measurement (chief ↔ follower distance) by
    /// scaling the estimated range toward the measured one (a bearing-
    /// free update: the range error distributes over the line of sight).
    pub fn update_isl_range(&mut self, measured_range_m: f32) {
        let est = (self.rel_pos.iter().map(|p| p * p).sum::<f32>()).sqrt();
        if est < 1e-6 {
            self.isl_updates += 1;
            return;
        }
        let scale = measured_range_m / est;
        for p in &mut self.rel_pos {
            *p *= 0.9 + 0.1 * scale; // damped fusion
        }
        self.isl_updates += 1;
    }

    /// Advance one control cycle: propagate the CW dynamics, apply the
    /// PD correction bounded by `dv_max`. Returns the commanded ΔV.
    pub fn step(&mut self, model: &CwModel, dt_s: f32) -> [f32; 3] {
        (self.rel_pos, self.rel_vel) = model.step(&self.rel_pos, &self.rel_vel, dt_s);
        let mut dv = [0.0f32; 3];
        for i in 0..3 {
            let err = self.offset_cmd_m[i] - self.rel_pos[i];
            dv[i] = (self.kp * err - self.kd * self.rel_vel[i]).clamp(-self.dv_max, self.dv_max);
            self.rel_vel[i] += dv[i];
        }
        dv
    }

    /// Current relative position estimate.
    #[must_use]
    pub fn rel_pos(&self) -> [f32; 3] {
        self.rel_pos
    }

    /// ISL updates received.
    #[must_use]
    pub fn isl_updates(&self) -> u32 {
        self.isl_updates
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> CwModel {
        CwModel {
            n: 2.0 * core::f32::consts::PI / 5400.0, // LEO ~90 min
        }
    }

    #[test]
    fn follower_holds_the_formation_slot() {
        let m = model();
        let mut f = Follower::new(
            [500.0, 0.0, 0.0], // 500 m along-track slot
            [500.0, 0.0, 0.0],
            [0.0; 3],
        );
        let dt = 1.0f32;
        for _ in 0..3000 {
            f.update_isl_range(500.0); // perfect ISL ranges
            f.step(&m, dt);
        }
        let p = f.rel_pos();
        let err = (p[0] - 500.0).abs().max(p[1].abs()).max(p[2].abs());
        assert!(err < 5.0, "holds the slot within 5 m, got {err:.2} m");
        assert!(f.isl_updates() == 3000);
    }

    #[test]
    fn follower_recovers_from_a_drift() {
        let m = model();
        // Inserted 100 m off-slot with cross-track velocity.
        let mut f = Follower::new([500.0, 0.0, 0.0], [600.0, 30.0, 10.0], [0.05, -0.02, 0.0]);
        let dt = 1.0f32;
        for k in 0..6000 {
            let range = 500.0 + if k < 3000 { 90.0 } else { 0.0 };
            f.update_isl_range(range);
            f.step(&m, dt);
        }
        let p = f.rel_pos();
        let err = (p[0] - 500.0).abs().max(p[1].abs()).max(p[2].abs());
        assert!(err < 10.0, "recovered to the slot, residual {err:.2} m");
    }

    #[test]
    fn cw_free_drift_along_track_grows() {
        // Pure along-track offset with zero velocity drifts slowly (the
        // 3n²x term) — sanity on the dynamics signs.
        let m = model();
        let (p, v) = ([100.0, 0.0, 0.0], [0.0; 3]);
        let (p2, _v2) = m.step(&p, &v, 1.0);
        assert!(p2[0] > 100.0, "3n²x accelerates outward along-track");
    }
}
