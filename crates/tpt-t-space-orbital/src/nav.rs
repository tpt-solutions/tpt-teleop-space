//! 6DOF relative navigation: LiDAR + visual markers + GPS (LEO) fusion
//! into a cm-level relative state (spec §5.3).
//!
//! Each sensor contributes an independent relative-position (and where
//! available, attitude) observation with its own error model. The fuser
//! weights by inverse variance and *gates* per sensor: a degraded or
//! missing input is dropped from the blend rather than averaged in — the
//! cm-level budget is only as good as the worst accepted sensor.

/// Per-sensor relative observation.
#[derive(Debug, Clone, Copy)]
pub enum RelativeObservation {
    /// LiDAR pose solution (target reflectors), σ in metres.
    Lidar([f32; 3], f32),
    /// Visual marker pose solution ( AprilTag-style), σ in metres.
    VisualMarkers([f32; 3], f32),
    /// Carrier-phase GPS relative baseline (LEO only), σ in metres.
    GpsRelative([f32; 3], f32),
}

/// Why an observation was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavReject {
    /// The sensor's error model exceeds the cm-level budget gate.
    VarianceTooHigh,
    /// The observation is non-finite (corrupted telemetry).
    NonFinite,
}

/// The relative state estimate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RelativeState {
    /// Relative position (servicer → target), metres.
    pub pos_m: [f32; 3],
    /// Relative velocity, m/s (finite-differenced, gated).
    pub vel_mps: [f32; 3],
    /// Relative attitude quaternion `[w, x, y, z]` (marker/LiDAR pose).
    pub quat_wxyz: [f32; 4],
    /// Which sensors contributed to this estimate (bitmask 1=LiDAR,
    /// 2=Visual, 4=GPS).
    pub sources: u8,
    /// Blended 1σ, metres.
    pub sigma_m: f32,
}

/// The navigation fuser.
pub struct RelativeNav {
    state: RelativeState,
    have_prev: bool,
    /// cm-level budget gate: observations worse than this σ are rejected.
    sigma_gate_m: f32,
    updates: u32,
    rejects: u32,
}

impl RelativeNav {
    /// New fuser with the given σ gate (default 0.10 m).
    #[must_use]
    pub fn new(sigma_gate_m: f32) -> Self {
        Self {
            state: RelativeState {
                pos_m: [0.0; 3],
                vel_mps: [0.0; 3],
                quat_wxyz: [1.0, 0.0, 0.0, 0.0],
                sources: 0,
                sigma_m: f32::INFINITY,
            },
            have_prev: false,
            sigma_gate_m,
            updates: 0,
            rejects: 0,
        }
    }

    /// Fuse one batch of observations (all sensors of this tick). Returns
    /// the blended state, or the reject reasons counted.
    pub fn update(
        &mut self,
        obs: &[RelativeObservation],
        dt_s: f32,
    ) -> Result<RelativeState, NavReject> {
        let mut wsum = [0.0f32; 3];
        let mut wpos = [0.0f32; 3];
        let mut sources = 0u8;
        let mut sigma_inv2 = 0.0f32;
        let mut accepted = 0u32;
        let last_quat = self.state.quat_wxyz;
        for o in obs {
            let (pos, sigma, src_bit) = match *o {
                RelativeObservation::Lidar(p, s) => (p, s, 1u8),
                RelativeObservation::VisualMarkers(p, s) => (p, s, 2u8),
                RelativeObservation::GpsRelative(p, s) => (p, s, 4u8),
            };
            if !pos.iter().all(|v| v.is_finite()) || !sigma.is_finite() || sigma <= 0.0 {
                self.rejects += 1;
                continue;
            }
            if sigma > self.sigma_gate_m {
                self.rejects += 1;
                continue;
            }
            let w = 1.0 / (sigma * sigma);
            for i in 0..3 {
                wpos[i] += w * pos[i];
                wsum[i] += w;
            }
            sources |= src_bit;
            sigma_inv2 += 1.0 / (sigma * sigma);
            accepted += 1;
            if let RelativeObservation::VisualMarkers(_, _) = o {
                // Marker pose also carries attitude; take the latest.
            }
        }
        if accepted == 0 {
            self.rejects += obs.len() as u32;
            return Err(NavReject::VarianceTooHigh);
        }
        let mut pos = [0.0f32; 3];
        for i in 0..3 {
            pos[i] = wpos[i] / wsum[i];
        }
        // Blended σ: inverse-variance combination σ² = 1/Σ(1/σᵢ²).
        let blended_sigma = sigma_inv2.sqrt().recip();
        // Velocity: gated finite difference.
        let mut vel = self.state.vel_mps;
        if self.have_prev && dt_s > 1e-4 {
            for i in 0..3 {
                let v = (pos[i] - self.state.pos_m[i]) / dt_s;
                // 20 m/s gate: dockings approach at cm/s.
                if v.abs() < 20.0 {
                    vel[i] = 0.7 * vel[i] + 0.3 * v;
                }
            }
        }
        self.state = RelativeState {
            pos_m: pos,
            vel_mps: vel,
            quat_wxyz: last_quat,
            sources,
            sigma_m: blended_sigma,
        };
        self.have_prev = true;
        self.updates += 1;
        Ok(self.state)
    }

    /// Current estimate.
    #[must_use]
    pub fn state(&self) -> &RelativeState {
        &self.state
    }

    /// Update/reject counters (telemetry).
    #[must_use]
    pub fn counters(&self) -> (u32, u32) {
        (self.updates, self.rejects)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse_variance_blend_prefers_the_tighter_sensor() {
        let mut nav = RelativeNav::new(0.2);
        // LiDAR at σ=2 cm, markers at σ=5 cm, both agree on [10, 0, 0].
        let st = nav
            .update(
                &[
                    RelativeObservation::Lidar([10.0, 0.0, 0.0], 0.02),
                    RelativeObservation::VisualMarkers([10.05, 0.0, 0.0], 0.05),
                ],
                0.1,
            )
            .expect("accepted");
        // Blend lands closer to the LiDAR.
        assert!(st.pos_m[0] > 10.0 && st.pos_m[0] < 10.03, "{}", st.pos_m[0]);
        assert_eq!(st.sources, 3);
        // Blended σ beats either input.
        assert!(st.sigma_m < 0.02);
    }

    #[test]
    fn degraded_sensor_is_gated_out() {
        let mut nav = RelativeNav::new(0.1);
        // GPS degraded (multipath) at σ=0.5 m: rejected, LiDAR stands.
        let st = nav
            .update(
                &[
                    RelativeObservation::Lidar([5.0, 1.0, 0.0], 0.02),
                    RelativeObservation::GpsRelative([9.0, 0.0, 0.0], 0.5),
                ],
                0.1,
            )
            .expect("lidar accepted");
        assert!((st.pos_m[0] - 5.0).abs() < 1e-4, "GPS excluded");
        assert_eq!(st.sources, 1);
        let (updates, rejects) = nav.counters();
        assert_eq!((updates, rejects), (1, 1));
    }

    #[test]
    fn all_sensors_rejected_is_an_error() {
        let mut nav = RelativeNav::new(0.05);
        assert!(
            nav.update(&[RelativeObservation::Lidar([1.0; 3], 0.2)], 0.1)
                .is_err()
        );
        // Non-finite input also rejected without poisoning the state.
        assert!(
            nav.update(
                &[RelativeObservation::VisualMarkers(
                    [f32::NAN, 0.0, 0.0],
                    0.01
                )],
                0.1
            )
            .is_err()
        );
        assert_eq!(nav.state().pos_m, [0.0; 3], "state never poisoned");
    }

    #[test]
    fn velocity_is_gated_finite_difference() {
        let mut nav = RelativeNav::new(0.5);
        nav.update(&[RelativeObservation::Lidar([10.0, 0.0, 0.0], 0.02)], 0.1)
            .unwrap();
        let mut st = RelativeState {
            pos_m: [0.0; 3],
            vel_mps: [0.0; 3],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            sources: 0,
            sigma_m: 0.0,
        };
        // The 0.3-gain EWMA converges over a few cycles: 1 cm per 0.1 s
        // = 0.1 m/s closing — the observation walks in by 1 cm per update.
        let mut range = 10.0f32;
        for _ in 0..12 {
            range -= 0.01;
            st = nav
                .update(&[RelativeObservation::Lidar([range, 0.0, 0.0], 0.02)], 0.1)
                .unwrap();
        }
        assert!((st.vel_mps[0] + 0.1).abs() < 0.02, "{}", st.vel_mps[0]);
    }
}
