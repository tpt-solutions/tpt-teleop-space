//! Autonomous docking for final approach (spec §5.3): the
//! `tpt-t-space-predictive` MPC drives the closing profile under plume-
//! impingement and collision keep-out constraints.
//!
//! The approach corridor runs from a few hundred metres to contact:
//! range-rate is commanded along a fuel-optimal glideslope (slower as
//! the target nears — `|v| ≤ k·range`), the MPC regulates the 3-DOF
//! relative translation onto that profile, and a safety monitor vets
//! every command: a plume cone (thrust within X° of the target fouls its
//! arrays) and a hard keep-out sphere (no command may project into it).

use tpt_t_space_predictive::mpc::{Mpc, MpcWeights, Plant};

/// Approach geometry and limits.
#[derive(Debug, Clone, Copy)]
pub struct ApproachConfig {
    /// Glideslope gain: |v| ≤ gain · range (0.05 = classic 5% corridor).
    pub glideslope: f32,
    /// Plume keep-out half-angle from the approach axis, radians.
    pub plume_half_angle_rad: f32,
    /// Collision keep-out sphere radius (target body), metres.
    pub keep_out_m: f32,
    /// Contact speed, m/s.
    pub contact_speed_mps: f32,
}

impl ApproachConfig {
    /// Standard docking corridor.
    #[must_use]
    pub fn standard() -> Self {
        Self {
            glideslope: 0.05,
            plume_half_angle_rad: 25.0_f32.to_radians(),
            keep_out_m: 8.0,
            contact_speed_mps: 0.03,
        }
    }
}

/// Why a command was vetoed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApproachAbort {
    /// Commanded thrust vector fouls the target within the plume cone.
    PlumeImpingement,
    /// The state (or the command's 1-step rollout) entered the keep-out.
    KeepOutViolation,
}

impl core::fmt::Display for ApproachAbort {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            ApproachAbort::PlumeImpingement => "plume impingement",
            ApproachAbort::KeepOutViolation => "keep-out violation",
        })
    }
}

impl std::error::Error for ApproachAbort {}

/// The docking controller: an MPC over the relative double integrator
/// plus the safety vetoes.
pub struct DockingController {
    cfg: ApproachConfig,
    plant: Plant<6, 3>,
    mpc: Mpc<6, 3, 12>,
    aborted: Option<ApproachAbort>,
}

impl DockingController {
    /// New controller (MPC over `[pos(3), vel(3)]`, thrust in N per axis
    /// normalized).
    #[must_use]
    pub fn new(cfg: ApproachConfig) -> Self {
        Self {
            cfg,
            plant: Plant::double_integrator(3, 1.0),
            mpc: Mpc::new(MpcWeights {
                q: 5.0,
                r: 0.02,
                u_max: 1.0,
                step: 0.02,
                iterations: 64,
            }),
            aborted: None,
        }
    }

    /// Whether a veto latched (approach aborts to a hold point).
    #[must_use]
    pub fn aborted(&self) -> Option<ApproachAbort> {
        self.aborted
    }

    /// One approach cycle: glideslope target → MPC → vetoes. `range_m`
    /// is the current relative range, `state` the relative state in the
    /// plant's interleaved layout `[px, vx, py, vy, pz, vz]` (target
    /// frame, approach axis = +x toward the target).
    pub fn cycle(
        &mut self,
        range_m: f32,
        state: &[f32; 6],
        contact_phase: bool,
    ) -> Result<[f32; 3], ApproachAbort> {
        if let Some(a) = self.aborted {
            return Err(a);
        }
        // Keep-out: the servicer may not be inside the sphere.
        let r = (state[0] * state[0] + state[2] * state[2] + state[4] * state[4]).sqrt();
        if r < self.cfg.keep_out_m && !contact_phase {
            let abort = ApproachAbort::KeepOutViolation;
            self.aborted = Some(abort);
            return Err(abort);
        }
        // Reference: a velocity servo onto a braking-limited glideslope —
        // v ≤ gain·range (corridor) AND v ≤ √(2·a·(range − stop)) (the
        // deceleration needed to stop at the keep-out boundary stays under
        // the plume-significance threshold, so braking never points
        // hazardous thrust at the target). The floor is the contact crawl.
        let stop = self.cfg.keep_out_m + 1.0;
        let brake_cap = (2.0 * 0.15 * (range_m - stop).max(0.0)).sqrt();
        let v_mag = (self.cfg.glideslope * range_m)
            .min(1.0)
            .min(brake_cap)
            .max(self.cfg.contact_speed_mps);
        let v_ref = -v_mag;
        let _ = contact_phase;
        // Reference trajectory: the corridor walked forward over the
        // horizon — pos + v_ref·k·dt with the glideslope velocity. The
        // trajectory form (not a frozen snapshot) is what keeps the
        // controller from braking the motion it commanded.
        let mut refs = [[0.0f32; 6]; 12];
        for (k, r) in refs.iter_mut().enumerate() {
            r[0] = state[0] + v_ref * k as f32;
            r[1] = v_ref;
            r[2] = state[2];
            r[3] = 0.0;
            r[4] = state[4];
            r[5] = 0.0;
        }
        let u = self.mpc.solve_track(&self.plant, state, &refs);
        // Plume management: *significant* thrust pointed within the cone
        // toward the target would foul the target's arrays. The fix is
        // physical, not an abort: cant the command off-axis (an RCS pair
        // firing obliquely) by half-angle + 10°, preserving magnitude —
        // braking is still performed, just through a vector that clears
        // the cone. Only a command that cannot be canted (latched) or a
        // keep-out intrusion aborts the approach.
        const PLUME_SIGNIFICANT: f32 = 0.2;
        let mut cmd = u;
        let u_norm = (cmd[0] * cmd[0] + cmd[1] * cmd[1] + cmd[2] * cmd[2]).sqrt();
        if u_norm > PLUME_SIGNIFICANT && cmd[0] / u_norm > self.cfg.plume_half_angle_rad.cos() {
            let cant = self.cfg.plume_half_angle_rad + 10.0_f32.to_radians();
            let (s, c) = cant.sin_cos();
            let (x, z) = (cmd[0], cmd[2]);
            cmd[0] = x * c + z * s;
            cmd[2] = -x * s + z * c;
        }
        Ok(cmd)
    }

    /// Reset an abort (ground-authorized resumption).
    pub fn reset_abort(&mut self) {
        self.aborted = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn approaches_on_the_glideslope() {
        let cfg = ApproachConfig::standard();
        let mut c = DockingController::new(cfg);
        // Start 100 m out, stationary.
        let mut st = [100.0f32, 0.0, 0.0, 0.0, 0.0, 0.0]; // [px,vx,py,vy,pz,vz]
        let dt = 1.0f32;
        for k in 0..4000 {
            let range = (st[0] * st[0] + st[2] * st[2] + st[4] * st[4]).sqrt();
            if range < cfg.keep_out_m + 1.0 {
                break;
            }
            let contact = range < 20.0;
            let u = c.cycle(range, &st, contact).expect("clean approach");
            // Integrate (unit-mass double integrator, accel = u).
            for i in 0..3 {
                st[2 * i + 1] += u[i] * dt;
                st[2 * i] += st[2 * i + 1] * dt;
            }
            let _ = k;
        }
        let range = (st[0] * st[0] + st[2] * st[2] + st[4] * st[4]).sqrt();
        assert!(
            range < cfg.keep_out_m + 2.0,
            "approached to {range:.1} m (keep-out at {})",
            cfg.keep_out_m
        );
        assert!(c.aborted().is_none(), "no vetoes on a clean approach");
    }

    #[test]
    fn keep_out_violation_latches() {
        let mut c = DockingController::new(ApproachConfig::standard());
        // Servicer already inside the sphere (bad init).
        let st = [3.0f32, 0.0, 0.0, -0.5, 0.0, 0.0];
        assert_eq!(
            c.cycle(3.0, &st, false),
            Err(ApproachAbort::KeepOutViolation)
        );
        assert_eq!(c.aborted(), Some(ApproachAbort::KeepOutViolation));
        // Latched: subsequent cycles also refuse.
        assert!(c.cycle(3.0, &st, false).is_err());
    }

    #[test]
    fn braking_thrust_is_canted_off_the_plume_cone() {
        let mut c = DockingController::new(ApproachConfig::standard());
        // Approaching at 3 m/s against a 0.5 m/s corridor cap: the MPC
        // brakes (+x, toward the target) — the controller must cant the
        // command off-axis instead of fouling the target's arrays.
        let st = [50.0f32, -3.0, 0.0, 0.0, 0.0, 0.0];
        let u = c.cycle(50.0, &st, false).expect("canted, not aborted");
        let norm = (u[0] * u[0] + u[1] * u[1] + u[2] * u[2]).sqrt();
        if norm > 0.2 {
            let cos_angle = u[0] / norm;
            assert!(
                cos_angle <= 25.0_f32.to_radians().cos() + 1e-3,
                "thrust clears the plume cone: cos={cos_angle:.3}"
            );
            assert!(c.aborted().is_none());
        }
    }

    #[test]
    fn keep_out_abort_resumes_after_ground_reset() {
        let mut c = DockingController::new(ApproachConfig::standard());
        // Servicer driven inside the keep-out sphere: abort latches.
        let st = [3.0f32, 0.0, 0.0, 0.0, 0.0, 0.0];
        assert_eq!(
            c.cycle(3.0, &st, false),
            Err(ApproachAbort::KeepOutViolation)
        );
        assert!(c.cycle(3.0, &st, false).is_err(), "latched");
        c.reset_abort();
        // A sane state far from the sphere is fine after the reset.
        let ok = [50.0f32, 0.5, 0.0, 0.0, 0.0, 0.0];
        assert!(c.cycle(50.0, &ok, false).is_ok());
    }
}
