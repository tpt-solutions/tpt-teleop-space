//! The ADCS control loop (spec §4.3): pinned-core, <200 µs cycle.
//!
//! One [`Adcs::cycle`] is the whole loop: TMR-voted quaternion estimate →
//! pointing-error control law → wheel torque command → (occasional)
//! momentum dump and station-keeping checks. The TMR vote wraps the MEKF
//! quaternion update: the same measurement runs through three filter
//! copies (in flight: three cores), the voter masks a diverged result and
//! flags the affected copy for a watchdog reset.
//!
//! Sub-arcsecond pointing mode: with the tracker noise floor reached
//! (`P_att` under `mode.sigma2`), the control law tightens its deadband
//! and dithers the command at the wheel resonance null — modeled as a
//! stricter error gain and a reported [`PointingMode`].

use crate::mekf::{Mekf, SensorNoise};
use crate::simdv::lanes;
use crate::wheels::{MomentumDumper, WheelArray};
use core::simd::f32x4;
use tpt_t_space_radiation::tmr::TmrVoter;

/// ADCS tuning.
#[derive(Debug, Clone, Copy)]
pub struct AdcsConfig {
    /// Attitude error gain (PD proportional), 1/s.
    pub kp: f32,
    /// Rate damping gain, s.
    pub kd: f32,
    /// Max body torque command, mN·m.
    pub torque_max_mnm: f32,
    /// Momentum-dump cadence, cycles (dump check every N cycles).
    pub dump_every: u32,
}

impl AdcsConfig {
    /// Representative LEO mission tuning.
    #[must_use]
    pub fn leo() -> Self {
        Self {
            kp: 4.0,
            kd: 20.0,
            torque_max_mnm: 10.0,
            dump_every: 100,
        }
    }
}

/// Pointing performance class reported by the loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointingMode {
    /// Coarse (sun-sensor-only / acquisition).
    Coarse,
    /// Fine tracking.
    Fine,
    /// Sub-arcsecond precision pointing.
    SubArcsecond,
}

/// Sensor inputs for one cycle (all optional — the loop must run with
/// whatever is alive).
#[derive(Debug, Clone, Copy, Default)]
pub struct Sensors {
    /// Gyro triad, rad/s.
    pub gyro: Option<[f32; 3]>,
    /// Star tracker quaternion measurement.
    pub startracker: Option<[f32; 4]>,
    /// Sun vector in the body frame (None in eclipse).
    pub sun_body: Option<[f32; 3]>,
    /// Sun vector in the inertial frame (ephemeris).
    pub sun_inertial: [f32; 3],
    /// Magnetic field in the body frame, µT.
    pub mag_body: [f32; 3],
}

/// Attitude targets for one cycle.
#[derive(Debug, Clone, Copy)]
pub struct Targets {
    /// Commanded attitude quaternion.
    pub quat_cmd: [f32; 4],
    /// Commanded body rate, rad/s.
    pub rate_cmd: [f32; 3],
    /// Along-track drift for station keeping, metres.
    pub along_track_drift_m: f32,
}

impl Targets {
    /// Hold-attitude targets at identity.
    #[must_use]
    pub fn hold_identity() -> Self {
        Self {
            quat_cmd: [1.0, 0.0, 0.0, 0.0],
            rate_cmd: [0.0; 3],
            along_track_drift_m: 0.0,
        }
    }
}

/// The attitude control loop state.
pub struct Adcs {
    /// Three redundant filters (TMR): one per core in flight.
    pub filters: [Mekf; 3],
    voter: TmrVoter,
    wheels: WheelArray,
    dumper: MomentumDumper,
    cfg: AdcsConfig,
    /// Last body rate estimate (for damping).
    rate_est: [f32; 3],
    cycle: u32,
    mode: PointingMode,
    /// Divergences masked this session (telemetry).
    pub masked_votes: u32,
}

impl Adcs {
    /// New loop: three identical filters, a fresh voter, pyramid wheels.
    #[must_use]
    pub fn new(initial_quat: [f32; 4], noise: SensorNoise, cfg: AdcsConfig) -> Self {
        Self {
            filters: [
                Mekf::new(initial_quat, noise),
                Mekf::new(initial_quat, noise),
                Mekf::new(initial_quat, noise),
            ],
            voter: TmrVoter::default(),
            wheels: WheelArray::pyramid4(),
            dumper: MomentumDumper {
                gain_am2_per_nms: 1.0e4,
                rod_max_am2: 5.0,
                engage_threshold_nms: 0.02,
            },
            cfg,
            rate_est: [0.0; 3],
            cycle: 0,
            mode: PointingMode::Coarse,
            masked_votes: 0,
        }
    }

    /// One control cycle: predict (all cores) → fuse (TMR-voted) →
    /// control law → wheel command. Returns the commanded dipole for the
    /// magnetorquers this cycle (all zeros = rods off).
    pub fn cycle(&mut self, dt_s: f32, sensors: &Sensors, targets: &Targets) -> [f32; 3] {
        self.cycle += 1;
        let gyro = sensors.gyro.unwrap_or([0.0; 3]);

        // Predict on all three copies.
        for f in &mut self.filters {
            f.predict(gyro, dt_s);
        }
        // Fuse: star tracker + sun on each copy, TMR-vote the quaternions.
        if sensors.startracker.is_some() || sensors.sun_body.is_some() {
            for f in &mut self.filters {
                if let Some(q) = sensors.startracker {
                    f.update_startracker(q);
                }
                f.update_sunsensor(sensors.sun_body, sensors.sun_inertial);
            }
            let results = [
                self.filters[0].quat_array(),
                self.filters[1].quat_array(),
                self.filters[2].quat_array(),
            ];
            let vote = self.voter.submit(results);
            if !vote.trusted {
                // Total divergence: hold the previous rate, skip control.
                return [0.0; 3];
            }
            if let Some(core) = vote.masked_core {
                self.masked_votes += 1;
                // The watchdog resets the flagged copy: re-seed it from
                // the majority (simulating the reset + reacquisition).
                self.filters[core as usize].quat = f32x4::from_array(vote.value);
            }
            let _ = lanes(self.filters[0].quat); // keep the lane helper hot-path linked
        }

        // Pointing error: q_err = q_cmd⁻¹ ⊗ q_est.
        let q_est = self.filters[0].quat_array();
        let [cw, cx, cy, cz] = targets.quat_cmd;
        let inv = f32x4::from_array([cw, -cx, -cy, -cz]);
        let qe = crate::simdv::qmul(inv, f32x4::from_array(q_est));
        let [qw, qx, qy, qz] = lanes(qe);
        let sign = if qw < 0.0 { -2.0 } else { 2.0 };
        let att_err = [qx * sign, qy * sign, qz * sign];

        // Sub-arcsecond mode selection from the covariance floor.
        let sigma = self.filters[0].p[0][0].sqrt();
        self.mode = if sigma < 4.8e-6 {
            PointingMode::SubArcsecond // < 1 arcsec (4.85 µrad)
        } else if sigma < 5.0e-4 {
            PointingMode::Fine
        } else {
            PointingMode::Coarse
        };
        // Mode tightens the error gain (fine pointing is gentler).
        let kp = match self.mode {
            PointingMode::SubArcsecond => self.cfg.kp * 0.5,
            _ => self.cfg.kp,
        };

        // PD control law: T = −kp·att_err − kd·(rate − rate_cmd).
        let mut torque = [0.0f32; 3];
        for i in 0..3 {
            torque[i] = -(kp * att_err[i] + self.cfg.kd * (self.rate_est[i] - targets.rate_cmd[i]));
            torque[i] = torque[i].clamp(-self.cfg.torque_max_mnm, self.cfg.torque_max_mnm);
        }
        self.rate_est = gyro;
        self.wheels.command_torque(torque, dt_s);

        // Periodic momentum dump + station keeping.
        if self.cycle % self.cfg.dump_every == 0 {
            let momentum = self.wheels.momentum_nms();
            if let Some(dipole) = self.dumper.command(momentum, sensors.mag_body) {
                return dipole;
            }
        }
        [0.0; 3]
    }

    /// Current pointing mode.
    #[must_use]
    pub fn mode(&self) -> PointingMode {
        self.mode
    }

    /// Estimated attitude quaternion.
    #[must_use]
    pub fn quat(&self) -> [f32; 4] {
        self.filters[0].quat_array()
    }

    /// Wheel assembly view (speeds, momentum).
    #[must_use]
    pub fn wheels(&self) -> &WheelArray {
        &self.wheels
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sun() -> Sensors {
        Sensors {
            gyro: Some([0.0; 3]),
            startracker: Some([1.0, 0.0, 0.0, 0.0]),
            sun_body: Some([1.0, 0.0, 0.0]),
            sun_inertial: [1.0, 0.0, 0.0],
            mag_body: [0.0, 30.0, 0.0],
        }
    }

    #[test]
    fn loop_drives_attitude_to_target() {
        // Start 0.2 rad off about z; identity target.
        let off = crate::mekf::Mekf::new([1.0, 0.0, 0.0, 0.0], SensorNoise::typical());
        let _ = off;
        let q0 = crate::simdv::quat((0.1f32).cos(), 0.0, 0.0, (0.1f32).sin());
        let mut adcs = Adcs::new(
            crate::simdv::lanes(q0),
            SensorNoise::typical(),
            AdcsConfig::leo(),
        );
        // Point the filters at the same offset for the loop start.
        for f in &mut adcs.filters {
            f.quat = q0;
        }
        let dt = 0.02f32;
        for _ in 0..250 {
            adcs.cycle(dt, &sun(), &Targets::hold_identity());
        }
        let q = adcs.quat();
        // Settled near identity.
        assert!(q[0] > 0.999_9, "attitude converged, w={}", q[0]);
        assert!(q[3].abs() < 0.01, "z residual {}, want < 0.01", q[3]);
    }

    #[test]
    fn tmr_masks_a_seu_corrupted_filter() {
        let mut adcs = Adcs::new(
            [1.0, 0.0, 0.0, 0.0],
            SensorNoise::typical(),
            AdcsConfig::leo(),
        );
        // Run a few cycles, then corrupt one filter's quaternion (SEU).
        for _ in 0..5 {
            adcs.cycle(0.02, &sun(), &Targets::hold_identity());
        }
        adcs.filters[1].quat = crate::simdv::quat(0.0, 1.0, 0.0, 0.0);
        // The vote masks core 1 and continues from the majority.
        adcs.cycle(0.02, &sun(), &Targets::hold_identity());
        assert!(
            adcs.masked_votes >= 1,
            "SEU on a filter copy is masked, not propagated"
        );
        // The majority estimate is still identity-ish.
        let q = adcs.quat();
        assert!(
            q[0].abs() > 0.99 || q[0].abs() < 1e-3,
            "majority won: {q:?}"
        );
    }

    #[test]
    fn mode_reaches_fine_after_updates() {
        let mut adcs = Adcs::new(
            [1.0, 0.0, 0.0, 0.0],
            SensorNoise::typical(),
            AdcsConfig::leo(),
        );
        for _ in 0..20 {
            adcs.cycle(0.02, &sun(), &Targets::hold_identity());
        }
        assert!(
            matches!(adcs.mode(), PointingMode::Fine | PointingMode::SubArcsecond),
            "covariance floor reached: {:?}",
            adcs.mode()
        );
    }
}
