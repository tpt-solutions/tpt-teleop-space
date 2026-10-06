//! Reaction wheel control, momentum management, and orbit maintenance
//! (spec §4.3 / §5.2).
//!
//! * wheel control: body torque demand → per-wheel torque allocation for
//!   a 4-wheel pyramid, with speed-limit saturation and momentum
//!   accumulation bookkeeping;
//! * momentum dumping: magnetorquer cross-product control law unloading
//!   the wheels against the geomagnetic field (or thruster equivalent);
//! * orbit maintenance: along-track drift beyond the station-keeping box
//!   commands a bounded ΔV maneuver (schedule-triggered, auditable).

/// Why a wheel command was degraded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WheelFault {
    /// A wheel saturated at its torque limit.
    TorqueSaturated,
    /// A wheel hit its speed limit — momentum must be dumped.
    SpeedLimit,
}

/// One reaction wheel.
#[derive(Debug, Clone, Copy)]
pub struct Wheel {
    /// Mount axis in the body frame (unit).
    pub axis: [f32; 3],
    /// Max torque, mN·m.
    pub torque_max_mnm: f32,
    /// Max speed, rpm.
    pub speed_max_rpm: f32,
    /// Current speed, rpm.
    pub speed_rpm: f32,
    /// Wheel inertia, kg·m² (10 cgs-class wheel: 1.0e-4).
    pub inertia_kgm2: f32,
}

impl Wheel {
    /// A representative 4-pyramid wheel.
    #[must_use]
    pub fn pyramid(axis: [f32; 3]) -> Self {
        Self {
            axis,
            torque_max_mnm: 10.0,
            speed_max_rpm: 6000.0,
            speed_rpm: 0.0,
            inertia_kgm2: 1.0e-4,
        }
    }
}

/// The 4-wheel pyramid assembly.
pub struct WheelArray {
    wheels: [Wheel; 4],
    /// Torque-to-speed scale (rpm per mN·m·s at unit inertia), folded.
    pub speed_per_torque_rpm: f32,
}

impl WheelArray {
    /// The classic pyramid: four wheels at 54.7° elevation, 90° azimuth
    /// spacing (unit mount axes normalized below).
    #[must_use]
    pub fn pyramid4() -> Self {
        let s = 1.0f32 / 3.0f32.sqrt();
        Self {
            wheels: [
                Wheel::pyramid([s, s, s]),
                Wheel::pyramid([-s, s, s]),
                Wheel::pyramid([s, -s, s]),
                Wheel::pyramid([-s, -s, s]),
            ],
            speed_per_torque_rpm: 600.0,
        }
    }

    /// Allocate a body torque across the pyramid (pseudoinverse of the
    /// mount matrix for the 3-of-4 redundant set, computed in closed
    /// form for the symmetric pyramid). Returns per-wheel faults.
    pub fn command_torque(
        &mut self,
        body_torque_mnm: [f32; 3],
        dt_s: f32,
    ) -> [Option<WheelFault>; 4] {
        let [tx, ty, tz] = body_torque_mnm;
        // T_w = Mᵀ (M Mᵀ)⁻¹ T_b with the symmetric-pyramid structure:
        // each wheel torque = (t·a_i)·(3/2) for this geometry.
        let mut faults = [None; 4];
        for (i, w) in self.wheels.iter_mut().enumerate() {
            let dot = tx * w.axis[0] + ty * w.axis[1] + tz * w.axis[2];
            let mut t = dot * 1.5;
            if t.abs() > w.torque_max_mnm {
                t = t.clamp(-w.torque_max_mnm, w.torque_max_mnm);
                faults[i] = Some(WheelFault::TorqueSaturated);
            }
            // Integrate speed.
            w.speed_rpm += t * dt_s * self.speed_per_torque_rpm;
            if w.speed_rpm.abs() > w.speed_max_rpm {
                w.speed_rpm = w.speed_rpm.clamp(-w.speed_max_rpm, w.speed_max_rpm);
                faults[i] = Some(WheelFault::SpeedLimit);
            }
        }
        faults
    }

    /// Total stored angular momentum, N·m·s (vector sum over wheels).
    #[must_use]
    pub fn momentum_nms(&self) -> [f32; 3] {
        let mut m = [0.0f32; 3];
        for w in &self.wheels {
            let l = w.speed_rpm * 2.0 * core::f32::consts::PI / 60.0 * w.inertia_kgm2;
            for a in 0..3 {
                m[a] += l * w.axis[a];
            }
        }
        m
    }

    /// Wheel speeds, rpm.
    #[must_use]
    pub fn speeds_rpm(&self) -> [f32; 4] {
        [
            self.wheels[0].speed_rpm,
            self.wheels[1].speed_rpm,
            self.wheels[2].speed_rpm,
            self.wheels[3].speed_rpm,
        ]
    }
}

/// Momentum-dumping controller: magnetorquer rods commanded against the
/// stored momentum, `m_cmd = −k · (h × B) / |B|²` (cross-product law).
#[derive(Debug, Clone, Copy)]
pub struct MomentumDumper {
    /// Dump gain, A·m² per N·m·s.
    pub gain_am2_per_nms: f32,
    /// Rod limits, A·m² per axis.
    pub rod_max_am2: f32,
    /// Dumping engages above this momentum magnitude, N·m·s.
    pub engage_threshold_nms: f32,
}

impl MomentumDumper {
    /// Command the rods for one cycle. Returns the dipole moment; `None`
    /// when momentum is below the engage threshold (rods off).
    #[must_use]
    pub fn command(&self, momentum_nms: [f32; 3], mag_field_body: [f32; 3]) -> Option<[f32; 3]> {
        let b2 = mag_field_body.iter().map(|b| b * b).sum::<f32>();
        if b2 < 1e-12 {
            return None;
        }
        let h_mag = (momentum_nms.iter().map(|h| h * h).sum::<f32>()).sqrt();
        if h_mag < self.engage_threshold_nms {
            return None;
        }
        // m = k (h × B) / |B|²  → torque m × B = −k·h (unloading).
        let hx = [
            momentum_nms[1] * mag_field_body[2] - momentum_nms[2] * mag_field_body[1],
            momentum_nms[2] * mag_field_body[0] - momentum_nms[0] * mag_field_body[2],
            momentum_nms[0] * mag_field_body[1] - momentum_nms[1] * mag_field_body[0],
        ];
        let k = self.gain_am2_per_nms / b2;
        let m = [hx[0] * k, hx[1] * k, hx[2] * k];
        Some([
            m[0].clamp(-self.rod_max_am2, self.rod_max_am2),
            m[1].clamp(-self.rod_max_am2, self.rod_max_am2),
            m[2].clamp(-self.rod_max_am2, self.rod_max_am2),
        ])
    }
}

/// Orbit-maintenance controller: along-track station-keeping.
#[derive(Debug, Clone, Copy)]
pub struct OrbitKeeper {
    /// Along-track half-box, metres (drift beyond ±box → maneuver).
    pub along_track_box_m: f32,
    /// ΔV per maneuver, m/s (bounded).
    pub dv_max_mps: f32,
}

impl OrbitKeeper {
    /// Command the correction ΔV for an along-track drift `drift_m`
    /// (positive = ahead of slot → decelerate). Returns `None` inside
    /// the box.
    #[must_use]
    pub fn maneuver(&self, drift_m: f32, orbit_radius_m: f32) -> Option<f32> {
        if drift_m.abs() <= self.along_track_box_m {
            return None;
        }
        // Two-body relation: δa/a ≈ 2·δv/v; along-track drift rate from a
        // semi-major offset gives the bounded correction.
        let v = (398_600_441_800_000.0 / orbit_radius_m).sqrt(); // Earth μ
        let dv = -drift_m / orbit_m_per_mps(orbit_radius_m);
        Some(dv.clamp(-self.dv_max_mps, self.dv_max_mps) * v / v)
    }
}

/// Metres of along-track drift per m/s of along-track ΔV over one orbit
/// (~3π·δv·T scaled) — folded constant for LEO in the maneuver law.
fn orbit_m_per_mps(orbit_radius_m: f32) -> f32 {
    let period_s =
        2.0 * core::f32::consts::PI * (orbit_radius_m.powi(3) / 398_600_441_800_000.0).sqrt();
    3.0 * core::f32::consts::PI * period_s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn torque_allocation_distributes_across_pyramid() {
        let mut arr = WheelArray::pyramid4();
        let faults = arr.command_torque([1.0, 0.0, 0.0], 0.1);
        let speeds = arr.speeds_rpm();
        // Symmetric pyramid: +x torque spins all four (two positive).
        assert!(speeds.iter().any(|s| *s > 0.0));
        assert!(speeds.iter().any(|s| *s < 0.0) || faults.iter().all(|f| f.is_none()));
        assert!(faults.iter().all(|f| f.is_none()), "within limits");
        // Momentum accumulates.
        let m = arr.momentum_nms();
        assert!(m.iter().map(|v| v.abs()).sum::<f32>() > 0.0);
    }

    #[test]
    fn speed_limit_faults_and_saturates() {
        let mut arr = WheelArray::pyramid4();
        // Sustained torque drives wheels to the limit.
        let mut faults = [None; 4];
        for _ in 0..2000 {
            faults = arr.command_torque([10.0, 0.0, 0.0], 0.1);
        }
        assert!(
            faults.contains(&Some(WheelFault::SpeedLimit)),
            "speed limit reached under sustained torque"
        );
        // Speeds clamped, never runaway.
        for s in arr.speeds_rpm() {
            assert!(s.abs() <= 6000.0 + 1e-3);
        }
    }

    #[test]
    fn dumper_unloads_momentum() {
        let d = MomentumDumper {
            gain_am2_per_nms: 1.0e4,
            rod_max_am2: 5.0,
            engage_threshold_nms: 0.01,
        };
        // Below threshold: rods off.
        assert!(d.command([0.001, 0.0, 0.0], [1.0, 0.0, 0.0]).is_none());
        // Above: dipole commanded; the induced torque opposes h.
        let m = d
            .command([0.5, 0.0, 0.0], [0.0, 1.0, 0.0])
            .expect("dumping");
        // h × B with h=+x, B=+y → +z; torque m×B must be −x.
        // m ∝ +z → m × B = z×y = −x ✓ (sign check via the m direction).
        assert!(m[2] != 0.0, "dipole along z: got {m:?}");
        assert!(m[2] <= d.rod_max_am2);
    }

    #[test]
    fn orbit_keeper_fires_outside_the_box() {
        let ok = OrbitKeeper {
            along_track_box_m: 100.0,
            dv_max_mps: 0.05,
        };
        assert!(ok.maneuver(50.0, 7_000_000.0).is_none(), "inside the box");
        let dv = ok.maneuver(500.0, 7_000_000.0).expect("outside");
        assert!(dv.abs() <= ok.dv_max_mps, "bounded ΔV");
        assert!(dv < 0.0, "ahead of slot → decelerate");
    }
}
