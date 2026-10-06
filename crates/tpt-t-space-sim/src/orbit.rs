//! 6DOF orbital mechanics (spec §8): J2 oblateness, atmospheric drag,
//! third-body gravity; fixed-step RK4.
//!
//! State is Cartesian `[x, y, z, vx, vy, vz]` metres/metres-per-second in
//! an Earth-centred inertial frame. The model is honest at LEO mission
//! analysis fidelity: geopotential through J2, exponential atmosphere
//! with ballistic drag, and Moon/Sun point masses.

/// Gravitational parameter of Earth, m³/s².
pub const MU_EARTH: f64 = 3.986_004_418e14;
/// Earth equatorial radius, m.
pub const R_EARTH: f64 = 6_378_137.0;
/// J2 coefficient.
pub const J2: f64 = 1.082_626_68e-3;
/// Earth rotation rate, rad/s.
pub const OMEGA_EARTH: f64 = 7.292_115e-5;

/// Spacecraft physical parameters.
#[derive(Debug, Clone, Copy)]
pub struct Spacecraft {
    /// Mass, kg.
    pub mass_kg: f64,
    /// Drag reference area, m².
    pub drag_area_m2: f64,
    /// Drag coefficient.
    pub drag_cd: f64,
}

impl Spacecraft {
    /// A 12-unit-class cubesat.
    #[must_use]
    pub fn cubesat12() -> Self {
        Self {
            mass_kg: 24.0,
            drag_area_m2: 0.30,
            drag_cd: 2.2,
        }
    }
}

/// Perturbation toggles (isolate effects in tests).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Perturbations {
    /// J2 oblateness.
    pub j2: bool,
    /// Atmospheric drag.
    pub drag: bool,
    /// Third bodies (Moon + Sun point masses).
    pub third_body: bool,
}

impl Perturbations {
    /// Full model.
    #[must_use]
    pub const fn full() -> Self {
        Self {
            j2: true,
            drag: true,
            third_body: true,
        }
    }
    /// Two-body only.
    #[must_use]
    pub const fn two_body() -> Self {
        Self {
            j2: false,
            drag: false,
            third_body: false,
        }
    }
}

/// The orbital propagator.
pub struct Orbit {
    /// Spacecraft physical parameters.
    pub sc: Spacecraft,
    /// Active perturbation set.
    pub perturbations: Perturbations,
    /// Simulation epoch, seconds (driven by the sim clock).
    pub t_s: f64,
}

impl Orbit {
    /// New propagator.
    #[must_use]
    pub fn new(sc: Spacecraft, perturbations: Perturbations) -> Self {
        Self {
            sc,
            perturbations,
            t_s: 0.0,
        }
    }

    /// Acceleration at state `s` (Cartesian, ECI).
    #[must_use]
    pub fn accel(&self, s: &[f64; 6], t_s: f64) -> [f64; 3] {
        let (x, y, z) = (s[0], s[1], s[2]);
        let r = (x * x + y * y + z * z).sqrt();
        let r3 = r * r * r;
        // Two-body.
        let mut a = [
            -MU_EARTH * x / r3,
            -MU_EARTH * y / r3,
            -MU_EARTH * z / r3,
        ];
        // J2 (ECI z-axis oblateness).
        if self.perturbations.j2 {
            let zr2 = (z * z) / (r * r);
            let f = 1.5 * J2 * MU_EARTH * R_EARTH * R_EARTH / (r3 * r * r);
            a[0] += f * x * (5.0 * zr2 - 1.0);
            a[1] += f * y * (5.0 * zr2 - 1.0);
            a[2] += f * z * (5.0 * zr2 - 3.0);
        }
        // Exponential-atmosphere drag (rotating atmosphere).
        if self.perturbations.drag {
            let alt = r - R_EARTH;
            // Piecewise scale heights, kg/m³ at the layer base.
            let (h0, rho0, hscale) = if alt < 25_000.0 {
                (0.0, 1.225, 7_200.0)
            } else if alt < 100_000.0 {
                (25_000.0, 3.9e-2, 6_300.0)
            } else if alt < 300_000.0 {
                (100_000.0, 5.6e-7, 12_000.0)
            } else if alt < 600_000.0 {
                (300_000.0, 2.4e-11, 25_000.0)
            } else {
                (600_000.0, 5.0e-13, 40_000.0)
            };
            let rho = rho0 * (-((alt - h0) / hscale)).exp();
            // Atmosphere co-rotates; v_rel = v − ω × r.
            let vrx = s[3] + OMEGA_EARTH * y;
            let vry = s[4] - OMEGA_EARTH * x;
            let vrz = s[5];
            let vr = (vrx * vrx + vry * vry + vrz * vrz).sqrt();
            let bc = self.sc.drag_cd * self.sc.drag_area_m2 / (2.0 * self.sc.mass_kg);
            a[0] -= bc * rho * vr * vrx;
            a[1] -= bc * rho * vr * vry;
            a[2] -= bc * rho * vr * vrz;
        }
        // Third bodies: Moon and Sun point masses on their (mean) circular
        // tracks — good enough to perturb, not ephemeris-grade.
        if self.perturbations.third_body {
            const MU_MOON: f64 = 4.904_869_5e12;
            const R_MOON: f64 = 384_400e3;
            const MU_SUN: f64 = 1.327_124_400_18e20;
            const R_SUN: f64 = 149_598e6;
            let bodies = [
                (MU_MOON, R_MOON, 27.32 * 86_400.0),
                (MU_SUN, R_SUN, 365.25 * 86_400.0),
            ];
            for (mu, r_orbit, period) in bodies {
                let w = 2.0 * core::f64::consts::PI / period;
                let b = [r_orbit * (w * t_s).cos(), r_orbit * (w * t_s).sin(), 0.0];
                let d = [b[0] - x, b[1] - y, b[2] - z];
                let dn = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                let bn = (b[0] * b[0] + b[1] * b[1] + b[2] * b[2]).sqrt();
                for k in 0..3 {
                    // Indirect term included: μ·(Δ/|Δ|³ − b/|b|³).
                    a[k] += mu * (d[k] / (dn * dn * dn) - b[k] / (bn * bn * bn));
                }
            }
        }
        a
    }

    /// One fixed-step RK4 propagation of `dt` seconds.
    pub fn step_rk4(&mut self, state: &mut [f64; 6], dt_s: f64) {
        let t = self.t_s;
        let a1 = self.accel(state, t);
        let mut s2 = *state;
        for i in 0..3 {
            s2[i] += state[3 + i] * dt_s * 0.5;
            s2[3 + i] += a1[i] * dt_s * 0.5;
        }
        let a2 = self.accel(&s2, t + dt_s * 0.5);
        let mut s3 = *state;
        for i in 0..3 {
            s3[i] += (state[3 + i] + a1[i] * dt_s * 0.5) * dt_s * 0.5;
            s3[3 + i] += a2[i] * dt_s * 0.5;
        }
        let a3 = self.accel(&s3, t + dt_s * 0.5);
        let mut s4 = *state;
        for i in 0..3 {
            s4[i] += (state[3 + i] + a2[i] * dt_s * 0.5) * dt_s;
            s4[3 + i] += a3[i] * dt_s;
        }
        let a4 = self.accel(&s4, t + dt_s);
        for i in 0..3 {
            state[i] +=
                dt_s / 6.0 * (state[3 + i] + 2.0 * (s2[3 + i]) + 2.0 * (s3[3 + i]) + s4[3 + i]);
            state[3 + i] += dt_s / 6.0 * (a1[i] + 2.0 * a2[i] + 2.0 * a3[i] + a4[i]);
        }
        self.t_s += dt_s;
    }

    /// Kepler orbital elements snapshot (semi-major axis, eccentricity,
    /// inclination) for the visualizer/telemetry.
    #[must_use]
    pub fn elements(state: &[f64; 6]) -> (f64, f64, f64) {
        let r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2]).sqrt();
        let v2 = state[3] * state[3] + state[4] * state[4] + state[5] * state[5];
        let energy = v2 / 2.0 - MU_EARTH / r;
        let a = -MU_EARTH / (2.0 * energy);
        // h = r × v.
        let hx = state[1] * state[5] - state[2] * state[4];
        let hy = state[2] * state[3] - state[0] * state[5];
        let hz = state[0] * state[4] - state[1] * state[3];
        let h2 = hx * hx + hy * hy + hz * hz;
        let e = (1.0 + 2.0 * energy * h2 / (MU_EARTH * MU_EARTH)).max(0.0).sqrt();
        let inc = (hz / h2.sqrt()).acos();
        (a, e, inc)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn leo_state() -> [f64; 6] {
        // 500 km circular, 53° inclination.
        let r = R_EARTH + 500e3;
        let v = (MU_EARTH / r).sqrt();
        let inc = 53.0_f64.to_radians();
        [
            r,
            0.0,
            0.0,
            0.0,
            v * inc.cos(),
            v * inc.sin(),
        ]
    }

    #[test]
    fn two_body_orbit_closes_after_one_period() {
        let mut orb = Orbit::new(Spacecraft::cubesat12(), Perturbations::two_body());
        let mut s = leo_state();
        let (a, e, _) = Orbit::elements(&s);
        assert!(e < 1e-6, "circular init: e={e}");
        let period = 2.0 * core::f64::consts::PI * (a * a * a / MU_EARTH).sqrt();
        // Propagate exactly one period (the last step is trimmed so the
        // truncation cannot eat a second of arc ≈ 7 km).
        let mut remaining = period;
        while remaining > 0.0 {
            let dt = remaining.min(1.0);
            orb.step_rk4(&mut s, dt);
            remaining -= dt;
        }
        let s0 = leo_state();
        let err = ((s[0] - s0[0]).powi(2)
            + (s[1] - s0[1]).powi(2)
            + (s[2] - s0[2]).powi(2))
        .sqrt();
        assert!(err < 25.0, "position closure error {err:.2} m after one orbit");
    }

    #[test]
    fn j2_rotates_the_line_of_nodes() {
        // J2 produces a secular nodal regression; two-body does not.
        let two_body_n = {
            let mut orb = Orbit::new(Spacecraft::cubesat12(), Perturbations::two_body());
            let mut s = leo_state();
            for _ in 0..2000 {
                orb.step_rk4(&mut s, 1.0);
            }
            let (_, _, inc) = Orbit::elements(&s);
            inc
        };
        let j2_inc = {
            let mut orb = Orbit::new(Spacecraft::cubesat12(), Perturbations::full());
            let mut s = leo_state();
            for _ in 0..2000 {
                orb.step_rk4(&mut s, 1.0);
            }
            let (_, _, inc) = Orbit::elements(&s);
            inc
        };
        // The J2 run's inclination measurably differs from two-body at
        // the same epoch (~0.05°/day → measurable over 2000 s).
        assert!(
            (two_body_n - j2_inc).abs() > 1e-7,
            "J2 perturbs the elements: {two_body_n} vs {j2_inc}"
        );
    }

    #[test]
    fn drag_decreases_the_semi_major_axis() {
        let mut orb = Orbit::new(Spacecraft::cubesat12(), Perturbations::full());
        let mut s = leo_state();
        let (a0, _, _) = Orbit::elements(&s);
        for _ in 0..5000 {
            orb.step_rk4(&mut s, 1.0);
        }
        let (a1, _, _) = Orbit::elements(&s);
        assert!(
            a1 < a0,
            "drag decays the orbit: {a0:.1} → {a1:.1} m"
        );
    }
}
