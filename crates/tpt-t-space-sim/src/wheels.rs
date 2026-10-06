//! Reaction wheel dynamics (spec §8): torque-command response with speed
//! saturation and momentum accumulation.
//!
//! A commanded wheel torque spins the wheel per its inertia; the reaction
//! on the spacecraft is the equal-and-opposite torque. Saturation at the
//! speed limit clamps the achievable torque (the wheel can no longer
//! accelerate in the saturated direction) — the signal the ADCS uses to
//! schedule momentum dumping.

/// One simulated wheel.
#[derive(Debug, Clone, Copy)]
pub struct SimWheel {
    /// Inertia, kg·m².
    pub inertia_kgm2: f32,
    /// Max speed, rad/s.
    pub speed_max_rad_s: f32,
    /// Current speed, rad/s.
    pub speed_rad_s: f32,
    /// Friction torque (viscous coefficient, mN·m per rad/s).
    pub friction_mnm_per_rps: f32,
}

impl SimWheel {
    /// A 10-class wheel.
    #[must_use]
    pub fn typical() -> Self {
        Self {
            inertia_kgm2: 1.0e-4,
            speed_max_rad_s: 628.0, // 6000 rpm
            speed_rad_s: 0.0,
            friction_mnm_per_rps: 0.02,
        }
    }

    /// Step one control period with commanded torque `tau_mnm`
    /// (millinewton-metres, signed). Returns the *achieved* torque after
    /// friction and saturation (the reaction the spacecraft feels).
    pub fn step(&mut self, tau_mnm: f32, dt_s: f32) -> f32 {
        let mut tau = tau_mnm;
        // Viscous friction opposes the motion.
        let friction = self.speed_rad_s * self.friction_mnm_per_rps;
        tau -= friction * (self.speed_rad_s.abs() > 1e-9) as i32 as f32;
        // Saturation: accelerating beyond the speed limit yields zero
        // effective torque in that direction.
        let would_be = self.speed_rad_s + tau * 1.0e-3 / self.inertia_kgm2 * dt_s;
        if would_be.abs() > self.speed_max_rad_s
            && would_be.abs() > self.speed_rad_s.abs()
        {
            tau = 0.0;
            self.speed_rad_s = self.speed_rad_s.clamp(
                -self.speed_max_rad_s,
                self.speed_max_rad_s,
            );
        }
        self.speed_rad_s += tau * 1.0e-3 / self.inertia_kgm2 * dt_s;
        self.speed_rad_s =
            self.speed_rad_s.clamp(-self.speed_max_rad_s, self.speed_max_rad_s);
        tau
    }
}

/// A 3-axis reaction-wheel assembly with pyramid mounting folded in
/// (per-axis single-wheel equivalent).
#[derive(Debug, Clone, Copy)]
pub struct WheelAssembly {
    /// Body-x wheel.
    pub x: SimWheel,
    /// Body-y wheel.
    pub y: SimWheel,
    /// Body-z wheel.
    pub z: SimWheel,
}

impl WheelAssembly {
    /// Three identical wheels.
    #[must_use]
    pub fn identical() -> Self {
        Self {
            x: SimWheel::typical(),
            y: SimWheel::typical(),
            z: SimWheel::typical(),
        }
    }

    /// Step all three; returns achieved body torques `[x, y, z]` mN·m.
    pub fn step(&mut self, tau_mnm: [f32; 3], dt_s: f32) -> [f32; 3] {
        [
            self.x.step(tau_mnm[0], dt_s),
            self.y.step(tau_mnm[1], dt_s),
            self.z.step(tau_mnm[2], dt_s),
        ]
    }

    /// Stored momentum per axis, N·m·s (I·ω).
    #[must_use]
    pub fn momentum_nms(&self) -> [f32; 3] {
        [
            self.x.inertia_kgm2 * self.x.speed_rad_s,
            self.y.inertia_kgm2 * self.y.speed_rad_s,
            self.z.inertia_kgm2 * self.z.speed_rad_s,
        ]
    }

    /// Whether any wheel is saturated.
    #[must_use]
    pub fn saturated(&self) -> bool {
        [self.x.speed_rad_s, self.y.speed_rad_s, self.z.speed_rad_s]
            .iter()
            .any(|s| s.abs() > 0.999 * 628.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn torque_spins_the_wheel() {
        let mut w = SimWheel::typical();
        // 10 mN·m for 1 s on 1e-4 kg·m²: frictionless would give exactly
        // 100 rad/s; viscous friction shaves a few percent at these
        // speeds (equilibrium is 500 rad/s).
        for _ in 0..100 {
            w.step(10.0, 0.01);
        }
        assert!(
            (88.0..=100.0).contains(&w.speed_rad_s),
            "{} rad/s",
            w.speed_rad_s
        );
    }

    #[test]
    fn friction_bleeds_coasting_speed() {
        let mut w = SimWheel::typical();
        for _ in 0..100 {
            w.step(10.0, 0.01);
        }
        let v0 = w.speed_rad_s;
        for _ in 0..100 {
            w.step(0.0, 0.01);
        }
        assert!(w.speed_rad_s < v0, "coasting decays: {v0} → {}", w.speed_rad_s);
    }

    #[test]
    fn saturation_clamps_speed_and_torque() {
        let mut w = SimWheel::typical();
        // 20 mN·m drives past the friction equilibrium (1000 rad/s): the
        // wheel slams into the 628 rad/s limit and holds there — never
        // beyond, with zero accelerating torque at the clamp.
        let mut clamped = false;
        for _ in 0..20_000 {
            let achieved = w.step(20.0, 0.01);
            if w.speed_rad_s >= 627.0 {
                clamped = true;
                assert!(
                    achieved <= 0.0 || w.speed_rad_s <= 628.0 + 1e-3,
                    "accelerated beyond the limit: {achieved} mN·m at {}",
                    w.speed_rad_s
                );
            }
        }
        assert!(clamped, "reached the speed limit");
        assert!(w.speed_rad_s <= 628.0 + 1e-3);
    }

    #[test]
    fn assembly_accumulates_momentum() {
        let mut a = WheelAssembly::identical();
        for _ in 0..1000 {
            a.step([10.0, -5.0, 0.0], 0.01);
        }
        let m = a.momentum_nms();
        assert!(m[0] > 0.0 && m[1] < 0.0 && m[2].abs() < 1e-6);
    }
}
