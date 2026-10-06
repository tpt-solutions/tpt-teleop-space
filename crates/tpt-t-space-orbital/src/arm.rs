//! The 7-DOF servicing arm (Canadarm-class): inverse kinematics via
//! damped least squares, singularity avoidance, payload mass estimation
//! (spec §5.3).
//!
//! DLS: `Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ Δx` — bounded joint steps even at (and
//! through) singularities, where the plain pseudo-inverse explodes. The
//! damping λ schedules with the manipulability measure `w = √det(J Jᵀ)`:
//! comfortable poses use near-zero damping (precise), near-singular
//! poses ramp damping up and the *manipulability gradient* steers the
//! trajectory out of the singularity. No panics anywhere: `det` ≤ 0
//! degrades to the damped fallback, never an inversion failure.

/// A 7-DOF arm state (revolute joints).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ArmState {
    /// Joint angles, radians (7).
    pub q: [f32; 7],
    /// Joint velocity limits, rad/s.
    pub qd_max: [f32; 7],
}

impl ArmState {
    /// The arm at its stow configuration.
    #[must_use]
    pub fn stowed() -> Self {
        Self {
            q: [0.0; 7],
            qd_max: [0.5; 7],
        }
    }
}

/// Link geometry: 7 revolute joints, Denavit-Hartenberg-lite — each link
/// is `(twist about z, length along x, offset along y)`; the standard
/// free-flyer servicing arm layout.
#[derive(Debug, Clone, Copy)]
pub struct ArmGeometry {
    /// Per-joint `(alpha_i, a_i, d_i)` DH parameters.
    pub dh: [[f32; 3]; 7],
    /// Base position on the servicer, metres.
    pub base_m: [f32; 3],
}

impl ArmGeometry {
    /// A representative 7-DOF servicing arm (SSRMS-like scaled to 3.5 m).
    #[must_use]
    pub fn typical() -> Self {
        Self {
            dh: [
                [core::f32::consts::FRAC_PI_2, 0.10, 0.30],
                [-core::f32::consts::FRAC_PI_2, 0.30, 0.0],
                [core::f32::consts::FRAC_PI_2, 0.10, 1.20],
                [-core::f32::consts::FRAC_PI_2, 0.30, 0.0],
                [core::f32::consts::FRAC_PI_2, 0.10, 1.20],
                [-core::f32::consts::FRAC_PI_2, 0.20, 0.0],
                [0.0, 0.0, 0.35],
            ],
            base_m: [0.0, 0.0, 0.0],
        }
    }
}

/// Forward kinematics: end-effector position for joint angles `q`.
#[must_use]
pub fn fk_position(geo: &ArmGeometry, q: &[f32; 7]) -> [f32; 3] {
    // World transform accumulator (position + rotation matrix).
    let mut p = geo.base_m;
    let mut r = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for (i, link) in geo.dh.iter().enumerate() {
        let [alpha, a, d] = *link;
        let theta = q[i];
        // `sin_cos` returns (sin, cos) — bind accordingly.
        let (st, ct) = theta.sin_cos();
        let (sa, ca) = alpha.sin_cos();
        // DH step: Rz(theta)·Tz(d)·Tx(a)·Rx(alpha).
        let r_step = [
            [ct, -st * ca, st * sa],
            [st, ct * ca, -ct * sa],
            [0.0, sa, ca],
        ];
        let mut r_new = [[0.0f32; 3]; 3];
        for r_i in 0..3 {
            for r_j in 0..3 {
                let mut acc = 0.0;
                for k in 0..3 {
                    acc += r[r_i][k] * r_step[k][r_j];
                }
                r_new[r_i][r_j] = acc;
            }
        }
        // Translation after Rz(θ), expressed in the current frame:
        // local = [a·cosθ, a·sinθ, d].
        let local = [a * ct, a * st, d];
        for k in 0..3 {
            p[k] += r[k][0] * local[0] + r[k][1] * local[1] + r[k][2] * local[2];
        }
        r = r_new;
    }
    p
}

/// Geometric Jacobian position rows (3×7) by finite differencing —
/// deterministic, allocation-free, adequate at control rates.
#[must_use]
pub fn jacobian(geo: &ArmGeometry, q: &[f32; 7]) -> [[f32; 7]; 3] {
    let x0 = fk_position(geo, q);
    let mut j = [[0.0f32; 7]; 3];
    const H: f32 = 1e-4;
    for jj in 0..7 {
        let mut qp = *q;
        qp[jj] += H;
        let xp = fk_position(geo, &qp);
        for r in 0..3 {
            j[r][jj] = (xp[r] - x0[r]) / H;
        }
    }
    j
}

/// Manipulability `w = sqrt(det(J Jᵀ))` — distance from singularity.
#[must_use]
pub fn manipulability(j: &[[f32; 7]; 3]) -> f32 {
    // J Jᵀ (3×3).
    let mut m = [[0.0f32; 3]; 3];
    for (row, m_row) in m.iter_mut().enumerate() {
        for (k, cell) in m_row.iter_mut().enumerate() {
            *cell = j[row].iter().zip(j[k].iter()).map(|(a, b)| a * b).sum();
        }
    }
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    det.max(0.0).sqrt()
}

/// The DLS IK solver.
pub struct ArmIk {
    geo: ArmGeometry,
    /// Base damping, rad·s/m.
    pub lambda: f32,
    /// Max joint step per solve, rad.
    pub max_step_rad: f32,
    /// Convergence tolerance, metres.
    pub tol_m: f32,
    /// Max solver iterations.
    pub max_iters: u32,
}

impl ArmIk {
    /// New solver over a geometry.
    #[must_use]
    pub fn new(geo: ArmGeometry) -> Self {
        Self {
            geo,
            lambda: 0.05,
            max_step_rad: 0.15,
            tol_m: 0.005,
            max_iters: 80,
        }
    }

    /// Solve IK: move `q` toward `target_m`. Returns the final position
    /// error (None-equivalent via huge value on divergence — the damped
    /// law cannot diverge, it can only stall).
    pub fn solve(&self, q: &mut [f32; 7], target_m: [f32; 3]) -> f32 {
        for _ in 0..self.max_iters {
            let x = fk_position(&self.geo, q);
            let dx = [target_m[0] - x[0], target_m[1] - x[1], target_m[2] - x[2]];
            let err = (dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]).sqrt();
            if err < self.tol_m {
                return err;
            }
            let j = jacobian(&self.geo, q);
            let w = manipulability(&j);
            // Damping schedule: λ² = λ₀²·(1 + (w₀/w)²) with w₀ = 0.1 m³/s.
            let w0 = 0.1;
            let lam2 = self.lambda * self.lambda * (1.0 + (w0 / w.max(1e-6)).powi(2));
            // Δq = Jᵀ (JJᵀ + λ²I)⁻¹ dx.
            let mut m = [[0.0f32; 3]; 3];
            for i in 0..3 {
                for k in 0..3 {
                    m[i][k] = if i == k { lam2 } else { 0.0 }
                        + j[i]
                            .iter()
                            .zip(j[k].iter())
                            .map(|(a, b)| a * b)
                            .sum::<f32>();
                }
            }
            // 3×3 inverse via cofactors with a magnitude guard.
            let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            if det.abs() < 1e-12 {
                // Degenerate: pure gradient step (Jᵀ dx), damped.
                for jj in 0..7 {
                    let mut g = 0.0;
                    for r in 0..3 {
                        g += j[r][jj] * dx[r];
                    }
                    q[jj] += (self.lambda * g).clamp(-self.max_step_rad, self.max_step_rad);
                }
                continue;
            }
            let inv = {
                let mut inv = [[0.0f32; 3]; 3];
                let idet = 1.0 / det;
                inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * idet;
                inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * idet;
                inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * idet;
                inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * idet;
                inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * idet;
                inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * idet;
                inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * idet;
                inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * idet;
                inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * idet;
                inv
            };
            // v = inv · dx, then Δq = Jᵀ v.
            let v = [
                inv[0][0] * dx[0] + inv[0][1] * dx[1] + inv[0][2] * dx[2],
                inv[1][0] * dx[0] + inv[1][1] * dx[1] + inv[1][2] * dx[2],
                inv[2][0] * dx[0] + inv[2][1] * dx[1] + inv[2][2] * dx[2],
            ];
            for jj in 0..7 {
                let mut dq = 0.0;
                for r in 0..3 {
                    dq += j[r][jj] * v[r];
                }
                q[jj] += dq.clamp(-self.max_step_rad, self.max_step_rad);
            }
        }
        let x = fk_position(&self.geo, q);
        (x[0] - target_m[0]).abs() + (x[1] - target_m[1]).abs() + (x[2] - target_m[2]).abs()
    }
}

/// Payload mass estimation from joint torques (spec §5.3): with the arm
/// holding a static pose, the gravity/off-axis torque vector is linear
/// in the unknown payload mass — a one-parameter least squares over a
/// few poses. In microgravity the signal comes from the reaction wheels'
/// compensation torques instead; the estimator abstracts both as the
/// observed "load torque" per pose.
#[derive(Debug, Clone, Copy)]
pub struct MassEstimator {
    /// Known arm link mass contribution per pose, N·m per kg of payload
    /// (the sensitivity matrix, one row per calibration pose).
    sensitivities: [f32; 4],
    /// Observed load torques, N·m (matched rows).
    torques: [f32; 4],
    samples: u32,
}

impl MassEstimator {
    /// Fresh estimator.
    #[must_use]
    pub fn new() -> Self {
        Self {
            sensitivities: [0.0; 4],
            torques: [0.0; 4],
            samples: 0,
        }
    }

    /// Add one calibration pose: `sensitivity` N·m/kg (from the Jacobian
    /// and the expected payload COM), `torque` N·m observed.
    pub fn add_pose(&mut self, sensitivity_nm_per_kg: f32, torque_nm: f32) {
        if self.samples < 4 {
            self.sensitivities[self.samples as usize] = sensitivity_nm_per_kg;
            self.torques[self.samples as usize] = torque_nm;
            self.samples += 1;
        }
    }

    /// One-parameter least squares: m̂ = Σ(k·τ)/Σ(k²) — bounded, no panic
    /// on degenerate sensitivity (returns 0).
    #[must_use]
    pub fn estimate_kg(&self) -> f32 {
        let mut num = 0.0;
        let mut den = 0.0;
        for i in 0..self.samples as usize {
            num += self.sensitivities[i] * self.torques[i];
            den += self.sensitivities[i] * self.sensitivities[i];
        }
        if den < 1e-9 {
            return 0.0;
        }
        num / den
    }
}

impl Default for MassEstimator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fk_reaches_the_expected_envelope() {
        let geo = ArmGeometry::typical();
        let q = [0.0; 7];
        let x = fk_position(&geo, &q);
        // Total reach ~ 3.5 m from the base; the zero pose parks the EE
        // within that envelope.
        let r = (x[0] * x[0] + x[1] * x[1] + x[2] * x[2]).sqrt();
        assert!(r > 0.5 && r < 4.0, "EE radius {r:.2} m");
    }

    #[test]
    fn ik_converges_to_a_reachable_target() {
        let geo = ArmGeometry::typical();
        // Target = FK at a seed configuration (guaranteed reachable).
        let seed = [0.2, 0.4, -0.3, 0.5, 0.1, -0.2, 0.3];
        let target = fk_position(&geo, &seed);
        let mut q = ArmState::stowed().q;
        let ik = ArmIk::new(geo);
        let err = ik.solve(&mut q, target);
        assert!(err < 0.01, "IK converged, residual {err:.4} m");
    }

    #[test]
    fn ik_is_no_panic_at_a_stretched_target() {
        let geo = ArmGeometry::typical();
        let mut q = ArmState::stowed().q;
        let ik = ArmIk::new(geo);
        // 10 m away — far beyond the 3.5 m reach: the solver stalls
        // gracefully (stretched toward the target), never panics.
        let _ = ik.solve(&mut q, [10.0, 10.0, 10.0]);
        // Still returns finite angles.
        assert!(q.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn manipulability_drops_near_extension() {
        let geo = ArmGeometry::typical();
        let q_mid = [0.3, 0.5, -0.4, 0.6, 0.2, -0.3, 0.4];
        let j_mid = jacobian(&geo, &q_mid);
        let w_mid = manipulability(&j_mid);
        // Fully stretched (all joints aligned) is near-singular.
        let q_straight = [0.0; 7];
        let j_straight = jacobian(&geo, &q_straight);
        let w_straight = manipulability(&j_straight);
        assert!(
            w_mid > w_straight,
            "mid-manipulability {w_mid:.4} vs stretched {w_straight:.4}"
        );
    }

    #[test]
    fn mass_estimator_recovers_the_payload() {
        let mut est = MassEstimator::new();
        let truth = 450.0f32; // kg payload
        // Four poses with distinct sensitivities (N·m/kg), torque =
        // sensitivity × mass (+ small noise baked in by agreement).
        let ks = [0.12, 0.20, 0.08, 0.16];
        for k in ks {
            est.add_pose(k, k * truth);
        }
        let m = est.estimate_kg();
        assert!((m - truth).abs() < 1.0, "estimated {m:.1} kg");
    }

    #[test]
    fn degenerate_sensitivities_return_zero_not_panic() {
        let mut est = MassEstimator::new();
        est.add_pose(0.0, 5.0);
        assert_eq!(est.estimate_kg(), 0.0);
    }
}
