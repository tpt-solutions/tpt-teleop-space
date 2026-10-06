//! The SIMD-accelerated Multiplicative Extended Kalman Filter (spec §4.3).
//!
//! State: attitude quaternion `q` (body → inertial) + gyro bias `b`.
//! Error state: 3-axis attitude error `δθ` + bias error `δb`; the 6×6
//! covariance `P` is row-major with the diagonal in the first blocks.
//!
//! Sensors fused:
//! * gyro triad (predict, `dt` per cycle),
//! * star tracker (attitude measurement — full-vector update),
//! * sun sensor (single vector measurement, gated on sun presence).
//!
//! No-panic guarantee: the 3×3 innovation covariance is inverted by
//! bounded Cholesky with a condition-number check; a corrupted sensor
//! (divergent innovation) falls back to a diagonal gain — the loop never
//! divides by zero and never panics.

use crate::simdv::{lanes, qconj, qmul, qnormalize, quat};
use core::simd::f32x4;

/// Filter dimensions.
pub const ERROR_DIM: usize = 6;

/// Sensor noise model (tuned per flight unit).
#[derive(Debug, Clone, Copy)]
pub struct SensorNoise {
    /// Gyro angle random walk, rad/√s.
    pub gyro_arw: f32,
    /// Gyro bias random walk, rad/s²/√s.
    pub gyro_bias_rw: f32,
    /// Star tracker measurement noise, rad (per axis).
    pub startracker_rad: f32,
    /// Sun sensor vector noise, unitless.
    pub sunsensor: f32,
}

impl SensorNoise {
    /// Representative cubesat-grade numbers.
    #[must_use]
    pub fn typical() -> Self {
        Self {
            gyro_arw: 0.001,
            gyro_bias_rw: 1.0e-5,
            startracker_rad: 5.0e-5, // ~10 arcsec
            sunsensor: 0.005,
        }
    }
}

/// The filter.
pub struct Mekf {
    /// Attitude quaternion `[w, x, y, z]`, body → inertial.
    pub quat: f32x4,
    /// Estimated gyro bias, rad/s `[x, y, z]`.
    pub bias: [f32; 3],
    /// Error-state covariance, row-major 6×6.
    pub p: [[f32; ERROR_DIM]; ERROR_DIM],
    noise: SensorNoise,
    /// Count of measurement updates rejected by the innovation gate.
    rejected_updates: u32,
}

impl Mekf {
    /// Initialize at a known attitude with zero covariance (post-deploy
    /// coarse acquisition completed).
    #[must_use]
    pub fn new(initial: [f32; 4], noise: SensorNoise) -> Self {
        let mut p = [[0.0; ERROR_DIM]; ERROR_DIM];
        for i in 0..3 {
            p[i][i] = 1.0e-4; // attitude uncertainty
            p[i + 3][i + 3] = 1.0e-6; // bias uncertainty
        }
        Self {
            quat: quat(initial[0], initial[1], initial[2], initial[3]),
            bias: [0.0; 3],
            p,
            noise,
            rejected_updates: 0,
        }
    }

    /// Predict one gyro sample (`omega` rad/s, `dt` s): propagate the
    /// quaternion and inflate the covariance with process noise.
    pub fn predict(&mut self, omega: [f32; 3], dt_s: f32) {
        // Bias-corrected rate.
        let w = [
            omega[0] - self.bias[0],
            omega[1] - self.bias[1],
            omega[2] - self.bias[2],
        ];
        // First-order quaternion integration: q ← q ⊗ δq(w·dt).
        let half = 0.5 * dt_s;
        let dq = quat(1.0, w[0] * half, w[1] * half, w[2] * half);
        self.quat = qnormalize(qmul(self.quat, dq));

        // Covariance propagation: P ← F P Fᵀ + Q with
        // F = [[I − ω×·dt, −I·dt], [0, I]]. The off-diagonal att↔bias
        // coupling built here is what makes the bias observable.
        let f = [
            [
                1.0 - (-w[2]) * dt_s * 0.0, // filled below
                w[2] * dt_s,
                -w[1] * dt_s,
                -dt_s,
                0.0,
                0.0,
            ],
            [-w[2] * dt_s, 1.0, w[0] * dt_s, 0.0, -dt_s, 0.0],
            [w[1] * dt_s, -w[0] * dt_s, 1.0, 0.0, 0.0, -dt_s],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ];
        let mut fp = [[0.0f32; ERROR_DIM]; ERROR_DIM];
        for i in 0..ERROR_DIM {
            for j in 0..ERROR_DIM {
                let mut acc = 0.0;
                for k in 0..ERROR_DIM {
                    acc += f[i][k] * self.p[k][j];
                }
                fp[i][j] = acc;
            }
        }
        for i in 0..ERROR_DIM {
            for j in 0..ERROR_DIM {
                let mut acc = 0.0;
                for k in 0..ERROR_DIM {
                    acc += fp[i][k] * f[j][k]; // (F P) Fᵀ
                }
                self.p[i][j] = acc;
            }
        }
        // Process noise: Q = [[(σg·dt)² I3, 0], [0, (σb·dt)² I3]].
        let q_att = (self.noise.gyro_arw * dt_s).powi(2);
        let q_bias = (self.noise.gyro_bias_rw * dt_s).powi(2);
        for i in 0..3 {
            self.p[i][i] += q_att;
            self.p[i + 3][i + 3] += q_bias;
        }
    }

    /// Star tracker update: `q_meas` is the tracker's inertial quaternion
    /// estimate. Innovation = attitude error between measurement and
    /// state; gain via bounded inversion of `S = H P Hᵀ + R`.
    pub fn update_startracker(&mut self, q_meas: [f32; 4]) -> bool {
        // δq = q_meas ⊗ q⁻¹ → error in the body frame.
        let err = qmul(
            quat(q_meas[0], q_meas[1], q_meas[2], q_meas[3]),
            qconj(self.quat),
        );
        let [w, x, y, z] = lanes(err);
        // First-order: δθ ≈ 2·vec(δq) (sign-corrected).
        let sign = if w < 0.0 { -2.0 } else { 2.0 };
        let innov = [x * sign, y * sign, z * sign];

        // Gate: a corrupted tracker (wrong seed frame, blinding) produces
        // a huge innovation — reject rather than corrupt the state.
        if innov.iter().any(|d| !d.is_finite()) || innov.iter().map(|d| d.abs()).sum::<f32>() > 0.5
        {
            self.rejected_updates += 1;
            return false;
        }

        // S = P_att + R (H picks the attitude rows).
        let r = self.noise.startracker_rad.powi(2);
        let mut s = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                s[i][j] = self.p[i][j] + if i == j { r } else { 0.0 };
            }
        }
        let Some(sinvs) = invert_spd3(s) else {
            // Ill-conditioned (blinded tracker): diagonal-gain fallback —
            // bounded, no panic. The correction still applies (damped),
            // the covariance damps by (1 − k).
            let mut dx = [0.0f32; 3];
            for i in 0..3 {
                let k = self.p[i][i] / (self.p[i][i] + r);
                dx[i] = innov[i] * k;
                self.damp_covariance(i, k);
            }
            let dq = quat(1.0, 0.5 * dx[0], 0.5 * dx[1], 0.5 * dx[2]);
            self.quat = qnormalize(qmul(self.quat, dq));
            return true;
        };
        // K = P Hᵀ S⁻¹ (6×3): attitude rows then bias rows.
        let mut k = [[0.0f32; 3]; ERROR_DIM];
        for i in 0..ERROR_DIM {
            for j in 0..3 {
                let mut acc = 0.0;
                for l in 0..3 {
                    acc += self.p[i][l] * sinvs[l][j];
                }
                k[i][j] = acc;
            }
        }
        // δx = K · innovation.
        let mut dx = [0.0f32; ERROR_DIM];
        for i in 0..ERROR_DIM {
            dx[i] = k[i][0] * innov[0] + k[i][1] * innov[1] + k[i][2] * innov[2];
        }
        // Apply: quaternion multiplicative correction, bias additive.
        let dq = quat(1.0, 0.5 * dx[0], 0.5 * dx[1], 0.5 * dx[2]);
        self.quat = qnormalize(qmul(self.quat, dq));
        for i in 0..3 {
            self.bias[i] += dx[i + 3];
        }
        // Covariance: P ← (I − K H) P.
        let mut out = self.p;
        for i in 0..ERROR_DIM {
            for j in 0..ERROR_DIM {
                let mut acc = self.p[i][j];
                for l in 0..3 {
                    acc -= k[i][l] * self.p[l][j];
                }
                out[i][j] = acc;
            }
        }
        self.p = out;
        true
    }

    /// Sun sensor update: `sun_body` is the measured sun direction in the
    /// body frame, `sun_inertial` the ephemeris direction. Gated on
    /// sun presence (`None` in eclipse — no update, no state change).
    pub fn update_sunsensor(&mut self, sun_body: Option<[f32; 3]>, sun_inertial: [f32; 3]) -> bool {
        let Some(meas) = sun_body else {
            return false; // eclipse: predict-only
        };
        // Predicted sun in the body frame: q maps body→inertial, so the
        // inertial sun vector maps into the body through the conjugate.
        let [w, x, y, z] = lanes(self.quat);
        let pred = crate::simdv::qrot(quat(w, -x, -y, -z), sun_inertial);
        // Innovation in the plane ⊥ pred: meas − pred (first order).
        let innov = [meas[0] - pred[0], meas[1] - pred[1], meas[2] - pred[2]];
        if innov.iter().any(|d| !d.is_finite()) {
            self.rejected_updates += 1;
            return false;
        }
        // H = [+skew(pred) | 0] (innovation y = z − ẑ with ẑ = pred +
        // δθ × pred → y = −δθ × pred... resolved below with the
        // innovation sign folded into K).
        let r = self.noise.sunsensor.powi(2);
        let skew = [
            [0.0, -pred[2], pred[1]],
            [pred[2], 0.0, -pred[0]],
            [-pred[1], pred[0], 0.0],
        ];
        // S = skew·P_att·skewᵀ + R·I.
        let mut s = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut acc = 0.0;
                for l in 0..3 {
                    for m in 0..3 {
                        acc += skew[i][l] * self.p[l][m] * skew[j][m];
                    }
                }
                s[i][j] = acc + if i == j { r } else { 0.0 };
            }
        }
        let Some(sinvs) = invert_spd3(s) else {
            self.rejected_updates += 1;
            return false;
        };
        // K = P Hᵀ S⁻¹ with H = +skew(pred): h(q ⊗ δq) ≈ pred +
        // skew(pred)·δθ, so the innovation y = meas − pred is explained
        // by a δθ rotated toward the measurement.
        let mut k = [[0.0f32; 3]; ERROR_DIM];
        for i in 0..ERROR_DIM {
            for j in 0..3 {
                let mut acc = 0.0;
                for l in 0..3 {
                    // (P Hᵀ)_i,l = Σ_m P_i,m·skew[l][m].
                    let mut ph = 0.0;
                    for m in 0..3 {
                        ph += self.p[i][m] * skew[l][m];
                    }
                    acc += ph * sinvs[l][j];
                }
                k[i][j] = acc;
            }
        }
        let mut dx = [0.0f32; ERROR_DIM];
        for i in 0..ERROR_DIM {
            dx[i] = k[i][0] * innov[0] + k[i][1] * innov[1] + k[i][2] * innov[2];
        }
        let dq = quat(1.0, 0.5 * dx[0], 0.5 * dx[1], 0.5 * dx[2]);
        self.quat = qnormalize(qmul(self.quat, dq));
        for i in 0..3 {
            self.bias[i] += dx[i + 3];
        }
        // P ← (I − K H) P = P − K·skew·P.
        let mut out = self.p;
        for i in 0..ERROR_DIM {
            for j in 0..ERROR_DIM {
                let mut acc = self.p[i][j];
                for l in 0..3 {
                    for m in 0..3 {
                        acc -= k[i][l] * skew[l][m] * self.p[m][j];
                    }
                }
                out[i][j] = acc;
            }
        }
        self.p = out;
        true
    }

    fn damp_covariance(&mut self, axis: usize, k: f32) {
        // (1 − k) scaling of the updated row/col — the diagonal-gain
        // covariance update.
        for j in 0..ERROR_DIM {
            self.p[axis][j] *= 1.0 - k;
            self.p[j][axis] *= 1.0 - k;
        }
    }

    /// Rejected measurement count (telemetry).
    #[must_use]
    pub fn rejected(&self) -> u32 {
        self.rejected_updates
    }

    /// Attitude as a unit `[w, x, y, z]` array.
    #[must_use]
    pub fn quat_array(&self) -> [f32; 4] {
        lanes(self.quat)
    }

    /// Rotate an inertial vector into the body frame with the current
    /// estimate.
    #[must_use]
    pub fn rot_to_body(&self, v_inertial: [f32; 3]) -> [f32; 3] {
        let [w, x, y, z] = lanes(self.quat);
        crate::simdv::qrot(quat(w, -x, -y, -z), v_inertial)
    }
}

/// Bounded inverse of a symmetric positive-definite 3×3 via Cholesky.
/// Returns `None` when a pivot approaches zero (condition-number gate) —
/// the caller falls back, never divides by zero.
#[must_use]
pub fn invert_spd3(m: [[f32; 3]; 3]) -> Option<[[f32; 3]; 3]> {
    // Cholesky L·Lᵀ = m.
    let mut l = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..=i {
            let mut acc = m[i][j];
            for k in 0..j {
                acc -= l[i][k] * l[j][k];
            }
            if i == j {
                if acc <= 1e-12 {
                    return None; // not SPD / singular to working precision
                }
                l[i][j] = acc.sqrt();
            } else {
                l[i][j] = acc / l[j][j];
            }
        }
    }
    // Condition gate: ratio of extreme pivots.
    let d = [l[0][0], l[1][1], l[2][2]];
    if d.iter().any(|v| !v.is_finite() || *v <= 1e-9) {
        return None;
    }
    // cond(A) = cond(L)² for SPD Cholesky — gate the factor at 1e3 so
    // matrices at 1e6+ condition fall back.
    let cond = d[0].max(d[1]).max(d[2]) / d[0].min(d[1]).min(d[2]);
    if cond > 1.0e3 {
        return None;
    }
    // Inverse of L (forward substitution), then M = L⁻ᵀ·L⁻¹.
    let mut li = [[0.0f32; 3]; 3];
    for c in 0..3 {
        li[c][c] = 1.0 / l[c][c];
        for r in (c + 1)..3 {
            let mut acc = 0.0;
            for k in c..r {
                acc += l[r][k] * li[k][c];
            }
            li[r][c] = -acc / l[r][r];
        }
    }
    let mut inv = [[0.0f32; 3]; 3];
    for i in 0..3 {
        for j in 0..3 {
            // Σ_k L⁻¹[k][i]·L⁻¹[k][j] = (L⁻ᵀ·L⁻¹)[i][j].
            let col_i = [li[0][i], li[1][i], li[2][i]];
            let col_j = [li[0][j], li[1][j], li[2][j]];
            let mut acc = 0.0;
            for (a, b) in col_i.iter().zip(col_j.iter()) {
                acc += a * b;
            }
            inv[i][j] = acc;
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn q_from_axis_angle(axis: [f32; 3], angle: f32) -> [f32; 4] {
        let n = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let (s, c) = (angle.sin(), angle.cos());
        [c, axis[0] / n * s, axis[1] / n * s, axis[2] / n * s]
    }

    #[test]
    fn startracker_update_converges_on_truth() {
        let truth = q_from_axis_angle([0.0, 0.0, 1.0], 0.7);
        let mut f = Mekf::new(
            q_from_axis_angle([0.0, 0.0, 1.0], 0.65), // 0.05 rad off
            SensorNoise::typical(),
        );
        for _ in 0..8 {
            assert!(f.update_startracker(truth));
        }
        let est = f.quat_array();
        let dot =
            (est[0] * truth[0] + est[1] * truth[1] + est[2] * truth[2] + est[3] * truth[3]).abs();
        assert!(dot > 0.999_999, "attitude converged, |dot|={dot}");
        // Covariance shrunk.
        assert!(f.p[0][0] < 1.0e-4);
    }

    #[test]
    fn gyro_bias_is_observable_over_time() {
        let truth_bias = [0.002, -0.001, 0.0005];
        let mut f = Mekf::new([1.0, 0.0, 0.0, 0.0], SensorNoise::typical());
        let truth = [1.0f32, 0.0, 0.0, 0.0];
        let mut q_true = [1.0f32, 0.0, 0.0, 0.0];
        let dt = 0.05f32;
        for _ in 0..2000 {
            // Gyro reads bias + small true rate.
            let omega = [truth_bias[0], truth_bias[1], truth_bias[2]];
            f.predict(omega, dt);
            // Truth integrates the same (bias IS the rate here — pure bias
            // estimation scenario, star tracker anchors attitude).
            f.update_startracker(truth);
            let _ = &mut q_true;
        }
        assert!(
            (f.bias[0] - truth_bias[0]).abs() < 5e-4,
            "bias x estimated {}, want {}",
            f.bias[0],
            truth_bias[0]
        );
    }

    #[test]
    fn corrupted_tracker_is_rejected_not_fused() {
        let mut f = Mekf::new([1.0, 0.0, 0.0, 0.0], SensorNoise::typical());
        // A blinded tracker reports garbage (NaN, then 1-radian flips).
        assert!(!f.update_startracker([f32::NAN, 0.0, 0.0, 1.0]));
        assert!(!f.update_startracker([0.0, 0.0, 0.0, 1.0]));
        assert_eq!(f.rejected(), 2);
        // State survived: identity quaternion intact.
        let q = f.quat_array();
        assert!((q[0] - 1.0).abs() < 1e-6);
    }

    #[test]
    fn sunsensor_updates_only_in_sun() {
        let mut f = Mekf::new([1.0, 0.0, 0.0, 0.0], SensorNoise::typical());
        // Eclipse: no update, returns false, state untouched.
        assert!(!f.update_sunsensor(None, [0.0, 0.0, 1.0]));
        // Truth: body yawed +0.05 rad about y. The inertial sun [0,0,1]
        // then appears in the body frame rotated the other way.
        let truth = q_from_axis_angle([0.0, 1.0, 0.0], 0.05);
        let sun_inertial = [0.0f32, 0.0, 1.0];
        // body = q_truth⁻¹ ⊗ sun_inertial ⊗ q_truth (vector sandwich).
        let tq = crate::simdv::quat(truth[0], truth[1], truth[2], truth[3]);
        let sun_body = crate::simdv::qrot(crate::simdv::qconj(tq), sun_inertial);
        assert!(
            sun_body[0] < -0.04,
            "sun appears at −x in body: {sun_body:?}"
        );
        for _ in 0..12 {
            assert!(f.update_sunsensor(Some(sun_body), sun_inertial));
        }
        let est = f.quat_array();
        let dot =
            (est[0] * truth[0] + est[1] * truth[1] + est[2] * truth[2] + est[3] * truth[3]).abs();
        assert!(dot > 0.9999, "sun-vector update converged, |dot|={dot}");
    }

    #[test]
    fn singular_matrix_falls_back_to_none() {
        assert!(invert_spd3([[0.0; 3]; 3]).is_none());
        // Near-singular (condition gate).
        let mut m = [[0.0f32; 3]; 3];
        m[0][0] = 1.0;
        m[1][1] = 1.0e-9;
        m[2][2] = 1.0;
        assert!(invert_spd3(m).is_none());
        // Well-conditioned: known inverse.
        let m = [[4.0, 0.0, 0.0], [0.0, 9.0, 0.0], [0.0, 0.0, 16.0]];
        let inv = invert_spd3(m).expect("diagonal SPD");
        assert!((inv[0][0] - 0.25).abs() < 1e-6);
        assert!((inv[2][2] - 0.0625).abs() < 1e-6);
    }
}
