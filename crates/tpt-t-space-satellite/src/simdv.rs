//! `f32x4` SIMD helpers for the quaternion/MEKF math.
//!
//! Quaternions live in `[w, x, y, z]` lanes of a `f32x4`; the helpers keep
//! the hot loop (predict + update every cycle) on SIMD registers.

use core::simd::f32x4;

/// Quaternion from lanes `[w, x, y, z]`.
#[must_use]
pub fn quat(w: f32, x: f32, y: f32, z: f32) -> f32x4 {
    f32x4::from_array([w, x, y, z])
}

/// Lane array view `[w, x, y, z]`.
#[must_use]
pub fn lanes(q: f32x4) -> [f32; 4] {
    q.to_array()
}

/// Hamilton product `a ⊗ b` (both `[w, x, y, z]`).
#[must_use]
pub fn qmul(a: f32x4, b: f32x4) -> f32x4 {
    let [aw, ax, ay, az] = a.to_array();
    let [bw, bx, by, bz] = b.to_array();
    quat(
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )
}

/// Quaternion conjugate.
#[must_use]
pub fn qconj(q: f32x4) -> f32x4 {
    let [w, x, y, z] = q.to_array();
    quat(w, -x, -y, -z)
}

/// Norm of a quaternion (all four lanes).
#[must_use]
pub fn qnorm(q: f32x4) -> f32 {
    let a = q * q;
    let [w2, x2, y2, z2] = a.to_array();
    (w2 + x2 + y2 + z2).sqrt()
}

/// Normalize; a zero quaternion (corrupt input) maps to the identity
/// instead of NaN — the no-panic guarantee at the SIMD layer.
#[must_use]
pub fn qnormalize(q: f32x4) -> f32x4 {
    let n = qnorm(q);
    if n < 1e-12 {
        return quat(1.0, 0.0, 0.0, 0.0);
    }
    q / f32x4::splat(n)
}

/// Rotate a vector by a unit quaternion (the sandwich product).
#[must_use]
pub fn qrot(q: f32x4, v: [f32; 3]) -> [f32; 3] {
    let vv = quat(0.0, v[0], v[1], v[2]);
    let out = qmul(qmul(q, vv), qconj(q));
    let [_w, x, y, z] = out.to_array();
    [x, y, z]
}

/// Rotate vector `v` by the small rotation `dtheta` (rad, body frame)
/// — first-order exact for the error-state update pattern.
#[must_use]
pub fn rot3(v: [f32; 3], dtheta: [f32; 3]) -> [f32; 3] {
    // v' = v + dtheta × v (first order)
    [
        v[0] + dtheta[1] * v[2] - dtheta[2] * v[1],
        v[1] + dtheta[2] * v[0] - dtheta[0] * v[2],
        v[2] + dtheta[0] * v[1] - dtheta[1] * v[0],
    ]
}

/// 3×3 (row-major) times vector.
#[must_use]
pub fn mat3vec(m: [[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    let mut out = [0.0; 3];
    for (r, o) in out.iter_mut().enumerate() {
        let row = f32x4::from_array([m[r][0], m[r][1], m[r][2], 0.0]);
        let vec = f32x4::from_array([v[0], v[1], v[2], 0.0]);
        // Horizontal add of the first three lanes.
        let prod = row * vec;
        let [a, b, c, _] = prod.to_array();
        *o = a + b + c;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn identity_rotation_preserves_vectors() {
        let q = quat(1.0, 0.0, 0.0, 0.0);
        let v = qrot(q, [1.0, 2.0, 3.0]);
        assert!(close(v[0], 1.0) && close(v[1], 2.0) && close(v[2], 3.0));
    }

    #[test]
    fn ninety_degree_z_rotation() {
        // +90° about z: x axis → y axis.
        let q = quat(0.5_f32.sqrt(), 0.0, 0.0, 0.5_f32.sqrt());
        let v = qrot(q, [1.0, 0.0, 0.0]);
        assert!(close(v[0], 0.0) && close(v[1], 1.0), "got {v:?}");
    }

    #[test]
    fn double_conjugate_composition() {
        // Unit inputs (qrot with non-unit quaternions scales the vector).
        let qa = qnormalize(quat(0.9, 0.1, 0.2, 0.3));
        let qb = qnormalize(quat(0.8, -0.1, 0.4, 0.1));
        // Applying qa then qb composes as qb ⊗ qa (Hamilton order).
        let q = qnormalize(qmul(qb, qa));
        let v = [0.3, -1.2, 2.0];
        let a = qrot(q, v);
        let b = qrot(qb, qrot(qa, v));
        assert!(close(a[0], b[0]) && close(a[1], b[1]) && close(a[2], b[2]));
    }

    #[test]
    fn zero_quaternion_normalizes_to_identity_not_nan() {
        let q = qnormalize(quat(0.0, 0.0, 0.0, 0.0));
        let [w, x, y, z] = lanes(q);
        assert!(close(w, 1.0) && x == 0.0 && y == 0.0 && z == 0.0);
    }

    #[test]
    fn matvec_matches_scalar() {
        let m = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let v = [1.0, -1.0, 2.0];
        let out = mat3vec(m, v);
        assert!(close(out[0], 1.0 - 2.0 + 6.0));
        assert!(close(out[1], 4.0 - 5.0 + 12.0));
        assert!(close(out[2], 7.0 - 8.0 + 18.0));
    }
}
