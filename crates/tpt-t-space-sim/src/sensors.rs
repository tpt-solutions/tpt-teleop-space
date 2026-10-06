//! Sensor noise models (spec §8): star-tracker blinding, gyro bias
//! random walk, IMU noise.
//!
//! All stochastic behavior flows from one deterministic xorshift PRNG
//! seeded at construction — the whole sim replays exactly from its seed.

/// Deterministic xorshift64* PRNG.
#[derive(Debug, Clone, Copy)]
pub struct SimRng(u64);

impl SimRng {
    /// Seed (0 maps to 1).
    #[must_use]
    pub fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }

    /// Next raw u64.
    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Uniform `[-1, 1)`.
    pub fn uniform_pm1(&mut self) -> f32 {
        (self.next_u64() >> 40) as f32 / (1u32 << 23) as f32 - 1.0
    }

    /// Gaussian (Box–Muller, cached pair not kept — fine at sim rates).
    pub fn gaussian(&mut self, sigma: f32) -> f32 {
        let u1 = (self.uniform_pm1() + 1.0) / 2.0;
        let u2 = self.uniform_pm1();
        let mag = (-2.0 * u1.max(1e-9).ln()).sqrt();
        mag * (2.0 * core::f32::consts::PI * u2).cos() * sigma
    }
}

/// Star tracker model: measurement noise plus sun-in-FOV blinding.
#[derive(Debug, Clone, Copy)]
pub struct StarTracker {
    /// Measurement noise σ, rad per axis.
    pub noise_rad: f32,
    /// Sun angle (between boresight and sun) below which the tracker is
    /// blinded, radians.
    pub blinding_angle_rad: f32,
    blinded: bool,
}

impl StarTracker {
    /// New tracker.
    #[must_use]
    pub fn new(noise_rad: f32, blinding_angle_rad: f32) -> Self {
        Self {
            noise_rad,
            blinding_angle_rad,
            blinded: false,
        }
    }

    /// Update the geometry: boresight (body) vs sun vector (body). Sets
    /// the blinding latch; while blinded, [`Self::measure`] returns
    /// `None`.
    pub fn set_geometry(&mut self, boresight: [f32; 3], sun_body: [f32; 3]) {
        let dot = boresight[0] * sun_body[0]
            + boresight[1] * sun_body[1]
            + boresight[2] * sun_body[2];
        let bn = (boresight[0] * boresight[0]
            + boresight[1] * boresight[1]
            + boresight[2] * boresight[2])
        .sqrt();
        let sn = (sun_body[0] * sun_body[0]
            + sun_body[1] * sun_body[1]
            + sun_body[2] * sun_body[2])
        .sqrt();
        self.blinded = if bn < 1e-6 || sn < 1e-6 {
            false
        } else {
            dot / (bn * sn) > self.blinding_angle_rad.cos()
        };
    }

    /// Whether currently blinded.
    #[must_use]
    pub fn blinded(&self) -> bool {
        self.blinded
    }

    /// One measurement of the true quaternion (with noise), or `None`
    /// while blinded.
    pub fn measure(&mut self, rng: &mut SimRng, true_quat: [f32; 4]) -> Option<[f32; 4]> {
        if self.blinded {
            return None;
        }
        // Small-angle noise on the vector part + renormalize.
        let n = [
            rng.gaussian(self.noise_rad) * 0.5,
            rng.gaussian(self.noise_rad) * 0.5,
            rng.gaussian(self.noise_rad) * 0.5,
        ];
        let mut q = true_quat;
        // q ⊗ (1, n/2).
        let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
        q[0] = w - n[0] * x / 2.0 - n[1] * y / 2.0 - n[2] * z / 2.0;
        q[1] = w * n[0] / 2.0 + x + n[1] * z / 2.0 - n[2] * y / 2.0;
        q[2] = w * n[1] / 2.0 - n[0] * z / 2.0 + y + n[2] * x / 2.0;
        q[3] = w * n[2] / 2.0 + n[0] * y / 2.0 - n[1] * x / 2.0 + z;
        let norm = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
        for v in &mut q {
            *v /= norm.max(1e-9);
        }
        Some(q)
    }
}

/// Gyro model: scale-correct rate + bias random walk + white noise.
#[derive(Debug, Clone, Copy)]
pub struct Gyro {
    /// Bias random-walk σ per √s, rad/s.
    pub bias_rw: f32,
    /// White noise σ, rad/s.
    pub noise: f32,
    /// Current bias, rad/s.
    pub bias: [f32; 3],
}

impl Gyro {
    /// New gyro at zero bias.
    #[must_use]
    pub fn new(bias_rw: f32, noise: f32) -> Self {
        Self {
            bias_rw,
            noise,
            bias: [0.0; 3],
        }
    }

    /// One sample of the true rate: the reading drifts as the bias walks.
    pub fn measure(&mut self, rng: &mut SimRng, dt_s: f32, true_rate: [f32; 3]) -> [f32; 3] {
        for b in &mut self.bias {
            *b += rng.gaussian(self.bias_rw * dt_s.sqrt());
        }
        let mut out = [0.0f32; 3];
        for i in 0..3 {
            out[i] = true_rate[i] + self.bias[i] + rng.gaussian(self.noise);
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rng_is_deterministic_per_seed() {
        let mut a = SimRng::new(42);
        let mut b = SimRng::new(42);
        for _ in 0..100 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn gaussian_noise_has_the_right_scale() {
        let mut rng = SimRng::new(7);
        let mut sum = 0.0f32;
        let mut sumsq = 0.0f32;
        let n = 20_000;
        for _ in 0..n {
            let g = rng.gaussian(0.001);
            sum += g;
            sumsq += g * g;
        }
        let mean = sum / n as f32;
        let var = sumsq / n as f32 - mean * mean;
        assert!(mean.abs() < 2e-4, "mean {mean}");
        assert!((var - 1e-6).abs() < 3e-7, "var {var}");
    }

    #[test]
    fn tracker_blinds_when_the_sun_enters_the_fov() {
        let mut t = StarTracker::new(5e-5, 30.0_f32.to_radians());
        t.set_geometry([0.0, 0.0, 1.0], [0.0, 0.0, 1.0]); // sun dead-on
        let mut rng = SimRng::new(1);
        assert!(t.blinded());
        assert!(t.measure(&mut rng, [1.0, 0.0, 0.0, 0.0]).is_none());
        t.set_geometry([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]); // sun 90° off
        assert!(!t.blinded());
        let q = t.measure(&mut rng, [1.0, 0.0, 0.0, 0.0]).expect("visible");
        assert!((q[0] - 1.0).abs() < 0.01, "near identity with noise");
    }

    #[test]
    fn gyro_bias_walks_and_shows_in_the_reading() {
        let mut g = Gyro::new(1e-5, 1e-4);
        let mut rng = SimRng::new(9);
        let mut mean_bias = 0.0f32;
        for _ in 0..5_000 {
            let reading = g.measure(&mut rng, 0.01, [0.0; 3]);
            mean_bias += reading[0];
        }
        // A static true rate with a wandering bias: the mean reading is
        // small (bias is zero-mean) but NOT identically zero.
        assert!(mean_bias.abs() < 0.05, "zero-mean drift: {mean_bias}");
        assert!(g.bias.iter().any(|b| b.abs() > 1e-7), "bias walked");
    }
}
