//! Wheel odometry: differential-drive integration with slip detection.
//!
//! Per tick the encoders report wheel angles; the unicycle delta
//! `(Δs, Δθ)` integrates into the site-frame pose. The commanded
//! kinematics provide the no-slip expectation — when the measured arc
//! diverges from the commanded one, [`Odometry::slip_ratio`] leaves the
//! healthy band and the VIO fusion (not the wheels) is trusted.

/// Rover geometry and limits.
#[derive(Debug, Clone, Copy)]
pub struct RoverGeometry {
    /// Wheel track (left↔right contact distance), metres.
    pub track_m: f32,
    /// Wheel radius, metres.
    pub wheel_radius_m: f32,
    /// Encoder counts per wheel revolution.
    pub counts_per_rev: f32,
}

impl RoverGeometry {
    /// A Mars-class medium rover.
    #[must_use]
    pub fn typical() -> Self {
        Self {
            track_m: 1.4,
            wheel_radius_m: 0.15,
            counts_per_rev: 4096.0,
        }
    }
}

/// Site-frame pose (local level, x north-ish/forward of the sol).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SitePose {
    /// Position, metres.
    pub x_m: f32,
    /// Position, metres.
    pub y_m: f32,
    /// Heading, radians.
    pub yaw_rad: f32,
}

/// One tick of encoder deltas (signed counts) + what was commanded.
#[derive(Debug, Clone, Copy)]
pub struct EncoderTick {
    /// Left wheel delta, counts (positive = forward).
    pub left_counts: f32,
    /// Right wheel delta, counts.
    pub right_counts: f32,
    /// Commanded arc length this tick, metres (from the planner/follower).
    pub commanded_arc_m: f32,
}

/// Wheel odometry integrator.
#[derive(Debug, Clone, Copy)]
pub struct Odometry {
    geom: RoverGeometry,
    /// Integrated site-frame pose.
    pub pose: SitePose,
    /// Accumulated distance travelled, metres.
    pub distance_m: f32,
    /// EWMA of the slip ratio (measured arc / commanded arc).
    slip_ewma: f32,
}

impl Odometry {
    /// New odometry at the given pose.
    #[must_use]
    pub fn new(geom: RoverGeometry, pose: SitePose) -> Self {
        Self {
            geom,
            pose,
            distance_m: 0.0,
            slip_ewma: 1.0,
        }
    }

    /// Integrate one encoder tick.
    pub fn step(&mut self, tick: &EncoderTick) {
        let m_per_count =
            2.0 * core::f32::consts::PI * self.geom.wheel_radius_m / self.geom.counts_per_rev;
        let dl = tick.left_counts * m_per_count;
        let dr = tick.right_counts * m_per_count;
        let ds = (dl + dr) / 2.0;
        let dtheta = (dr - dl) / self.geom.track_m;
        // Exact unicycle arc integration.
        let yaw_mid = self.pose.yaw_rad + dtheta / 2.0;
        self.pose.x_m += ds * yaw_mid.cos();
        self.pose.y_m += ds * yaw_mid.sin();
        self.pose.yaw_rad += dtheta;
        self.distance_m += ds.abs();
        // Slip EWMA against the command.
        if tick.commanded_arc_m.abs() > 1e-6 {
            let ratio = ds.abs() / tick.commanded_arc_m.abs();
            self.slip_ewma = 0.9 * self.slip_ewma + 0.1 * ratio;
        }
    }

    /// Slip ratio (1.0 = perfect traction; <1 = sinking/spinning).
    #[must_use]
    pub fn slip_ratio(&self) -> f32 {
        self.slip_ewma
    }

    /// Whether slip exceeds the trusted-wheel band (the VIO takes over).
    #[must_use]
    pub fn slipping(&self) -> bool {
        !(0.85..=1.15).contains(&self.slip_ewma)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn counts_for(metres: f32, geom: &RoverGeometry) -> f32 {
        metres * geom.counts_per_rev / (2.0 * core::f32::consts::PI * geom.wheel_radius_m)
    }

    #[test]
    fn straight_drive_integrates_forward() {
        let geom = RoverGeometry::typical();
        let mut odo = Odometry::new(geom, SitePose::default());
        let c = counts_for(0.5, &geom);
        for _ in 0..10 {
            odo.step(&EncoderTick {
                left_counts: c,
                right_counts: c,
                commanded_arc_m: 0.5,
            });
        }
        assert!((odo.pose.x_m - 5.0).abs() < 1e-3, "x={}", odo.pose.x_m);
        assert!(odo.pose.y_m.abs() < 1e-4);
        assert!(odo.pose.yaw_rad.abs() < 1e-6);
        assert!(!odo.slipping());
    }

    #[test]
    fn differential_counts_turn_the_rover() {
        let geom = RoverGeometry::typical();
        let mut odo = Odometry::new(geom, SitePose::default());
        // One full circle: ds per tick = track (Δθ=1 rad per tick… simpler:
        // drive with Δθ = 0.1 rad per tick, 10 ticks → 1 rad total.
        let dtheta = 0.1f32;
        let ds = 0.1f32; // arc per tick
        // Per-wheel arcs: dl = ds − Δθ·track/2, dr = ds + Δθ·track/2.
        let dl = ds - dtheta * geom.track_m / 2.0;
        let dr = ds + dtheta * geom.track_m / 2.0;
        for _ in 0..10 {
            odo.step(&EncoderTick {
                left_counts: counts_for(dl, &geom),
                right_counts: counts_for(dr, &geom),
                commanded_arc_m: ds,
            });
        }
        assert!(
            (odo.pose.yaw_rad - 1.0).abs() < 1e-4,
            "yaw={}",
            odo.pose.yaw_rad
        );
        let _ = dl;
    }

    #[test]
    fn wheel_slip_is_detected_against_the_command() {
        let geom = RoverGeometry::typical();
        let mut odo = Odometry::new(geom, SitePose::default());
        let c = counts_for(0.25, &geom);
        // Commanded 0.5 m per tick, wheels only deliver half: sinking in
        // loose regolith.
        for _ in 0..40 {
            odo.step(&EncoderTick {
                left_counts: c,
                right_counts: c,
                commanded_arc_m: 0.5,
            });
        }
        assert!(odo.slipping(), "slip ratio {}", odo.slip_ratio());
        assert!(odo.slip_ratio() < 0.85);
    }
}
