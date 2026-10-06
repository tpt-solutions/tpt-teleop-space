#![warn(missing_docs)]
#![allow(incomplete_features)]
#![feature(portable_simd)]
//! `tpt-t-space-rover` — planetary surface operations (spec §4.4 / §5.1).
//!
//! * [`odometry`] — wheel odometry (differential drive) with slip
//!   detection against the commanded kinematics.
//! * [`vio`] — visual-inertial odometry: `f32x4`-based feature tracking
//!   fused with gyro rates; keeps the pose estimate honest when the
//!   wheels slip.
//! * [`hazards`] — autonomous hazard avoidance consuming the
//!   `tpt-t-space-predictive` DEM + planner: the 25°-slope/15 m-detour
//!   reflex, entirely onboard.
//! * [`science`] — multi-sol science sequencing (deploy arm, press drill,
//!   extract core, seal tube, stow) with per-step torque/position
//!   verification and automatic rollback.
//! * [`wheel_health`] — motor-current + visual-wear trending for
//!   impending wheel-failure detection (Curiosity-style damage).
//!
//! Nightly-only (`portable_simd`); excluded from the stable workspace.

pub mod hazards;
pub mod odometry;
pub mod science;
pub mod vio;
pub mod wheel_health;
