#![warn(missing_docs)]
#![allow(incomplete_features)]
#![feature(portable_simd)]
//! `tpt-t-space-satellite` — the Attitude Determination and Control System
//! (spec §4.3).
//!
//! A satellite tumbling out of control is a mission-killer: the ADCS loop
//! runs at 100 Hz+ with absolute determinism and completes in <200 µs on a
//! pinned real-time core.
//!
//! * [`simdv`] — `f32x4` helpers (quaternion multiply, normalize, 3×3
//!   matvec) on `core::simd`.
//! * [`mekf`] — the SIMD-accelerated Multiplicative Extended Kalman Filter
//!   fusing star tracker, sun sensor, and gyros into the quaternion
//!   estimate. No panics: the gain inversion uses a bounded Cholesky with
//!   a condition-number check and falls back to a diagonal solution under
//!   corrupted sensor data.
//! * [`wheels`] — reaction wheel control with speed limits, momentum
//!   accumulation, magnetorquer dumping, and orbit-maintenance maneuvers.
//! * [`adcs`] — the pinned-core control loop tying it together, with
//!   TMR-voted quaternion updates (`tpt-t-space-radiation`) and the
//!   sub-arcsecond pointing mode.
//! * [`formation`] — multi-satellite relative orbit maintenance over
//!   inter-satellite links (Clohessy–Wiltshire).
//!
//! Nightly-only (`portable_simd`); excluded from the stable workspace.

pub mod adcs;
pub mod formation;
pub mod mekf;
pub mod simdv;
pub mod wheels;
