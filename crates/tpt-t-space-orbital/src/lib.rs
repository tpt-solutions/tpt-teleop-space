#![warn(missing_docs)]
//! `tpt-t-space-orbital` — rendezvous & proximity operations (spec §3 /
//! §5.3).
//!
//! * [`nav`] — 6DOF relative navigation fusing LiDAR, visual markers, and
//!   GPS (LEO-only) into a cm-level relative state; per-sensor health
//!   gating drops degraded inputs instead of averaging them in.
//! * [`docking`] — autonomous docking for final approach: the
//!   `tpt-t-space-predictive` MPC drives the closing profile under a
//!   plume-impingement keep-out shell and a collision keep-out sphere.
//! * [`arm`] — the 7-DOF servicing arm: damped-least-squares inverse
//!   kinematics (no-panic under singular Jacobians), manipulability-based
//!   singularity avoidance, and payload mass estimation from joint torques.
//!
//! Stable crate — the SIMD-heavy estimation lives in the nightly
//! satellite/rover crates; this one flies on the deterministic stack.

pub mod arm;
pub mod docking;
pub mod nav;
