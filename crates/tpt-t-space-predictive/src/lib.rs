#![warn(missing_docs)]
//! `tpt-t-space-predictive` — onboard model-predictive control and
//! autonomous decision-making (spec §4.4).
//!
//! A Mars rover cannot wait 12 minutes for Earth to tell it to stop before
//! driving off a cliff. This crate is the onboard autonomy core:
//!
//! * [`mpc`] — bounded linear MPC (projected-gradient QP) over fixed
//!   buffers: a finite-horizon controller for docking approaches, station
//!   keeping, and rover guidance, deterministic per cycle.
//! * [`dem`] — local Digital Elevation Model generation from stereo camera
//!   disparities at 10 Hz, with slope/normal analysis and hazard flagging.
//! * [`planner`] — the candidate-trajectory path planner: thousands of
//!   arc candidates evaluated in a structure-of-arrays layout (which the
//!   optimizer auto-vectorizes on stable Rust) selecting the safest path
//!   within tilt and traction limits — the <50 ms replan budget.
//! * [`decision`] — the autonomous decision framework both the rover
//!   (hazard avoidance) and the orbital servicer (docking approach)
//!   consume: an `Evaluator` trait plus a fixed-top-N selection loop.
//!
//! Everything runs on static buffers: no allocation per cycle, no locks,
//! no panics — corrupted sensor input degrades the plan, never the loop.

pub mod decision;
pub mod dem;
pub mod mpc;
pub mod planner;
