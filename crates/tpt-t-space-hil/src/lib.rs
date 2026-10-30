//! `tpt-t-space-hil` — the Phase 16 bring-up rig, run in simulation.
//!
//! Phase 16 needs flight-class hardware, radiation beam time, a star tracker
//! rig, a light-time test rig and a thermal-vacuum chamber. None of that is
//! available in this repository. Each module here stands in for one item
//! with a model of the missing hardware, and runs the shipped workspace code
//! against it:
//!
//! * [`bringup`] — a simulated dev board: SRAM with stuck-at faults, the
//!   watchdog kick register, and NVRAM that survives a power cycle.
//! * [`watchdog`] — the real `edge::wdt::Watchdog` fed by a simulated loop.
//! * [`scrub`] — the real Hamming scrubber under simulated upsets, swept over
//!   rate and scrub period.
//! * [`timing`] — sensor feeds and their transport latency against the
//!   control loop's freshness limits.
//! * [`link`] — DTN commands over a light-time and occultation profile.
//! * [`headroom`] — host timings of the hot paths, and how much slower a
//!   target could be before each spec budget fails.
//! * [`environment`] — a battery temperature cycle through the real thermal
//!   subsystem.
//!
//! **These are simulation results.** They test the logic under modelled
//! conditions and cannot show how the real parts behave. The hardware items
//! in `todo.md` stay open until the same checks run on real hardware.

#![warn(missing_docs)]

pub mod bringup;
pub mod environment;
pub mod headroom;
pub mod link;
pub mod scrub;
pub mod timing;
pub mod watchdog;
