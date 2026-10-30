#![warn(missing_docs)]
//! `tpt-t-space-sim` — the headless simulator (spec §8).
//!
//! Testing space software on Earth means mimicking conditions that are
//! fundamentally impossible to replicate perfectly; the simulator
//! provides the physics and fault models every other crate is validated
//! against:
//!
//! * [`orbit`] — 6DOF orbital mechanics: J2 oblateness, atmospheric
//!   drag, third-body gravity, RK4-integrated;
//! * [`wheels`] — reaction wheel / CMG dynamics: torque-command response
//!   with speed saturation and momentum accumulation;
//! * [`sensors`] — sensor noise models: star-tracker blinding (sun in
//!   FOV), gyro bias random walk, IMU noise;
//! * [`radiation_env`] — the radiation environment: deterministic
//!   bit-flip injection into memory regions to exercise TMR voting and
//!   memory scrubbing;
//! * [`dtn_link`] — DTN network blackout and delay profiles (Mars–Earth
//!   light time, occultation windows);
//! * [`clock`] — the variable-speed simulation clock: multi-year cruise
//!   phases or complex rendezvous in minutes of wall time;
//! * [`replay`] — the shared replay file format (CSV) for the visualizer
//!   and the `tpts` CLI;
//! * [`scenario`] — the canned LEO survey and the spec §6 Mars flow.
//! * [`profile`] — mission-profile presets (LEO cubesat, Mars rover, GEO servicer);
//! * [`campaign`] — seeded fault-injection sweeps over SEU rate and blackout length.
//!
//! Everything is deterministic: one seed drives the whole sim, so a
//! failing scenario reproduces exactly.

pub mod campaign;
pub mod checkpoint;
pub mod clock;
pub mod dtn_link;
pub mod orbit;
pub mod profile;
pub mod radiation_env;
pub mod replay;
pub mod scenario;
pub mod script;
pub mod sensors;
pub mod wheels;
