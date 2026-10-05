#![warn(missing_docs)]
//! `tpt-t-space-radiation` — software fault tolerance for the radiation
//! environment (spec §4.1).
//!
//! Hardware rad-hardening lags commercial silicon; software compensates:
//!
//! * [`scrub`] — continuous background memory scrubbing: every protected
//!   64-bit word carries a Hamming(72,64) SECDED code; a low-priority walk
//!   corrects single-bit upsets (SEUs) before they accumulate into
//!   multi-bit failures, with a CRC32 region check as the backstop.
//! * [`tmr`] — Triple Modular Redundancy: critical calculations execute
//!   three times (separate cores); the voter masks a diverged result,
//!   flags the affected core for a watchdog reset — all in <50 µs with
//!   zero heap allocation.
//! * [`seu`] — SEU detection/recovery bookkeeping: a bounded event log and
//!   counters feeding the telemetry log and the ground health report.
//!
//! The crate is `std`-free in its hot path (no allocation, no locks, no
//! panics); the only `std` surface is test/bench scaffolding.

pub mod scrub;
pub mod seu;
pub mod tmr;
