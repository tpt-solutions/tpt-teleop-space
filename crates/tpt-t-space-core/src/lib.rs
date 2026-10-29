#![warn(missing_docs)]
//! `tpt-t-space-core` — the shared foundation for the `tpt-teleop-space`
//! workspace.
//!
//! Provides:
//! * [`machine`] — the mission state machine (Cruise → Approach → Operations
//!   → Safe Mode) for rovers, orbiters, and deep-space probes.
//! * [`bus`] — lock-free single-producer / single-consumer ring buffers used
//!   for zero-alloc inter-crate event routing on the hot path.
//! * [`event_loop`] — a central event loop that drains the bus and
//!   dispatches events without allocating per tick.
//! * [`snapshot`] — persistent state snapshotting into a caller-owned NVRAM
//!   byte buffer, CRC32-guarded, so the loop survives processor resets
//!   without losing mission context.
//! * [`safe_mode`] — the single, atomic, zero-allocation Safe Mode
//!   transition shared by `thermal`, `power`, and every anomaly source
//!   (payload shed → sun-point slew → low-rate DTN beacon).
//! * [`crc`] — table-free CRC32 shared by the scrubber, DTN, and snapshots.
//! * [`wire`] — the zero-copy rkyv wire-type prelude shared by the other
//!   space crates (poses, attitudes, body commands, health).
//!
//! Design invariants (inherited from the `tpt-teleop` sibling repos): no
//! async runtime, no `serde`, no channels-with-mutexes. The forward data
//! plane makes no per-event heap allocations and takes no locks on the
//! steady-state path.

pub mod blackbox;
pub mod bus;
pub mod crc;
pub mod event_loop;
pub mod machine;
pub mod safe_mode;
pub mod snapshot;
pub mod wire;
