#![warn(missing_docs)]
#![cfg_attr(not(test), no_std)]
//! `tpt-t-space-edge` — the bare-metal RTOS execution layer (spec §3 /
//! §7 "Edge Execution").
//!
//! Space-grade microcontrollers (RAD750, LEON3, rad-hard ARM Cortex-R)
//! get no operating system to lean on: this layer provides the minimal,
//! fully static execution environment the workspace's crates run on:
//!
//! * [`reg`] — volatile register-cell abstraction (`read`/`write`/
//!   `modify` over raw addresses), the foundation of every driver;
//! * [`wdt`] — hardware watchdog management: windowed feeding with a
//!   deadline schedule the loop must hit, bite-counter bookkeeping, and
//!   the reset-cause latch the boot path reads;
//! * [`sched`] — a static round-robin task table (compile-time-known
//!   tasks, static stacks by contract) with per-task deadline-miss
//!   accounting for the traceability audits.
//!
//! `no_std` is validated on `aarch64-unknown-none-softfloat` in CI; on
//! the host the same code runs under `cfg(test)` for the unit tests.

pub mod reg;
pub mod sched;
pub mod wdt;
