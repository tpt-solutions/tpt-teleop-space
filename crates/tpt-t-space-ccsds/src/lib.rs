#![warn(missing_docs)]
#![forbid(unsafe_code)]
//! `tpt-t-space-ccsds` — zero-allocation, zero-copy CCSDS packet parsing
//! (spec §3 / §7).
//!
//! Replaces the heavy CCSDS libraries banned by the dependency matrix: the
//! parsers here return *borrowed views* over caller-owned byte slices, so a
//! parsed packet costs one Rust struct on the stack and nothing on the heap.
//! Malformed input is rejected with [`None`]/[`ParseError`] — never a panic
//! — which the deterministic mutation fuzzer and the chunk-split property
//! tests prove across arbitrary truncation points.
//!
//! # Modules
//!
//! * [`packet`] — the CCSDS Space Packet primary header (CCSDS 133.0-B-2):
//!   the shared 6-byte header of both telemetry and telecommand packets.
//! * [`tm`] — TM source packets (CCSDS 132.0-B-3) and loss-tolerant packet
//!   stream extraction over downlink buffers.
//! * [`tc`] — TC packets (CCSDS 232.0-B-3) with optional CRC-16 error
//!   control validation.

pub mod defs;
pub mod packet;
pub mod tc;
pub mod tm;
