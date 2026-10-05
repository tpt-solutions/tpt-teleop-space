#![warn(missing_docs)]
//! `tpt-t-space-dtn` — delay-tolerant networking for the deep-space link
//! (spec §4.2).
//!
//! TCP/IP fails catastrophically at 20-minute round-trip times and
//! multi-hour link blackouts (planetary occultation); this crate implements
//! the store-and-forward stack instead:
//!
//! * [`bundle`] — BPv7-style bundle records with custody transfer, archived
//!   by rkyv *directly into pre-allocated NVRAM sectors* — zero dynamic
//!   memory during blackouts, and an archived bundle maps back out of
//!   NVRAM with zero copies.
//! * [`ltp`] — Licklider Transmission Protocol sessions: block
//!   segmentation, checkpoint/report/ack control flow, exact resume after
//!   link loss.
//! * [`link`] — the link manager tying it together: bundles persist
//!   through link drops with a per-sector transmission cursor (resume
//!   exactly where the link died), plus the hash-verified telemetry log
//!   whose chain digest proves to the ground that reconnected downlink is
//!   complete and ordered.
//!
//! No allocation after construction, no locks, no panics: every rejection
//! is a typed error the loop routes through fault management.

pub mod bundle;
pub mod link;
pub mod ltp;
