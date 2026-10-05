#![warn(missing_docs)]
//! `tpt-t-domain-bridge` — the universal teleoperation adapter contract.
//!
//! This is the Domain Teleoperation Interface (DTI) defined by the tpt-teleop
//! integration model (`bridge spec.txt`): every domain repo implements
//! [`DomainTeleopInterface`] so the core teleop system can drive any robot —
//! rover, satellite, orbital servicer — without knowing what it is.
//!
//! # Placement note
//!
//! The canonical home for this crate is the `tpt-teleop` repo (it is the one
//! crate domain repos *depend on*, not implement). During co-development it is
//! vendored in the `tpt-teleop-space` workspace so the domain crates can use a
//! plain path dependency; when the `tpt-teleop` repo publishes it, swap the
//! path dependency for the crates.io/git dependency — the public API here is
//! the stable contract. The vendored modules (`wire`, `dti`, `safety`,
//! `assist`) are byte-for-byte the universal contract; only the doc examples
//! name the space domain.
//!
//! # Modules
//!
//! * [`wire`] — zero-copy rkyv wire types (`ControlCommand`, `SensorFeed`,
//!   `DomainState`, `DomainTelemetry`, haptic types).
//! * [`dti`] — the [`DomainTeleopInterface`] trait every domain implements.
//! * [`safety`] — the universal autonomy↔teleop safety state machine.
//! * [`assist`] — the `request_teleop_assistance()` API surface a domain's
//!   safety subsystem calls when autonomy can no longer continue safely.
//!
//! # The legacy `ControlCommand` discrepancy (resolved)
//!
//! `bridge spec.txt` flags that `tpt-t-core::ser::cmd::ControlCommand` in the
//! `tpt-teleop` repo is a flat 56-byte POD type carrying only raw
//! axis/velocity/sequence fields — no `operator_id`, `InputState`, or
//! `ButtonState` — while this crate's [`wire::ControlCommand`] is the full
//! aspirational struct. The reconciliation policy:
//!
//! 1. [`wire::ControlCommand`] (this crate) is canonical going forward.
//! 2. The legacy 56-byte POD is a *transport encoding* of the same command,
//!    not a competing type: its flat axis words map onto
//!    [`wire::InputState::axes`] of [`wire::ControlCommand::primary_input`],
//!    its sequence number / timestamp onto [`wire::ControlCommand::seq`] /
//!    [`wire::ControlCommand::timestamp_us`], and its spare axis words onto
//!    `secondary_input`. `operator_id` has no legacy field; legacy sessions
//!    use [`wire::OperatorId::LEGACY_ANON`].
//! 3. Translation happens once, at the transport boundary (`tpt-t-link`
//!    codec); domain adapters only ever see the canonical type.
//!
//! # Space-domain mapping
//!
//! The space adapter (`tpt-t-space-teleop`) rides the DTI over the deep-space
//! DTN link (`tpt-t-space-dtn`) instead of a realtime network: control
//! commands arrive as *sequences* with custody transfer, and the light-speed
//! delay is masked with predictive display on the operator side. The DTI
//! contract itself is unchanged — see the adapter crate docs.
//!
//! # Design invariants
//!
//! No async runtime, no `serde`, no `std::sync` locks on the data path. Wire
//! types are fixed-size POD-like records (bounded arrays + length fields
//! instead of `Vec`; pixel/point payloads are referenced by offset into
//! out-of-band shared buffers) so archived views can be mapped over shared
//! memory or the wire with zero copies and zero per-event allocations.

pub mod assist;
pub mod dti;
pub mod safety;
pub mod wire;
