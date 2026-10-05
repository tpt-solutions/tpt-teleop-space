# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] — v1 Monolithic Release

### Added
- Phase 0: Cargo workspace scaffold, MIT-chain `deny.toml` with the spec §7
  function bans, CI/lint workflows (stable + nightly + bare-metal + visualizer),
  `rust-toolchain.toml`, audit scripts (lock/alloc/recursion/panic), docs.
- Phase 1: `tpt-t-domain-bridge` — DTI trait, zero-copy rkyv wire types,
  universal safety state machine, `request_teleop_assistance()` API
  (vendored from `tpt-teleop` during co-development).
- Phase 2: `tpt-t-space-core` — mission state machine (Cruise → Approach →
  Operations → Safe Mode), lock-free SPSC message bus, central event loop,
  CRC32, persistent snapshot store, rkyv wire prelude.
- Phase 3: `tpt-t-space-radiation` — Hamming(72,64) memory scrubber with
  CRC32 region walk, TMR voter with core-reset triage (<50µs budget), SEU
  detection/recovery bookkeeping, zero-alloc proof.
- Phase 4: `tpt-t-space-ccsds` — zero-allocation, zero-copy CCSDS Space
  Packet / TM / TC parsers with deterministic fuzz + property tests.
- Phase 5: `tpt-t-space-dtn` — BPv7-style bundles with custody transfer,
  LTP sessions with checkpoints/reports, NVRAM sector store with in-place
  rkyv archival, exact-resume store-and-forward, hash-chained telemetry log,
  multi-hour blackout validation.
- Phase 6: `tpt-t-space-thermal` (heater/radiator/battery monitoring with
  hysteresis), `tpt-t-space-power` (solar pointing, battery cycling, eclipse
  load shedding, dust-storm survival trigger), and the shared single atomic
  zero-alloc Safe Mode transition in `core` (payload shed → sun-point slew →
  low-rate DTN beacon).
- Phase 7: `tpt-t-space-predictive` — bounded MPC, stereo-DEM generation,
  candidate-trajectory path planner (SoA, auto-vectorizing), and the
  autonomous-decision framework reused by rover hazard avoidance and orbital
  docking (<50ms replan budget).
- Phase 8: `tpt-t-space-satellite` (nightly) — SIMD-accelerated MEKF fusing
  star tracker/sun sensor/gyro, reaction wheel control, momentum dumping,
  orbit maintenance, TMR-voted quaternion updates, no-panic bounded
  pseudo-inverse, sub-arcsecond pointing mode, formation flying
  (<200µs ADCS loop budget).
- Phase 9: `tpt-t-space-rover` (nightly) — wheel odometry, VIO, hazard
  avoidance on the predictive DEM/planner, multi-sol science sequencing with
  sensor-verified steps and rollback, wheel health monitoring, and the 25°
  slope → 15m detour <50ms end-to-end test.
- Phase 10: `tpt-t-space-orbital` — 6DOF relative navigation (LiDAR +
  visual markers + LEO GPS), autonomous docking via MPC with plume
  impingement and keep-out management, 7-DOF arm with damped-least-squares
  IK, singularity avoidance, payload mass estimation.
- Phase 11: `tpt-t-space-edge` — `#![no_std]` bare-metal layer: register
  abstraction, hardware watchdog management, static RTOS task table;
  validated on `aarch64-unknown-none-softfloat`.
- Phase 12: `tpt-t-space-teleop` — the DTI adapter: command translation with
  slew limiting and blend handover, predictive display masking light-speed
  delay, uploaded command sequencing with hazard avoidance, telemetry/image
  bandwidth optimization, DTN custody-verified transport.
- Phase 13: `tpt-t-space-sim` — 6DOF orbital mechanics (J2, drag, third
  body), reaction wheel dynamics, sensor noise models, radiation bit-flip
  injection, DTN blackout/light-time profiles, variable-speed clock; and the
  excluded `tpt-t-space-visualizer` (egui renderer: trajectories, attitude,
  terrain, power/thermal budgets).
- Phase 14: the complete Mars Sample Collection data-flow integration test
  (spec §6), the dependency-matrix conformance audit, cross-workspace
  zero-alloc/zero-lock verification, full `cargo-deny` pass.
- Phase 15: ECSS/NASA-STD readiness reviews, no-alloc/no-recursion/
  no-panic audits, state-transition traceability review, Safe Mode FMEA
  (`docs/fmea-safe-mode.md`).
- Phase 17: stable + nightly clippy clean, all spec benchmarks gated,
  developer docs complete.

## [Unreleased]

### Added
- Initial spec docs (`spec.txt`, `bridge spec.txt`) and phased project todo.
