# tpt-teleop-space

**Tele-Presence Teleoperation — Extraterrestrial & Orbital Operations** — a
hyper-optimized, zero-bloat, fault-tolerant Rust middleware workspace for
planetary rovers, satellite constellations, orbital servicing vehicles, and
deep-space probes.

Sister repo to [`tpt-teleop`](https://github.com/tpt-solutions/tpt-teleop)
(video/control transport), [`tpt-teleop-marine`](https://github.com/tpt-solutions/tpt-teleop-marine),
[`tpt-teleop-fleet`](https://github.com/tpt-solutions/tpt-teleop-fleet), and
the other `tpt-teleop-*` domain repos. The repository retains the full
`tpt-teleop-space` name; all internal crates use the simplified
`tpt-t-space-` prefix.

Built for the most isolated, unforgiving environment there is — where a Mars
rover waits 4–24 light-minutes for Earth, a probe is unrecoverable for
decades, and a single flipped bit can end a billion-dollar mission. Every
design choice prioritizes mission survival, data integrity, and absolute
reliability: a custom DTN stack that survives multi-hour blackouts, a
software radiation-hardening layer (memory scrubbing + TMR voting),
deterministic SIMD-accelerated ADCS control, and predictive onboard autonomy
— all with no async runtime, no serde, no channels-with-mutexes.

## Status

🚧 **In development** — working through the phase plan in
[`todo.md`](todo.md) (foundation → bridge → core → radiation → space
protocols → DTN → survival → autonomy → vehicles → edge → teleop → sim →
integration → compliance → release). See [`docs/quickstart.md`](docs/quickstart.md)
to build it.

| CI | Lint | Deps |
|----|------|------|
| ![build](https://github.com/tpt-solutions/tpt-teleop-space/actions/workflows/ci.yml/badge.svg) | ![lint](https://github.com/tpt-solutions/tpt-teleop-space/actions/workflows/lint.yml/badge.svg) | ![deny](https://github.com/tpt-solutions/tpt-teleop-space/actions/workflows/lint.yml/badge.svg) |

## Workspace Crates

| Crate | Purpose |
|-------|---------|
| `tpt-t-domain-bridge` | `DomainTeleopInterface` (DTI) trait, wire types, universal safety FSM, assist API *(vendored from `tpt-teleop`)* |
| `tpt-t-space-core` | Mission state machine (Cruise → Approach → Operations → Safe Mode), lock-free message bus, event loop, persistent snapshots, rkyv wire prelude |
| `tpt-t-space-radiation` | Continuous memory scrubbing (Hamming + CRC32), <50µs TMR voter, SEU detection & recovery |
| `tpt-t-space-ccsds` | Zero-allocation, zero-copy CCSDS TM/TC packet parsers |
| `tpt-t-space-dtn` | Custom BPv7-style bundles with custody transfer, LTP, NVRAM store-and-forward across multi-hour blackouts |
| `tpt-t-space-thermal` | Heater/radiator/battery monitoring → Safe Mode triggers |
| `tpt-t-space-power` | Solar pointing, battery cycling, eclipse load shedding, dust-storm survival |
| `tpt-t-space-predictive` | Onboard MPC, stereo DEM at 10Hz, trajectory planner, autonomous decision framework |
| `tpt-t-space-satellite` | ADCS: SIMD MEKF, reaction wheels, momentum dumping, formation flying *(nightly crate)* |
| `tpt-t-space-rover` | Odometry, VIO, hazard avoidance, science sequencing, wheel health *(nightly crate)* |
| `tpt-t-space-orbital` | Rendezvous & proximity ops: relative nav, autonomous docking, 7-DOF arm |
| `tpt-t-space-edge` | `no_std` bare-metal RTOS layer, register-level peripherals, watchdogs |
| `tpt-t-space-teleop` | `DomainTeleopInterface` adapter, predictive display, command sequencing, DTN transport |
| `tpt-t-space-sim` | Headless 6DOF simulator: orbital mechanics, sensor noise, radiation injection, DTN blackouts |
| `tpt-t-space-visualizer` | `egui` visualizer: trajectories, attitude, terrain, budgets *(excluded crate)* |

## The Zero-Allocation Data Path

```
Sensor ──Ingest──▶ Fuse/Route ──▶ Mission state (in-place)
                                          │
                                 Serialize ──▶ DTN/NVRAM sector (rkyv, in place)
Total allocations (steady state): 0        Total mutex locks: 0
```

The forward data plane is built on lock-free SPSC rings (`tpt-t-space-core`)
and proven zero-alloc on the hot path; see `tools/lock-audit.sh` (hard gate),
`tools/alloc-audit.sh`, `tools/no-recursion.sh`, and
`tools/no-panic-audit.sh`.

## Developer Experience

Path deps during co-development: `tpt-t-space-*` crates depend on
`tpt-t-space-core` and, where relevant, on `tpt-t-domain-bridge` (vendored
via path dep). Nightly is required **only** for `tpt-t-space-satellite` and
`tpt-t-space-rover` (`#![feature(portable_simd)]`); both are excluded from
the stable workspace and build in isolation. See
[`docs/quickstart.md`](docs/quickstart.md).

## Licensing

Dual-licensed under either of:

 * Apache License, Version 2.0 — [LICENSE-APACHE](LICENSE-APACHE)
 * MIT license — [LICENSE-MIT](LICENSE-MIT)

Copyright © 2026 TPT Solutions.

Dependency policy ("the MIT chain"): dependencies are restricted to MIT,
BSD-2/3-Clause, ISC, Zlib, and MPL-2.0. Dual MIT/Apache crates are resolved
strictly under MIT; strictly Apache-2.0-only crates are banned. Enforced by
[`deny.toml`](deny.toml) in CI, alongside the spec §7 function bans
(nalgebra/ndarray, tokio, serde/bincode, heavy CCSDS stacks, …).
