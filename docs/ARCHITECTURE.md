# Architecture — tpt-teleop-space

> Status: living document. Grows as crates land; the crate map below is the
> full v1 target (spec §3).

## Workspace Map

```
crates/
  tpt-t-domain-bridge/      # DTI trait, wire types, safety FSM, assist API (Phase 1)
  tpt-t-space-core/         # mission FSM, lock-free bus, event loop, snapshots, wire prelude (Phase 2)
  tpt-t-space-radiation/    # memory scrubbing, TMR voter, SEU recovery (Phase 3)
  tpt-t-space-ccsds/        # zero-copy CCSDS TM/TC parsers (Phase 4)
  tpt-t-space-dtn/          # BPv7 bundles, LTP, NVRAM store-and-forward (Phase 5)
  tpt-t-space-thermal/      # heaters, radiators, battery temps → Safe Mode (Phase 6)
  tpt-t-space-power/        # solar pointing, battery cycling, load shedding (Phase 6)
  tpt-t-space-predictive/   # MPC, stereo DEM, path planner framework (Phase 7)
  tpt-t-space-satellite/    # ADCS: SIMD MEKF, wheels, momentum dumping — nightly (Phase 8)
  tpt-t-space-rover/        # odometry, VIO, hazard avoidance, science sequencing — nightly (Phase 9)
  tpt-t-space-orbital/      # RPO: relative nav, docking, 7-DOF arm (Phase 10)
  tpt-t-space-edge/         # no_std bare-metal RTOS layer, watchdogs (Phase 11)
  tpt-t-space-teleop/       # DomainTeleopInterface adapter, predictive display, DTN transport (Phase 12)
  tpt-t-space-sim/          # headless 6DOF orbital mechanics, sensor + radiation + DTN models (Phase 13)
  tpt-t-space-visualizer/   # egui renderer — excluded from workspace (Phase 13)
```

## Dependency Direction

```
                tpt-t-domain-bridge  (universal teleop contract)
                        ▲
tpt-t-space-core ◀── domain crates (radiation, ccsds, dtn, thermal, …)
        ▲                       ▲
        └──── vehicle crates (predictive → orbital; satellite/rover nightly) ── adapter
                                     (tpt-t-space-teleop) ◀── sim
```

- `tpt-t-space-core` is the shared foundation: mission state machine
  (Cruise → Approach → Operations → Safe Mode), lock-free bus, central event
  loop, persistent snapshot store, CRC32, and the rkyv wire-type prelude every
  space crate re-exports for zero-copy interchange.
- `tpt-t-domain-bridge` is the universal teleop contract (vendored from
  `tpt-teleop` during co-development; see `docs/quickstart.md`).
- Vehicle crates consume domain crates; the `tpt-t-space-teleop` adapter is
  the only crate that speaks DTI to the core teleop system.
- `tpt-t-space-satellite` and `tpt-t-space-rover` are nightly-only
  (`portable_simd`) and excluded from the stable workspace; other crates
  depend on their *wire types* through `tpt-t-space-core`, never on the
  nightly crates directly, so the stable workspace builds without them.

## Crate Responsibilities

Each crate owns one spec section and one performance budget:

| Crate | Spec | Budget |
|-------|------|--------|
| `radiation` | §4.1 | <50µs TMR vote-and-reset-trigger, zero alloc |
| `ccsds` | §3 / §7 | zero-copy TM/TC parse, no per-packet allocation |
| `dtn` | §4.2 | multi-hour blackout survival, NVRAM-resident bundles |
| `thermal`/`power` | §4.5 | single atomic zero-alloc Safe Mode transition |
| `predictive` | §4.4 | <50ms full replan (40ms for a 15m detour) |
| `satellite` | §4.3 | <200µs ADCS cycle @ 100Hz+, sub-arcsecond pointing |
| `rover` | §4.4 | 10Hz DEM, 25° slope → 15m detour <50ms |
| `edge` | §3 | no_std, direct registers, watchdog feed |
| `sim` | §8 | variable-speed (years of cruise in minutes) |

## Design Invariants

1. **No async runtime** — thread-per-core, event loops over lock-free rings.
2. **No serde/bincode** — rkyv zero-copy with `#[repr(C)]` POD-like records;
   bundles serialize straight into pre-allocated NVRAM sectors.
3. **No locks on the hot path** — SPSC rings and atomics only; enforced by
   `tools/lock-audit.sh` as a CI hard gate.
4. **Deterministic control** — every safety/mission decision is a traceable
   FSM transition, never ML.
5. **Bounded memory** — all streaming structures are fixed-capacity (rings,
   NVRAM sector pools, fixed-size DEM grids).
6. **Certification posture** (spec §2) — no dynamic allocation after
   initialization, no recursion, no panics in the hot path, complete
   traceability of state transitions (ECSS-E-ST-40C / NASA-STD-8739.8
   readiness; see `tools/` audits and `docs/fmea-safe-mode.md`).
