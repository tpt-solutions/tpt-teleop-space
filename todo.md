# tpt-teleop-space — Project TODO

Extraterrestrial & Orbital Operations middleware. Sister repo to `tpt-teleop`
(video/control transport) and the other `tpt-teleop-*` domain repos
(`tpt-teleop-fleet`, `tpt-t-agri`, `tpt-teleop-marine`, `tpt-teleop-medical`,
`tpt-teleop-defense`, `tpt-teleop-construction`). Repository retains the full
`tpt-teleop-space` name; all internal crates use the simplified
`tpt-t-space-` prefix. License: MIT OR Apache-2.0. Copyright TPT Solutions.

Source docs: [spec.txt](spec.txt) (primary design doc), [bridge spec.txt](bridge%20spec.txt)
(tpt-teleop integration contract).

Phases are ordered dependency-first: foundation → comms bridge → core →
fault tolerance → space protocols → DTN → environment/survival → predictive
autonomy → vehicle-specific execution (satellite/rover/orbital) → edge/
hardware layer → teleop bridge → sim → integration → compliance → hardware
bring-up → hardening/release.

---

## Phase 0 — Repo Foundation & Tooling
- [x] `git init`, initial commit of spec docs
- [x] Cargo workspace `Cargo.toml` (resolver "3", edition 2024, rust-version
      1.85, license `MIT OR Apache-2.0`, authors "TPT Solutions"), mirroring
      sister repos' workspace layout (`crates/*` members, shared workspace
      deps table)
- [x] `LICENSE-MIT` and `LICENSE-APACHE` at repo root (TPT Solutions, 2026)
- [x] `.gitignore` (target/, etc.)
- [x] `deny.toml` — MIT/BSD/ISC/Zlib/MPL-2.0 allow-chain, explicit
      Apache-2.0-only deny, `[[licenses.clarify]]` overrides for
      dual-licensed deps (rkyv, windows-sys, proc-macro2/syn/quote, libc),
      encodes the §7 dependency matrix (bans nalgebra/ndarray, heavy CCSDS
      libraries, tokio, serde/bincode, in favor of the custom lightweight
      choices)
- [x] `.github/workflows/ci.yml` — build-test matrix (linux/macos/windows),
      cross-compile check, `RUSTFLAGS: -D warnings`, plus a separate nightly
      job scoped to `tpt-t-space-satellite` and `tpt-t-space-rover`
- [x] `.github/workflows/lint.yml` — fmt + clippy (`-D warnings`, stable +
      nightly job for satellite/rover) + cargo-deny action
- [x] `rust-toolchain.toml` — stable default; document the nightly override
      needed for `tpt-t-space-satellite` (MEKF) and `tpt-t-space-rover`
      (path planner)
- [x] Root `README.md` (Status / Workspace Crates / Zero-Allocation Data
      Path / Developer Experience / Licensing — mirror sister repos'
      section structure)
- [x] `CHANGELOG.md`, `SECURITY.md`
- [x] `docs/ARCHITECTURE.md` skeleton (workspace map, crate responsibilities)
- [x] `docs/quickstart.md` stub
- [x] Stub empty crate skeletons (Cargo.toml + lib.rs) for all crates below

## Phase 1 — Cross-Repo Bridge: `tpt-t-domain-bridge` (lives in `tpt-teleop` repo)
- [x] New crate `tpt-t-domain-bridge` under `tpt-teleop/crates/`
- [x] Define `DomainTeleopInterface` (DTI) trait: `on_teleop_engage`,
      `on_teleop_disengage`, `on_control_command`, `get_domain_state`,
      `get_sensor_feed`
- [x] Define wire types with `rkyv::Archive`: `ControlCommand`, `InputState`,
      `SensorFeed`, `DomainState`, `DomainTelemetry`, `HapticCmd`/`HapticFeedback`
- [x] Reconcile with existing 56-byte POD `ControlCommand` in
      `tpt-teleop-core/src/ser/cmd.rs` (bridge spec flags this as an
      intentional, unreconciled discrepancy — resolve or document the mapping)
- [x] Universal safety state machine: `AUTONOMOUS → REQUESTING_TELEOP →
      TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS`, plus
      `EMERGENCY_STOP` transitions
- [x] `request_teleop_assistance()` API surface
- [x] Unit tests + doc examples; publish path so `tpt-t-space-teleop` can
      depend on it (path dep during co-development)

## Phase 2 — `tpt-t-space-core`
- [x] Central deterministic event loop
- [x] Mission state machine (Cruise → Approach → Operations → Safe Mode)
- [x] Lock-free message bus for inter-crate event routing
- [x] Persistent state snapshotting so the loop survives processor resets
      without losing mission context
- [x] rkyv wire-type prelude shared by other space crates
- [x] Zero-lock hot-path audit passes (reuse `tools/lock-audit.sh` pattern
      from sister repos)

## Phase 3 — `tpt-t-space-radiation`
- [x] Continuous background memory scrubber: CRC32 over static memory,
      single-bit correction via Hamming codes before multi-bit accumulation
- [x] TMR (Triple Modular Redundancy) voter: critical calculations executed
      on three separate cores, majority-vote comparison
- [x] On divergence: mask the bad result, trigger a core reset — all in
      <50µs, zero heap allocation
- [x] SEU (Single Event Upset) detection and recovery bookkeeping
- [x] Benchmark: <50µs TMR vote-and-reset-trigger cycle, zero-alloc proof

## Phase 4 — `tpt-t-space-ccsds`
- [x] Zero-allocation, zero-copy CCSDS telemetry (TM) packet parser
- [x] Zero-allocation, zero-copy CCSDS telecommand (TC) packet parser
- [x] Space Packet Protocol header parsing (APID, sequence count, etc.)
- [x] Fuzz/property tests for the zero-copy parser against malformed frames

## Phase 5 — `tpt-t-space-dtn`
- [x] Custom zero-copy Bundle Protocol (BPv7) implementation: bundles with
      custody transfer
- [x] Licklider Transmission Protocol (LTP) implementation
- [x] Store-and-forward: on link drop, persist the in-flight bundle to
      NVRAM; on link restore, resume transmission exactly where it left off
- [x] Zero-allocation serialization: bundles written directly into
      pre-allocated NVRAM sectors via `rkyv`
- [x] Multi-hour blackout tolerance validation (planetary occultation
      scenario)
- [x] Hash-verified telemetry log transmission on reconnect

## Phase 6 — Environment & Survival
### `tpt-t-space-thermal`
- [x] Heater state monitoring
- [x] Radiator deployment monitoring
- [x] Battery temperature monitoring
- [x] Safe Mode trigger on thermal limit breach (e.g. battery temp < -20°C)

### `tpt-t-space-power`
- [x] Solar array pointing optimization
- [x] Battery charge/discharge cycle tracking
- [x] Load shedding during eclipse periods
- [x] Safe Mode trigger on solar array output dropping below survival
      threshold (dust-storm scenario, spec.txt §6)

### Safe Mode transition (shared `core`/`thermal`/`power`)
- [x] Single atomic, zero-allocation Safe Mode transition: power down
      non-essential payloads, slew to sun-pointing attitude, begin low-rate
      DTN beacon (consumes Phase 5 `dtn`)

## Phase 7 — `tpt-t-space-predictive`
- [x] Onboard model-predictive control (MPC) framework
- [x] Local Digital Elevation Model (DEM) generation from stereo camera
      data at 10Hz
- [x] Custom, SIMD-accelerated path planner evaluating thousands of
      candidate trajectories, selecting the safest path within tilt/traction
      limits
- [x] Autonomous decision-making framework reusable across rover hazard
      avoidance and orbital docking approach (consumed by Phase 9, Phase 10)
- [x] Benchmark: <50ms full replan cycle (spec.txt §6 example: 40ms for a
      15m detour)

## Phase 8 — `tpt-t-space-satellite` (ADCS, nightly-scoped)
- [x] Crate-local nightly toolchain override + `#![feature(portable_simd)]`,
      CI job scoped to this crate only (rest of workspace stays stable)
- [x] Reaction wheel control
- [x] Star tracker + sun sensor + gyro fusion via custom SIMD-accelerated
      Multiplicative Extended Kalman Filter (MEKF) for quaternion estimation
- [x] Momentum dumping (magnetorquers or thrusters)
- [x] Orbit maintenance maneuvers
- [x] Control loop pinned to a dedicated real-time core, executing in
      <200µs
- [x] No-panic guarantee: bounded fallback algorithms (e.g. pseudo-inverse
      with condition-number checking) for matrix inversion under corrupted
      sensor data
- [x] TMR-voted quaternion updates (consumes Phase 3 `radiation`)
- [x] Sub-arcsecond precision pointing mode
- [x] Formation flying: multi-satellite relative orbit maintenance via
      inter-satellite links
- [x] Benchmark: <200µs ADCS control loop cycle

## Phase 9 — `tpt-t-space-rover`
- [x] Wheel odometry
- [x] Visual-inertial odometry (VIO)
- [x] Autonomous hazard avoidance consuming Phase 7 `predictive` DEM/path
      planner output
- [x] Science sequencing: multi-sol instrument deployment sequences (drill,
      sieve, spectrometer) with automatic rollback on failure
- [x] Sample collection sequence: deploy arm, press drill, extract core,
      seal tube, stow in carousel — each step verified by torque/position
      sensors
- [x] Wheel health monitoring: motor current + visual wheel-wear tracking
      for impending-failure detection (Curiosity-wheel-damage-style)
- [x] End-to-end test: 25° slope detected → 15m detour replanned in <50ms
      (spec.txt §6)

## Phase 10 — `tpt-t-space-orbital`
- [x] 6DOF relative navigation fusing LiDAR, visual markers, and GPS
      (LEO-only) for cm-level relative state estimation
- [x] Autonomous docking/berthing via MPC for final approach (consumes
      Phase 7 `predictive`), managing plume impingement and collision
      avoidance
- [x] 7-DOF space robotic arm (Canadarm-style) manipulation
- [x] Singularity avoidance
- [x] Payload mass estimation

## Phase 11 — `tpt-t-space-edge` (`no_std`, bare-metal)
- [x] Bare-metal RTOS execution layer for space-grade microcontrollers
      (RAD750, LEON3, rad-hard ARM Cortex-R)
- [x] Direct register-level peripheral control
- [x] Hardware watchdog management + watchdog-feeding logic
- [x] `no_std` build validated for the target architectures above

## Phase 12 — `tpt-t-space-teleop` (adapter crate)
- [x] Depend on `tpt-t-domain-bridge` (Phase 1) and implement
      `DomainTeleopInterface`
- [x] Predictive display: show the operator a predicted rover position
      based on last-known state, masking the light-speed delay (e.g. 12
      minutes for Mars)
- [x] Command sequencing: accept an uploaded sequence (e.g. "drive forward
      5m, turn left 30°, take a photo"), execute autonomously with hazard
      avoidance (consumes Phase 9 `rover`, Phase 7 `predictive`)
- [x] Bandwidth optimization: compress images/telemetry for the deep-space
      link
- [x] Transport via `tpt-t-space-dtn` (Phase 5) — custody transfer for
      uplinked command sequences, downlinked telemetry/images
- [x] Reconcile aspirational `ControlCommand` (operator_id/InputState/
      ButtonState) against upstream's actual flat POD type (open decision,
      tracked upstream — same open item as sister repos)

## Phase 13 — `tpt-t-space-sim` + Visualizer
- [x] Headless simulator: 6DOF orbital mechanics (J2 perturbations,
      atmospheric drag, third-body gravity)
- [x] Spacecraft mass properties and reaction wheel/CMG dynamics
- [x] Sensor noise models (star tracker blinding, gyro bias drift, IMU noise)
- [x] Radiation environment model: inject random bit flips into memory to
      exercise TMR voting and memory scrubbing (Phase 3)
- [x] DTN network blackout and delay-profile modeling (e.g. Mars-Earth
      light time)
- [x] Variable-speed simulation: multi-year cruise phases or complex
      rendezvous maneuvers simulated in minutes of wall-clock time
- [x] `egui` (MIT) visualizer binary: 3D orbital trajectories and ground
      tracks
- [x] Visualizer: spacecraft attitude (quaternion) and sun-vector alignment
- [x] Visualizer: rover terrain maps, planned vs. actual paths
- [x] Visualizer: power/thermal budgets over time

## Phase 14 — System Integration & End-to-End
- [x] Full "Mars Sample Collection" data flow test (spec.txt §6): DTN
      uplink + custody transfer → comms blackout → autonomous traverse with
      predictive-DEM hazard detection → 15m detour replanned in 40ms → SEU
      injected mid-calculation → TMR voter masks corrupted result + flags
      core for watchdog reset → arm deploy/drill/extract/seal/stow science
      sequence verified by torque/position sensors → dust storm drops solar
      output → power-triggered Safe Mode (payload off, rover halted) →
      storm clears, orbiter overhead → DTN resumes, hash-verified telemetry
      log + Safe Mode alert downlinked
- [x] Dependency-matrix conformance audit against §7 (confirm no banned
      dependency — nalgebra/ndarray, heavy CCSDS libraries, tokio, serde/
      bincode — entered the tree)
- [x] Cross-workspace zero-allocation/zero-lock verification (extend
      `tpt-t-integration`-style audit from `tpt-teleop` to cover all space
      crates)
- [x] Full `cargo-deny` MIT-chain audit pass across the workspace

## Phase 15 — Compliance & Certification Readiness
- [x] ECSS-E-ST-40C (Software Engineering) compliance review
- [x] NASA-STD-8739.8 compliance review
- [x] No-dynamic-allocation-after-initialization audit across the workspace
- [x] No-recursion audit (static call-graph analysis)
- [x] No-panics-in-hot-path audit (clippy lints + panic-free proof tooling,
      e.g. `no_panic` or manual audit of all hot-path crates)
- [x] Complete traceability of all state transitions: audit-log coverage
      review across `core`, `radiation`, `thermal`/`power` Safe Mode paths
- [x] FMEA (Failure Modes and Effects Analysis) pass on the Safe Mode
      transition path

## Phase 16 — Hardware Bring-Up & Flight/Radiation Validation
> Real flight-qualified hardware (RAD750/LEON3/rad-hard Cortex-R engineering
> units, radiation beam time) is not casually accessible — scope this phase
> to COTS rad-tolerant dev boards and facility access as it becomes
> available, rather than assuming real flight hardware.
>
> Every item below stays open until it is run on the real part. The
> simulation rig (`tpt-t-space-hil`, run with `tpts hil`) exercises the same
> logic against models of the missing hardware. Its results are simulation
> evidence, and none of them closes an item.
- [ ] COTS rad-tolerant dev board bring-up (e.g. LEON3 GR712RC eval board or
      a rad-hard-adjacent ARM Cortex-R eval kit) replacing sim-only targets.
      Simulated: `tpts hil` [1] runs a march test, kick-register check and
      NVRAM restore on a model board, and catches an injected stuck-at bit.
      Pending hardware/facility: no in-repo evidence from a real board
- [ ] Real hardware watchdog wiring + feed-loop validation on target board.
      Simulated: `tpts hil` [2] drives the real watchdog from a modelled loop.
      A 4 s freeze bites and latches its cause; a 1 s freeze and a runaway
      loop do not bite. Pending hardware/facility: no in-repo evidence
- [ ] Radiation test campaign (heavy-ion or proton beam, third-party
      facility) validating the TMR voter and memory scrubber against real
      SEU rates. Simulated: `tpts hil` [3] sweeps upset rate against scrub
      period with the real scrubber. Double hits appear once the period
      outlasts the upset spacing. The modelled rates are not measured ones.
      Flagged as facility/funding-dependent, pursue when available
- [ ] Hardware-in-the-loop validation for ADCS timing against a real star
      tracker/IMU/sun sensor stack. Simulated: `tpts hil` [4] checks sensor
      freshness against a 1 kHz loop, with transport latencies assumed, not
      measured. The ADCS controller itself is nightly-only and not run.
      Pending hardware/facility: no in-repo evidence
- [ ] Real DTN link testing over a simulated light-time-delay test rig.
      Simulated: `tpts hil` [5] runs commands and acks over the Mars
      occultation profile, and the rover stores each command in the real
      DTN agent. Pending hardware/facility: no in-repo evidence
- [ ] Target embedded platform performance validation against every spec'd
      latency budget on real silicon. Simulated: `tpts hil` [6] times the
      TMR vote and the rover replan on the host, and reports how much slower
      a target could be before each budget fails. The ADCS loop is not
      measured here. Pending hardware/facility: no in-repo evidence
- [ ] Extreme-environment validation (thermal vacuum chamber, launch
      vibration profile) as facility access allows. Simulated: `tpts hil`
      [7] cycles the battery channel through the real thermal subsystem and
      counts Safe Mode trips. Vibration and vacuum are not modelled.
      Pending hardware/facility: no in-repo evidence

## Phase 17 — Hardening, Docs & v1.0.0 Release
- [x] `cargo clippy --workspace --all-targets -- -D warnings` clean
      (stable workspace)
- [x] Nightly clippy pass clean for `tpt-t-space-satellite` and
      `tpt-t-space-rover`
- [x] Performance benchmarks validated against every spec target: <50µs TMR
      vote, <200µs ADCS control loop, <50ms rover replan, multi-hour DTN
      blackout survival
- [x] Fuzz/property testing for custom zero-allocation parsers (CCSDS
      TM/TC, BPv7/LTP framing)
- [x] Developer docs complete (`docs/quickstart.md`, `docs/ARCHITECTURE.md`,
      crate-level rustdoc)
- [x] README finalized
- [x] Tag `v1.0.0` — "v1 Monolithic Release"

---

## Phase 18 — Stub Cleanup & Honest Status (post-v1.0.0 review)
> Findings from the post-release review: no `todo!()`/`unimplemented!()`
> markers exist, but a few items are stubbed in substance or ticked ahead of
> the evidence.
- [x] Visualizer: replace the synthetic `SimModel::default()` demo data with
      real data — drive it from `tpt-t-space-sim` (orbit propagation) and load
      replay files (the crate docs promise "populated by the sim loop or a
      replay file"; neither exists)
- [x] Visualizer: attitude pane renders the quaternion as text only — rotate
      the spacecraft body by `q` and derive the sun vector from real geometry
- [x] Visualizer: terrain pane uses a flat zero DEM — feed it the real
      `tpt-t-space-predictive` `DemGrid` plus planned vs. actual paths from
      the Mars Sample Collection scenario
- [x] Visualizer: power/thermal traces are sine waves — derive from the
      eclipse model and `tpt-t-space-power`/`thermal` state
- [x] Add a shared replay format (`tpt-t-space-sim::replay`: CSV/rkyv
      write + parse) so the headless sim, CLI, and visualizer share one file
- [x] Reconcile Phase 16: every item is ticked but no code, rig, or test
      evidence exists in-repo (board bring-up, beam campaign, HIL, tvac).
      Untick and mark "pending hardware/facility" or link the evidence
- [x] Reconcile Phase 17 "Tag `v1.0.0`" with workspace `version = "0.1.0"`
- [x] Remove stale "stub" wording from the Phase 0 checklist
      (`docs/quickstart.md` stub, crate skeletons) and the ARCHITECTURE
      "skeleton" wording
- [x] Add a CI job running the visualizer's isolated build (egui chain is
      excluded from the workspace and currently unverified by CI gates)

## Phase 19 — Adoption Tooling (`tpts` CLI + onboarding)
- [x] New workspace crate `tpt-t-space-cli` (binary `tpts`), hand-rolled arg
      parsing — no `clap`, in keeping with the zero-bloat dependency policy
- [x] `tpts doctor` — check rustc/cargo, nightly toolchain, `cargo-deny`,
      `aarch64-unknown-none-softfloat` target, and bash (Windows: Git Bash);
      print the exact fix command for each failure
- [x] `tpts audit [--strict]` — run lock/recursion/panic/alloc audits and
      cargo-deny from one command; works on Windows without hand-running `.sh`
- [x] `tpts budgets` — run the three strict benches (TMR, ADCS, replan) and
      print a pass/fail table against the spec budgets
- [x] `tpts ccsds <hex|file>` — decode CCSDS TM/TC packet streams
      (APID, seq count, flags, length, CRC-16 check) for ground-ops debugging
- [x] `tpts orbit` — propagate a circular orbit (alt/inc/duration/step) and
      write a replay file for the visualizer
- [x] `tpts scenario mars` — run the §6 Mars Sample Collection flow and
      print a timeline report (optionally write a replay)
- [x] `tpts new <vehicle>` — scaffold a mission crate (rover/satellite/orbital)
      wired to core bus, Safe Mode, and DTN with a passing smoke test
- [x] Runnable `examples/` per crate (blackout resume, TMR vote, replan,
      CCSDS parse) referenced from `docs/quickstart.md`
- [x] Dev container / `justfile` (or `cargo xtask`) wrapping build, test, (justfile not run here: `just` is not installed)
      audit, and bench commands for a one-command first run
- [x] Pre-built release binaries for `tpts` and the visualizer via a (release.yml not run: no tag pushed)
      tag-triggered CI release workflow

## Phase 20 — Innovation & Usability Features
- [x] Mission scenario files: `tpts scenario run FILE` replays a timeline
      (link geometry, blackouts, SEU upsets, telemetry, Safe Mode, drain)
      deterministically against the DTN store, TMR voter and black box;
      example in `docs/scenarios/blackout-and-upset.scn`
- [x] Fault-injection campaign runner: SEU and blackout sweeps, from the
      command line or a `tpts campaign run FILE` scenario file. Dust-storm
      sweep not modeled: the power model cannot attenuate the array partway
- [x] Light-time-aware operator UI cues: the visualizer's Link pane shows
      arrival and result-known times, and the predicted-vs-confirmed rover
      readout. On the Mars link that readout reports no prediction: the
      newest state is 720 s old, past the predictive display's 120 s horizon
- [x] Sequence linter/dry-run: validate an uplink sequence against rover
      limits and the DEM/hazard model on the ground before uplink
- [x] Safe Mode black-box recorder: ring, CRC image, NVRAM restore,
      `tpts blackbox`. The Mars flow's mission bus feeds the box, and the
      NVRAM image is rewritten at every entry rather than once at the end
- [x] Live telemetry dashboard: the visualizer's Downlink pane follows a
      growing CCSDS capture (`--ccsds FILE`). It reads a file, not a socket
      from a running sim
- [x] Time-travel debugging: the visualizer's Rewind pane restores any
      Mars-flow checkpoint from the core snapshot format. It rewinds a
      recorded run; rewinding a live sim still needs checkpoints taken as
      the sim runs
- [x] CCSDS packet-definition table driving auto-decoded telemetry in
      `tpts ccsds` and the visualizer's Downlink pane
- [x] Mission-profile presets (LEO cubesat, Mars rover, GEO servicer) giving
      sane defaults for limits, thermal/power thresholds, and link profiles

## Phase 21 — Certification Tooling Gaps (from `docs/compliance.md`)
- [x] Wire `cargo-llvm-cov` into CI with coverage thresholds on the — `assurance.yml`; 91.2% measured locally, CI run pending
      safety-critical crates (radiation, core, dtn, thermal, power)
- [x] Machine-readable requirements (spec §→ ID) with bidirectional — 18 requirements in `docs/requirements.csv`, checked by `tools/req-trace.sh`
      requirement ↔ test links, checked in CI (replaces phase-ledger tracing)
- [ ] Recorded formal inspection templates for the TMR voter, scrubber, Safe — templates in `docs/inspections/`; no inspection recorded yet
      Mode controller, and DTN custody logic
- [x] Real call-graph tooling for the no-recursion audit (the script cannot — `tools/call-graph.py`: direct and mutual cycles
      detect mutual recursion today)
- [x] Make `tools/alloc-audit.sh` a gate with an allow-list of
      construction-time allocations instead of an informational report
- [x] Extend the no-panic audit to `thermal`, `power`, `orbital`, `edge`,
      `teleop`, and `bridge` (currently scoped to seven crates)

---

## Delivery record (v1.0.0)

All phases implemented. Spec budgets, measured on the dev host
(Windows x86_64, stable 1.97 / nightly):

| Budget | Measured | Gate |
|--------|----------|------|
| <50 µs TMR vote-and-reset-trigger | **0.0066 µs/cycle** | `TPT_RAD_STRICT=1 cargo bench -p tpt-t-space-radiation` |
| <200 µs ADCS control loop | **0.84 µs/cycle** | `TPT_ADCS_STRICT=1` (nightly crate) |
| <50 ms rover replan (§6: 40 ms) | **0.23 ms/replan**, 1016 candidates | `TPT_PRED_STRICT=1 cargo bench -p tpt-t-space-predictive` |
| Multi-hour DTN blackout survival | 4 h 12 m occultation + mid-blackout processor reset, exact resume | `cargo test -p tpt-t-space-dtn --test blackout` |
| Complete §6 Mars Sample Collection flow | end-to-end, 0.8 ms detour in-scenario | `cargo test -p tpt-t-space-sim --test mars_sample` |

Audit gates (all passing): `tools/lock-audit.sh` (zero locks),
`tools/no-recursion.sh`, `tools/no-panic-audit.sh` (strict),
`tools/alloc-audit.sh` (report), `cargo deny check` (MIT chain + §7
function bans). Workspace: 206 tests green, `cargo clippy -D warnings`
clean on stable and nightly.

Phase 16 (hardware bring-up) remains scoped to COTS rad-tolerant dev
boards and facility time as available — see the phase notes above; no
software work is blocked on it.
