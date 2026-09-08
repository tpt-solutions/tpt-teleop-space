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
- [ ] `git init`, initial commit of spec docs
- [ ] Cargo workspace `Cargo.toml` (resolver "3", edition 2024, rust-version
      1.85, license `MIT OR Apache-2.0`, authors "TPT Solutions"), mirroring
      sister repos' workspace layout (`crates/*` members, shared workspace
      deps table)
- [ ] `LICENSE-MIT` and `LICENSE-APACHE` at repo root (TPT Solutions, 2026)
- [ ] `.gitignore` (target/, etc.)
- [ ] `deny.toml` — MIT/BSD/ISC/Zlib/MPL-2.0 allow-chain, explicit
      Apache-2.0-only deny, `[[licenses.clarify]]` overrides for
      dual-licensed deps (rkyv, windows-sys, proc-macro2/syn/quote, libc),
      encodes the §7 dependency matrix (bans nalgebra/ndarray, heavy CCSDS
      libraries, tokio, serde/bincode, in favor of the custom lightweight
      choices)
- [ ] `.github/workflows/ci.yml` — build-test matrix (linux/macos/windows),
      cross-compile check, `RUSTFLAGS: -D warnings`, plus a separate nightly
      job scoped to `tpt-t-space-satellite` and `tpt-t-space-rover`
- [ ] `.github/workflows/lint.yml` — fmt + clippy (`-D warnings`, stable +
      nightly job for satellite/rover) + cargo-deny action
- [ ] `rust-toolchain.toml` — stable default; document the nightly override
      needed for `tpt-t-space-satellite` (MEKF) and `tpt-t-space-rover`
      (path planner)
- [ ] Root `README.md` (Status / Workspace Crates / Zero-Allocation Data
      Path / Developer Experience / Licensing — mirror sister repos'
      section structure)
- [ ] `CHANGELOG.md`, `SECURITY.md`
- [ ] `docs/ARCHITECTURE.md` skeleton (workspace map, crate responsibilities)
- [ ] `docs/quickstart.md` stub
- [ ] Stub empty crate skeletons (Cargo.toml + lib.rs) for all crates below

## Phase 1 — Cross-Repo Bridge: `tpt-t-domain-bridge` (lives in `tpt-teleop` repo)
- [ ] New crate `tpt-t-domain-bridge` under `tpt-teleop/crates/`
- [ ] Define `DomainTeleopInterface` (DTI) trait: `on_teleop_engage`,
      `on_teleop_disengage`, `on_control_command`, `get_domain_state`,
      `get_sensor_feed`
- [ ] Define wire types with `rkyv::Archive`: `ControlCommand`, `InputState`,
      `SensorFeed`, `DomainState`, `DomainTelemetry`, `HapticCmd`/`HapticFeedback`
- [ ] Reconcile with existing 56-byte POD `ControlCommand` in
      `tpt-teleop-core/src/ser/cmd.rs` (bridge spec flags this as an
      intentional, unreconciled discrepancy — resolve or document the mapping)
- [ ] Universal safety state machine: `AUTONOMOUS → REQUESTING_TELEOP →
      TELEOP_ACTIVE → RETURNING_TO_AUTONOMY → AUTONOMOUS`, plus
      `EMERGENCY_STOP` transitions
- [ ] `request_teleop_assistance()` API surface
- [ ] Unit tests + doc examples; publish path so `tpt-t-space-teleop` can
      depend on it (path dep during co-development)

## Phase 2 — `tpt-t-space-core`
- [ ] Central deterministic event loop
- [ ] Mission state machine (Cruise → Approach → Operations → Safe Mode)
- [ ] Lock-free message bus for inter-crate event routing
- [ ] Persistent state snapshotting so the loop survives processor resets
      without losing mission context
- [ ] rkyv wire-type prelude shared by other space crates
- [ ] Zero-lock hot-path audit passes (reuse `tools/lock-audit.sh` pattern
      from sister repos)

## Phase 3 — `tpt-t-space-radiation`
- [ ] Continuous background memory scrubber: CRC32 over static memory,
      single-bit correction via Hamming codes before multi-bit accumulation
- [ ] TMR (Triple Modular Redundancy) voter: critical calculations executed
      on three separate cores, majority-vote comparison
- [ ] On divergence: mask the bad result, trigger a core reset — all in
      <50µs, zero heap allocation
- [ ] SEU (Single Event Upset) detection and recovery bookkeeping
- [ ] Benchmark: <50µs TMR vote-and-reset-trigger cycle, zero-alloc proof

## Phase 4 — `tpt-t-space-ccsds`
- [ ] Zero-allocation, zero-copy CCSDS telemetry (TM) packet parser
- [ ] Zero-allocation, zero-copy CCSDS telecommand (TC) packet parser
- [ ] Space Packet Protocol header parsing (APID, sequence count, etc.)
- [ ] Fuzz/property tests for the zero-copy parser against malformed frames

## Phase 5 — `tpt-t-space-dtn`
- [ ] Custom zero-copy Bundle Protocol (BPv7) implementation: bundles with
      custody transfer
- [ ] Licklider Transmission Protocol (LTP) implementation
- [ ] Store-and-forward: on link drop, persist the in-flight bundle to
      NVRAM; on link restore, resume transmission exactly where it left off
- [ ] Zero-allocation serialization: bundles written directly into
      pre-allocated NVRAM sectors via `rkyv`
- [ ] Multi-hour blackout tolerance validation (planetary occultation
      scenario)
- [ ] Hash-verified telemetry log transmission on reconnect

## Phase 6 — Environment & Survival
### `tpt-t-space-thermal`
- [ ] Heater state monitoring
- [ ] Radiator deployment monitoring
- [ ] Battery temperature monitoring
- [ ] Safe Mode trigger on thermal limit breach (e.g. battery temp < -20°C)

### `tpt-t-space-power`
- [ ] Solar array pointing optimization
- [ ] Battery charge/discharge cycle tracking
- [ ] Load shedding during eclipse periods
- [ ] Safe Mode trigger on solar array output dropping below survival
      threshold (dust-storm scenario, spec.txt §6)

### Safe Mode transition (shared `core`/`thermal`/`power`)
- [ ] Single atomic, zero-allocation Safe Mode transition: power down
      non-essential payloads, slew to sun-pointing attitude, begin low-rate
      DTN beacon (consumes Phase 5 `dtn`)

## Phase 7 — `tpt-t-space-predictive`
- [ ] Onboard model-predictive control (MPC) framework
- [ ] Local Digital Elevation Model (DEM) generation from stereo camera
      data at 10Hz
- [ ] Custom, SIMD-accelerated path planner evaluating thousands of
      candidate trajectories, selecting the safest path within tilt/traction
      limits
- [ ] Autonomous decision-making framework reusable across rover hazard
      avoidance and orbital docking approach (consumed by Phase 9, Phase 10)
- [ ] Benchmark: <50ms full replan cycle (spec.txt §6 example: 40ms for a
      15m detour)

## Phase 8 — `tpt-t-space-satellite` (ADCS, nightly-scoped)
- [ ] Crate-local nightly toolchain override + `#![feature(portable_simd)]`,
      CI job scoped to this crate only (rest of workspace stays stable)
- [ ] Reaction wheel control
- [ ] Star tracker + sun sensor + gyro fusion via custom SIMD-accelerated
      Multiplicative Extended Kalman Filter (MEKF) for quaternion estimation
- [ ] Momentum dumping (magnetorquers or thrusters)
- [ ] Orbit maintenance maneuvers
- [ ] Control loop pinned to a dedicated real-time core, executing in
      <200µs
- [ ] No-panic guarantee: bounded fallback algorithms (e.g. pseudo-inverse
      with condition-number checking) for matrix inversion under corrupted
      sensor data
- [ ] TMR-voted quaternion updates (consumes Phase 3 `radiation`)
- [ ] Sub-arcsecond precision pointing mode
- [ ] Formation flying: multi-satellite relative orbit maintenance via
      inter-satellite links
- [ ] Benchmark: <200µs ADCS control loop cycle

## Phase 9 — `tpt-t-space-rover`
- [ ] Wheel odometry
- [ ] Visual-inertial odometry (VIO)
- [ ] Autonomous hazard avoidance consuming Phase 7 `predictive` DEM/path
      planner output
- [ ] Science sequencing: multi-sol instrument deployment sequences (drill,
      sieve, spectrometer) with automatic rollback on failure
- [ ] Sample collection sequence: deploy arm, press drill, extract core,
      seal tube, stow in carousel — each step verified by torque/position
      sensors
- [ ] Wheel health monitoring: motor current + visual wheel-wear tracking
      for impending-failure detection (Curiosity-wheel-damage-style)
- [ ] End-to-end test: 25° slope detected → 15m detour replanned in <50ms
      (spec.txt §6)

## Phase 10 — `tpt-t-space-orbital`
- [ ] 6DOF relative navigation fusing LiDAR, visual markers, and GPS
      (LEO-only) for cm-level relative state estimation
- [ ] Autonomous docking/berthing via MPC for final approach (consumes
      Phase 7 `predictive`), managing plume impingement and collision
      avoidance
- [ ] 7-DOF space robotic arm (Canadarm-style) manipulation
- [ ] Singularity avoidance
- [ ] Payload mass estimation

## Phase 11 — `tpt-t-space-edge` (`no_std`, bare-metal)
- [ ] Bare-metal RTOS execution layer for space-grade microcontrollers
      (RAD750, LEON3, rad-hard ARM Cortex-R)
- [ ] Direct register-level peripheral control
- [ ] Hardware watchdog management + watchdog-feeding logic
- [ ] `no_std` build validated for the target architectures above

## Phase 12 — `tpt-t-space-teleop` (adapter crate)
- [ ] Depend on `tpt-t-domain-bridge` (Phase 1) and implement
      `DomainTeleopInterface`
- [ ] Predictive display: show the operator a predicted rover position
      based on last-known state, masking the light-speed delay (e.g. 12
      minutes for Mars)
- [ ] Command sequencing: accept an uploaded sequence (e.g. "drive forward
      5m, turn left 30°, take a photo"), execute autonomously with hazard
      avoidance (consumes Phase 9 `rover`, Phase 7 `predictive`)
- [ ] Bandwidth optimization: compress images/telemetry for the deep-space
      link
- [ ] Transport via `tpt-t-space-dtn` (Phase 5) — custody transfer for
      uplinked command sequences, downlinked telemetry/images
- [ ] Reconcile aspirational `ControlCommand` (operator_id/InputState/
      ButtonState) against upstream's actual flat POD type (open decision,
      tracked upstream — same open item as sister repos)

## Phase 13 — `tpt-t-space-sim` + Visualizer
- [ ] Headless simulator: 6DOF orbital mechanics (J2 perturbations,
      atmospheric drag, third-body gravity)
- [ ] Spacecraft mass properties and reaction wheel/CMG dynamics
- [ ] Sensor noise models (star tracker blinding, gyro bias drift, IMU noise)
- [ ] Radiation environment model: inject random bit flips into memory to
      exercise TMR voting and memory scrubbing (Phase 3)
- [ ] DTN network blackout and delay-profile modeling (e.g. Mars-Earth
      light time)
- [ ] Variable-speed simulation: multi-year cruise phases or complex
      rendezvous maneuvers simulated in minutes of wall-clock time
- [ ] `egui` (MIT) visualizer binary: 3D orbital trajectories and ground
      tracks
- [ ] Visualizer: spacecraft attitude (quaternion) and sun-vector alignment
- [ ] Visualizer: rover terrain maps, planned vs. actual paths
- [ ] Visualizer: power/thermal budgets over time

## Phase 14 — System Integration & End-to-End
- [ ] Full "Mars Sample Collection" data flow test (spec.txt §6): DTN
      uplink + custody transfer → comms blackout → autonomous traverse with
      predictive-DEM hazard detection → 15m detour replanned in 40ms → SEU
      injected mid-calculation → TMR voter masks corrupted result + flags
      core for watchdog reset → arm deploy/drill/extract/seal/stow science
      sequence verified by torque/position sensors → dust storm drops solar
      output → power-triggered Safe Mode (payload off, rover halted) →
      storm clears, orbiter overhead → DTN resumes, hash-verified telemetry
      log + Safe Mode alert downlinked
- [ ] Dependency-matrix conformance audit against §7 (confirm no banned
      dependency — nalgebra/ndarray, heavy CCSDS libraries, tokio, serde/
      bincode — entered the tree)
- [ ] Cross-workspace zero-allocation/zero-lock verification (extend
      `tpt-t-integration`-style audit from `tpt-teleop` to cover all space
      crates)
- [ ] Full `cargo-deny` MIT-chain audit pass across the workspace

## Phase 15 — Compliance & Certification Readiness
- [ ] ECSS-E-ST-40C (Software Engineering) compliance review
- [ ] NASA-STD-8739.8 compliance review
- [ ] No-dynamic-allocation-after-initialization audit across the workspace
- [ ] No-recursion audit (static call-graph analysis)
- [ ] No-panics-in-hot-path audit (clippy lints + panic-free proof tooling,
      e.g. `no_panic` or manual audit of all hot-path crates)
- [ ] Complete traceability of all state transitions: audit-log coverage
      review across `core`, `radiation`, `thermal`/`power` Safe Mode paths
- [ ] FMEA (Failure Modes and Effects Analysis) pass on the Safe Mode
      transition path

## Phase 16 — Hardware Bring-Up & Flight/Radiation Validation
> Real flight-qualified hardware (RAD750/LEON3/rad-hard Cortex-R engineering
> units, radiation beam time) is not casually accessible — scope this phase
> to COTS rad-tolerant dev boards and facility access as it becomes
> available, rather than assuming real flight hardware.
- [ ] COTS rad-tolerant dev board bring-up (e.g. LEON3 GR712RC eval board or
      a rad-hard-adjacent ARM Cortex-R eval kit) replacing sim-only targets
- [ ] Real hardware watchdog wiring + feed-loop validation on target board
- [ ] Radiation test campaign (heavy-ion or proton beam, third-party
      facility) validating the TMR voter and memory scrubber against real
      SEU rates — flagged as facility/funding-dependent, pursue when
      available
- [ ] Hardware-in-the-loop validation for ADCS timing against a real star
      tracker/IMU/sun sensor stack
- [ ] Real DTN link testing over a simulated light-time-delay test rig
- [ ] Target embedded platform performance validation against every spec'd
      latency budget on real silicon
- [ ] Extreme-environment validation (thermal vacuum chamber, launch
      vibration profile) as facility access allows

## Phase 17 — Hardening, Docs & v1.0.0 Release
- [ ] `cargo clippy --workspace --all-targets -- -D warnings` clean
      (stable workspace)
- [ ] Nightly clippy pass clean for `tpt-t-space-satellite` and
      `tpt-t-space-rover`
- [ ] Performance benchmarks validated against every spec target: <50µs TMR
      vote, <200µs ADCS control loop, <50ms rover replan, multi-hour DTN
      blackout survival
- [ ] Fuzz/property testing for custom zero-allocation parsers (CCSDS
      TM/TC, BPv7/LTP framing)
- [ ] Developer docs complete (`docs/quickstart.md`, `docs/ARCHITECTURE.md`,
      crate-level rustdoc)
- [ ] README finalized
- [ ] Tag `v1.0.0` — "v1 Monolithic Release"
