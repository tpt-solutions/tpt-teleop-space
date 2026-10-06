# Compliance Readiness — ECSS-E-ST-40C & NASA-STD-8739.8 (Phase 15)

> Status: **architected for certification, not yet certified.** This
> document maps the standard's software-engineering requirements onto the
> concrete mechanisms in the workspace and names what a formal campaign
> still needs.

## The four spaceflight software prohibitions (spec §2)

| Requirement | Mechanism | Enforcement |
|-------------|-----------|-------------|
| No dynamic memory allocation after initialization | Fixed-capacity structures everywhere: SPSC rings, NVRAM sector pools, fixed task tables, exact-size DEM/MPC buffers | `tools/alloc-audit.sh` (informational report — reviewers confirm each hit is construction-time); ownership rules documented per crate |
| No recursion | All traversals are iterative (scrub pass, planner arc walk, bundle store scan) | `tools/no-recursion.sh` (CI hard gate); mutual recursion excluded by code review note in the script |
| No panics in the hot path | `try_*` APIs with typed errors; bounded fallbacks (diagonal Kalman gain, DLS damping); spin-wait backpressure instead of abort | `tools/no-panic-audit.sh` — **strict mode (`TPT_NO_PANIC_STRICT=1`) passes** over the hot-path crates |
| Complete traceability of state transitions | Every FSM has a closed transition table with typed illegal-transition errors; SEU book, custody states, and snapshot CRCs are persistent | `docs/fmea-safe-mode.md` § traceability; unit tests enumerate the legal graphs |

## ECSS-E-ST-40C mapping (selected)

| Area | ECSS expectation | Workspace status |
|------|------------------|------------------|
| Software architecture (§5) | Layered, modular, justified deps | 15 crates, one responsibility each; `docs/ARCHITECTURE.md`; `deny.toml` bans by function (spec §7) |
| Design prohibitions (§6) | No dynamic memory/recursion/panics per criticality | See table above — all four mechanically enforced |
| Verification (§7) | Unit + integration + validation traceability | 206 workspace tests + nightly-crate suites + the §6 Mars Sample Collection end-to-end test; every spec budget has a gate (`docs/quickstart.md`) |
| FMEA (§8) | Failure modes on critical paths | `docs/fmea-safe-mode.md` |
| Traceability (§10) | Bidirectional requirement ↔ test | `todo.md` is the requirement/phase ledger; each phase's commit carries its tests — a formal tool (e.g.reqif) is future work |
| Toolchain (§11) | Pinned, reproducible | `rust-toolchain.toml` (stable + crate-local nightly), `Cargo.lock` committed |

## NASA-STD-8739.8 mapping (selected)

| Area | Expectation | Workspace status |
|------|-------------|------------------|
| Software safety | Hazard analysis drives design | Safe Mode FMEA; hazard-driven design notes in crate docs (plume veto, keep-out, e-stop) |
| Coding standards | Enforced style + safety lints | `cargo fmt --check`, `cargo clippy -D warnings` (stable + nightly crates), workspace lints table |
| Inspections | Reviewed critical components | Review coverage is repo history + PR process (formal inspection records: future work) |
| Testing depth | Statement/branch coverage on critical paths | Coverage *measurement* not yet wired (add `cargo-llvm-cov` in CI — future work) |

## What a formal certification campaign still needs

1. **Independent verification & validation** — the audits here are
   self-imposed; ECSS expects IV&V by a separate team.
2. **Requirements tooling** — machine-readable requirements with
   bidirectional test links (this repo tracks phases, not requirements).
3. **Coverage measurement** — `cargo-llvm-cov` in CI with thresholds on
   the safety-critical crates.
4. **Formal inspections** — recorded walkthroughs of the TMR voter,
   scrubber, Safe Mode controller, and DTN custody logic.
5. **Target qualification** — Phase 16 hardware campaigns (see `todo.md`).
