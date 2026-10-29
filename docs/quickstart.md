# Quickstart — tpt-teleop-space

> Build, test, and gate the workspace. Every spec budget has an in-repo
> enforcement switch.

## Prerequisites

- Rust stable (toolchain pinned via `rust-toolchain.toml`).
- Nightly is **only** required for the `tpt-t-space-satellite` and
  `tpt-t-space-rover` crates (`#![feature(portable_simd)]`); those crates are
  excluded from the stable workspace and built by dedicated CI jobs.

## Build & test the workspace

```sh
# Whole workspace (stable)
cargo build --workspace --all-targets
cargo test  --workspace

# Lint / format
cargo fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings

# Dependency license + function-ban audit (MIT chain)
cargo deny check

# Nightly SIMD crates (in isolation)
(cd crates/tpt-t-space-satellite && cargo build)
(cd crates/tpt-t-space-rover && cargo build)

# no_std bare-metal edge crate
cargo build -p tpt-t-space-edge --target aarch64-unknown-none-softfloat
```

## Benchmarks

Every spec target has an in-repo gate:

```sh
# <50 µs TMR vote-and-reset-trigger (asserted in the budget test + bench)
TPT_RAD_STRICT=1 cargo bench --manifest-path crates/tpt-t-space-radiation/Cargo.toml

# <200 µs ADCS control loop (measured in µs on x86)
(cd crates/tpt-t-space-satellite && TPT_ADCS_STRICT=1 cargo bench)

# <50 ms rover replan (40 ms budget for a 15 m detour)
TPT_PRED_STRICT=1 cargo bench --manifest-path crates/tpt-t-space-predictive/Cargo.toml

# Multi-hour DTN blackout survival + Mars Sample Collection data flow
cargo test -p tpt-t-space-sim --test mars_sample -- --nocapture
```

## The `tpts` CLI

`tpt-t-space-cli` builds one binary, `tpts`, for the chores around the
workspace. It works the same on Windows, macOS and Linux:

```sh
cargo run -q -p tpt-t-space-cli -- doctor              # toolchain check, with the fix for each gap
cargo run -q -p tpt-t-space-cli -- audit --strict      # all audits + cargo-deny in one command
cargo run -q -p tpt-t-space-cli -- budgets             # strict benches as a pass/fail table
cargo run -q -p tpt-t-space-cli -- ccsds "01 23 C0 05 00 03 DE AD BE EF"   # one TM packet
cargo run -q -p tpt-t-space-cli -- ccsds stream.bin --crc                   # file input, TC frames with CRC-16
cargo run -q -p tpt-t-space-cli -- orbit --orbits 2 --out orbit.replay
cargo run -q -p tpt-t-space-cli -- scenario mars --out mars.replay
cargo run -q -p tpt-t-space-cli -- new rover           # scaffold a mission crate with a smoke test
cargo run -q -p tpt-t-space-cli -- profile             # mission-profile presets (LEO, Mars, GEO)
cargo run -q -p tpt-t-space-cli -- seq 'drive 1; turn 90; drive 3'   # lint + dry-run an uplink over the Mars terrain
cargo run -q -p tpt-t-space-cli -- campaign seu        # SEU-rate sweep against the TMR voter
cargo run -q -p tpt-t-space-cli -- campaign blackout   # blackout-length sweep against the DTN store
cargo run -q -p tpt-t-space-cli -- blackbox <hex|file> # decode a Safe Mode black-box image
```

On Windows the audit scripts run under Git Bash. `tpts` looks for it at
`Program Files\Git\bin\bash.exe`; set `TPT_BASH` to use another copy. A bare
`bash` is not used there because `System32\bash.exe` is the WSL launcher.

The `just` recipes wrap the same commands (`just check`, `just budgets`,
`just visualize`); see the `justfile` at the repository root.

## Worked examples

Each of these is a short, runnable walk through one part of the system:

```sh
cargo run -p tpt-t-space-dtn --example blackout_resume      # bundle resumes across a link drop
cargo run -p tpt-t-space-radiation --example tmr_vote       # TMR masks a bit flip
cargo run -p tpt-t-space-predictive --release --example replan  # detour around a 25° slope
cargo run -p tpt-t-space-ccsds --example parse_tm           # zero-copy TM packet parse
```

## Crate layout

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full map and each crate's
spec section + performance budget.

## Cross-repo bridge

`Phase 1` is `tpt-t-domain-bridge`. Its canonical home is the `tpt-teleop`
repo (every domain repo depends on it); during co-development it is vendored
in this workspace at `crates/tpt-t-domain-bridge` so domain crates use a
plain path dependency:

```toml
[dependencies]
tpt-t-domain-bridge = { workspace = true }
```

When `tpt-teleop` publishes the bridge, switch the dependency over — the
public API (DTI trait, wire types, safety state machine, assist API) is the
stable contract.
