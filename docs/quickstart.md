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
cargo build --manifest-path crates/tpt-t-space-satellite/Cargo.toml
cargo build --manifest-path crates/tpt-t-space-rover/Cargo.toml

# no_std bare-metal edge crate
cargo build -p tpt-t-space-edge --target aarch64-unknown-none-softfloat
```

## Benchmarks

Every spec target has an in-repo gate:

```sh
# <50 µs TMR vote-and-reset-trigger (asserted in the budget test + bench)
TPT_RAD_STRICT=1 cargo bench --manifest-path crates/tpt-t-space-radiation/Cargo.toml

# <200 µs ADCS control loop (measured in µs on x86)
TPT_ADCS_STRICT=1 cargo bench --manifest-path crates/tpt-t-space-satellite/Cargo.toml

# <50 ms rover replan (40 ms budget for a 15 m detour)
TPT_PRED_STRICT=1 cargo bench --manifest-path crates/tpt-t-space-predictive/Cargo.toml

# Multi-hour DTN blackout survival + Mars Sample Collection data flow
cargo test -p tpt-t-space-sim --test mars_sample -- --nocapture
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
