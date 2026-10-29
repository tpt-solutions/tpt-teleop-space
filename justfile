# tpt-teleop-space task runner — https://just.systems
#
# One-command wrappers for the build, test, audit, and bench gates that
# docs/quickstart.md describes. Recipes run under bash (Git Bash on Windows).

set shell := ["bash", "-euo", "pipefail", "-c"]

# List the recipes.
default:
    @just --list

# Build the whole stable workspace, every target.
build:
    cargo build --workspace --all-targets

# Run the workspace test suite.
test:
    cargo test --workspace

# Format check and clippy with warnings denied (what CI runs).
lint:
    cargo fmt --all -- --check
    cargo clippy --workspace --all-targets -- -D warnings

# Lock, recursion, panic, and allocation audits plus cargo-deny.
# Pass --strict for the no-panic gate with TPT_NO_PANIC_STRICT=1.
audit *flags:
    cargo run -q -p tpt-t-space-cli -- audit {{flags}}

# The three strict benches against the spec budgets.
budgets:
    cargo run -q -p tpt-t-space-cli -- budgets

# Toolchain and tool check with the fix for each gap.
doctor:
    cargo run -q -p tpt-t-space-cli -- doctor

# Run the four worked examples.
examples:
    cargo run -q -p tpt-t-space-dtn --example blackout_resume
    cargo run -q -p tpt-t-space-radiation --example tmr_vote
    cargo run -q -p tpt-t-space-predictive --release --example replan
    cargo run -q -p tpt-t-space-ccsds --example parse_tm

# Write the spec §6 Mars replay and open the visualizer on it.
visualize:
    cargo run -q -p tpt-t-space-cli -- scenario mars --out mars.replay
    cargo run --manifest-path crates/tpt-t-space-visualizer/Cargo.toml -- mars.replay

# Requirement ↔ test traceability (docs/requirements.csv).
req:
    bash tools/req-trace.sh

# Line coverage for the safety-critical crates, gated at 90%.
coverage:
    cargo llvm-cov -p tpt-t-space-radiation -p tpt-t-space-core -p tpt-t-space-dtn -p tpt-t-space-thermal -p tpt-t-space-power --fail-under-lines 90 --summary-only

# Everything a pull request needs: lint, test, and the audits.
check: lint test audit req
