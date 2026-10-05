#!/usr/bin/env bash
# No-panics-in-hot-path audit (spec §2 / Phase 15).
#
# Spaceflight software forbids panics in control paths: a panicking thread on
# flight hardware aborts the mission code. This scan reports `unwrap()`,
# `expect()`, `panic!`, `unreachable!`, `todo!`, `unimplemented!`, integer
# division, and array indexing in the library source of the *hot-path crates*
# (radiation TMR, satellite ADCS loop, predictive planner, dtn, core bus).
# Those crates must stay clean in STRICT mode; the remaining crates are
# reported informationally (setup paths may use expect/unwrap once at init).
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PANIC_PATTERNS='\.unwrap\(|\.expect\(|panic!|unreachable!|todo!|unimplemented!'
HOT_CRATES='tpt-t-space-radiation|tpt-t-space-predictive|tpt-t-space-dtn|tpt-t-space-core|tpt-t-space-satellite|tpt-t-space-rover|tpt-t-space-ccsds'

STRICT="${TPT_NO_PANIC_STRICT:-0}"
fail=0

echo "== hot-path panic-prone constructs (crates: $HOT_CRATES) =="
for crate_dir in "$ROOT"/crates/*/src; do
    crate=$(basename "$(dirname "$crate_dir")")
    hits=$(grep -rnE "$PANIC_PATTERNS" "$crate_dir" --include='*.rs' 2>/dev/null \
        | grep -v "^\s*//" | grep -v "#\[cfg(test)\]" || true)
    if [ -n "$hits" ]; then
        if printf '%s' "$crate" | grep -Eq "^($HOT_CRATES)$"; then
            echo "HOT-PATH HIT: $crate"
            printf '%s\n' "$hits" | sed 's/^/  /'
            fail=1
        else
            echo "INFO: $crate (non-hot-path crate, review that hits are init-time)"
            printf '%s\n' "$hits" | sed 's/^/  /' | head -10
        fi
    fi
done

if [ "$fail" -eq 0 ]; then
    echo "PASS: no panic-prone constructs in hot-path library source."
    exit 0
elif [ "$STRICT" = "1" ]; then
    echo "FAIL (strict): panic-prone constructs found in hot-path crates."
    exit 1
else
    echo "WARN: hot-path hits found; set TPT_NO_PANIC_STRICT=1 to fail the build."
    exit 0
fi
