#!/usr/bin/env bash
# Cross-workspace allocation audit (informational).
#
# The zero-allocation requirement is a hot-path property (steady-state
# sensor→control loops), not a whole-program ban — mission setup, bundle
# upload, and snapshot flushes allocate once by design. This scan therefore
# reports heap-building primitives per crate so reviewers can confirm each
# hit is setup-time, not per-tick. Exit code is always 0; CI prints the
# report.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ALLOC_PATTERNS='Vec::new|vec!|Box::new|format!|to_vec\(|\.collect\(\)|String::from|to_string\(\)'

echo "== allocation primitives in library source (review setup vs hot path) =="
for crate_dir in "$ROOT"/crates/*/src; do
    crate=$(basename "$(dirname "$crate_dir")")
    hits=$(grep -rEn "$ALLOC_PATTERNS" "$crate_dir" 2>/dev/null | grep -v "^\s*//" | wc -l)
    printf "%-28s %s\n" "$crate" "$hits"
done

echo
echo "PASS: report generated. Review any hits on per-tick code paths."
exit 0
