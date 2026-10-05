#!/usr/bin/env bash
# No-recursion audit (spec §2 / Phase 15: ECSS-E-ST-40C stack-overflow
# prevention).
#
# Spaceflight software bans recursion: unbounded stack growth cannot be
# tolerated on flight hardware with static stack budgets. This is a
# conservative static approximation of call-graph analysis: for every `fn`
# in library source, flag definitions whose body contains a call to its own
# name. Mutual recursion (a ↔ b) is not caught by a per-function scan and is
# instead covered by review — this script catches the dominant accidental
# form (direct self-recursion, typically in tree/graph walks).
#
# Hits in tests/benches are informational only (not flight code paths).
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
fail=0

for crate_dir in "$ROOT"/crates/*/src; do
    crate=$(basename "$(dirname "$crate_dir")")
    while IFS= read -r hit; do
        [ -z "$hit" ] && continue
        f="${hit%%:*}"
        line="${hit#*:}"
        # fn name from the definition line
        fname="$(sed -n "${line}s/^.*fn \([a-zA-Z0-9_]*\).*/\1/p" "$f")"
        [ -z "$fname" ] && continue
        # does this fn's brace-balanced body call itself?
        body="$(awk -v start="$line" 'NR>start && /^\}/ {exit} NR>start {print}' "$f")"
        if printf '%s' "$body" | grep -Eq "\b${fname}\b\s*\("; then
            echo "FAIL: $crate/$f: fn '$fname' appears to recurse"
            fail=1
        fi
    done < <(grep -rn "fn " "$crate_dir" --include='*.rs' | grep -v "^\s*//")
done

if [ "$fail" -eq 0 ]; then
    echo "PASS: no direct self-recursion in library source."
    exit 0
else
    echo "Recursion is banned in flight software (static stack budgets)."
    exit 1
fi
