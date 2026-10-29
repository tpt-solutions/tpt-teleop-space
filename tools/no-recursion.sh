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

# Scan from the repo root with relative paths. Absolute Windows paths
# (`C:/...`) contain a colon that breaks the `file:line:text` split below,
# which silently skipped every function on Windows hosts.
cd "$ROOT"

for crate_dir in crates/*/src; do
    crate=$(basename "$(dirname "$crate_dir")")
    while IFS= read -r hit; do
        [ -z "$hit" ] && continue
        f="${hit%%:*}"
        line="${hit#*:}"
        line="${line%%:*}"
        # fn name from the definition line
        fname="$(sed -n "${line}s/^.*fn \([a-zA-Z0-9_]*\).*/\1/p" "$f")"
        [ -z "$fname" ] && continue
        # The brace-balanced body (indented methods included): from the
        # first `{` after the signature to the matching `}`. A `;` before
        # any `{` means a bodiless trait declaration.
        body="$(awk -v start="$line" '
            NR < start { next }
            {
                sub(/\/\/.*/, "")  # comments can mention a call without making one
                n = gsub(/\{/, "{"); m = gsub(/\}/, "}")
                if (!opened) {
                    if (n == 0 && $0 ~ /;/) exit
                    if (n == 0) next
                    opened = 1
                }
                depth += n - m
                if (NR > start) print
                if (depth <= 0) exit
            }' "$f")"
        # A self-call is a bare `name(`, `Self::name(`, or `self.name(`.
        # Calls through another type (`Other::name(`) or receiver
        # (`x.name(`) are different functions and are not counted.
        if printf '%s' "$body" | grep -Eq "(^|[^A-Za-z0-9_:.])${fname}[[:space:]]*\(|(Self::|self\.)${fname}[[:space:]]*\("; then
            echo "FAIL: $crate/$f: fn '$fname' appears to recurse"
            fail=1
        fi
    done < <(grep -rn "fn " "$crate_dir" --include='*.rs' | grep -v "^\s*//")
done

# Mutual recursion (a → b → a) and longer loops need a call graph; see
# call-graph.py. Run both checks so the gate covers every cycle length.
# Probe each name: on Windows `python3` can exist as a Store alias that
# prints a message instead of running.
PY=""
for candidate in python3 python; do
    if "$candidate" -c "import sys" >/dev/null 2>&1; then
        PY="$candidate"
        break
    fi
done
if [ -z "$PY" ]; then
    echo "FAIL: python is required for the call-graph check (mutual recursion)."
    exit 1
fi
if ! "$PY" "$ROOT/tools/call-graph.py"; then
    fail=1
fi

if [ "$fail" -eq 0 ]; then
    echo "PASS: no direct or mutual recursion in library source."
    exit 0
else
    echo "Recursion is banned in flight software (static stack budgets)."
    exit 1
fi
