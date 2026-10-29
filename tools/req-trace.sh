#!/usr/bin/env bash
# Requirement ↔ test traceability gate (spec §2 / Phase 21).
#
# docs/requirements.csv is the machine-readable requirement list. A test that
# verifies a requirement carries a `// REQ: <id>` comment on the line above
# its #[test] attribute. The gate checks both directions:
#   * every requirement has at least one verifying test (no unverified rows),
#   * every marker names a known requirement (no stale or misspelled links).
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
CSV="docs/requirements.csv"

# Requirement ids: first column after the header, CR-tolerant.
ids=$(tail -n +2 "$CSV" | tr -d '\r' | cut -d, -f1 | grep -v '^$' || true)
fail=0

dups=$(printf '%s\n' "$ids" | sort | uniq -d)
if [ -n "$dups" ]; then
    echo "DUPLICATE requirement ids: $dups"
    fail=1
fi

# Markers in the code: file:line:id.
markers=$(grep -rnoE --include='*.rs' '// REQ: [A-Z]+-[0-9]+' crates | sed -E 's/^([^:]+):([0-9]+):\/\/ REQ: (.*)$/\1:\2:\3/' || true)

while IFS= read -r m; do
    [ -z "$m" ] && continue
    id="${m##*:}"
    if ! printf '%s\n' "$ids" | grep -qxF "$id"; then
        echo "UNKNOWN: ${m%:*} names '$id', which is not in $CSV"
        fail=1
    fi
done <<< "$markers"

verified=0
total=0
while IFS= read -r id; do
    [ -z "$id" ] && continue
    total=$((total + 1))
    if printf '%s\n' "$markers" | grep -qE ":${id}\$"; then
        verified=$((verified + 1))
    else
        echo "UNVERIFIED: $id has no test marked '// REQ: $id'"
        fail=1
    fi
done <<< "$ids"

echo "requirements verified: $verified / $total"
if [ "$fail" -eq 0 ]; then
    echo "PASS: every requirement has a verifying test and every marker is known."
    exit 0
fi
echo "FAIL: traceability gaps (see above)."
exit 1
