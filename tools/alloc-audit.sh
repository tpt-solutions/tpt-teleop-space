#!/usr/bin/env bash
# Allocation gate (spec §2 / Phase 21).
#
# The zero-allocation requirement is a hot-path property: steady-state
# sensor→control loops must not touch the heap. Mission setup, bundle
# upload, and snapshot flushes allocate once by design. This gate scans the
# hot-path library source (test modules stripped) for heap-building
# primitives and fails on any hit that is not listed in
# tools/alloc-allowlist.txt. Each allow-list entry carries its reason, and an
# entry that matches nothing fails too, so the list cannot rot.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ALLOW_FILE="$ROOT/tools/alloc-allowlist.txt"
ALLOC_PATTERNS='Vec::new|vec!|Box::new|format!|to_vec\(|\.collect\(\)|String::from|to_string\(\)'
HOT_CRATES='tpt-t-space-radiation|tpt-t-space-predictive|tpt-t-space-dtn|tpt-t-space-core|tpt-t-space-satellite|tpt-t-space-rover|tpt-t-space-ccsds|tpt-t-space-thermal|tpt-t-space-power|tpt-t-space-orbital|tpt-t-space-edge|tpt-t-space-teleop|tpt-t-domain-bridge'

# Print the file with `#[cfg(test)] mod tests { … }` blanked out, keeping
# line numbers. Brace depth is tracked from the `mod tests` line.
strip_tests() {
  awk '
    /mod tests/ { intest = 1; depth = 0 }
    intest {
      n = gsub(/\{/, "{")
      m = gsub(/\}/, "}")
      depth += n - m
      if (depth <= 0) intest = 0
      print ""
      next
    }
    { print }
  ' "$1"
}

# Load the allow-list: crate|path|needle|reason, one per line.
a_crate=(); a_path=(); a_needle=(); a_reason=()
while IFS= read -r line; do
  line="${line%$'\r'}"  # tolerate CRLF checkouts
  case "$line" in ''|\#*) continue ;; esac
  IFS='|' read -r c p needle reason <<< "$line"
  a_crate+=("$c"); a_path+=("$p"); a_needle+=("$needle"); a_reason+=("$reason")
done < "$ALLOW_FILE"
used=()

fail=0
checked=0
allowed=0
cd "$ROOT"
for crate_dir in crates/*/src; do
  crate=$(basename "$(dirname "$crate_dir")")
  printf '%s' "$crate" | grep -Eq "^($HOT_CRATES)$" || continue
  while IFS= read -r f; do
    while IFS= read -r hit; do
      [ -z "$hit" ] && continue
      n="${hit%%:*}"
      text="${hit#*:}"
      checked=$((checked + 1))
      matched=0
      for i in "${!a_crate[@]}"; do
        if [ "${a_crate[$i]}" = "$crate" ] && [ "${a_path[$i]}" = "$f" ] \
           && [[ "$text" == *"${a_needle[$i]}"* ]]; then
          used[i]=1
          matched=1
          break
        fi
      done
      if [ "$matched" -eq 1 ]; then
        allowed=$((allowed + 1))
      else
        echo "UNLISTED: $f:$n:${text}"
        fail=1
      fi
    done < <(strip_tests "$f" | grep -nE "$ALLOC_PATTERNS" | grep -Ev '^[0-9]+:[[:space:]]*//' || true)
  done < <(find "$crate_dir" -name '*.rs' | sort)
done

for i in "${!a_crate[@]}"; do
  if [ -z "${used[i]:-}" ]; then
    echo "STALE: allow-list entry matches nothing: ${a_path[$i]} '${a_needle[$i]}'"
    fail=1
  fi
done

echo "allocation sites checked: $checked (allowed by reason: $allowed)"
if [ "$fail" -eq 0 ]; then
  echo "PASS: no unlisted heap allocation in hot-path library source."
  exit 0
else
  echo "FAIL: unlisted or stale allocation entries (see above)."
  exit 1
fi
