#!/usr/bin/env bash
# No-panics-in-hot-path audit (spec §2 / Phase 15).
#
# Spaceflight software forbids panics in control paths: a panicking thread
# on flight hardware aborts the mission code. This scan reports `unwrap()`,
# `expect()`, `panic!`, `unreachable!`, `todo!`, and `unimplemented!` in
# the library source of the *hot-path crates* (radiation TMR, predictive
# planner, dtn, core bus, ccsds, satellite ADCS, rover). Those crates must
# stay clean in STRICT mode. `#[cfg(test)]` test modules inside src/ are
# stripped before the scan — they are not flight code.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PANIC_PATTERNS='\.unwrap\(|\.expect\(|panic!|unreachable!|todo!|unimplemented!'
HOT_CRATES='tpt-t-space-radiation|tpt-t-space-predictive|tpt-t-space-dtn|tpt-t-space-core|tpt-t-space-satellite|tpt-t-space-rover|tpt-t-space-ccsds'

STRICT="${TPT_NO_PANIC_STRICT:-0}"
fail=0

# Strip `#[cfg(test)] mod tests { ... }` blocks: track brace depth from
# the `mod tests` line (rustfmt keeps the bodies one-indent, braces
# balanced per line).
strip_tests() {
  awk '
    /mod tests/ { intest = 1; depth = 0 }
    intest {
      n = gsub(/\{/, "{")
      m = gsub(/\}/, "}")
      depth += n - m
      if (depth <= 0) intest = 0
      next
    }
    { print }
  ' "$1"
}

echo "== hot-path panic-prone constructs =="
for crate_dir in "$ROOT"/crates/*/src; do
  crate=$(basename "$(dirname "$crate_dir")")
  hits=""
  while IFS= read -r f; do
    stripped=$(strip_tests "$f")
    filtered=$(printf '%s\n' "$stripped" | grep -nE "$PANIC_PATTERNS" 2>/dev/null | grep -v '^\s*//' || true)
    [ -n "$filtered" ] && hits+="$f:$filtered"$'\n'
  done < <(find "$crate_dir" -name '*.rs')
  hits="${hits%$'\n'}"
  if [ -n "$hits" ]; then
    if printf '%s' "$crate" | grep -Eq "^($HOT_CRATES)$"; then
      echo "HOT-PATH HIT: $crate"
      printf '%s\n' "$hits" | sed 's/^/  /'
      fail=1
    else
      echo "INFO: $crate (non-hot-path crate; verify hits are init-time)"
      printf '%s\n' "$hits" | sed 's/^/  /' | head -8
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
