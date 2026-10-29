# Inspection record: TMR voter

- **Component:** `tpt-t-space-radiation`, `src/tmr.rs` (`TmrVoter::submit`, `take_reset_requests`)
- **Requirements:** TMR-001, TMR-002, TMR-003
- **Commit inspected:**
- **Inspector / date:**
- **Outcome:** [ ] accepted  [ ] accepted with actions  [ ] rejected

## Checklist

| # | Check | Requirement | Result (pass / fail / n/a) | Notes |
|---|-------|-------------|----------------------------|-------|
| 1 | A single diverging core is outvoted and its value is not returned | TMR-001 | | |
| 2 | The masked core index is reported and its reset bit is set once per vote | TMR-001 | | |
| 3 | With all three cores diverging, `trusted` is false and no value is trusted | TMR-002 | | |
| 4 | Reset requests are cleared by `take_reset_requests` and not replayed | TMR-001 | | |
| 5 | The divergence limit gates reset flagging, as documented | TMR-003 | | |
| 6 | The vote path has no heap allocation, lock, or panic site (see `tools/alloc-audit.sh`, `lock-audit.sh`, `no-panic-audit.sh`) | TMR-001 | | |
| 7 | The vote time is within the 50 µs budget (`TPT_RAD_STRICT=1 cargo bench -p tpt-t-space-radiation`) | TMR-001 | | |

## Findings

-
