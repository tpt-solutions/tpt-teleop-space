# Inspection record: Safe Mode controller

- **Component:** `tpt-t-space-core`, `src/safe_mode.rs` (`SafeModeController::enter`, `recover`) and `src/machine.rs` (`MissionState::next`)
- **Requirements:** SM-001, SM-002, FSM-001
- **Commit inspected:**
- **Inspector / date:**
- **Outcome:** [ ] accepted  [ ] accepted with actions  [ ] rejected

## Checklist

| # | Check | Requirement | Result (pass / fail / n/a) | Notes |
|---|-------|-------------|----------------------------|-------|
| 1 | Entry runs shed-payloads, slew-to-sun, and beacon start, in that order, before returning | SM-001 | | |
| 2 | An entry that fails part-way is not reported as entered | SM-001 | | |
| 3 | Re-entering while already in Safe Mode runs no action a second time | SM-002 | | |
| 4 | Recovery is only possible from Safe Mode, and a later entry is a fresh episode | SM-002 | | |
| 5 | Every transition not in the legal graph is rejected, and the rejection names both states | FSM-001 | | |
| 6 | The legal graph in `machine.rs` matches the spec's Safe Mode rules, including Fault and Teleop edges | FSM-001 | | |
| 7 | Entry is reachable from every state, as the survival trigger requires | SM-001 | | |

## Findings

-
