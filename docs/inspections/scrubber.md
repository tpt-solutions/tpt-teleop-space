# Inspection record: memory scrubber

- **Component:** `tpt-t-space-radiation`, `src/scrub.rs` (Hamming(72,64) correction, CRC backstop)
- **Requirements:** SCR-001, SCR-002, SCR-003
- **Commit inspected:**
- **Inspector / date:**
- **Outcome:** [ ] accepted  [ ] accepted with actions  [ ] rejected

## Checklist

| # | Check | Requirement | Result (pass / fail / n/a) | Notes |
|---|-------|-------------|----------------------------|-------|
| 1 | Every single-bit error in a protected word is corrected, including the parity bits | SCR-001 | | |
| 2 | A double-bit error is reported as uncorrectable, never returned as corrected data | SCR-002 | | |
| 3 | The CRC backstop detects corruption that the per-word code misses | SCR-003 | | |
| 4 | Corrected words are written back, and healed parity is counted | SCR-001 | | |
| 5 | Shape mismatches between the region and the code table are reported, not ignored | SCR-002 | | |
| 6 | The scrub pass is bounded in time and does not allocate per word | SCR-001 | | |

## Findings

-
