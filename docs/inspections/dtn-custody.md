# Inspection record: DTN custody logic

- **Component:** `tpt-t-space-dtn`, `src/bundle.rs` (`BundleStore`: `accept_custody`, `recover`), `src/link.rs` (`DtnAgent::poll`, `deliver`)
- **Requirements:** DTN-001, DTN-002
- **Commit inspected:**
- **Inspector / date:**
- **Outcome:** [ ] accepted  [ ] accepted with actions  [ ] rejected

## Checklist

| # | Check | Requirement | Result (pass / fail / n/a) | Notes |
|---|-------|-------------|----------------------------|-------|
| 1 | While the link is down, `poll` reports `LinkDown` and transmits nothing | DTN-001 | | |
| 2 | Transmission resumes at the saved cursor after a link drop, with no gap or repeat | DTN-001 | | |
| 3 | A sector is released only after `deliver`, never by poll alone, except once fully transmitted and acknowledged | DTN-001 | | |
| 4 | `accept_custody` rewrites metadata in place with the same length and a fresh CRC | DTN-002 | | |
| 5 | `recover` rejects a sector whose CRC does not match and frees it | DTN-002 | | |
| 6 | Custody transfer does not allocate (see `tools/alloc-audit.sh`) | DTN-002 | | |
| 7 | The NVRAM image is the only state needed to resume after a processor reset | DTN-001 | | |

## Findings

-
