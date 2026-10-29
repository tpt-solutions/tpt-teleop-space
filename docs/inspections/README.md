# Formal inspection templates

Each file is a blank inspection record for one safety-critical component.
Copy it, fill in the header and each checklist row, and keep the completed
copy with the release evidence. The templates are not inspections on their
own: an inspection is recorded only when a reviewer has worked through the
checklist against the code at a named commit.

Every checklist row names the requirement IDs it covers
([`../requirements.csv`](../requirements.csv)). The traceability gate
(`tools/req-trace.sh`) confirms that each requirement has a test; the
inspection confirms the design and code match what the test asserts.

| Template | Component | Requirements |
|----------|-----------|--------------|
| [tmr-voter.md](tmr-voter.md) | TMR voter (`tpt-t-space-radiation`) | TMR-001, TMR-002, TMR-003 |
| [scrubber.md](scrubber.md) | Memory scrubber (`tpt-t-space-radiation`) | SCR-001, SCR-002, SCR-003 |
| [safe-mode.md](safe-mode.md) | Safe Mode controller (`tpt-t-space-core`) | SM-001, SM-002, FSM-001 |
| [dtn-custody.md](dtn-custody.md) | DTN custody logic (`tpt-t-space-dtn`) | DTN-001, DTN-002 |
