# Security Policy

## Supported Versions

| Version | Supported |
|---------|-----------|
| 1.0.x   | ✅        |
| < 1.0   | ❌        |

## Reporting a Vulnerability

**Do not open a public issue for security vulnerabilities.**

Report privately to <security@tpt.solutions> with:

- Affected crate(s) and version(s)
- A description of the vulnerability and its impact
- Steps to reproduce or a proof of concept
- Any suggested mitigation

You will receive an acknowledgment within 48 hours. Please allow up to 90
days for a fix and coordinated disclosure.

## Scope Notes

`tpt-teleop-space` runs safety-critical, fully autonomous software on
planetary rovers, satellites, and deep-space probes that may be unreachable
for hours (occultation) to decades (deep space). Vulnerabilities that could
cause:

- loss of attitude control or navigation state,
- failure of the TMR voter, memory scrubber, or Safe Mode transition,
- corruption or loss of DTN bundles, telemetry logs, or custody state,
- unauthenticated command injection over CCSDS TC or DTN uplinks, or
- watchdog starvation / unintended reboot loops on the edge layer

are treated as **critical** regardless of CVSS base score.

## Design Posture

- No async runtime, no heap allocation after initialization, and no locks on
  the realtime data path; memory safety is enforced by the Rust type system
  (`#![forbid(...)]` where practical, `unsafe` audited per `tools/` scripts).
- Dependencies are pinned to the MIT/BSD/ISC chain via `deny.toml` and
  function-banned per the spec §7 matrix; the advisory database is checked on
  every build.
- Deterministic state machines (mission, safety, Safe Mode) with complete,
  auditable transition tables — no ML on control paths.
- Radiation fault tolerance (scrubbing + TMR) and DTN store-and-forward are
  exercised continuously by the simulator's fault-injection models.
