# FMEA — Safe Mode Transition (Phase 15)

> Failure Modes and Effects Analysis for the single, atomic, zero-
> allocation Safe Mode transition (`tpt-t-space-core::safe_mode`), the
> triggers that arm it (`tpt-t-space-thermal`, `tpt-t-space-power`), and
> the recovery path. Severity classes: **S1** loss of mission, **S2** loss
> of capability, **S3** degraded performance.

| # | Failure mode | Cause | Effect | Sev | Detection | Mitigation in design | Residual risk |
|---|--------------|-------|--------|-----|-----------|----------------------|---------------|
| 1 | Trigger never fires | Thermal channel sensor stuck in-range (plausible value from a dead sensor) | Asset stays in hot/cold soak beyond limits | S1 | Cross-channel voting (sensor plausibility), ground health report | Thermal channels are independent sensors; heater-feedback faults surface in `health_bits` | Ground pass latency (hours); accepted — two independent channels must agree to *be quiet* |
| 2 | Trigger fires spuriously (no real anomaly) | Noisy sample + debounce boundary | Unnecessary survival entry: payloads off, science lost | S2 | Entry is recorded with cause + timestamp (`transitions()`) | Debounce window (5 s thermal, 60 s power) filters single-sample noise; idempotent re-entry performs no actuator replay | Science sequence restart cost after recovery |
| 3 | Entry sequence partially applied (shed without slew) | Power loss mid-transition; actuator fault | Thermal positive lost; beacon silent | S1 | `enter()` performs all three actions unconditionally before latching; watchdog bites if the loop stalls mid-entry | Atomic design: no intermediate state exists to be observed; watchdog reset re-runs the entry from the top | Total power loss — nothing is recoverable in software |
| 4 | Beacon never starts | DTN agent store full at entry | Ground never learns of Safe Mode until a scheduled pass | S2 | Beacon action returns through `SafeModeActions`; the health report carries `SafeModeState` | Beacon payload is a fixed 1-frame record (always fits one NVRAM sector); store reserves one sector for it | Store exhausted by telemetry backlog — load shedding (Phase 6) drops backlog priority first |
| 5 | Recovery executed while fault persists | Operator/ground error, or clock fault | Re-entry loop; actuator wear | S2 | Re-entry is idempotent and counted (`transitions()`); flapping visible to ground | `recover()` requires the caller to have cleared the *source* condition; re-entry performs the full sequence again (safe by construction) | Chattering bounded by sensor debounce + ground review of the transition log |
| 6 | Safe Mode entered during teleop session | Thermal/power trigger mid-session | Operator loses control; session torn down | S2 | Safety state machine (`tpt-t-domain-bridge`) + mission FSM both record | Survival beats teleop by policy (asset > mission > session); the adapter refuses re-engage while `hazard_flag` latches | Operator surprise — mitigated by the alert downlink (#4) |
| 7 | Watchdog reset during entry | The entry loop overruns its budget | Core resets, entry half-applied | S1 | Bite latches `ResetCause::WatchdogBite`; boot path reads cause | Entry is three fixed-cost action calls (µs each) — budget is orders of magnitude under the watchdog window; boot path re-runs entry | Re-entry loop if an actuator line is shorted — watchdog continues resetting; ground sees repeated bites in the log |
| 8 | Snapshot corrupt at reset recovery | SEU multi-bit hit on the NVRAM image | Mission context lost, fresh start | S2 | CRC32 + `bytecheck` on restore; corrupt → `None` | The snapshot policy is fail-safe: a corrupt snapshot yields a *fresh* mission, never a poisoned one | In-progress science progress lost (sequence restarts) |

**Conclusion:** every S1 path requires either total power loss or physical
actuator failure — outside software control — and all S2 paths are
bounded, recorded, and recoverable. The transition's atomicity (no
intermediate observable state) and idempotence (no replay on re-entry)
close the classic partial-application and chatter hazards.

## Traceability of state transitions (Phase 15, item 6)

Every state transition in the workspace flows through an auditable FSM
with complete transition tables:

| FSM | Crate | Audit surface |
|-----|-------|---------------|
| Mission (Cruise → Approach → Operations → Safe Mode) | `tpt-t-space-core::machine` | `TransitionError` on every illegal request; unit tests prove the full legal graph |
| Universal safety (autonomy ↔ teleop) | `tpt-t-domain-bridge::safety` | `IllegalTransition` carries from-state + event; `Display` uses the spec's state names |
| Safe Mode lifecycle | `tpt-t-space-core::safe_mode` | Cause + timestamp + transition counter per episode |
| Science sequencing | `tpt-t-space-rover::science` | Per-step verified/failure/rollback trail (`audit()`) |
| LTP sessions | `tpt-t-space-dtn::ltp` | Session state machine with closed/acked byte counters |
| DTN custody | `tpt-t-space-dtn::bundle` | `CustodyState` recorded in the CRC-guarded sector (survives reset) |
| SEU book | `tpt-t-space-radiation::seu` | Exact lifetime totals survive ring truncation |

The four `tools/` audits (lock, alloc, recursion, panic) gate these
invariants in CI.
