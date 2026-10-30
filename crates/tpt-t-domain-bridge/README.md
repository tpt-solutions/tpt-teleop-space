# tpt-t-domain-bridge

The Domain Teleoperation Interface (DTI) shared by every `tpt-teleop-*`
domain repository: universal wire types, the safety state machine, and the
teleop-assistance request gate (`request_teleop_assistance()`).

This crate is vendored from `tpt-teleop` during co-development, so that
`tpt-teleop-space` speaks the same contract without a cross-repo dependency.
The public API is the stable contract; the canonical home is the `tpt-teleop`
repo.

## Contents

- **DTI trait** (`dti`): `DomainTeleopInterface`, implemented by every domain
  adapter. Methods: `on_teleop_engage`, `on_teleop_disengage`,
  `on_control_command`, `get_domain_state`, `get_sensor_feed`. Errors are
  reported as `DtiError`.
- **Wire types** (`wire`): zero-copy `rkyv::Archive` records. Commands and
  state: `ControlCommand`, `InputState`, `Pose6Dof`, `Button`, `ButtonState`,
  `OperatorId`, `DomainState`, `DomainTelemetry`. Sensing: `SensorFeed`,
  `VideoFrame`, `PixelFormat`, `PointCloud`, `AudioStream`. Feedback:
  `HapticCmd`, `HapticFeedback`. Batches are bounded
  (`MAX_VIDEO_FRAMES`, `MAX_POINT_CLOUDS`), and `serialize` / `access` /
  `deserialize` helpers handle the byte boundary.
- **Safety** (`safety`): `SafetyStateMachine` over `SafetyState`
  (`AUTONOMOUS` → `REQUESTING_TELEOP` → `TELEOP_ACTIVE` →
  `RETURNING_TO_AUTONOMY` → `AUTONOMOUS`, with `EMERGENCY_STOP` reachable from
  any state). Events are `SafetyEvent`. Illegal event/state pairs return
  `IllegalTransition` instead of panicking, so the caller can route them
  through fault handling.
- **Assist** (`assist`): `AssistRequest`, `AssistReason`, `Urgency`, and
  `AssistRequester`. `AssistRequester::request` is the `request_teleop_assistance()`
  entry point: it stamps each request with a sequence number and timestamp,
  enforces a re-request cooldown so a flapping fault cannot flood the operator
  queue, and delivers through an `AssistSink` (a trait, also implemented for
  closures). A rejected delivery is returned as `AssistError::SinkRejected` so
  the caller can retry on the next tick.

## Design invariants

- No async runtime, no `serde`, and no `std::sync` locks on the data path.
- Wire types are fixed-size records (bounded arrays and length fields, not
  `Vec`). Pixel and point payloads are referenced by offset into out-of-band
  buffers, so archived views can be mapped over shared memory or the wire with
  zero copies and no per-event allocation.
- Archived types are validated with `bytecheck` before use.

## Testing

```sh
cargo test -p tpt-t-domain-bridge
```

## Part of tpt-teleop-space

See the [workspace README](../../README.md) and the
[bridge specification](../../bridge%20spec.txt) for the integration contract.
The change history is in [CHANGELOG.md](CHANGELOG.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
