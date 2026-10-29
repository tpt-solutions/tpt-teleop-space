# tpt-t-domain-bridge

The Domain Teleoperation Interface (DTI) shared by every `tpt-teleop-*`
domain repository: universal wire types, the safety state machine, and the
`request_teleop_assistance()` API.

This crate is vendored from `tpt-teleop` during co-development, so that
`tpt-teleop-space` speaks the same contract without a cross-repo dependency.

## Contents

- **DTI trait** (`dti`): `on_teleop_engage`, `on_teleop_disengage`,
  `on_control_command`, `get_domain_state`, `get_sensor_feed`.
- **Wire types** (`wire`): `ControlCommand`, `InputState`, `SensorFeed`,
  `DomainState`, `DomainTelemetry`, `HapticCmd` / `HapticFeedback`, all
  `rkyv::Archive`.
- **Safety** (`safety`): the universal safety state machine and `SafetyEvent`.
- **Assist** (`assist`): `AssistRequest` and `AssistRequester`. The requester
  enforces a re-request cooldown so a flapping fault cannot flood the operator
  queue.

## Part of tpt-teleop-space

See the [workspace README](../../README.md) and the
[bridge specification](../../bridge%20spec.txt) for the integration contract.

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
