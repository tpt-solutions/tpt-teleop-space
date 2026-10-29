# tpt-t-space-power

Power subsystem management for `tpt-teleop-space`: solar array pointing,
battery cycle tracking, eclipse load shedding, and the dust-storm survival
trigger.

## Features

- **Solar pointing**: a bounded grid search over the two gimbal axes maximizes
  `cos(incidence)` against the sun vector. It is deterministic and
  allocation-free.
- **Battery tracking**: coulomb counting with a depth-of-discharge accumulator.
- **Eclipse load shedding**: three priority tiers.
- **Survival trigger**: solar output below the survival threshold for a
  sustained interval (a dust storm, for example) arms the shared Safe Mode
  transition in `tpt-t-space-core`.

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
