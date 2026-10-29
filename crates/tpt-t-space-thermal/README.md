# tpt-t-space-thermal

Thermal management for `tpt-teleop-space`. It monitors heater states, radiator
deployment, and battery temperatures.

A limit breach, such as a battery below −20 °C, arms the shared Safe Mode
trigger in `tpt-t-space-core`, so the spacecraft protects itself.

## Design

Every monitor is a hysteresis-based finite-state watcher. A limit must be
crossed and held for a debounce interval before it reports, so one noisy sample
cannot fire the survival transition. Monitors are fixed-size, allocation-free,
and panic-free.

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
