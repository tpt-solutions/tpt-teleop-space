# tpt-t-space-rover

Planetary surface operations for `tpt-teleop-space`: wheel and visual-inertial
odometry, autonomous hazard avoidance, science sequencing with rollback, and
wheel health monitoring.

> **Nightly only.** The crate uses `#![feature(portable_simd)]`, so it is
> excluded from the default stable workspace build. It carries its own
> `rust-toolchain.toml` and is built by the nightly CI job.

## Modules

- `odometry`: wheel odometry for a differential drive, with slip handling.
- `vio`: visual-inertial odometry.
- `hazards`: hazard avoidance on the `tpt-t-space-predictive` DEM and path
  planner. `HazardAvoidance::traverse_tick` runs the 10 Hz loop: stereo, DEM,
  replan, follow.
- `science`: multi-sol science sequencing with sensor-verified steps and rollback.
- `wheel_health`: wheel health monitoring.

The integration test in `tests/detour.rs` covers the 25° slope to 15 m detour
scenario, replanned under the 50 ms budget with no ground input.

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
