# tpt-t-space-satellite

Attitude Determination and Control System (ADCS) for `tpt-teleop-space`:
SIMD-accelerated MEKF, reaction wheel control, momentum dumping, and formation
flying.

> **Nightly only.** The crate uses `#![feature(portable_simd)]`, so it is
> excluded from the default stable workspace build. It carries its own
> `rust-toolchain.toml` and is built by the nightly CI job.

## Modules

- `mekf`: multiplicative extended Kalman filter fusing star tracker, sun sensor,
  and gyro measurements.
- `simdv`: SIMD vector helpers used by the filter.
- `wheels`: reaction wheel control and momentum dumping.
- `formation`: formation flying.
- `adcs`: the control loop. One `Adcs::cycle` runs a TMR-voted quaternion
  estimate, the pointing control law, and the wheel torque command. The cycle
  budget is under 200 µs.

Sub-arcsecond pointing mode tightens the deadband once the tracker noise floor
is reached.

## Benchmarks

```sh
cargo +nightly bench -p tpt-t-space-satellite
```

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
