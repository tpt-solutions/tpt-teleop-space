# tpt-t-space-sim

The headless simulator for `tpt-teleop-space`. It provides the physics and
fault models that the other crates are validated against, so space software
can be tested on Earth.

## Modules

- `orbit`: 6DOF orbital mechanics, including J2, drag, and third-body
  perturbations.
- `wheels`: reaction wheel dynamics.
- `sensors`: sensor noise models.
- `radiation_env`: radiation bit-flip injection.
- `dtn_link`: DTN blackout windows (planetary occultation, dust storms) and
  signal-path delay (Mars–Earth light time). The profile is deterministic: given
  a simulated time, it answers whether the link is up.
- `clock`: variable-speed simulation clock. Multi-year cruise phases run in
  minutes of wall time.

## Part of tpt-teleop-space

See the [workspace README](../../README.md) and the
[quickstart](../../docs/quickstart.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
