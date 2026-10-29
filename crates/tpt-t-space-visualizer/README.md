# tpt-t-space-visualizer

An egui renderer for `tpt-teleop-space`. It shows four panes over the
simulator's data model:

- **Orbit**: 3D trajectory and ground track.
- **Attitude**: quaternion and sun-vector alignment.
- **Terrain**: rover DEM with planned and actual paths.
- **Budgets**: power and thermal traces over time.

## Running

```sh
cargo run --manifest-path crates/tpt-t-space-visualizer/Cargo.toml
```

This crate is **excluded from the workspace**. The egui and winit dependency
chain sits outside the MIT-chain `deny.toml` policy that workspace members are
audited under. It builds in isolation with its own CI job, and it consumes only
the workspace's public types.

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
