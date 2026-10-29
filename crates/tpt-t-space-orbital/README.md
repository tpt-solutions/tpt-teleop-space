# tpt-t-space-orbital

Rendezvous and Proximity Operations for `tpt-teleop-space`: 6DOF relative
navigation, autonomous docking via MPC, and a 7-DOF robotic servicing arm.

## Modules

- `nav`: 6DOF relative navigation from LiDAR, visual markers, and LEO GPS.
- `docking`: autonomous final approach via MPC, with plume impingement and
  keep-out management.
- `arm`: 7-DOF arm (Canadarm-class) control.
  - Inverse kinematics by damped least squares, `Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ Δx`.
  - Damping scheduled from the manipulability measure, so joint steps stay
    bounded through singularities.
  - Payload mass estimation.

The arm code never panics. A non-positive determinant falls back to the damped
solution instead of failing an inversion.

## Examples

The `examples/` directory holds runnable scenarios:

```sh
cargo run -p tpt-t-space-orbital --example <name>
```

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
