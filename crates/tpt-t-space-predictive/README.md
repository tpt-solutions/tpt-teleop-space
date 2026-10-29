# tpt-t-space-predictive

Onboard predictive autonomy for `tpt-teleop-space`: bounded MPC, stereo DEM
generation, candidate-trajectory path planning, and the autonomous decision
framework reused by rover hazard avoidance and orbital docking.

## Modules

- `mpc`: bounded model predictive control.
- `dem`: local Digital Elevation Model generated from stereo disparities at
  10 Hz. A pinhole model converts disparity to depth.
- `planner`: candidate-trajectory path planner, with a structure-of-arrays
  layout that auto-vectorizes.
- `decision`: the reusable autonomous decision framework.
  - `Evaluator` is the contract a domain implements.
  - `select_best` picks the top candidate behind a hard safety gate. A candidate
    that fails `EvalOutcome::safe` can never be selected, whatever its score.
  - Deterministic and allocation-free.

## Benchmarks

```sh
cargo bench -p tpt-t-space-predictive
```

The `replan` benchmark measures the replan budget (50 ms target).

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
