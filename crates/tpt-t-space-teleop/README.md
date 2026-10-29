# tpt-t-space-teleop

The space-domain teleoperation adapter for `tpt-teleop-space`. It implements
`DomainTeleopInterface` from `tpt-t-domain-bridge` for deep-space links.

## Modules

- `transport`: DTN custody-verified transport.
- `display`: predictive display. It shows the operator a predicted asset
  position, masking the light-speed delay (about 12 minutes to Mars).
- `sequencing`: uploaded command sequencing with hazard avoidance. Command
  translation includes slew limiting and blend handover.
- `bandwidth`: telemetry bandwidth optimization. Telemetry uses delta and RLE
  coding, and images use subsample and quantize coding. Both are
  dependency-free and lossy by contract, with bounded error and documented
  ratios. The ground decoders mirror these formats.

## Part of tpt-teleop-space

See the [workspace README](../../README.md) and the
[bridge specification](../../bridge%20spec.txt).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
