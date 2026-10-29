# tpt-t-space-radiation

Software fault tolerance for the radiation environment, for `tpt-teleop-space`.
Hardware rad-hardening lags commercial silicon, so this crate compensates in
software.

## Modules

- `scrub`: background memory scrubbing. Each protected 64-bit word carries a
  Hamming(72,64) SECDED code, and a low-priority walk corrects single-bit upsets
  before they accumulate. A CRC32 region check is the backstop.
- `tmr`: Triple Modular Redundancy voter. Critical calculations run three times
  on separate cores; the voter masks a diverged result and flags the affected
  core for a watchdog reset. The vote budget is under 50 µs.
- `seu`: single-event-upset detection and recovery bookkeeping.

## Benchmarks

```sh
cargo bench -p tpt-t-space-radiation
```

The `tmr` benchmark measures the voter.

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
