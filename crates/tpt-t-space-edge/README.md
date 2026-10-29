# tpt-t-space-edge

The bare-metal RTOS execution layer for `tpt-teleop-space`. The crate is
`#![no_std]`, with no operating system and no heap.

Space-grade microcontrollers (RAD750, LEON3, rad-hard ARM Cortex-R) get no OS
to rely on. This crate provides the minimal, fully static environment that the
other workspace crates run on.

## Modules

- `reg`: volatile register-cell abstraction (`read`, `write`, `modify` over raw
  addresses), the base of every driver.
- `wdt`: hardware watchdog management. Windowed feeding against a deadline
  schedule, bite-counter bookkeeping, and a reset-cause latch the boot path reads.
- `sched`: static RTOS task table.

## Building

Host builds run the unit tests with `std` enabled through the test cfg.
Bare-metal validation targets `aarch64-unknown-none-softfloat`:

```sh
cargo build -p tpt-t-space-edge --target aarch64-unknown-none-softfloat
```

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
