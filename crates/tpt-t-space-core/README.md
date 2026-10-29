# tpt-t-space-core

The foundation crate of `tpt-teleop-space`: mission state machine, lock-free
message bus, central event loop, persistent snapshots, and the shared rkyv
wire-type prelude.

## Modules

- `machine`: mission state machine (Cruise → Approach → Operations → Safe Mode).
- `bus`: lock-free single-producer / single-consumer ring buffer, and the
  fan-in `Bus` that composes SPSC rings into a multi-producer, single-consumer
  path. Wait-free on the fast path, with no heap allocation after construction.
- `event_loop`: the central event loop that drives the mission.
- `safe_mode`: the single atomic, zero-allocation Safe Mode transition (payload
  shed, then sun-point slew, then low-rate DTN beacon). Other crates arm it.
- `snapshot`: persistent snapshot store.
- `crc`: table-free CRC32 (IEEE 802.3), shared by the scrubber, DTN payloads,
  and snapshots.
- `wire`: the rkyv wire-type prelude that other crates depend on.

## Part of tpt-teleop-space

See the [workspace README](../../README.md) and
[`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
