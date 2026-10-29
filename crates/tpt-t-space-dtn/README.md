# tpt-t-space-dtn

Delay-Tolerant Networking for `tpt-teleop-space`: BPv7-style bundles with
custody transfer, LTP sessions, and NVRAM store-and-forward that survives
multi-hour blackouts.

## Modules

- `bundle`: `Bundle` records (endpoints, timestamps, lifetime, custody state,
  payload length), archived with rkyv directly into pre-allocated NVRAM sectors.
- `ltp`: Licklider Transmission Protocol sessions with checkpoints and reports.
- `link`: link handling for store-and-forward.

## Sector format

Each NVRAM sector is self-describing and CRC-guarded. After a processor reset,
the store is revalidated from NVRAM without trusting RAM.

```text
[0..4)   magic "DTN1"          [16..20) CRC32 over meta+payload
[4..8)   meta archived len     [20..20+M) rkyv-archived Bundle
```

Payload bytes follow the archived metadata inside the sector.

## Features

- Exact-resume store-and-forward.
- Hash-chained telemetry log.
- Validated against a 4-hour occultation with a mid-blackout processor reset.

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
