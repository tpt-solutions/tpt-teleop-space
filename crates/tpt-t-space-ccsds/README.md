# tpt-t-space-ccsds

Zero-allocation, zero-copy CCSDS Space Packet, Telemetry (TM) and Telecommand
(TC) parsers for `tpt-teleop-space`.

Parsers return *borrowed views* over caller-owned byte slices, so a parsed
packet costs one struct on the stack and nothing on the heap. Malformed input
is rejected with `None` or `ParseError`, never a panic.

## Modules

- `packet`: CCSDS Space Packet primary header and data field.
- `tm`: Telemetry transfer frames.
- `tc`: Telecommand transfer frames.

## Verification

- Deterministic mutation fuzzing of every parser.
- Property tests that split input at arbitrary truncation points.

The crate has no dependencies. The heavy CCSDS libraries are excluded by the
workspace dependency matrix (`deny.toml`).

## Part of tpt-teleop-space

See the [workspace README](../../README.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
