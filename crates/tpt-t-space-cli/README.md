# tpt-t-space-cli

`tpts`, the adoption CLI for `tpt-teleop-space`. It collects the chores around
the workspace into one binary that works the same on Windows, macOS and Linux.

## Commands

- `tpts doctor`: check rustc, cargo, the nightly toolchain, the aarch64 target,
  cargo-deny, and bash. Each gap is printed with the command that fixes it.
- `tpts audit [--strict]`: run the lock, recursion, panic, and allocation audits
  and `cargo deny check`, then print a pass/fail table.
- `tpts budgets`: run the three strict benches against the spec budgets.
- `tpts ccsds <hex|file> [--crc]`: decode a CCSDS Space Packet stream (APID,
  sequence count, flags, length, optional CRC-16 check).
- `tpts orbit [--orbits N] [--step S] [--out FILE]`: propagate a LEO survey and
  write a replay file for the visualizer.
- `tpts scenario mars [--out FILE]`: run the spec §6 Mars Sample Collection
  flow, print its timeline, and write a replay file.
- `tpts profile [NAME]`: list the mission-profile presets (LEO cubesat, Mars
  rover, GEO servicer), or show one in full.
- `tpts seq <steps>`: lint an uplink sequence and dry-run it over the Mars
  terrain, using the rover's hazard model. Steps are `drive M`, `turn D`, and
  `capture ID`, separated by `;` or `,`.
- `tpts campaign seu|blackout [--seeds N]`: a seeded fault-injection sweep
  against the TMR voter (SEU rate) or the DTN store (blackout length).
- `tpts blackbox <hex|file>`: decode a Safe Mode black-box image.
- `tpts new <rover|satellite|orbital> [--name NAME]`: scaffold a mission crate
  under `crates/`, wired to the core Safe Mode controller and the DTN agent,
  with a smoke test.

## Part of tpt-teleop-space

See the [workspace README](../../README.md) and the
[quickstart](../../docs/quickstart.md).

## License

Licensed under either of [MIT](../../LICENSE-MIT) or
[Apache-2.0](../../LICENSE-APACHE) at your option.
