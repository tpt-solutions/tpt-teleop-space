# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this crate adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Phase 20: `script` module, timeline scenario files (link geometry, blackouts,
  SEU upsets, telemetry, Safe Mode, drain) replayed deterministically against
  the DTN store, TMR voter and black box.
- Phase 20: `checkpoint` module, mission-context checkpoints of the Mars flow,
  stored in the core snapshot format so a viewer can rewind to any of them.
- Phase 20: the Mars flow's black box is fed through the mission bus, and its
  NVRAM image is rewritten at every entry rather than once at the end.
- Phase 18: `replay` module, the CSV replay format shared by the sim, the
  `tpts` CLI, and the visualizer. Exact round-trip; line-numbered errors.
- Phase 18: `scenario` module with the canned LEO survey (J2/drag orbit,
  zenith-pointing attitude, eclipse-driven power and battery thermal) and the
  spec §6 Mars Sample Collection flow. The integration test asserts on the
  same run the CLI prints.

- Phase 20: `profile` presets (LEO cubesat, Mars rover, GEO servicer) that
  reuse the workspace's own link, thermal, rover, and power parameters.
- Phase 20: `campaign` fault-injection sweeps over SEU rate (against the real
  TMR voter) and blackout length (against the real DTN store), seeded and
  deterministic.

## [0.1.0]

### Added
- Phase 13: headless simulator with 6DOF orbital mechanics (J2, drag, third
  body), reaction wheel dynamics, sensor noise models, radiation bit-flip
  injection, DTN blackout and light-time profiles, and a variable-speed clock.
