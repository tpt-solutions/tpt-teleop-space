# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this crate adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Phase 20: `tpts scenario run FILE` replays a timeline scenario file, and
  `tpts campaign run FILE` runs a sweep described by a file.
- Phase 19: the `tpts` binary with `doctor`, `audit [--strict]`, `budgets`,
  `ccsds`, `orbit`, `scenario mars`, and `new`. Argument parsing is hand-rolled
  to keep the zero-dependency policy.
- Phase 20: `profile` (mission-profile presets), `seq` (uplink sequence linter
  and dry run over the Mars terrain), `campaign` (SEU-rate and blackout-length
  sweeps), and `blackbox` (Safe Mode black-box decoder). `ccsds` now decodes
  payload fields through the packet-definition table.
- `campaign run FILE`: fault sweeps declared in a line-based scenario file,
  with errors that name the line. `scenario mars --blackbox FILE` writes the
  recorded Safe Mode image.
- On Windows, the audit scripts run under Git Bash, located by absolute path
  (`TPT_BASH` overrides it). A bare `bash` resolves to the WSL launcher in
  `System32`, which can block.
