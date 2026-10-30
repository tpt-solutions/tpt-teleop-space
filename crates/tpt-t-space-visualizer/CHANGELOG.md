# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this crate adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Phase 20: Downlink pane decodes CCSDS telemetry through the packet-definition
  table and follows a growing capture (`--ccsds FILE`).
- Phase 20: Rewind pane restores Mars-flow checkpoints and shows the timeline
  up to each one. Loaded with the demo, not with a replay file.
- Phase 20: Link pane gives a predicted-vs-confirmed rover readout using the
  predictive display, which reports no prediction on the 12-minute Mars link.

## [0.1.0]

### Added
- Phase 13: egui visualizer with orbit, attitude, terrain, and budget panes.
  Excluded from the workspace build; built in isolation with its own CI job.
