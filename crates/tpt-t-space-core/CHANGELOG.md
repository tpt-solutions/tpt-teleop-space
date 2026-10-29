# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this crate adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0]

### Added
- Phase 2: mission state machine (Cruise → Approach → Operations → Safe Mode),
  lock-free SPSC message bus, central event loop, CRC32, persistent snapshot
  store, and the rkyv wire prelude.
- Phase 6: shared zero-allocation Safe Mode transition (payload shed, sun-point
  slew, low-rate DTN beacon).
