# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this crate adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0]

### Added
- Phase 8 (nightly only): SIMD-accelerated MEKF fusing star tracker, sun sensor,
  and gyro; reaction wheel control; momentum dumping; orbit maintenance;
  TMR-voted quaternion updates; no-panic bounded pseudo-inverse; sub-arcsecond
  pointing mode; and formation flying (under the 200 µs ADCS loop budget).
