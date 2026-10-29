# Changelog

All notable changes to this crate are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this crate adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.1.0]

### Added
- Phase 11: `#![no_std]` bare-metal layer with register abstraction, hardware
  watchdog management, and static RTOS task table, validated on
  `aarch64-unknown-none-softfloat`.
