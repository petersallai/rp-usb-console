# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2025-09-30

### Added
- Initial release of rp-usb-console
- Zero-heap USB CDC logging for RP2040 with Embassy
- Bidirectional communication over USB CDC ACM
- Fixed-size 255-byte log message buffers
- Automatic packet fragmentation for reliable transmission
- Non-blocking operation with message dropping on full channels
- Support for standard Rust `log` crate integration
- Command reception channel for host-to-device communication
- Embassy async task integration
- Comprehensive documentation and examples

### Features
- `LogMessage` struct with USB packet fragmentation support
- `start()` function for easy initialization
- Three async tasks for USB device management, TX, and RX
- CRLF line ending support
- Truncation indicator for oversized messages

[Unreleased]: https://github.com/moonblokz/rp-usb-console/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/moonblokz/rp-usb-console/releases/tag/v0.1.0