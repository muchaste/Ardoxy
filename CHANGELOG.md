# Changelog

All notable changes to this project are documented in this file.

## [2.0.0] - 2026-09-08

### Added
- Ardoxy-OS: runtime-configurable Arduino sketches paired with a Python GUI (`utils/ardoxy_gui/ardoxy_gui.py`), replacing the need to edit and re-upload sketches for each experiment.
  - Live mode (`examples/ardoxy_live/ardoxy_live.ino`) — PC stays connected for the whole experiment; MEASURE, SETPOINT, and SEQUENCE control modes.
  - Standalone mode (`examples/ardoxy_standalone/ardoxy_standalone.ino`, `examples/ardoxy_standalone_20x4lcd/ardoxy_standalone_20x4lcd.ino`) — up to 8 independently configured/scheduled channels across two FireStingO2 sensors, SD card logging, RTC-scheduled starts, automatic power-outage recovery.
- New dedicated example sketches: `standalone_solenoid_8ch`, `manual_communication_uno`.
- Citation reference (Mucha 2025, J Exp Biol) added to all example sketch headers and library source headers.
- README: "Published Use Cases" section linking studies that have used Ardoxy.

### Changed
- README restructured to document both Ardoxy-OS modes and all example sketches; Ardoxy-OS status updated from "under development" to stable.

## [1.0.1] - previous release
- See git history for changes prior to 2.0.0.
