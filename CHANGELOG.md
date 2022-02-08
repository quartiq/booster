# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
* Fixed an issue with spurious resets by ignoring bus error detections on I2C (Refer to [issue
  #128](https://github.com/quartiq/booster/issues/128) for more information.
* Fixed an issue where I2C NACK is encountered randomly while setting the MAX6639 fan speed
  ([#140](https://github.com/quartiq/booster/issues/140)), by using a I2C transfer retry mechanism.
  [#158](https://github.com/quartiq/booster/pull/158)
* Changes to interlock transforms now properly updates the thresholds
* The USB serial port now properly enumerates on Windows

### Changed
* Code reorganized to move hardware configuration into a module
* Added runtime settings control via Miniconf, removed "Properties" API
* Telemetry period is now configurable between 0.5 and 13 seconds
* Fans now operate at a constant, configurable duty cycle whenever an RF channel is enabled
* Device now uses a proper USB VID/PID pair
* The `service` command now prints the detected firmware version and hardware revision of the
  mainboard
* RF channel state machinery was refactored to be more robust

### Added
* Output interlock threshold is now limited to a maximum of 47 dBm
