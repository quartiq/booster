# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.6.0] - 2024-08-28

### Changed
* Network stack is randomly seeded on startup so that random ports are used.
* Serial terminal replaced with `menu` for simplicity
* The broker can now be specified using DNS
* Updated to RTIC v2 and async support. MSRV bumped to 1.75.0
* All settings, including RF channel settings, are now exposed via the USB interface.
  This means that MQTT is no longer required for basic device operation. Any settings saved on
  the device will remain there, but newer settings saved via the USB interface will be stored in
  mainboard flash and will overwrite EEPROM-based settings during device boot. Reversion to older
  firmware variants will still be able to use existing EEPROM settings, but the EEPROM contents
  are no longer modified when settings are changed.
* [python] Python client has had `TelemetryReader` removed, `BoosterApi` was renamed to `Booster`
  and now takes an `aiomqtt-client` as the constructor.

### Fixed
* Heavy network traffic no longer causes Booster to encounter watchdog resets or other spurious
resets.

## [0.5.0] - 2023-07-03

### Added
* Support for different ethernet daughterboards using the ENC424J00 has been added.
* Serial port now supports TeraTerm + Putty
* Serial port now supports backspacing
* Support for static IP + netmask and gateway configuration
    * NOTE: Boosters that used firmware from before 0.4.0 will return to old network configurations.
    To re-enable DHCP, reconfigure the IP address to 0.0.0.0/0
* Support added for Booster v1.6 hardware

### Changed
* Removed custom `mutex` implementation in favor of leveraging `shared-bus`
* Fans will now remain on whenever a channel is powered as opposed to actively outputting RF. This
  addresses heating issues observed with some Booster configurations resulting in channel wedging.
* Relicensed as MIT/Apache
* Miniconf, minireq, and minimq versions updated to incorporate efficiency updates and bugfixes
* External MAC code has been removed, smoltcp updated to 0.9 to reduce memory and networking
  overhead

### Fixed
* A few dependencies were deprecated because changes landed upstream. These dependencies were
  corrected to point upstream.
* The ethernet PHY is now detected at run-time as opposed to compile time, so there is now a single
  booster binary. The detected PHY can be found in the service prompt over the USB interface.
* Fixed an issue where the device may permanently disconnect from MQTT brokers

## [0.4.0]

### Added
* The W5500 now operates as an external MAC, and smoltcp is used as the network stack.
* DHCP support has been added, `netmask`, `ip-address`, and `gateway` settings have been removed.
  Settings are backwards compatible with previous Booster releases.
* Fan speed is now stored in EEPROM and configurable via the serial interface.

## [0.3.0]

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
* Added runtime settings control via Miniconf, removed "Properties" API. This has changed the MQTT topic layout and API.
* Telemetry period is now configurable between 0.5 and 13 seconds
* Fans now operate at a constant, configurable duty cycle whenever an RF channel is enabled
* Device now uses a proper USB VID/PID pair
* The `service` command now prints the detected firmware version and hardware revision of the
  mainboard
* RF channel state machinery was refactored to be more robust

### Added
* Output interlock threshold is now limited to a maximum of 47 dBm

[Unreleased]: https://github.com/quartiq/booster/compare/v0.6.0...HEAD
[0.6.0]: https://github.com/quartiq/booster/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/quartiq/booster/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/quartiq/booster/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/quartiq/booster/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/quartiq/booster/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/quartiq/booster/compare/v0.0.1...v0.1.0
