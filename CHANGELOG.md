# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

* Fixed an issue with spurious resets by ignoring bus error detections on I2C (Refer to [issue
  #128](https://github.com/quartiq/booster/issues/128) for more information.
* Added transform expression to CLI tool help ([#153](https://github.com/quartiq/booster/pull/153)).
* Added support for ENC424J600 PHYs using Smoltcp. Bumped Minimq version.
  [#156](https://github.com/quartiq/booster/pull/156)
* Fixed an issue where I2C NACK is encountered randomly while setting the MAX6639 fan speed
  ([#140](https://github.com/quartiq/booster/issues/140)), by using a I2C transfer retry mechanism.
  [#158](https://github.com/quartiq/booster/pull/158)
* Code reorganized to move hardware configuration into a module
* Added runtime settings control via Miniconf
* Telemetry period is now configurable between 0.5 and 13 seconds
