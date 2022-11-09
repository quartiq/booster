[![QUARTIQ Matrix Chat](https://img.shields.io/matrix/quartiq:matrix.org)](https://matrix.to/#/#quartiq:matrix.org)

# Booster NGFW (Next-Generation Firmware)

Updated firmware for the Sinara Booster hardware

This software is a complete rewrite of the Booster firmware to align it with
the other projects (Stabilizer, Thermostat, ARTIQ), to resolve quality issues
in the original firmware, and to enable continued maintenance and improvements.

# Documentation

Please see the [Online Documentation](https://quartiq.de/booster-doc) for usage information.

# License

Copyright (C) 2020-2022 QUARTIQ GmbH

Licensed as MIT or Apache-2.0
# Hardware

Booster is an 8 channel RF power amplifier in the Sinara open hardware ecosystem.
The open hardware designs and hardware discussions are located at
https://github.com/sinara-hw/Booster/wiki.
The hardware is available from Creotech, QUARTIQ, Technosyste, and M-Labs.

# Generating Releases

When a release is ready, `develop` must be merged into `master`.

The corresponding merge commit is then tagged with the version in the form of `vX.Y.Z`, where X, Y,
and Z are the semantic version major, minor, and patch versions. The tag must be pushed using `git
push origin vX.Y.Z`. This will automatically trigger CI to generate the release.

After the tag is generated, `master` must be merged back into `develop`.
