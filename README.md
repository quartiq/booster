# Booster NGFW (Next-Generation Firmware)

Updated firmware for the Sinara Booster hardware

This software is a complete rewrite of the Booster firmware to align it with
the other projects (Stabilizer, Thermostat, ARTIQ), to resolve quality issues
in the original firmware, and to enable continued maintenance and improvements.

The firmware is offered as an option to Booster and licensed on a restrictive
per-device basis with redistribution and reuse prohibited. End users obtain
access to the source code to modify it but the number of installed instances is
limited and certified. Once the development and maintenance investments have
been recovered the firmware will be offered under an open source license.

# License

Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
Unauthorized usage, editing, or copying of this code or the contents of this
this repository, via any medium is strictly prohibited.
Proprietary and confidential.

You may purchase a [license](LICENSE) to use this software from
[QUARTIQ](mailto:sales@quartiq.com).


# DFU Instructions

**Prerequisites**
* Ensure `dfu-util` is installed. On Ubuntu, install it from `apt` using `sudo apt-get install
dfu-util`

The following instructions describe the process of uploading a new firmware image over the DFU
Bootloader USB interface.

1. Generate the firmware image: `cargo build`
    - Note: You may append `--release` to build the firmware in `Release` mode.
    - Note: You may also use the latest release instead of building firmware.

1. Generate the DFU file for your firmware build: `dfu-tool convert target/thumbv7em-none-eabihf/debug/booster booster.dfu`
    - Note: If you are generating a `Release` build, replace `debug` with `release` in the above command.

1. Reset Booster into DFU mode:
    - Insert a pin into the DFU Bootloader hole to press the DFU button
    - While the DFU button is pressed, power cycle booster by turning off the power switch for at
    least 10 seconds and then turn the power switch on.

1. Verify Booster is in DFU mode: `dfu-util -l` should show 4 entries beginning with `Found DFU: [0483:df11]`
1. Upload the DFU file to Booster:
```
dfu-util -d 0483:df11 -a 0 --download booster.dfu
```
