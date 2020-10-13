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

# Hardware

Booster is an 8 channel RF power amplifier in the Sinara open hardware ecosystem.
The open hardware designs and hardware discussions are located at
https://github.com/sinara-hw/Booster/wiki.
The hardware is available from Creotech, QUARTIQ, Technosyste, and M-Labs.

# Getting Started

This section is intended to acquaint the user with the Booster NGFW application.

There are three ways to interface with the Booster application:
1. Pressing any of two the front-panel buttons
1. Communicating with Booster over the USB port
1. Communicating with Booster over ethernet via MQTT

## Front Panel

There are two buttons on the front of booster. One is labeled "Interlock Reset" and the other is
labeled "Standby".

The "Standby" button places all channels into an RF-disabled state. In this state, the channel is
not powered and no output will be generated. Pressing the "Standby" button repeatedly will not have
any effect.

The "Interlock Reset" has the effect of powering up all of the channels and resetting any tripped
interlocks. After powering up, channels require a few hundred milliseconds to fully enable, so there
is a short delay after the button press before outputs fully enable. The "Interlock Reset" button
may be pressed while all channels are already powered to reset any tripped interlocks to re-enable
RF output.

Each output channel is composed of 3 LEDs - one red, one yellow/orange, and one green. If no RF
module is installed for a given channel, no LEDs on the channel will be illuminated. If an RF
channel is detected, at least one of the LEDs will illuminate.

The red LED is illuminated if a channel fault is detected. A fault cannot be cleared by the
"Interlock Reset" button, the channel is powered down, and the channel cannot be used. The only way
to clear this type of error is to power cycle Booster. The red LED nominally indicates a fault at a
hardware level that likely requires further investigation. Any indication of the red LED should
warrant further investigation of the device.

The yellow LED and the green LED are used to indicate the status of an operational RF amplification
channel. When the yellow LED is illuminated, the RF input is disabled on the channel. When a
channel is in standby or powered down, the yellow LED will illuminate without the green LED.

The green LED indicates that the RF channel is powered.  If only the green LED is illuminated,
the channel is operational and outputting normally. If the green LED and the yellow LED are both
illuminated, this indicates that an RF channel interlock has tripped. The
channel may be reset from this state by pressing the "Interlock Reset" button. 

The ethernet port contains a green and an orange/yellow LED. The yellow LED illuminates when Booster
has successfully connected with an ethernet switch. The green LED will flash whenever there is
ethernet traffic detected.

## USB Port

The USB port on booster enumerates as a serial port and can be opened with any terminal emulation
program (e.g. Pyserial's miniterm, TeraTerm, picoterm, putty, or your desired serial port reader). The USB
port serves two purposes:
* Logs are generated out the USB port
* Basic network and MQTT configuration can be completed

No control of the RF channels is exposed over the USB serial port. Channels may only be controlled
over MQTT or through the front-panel buttons.

When connecting to the USB port, a help menu is displayed to the user to outline what can be
configured over the USB port. Any configuration made over the USB port will not take effect
immediately - in order for new configurations to apply, booster must be reset.

The USB port allows for configuration of:
* The MQTT ID of Booster
* The MQTT broker IP address
* The ethernet netmask
* The ethernet gateway (if connected to the internet)
* The IP address of booster

Additionally, the USB port allows the user to:
* Read the MAC address of Booster
* Reset booster
* Enter DFU mode for upgrading firmware

## Ethernet

Booster uses MQTT for telemetry and control of RF channels. All booster MQTT topics are prefixed
with the booster ID, which defaults to a combination of `booster` and the MAC address of the device.
Telemetry is generated on the `<ID>/ch<N>` topics, where N is an integer from 0 to 7. Telemetry is
only reported for connected channels and is nominally transmitted at a rate of 2Hz. Telemetry is
transmitted in human-readable JSON format for logging purposes.

Booster also supports a control interface over MQTT using the following topics:
* `<ID>/channel/state` - Used to configure the power state of an RF channel
* `<ID>/channel/tune` - Used to tune the drain current of an RF channel to tune the amplification
* `<ID>/channel/read` - Used to read Booster properties, such as the interlock thresholds or the
  power transforms
* `<ID>/channel/write` - Used to write Booster properties, such as the interlock thresholds or the
  power transforms

When a valid message is received on any of these channels, Booster will process the message and
respond to the `ResponseTopic` property provided with the MQTT message. If no `ResponseTopic`
property is provided, Booster will respond by default to `<ID>/log`.

For a reference implementation of the API to control booster over MQTT, refer to `booster.py`

### Special Note on Booster properties

When reading or writing Booster properties of `<ID>/channel/read` or `<ID>/channel/write`, the
serialized property is encoded in JSON format, but all double quotes (") are replaced with a single
quote (').


## Configuring MQTT

Booster utilizes MQTT for much of the configuration, control, and telemetry functionality. In order
to use MQTT, a broker must be run on a central machine for Booster to connect to. The broker must accept connections without authentication and must support MQTT version 5.

The recommended MQTT broker to work with booster is the Mosquitto broker. This can be installed in
Ubuntu by running: `sudo apt-get install mosquitto`. Note that `mosquitto` in Debian stable/buster does not yet support MQTT version 5.

Because Booster currently works with static IP addresses, it is recommended to operate Booster on a
closed, internal network. As such, it is likely that you may need to configure IP routing on the
broker computer so that communication with Booster can succeed.

Given an ethernet interface on the broker machine called `<ETHERNET_INTERFACE>`, the broker can be
configured with an IP address of 10.0.0.2 on the local machine and told to communicate with a
booster using an IP address of 10.0.0.X on the provided interface using the following commands:

```
sudo ip route add 10.0.0.0/24 dev <ETHERNET_INTERFACE>
sudo ip address add 10.0.0.2 dev <ETHERNET_INTERFACE>
```

Note that you will likely also need to run mosquitto with a special configuration file. Contents of
the configuration file `mosquitto.conf` may look as follows:
```
bind_interface <ETHERNET_INTERFACE>
```

Mosquitto would then be started by calling `moqsuitto -c mosquitto.conf` to run mosquitto on the
provided ethernet interface.

## Configuring Booster

Once an MQTT broker is connected and communicating with Booster, the `booster.py` Python script can
be used to configure Booster RF channels.

To use `booster.py`, first install the prerequisites:
```
python -m pip install gmqtt
```

To enable a specific RF channel:
```
python booster.py --booster-id <ID> <CHANNEL> --enable
```

To disable a specific RF channel:
```
python booster.py --booster-id <ID> <CHANNEL> --disable
```

To configure interlock thresholds for a channel:
```
python booster.py --booster-id <ID> <CHANNEL> --thresholds <OUTPUT_DBM> <REFLECTED_DBM>
```

To tune the bias current of an RF channel to a desired drain current:
```
python booster.py --booster-id <ID> <CHANNEL> --bias <DRAIN_CURRENT>
```

Once a channel is configured as desired, the configuration can be stored permanently in Booster
using:
```
python booster.py --booster-id <ID> <CHANNEL> --save
```

When settings are saved in booster, the current channel configuration will be the default state of
the channel when Booster boots. Note that saving channel settings overwrites any existing channel configuration and calibrations including those from the old legacy firmware. The legacy firmware settings are incompatible.


# DFU Instructions

**Prerequisites**
* Ensure `dfu-util` is installed. On Ubuntu, install it from `apt` using `sudo apt-get install
dfu-util`
* If building your own firmware, install `cargo-binutils`: `cargo install cargo-binutils`

The following instructions describe the process of uploading a new firmware image over the DFU
Bootloader USB interface.

1. Generate the firmware image: `cargo build`
    - Note: You may append `--release` to build the firmware with more optimization and less debugging information.
    - Note: You may also use the latest [pre-built](https://github.com/quartiq/booster/releases) assets instead of building firmware.

1. Generate the binary file for your firmware build: `cargo objcopy -- -O binary booster.bin`
    - Note: If you built with `--release`, replace `debug` with `release` in the above command.

1. Reset Booster into DFU mode:
    - Insert a pin into the DFU Bootloader hole to press the DFU button
    - While the DFU button is pressed, power cycle booster by turning off the power switch for at
    least 10 seconds and then turn the power switch on.

1. Verify Booster is in DFU mode: `dfu-util -l` should show 4 entries beginning with `Found DFU: [0483:df11]`
1. Upload the DFU file to Booster:
```
dfu-util -a 0 -s 0x08000000:leave --download booster.bin
```
