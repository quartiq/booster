# Booster Project Plan

This document outlines the initial software development plan for updated Booster firmware. Booster
is a multichannel RF power amplifier supporting configurable power thresholds and amplification
levels.

This plan includes implementation details that may change during the development.
Software will be developed using Rust and the Real-Time Interrupt-driven Concurrency (RTIC)
framework.

## Features

Booster firmware will support both a USB and ethernet interface for control and configuration.

The ethernet interface will employ an MQTT 5.0 client for broadcasting telemetry and accepting
control commands.

The USB interface will be presented as a simple virtual COM port (USB-CDC). This interface is
intended to provide a basic logging interface as well as basic functions to set and query the
configured network settings.

## Functionality Overview

Booster consists of up to 8 RF channel inputs that are connected via GPIO pins as well as a single
I2C bus that is split to all channels. Channels may or may not be present during run-time.

Booster also provides a minimal visual user interface composed of a single red, green, and yellow
LED for each of the 8 output channels. There are also two user push-buttons on the front display.

### Booster Channels
Each channel is composed of two configurable hardware interlock thresholds. When either interlock is
tripped, the corresponding output channel is disabled. Channels may be re-enabled by button press or network by
disabling and re-enabling the channel.

There are two interlocks per channel:
* A reflected power interlock
* A forward power interlock

There are RF power measurement devices to measure both forward and reflected power. These measured
signals are then compared against an internally-generated analog reference to trip the interlock.
References are generated independently for both forward and reflected power.

Each channel also provides a means to measure the input power signal as well as the output channel
temperature.

All channels expose an EEPROM with a EUI-48 identifier.

The booster main-board also supports the following control/measurements for each channel:
* Analog bias voltage control for adjusting the channel gain
* Voltage measurement of the 5V main power rail provided to the channels
* Current measurement of the 5V pre-amp power rail provided to the channels.
* Current measurement of the 28V rail provided to the channels.
* Output overdrive interlock trip indication
* Reflected power overdrive interlock trip indication
* Channel output enable/disable
* Channel power-up / power-down
* Analog voltage indicating transmitted power
* Analog voltage indicating reflected power
* Configurable channel alert notification

### User Interface

Planned LED indications:
* GREEN - Channel is present and output is enabled.
* YELLOW - Channel interlocks tripped.
* RED - Channel is not detected, has experienced an over-temperature condition, or has experienced
  an over-current condition.

Push button assignments (TBD):
* PB1 (short press) - Interlock Reset (reset any enabled channels)
* PB2 (short press) - Standby (disable all channels)

## Software Overview

The final firmware of Booster will be composed of RTIC tasks:
* Priority 2 (Highest) - Channel scan task
* Priority 1 (Mid) - Channel telemetry task
* Priority 0 (Lowest) - Idle task, Button Task

The channel scan task will be invoked at a periodic interval of 500ms (TBD). This task will measure
the current state of output channels.  This task will be fast enough such that the user will not
experience any delay when using the ethernet or USB-based interfaces. After channel temperature
measurements have been conducted, this task will update the speed of the cooling fans as necessary.

The telemetry task will take periodic measurements of the channel state and report them over an MQTT
telemetry interface.

The idle task will be composed of servicing the network stack and MQTT ethernet interface as well as
the USB terminal interface. When neither of these interfaces requires servicing, the device will
enter a low-power mode until the next task is invoked.

The button task periodically polls the button states to debounce them and update the channel states
as required.

### Dynamic Channel Connections
Note that RF channels may or may not be connected to Booster at any given point. Firmware is
responsible for detecting a disconnected channel and handling this accordingly. Channel states will
be determined at startup. Channels will not be dynamically connectable/disconnectable during
operation.

### Development Plan

Development has been broken into a number of steps outlined below:

1. Interlock threshold configuration
1. Channel device driver development
1. Control and management of RF channels
1. Display of information to the user interface
1. Implementation of an MQTT TCP-based networking interface
1. Channel output power configuration
1. USB serial terminal interface
1. Channel fan control
1. Booster settings and configuration storage
1. DFU support

#### Interlock threshold configuration

In order to prevent damage to the device during development, the first task in development will
involve configuring the channel interlock thresholds to a safe value. This must be done to prevent
damage to the RF channels due to excessive reflected output power.

This task involves writing a device driver for the I2C mux as well as the AD5627 reference
generator.

#### Channel Device Drivers

The next step in development will involve writing drivers for controlling the various peripheral
devices on each output channel over I2C. These drivers are necessary for implementing the underlying
measurements and control of each output channel.

#### Control and Management of RF Channels

Once all device drivers are implemented (or as each driver individually is added), they can be added
to the control structure for each RF channel. The application will need to be able to individually
control and measure a single channel at a time. Incorporating all of the device drivers for a single
channel into a single structure will allow for the channel to provide meaningful information outside
of analog voltages and trip-states. The channel management and control structure models the paradigm
of Object-Oriented Programming (OOP) such that the channel can be measured or controlled directly in
an understandable fashion.

This is the heart of the Booster firmware, so it should be completed earliest in the development
process.

This task will involve implementing the high-priority channel measurement task.

Exposing the management of individual channels is not incorporated in this step as there is no
external interface available yet.

#### Control of User Interface

Once channel management and control is in place, the user interface will be implemented such that
channel measurements can be indicated to the user via the LED indications.

This task will also implement control of the two user push buttons to allow a reset and re-enable of
all output channels.

This task does not include implementation of the USB or networking interface.

#### MQTT Networking Interface

The next development step that will be taken will expose an MQTT networking interface for telemetry
and control of Booster. This will expose all channel status and measurements as published topics and
will allow the user to enable, reset, or disable output channels. It will also allow the user to
calibrate the output bias to tune the output power of a specific channel. Finally, the MQTT
interface will also allow the user to configure output and reflected power interlock thresholds.

#### Channel Output Power Configuration

Once an MQTT interface is exposed, an MQTT client program will be created such that the user may
specify a desired output current for a specific channel and the client will adjust the output bias
control for a channel until the desired output current is achieved.

#### USB Serial Terminal

Booster will also allow the user to configure the IP address of the device and query information
over the USB interface. This interface is also intended to be used for generating human-readable log
outputs. Currently, the only plan for control over the USB interface is to allow configuration of
the Booster network interface to facilitate the usage of MQTT.

#### Fan Control

Booster will need to manage the temperature of output channels to prevent equipment damage. To
achieve this, there are a number of chassis-mounted fans that Booster will control using a PID
algorithm to modulate fan speed. Fan speed adjustmenets and status will be reported over the
telemetry interface. The high-priority task will be updated to incorporate adjustments to fan speed
based upon measured channel temperature.

#### Persistent Settings

Booster will contain a number of settings that should be persisted between power cycles. Examples of
such settings include:
* Channel output bias set point
* Channel interlock thresholds
* Channel EUI-48 indication
* Configured IP address
* Configured MQTT broker (TBD)

These settings will need to be stored either main-board or RF-channel EEPROM and loaded upon device
initialization. Note that configuration settings should also be loaded whenever an RF channel is
detected as newly connected to Booster.

#### DFU Support

Booster firmware will need to support field-updates. To accomplish this, the built-in STM32 DFU will
be used to allow in-field programming of new binaries. The final step of booster development will be
exposing a means of rebooting firmware into DFU mode via the USB interface to allow for remote
firmware updates.

This task will also incorporate documentation for the steps needed to update booster firmware.
