#!/bin/bash

# Title:
#   Booster hardware-in-the-loop (HITL) test script.

# Enable shell operating mode flags.
set -eux

function finish {
    # Always disable the mains supply to Booster using the MQTT-enabled power switch upon
    # completion. Leaving booster enabled can be annoying because of the cooling fans.
    mosquitto_pub -h mqtt.ber.quartiq.de -t cmnd/tasmota_4F64D0/POWER -m OFF
}
trap finish EXIT

# When only one booster is connected, we can use the discovery prefix for all activities.
PREFIX=dt/sinara/booster/+

# Set up python for testing
python3 -m venv --system-site-packages vpy
. vpy/bin/activate

# Install Miniconf utilities for configuring stabilizer.
python3 -m pip install --upgrade pip
python3 -m pip install -e py

# Enable the mains supply to Booster using the MQTT-enabled power switch.
mosquitto_pub -h mqtt.ber.quartiq.de -t cmnd/tasmota_4F64D0/POWER -m ON

# Give booster a moment to power up after enabling the mains supply
sleep 5

probe-rs download --chip STM32F407ZGTx --log-file /dev/null --probe 0483:3754:003C002F5632500A20313236 target/thumbv7em-none-eabihf/release/booster 
probe-rs reset --chip STM32F407ZGTx --log-file /dev/null --probe 0483:3754:003C002F5632500A20313236 --connect-under-reset

# Sleep to allow flashing, booting, DHCP, MQTT
sleep 30

# Test pinging Booster. This exercises that:
# * Booster's network is functioning as intended
# * The Booster application is operational
ping -c 5 -w 20 booster-hitl

# Test the MQTT interface.
python3 -m miniconf --discover $PREFIX /telemetry_period=5

# Test basic operation
python3 hitl/basic.py
