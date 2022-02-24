#!/bin/bash

# Title:
#   Booster hardware-in-the-loop (HITL) test script.

# Enable shell operating mode flags.
set -eux

# Stabilizer device prefix for HITL
PREFIX=dt/sinara/booster/+
BOOSTER_IP=10.35.20.200
BROKER=10.35.20.1

# Set up python for testing
python3 -m venv --system-site-packages vpy
. vpy/bin/activate

# Install Miniconf utilities for configuring stabilizer.
python3 -m pip install -e py

cargo embed --release

# Sleep to allow flashing, booting, DHCP, MQTT
sleep 30

# Test pinging Booster. This exercises that:
# * Booster's network is functioning as intended
# * The Booster application is operational
ping -c 5 -w 20 $BOOSTER_IP

# Test the MQTT interface.
python3 -m miniconf --broker $BROKER --discover $PREFIX telemetry_period=5

# Test basic operation
python3 hitl/basic.py -b $BROKER
