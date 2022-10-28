#!/bin/bash

# Title:
#   Booster hardware-in-the-loop (HITL) test script.

# Enable shell operating mode flags.
set -eux

# When only one booster is connected, we can use the discovery prefix for all activities.
PREFIX=dt/sinara/booster/+

# Set up python for testing
python3 -m venv --system-site-packages vpy
. vpy/bin/activate

# Install Miniconf utilities for configuring stabilizer.
python3 -m pip install -e py

cargo embed --release

# Test basic operation
python3 hitl/basic.py
