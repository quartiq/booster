# Booster NGFW (Next-Generation Firmware) User Manual

This document is intended to provide an overview of the Booster NG firmware, which is a rewrite of
Booster firmware to align it with other projects, resolve qualify issues, and enable continued
maintenance and improvements.

# Hardware

![Booster Creotech Front Panel](assets/booster-creotech-v1.3.jpg)

Booster is an 8 channel RF power amplifier in the Sinara open hardware ecosystem.
The open hardware designs and hardware discussions are located at
https://github.com/sinara-hw/Booster/wiki.
The hardware is available from Creotech, QUARTIQ, Technosystem, and M-Labs.

# Getting Started

There are three ways to interface with the Booster application that are covered in the chapters of
this manual:
1. Pressing any of two the front-panel buttons
1. Communicating with Booster over the USB port
1. Communicating with Booster over ethernet via MQTT

# Fault Mode

When Booster encounters a software fault, it goes into a safe mode where no channels will be
enabled. To acknowledge and clear the fault, utilize the `service` command from the front panel USB
port.
