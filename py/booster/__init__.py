#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Booster NGFW over MQTT.
"""
import asyncio
import time
import json
import enum
import logging

import miniconf
from aiomqtt import MqttError

LOGGER = logging.getLogger(__name__)

# A list of channel enumeration names. The index in the list corresponds with the channel name.
CHANNEL = [
    "Zero",
    "One",
    "Two",
    "Three",
    "Four",
    "Five",
    "Six",
    "Seven",
]


class Action(enum.Enum):
    """Represents an action that can be taken on channel state."""

    ReadBiasCurrent = "read-bias"
    Save = "save"


class Booster:
    """An asynchronous API for controlling booster using the MQTT control interface."""

    def __init__(self, client, prefix):
        """Consructor.

        Args:
            client: A connected MQTT5 client.
            prefix: The prefix of the booster to control.
        """
        self.prefix = prefix
        self.miniconf = miniconf.Miniconf(client, prefix)

    async def perform_action(self, action: Action, channel: str):
        """Send a command to a booster control topic.

        Args:
            action: The action to take
            channel: The channel on which to perform the action.

        Returns:
            The received response to the action.
        """
        message = json.dumps({"channel": CHANNEL[channel]})

        await self.miniconf._do(
            f"{self.prefix}/command/{action.value}", payload=message
        )

    async def tune_bias(self, channel, current):
        """Set a booster RF bias current.

        Args:
            channel: The channel index to configure.
            current: The bias current.

        Returns:
            (Vgs, Ids) where Vgs is the actual bias voltage and Ids is
            the measured RF amplifier drain current.
        """
        # Power up the channel. Wait for the channel to fully power-up before continuing.
        await self.miniconf.set(f"/channel/{channel}/state", "Powered")
        await asyncio.sleep(0.4)

        async def set_bias(voltage):
            await self.miniconf.set(f"/channel/{channel}/bias_voltage", voltage)
            # Sleep 100 ms for bias current to settle and for ADC to take current measurement.
            await asyncio.sleep(0.2)
            response = await self.perform_action(Action.ReadBiasCurrent, channel)
            vgs, ids = response["vgs"], response["ids"]
            print(f"Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA")
            return vgs, ids

        # v_gsq from datasheet
        voltage = -2.1
        vgs_max = -0.3
        ids_max = 0.2

        # scan upwards in steps of 20 mV to just above target
        last_ids = 0.0
        while True:
            if voltage > vgs_max:
                raise ValueError(f"Voltage out of bounds")
            vgs, ids = await set_bias(voltage)
            if ids > ids_max:
                raise ValueError(f"Ids out of range")
            if ids < last_ids - 0.02:
                raise ValueError(f"Foldback")
            last_ids = ids
            if ids > current:
                break
            voltage += 0.02

        # scan downwards in steps of 1 mV to just below target
        # Set the lower limit a few steps beyond the upper limit. We may have overshot the upper
        # limit a bit.
        lower_limit = voltage - 0.07
        while True:
            voltage -= 0.001
            if not lower_limit <= voltage:
                raise ValueError(f"Voltage out of bounds")
            vgs, ids = await set_bias(voltage)
            if ids > ids_max:
                raise ValueError(f"Ids out of range")
            if ids <= current:
                break

        return vgs, ids
