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
import sys

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

        return await self.miniconf._do(
            f"{self.miniconf.prefix}/command/{action.value}", payload=message
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
            # Sleep 200 ms for bias current to settle and for ADC to take current measurement.
            await asyncio.sleep(0.2)
            response = await self.perform_action(Action.ReadBiasCurrent, channel)
            response = json.loads(response[0])
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

    async def calibrate(self, channel, transform):
        """Calibrate a linear transform.

        Args:
            channel: The channel to calibrate the transform on.
            transform: The transform to calibrate (input, output, reflected)

        Returns:
            The transform
        """
        backup = {}
        for k in f"/channel/{channel}/state /telemetry_period".split():
            backup[k] = json.loads(await self.miniconf.get(k))

        try:
            print(
                f"""Calibrating the `{transform}` power detector linear transform on channel {channel}.

The transform calibration routine requires two different operating powers.
For optimal accuracy, they should support the desired operating power range
and both be at the same input signal carrier frequency representative of the
desired operating frequency range.

For each power condition, establish it, determine the true power by measuring it and
input it at the prompt. The routine determines the apparent power and computes a new
corrected transform."""
            )
            await self.miniconf.set(f"/channel/{channel}/state", "Enabled")
            await asyncio.sleep(0.4)
            await self.miniconf.set(f"/telemetry_period", 1)

            # This is merely to avoid the queue full warnings
            # and the inconvenient async-iterator-only interface of aiomqtt
            queue = asyncio.Queue(1)

            async def tele():
                async with miniconf.Client(
                    self.miniconf.client._hostname,
                    protocol=miniconf.MQTTv5,
                ) as tele:
                    topic = f"{self.miniconf.prefix}/telemetry/ch{channel}"
                    await tele.subscribe(topic)
                    try:
                        async for msg in tele.messages:
                            try:
                                queue.put_nowait(msg)
                            except asyncio.QueueFull:
                                continue
                    except asyncio.CancelledError:
                        pass
                    finally:
                        await tele.unsubscribe(topic)

            cal = []
            async with asyncio.TaskGroup() as tg:
                tele = tg.create_task(tele())
                for i in range(2):
                    while True:
                        try:
                            true = float(
                                await ainput(
                                    f"Enter current true `{transform}` power in dBm:"
                                )
                            )
                            break
                        except ValueError as e:
                            print(f"Error: {e}, try again")
                    while True:
                        try:
                            queue.get_nowait()
                        except asyncio.QueueEmpty:
                            break
                    msg = json.loads((await queue.get()).payload)
                    if (
                        msg["reflected_overdrive"]
                        or msg["output_overdrive"]
                        or msg["alert"]
                        or msg["state"] != "Enabled"
                    ):
                        raise ValueError(
                            "Channel tripped, overdriven, or in alert condition: ", msg
                        )
                    apparent = msg[f"{transform}_power"]
                    print(f"Current apparent `{transform}` power: {apparent} dBm")
                    cal.append((apparent, true))
                tele.cancel()

            current = json.loads(
                await self.miniconf.get(
                    f"/channel/{channel}/{transform}_power_transform"
                )
            )
            print(f"Current `{transform}` transform: {current}")
            (qa, pa), (qb, pb) = cal
            pm = (pa + pb) / 2
            qm = (qa + qb) / 2
            dpdq = (pa - pb) / (qa - qb)
            new = dict(
                slope=dpdq * current["slope"],
                offset=pm + dpdq * (current["offset"] - qm),
            )
            print(f"Proposed new `{transform}` transform: {new}")
            if await ainput("Enter `y` to set new transform:") == "y":
                await self.miniconf.set(
                    f"/channel/{channel}/{transform}_power_transform", new
                )
                print("Set (use `save` to write to flash memory)")
            else:
                print("Not set")

        finally:
            for k, v in backup.items():
                await self.miniconf.set(k, v)


async def ainput(string: str) -> str:
    print(string, end=" ", flush=True)
    return (await asyncio.to_thread(sys.stdin.readline)).rstrip("\n")
