#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Booster NGFW over MQTT.
"""
import asyncio
import time
import json
import enum

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
    """ Represents an action that can be taken on channel state. """
    ReadBiasCurrent = 'read-bias'
    Save = 'save'


class Booster:
    """ An asynchronous API for controlling booster using the MQTT control interface. """


    def __init__(self, client, prefix, settings_interface):
        """ Consructor.

        Args:
            client: A connected MQTT5 client.
            prefix: The prefix of the booster to control.
        """
        self.client = client
        self.prefix = prefix
        self.command_complete = asyncio.Event()
        self.settings_interface = miniconf.Miniconf(client, prefix)
        self.listener = asyncio.create_task(self._listen())
        self.response_topic = f"{prefix}/command/response"
        self.request_id = 0
        self.inflight = {}


    async def _listen(self):
        await self.client.subscribe(self.response_topic)
        LOGGER.info(f"Subscribed to {self.response_topic}")
        self.subscribed.set()
        try:
            async for message in self.client.messages:
                self._dispatch(message)
        except (asyncio.CancelledError, MqttError):
            pass
        finally:
            try:
                await self.client.unsubscribe(self.response_topic)
                LOGGER.info(f"Unsubscribed from {self.response_topic}")
            except MqttError:
                pass

    def _handle_response(self, client, topic, payload, _qos, properties):
        """ Callback function for when messages are received over MQTT.

        Args:
            client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
            qos: The quality-of-service of the message.
            properties: Any properties associated with the message.
        """
        if topic != f'{self.prefix}/command/response':
            raise Exception(f'Unknown topic: {topic}')

        # Indicate a response was received.
        request_id = int.from_bytes(properties['correlation_data'][0], 'big')
        assert len(properties['user_property']) == 1, 'Unexpected number of user properties'
        response_prop = properties['user_property'][0]
        assert response_prop[0] == 'code'
        self.inflight[request_id].set_result((response_prop[1], json.loads(payload)))
        del self.inflight[request_id]


    async def perform_action(self, action: Action, channel: str):
        """ Send a command to a booster control topic.

        Args:
            action: The action to take
            channel: The channel on which to perform the action.

        Returns:
            The received response to the action.
        """
        assert self.request_id not in self.inflight
        request_id = self.request_id
        self.request_id += 1

        message = json.dumps({
            'channel': CHANNEL[channel],
        })

        result = asyncio.get_running_loop().create_future()
        self.inflight[request_id] = result

        self.client.publish(
            f'{self.prefix}/command/{action.value}', payload=message, qos=0, retain=False,
            response_topic=f'{self.prefix}/command/response',
            correlation_data=request_id.to_bytes(4, 'big'))

        # Check the response code.
        code, response = await result
        assert code == 'Ok', f'Request failed: {response}'
        return response


    async def tune_bias(self, channel, current):
        """ Set a booster RF bias current.

        Args:
            channel: The channel index to configure.
            current: The bias current.

        Returns:
            (Vgs, Ids) where Vgs is the actual bias voltage and Ids is
            the measured RF amplifier drain current.
        """
        # Power up the channel. Wait for the channel to fully power-up before continuing.
        await self.settings_interface.set(f'/channel/{channel}/state', "Powered")
        await asyncio.sleep(0.4)

        async def set_bias(voltage):
            await self.settings_interface.set(f'/channel/{channel}/bias_voltage', voltage)
            # Sleep 100 ms for bias current to settle and for ADC to take current measurement.
            await asyncio.sleep(0.2)
            response = await self.perform_action(Action.ReadBiasCurrent, channel)
            vgs, ids = response['vgs'], response['ids']
            print(f'Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA')
            return vgs, ids

        # v_gsq from datasheet
        voltage = -2.1
        vgs_max = -0.3
        ids_max = .2

        # scan upwards in steps of 20 mV to just above target
        last_ids = 0.
        while True:
            if voltage > vgs_max:
                raise ValueError(f'Voltage out of bounds')
            vgs, ids = await set_bias(voltage)
            if ids > ids_max:
                raise ValueError(f'Ids out of range')
            if ids < last_ids - .02:
                raise ValueError(f'Foldback')
            last_ids = ids
            if ids > current:
                break
            voltage += .02

        # scan downwards in steps of 1 mV to just below target
        # Set the lower limit a few steps beyond the upper limit. We may have overshot the upper
        # limit a bit.
        lower_limit = voltage - 0.07
        while True:
            voltage -= .001
            if not lower_limit <= voltage:
                raise ValueError(f'Voltage out of bounds')
            vgs, ids = await set_bias(voltage)
            if ids > ids_max:
                raise ValueError(f'Ids out of range')
            if ids <= current:
                break

        return vgs, ids
