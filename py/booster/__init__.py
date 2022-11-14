#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Booster NGFW over MQTT.
"""
import asyncio
import time
import json
import enum

from gmqtt import Client as MqttClient
import miniconf

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


class TelemetryReader:
    """ Helper utility to read telemetry. """

    @classmethod
    async def create(cls, prefix, broker, channel):
        """Create a connection to the broker and an MQTT device using it."""
        client = MqttClient(client_id='')
        await client.connect(broker)
        return cls(client, f'{prefix}/telemetry/ch{channel}')


    def __init__(self, client, topic):
        """ Constructor. """
        self.client = client
        self._telemetry_event = None
        self._last_telemetry = None
        self._last_telemetry_timestamp = None
        self.client.on_message = self._handle_telemetry
        self._telemetry_topic = topic
        self.client.subscribe(self._telemetry_topic)


    def _handle_telemetry(self, _client, topic, payload, _qos, _properties):
        """ Handle incoming telemetry messages over MQTT. """
        assert topic == self._telemetry_topic
        self._last_telemetry = json.loads(payload)
        self._last_telemetry_timestamp = time.time()
        if self._telemetry_event:
            self._telemetry_event.set()


    def get_latest_telemetry(self):
        """ Get the latest telemetry and the time at which it arrived. """
        return self._last_telemetry_timestamp, self._last_telemetry


    async def get_next_telemetry(self):
        """ Get the next telemetry message that arrives. """
        self._telemetry_event = asyncio.Event()
        await self._telemetry_event.wait()
        return self.get_latest_telemetry()


class BoosterApi:
    """ An asynchronous API for controlling booster using the MQTT control interface. """

    @classmethod
    async def create(cls, prefix, broker, timeout=30):
        """ Create a connection to MQTT for communication with booster.

        Args:
            prefix: An optionally-specified prefix of the device to connect to. If unspecified, a
                booster will be discovered instead.
            broker: The address of the broker.
            timeout: The maximum amount of time to discover boosters for.
        """
        # If the user did not provide a prefix, try to find one.
        if not prefix:
            devices = await miniconf.discover(broker, 'dt/sinara/booster/+', timeout)

            if not devices:
                raise Exception('No Boosters found')

            assert len(devices) == 1, \
                    f'Multiple Boosters found: {devices}. Please specify one with --prefix'

            prefix = devices.pop()

        settings_interface = await miniconf.Miniconf.create(prefix, broker)
        client = MqttClient(client_id='')
        await client.connect(broker)
        client.subscribe(f"{prefix}/command/response")
        return cls(client, prefix, settings_interface)


    def __init__(self, client, prefix, settings_interface):
        """ Consructor.

        Args:
            client: A connected MQTT5 client.
            prefix: The prefix of the booster to control.
        """
        self.client = client
        self.prefix = prefix
        self.command_complete = asyncio.Event()
        self.client.on_message = self._handle_response
        self.settings_interface = settings_interface
        self.request_id = 0
        self.inflight = {}


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
        self.inflight[request_id].set_result(json.loads(payload))
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
        response = await result
        assert response['code'] == 0, f'Request failed: {response}'
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
        await self.settings_interface.command(f'channel/{channel}/state', "Powered", retain=False)
        await asyncio.sleep(0.4)

        async def set_bias(voltage):
            await self.settings_interface.command(f'channel/{channel}/bias_voltage',
                                                  voltage, retain=False)
            # Sleep 100 ms for bias current to settle and for ADC to take current measurement.
            await asyncio.sleep(0.1)
            response = await self.perform_action(Action.ReadBiasCurrent, channel)
            response = json.loads(response['data'])
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
        vgs_max = voltage

        # scan downwards in steps of 1 mV to just below target
        while True:
            voltage -= .001
            if not vgs_max - .03 <= voltage <= vgs_max:
                raise ValueError(f'Voltage out of bounds')
            vgs, ids = await set_bias(voltage)
            if ids > ids_max:
                raise ValueError(f'Ids out of range')
            if ids <= current:
                break

        return vgs, ids
