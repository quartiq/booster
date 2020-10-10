#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Booster NGFW over MQTT.
"""
import argparse
import asyncio
import enum
import json
import time

from gmqtt  import Client as MqttClient

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

class PropertyId(enum.Enum):
    """ Represents a property ID for Booster RF channels. """
    InterlockThresholds = 'InterlockThresholds'
    OutputPowerTransform = 'OutputPowerTransform'
    InputPowerTransform = 'InputPowerTransform'
    ReflectedPowerTransform = 'ReflectedPowerTransform'


class Action(enum.Enum):
    """ Represents an action that can be taken on channel state. """
    Enable = 'Enable'
    Disable = 'Disable'
    Powerup = 'Powerup'
    Save = 'Save'


def generate_request(**kwargs):
    """ Generate an serialized request for Booster.

    Args:
        kwargs: A list of keyword-value argument pairs to construct the message from.
    """
    return json.dumps(kwargs)


class BoosterApi:
    """ An asynchronous API for controlling booster using the MQTT control interface. """

    @classmethod
    async def create(cls, booster_id, broker):
        """ Create a connection to MQTT for communication with booster. """
        client = MqttClient(client_id='')
        await client.connect(broker)
        client.subscribe(f"{booster_id}/control")
        return cls(client, booster_id)


    def __init__(self, client, booster_id):
        """ Consructor.

        Args:
            client: A connected MQTT5 client.
            booster_id: The ID of the booster to control.
        """
        self.client = client
        self.booster_id = booster_id
        self.command_complete = asyncio.Event()
        self.client.on_message = self._handle_response
        self.response = None


    def _handle_response(self, client, topic, payload, *_args, **_kwargs):
        """ Callback function for when messages are received over MQTT.

        Args:
            client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
            qos: The quality-of-service of the message.
            properties: Any properties associated with the message.
        """
        if topic != f'{self.booster_id}/control':
            raise Exception(f'Unknown topic: {topic}')

        # Indicate a response was received.
        self.response = json.loads(payload)
        self.command_complete.set()


    async def command(self, topic, message):
        """ Send a command to a booster control topic.

        Args:
            topic: The topic to send the command to.
            message: The message to send to the provided topic.

        Returns:
            The received response to the command.
        """
        self.command_complete.clear()
        self.client.publish(
            f'{self.booster_id}/{topic}', payload=message, qos=0, retain=False,
            response_topic=f'{self.booster_id}/control')
        await self.command_complete.wait()

        # Check the response code.
        assert self.response['code'] == 200, f'Request failed: {self.response}'
        response = self.response
        self.response = None
        return response


    async def enable_channel(self, channel):
        """ Enable a booster channel.

        Args:
            channel: The channel index to enable.
        """
        await self._update_channel_state(channel, Action.Enable)


    async def disable_channel(self, channel):
        """ Disable a booster channel.

        Args:
            channel: The channel index to disble.
        """
        await self._update_channel_state(channel, Action.Disable)


    async def save_channel(self, channel):
        """ Save a booster channel state.

        Args:
            channel: The channel index to save.
        """
        await self._update_channel_state(channel, Action.Save)


    async def _update_channel_state(self, channel, action):
        """ Update the state of a booster RF channel.

        Args:
            channel: The channel to update
            action: The action to take on the channel.
        """
        request = generate_request(channel=CHANNEL[channel], action=action.value)
        await self.command('channel/state', request)


    async def read_property(self, channel, prop):
        """ Read a property from the provided channel.

        Args:
            channel: The channel to write a property on.
            prop_id: The ID of the property to read.

        Returns:
            A dict representing the read property.
        """
        request = generate_request(prop=prop.value, channel=CHANNEL[channel])

        response = await self.command("channel/read", request)
        return json.loads(response['data'].replace("'", '"'))


    async def write_property(self, channel, prop_id, value):
        """ Write a property on the provided channel.

        Args:
            channel: The channel to write a property on.
            prop_id: The ID of the property to write.
            value: The value to write to the property.
        """
        request = generate_request(prop=prop_id.value, channel=CHANNEL[channel],
                                   data=json.dumps(value).replace('"', "'"))

        await self.command("channel/write", request)


    async def set_bias(self, channel, voltage):
        """ Set a booster RF bias voltage.

        Args:
            channel: The channel index to configure.
            voltage: The bias voltage.

        Returns:
            (Vgs, Ids) where Vgs is the bias voltage and Ids is the RF amplifier drain
            current.
        """

        # Power up the channel. Wait 200ms for the channel to fully power-up before continuing.
        await self._update_channel_state(channel, Action.Powerup)
        time.sleep(0.4)

        request = generate_request(channel=CHANNEL[channel],
                                   voltage=voltage)
        response = await self.command("channel/bias", request)

        return (response['vgs'], response['ids'])

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
        await self._update_channel_state(channel, Action.Powerup)
        time.sleep(0.4)

        async def set(voltage):
            request = generate_request(channel=CHANNEL[channel],
                                       voltage=voltage)
            response = await self.command("channel/bias", request)
            vgs, ids = response['vgs'], response['ids']
            print(f'Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA')
            return vgs, ids

        # v_gsq from datasheet
        voltage = -2.1
        vgs_max = -0.3
        ids_max = .2

        # scan upwards in steps of 20 mV to just exceed the target
        last_ids = 0.
        while True:
            if voltage > vgs_max:
                raise ValueError(f'Voltage out of bounds')
            vgs, ids = await set(voltage)
            if ids > ids_max:
                raise ValueError(f'Ids out of range')
            if ids < last_ids - .02:
                raise ValueError(f'Foldback')
            last_ids = ids
            if ids > current:
                break
            voltage += .02
        vgs_max = voltage

        # scan downwards in steps of 1 mV to just meed the target
        while True:
            voltage -= .001
            if not vgs_max - .03 <= voltage <= vgs_max:
                raise ValueError(f'Voltage out of bounds')
            vgs, ids = await set(voltage)
            if ids > ids_max:
                raise ValueError(f'Ids out of range')
            if ids <= current:
                break

        return vgs, ids


async def channel_configuration(args):
    """ Configure an RF channel. """

    # Establish a communication interface with Booster.
    interface = await BoosterApi.create(args.booster_id, args.broker)

    if args.enable:
        await interface.enable_channel(args.channel)
        print(f'Channel {args.channel} enabled')

    if args.thresholds:
        thresholds = {
            'output': args.thresholds[0],
            'reflected': args.thresholds[1],
        }
        await interface.write_property(args.channel, PropertyId.InterlockThresholds, thresholds)
        print(f'Channel {args.channel}: Output power threshold = {args.thresholds[0]} dBm, '
              f'Reflected power interlock threshold = {args.thresholds[1]} dBm')

    if args.bias:
        vgs, ids = await interface.set_bias(args.channel, args.bias)
        print(f'Channel {args.channel}: Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA')

    if args.tune:
        vgs, ids = await interface.tune_bias(args.channel, args.tune)
        print(f'Channel {args.channel}: Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA')

    if args.disable:
        await interface.disable_channel(args.channel)
        print(f'Channel {args.channel} disabled')

    if args.save:
        await interface.save_channel(args.channel)
        print(f'Channel {args.channel} configuration saved')

    await interface.client.disconnect()


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(
        description='Modify booster RF channel configuration',
        epilog='*Note*: The positional argument should be the channel index. For example, to '
               'enable channel 0, the call would appear as `python booster.py 0 --enable`')
    parser.add_argument('--booster-id', required=True, type=str,
                        help='The identifier of the booster to configure')
    parser.add_argument('channel', type=int, choices=list(range(8)))
    parser.add_argument('--broker', default='10.0.0.2', type=str, help='The MQTT broker address')
    parser.add_argument('--bias', type=float,
                        help='Set the RF channel bias voltage to the provided value')
    parser.add_argument('--tune', type=float,
                        help='Tune the RF channel bias current to the provided value')
    parser.add_argument('--enable', action='store_true', help='Enable the RF channel')
    parser.add_argument('--disable', action='store_true', help='Disable the RF channel')
    parser.add_argument('--thresholds', type=float, nargs=2,
                        help='The interlock thresholds in the following order: '
                             '<output> <reflected>')
    parser.add_argument('--save', action='store_true', help='Save the RF channel configuration')

    loop = asyncio.get_event_loop()
    loop.run_until_complete(channel_configuration(parser.parse_args()))


if __name__ == '__main__':
    main()
