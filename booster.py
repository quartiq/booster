#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Booster NGFW over MQTT.
"""
import argparse
import asyncio
import json

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

def generate_request(**kwargs):
    """ Generate an serialized request for Booster.

    Args:
        kwargs: A list of keyword-value argument pairs to construct the message from.
    """
    return str(kwargs).replace("'", '"')


class BoosterApi:
    """ An asynchronous API for controlling booster using the MQTT control interface. """

    @classmethod
    async def create(cls):
        """ Create a connection to MQTT for communication with booster. """
        client = MqttClient(client_id='')
        await client.connect("10.0.0.2")
        client.subscribe("booster/control")
        return cls(client)


    def __init__(self, client):
        """ Consructor.

        Args:
            client: A connected MQTT5 client.
        """
        self.client = client
        self.command_complete = asyncio.Event()
        self.client.on_message = self._handle_response
        self.response = None


    def _handle_response(self, client, topic, payload, qos, properties):
        """ Callback function for when messages are received over MQTT.

        Args:
            client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
            qos: The quality-of-service of the message.
            properties: Any properties associated with the message.
        """
        if topic != 'booster/control':
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
            topic, payload=message, qos=0, retain=False, response_topic='booster/control')
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
        request = generate_request(channel=CHANNEL[channel])
        await self.command("booster/enable", request)


    async def disable_channel(self, channel):
        """ Disable a booster channel.

        Args:
            channel: The channel index to disble.
        """
        request = generate_request(channel=CHANNEL[channel])
        await self.command("booster/disable", request)


    async def set_channel_thresholds(self, channel, output_power, reflected_power):
        """ Configure booster channel thresholds.

        Args:
            channel: The channel index to configure.
            output_power: The desired output power interlock threshold in dBm.
            reflected_power: The desired reflected power interlock threshold in dBm.
        """
        request = generate_request(channel=CHANNEL[channel],
                                   output_power=float(output_power),
                                   reflected_power=float(reflected_power))
        await self.command("booster/thresholds", request)


    async def tune_bias(self, channel, desired_current):
        """ Tune a booster RF bias current.

        Args:
            channel: The channel index to configure.
            desired_current: The desired booster output current.

        Returns:
            (Vgs, Ids) where Vgs is the tuned bias voltage and Ids is the RF amplifier drain
            current.
        """
        request = generate_request(channel=CHANNEL[channel],
                                   current=float(desired_current))
        response = await self.command("booster/tune", request)

        return (response['vgs'], response['ids'])


STOP = asyncio.Event()

async def channel_configuration(args):
    """ Configure an RF channel. """

    # Establish a communication interface with Booster.
    interface = await BoosterApi.create()

    if args.enable:
        await interface.enable_channel(args.channel)

    if args.thresholds:
        await interface.set_channel_thresholds(args.channel, args.thresholds[0], args.thresholds[1])

    if args.bias:
        await interface.tune_bias(args.channel, args.bias)

    if args.disable:
        await interface.disable_channel(args.channel)

    await interface.client.disconnect()


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser()
    parser.add_argument('channel', type=int, choices=list(range(8)))
    parser.add_argument('--bias', type=float,
                        help='Tune the RF channel bias current to the provided value')
    parser.add_argument('--enable', action='store_true', help='Enable the RF channel')
    parser.add_argument('--disable', action='store_true', help='Disable the RF channel')
    parser.add_argument('--thresholds', type=float, nargs=2,
                        help='The interlock thresholds in the following order: '
                             '<output> <reflected>')

    loop = asyncio.get_event_loop()
    loop.run_until_complete(channel_configuration(parser.parse_args()))


if __name__ == '__main__':
    main()