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

from miniconf import Miniconf

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
    ReadBiasCurrent = 'ReadBiasCurrent'
    Save = 'Save'


def generate_request(**kwargs):
    """ Generate an serialized request for Booster.

    Args:
        kwargs: A list of keyword-value argument pairs to construct the message from.
    """
    return json.dumps(kwargs)


# A dictionary of all the available commands, their number of arguments, argument type, and help
# information about the command.
CMDS = {
    'save': {
        'nargs': 0,
        'help': 'Save channel configuration',
    },
    'tune': {
        'nargs': 1,
        'type': float,
        'help': 'Tune the channel RF drain current to the specified amps',
    },
}

def parse_command(entry):
    """ Parse a command string into a command and arguments. """
    try:
        pieces = entry.split('=')
        assert 1 <= len(pieces) <= 2, 'Invalid command format'

        cmd = pieces[0]
        tail = pieces[1] if len(pieces) > 1 else None
        args = tail.split(',') if tail else []

        assert cmd in CMDS, f'Unknown command specified: {cmd}'
        assert CMDS[cmd]['nargs'] == len(args), \
            f'Invalid args specified. Expected {CMDS[cmd]["nargs"]}, but found {len(args)}'
        return (cmd, [CMDS[cmd]['type'](x) for x in args])

    except Exception as exception:
        raise Exception(f'Failed to parse command "{entry}": {exception}')


class BoosterApi:
    """ An asynchronous API for controlling booster using the MQTT control interface. """

    @classmethod
    async def create(cls, booster_id, broker):
        """ Create a connection to MQTT for communication with booster. """
        settings_interface = await Miniconf.create(f'dt/sinara/booster/{booster_id}', broker)
        client = MqttClient(client_id='')
        await client.connect(broker)
        client.subscribe(f"dt/sinara/booster/{booster_id}/control/response")
        return cls(client, booster_id, settings_interface)


    def __init__(self, client, booster_id, settings_interface):
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
        self.settings_interface = settings_interface


    def _handle_response(self, client, topic, payload, *_args, **_kwargs):
        """ Callback function for when messages are received over MQTT.

        Args:
            client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
            qos: The quality-of-service of the message.
            properties: Any properties associated with the message.
        """
        if topic != f'dt/sinara/booster/{self.booster_id}/control/response':
            raise Exception(f'Unknown topic: {topic}')

        # Indicate a response was received.
        self.response = json.loads(payload)
        self.command_complete.set()


    async def perform_action(self, action: Action, channel: str):
        """ Send a command to a booster control topic.

        Args:
            action: The action to take
            channel: The channel on which to perform the action.

        Returns:
            The received response to the action.
        """
        self.command_complete.clear()
        message = generate_request(channel=CHANNEL[channel], action=action.value)
        self.client.publish(
            f'dt/sinara/booster/{self.booster_id}/control', payload=message, qos=0, retain=False,
            response_topic=f'dt/sinara/booster/{self.booster_id}/control/response')
        await self.command_complete.wait()

        # Check the response code.
        assert self.response['code'] == 200, f'Request failed: {self.response}'
        response = self.response
        self.response = None
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
        await self.settings_interface.command(f'channel/{channel}/output_disable', True, retain=False)
        await self.settings_interface.command(f'channel/{channel}/enabled', True, retain=False)
        await asyncio.sleep(0.4)

        async def set_bias(voltage):
            await self.settings_interface.command(f'channel/{channel}/bias_voltage',
                                                  voltage, retain=False)
            response = await self.perform_action(Action.ReadBiasCurrent, channel)
            response = json.loads(response['msg'])
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


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description='Modify booster RF channel configuration')
    parser.add_argument('--booster-id', required=True, type=str,
                        help='The identifier of the booster to configure')
    parser.add_argument('--channel', required=True, type=int, choices=range(8),
                        help='The RF channel index to control')
    parser.add_argument('--broker', default='10.0.0.2', type=str, help='The MQTT broker address')

    command_help = 'Individual commands. Options:\n'
    for cmd, info in CMDS.items():
        line = f'{cmd}'
        if info['nargs'] == 1:
            line += '=x'
        if info['nargs'] == 2:
            line += '=x,y'

        command_help += f'{line:<30} | {info["help"]}\n'

    parser.add_argument('commands', nargs='+', help=command_help)

    async def channel_configuration(args):
        """ Configure an RF channel. """

        # Establish a communication interface with Booster.
        interface = await BoosterApi.create(args.booster_id, args.broker)

        for command in args.commands:
            command, cmd_args = parse_command(command)
            if command == 'save':
                await interface.perform_action(Action.Save, args.channel)
                print(f'Channel {args.channel} configuration saved')
            elif command == 'tune':
                vgs, ids = await interface.tune_bias(args.channel, cmd_args[0])
                print(f'Channel {args.channel}: Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA')

    loop = asyncio.get_event_loop()
    loop.run_until_complete(channel_configuration(parser.parse_args()))


if __name__ == '__main__':
    main()
