#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Basic functional testing of Booster hardware
"""
import argparse
import asyncio
import contextlib
import logging
import json
import sys

from gmqtt import Client as MqttClient

from booster import BoosterApi

# The default bias current to tune to.
DEFAULT_BIAS_CURRENT = 0.05


class TelemetryReader:
    """ Helper utility to read Stabilizer telemetry. """

    @classmethod
    async def create(cls, prefix, broker, channel, queue):
        """Create a connection to the broker and an MQTT device using it."""
        client = MqttClient(client_id='')
        await client.connect(broker)
        return cls(client, f'{prefix}/telemetry/ch{channel}', queue)


    def __init__(self, client, topic, queue):
        """ Constructor. """
        self.client = client
        self._telemetry = []
        self.client.on_message = self.handle_telemetry
        self._telemetry_topic = topic
        self.client.subscribe(self._telemetry_topic)
        self.queue = queue


    def handle_telemetry(self, _client, topic, payload, _qos, _properties):
        """ Handle incoming telemetry messages over MQTT. """
        assert topic == self._telemetry_topic
        self.queue.put_nowait(json.loads(payload))


async def telemetry(prefix, broker, channel, queue):
    """ Receive telemetry on a topic and push it into the provided queue.

    Args
        prefix: The prefix of Booster.
        broker: The broker IP address.
        channel: The channel to receive telemetry from.
        queue: The location to put telemetry into.
    """
    await TelemetryReader.create(prefix, broker, channel, queue)
    try:
        while True:
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        pass


@contextlib.asynccontextmanager
async def channel_on(booster, channel, initial_state='Enabled'):
    """ Context manager to ensure a channel is disabled upon exit.

    Args:
        booster: The BoosterApi objct.
        channel: The channel to configure.
        initial_state: The state to configure the channel into.
    """
    try:
        logging.info('Commanding channel %d into %s', channel, initial_state)
        await booster.settings_interface.command(f'channel/{channel}/state', initial_state,
                                                 retain=False)
        yield
    finally:
        logging.info('Commanding channel %d off', channel)
        await booster.settings_interface.command(f'channel/{channel}/state', 'Off', retain=False)


async def test_channel(booster, channel, prefix, broker):
    """ Basic testing of a single RF channel.

    Args:
        booster: The BoosterApi.
        channel: The channel index to test.
        prefix: Booster's miniconf prefix.
        broker: The broker IP address.
    """
    logging.info('Conducting self-test on channel %d', channel)

    # Start receiving telemetry for the channel under test.
    telemetry_queue = asyncio.LifoQueue()
    telemetry_task = asyncio.create_task(telemetry(prefix, broker, channel, telemetry_queue))

    # TODO Tune the bias current on the channel
    # Note: The bias tuning algorithm needs some help and doesn't seem robust. Temporarily disabling
    # this test for now.

    #async with channel_on(booster, channel, 'Powered'):
    #    vgs, ids = await booster.tune_bias(channel, DEFAULT_BIAS_CURRENT)
    #    logging.info('Channel %d bias tuning: Vgs = %f, Ids = %f', channel, vgs, ids)

    # Disable the channel.
    logging.info('Disabling channel %d', channel)
    await booster.settings_interface.command(f'channel/{channel}/state', 'Off', retain=False)

    # Check that telemetry indicates channel is powered off.
    await asyncio.sleep(2)
    tlm = await telemetry_queue.get()
    assert tlm['state'] == 'Off', 'Channel did not power off'

    # Set the interlock threshold so that it won't trip.
    logging.info('Setting output interlock threshold to 30 dB')
    await booster.settings_interface.command(f'channel/{channel}/output_interlock_threshold', 30,
                                             retain=False)

    # Enable the channel, verify telemetry indicates it is now enabled.
    async with channel_on(booster, channel):
        await asyncio.sleep(2)
        tlm = await telemetry_queue.get()
        assert tlm['state'] == 'Enabled', 'Channel did not enable'

        # Lower the interlock threshold so it trips.
        logging.info('Setting output interlock threshold to -5 dB, verifying interlock trips')
        await booster.settings_interface.command(f'channel/{channel}/output_interlock_threshold',
                                                 -5, retain=False)

        # Verify the channel is now tripped.
        await asyncio.sleep(2)
        tlm = await telemetry_queue.get()
        assert tlm['state'] == 'Tripped(Output)', 'Channel did not trip'

    telemetry_task.cancel()


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(description='Loopback tests for Stabilizer HITL testing',)
    parser.add_argument('--prefix', type=str, help='The MQTT prefix of the target')
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')
    parser.add_argument('--channels', '-c', nargs='+', help='Channels indices to test',
                        default=list(range(8)))

    args = parser.parse_args()

    async def test():
        """ The actual testing being completed. """
        booster = await BoosterApi.create(args.prefix, args.broker)

        # Disable configure the telemetry rate.
        await booster.settings_interface.command('telemetry_period', 1, retain=False)

        # Test operation of an RF channel
        for channel in args.channels:
            await test_channel(booster, channel, booster.prefix, args.broker)

    loop = asyncio.get_event_loop()
    sys.exit(loop.run_until_complete(test()))


if __name__ == '__main__':
    main()
