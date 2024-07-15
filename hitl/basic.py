#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Basic functional testing of Booster hardware
"""
import argparse
import asyncio
import contextlib
import sys
import time
import json
import logging

from booster import Booster
import miniconf

# The default bias current to tune to.
DEFAULT_BIAS_CURRENT = 0.05


async def periodic_check(awaitable, timeout: float):
    """ Periodically check a condition for a predefined amount of time until it evaluates true. """
    start = time.time()

    while (time.time() - start) < timeout:
        result = await awaitable()
        if result:
            return

    raise Exception('Condition did not occur within expected timeout')


@contextlib.asynccontextmanager
async def channel_on(booster, channel, initial_state='Enabled'):
    """ Context manager to ensure a channel is disabled upon exit.

    Args:
        booster: The Booster connection
        channel: The channel to configure.
        initial_state: The state to configure the channel into.
    """
    try:
        print(f'Commanding channel {channel} into {initial_state}')
        await booster.miniconf.set(f'/channel/{channel}/state', initial_state)
        yield
    finally:
        print(f'Commanding channel {channel} off')
        await booster.miniconf.set(f'/channel/{channel}/state', 'Off')


async def test_channel(booster, channel, tele_queue):
    """ Basic testing of a single RF channel.

    Args:
        booster: The Booster connection.
        channel: The channel index to test.
        tele: The received telemetry
    """
    print(f'-> Conducting self-test on channel {channel}')

    # Tune the bias current on the channel
    async with channel_on(booster, channel, 'Powered'):
        vgs, ids = await booster.tune_bias(channel, DEFAULT_BIAS_CURRENT)
        print(f'Channel {channel} bias tuning: Vgs = {vgs}, Ids = {ids}')

    # Disable the channel.
    await booster.miniconf.set(f'/channel/{channel}/state', 'Off')

    # Check that telemetry indicates channel is powered off.
    async def is_off() -> bool:
        msg = await tele_queue.__anext__()
        tlm = json.loads(msg.payload)
        return tlm['state'] == 'Off'

    await periodic_check(is_off, timeout=5)

    # Set the interlock threshold so that it won't trip.
    print('Setting output interlock threshold to 30 dB')
    await booster.miniconf.set(f'/channel/{channel}/output_interlock_threshold', 30)

    # Enable the channel, verify telemetry indicates it is now enabled.
    async with channel_on(booster, channel):
        async def is_enabled() -> bool:
            msg = await tele_queue.__anext__()
            tlm = json.loads(msg.payload)
            return tlm['state'] == 'Enabled'

        await periodic_check(is_enabled, timeout=5)

        # Lower the interlock threshold so it trips.
        print('Setting output interlock threshold to -5 dB, verifying interlock trips')
        await booster.miniconf.set(f'/channel/{channel}/output_interlock_threshold', -5.7)

        async def is_tripped() -> bool:
            msg = await tele_queue.__anext__()
            tlm = json.loads(msg.payload)
            return tlm['state'] == 'Tripped(Output)'

        # Verify the channel is now tripped.
        await periodic_check(is_tripped, timeout=5)

    print(f'Channel {channel}: PASS')
    print('')


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(description='Loopback tests for Booster HITL testing')
    parser.add_argument('--prefix', default='dt/sinara/booster/+', type=str,
                        help='The prefix of the booster to test')
    parser.add_argument("--no-discover", "-d", action="store_true",
                        help="Do not discover device prefix.")
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')
    parser.add_argument('--channels', '-c', nargs='+', help='Channels indices to test',
                        default=list(range(8)))

    args = parser.parse_args()

    async def test():
        """ The actual testing being completed. """
        async with miniconf.Client(
            args.broker,
            protocol=miniconf.MQTTv5,
            logger=logging.getLogger("aiomqtt-client"),
            queue_type=asyncio.LifoQueue,
            max_queued_incoming_messages=1,
        ) as client:
            if not args.no_discover:
                prefix, _alive = await miniconf.discover_one(client, args.prefix)
            else:
                prefix = args.prefix

            booster = Booster(client, prefix)

            # Disable configure the telemetry rate.
            await booster.miniconf.set('/telemetry_period', 1)

            # Test operation of an RF channel
            async with miniconf.Client(
                args.broker, protocol=miniconf.MQTTv5,
                logger=logging.getLogger("aiomqtt-client")
            ) as tele:
                for channel in args.channels:
                    await tele.subscribe(f"{prefix}/telemetry/ch{channel}")
                    await test_channel(booster, channel, tele.messages)
                    await tele.unsubscribe(f"{prefix}/telemetry/ch{channel}")

    loop = asyncio.get_event_loop()
    sys.exit(loop.run_until_complete(test()))


if __name__ == '__main__':
    import os
    import sys
    if sys.platform.lower() == "win32" or os.name.lower() == "nt":
        from asyncio import set_event_loop_policy, WindowsSelectorEventLoopPolicy

        set_event_loop_policy(WindowsSelectorEventLoopPolicy())

    main()
