#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides a means of calibrating booster power transformations.
"""

import argparse
import asyncio
from booster import BoosterApi, PropertyId

async def transform_configuration(args):
    """ Configure an RF channel power transform. """
    # Establish a communication interface with Booster.
    interface = await BoosterApi.create(args.booster_id, args.broker)

    # Convert the transform to the property ID
    if args.write:
        transform = {
            'slope': args.write[0],
            'offset': args.write[1],
        }
        await interface.write_property(args.channel, PropertyId(args.transform), transform)
    else:
        transform = await interface.read_property(args.channel, PropertyId(args.transform))

    print(f'Channel {args.channel}: {args.transform} slope = {transform["slope"]}, '
          f'offset = {transform["offset"]}')

def main():
    """ Main program entry point. """

    parser = argparse.ArgumentParser(description='Read and modify booster channel properties')
    parser.add_argument('--booster-id', required=True, type=str,
                        help='The identifier of the booster to configure')
    parser.add_argument('channel', type=int, choices=list(range(8)))
    parser.add_argument('--broker', default='10.0.0.2', type=str, help='The MQTT broker address')

    parser.add_argument('transform', type=str, choices=[
        x.value for x in PropertyId if x is not PropertyId.InterlockThresholds])

    parser.add_argument('--write', type=float, nargs=2,
                        help='The new transform in the form of <slope> <intercept>')

    loop = asyncio.get_event_loop()
    loop.run_until_complete(transform_configuration(parser.parse_args()))


if __name__ == '__main__':
    main()
