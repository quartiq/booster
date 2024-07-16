#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Booster NGFW over MQTT.
"""
import argparse
import asyncio

from . import Booster, Action
import miniconf
import logging


# A dictionary of all the available commands, their number of arguments, argument type, and help
# information about the command.
CMDS = {
    "save": {
        "nargs": 0,
        "help": "Save channel configuration",
    },
    "tune": {
        "nargs": 1,
        "type": float,
        "help": "Tune the channel RF drain current to the specified amps",
    },
}


def parse_command(entry):
    """Parse a command string into a command and arguments."""
    try:
        pieces = entry.split("=")
        assert 1 <= len(pieces) <= 2, "Invalid command format"

        cmd = pieces[0]
        tail = pieces[1] if len(pieces) > 1 else None
        args = tail.split(",") if tail else []

        assert cmd in CMDS, f"Unknown command specified: {cmd}"
        assert CMDS[cmd]["nargs"] == len(
            args
        ), f'Invalid args specified. Expected {CMDS[cmd]["nargs"]}, but found {len(args)}'
        return (cmd, [CMDS[cmd]["type"](x) for x in args])

    except Exception as exception:
        raise Exception(f'Failed to parse command "{entry}": {exception}')


def main():
    """Main program entry point."""
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawTextHelpFormatter,
        description="Modify booster RF channel configuration",
    )
    parser.add_argument(
        "--prefix",
        default="dt/sinara/booster/+",
        type=str,
        help="The prefix of the booster to configure",
    )
    parser.add_argument(
        "--no-discover",
        "-d",
        action="store_true",
        help="Do not discover device prefix.",
    )
    parser.add_argument(
        "--channel",
        required=True,
        type=int,
        choices=range(8),
        help="The RF channel index to control",
    )
    parser.add_argument(
        "--broker", "-b", default="mqtt", type=str, help="The MQTT broker address"
    )

    command_help = "Individual commands. Options:\n"
    for cmd, info in CMDS.items():
        line = f"{cmd}"
        if info["nargs"] == 1:
            line += "=x"
        if info["nargs"] == 2:
            line += "=x,y"

        command_help += f'{line:<30} | {info["help"]}\n'

    parser.add_argument("commands", nargs="+", help=command_help)

    async def channel_configuration(args):
        """Configure an RF channel."""
        async with miniconf.Client(
            args.broker,
            protocol=miniconf.MQTTv5,
            logger=logging.getLogger("aiomqtt-client"),
        ) as client:
            if not args.no_discover:
                prefix, _alive = await miniconf.discover_one(client, args.prefix)
            else:
                prefix = args.prefix

            # Establish a communication interface with Booster.
            booster = Booster(client, prefix)

            for command in args.commands:
                command, cmd_args = parse_command(command)
                if command == "save":
                    await booster.perform_action(Action.Save, args.channel)
                    print(f"Channel {args.channel} configuration saved")
                elif command == "tune":
                    vgs, ids = await booster.tune_bias(args.channel, cmd_args[0])
                    print(
                        f"Channel {args.channel}: Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA"
                    )

    loop = asyncio.get_event_loop()
    loop.run_until_complete(channel_configuration(parser.parse_args()))


if __name__ == "__main__":
    import os
    import sys

    if sys.platform.lower() == "win32" or os.name.lower() == "nt":
        from asyncio import set_event_loop_policy, WindowsSelectorEventLoopPolicy

        set_event_loop_policy(WindowsSelectorEventLoopPolicy())

    main()
