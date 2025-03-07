#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Booster NGFW over MQTT.
"""
import argparse
import asyncio
import logging

from . import Booster, Action
import miniconf

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
    "calibrate": {
        "nargs": 1,
        "type": str,
        "help": "Calibrate a transform (input, output, or reflected)",
    }
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


class CommaSeparatedChannelList(argparse.Action):
    def __call__(self, parser, namespace, values, option_string=None):
        channels = [int(channel) for channel in values.split(',')]
        for channel in channels:
            self._validate_channel(parser, channel)
        setattr(
            namespace,
            self.dest,
            channels
        )

    def _validate_channel(self, parser, channel):
        if channel not in range(8):
            parser.error('channel must be between 0 and 7')


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
        default=[0, 1, 2, 3, 4, 5, 6, 7],
        action=CommaSeparatedChannelList,
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
                for channel in args.channel:
                    if command == "save":
                        await booster.perform_action(Action.Save, channel)
                        print(f"Channel {channel} configuration saved")
                    elif command == "tune":
                        vgs, ids = await booster.tune_bias(channel, cmd_args[0])
                        print(
                            f"Channel {channel}: Vgs = {vgs:.3f} V, Ids = {ids * 1000:.2f} mA"
                        )
                    elif command == "calibrate":
                        await booster.calibrate(channel, cmd_args[0])

    asyncio.run(channel_configuration(parser.parse_args()))


if __name__ == "__main__":
    import os
    import sys

    if sys.platform.lower() == "win32" or os.name.lower() == "nt":
        from asyncio import set_event_loop_policy, WindowsSelectorEventLoopPolicy

        set_event_loop_policy(WindowsSelectorEventLoopPolicy())

    main()
