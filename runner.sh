#!/usr/bin/bash -e

# Start up OpenOCD
openocd -l openocd.log -f openocd.cfg &

# Start GDB
gdb-multiarch -q -x openocd.gdb $1

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
