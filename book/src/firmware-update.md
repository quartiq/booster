# Firmware Upgrade Instructions

These instructions provide information on how to upgrade the firmware on Booster to a newer release.

**Prerequisites**
* Ensure `dfu-util` is installed. On Ubuntu, install it from `apt` using `sudo apt-get install
dfu-util`
* If building your own firmware, [`cargo-binutils`](https://github.com/rust-embedded/cargo-binutils#installation)) must be installed:
```
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

The following instructions describe the process of uploading a new firmware image over the DFU
Bootloader USB interface.

1. Generate the firmware image: `cargo build`
    - Note: You may append `--release` to build the firmware with more optimization and less debugging information.
    - Note: You may also use the latest [pre-built](https://github.com/quartiq/booster/releases) assets instead of building firmware.

1. Generate the binary file for your firmware build: `cargo objcopy -- -O binary booster.bin`
    - Note: If you built with `--release`, use the commmand: `cargo objcopy --release -- -O binary booster.bin`

1. Reset Booster into DFU mode. This can be done via the USB serial port or by doing the following:
    - Insert a pin into the DFU Bootloader hole to press the DFU button
    - While the DFU button is pressed, power cycle booster by turning off the power switch for at
    least 10 seconds and then turn the power switch on.

1. Verify Booster is in DFU mode: `dfu-util -l` should show 4 entries beginning with `Found DFU: [0483:df11]`
1. Upload the DFU file to Booster:
```
dfu-util -a 0 -s 0x08000000:leave --download booster.bin
```
