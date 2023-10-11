//! Booster NGFW Application
use super::{
    hardware::{platform, UsbBus},
    BoosterSettings,
};
use bbqueue::BBBuffer;
use usbd_serial::UsbError;

use core::fmt::Write;

/// A static-scope BBqueue for handling serial output buffering.
static OUTPUT_BUFFER: BBBuffer<1024> = BBBuffer::new();

const ROOT_MENU: menu::Menu<Context> = menu::Menu {
    label: "root",
    items: &[
        &menu::Item {
            command: "reset",
            help: Some("Resets the device"),
            item_type: menu::ItemType::Callback {
                function: handle_reset,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "dfu",
            help: Some("Resets the device into DFU mode for firmware upgrade"),
            item_type: menu::ItemType::Callback {
                function: handle_dfu,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "service",
            help: Some("Read the device service information. Service information clears once read"),
            item_type: menu::ItemType::Callback {
                function: handle_service,
                parameters: &[],
            },
        },
        &menu::Item {
            command: "read",
            help: Some("Read a property from the device"),
            item_type: menu::ItemType::Callback {
                function: handle_property_read,
                parameters: &[menu::Parameter::Optional {
                    parameter_name: "property",
                    help: Some(
                        "The name of the property to read.

Note:
* If a property is not specified, all properties are read.

Available Properties:
* id: The MQTT ID of the device
* mac: The MAC address of the device
* broker-address: The MQTT broker IP address
* gateway: The internet gateway address (Unused when DHCP is enabled)
* ip-address: The static IP address of Booster (0.0.0.0 indicates DHCP usage)
* netmask: The netmask to use when DHCP is disabled
* fan: The duty cycle of fans when channels are enabled",
                    ),
                }],
            },
        },
        &menu::Item {
            command: "write",
            help: Some(
                "Updates the value of a property on the device.

Notes:
* Netmasks must contain only leading data. E.g. 255.255.0.0
* Fan speeds specify the default fan speed duty cycle, and is specified as [0, 1.0]
* MQTT client IDs must be 23 or less ASCII characters.

Examples:
    # Use DHCP for IP address allocation
    write ip-address 0.0.0.0

    # Set fans to 45% duty cycle
    write fan 0.45

    # Update the Booster MQTT ID
    write id my-booster
",
            ),
            item_type: menu::ItemType::Callback {
                function: handle_property_write,
                parameters: &[
                    menu::Parameter::Mandatory {
                        parameter_name: "property",
                        help: Some(
                            "The name of the property to write.

May be one of:
* [broker-address, gateway, ip-address, netmask] <IP>
* id <ID>
* fan <DUTY>
",
                        ),
                    },
                    menu::Parameter::Mandatory {
                        parameter_name: "value",
                        help: Some(
                            "Specifies the value to be written to the property.

The format of the value depends on the property to be written.

Examples:
* <IP>: 192.168.1.1
* <ID>: my-booster
* <DUTY>: 0.45
",
                        ),
                    },
                ],
            },
        },
    ],
    entry: None,
    exit: None,
};

struct OutputBuffer(bbqueue::Producer<'static, 1024>);

impl OutputBuffer {
    pub fn write(&mut self, data: &[u8]) {
        // If we overflow the output buffer, allow the write to be silently truncated. The issue
        // will likely be cleared up as data is processed.
        let Ok(mut grant) = self
            .0
            .grant_exact(data.len() + data.iter().filter(|&x| *x == b'\n').count())
            .or_else(|_| self.0.grant_max_remaining(data.len()))
        else {
            return;
        };

        let buf = grant.buf();
        let mut buf_iter = buf.iter_mut();

        // Copy the data into the outbound buffer. For each Newline encounteded, append a
        // carriage return to support formating in raw terminals such as Putty.
        for val in data.iter() {
            if let Some(pos) = buf_iter.next() {
                *pos = *val;
            }

            if *val == b'\n' {
                if let Some(pos) = buf_iter.next() {
                    *pos = b'\r';
                }
            }
        }

        let length = {
            // Count returns the remaining number of positions, which is nominally zero.
            // The length of the amount of data we pushed into the iterator is thus the
            // original buffer length minus the remaining items.
            let remainder = buf_iter.count();
            buf.len() - remainder
        };
        grant.commit(length);
    }
}

impl core::fmt::Write for OutputBuffer {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

struct Context {
    output_buffer: OutputBuffer,
    metadata: &'static crate::hardware::metadata::ApplicationMetadata,
    settings: BoosterSettings,
}

impl Context {
    pub fn write(&mut self, data: &[u8]) {
        self.output_buffer.write(data)
    }
}

impl core::fmt::Write for Context {
    /// Write data to the serial terminal.
    ///
    /// # Note
    /// The terminal uses an internal buffer. Overflows of the output buffer are silently ignored.
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.output_buffer.write_str(s)
    }
}

fn handle_reset<'a>(
    _menu: &menu::Menu<Context>,
    _item: &menu::Item<Context>,
    _args: &[&str],
    _context: &mut Context,
) {
    // Power off all output channels and reset the MCU.
    cortex_m::interrupt::disable();
    platform::shutdown_channels();
    cortex_m::peripheral::SCB::sys_reset();
}

fn handle_dfu<'a>(
    _menu: &menu::Menu<Context>,
    _item: &menu::Item<Context>,
    _args: &[&str],
    _context: &mut Context,
) {
    cortex_m::interrupt::disable();

    // Power off all output channels and reset the MCU.
    platform::shutdown_channels();

    platform::reset_to_dfu_bootloader();
}

fn handle_service<'a>(
    _menu: &menu::Menu<Context>,
    _item: &menu::Item<Context>,
    _args: &[&str],
    context: &mut Context,
) {
    writeln!(
        context,
        "{:<20}: {} [{}]",
        "Version", context.metadata.firmware_version, context.metadata.profile,
    )
    .unwrap();
    writeln!(
        context,
        "{:<20}: {}",
        "Hardware Revision", context.metadata.hardware_version
    )
    .unwrap();
    writeln!(
        context,
        "{:<20}: {}",
        "Rustc Version", context.metadata.rust_version
    )
    .unwrap();
    writeln!(context, "{:<20}: {}", "Features", context.metadata.features).unwrap();
    writeln!(context, "{:<20}: {}", "Detected Phy", context.metadata.phy).unwrap();
    writeln!(
        context,
        "{:<20}: {}",
        "Panic Info", context.metadata.panic_info
    )
    .unwrap();
    writeln!(
        context,
        "{:<20}: {}",
        "Watchdog Detected", context.metadata.watchdog
    )
    .unwrap();

    // Use this as a mechanism for the user to "acknowledge" the service state of
    // the device. This will allow RF channels to re-enable.
    platform::clear_reset_flags();
}

fn handle_property_read<'a>(
    _menu: &menu::Menu<Context>,
    item: &menu::Item<Context>,
    args: &[&str],
    context: &mut Context,
) {
    let property = menu::argument_finder(item, args, "property").unwrap();

    let mut num_printed = 0;
    if property.map(|prop| prop == "id").unwrap_or(true) {
        writeln!(context.output_buffer, "id: {}", context.settings.id()).unwrap();
        num_printed += 1;
    }

    if property.map(|prop| prop == "mac").unwrap_or(true) {
        writeln!(context.output_buffer, "mac: {}", context.settings.mac()).unwrap();
        num_printed += 1;
    }

    if property
        .map(|prop| prop == "broker-address")
        .unwrap_or(true)
    {
        writeln!(
            context.output_buffer,
            "broker-address: {}",
            context.settings.broker()
        )
        .unwrap();
        num_printed += 1;
    }

    if property.map(|prop| prop == "gateway").unwrap_or(true) {
        writeln!(
            context.output_buffer,
            "gateway: {}",
            context.settings.broker()
        )
        .unwrap();
        num_printed += 1;
    }

    if property.map(|prop| prop == "ip-address").unwrap_or(true) {
        writeln!(
            context.output_buffer,
            "ip-address: {}",
            context.settings.ip_address()
        )
        .unwrap();
        num_printed += 1;
    }

    if property.map(|prop| prop == "netmask").unwrap_or(true) {
        writeln!(
            context.output_buffer,
            "netmask: {}",
            context.settings.netmask()
        )
        .unwrap();
        num_printed += 1;
    }

    if property.map(|prop| prop == "fan").unwrap_or(true) {
        writeln!(
            context.output_buffer,
            "fan: {:.2} %",
            context.settings.fan_speed() * 100.0
        )
        .unwrap();
        num_printed += 1;
    }

    if num_printed == 0 {
        // Note: Unwrap. If property is None, we should have printed something.
        writeln!(
            context,
            "Unknown property specified: `{}`",
            property.unwrap()
        )
        .unwrap();
    }
}

fn handle_property_write<'a>(
    _menu: &menu::Menu<Context>,
    item: &menu::Item<Context>,
    args: &[&str],
    context: &mut Context,
) {
    let value = menu::argument_finder(item, args, "value").unwrap().unwrap();
    match menu::argument_finder(item, args, "property")
        .unwrap()
        .unwrap()
    {
        "broker-address" => match value.parse() {
            Ok(value) => context.settings.set_broker(value),
            Err(e) => writeln!(context, "Invalid IP: {:?}", e).unwrap(),
        },
        "gateway" => match value.parse() {
            Ok(value) => context.settings.set_gateway(value),
            Err(e) => writeln!(context, "Invalid IP: {:?}", e).unwrap(),
        },
        "ip-address" => match value.parse() {
            Ok(value) => context.settings.set_ip_address(value),
            Err(e) => writeln!(context, "Invalid IP: {:?}", e).unwrap(),
        },
        "netmask" => match value.parse() {
            Ok(value) => context.settings.set_netmask(value),
            Err(e) => writeln!(context, "Invalid IP: {:?}", e).unwrap(),
        },
        "id" => {
            if context.settings.set_id(value).is_err() {
                writeln!(
                    context,
                    "The provided ID is invalid.
It may only be 23 characters or less and contain alphanumeric characters (or '-')"
                )
                .unwrap();
            }
        }
        "fan" => match value.parse() {
            Ok(value) => context.settings.set_fan_speed(value),
            Err(e) => writeln!(context, "{:?}", e).unwrap(),
        },
        other => {
            writeln!(context,
                "Unknown property: `{}`. Available options: [broker-address, gateway, ip-address, netmask, id, fan]", other).unwrap();
        }
    }

    // Warn the user if there are settings in memory that differ from those that are
    // currently operating.
    if context.settings.are_dirty() {
        writeln!(
            context,
            "Settings in memory may differ from currently operating settings. \
Reset the device to apply settings."
        )
        .unwrap();
    }
}

/// A serial terminal for allowing the user to interact with Booster over USB.
pub struct SerialTerminal {
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    menu: menu::Runner<'static, Context>,
    output_buffer_consumer: bbqueue::Consumer<'static, 1024>,
    prompted: bool,
}

impl SerialTerminal {
    /// Construct a terminal for interacting with the user.
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
        settings: BoosterSettings,
        metadata: &'static crate::hardware::metadata::ApplicationMetadata,
    ) -> Self {
        let (producer, consumer) = OUTPUT_BUFFER.try_split().unwrap();
        let context = Context {
            settings,
            output_buffer: OutputBuffer(producer),
            metadata,
        };

        let input_buffer = cortex_m::singleton!(: [u8; 128] = [0; 128]).unwrap();
        Self {
            usb_device,
            usb_serial,
            menu: menu::Runner::new(&ROOT_MENU, input_buffer, context),
            output_buffer_consumer: consumer,
            prompted: false,
        }
    }

    fn flush(&mut self) {
        let read = match self.output_buffer_consumer.read() {
            Ok(grant) => grant,
            Err(bbqueue::Error::InsufficientSize) => return,
            error => error.unwrap(),
        };

        match self.usb_serial.write(read.buf()) {
            Ok(count) => {
                // Release the processed bytes from the buffer.
                read.release(count);
            }
            Err(UsbError::WouldBlock) => {
                read.release(0);
            }
            Err(_) => {
                let len = read.buf().len();
                read.release(len);
            }
        };
    }

    pub fn write(&mut self, data: &[u8]) {
        self.menu.context.write(data);
    }

    /// Poll the serial terminal and process any necessary updates.
    pub fn process(&mut self) {
        self.flush();

        if !self.usb_serial.dtr() {
            self.prompted = false;
        }

        if self.usb_serial.dtr() && !self.prompted {
            self.menu.prompt(true);
            self.prompted = true;
        }

        // Update the USB serial port.
        if !self.usb_device.poll(&mut [&mut self.usb_serial]) {
            return;
        }

        // Consume data from the serial port.
        let mut buffer = [0u8; 64];
        match self.usb_serial.read(&mut buffer) {
            Ok(count) => {
                for &value in &buffer[..count] {
                    self.menu.input_byte(value);
                }
            }
            // If there's no data available, don't process anything.
            Err(UsbError::WouldBlock) => {}

            // If USB encountered an error, it's likely the port was disconnected. Reset our
            // buffers.
            Err(_) => {
                // TODO: Does this reset the menu linefeeder?
                self.menu.prompt(true);

                self.output_buffer_consumer
                    .read()
                    .map(|grant| {
                        let len = grant.buf().len();
                        grant.release(len);
                    })
                    .ok();
            }
        }
    }
}
