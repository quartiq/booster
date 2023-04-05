//! Booster NGFW Application
use super::{
    hardware::{platform, UsbBus},
    BoosterSettings,
};
use bbqueue::BBBuffer;
use core::convert::{TryFrom, TryInto};
use heapless::{String, Vec};
use logos::Logos;
use usbd_serial::UsbError;

use core::{fmt::Write, str::FromStr};
use minimq::embedded_nal::Ipv4Addr;

#[derive(Logos)]
enum Token {
    #[error]
    #[regex(r"[ \t\n\f\r]+", logos::skip)]
    Error,

    #[token("reset")]
    Reset,

    #[token("dfu")]
    Dfu,

    #[token("help")]
    Help,

    #[token("read")]
    Read,

    #[token("write")]
    Write,

    #[token("mac")]
    Mac,

    #[token("id")]
    Identifier,

    #[token("broker-address")]
    BrokerAddress,

    #[token("gateway")]
    Gateway,

    #[token("netmask")]
    Netmask,

    #[token("ip-address")]
    StaticIpAddress,

    #[token("fan")]
    FanSpeed,

    #[regex(r"[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+", |lex| lex.slice().parse())]
    IpAddress(Ipv4Addr),

    #[regex(r"[a-zA-Z0-9-]+")]
    DeviceIdentifier,

    #[token("service")]
    ServiceInfo,
}

#[derive(PartialEq)]
pub enum Property {
    Mac,
    BrokerAddress,
    Identifier,
    FanSpeed,
    IpAddress,
    Netmask,
    Gateway,
}

pub enum Request {
    Reset,
    ResetBootloader,
    Help,
    Read(Property),
    ServiceInfo,
    WriteIpAddress(Property, Ipv4Addr),
    WriteIdentifier(String<32>),
    WriteFanSpeed(f32),
}

impl TryFrom<Token> for Property {
    type Error = ();
    fn try_from(token: Token) -> Result<Property, ()> {
        let property = match token {
            Token::Mac => Property::Mac,
            Token::BrokerAddress => Property::BrokerAddress,
            Token::Identifier => Property::Identifier,
            Token::FanSpeed => Property::FanSpeed,
            Token::StaticIpAddress => Property::IpAddress,
            Token::Gateway => Property::Gateway,
            Token::Netmask => Property::Netmask,
            _ => return Err(()),
        };

        Ok(property)
    }
}

fn get_property_string(prop: Property, settings: &BoosterSettings) -> String<128> {
    let mut msg = String::<128>::new();
    match prop {
        Property::Identifier => writeln!(&mut msg, "{}", settings.id()).unwrap(),
        Property::Mac => writeln!(&mut msg, "{}", settings.mac()).unwrap(),
        Property::BrokerAddress => writeln!(&mut msg, "{}", settings.broker()).unwrap(),
        Property::Gateway => writeln!(&mut msg, "{}", settings.gateway()).unwrap(),
        Property::IpAddress => writeln!(&mut msg, "{}", settings.ip_address()).unwrap(),
        Property::Netmask => writeln!(&mut msg, "{}", settings.netmask()).unwrap(),
        Property::FanSpeed => writeln!(&mut msg, "{:.2} %", settings.fan_speed() * 100.0).unwrap(),
    };
    msg
}

/// A static-scope BBqueue for handling serial output buffering.
static OUTPUT_BUFFER: BBBuffer<1024> = BBBuffer::new();

/// A serial terminal for allowing the user to interact with Booster over USB.
pub struct SerialTerminal {
    settings: BoosterSettings,
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    input_buffer: Vec<u8, 128>,
    output_buffer_producer: bbqueue::Producer<'static, 1024>,
    output_buffer_consumer: bbqueue::Consumer<'static, 1024>,
    metadata: &'static crate::hardware::metadata::ApplicationMetadata,
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
        Self {
            settings,
            usb_device,
            usb_serial,
            input_buffer: Vec::new(),
            output_buffer_producer: producer,
            output_buffer_consumer: consumer,
            metadata,
        }
    }

    /// Poll the serial terminal and process any necessary updates.
    pub fn process(&mut self) {
        if let Some(request) = self.poll() {
            match request {
                Request::Help => self.print_help(),

                Request::Reset => {
                    // Power off all output channels and reset the MCU.
                    cortex_m::interrupt::disable();
                    platform::shutdown_channels();
                    cortex_m::peripheral::SCB::sys_reset();
                }

                Request::ResetBootloader => {
                    cortex_m::interrupt::disable();

                    // Power off all output channels and reset the MCU.
                    platform::shutdown_channels();

                    platform::reset_to_dfu_bootloader();
                }

                Request::ServiceInfo => {
                    let mut msg: String<256> = String::new();
                    writeln!(
                        &mut msg,
                        "{:<20}: {} [{}]",
                        "Version", self.metadata.firmware_version, self.metadata.profile,
                    )
                    .unwrap_or_else(|_| {
                        msg = String::from("Version: too long");
                    });
                    self.write(msg.as_bytes());

                    msg.clear();
                    writeln!(
                        &mut msg,
                        "{:<20}: {}",
                        "Hardware Revision", self.metadata.hardware_version
                    )
                    .unwrap_or_else(|_| {
                        msg = String::from("Hardware version: too long");
                    });
                    self.write(msg.as_bytes());

                    msg.clear();
                    writeln!(
                        &mut msg,
                        "{:<20}: {}",
                        "Rustc Version", self.metadata.rust_version,
                    )
                    .unwrap_or_else(|_| {
                        msg = String::from("Rustc Version: too long");
                    });
                    self.write(msg.as_bytes());

                    msg.clear();
                    writeln!(&mut msg, "{:<20}: {}", "Features", self.metadata.features)
                        .unwrap_or_else(|_| {
                            msg = String::from("Features: too long");
                        });
                    self.write(msg.as_bytes());

                    msg.clear();
                    writeln!(&mut msg, "{:<20}: {}", "Detected Phy", self.metadata.phy)
                        .unwrap_or_else(|_| {
                            msg = String::from("Detected Phy: too long");
                        });
                    self.write(msg.as_bytes());

                    msg.clear();
                    // Note(unwrap): The msg size is long enough to always contain the provided
                    // string.
                    write!(&mut msg, "{:<20}: ", "Panic Info").unwrap();
                    self.write(msg.as_bytes());
                    self.write(self.metadata.panic_info.as_bytes());
                    self.write("\n".as_bytes());

                    msg.clear();
                    // Note(unwrap): The msg size is long enough to be sufficient for all possible
                    // formats.
                    writeln!(
                        &mut msg,
                        "{:<20}: {}",
                        "Watchdog Detected", self.metadata.watchdog
                    )
                    .unwrap();
                    self.write(msg.as_bytes());

                    // Use this as a mechanism for the user to "acknowledge" the service state of
                    // the device. This will allow RF channels to re-enable.
                    platform::clear_reset_flags();
                }

                Request::WriteIpAddress(prop, addr) => match prop {
                    Property::BrokerAddress => self.settings.set_broker(addr),
                    Property::Gateway => self.settings.set_gateway(addr),
                    Property::Netmask => self.settings.set_netmask(addr),
                    Property::IpAddress => self.settings.set_ip_address(addr),
                    _ => self.write("Invalid property write\n".as_bytes()),
                },

                Request::WriteIdentifier(id) => {
                    if self.settings.set_id(id.as_str()).is_err() {
                        self.write("Invalid identifier\n".as_bytes());
                    }
                }

                Request::WriteFanSpeed(speed) => {
                    self.settings.set_fan_speed(speed);
                }

                Request::Read(prop) => {
                    let msg = get_property_string(prop, &self.settings);
                    self.write(msg.as_bytes());
                }
            }

            // Warn the user if there are settings in memory that differ from those that are
            // currently operating.
            if self.settings.are_dirty() {
                self.write(
                    "Settings in memory may differ from currently operating settings. \
                           Reset the device to apply settings.\n"
                        .as_bytes(),
                );
            }

            self.reset();
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
                self.input_buffer.clear();
                let len = read.buf().len();
                read.release(len);
            }
        };
    }

    /// Write data to the serial terminal.
    ///
    /// # Note
    /// The terminal uses an internal buffer. Overflows of the output buffer are silently ignored.
    pub fn write(&mut self, data: &[u8]) {
        // If we overflow the output buffer, allow the write to be silently truncated. The issue
        // will likely be cleared up as data is processed.
        match self
            .output_buffer_producer
            .grant_exact(data.len() + data.iter().filter(|&x| *x == b'\n').count())
            .or_else(|_| self.output_buffer_producer.grant_max_remaining(data.len()))
        {
            Ok(mut grant) => {
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
            Err(_) => return,
        };

        self.flush();
    }

    /// Poll the serial interface for any pending requests from the user.
    fn poll(&mut self) -> Option<Request> {
        // Update the USB serial port.
        if !self.usb_device.poll(&mut [&mut self.usb_serial]) {
            return None;
        }

        // Attempt to flush the output buffer.
        self.flush();

        // Consume data from the serial port.
        let mut buffer = [0u8; 64];
        match self.usb_serial.read(&mut buffer) {
            Ok(count) => {
                for &value in &buffer[..count] {
                    // Interpret the DEL and BS characters as a request to delete the most recently
                    // provided value. This supports Putty and TeraTerm defaults.
                    if value == b'\x08' || value == b'\x7F' {
                        // If there was previously data in the buffer, go back one, print an empty
                        // space, and then go back again to clear out the character on the user's
                        // screen.
                        if self.input_buffer.pop().is_some() {
                            // Note: we use this sequence because not all terminal emulators
                            // support the DEL character.
                            self.write(b"\x08 \x08");
                        }
                        continue;
                    }

                    // If we're echoing a CR, send a linefeed as well.
                    if value == b'\r' {
                        self.write(b"\n");
                    }

                    self.write(&[value]);

                    if self.input_buffer.push(value).is_err() {
                        // Make a best effort to inform the user of the overflow.
                        self.write("[!] Buffer overflow\n".as_bytes());
                        self.reset();
                        return None;
                    }
                }

                self.parse_buffer().unwrap_or_else(|msg| {
                    self.write(msg.as_bytes());
                    self.print_help();
                    self.reset();
                    None
                })
            }
            // If there's no data available, don't process anything.
            Err(UsbError::WouldBlock) => None,

            // If USB encountered an error, it's likely the port was disconnected. Reset our
            // buffers.
            Err(_) => {
                self.input_buffer.clear();
                self.output_buffer_consumer
                    .read()
                    .map(|grant| {
                        let len = grant.buf().len();
                        grant.release(len);
                    })
                    .ok();
                None
            }
        }
    }

    fn reset(&mut self) {
        self.input_buffer.clear();
        self.write("\n> ".as_bytes());
    }

    fn print_help(&mut self) {
        self.write(
            "\n
+----------------------+
| Booster Command Help :
+----------------------+
* `reset` - Resets the device
* `service` - Read the service information. Service infromation clears once read.
* `dfu` - Resets the device to DFU mode
* `read <PROP>` - Reads the value of <PROP>. <PROP> may be:
    - [broker-address, mac, id, fan, ip-address, netmask, gateway]
* `write [broker-address <IP> | gateway <IP> | ip-address <IP> | netmask <IP> | id <ID> | fan <DUTY>]
    - Writes the value of <IP> to the broker address, static IP, or gateway.
        * An `ip-address` of 0.0.0.0 will use DHCP
        * <IP> must be an IP address (e.g.  192.168.1.1)
        * Netmasks must contain only leading data. E.g. 255.255.0.0
    - Write the MQTT client ID of the device. <ID> must be 23 or less ASCII characters.
    - Write the <DUTY> default fan speed duty cycle, which is specified [0, 1.0].
"
            .as_bytes(),
        );
    }

    fn parse_buffer(&self) -> Result<Option<Request>, &'static str> {
        // If there's a line available in the buffer, parse it as a command.
        let mut lex = {
            if let Some(pos) = self
                .input_buffer
                .iter()
                .position(|&c| c == b'\n' || c == b'\r')
            {
                // Attempt to convert the slice to a string.
                let line =
                    core::str::from_utf8(&self.input_buffer[..pos]).map_err(|_| "Invalid bytes")?;
                Token::lexer(line)
            } else {
                // If there's no line available for parsing yet, there's no command to parse.
                return Ok(None);
            }
        };

        let command = lex.next().ok_or("Invalid command")?;
        let request = match command {
            Token::Reset => Request::Reset,
            Token::Dfu => Request::ResetBootloader,
            Token::Help => Request::Help,
            Token::ServiceInfo => Request::ServiceInfo,
            Token::Read => {
                // Validate that there is one valid token following.
                let property_token = lex.next().ok_or("Malformed command")?;

                // Check that the property is acceptable for a read.
                let property = match property_token.try_into() {
                    Ok(prop) => prop,
                    _ => return Err("Invalid property read"),
                };

                Request::Read(property)
            }
            Token::Write => {
                // Check that the property is acceptable for a write.
                match lex.next().ok_or("Malformed command")? {
                    token @ Token::BrokerAddress
                    | token @ Token::StaticIpAddress
                    | token @ Token::Gateway
                    | token @ Token::Netmask => {
                        if let Token::IpAddress(addr) = lex.next().ok_or("Malformed address")? {
                            Request::WriteIpAddress(token.try_into().unwrap(), addr)
                        } else {
                            return Err("Invalid property");
                        }
                    }
                    Token::Identifier => {
                        if let Token::DeviceIdentifier = lex.next().ok_or("Malformed ID")? {
                            if lex.slice().len() < 23 {
                                // The regex on this capture allow us to assume it is valid utf8, since
                                // we know it is alphanumeric.
                                let id: String<32> = String::from_str(
                                    core::str::from_utf8(lex.slice().as_bytes()).unwrap(),
                                )
                                .unwrap();

                                Request::WriteIdentifier(id)
                            } else {
                                return Err("ID too long");
                            }
                        } else {
                            return Err("Invalid property");
                        }
                    }
                    Token::FanSpeed => {
                        let fan_speed: f32 = lex
                            .remainder()
                            .trim()
                            .parse()
                            .map_err(|_| "Invalid float")?;
                        lex.bump(lex.remainder().len());
                        Request::WriteFanSpeed(fan_speed)
                    }
                    _ => return Err("Invalid property write"),
                }
            }
            _ => return Err("Invalid command"),
        };

        // Finally, verify that the lexer was consumed during parsing. Otherwise, the command
        // was malformed.
        if lex.next().is_some() {
            Err("Malformed command - trailing data\n")
        } else {
            Ok(Some(request))
        }
    }
}
