//! Booster NGFW Application
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use super::{hardware::platform, hardware::UsbBus, BoosterSettings};
use bbqueue::BBBuffer;
use heapless::{consts, String, Vec};
use logos::Logos;
use usbd_serial::UsbError;

use core::fmt::Write;
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

    #[token("netmask")]
    Netmask,

    #[token("gateway")]
    Gateway,

    #[token("ip-address")]
    SelfAddress,

    #[token("broker-address")]
    BrokerAddress,

    #[regex(r"[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+", |lex| lex.slice().parse())]
    IpAddress(Ipv4Addr),

    #[regex(r"[a-zA-Z0-9]+")]
    DeviceIdentifier,

    #[token("service")]
    ServiceInfo,
}

#[derive(PartialEq)]
pub enum Property {
    Mac,
    SelfAddress,
    BrokerAddress,
    Netmask,
    Gateway,
    Identifier,
}

pub enum Request {
    Reset,
    ResetBootloader,
    Help,
    Read(Property),
    ServiceInfo,
    WriteIpAddress(Property, Ipv4Addr),
    WriteIdentifier(String<consts::U32>),
}

fn get_property_string(prop: Property, settings: &BoosterSettings) -> String<consts::U128> {
    let mut msg = String::<consts::U128>::new();
    match prop {
        Property::Identifier => write!(&mut msg, "{}\n", settings.id()).unwrap(),
        Property::Mac => write!(&mut msg, "{}\n", settings.mac()).unwrap(),
        Property::SelfAddress => write!(&mut msg, "{}\n", settings.ip()).unwrap(),
        Property::BrokerAddress => write!(&mut msg, "{}\n", settings.broker()).unwrap(),
        Property::Netmask => write!(&mut msg, "{}\n", settings.subnet()).unwrap(),
        Property::Gateway => write!(&mut msg, "{}\n", settings.gateway()).unwrap(),
    };
    msg
}

/// A static-scope BBqueue for handling serial output buffering.
static OUTPUT_BUFFER: BBBuffer<bbqueue::consts::U1024> = BBBuffer(bbqueue::ConstBBBuffer::new());

/// A serial terminal for allowing the user to interact with Booster over USB.
pub struct SerialTerminal {
    settings: BoosterSettings,
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    input_buffer: Vec<u8, consts::U128>,
    output_buffer_producer: bbqueue::Producer<'static, bbqueue::consts::U1024>,
    output_buffer_consumer: bbqueue::Consumer<'static, bbqueue::consts::U1024>,
}

impl SerialTerminal {
    /// Construct a terminal for interacting with the user.
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
        settings: BoosterSettings,
    ) -> Self {
        let (producer, consumer) = OUTPUT_BUFFER.try_split().unwrap();
        Self {
            settings,
            usb_device,
            usb_serial,
            input_buffer: Vec::new(),
            output_buffer_producer: producer,
            output_buffer_consumer: consumer,
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

                #[cfg(feature = "unstable")]
                Request::ResetBootloader => {
                    cortex_m::interrupt::disable();

                    // Power off all output channels and reset the MCU.
                    platform::shutdown_channels();

                    platform::reset_to_dfu_bootloader();
                }

                #[cfg(not(feature = "unstable"))]
                Request::ResetBootloader => {
                    self.write(
                        "Reset to DFU bootloader not supported with stable toolchain".as_bytes(),
                    );
                }

                Request::ServiceInfo => {
                    let mut msg: String<consts::U256> = String::new();
                    write!(&mut msg, "{:<20}: {}\n", "Version", env!("VERSION")).unwrap_or_else(
                        |_| {
                            msg = String::from("Version: too long");
                        },
                    );
                    self.write(msg.as_bytes());

                    msg.clear();
                    write!(
                        &mut msg,
                        "{:<20}: {}\n",
                        "Git revision",
                        env!("GIT_REVISION")
                    )
                    .unwrap_or_else(|_| {
                        msg = String::from("Git revision: too long");
                    });
                    self.write(msg.as_bytes());

                    msg.clear();
                    write!(&mut msg, "{:<20}: {}\n", "Features", env!("ALL_FEATURES"))
                        .unwrap_or_else(|_| {
                            msg = String::from("Features: too long");
                        });
                    self.write(msg.as_bytes());

                    msg.clear();
                    // Note(unwrap): The msg size is long enough to always contain the provided
                    // string.
                    write!(&mut msg, "{:<20}: ", "Panic Info").unwrap();
                    self.write(msg.as_bytes());
                    self.write(
                        panic_persist::get_panic_message_bytes().unwrap_or("None".as_bytes()),
                    );
                    self.write("\n".as_bytes());

                    msg.clear();
                    // Note(unwrap): The msg size is long enough to be sufficient for all possible
                    // formats.
                    write!(
                        &mut msg,
                        "{:<20}: {}\n",
                        "Watchdog Detected",
                        platform::watchdog_detected()
                    )
                    .unwrap();
                    self.write(msg.as_bytes());

                    // Reading the panic message above clears the panic message, so similarly, we
                    // should also clear the watchdog once read.
                    platform::clear_reset_flags();
                }

                Request::WriteIpAddress(prop, addr) => match prop {
                    Property::SelfAddress => self.settings.set_ip_address(addr),
                    Property::BrokerAddress => self.settings.set_broker(addr),
                    Property::Gateway => self.settings.set_gateway(addr),
                    Property::Netmask => self.settings.set_netmask(addr),
                    _ => self.write("Invalid property write\n".as_bytes()),
                },

                Request::WriteIdentifier(id) => {
                    if self.settings.set_id(id.as_str()).is_err() {
                        self.write("Invalid identifier\n".as_bytes());
                    }
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
            .grant_exact(data.len())
            .or_else(|_| self.output_buffer_producer.grant_max_remaining(data.len()))
        {
            Ok(mut grant) => {
                let len = grant.buf().len();
                grant.buf().copy_from_slice(&data[..len]);
                grant.commit(len);
            }
            Err(_) => return,
        };

        self.flush();
    }

    /// Poll the serial interface for any pending requests from the user.
    fn poll(&mut self) -> Option<Request> {
        // Update the USB serial port.
        if self.usb_device.poll(&mut [&mut self.usb_serial]) == false {
            return None;
        }

        // Attempt to flush the output buffer.
        self.flush();

        // Consume data from the serial port.
        let mut buffer = [0u8; 64];
        match self.usb_serial.read(&mut buffer) {
            Ok(count) => {
                // Echo the data back.
                self.write(&buffer[..count]);

                match self.input_buffer.extend_from_slice(&buffer[..count]) {
                    Ok(_) => match self.parse_buffer() {
                        // If there's nothing requested yet, keep waiting for more data.
                        Ok(None) => None,

                        // If we got a request, clear the buffer and process it.
                        Ok(Some(result)) => Some(result),

                        // Otherwise, if there was an error, clear the buffer and print it.
                        Err(msg) => {
                            self.write(msg.as_bytes());
                            self.print_help();
                            self.reset();
                            None
                        }
                    },
                    Err(_) => {
                        // Make a best effort to inform the user of the overflow.
                        self.write("[!] Buffer overflow\n".as_bytes());
                        self.reset();
                        None
                    }
                }
            }
            // If there's no data available, don't process anything.
            Err(UsbError::WouldBlock) => None,

            // If USB encountered an error, it's likely the port was disconnected. Reset our
            // buffers.
            Err(_) => {
                self.input_buffer.clear();
                self.output_buffer_consumer
                    .read()
                    .and_then(|grant| {
                        let len = grant.buf().len();
                        grant.release(len);
                        Ok(())
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
* `dfu` - Resets the device to DFU mode
* `read <PROP>` - Reads the value of PROP. PROP may be [ip-address, broker-address, mac, id, \
gateway, netmask]
* `write <PROP> <IP>` - Writes the value of <IP> to <PROP>. <PROP> may be [ip-address, broker-address, \
netmask, gateway] and <IP> must be an IP address (e.g.  192.168.1.1)
* `write id <ID>` - Write the MQTT client ID of the device. <ID> must be 23 or less ASCII \
characters.
* `service` - Read the service information. Service infromation clears once read.
".as_bytes(),
        );
    }

    fn parse_buffer(&self) -> Result<Option<Request>, &'static str> {
        // If there's a line available in the buffer, parse it as a command.
        let mut lex = {
            if let Some(pos) = self.input_buffer.iter().position(|&c| c == '\n' as u8) {
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
                let property = match property_token {
                    Token::Mac => Property::Mac,
                    Token::SelfAddress => Property::SelfAddress,
                    Token::BrokerAddress => Property::BrokerAddress,
                    Token::Gateway => Property::Gateway,
                    Token::Netmask => Property::Netmask,
                    Token::Identifier => Property::Identifier,
                    _ => return Err("Invalid property read"),
                };

                Request::Read(property)
            }
            Token::Write => {
                // Validate that there are two valid token following.
                let property_token = lex.next().ok_or("Malformed command")?;

                // Check that the property is acceptable for a read.
                let property = match property_token {
                    Token::SelfAddress => Property::SelfAddress,
                    Token::BrokerAddress => Property::BrokerAddress,
                    Token::Gateway => Property::Gateway,
                    Token::Netmask => Property::Netmask,
                    Token::Identifier => Property::Identifier,
                    _ => return Err("Invalid property write"),
                };

                let value_token = lex.next().ok_or("Malformed property")?;

                match value_token {
                    Token::IpAddress(addr) => Request::WriteIpAddress(property, addr),
                    Token::DeviceIdentifier if property == Property::Identifier => {
                        if lex.slice().len() < 23 {
                            let vec =
                                Vec::<u8, consts::U32>::from_slice(lex.slice().as_bytes()).unwrap();

                            // The regex on this capture allow us to assume it is valid utf8, since
                            // we know it is alphanumeric.
                            let id = String::<consts::U32>::from_utf8(vec).unwrap();

                            Request::WriteIdentifier(id)
                        } else {
                            return Err("ID too long");
                        }
                    }
                    _ => return Err("Invalid write request"),
                }
            }
            _ => return Err("Invalid command"),
        };

        // Finally, verify that the lexer was consumed during parsing. Otherwise, the command
        // was malformed.
        if lex.next().is_some() {
            Err("Malformed command\n")
        } else {
            Ok(Some(request))
        }
    }
}
