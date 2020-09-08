use super::{Error, Settings, UsbBus};
use heapless::{consts, String, Vec};
use logos::Logos;
use usbd_serial::UsbError;

use core::fmt::Write;
use w5500::Ipv4Addr;

#[derive(Logos)]
enum Token {
    #[error]
    #[regex(r"[ \t\n\f\r]+", logos::skip)]
    Error,

    #[token("reset")]
    Reset,

    #[token("help")]
    Help,

    #[token("upgrade")]
    Upgrade,

    #[token("read")]
    Read,

    #[token("write")]
    Write,

    #[token("mac")]
    Mac,

    #[token("ip-address")]
    SelfAddress,

    #[token("broker-address")]
    BrokerAddress,

    #[regex(r"[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+", |lex| lex.slice().parse())]
    IpAddress(Ipv4Addr),
}

pub enum Property {
    Mac,
    SelfAddress,
    BrokerAddress,
}

pub enum Request {
    Reset,
    Upgrade,
    Help,
    Read(Property),
    WriteIpAddress(Property, Ipv4Addr),
}

struct RingBuffer {
    data: [u8; 512],
    head: usize,
    tail: usize,
}

impl RingBuffer {
    pub fn new() -> Self {
        Self {
            data: [0; 512],
            head: 0,
            tail: 0,
        }
    }

    fn size(&mut self) -> usize {
        if self.head > self.tail {
            self.data[self.head..].len() + self.data[..self.tail].len()
        } else {
            self.data[self.head..self.tail].len()
        }
    }

    /// Push data into the ring buffer tail.
    pub fn push(&mut self, data: &[u8]) -> usize {
        for (count, byte) in data.iter().enumerate() {
            // Check if the next byte will overflow.
            let next_tail = (self.tail + 1) % self.data.len();
            if self.head == next_tail {
                return count;
            }

            self.data[self.tail] = *byte;
            self.tail = next_tail;
        }

        data.len()
    }

    /// Remove data from the ring buffer head.
    pub fn pop(&mut self, count: usize) -> Result<(), Error> {
        if self.size() < count {
            return Err(Error::Bounds);
        }

        // Move the head forward to simulate removing data from the FIFO.
        self.head = (self.head + count) % self.data.len();
        Ok(())
    }

    /// Access the underlying memory of the ring buffer.
    pub fn as_slices<'a>(&'a self) -> (&'a [u8], &'a [u8]) {
        if self.head > self.tail {
            (&self.data[self.head..], &self.data[..self.tail])
        } else {
            (&self.data[self.head..self.tail], &[])
        }
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.tail = 0;
    }
}

pub struct SerialTerminal {
    settings: Settings,
    usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
    usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
    input_buffer: Vec<u8, consts::U128>,
    output_buffer: RingBuffer,
}

impl SerialTerminal {
    pub fn new(
        usb_device: usb_device::device::UsbDevice<'static, UsbBus>,
        usb_serial: usbd_serial::SerialPort<'static, UsbBus>,
        settings: Settings,
    ) -> Self {
        Self {
            settings,
            usb_device,
            usb_serial,
            input_buffer: Vec::new(),
            output_buffer: RingBuffer::new(),
        }
    }
    pub fn process(&mut self) {
        if let Some(request) = self.poll() {
            match request {
                Request::Help => self.print_help(),

                Request::Reset => {
                    self.write("WouldReset\n".as_bytes());
                }

                Request::Upgrade => {
                    self.write("WouldUpgrade\n".as_bytes());
                }

                Request::WriteIpAddress(prop, addr) => match prop {
                    Property::SelfAddress => {
                        self.settings.set_ip_address(addr);
                    }
                    Property::BrokerAddress => {
                        self.settings.set_broker(addr);
                    }
                    _ => self.write("Invalid property write\n".as_bytes()),
                },

                Request::Read(Property::Mac) => {
                    let mut msg = String::<consts::U128>::new();
                    write!(&mut msg, "{}\n", self.settings.mac_address).unwrap();
                    self.write(msg.as_bytes());
                }
                Request::Read(Property::SelfAddress) => {
                    let mut msg = String::<consts::U128>::new();
                    write!(&mut msg, "{}\n", self.settings.ip_address).unwrap();
                    self.write(msg.as_bytes());
                }

                Request::Read(Property::BrokerAddress) => {
                    let mut msg = String::<consts::U128>::new();
                    write!(&mut msg, "{}\n", self.settings.broker_address).unwrap();
                    self.write(msg.as_bytes());
                }
            }

            // TODO: Warn the user if there are settings in memory that differ from those that are
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
        // Write as much data as possible.
        let (head, tail) = self.output_buffer.as_slices();

        // TODO: Properly handle wouldblock/USB errors here.
        let num_written = match self.usb_serial.write(head) {
            Ok(count) => count,
            Err(UsbError::WouldBlock) => 0,
            Err(_) => 0,
        };

        if num_written < head.len() {
            self.output_buffer.pop(num_written).unwrap();
        } else {
            // If we wrote the whole head, try writing the tail as well.
            let num_written = match self.usb_serial.write(tail) {
                Ok(count) => count,
                Err(UsbError::WouldBlock) => 0,
                // If USB encountered an error, it's likely the port was disconnected. Reset our
                // buffers.
                Err(_) => {
                    self.input_buffer.clear();
                    self.output_buffer.clear();
                    return;
                }
            };

            let head_length = head.len();
            self.output_buffer.pop(head_length + num_written).unwrap();
        }
    }

    pub fn write(&mut self, data: &[u8]) {
        // If we overflow the output buffer, allow the write to be silently truncated. The issue
        // will likely be cleared up as data is processed.
        self.output_buffer.push(data);
        self.flush();
    }

    pub fn poll(&mut self) -> Option<Request> {
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
                self.output_buffer.clear();
                self.input_buffer.clear();
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
* `upgrade`  - Resets the device into DFU mode
* `read <PROP>` - Reads the value of PROP. PROP may be [ip-address, broker-address, mac]
* `write <PROP> <VAL>` - Writes the value of VAL to PROP. PROP may be [ip-address, broker-address] \
and VAL must be an IP address (e.g.  192.168.1.1)\n"
                .as_bytes(),
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
            Token::Upgrade => Request::Upgrade,
            Token::Help => Request::Help,
            Token::Read => {
                // Validate that there is one valid token following.
                let property_token = lex.next().ok_or("Malformed command")?;

                // Check that the property is acceptable for a read.
                let property = match property_token {
                    Token::Mac => Property::Mac,
                    Token::SelfAddress => Property::SelfAddress,
                    Token::BrokerAddress => Property::BrokerAddress,
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
                    _ => return Err("Invalid property write"),
                };

                if let Some(Token::IpAddress(addr)) = lex.next() {
                    Request::WriteIpAddress(property, addr)
                } else {
                    return Err("Invalid IP address write");
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
