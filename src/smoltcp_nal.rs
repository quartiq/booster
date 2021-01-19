// Ripped off from minimq example
use core::cell::RefCell;
use heapless::{consts, Vec};
use minimq::embedded_nal as nal;
use smoltcp as net;
use stm32f4xx_hal as hal;
use nal::nb;
use super::SPI;

use rtic::cyccnt::Instant;

#[derive(Debug)]
pub enum NetworkError {
    NoSocket,
    ConnectionFailure,
    ReadFailure,
    WriteFailure,
    Unsupported,
}

type NetworkInterface =
    net::iface::EthernetInterface<'static, 'static, 'static, enc424j600::smoltcp_phy::SmoltcpDevice<
        enc424j600::SpiEth::<SPI, hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>>
    >>;

pub struct NetworkStack<'a, 'b, 'c> {
    network_interface: RefCell<NetworkInterface>,
    sockets: RefCell<net::socket::SocketSet<'a, 'b, 'c>>,
    next_port: RefCell<u16>,
    unused_handles: RefCell<Vec<net::socket::SocketHandle, consts::U16>>,
    time_ms: RefCell<u32>,
    last_update_instant: RefCell<Option<Instant>>,
}

impl<'a, 'b, 'c> NetworkStack<'a, 'b, 'c> {
    pub fn new(
        interface: NetworkInterface,
        sockets: net::socket::SocketSet<'a, 'b, 'c>
    ) -> Self {
        let mut unused_handles: Vec<net::socket::SocketHandle, consts::U16> = Vec::new();
        for socket in sockets.iter() {
            unused_handles.push(socket.handle()).unwrap();
        }

        NetworkStack {
            network_interface: RefCell::new(interface),
            sockets: RefCell::new(sockets),
            next_port: RefCell::new(49152),
            unused_handles: RefCell::new(unused_handles),
            time_ms: RefCell::new(0),
            last_update_instant: RefCell::new(None),
        }
    }

    // Include auto_time_update to allow Instant::now() to not be called
    // Instant::now() is not safe to call in `init()` context
    pub fn update(&self, auto_time_update: bool) -> bool {
        if auto_time_update {
            // Check if it is the first time the stack has updated the time itself
            let now = match *self.last_update_instant.borrow() {
                // If it is the first time, do not advance time
                // Simply store the current instant to initiate time updating
                None => {
                    Instant::now()
                }

                // If it was updated before, advance time and update last_update_instant
                Some(instant) => {
                    // Calculate elapsed time
                    let now = Instant::now();
                    let duration = now.duration_since(instant);
                    // Adjust duration into ms (note: decimal point truncated)
                    self.advance_time(duration.as_cycles()/168_000);
                    now
                }
            };

            self.last_update_instant.replace(Some(now));
        }

        match self.network_interface.borrow_mut().poll(
            &mut self.sockets.borrow_mut(),
            net::time::Instant::from_millis(*self.time_ms.borrow() as i64),
        ) {
            Ok(changed) => changed == false,
            Err(e) => {
                info!("{:?}", e);
                true
            }
        }
    }

    pub fn advance_time(&self, duration: u32) {
        *self.time_ms.borrow_mut() += duration;
    }

    fn get_ephemeral_port(&self) -> u16 {
        // Get the next ephemeral port
        let current_port = self.next_port.borrow().clone();

        let (next, wrap) = self.next_port.borrow().overflowing_add(1);
        *self.next_port.borrow_mut() = if wrap { 49152 } else { next };

        return current_port;
    }
}

impl<'a, 'b, 'c> nal::TcpStack for NetworkStack<'a, 'b, 'c> {
    type TcpSocket = net::socket::SocketHandle;
    type Error = NetworkError;

    fn open(&self, _mode: nal::Mode) -> Result<Self::TcpSocket, Self::Error> {
        match self.unused_handles.borrow_mut().pop() {
            Some(handle) => {
                // Abort any active connections on the handle.
                let mut sockets = self.sockets.borrow_mut();
                let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(handle);
                internal_socket.abort();

                Ok(handle)
            }
            None => Err(NetworkError::NoSocket),
        }
    }

    // Ideally connect is only to be performed in `init()` of `main.rs`
    // Calling `Instant::now()` of `rtic::cyccnt` would face correctness issue during `init()`
    fn connect(
        &self,
        socket: Self::TcpSocket,
        remote: nal::SocketAddr,
    ) -> Result<Self::TcpSocket, Self::Error> {
        let address = {
            let mut sockets = self.sockets.borrow_mut();
            let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(socket);

            // If we're already in the process of connecting, ignore the request silently.
            if internal_socket.is_open() {
                return Ok(socket);
            }
    
            match remote.ip() {
                nal::IpAddr::V4(addr) => {
                    let octets = addr.octets();
                    let address = net::wire::Ipv4Address::new(octets[0], octets[1], octets[2], octets[3]);
                    internal_socket
                        .connect((address, remote.port()), self.get_ephemeral_port())
                        .map_err(|_| NetworkError::ConnectionFailure)?;
                    address
                }
                nal::IpAddr::V6(_) => {                    
                    // Match W5500 behavior: Reject the use of IPV6
                    return Err(NetworkError::Unsupported);
                }
            }
        };

        // Match W5500 behavior: Poll until connected
        loop {
            match self.is_connected(&socket) {
                Ok(true) => break,
                _ => {
                    let mut sockets = self.sockets.borrow_mut();
                    let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(socket);
                    // If the connect got ACK->RST, it will end up in Closed TCP state
                    // Perform reconnection in this case
                    // In all other scenario, simply wait for TCP connection to be established
                    if internal_socket.state() == net::socket::TcpState::Closed {
                        internal_socket.close();
                        internal_socket
                            .connect((address, remote.port()), self.get_ephemeral_port())
                            .map_err(|_| NetworkError::ConnectionFailure)?;
                    }
                }
            }

            // Avoid using Instant::now() and Advance time manually
            self.update(false);

            // Delay for 1 ms, minimum time unit of smoltcp
            // TODO: Allow clock configuration, if supported in main
            cortex_m::asm::delay(168_000_000/1_000);
            {
                self.advance_time(1);
            }
        }

        Ok(socket)
    }

    fn is_connected(&self, socket: &Self::TcpSocket) -> Result<bool, Self::Error> {
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*socket);

        Ok(socket.may_send() && socket.may_recv())
    }

    fn write(&self, socket: &mut Self::TcpSocket, buffer: &[u8]) -> nb::Result<usize, Self::Error> {
        let mut non_queued_bytes = &buffer[..];
        while non_queued_bytes.len() != 0 {
            let result = {
                let mut sockets = self.sockets.borrow_mut();
                let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*socket);
                let result = socket.send_slice(non_queued_bytes);
                result
            };

            match result {
                Ok(num_bytes) => {
                    // In case the buffer is filled up, push bytes into ethernet driver
                    if num_bytes != non_queued_bytes.len() {
                        self.update(true);
                    }
                    
                    // Process the unwritten bytes again, if any
                    non_queued_bytes = &non_queued_bytes[num_bytes..]
                }
                Err(_) => return Err(nb::Error::Other(NetworkError::WriteFailure))
            }
        }

        Ok(buffer.len())
    }

    fn read(
        &self,
        socket: &mut Self::TcpSocket,
        buffer: &mut [u8],
    ) -> nb::Result<usize, Self::Error> {
        // Enqueue received bytes into the TCP socket buffer
        self.update(true);

        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut net::socket::TcpSocket = &mut *sockets.get(*socket);

        let result = socket.recv_slice(buffer);
        match result {
            Ok(num_bytes) => Ok(num_bytes),
            Err(_) => Err(nb::Error::Other(NetworkError::ReadFailure)),
        }
    }

    fn close(&self, socket: Self::TcpSocket) -> Result<(), Self::Error> {
        let mut sockets = self.sockets.borrow_mut();
        let internal_socket: &mut net::socket::TcpSocket = &mut *sockets.get(socket);
        internal_socket.close();

        self.unused_handles.borrow_mut().push(socket).unwrap();
        Ok(())
    }
}