use super::clock::SystemTimer;
use crate::BoosterSettings;
use embedded_hal::blocking::delay::DelayUs;
use enc424j600::EthPhy;
use smoltcp_nal::smoltcp;

type Enc424j600 = enc424j600::Enc424j600<super::SPI, super::CS>;

const NUM_SOCKETS: usize = 3;

#[derive(Copy, Clone)]
pub struct TcpSocketStorage {
    rx: [u8; 1024],
    tx: [u8; 1024],
}

impl Default for TcpSocketStorage {
    fn default() -> Self {
        Self {
            rx: [0; 1024],
            tx: [0; 1024],
        }
    }
}

/// Containers for smoltcp-related network configurations
pub struct NetStorage {
    pub socket_storage: [smoltcp::iface::SocketStorage<'static>; NUM_SOCKETS],
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_SOCKETS],
}

// Mutable reference to a singleton instance of NetStorage
fn net_store_as_mut() -> &'static mut NetStorage {
    cortex_m::singleton!(: NetStorage = NetStorage {
        // Placeholder for the real IP address, which is initialized at runtime.
        ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
            smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
        )],
        neighbor_cache: [None; 8],
        routes_cache: [None; 8],
        tcp_socket_storage: [TcpSocketStorage::default(); NUM_SOCKETS],
        socket_storage: [smoltcp::iface::SocketStorage::EMPTY; NUM_SOCKETS],
    })
    .unwrap()
}

pub fn setup(
    spi: super::SPI,
    cs: super::CS,
    settings: &BoosterSettings,
    delay: &mut impl DelayUs<u16>,
) -> super::NetworkStackDrivers {
    use smoltcp as net;

    let mut enc424j600 =
        enc424j600::Enc424j600::new(spi, cs).cpu_freq_mhz(super::CPU_FREQ / 1_000_000);

    enc424j600.init(delay).expect("PHY initialization failed");
    // Overriding the MAC address stored on ENC424J600 is currently for consistency;
    // the value stored on the chip is currently unused for transmitting packets.
    enc424j600
        .write_mac_addr(settings.mac().as_bytes())
        .unwrap();

    let net_store = net_store_as_mut();

    let (device, phy_manager) = PhyManager::new(enc424j600);

    let mut eth_iface = {
        net_store.ip_addrs[0] = {
            let ip = settings.ip().octets();
            let subnet = settings.subnet().octets();
            net::wire::IpCidr::new(
                net::wire::IpAddress::from(net::wire::Ipv4Address::from_bytes(&ip)),
                net::wire::IpAddress::from(net::wire::Ipv4Address::from_bytes(&subnet))
                    .prefix_len()
                    .unwrap(),
            )
        };

        let routes = {
            let gateway = net::wire::Ipv4Address::from_bytes(&settings.gateway().octets());
            let mut routes = net::iface::Routes::new(&mut net_store.routes_cache[..]);
            routes.add_default_ipv4_route(gateway).unwrap();
            routes
        };

        let neighbor_cache = net::iface::NeighborCache::new(&mut net_store.neighbor_cache[..]);

        net::iface::InterfaceBuilder::new(device, &mut net_store.socket_storage[..])
            .hardware_addr(net::wire::HardwareAddress::Ethernet(
                smoltcp::wire::EthernetAddress::from_bytes(settings.mac().as_bytes()),
            ))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut net_store.ip_addrs[..])
            .routes(routes)
            .finalize()
    };

    for storage in net_store.tcp_socket_storage[..].iter_mut() {
        let tcp_socket = smoltcp::socket::TcpSocket::new(
            smoltcp::socket::TcpSocketBuffer::new(&mut storage.rx[..]),
            smoltcp::socket::TcpSocketBuffer::new(&mut storage.tx[..]),
        );

        eth_iface.add_socket(tcp_socket);
    }

    (
        phy_manager,
        smoltcp_nal::NetworkStack::new(eth_iface, SystemTimer::default()),
    )
}

use enc424j600::{rx::RxPacket, tx::TxPacket};

pub struct SmoltcpDevice {
    tx: &'static heapless::mpmc::MpMcQueue<TxPacket, 4>,
    rx: heapless::spsc::Consumer<'static, RxPacket, 4>,
}

pub struct PhyManager {
    phy: Enc424j600,
    pending_transmission: Option<TxPacket>,
    tx: &'static heapless::mpmc::MpMcQueue<TxPacket, 4>,
    rx: heapless::spsc::Producer<'static, RxPacket, 4>,
}

impl PhyManager {
    pub fn new(phy: Enc424j600) -> (SmoltcpDevice, PhyManager) {
        let rx_queue = cortex_m::singleton!(: heapless::spsc::Queue<RxPacket, 4> = heapless::spsc::Queue::new()).unwrap();
        let tx_queue = cortex_m::singleton!(: heapless::mpmc::MpMcQueue<TxPacket, 4> = heapless::mpmc::MpMcQueue::new()).unwrap();

        let (rx_producer, rx_consumer) = rx_queue.split();

        (
            SmoltcpDevice {
                tx: tx_queue,
                rx: rx_consumer,
            },
            PhyManager {
                phy,
                pending_transmission: None,
                tx: tx_queue,
                rx: rx_producer,
            },
        )
    }

    pub fn process(&mut self) {
        self.ingress();
        self.egress();
    }

    fn ingress(&mut self) {
        // If there's no space to enqueue read packets, don't try to receive at all.
        if !self.rx.ready() {
            return;
        }

        // Note(unwrap): We checked that there was sufficient queue space already, so this
        // should never fail.
        if let Ok(packet) = self.phy.recv_packet(false) {
            assert!(self.rx.enqueue(packet).is_ok());
        }
    }

    fn egress(&mut self) {
        // Attempt to send any pending frame.
        if let Some(packet) = self.pending_transmission.take() {
            if self.egress_packet(packet).is_err() {
                return;
            }
        }

        // Egress as many packets from the queue as possible.
        while let Some(packet) = self.tx.dequeue() {
            if self.egress_packet(packet).is_err() {
                return;
            }
        }
    }

    fn egress_packet(&mut self, packet: TxPacket) -> Result<(), ()> {
        if self.phy.send_packet(&packet).is_err() {
            // If we can't send the current packet, we have to store it until we can in the future.
            self.pending_transmission.replace(packet);
            return Err(());
        }

        Ok(())
    }
}

impl<'a> smoltcp::phy::Device<'a> for SmoltcpDevice {
    type RxToken = RxToken;
    type TxToken = TxToken;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.max_transmission_unit = enc424j600::RAW_FRAME_LENGTH_MAX;
        caps
    }

    fn receive(&mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        match self.rx.dequeue() {
            Some(rx_packet) => Some((RxToken(rx_packet), TxToken(&self.tx))),
            _ => None,
        }
    }

    fn transmit(&mut self) -> Option<Self::TxToken> {
        Some(TxToken(&self.tx))
    }
}

pub struct RxToken(RxPacket);

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(
        mut self,
        _timestamp: smoltcp::time::Instant,
        f: F,
    ) -> Result<R, smoltcp::Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, smoltcp::Error>,
    {
        let len = self.0.get_frame_length();
        let frame = self.0.get_mut_frame();
        f(&mut frame[..len])
    }
}

pub struct TxToken(&'static heapless::mpmc::MpMcQueue<TxPacket, 4>);

impl smoltcp::phy::TxToken for TxToken {
    fn consume<R, F>(
        self,
        _timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> Result<R, smoltcp::Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, smoltcp::Error>,
    {
        let mut tx_packet = TxPacket::new();
        let frame = tx_packet.get_mut_frame();
        assert!(len <= frame.len());
        let result = f(frame);

        match self.0.enqueue(tx_packet) {
            Ok(_) => result,

            // We couldn't enqueue the packet.
            Err(_) => Err(smoltcp::Error::Exhausted),
        }
    }
}
