use super::{Enc424j600, Ethernet};
use crate::BoosterSettings;
use cortex_m::peripheral::DWT;
use embedded_hal::blocking::delay::DelayUs;
use embedded_time::{clock, duration::*, Instant};
use smoltcp_nal::smoltcp;

/// Containers for smoltcp-related network configurations
pub struct NetStorage {
    pub socket_storage: [Option<smoltcp::socket::SocketSetItem<'static>>; 1],
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
    pub tx_storage: [u8; 4096],
    pub rx_storage: [u8; 1024],
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
        socket_storage: [None; 1],
        tx_storage: [0; 4096],
        rx_storage: [0; 1024],
    })
    .unwrap()
}

pub fn setup(
    mut enc424j600: Enc424j600,
    settings: &BoosterSettings,
    delay: &mut impl DelayUs<u16>,
) -> Ethernet {
    use smoltcp as net;

    enc424j600.init(delay).expect("PHY initialization failed");
    // Overriding the MAC address stored on ENC424J600 is currently for consistency;
    // the value stored on the chip is currently unused for transmitting packets.
    enc424j600
        .write_mac_addr(settings.mac().as_bytes())
        .unwrap();

    let net_store = net_store_as_mut();

    let eth_iface = {
        let device = enc424j600::smoltcp_phy::SmoltcpDevice::new(enc424j600);

        net_store.ip_addrs[0] = {
            let ip = settings.ip().octets();
            let subnet = settings.subnet().octets();
            net::wire::IpCidr::new(
                net::wire::IpAddress::from(net::wire::Ipv4Address::from_bytes(&ip)),
                net::wire::IpAddress::from(net::wire::Ipv4Address::from_bytes(&subnet))
                    .to_prefix_len()
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

        net::iface::EthernetInterfaceBuilder::new(device)
            .ethernet_addr(net::wire::EthernetAddress::from_bytes(
                settings.mac().as_bytes(),
            ))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut net_store.ip_addrs[..])
            .routes(routes)
            .finalize()
    };

    let sockets = {
        let mut sockets = net::socket::SocketSet::new(&mut net_store.socket_storage[..]);

        let tcp_socket = {
            let tx_buffer = net::socket::TcpSocketBuffer::new(&mut net_store.tx_storage[..]);
            let rx_buffer = net::socket::TcpSocketBuffer::new(&mut net_store.rx_storage[..]);

            net::socket::TcpSocket::new(rx_buffer, tx_buffer)
        };

        sockets.add(tcp_socket);
        sockets
    };

    Ethernet::new(eth_iface, sockets)
}

#[derive(Debug)]
pub enum ClockError {
    TimeFault,
}

/// Simple struct for implementing embedded_time::Clock
pub struct EpochClock<const CPUFREQ: u32> {
    /// Epoch time in milliseconds to store a greater value
    epoch_time_ms: Milliseconds<u32>,
    /// Epoch time in ticks to store a temporary value
    epoch_time_ticks: Instant<Self>,
}

impl<const CPUFREQ: u32> EpochClock<CPUFREQ> {
    pub fn new() -> Self {
        assert!(CPUFREQ == crate::CPU_FREQ);

        Self {
            epoch_time_ms: Milliseconds::<u32>::new(0),
            epoch_time_ticks: Instant::<Self>::new(0),
        }
    }

    /// Update the valid epoch time in milliseconds, and returns the value.
    ///
    /// Safe to call after RTIC #[init]. Returns Err() if DWT CYCCNT returns a
    /// smaller value than the last recorded time.
    pub fn now(&mut self) -> Result<u32, ClockError> {
        use clock::Clock;
        use core::convert::TryInto;

        let now = match self.try_now() {
            Ok(now) => now,
            Err(_) => return Err(ClockError::TimeFault),
        };
        let elapsed: Milliseconds<u32> = match now.checked_duration_since(&self.epoch_time_ticks) {
            Some(elapsed) => elapsed.try_into().map_err(|_| ClockError::TimeFault)?,
            None => return Err(ClockError::TimeFault),
        };
        self.epoch_time_ticks = now;
        self.epoch_time_ms = self.epoch_time_ms + elapsed;
        Ok(self.epoch_time_ms.integer())
    }
}

/// Implement a simple embedded_time::clock::Clock at the given CPU clock frequency
/// based on DWT CYCCNT.
///
/// This leverages "const generics" introduced in Rust 1.51.
impl<const CPUFREQ: u32> clock::Clock for EpochClock<CPUFREQ> {
    type T = u32;

    const SCALING_FACTOR: Fraction = Fraction::new(1, CPUFREQ);

    /// Returns directly from DWT CYCCNT. Not guaranteed to be valid.
    fn try_now(&self) -> Result<Instant<Self>, clock::Error> {
        Ok(Instant::new(DWT::get_cycle_count()))
    }
}
