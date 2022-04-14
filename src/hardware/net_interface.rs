use crate::BoosterSettings;
use smoltcp_nal::smoltcp;

use super::external_mac::SmoltcpDevice;

/// The number of TCP sockets supported in the network stack.
const NUM_TCP_SOCKETS: usize = 4;

/// Static storage for Smoltcp sockets and network state.
static mut NETWORK_STORAGE: NetStorage = NetStorage::new();

/// Containers for smoltcp-related network configurations
struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],

    // Note: There is an additional socket set item required for the DHCP socket.
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_TCP_SOCKETS + 1],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
}

impl NetStorage {
    const fn new() -> Self {
        NetStorage {
            // Placeholder for the real IP address, which is initialized at runtime.
            ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
                smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
            )],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [smoltcp::iface::SocketStorage::EMPTY; NUM_TCP_SOCKETS + 1],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
        }
    }
}

#[derive(Copy, Clone)]
struct TcpSocketStorage {
    rx_storage: [u8; 1024],

    // Note that TX storage is set to 4096 to ensure that it is sufficient to contain full
    // telemetry messages for all 8 RF channels.
    tx_storage: [u8; 4096],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            tx_storage: [0; 4096],
            rx_storage: [0; 1024],
        }
    }
}

/// Set up the network interface.
///
/// # Note
/// This function may only be called exactly once.
///
/// # Args
/// * `device` - The smoltcp interface device.
/// * `settings` - The device settings to use.
pub fn setup(
    device: SmoltcpDevice<'static>,
    settings: &BoosterSettings,
) -> smoltcp::iface::Interface<'static, SmoltcpDevice<'static>> {
    // Note(unsafe): This function may only be called once to initialize the network storage.
    let net_store = unsafe { &mut NETWORK_STORAGE };

    let mut interface = {
        net_store.ip_addrs[0] = {
            let ip = settings.ip().octets();
            let subnet = settings.subnet().octets();
            smoltcp::wire::IpCidr::new(
                smoltcp::wire::IpAddress::from(smoltcp::wire::Ipv4Address::from_bytes(&ip)),
                smoltcp::wire::IpAddress::from(smoltcp::wire::Ipv4Address::from_bytes(&subnet))
                    .prefix_len()
                    .unwrap(),
            )
        };

        let routes = {
            let gateway = smoltcp::wire::Ipv4Address::from_bytes(&settings.gateway().octets());
            let mut routes = smoltcp::iface::Routes::new(&mut net_store.routes_cache[..]);
            routes.add_default_ipv4_route(gateway).unwrap();
            routes
        };

        let neighbor_cache = smoltcp::iface::NeighborCache::new(&mut net_store.neighbor_cache[..]);

        smoltcp::iface::InterfaceBuilder::new(device, &mut net_store.sockets[..])
            .hardware_addr(smoltcp::wire::HardwareAddress::Ethernet(
                smoltcp::wire::EthernetAddress(settings.mac().octets),
            ))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut net_store.ip_addrs[..])
            .routes(routes)
            .finalize()
    };

    for storage in net_store.tcp_socket_storage[..].iter_mut() {
        let tcp_socket = {
            let rx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut storage.rx_storage[..]);
            let tx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut storage.tx_storage[..]);

            smoltcp::socket::TcpSocket::new(rx_buffer, tx_buffer)
        };

        interface.add_socket(tcp_socket);
    }

    // TODO: Enable after we remove static IP configurations.
    //interface.add_socket(smoltcp::socket::Dhcpv4Socket::new());

    interface
}
