use crate::BoosterSettings;
use smoltcp_nal::smoltcp;

use super::external_mac::SmoltcpDevice;

const NUM_TCP_SOCKETS: usize = 4;

/// Containers for smoltcp-related network configurations
pub struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],

    // Note: There is an additional socket set item required for the DHCP socket.
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_TCP_SOCKETS + 1],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
}

impl Default for NetStorage {
    fn default() -> Self {
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
pub struct TcpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 1024],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 1024],
        }
    }
}

pub fn setup(
    device: SmoltcpDevice<'static>,
    settings: &BoosterSettings,
) -> smoltcp::iface::Interface<'static, SmoltcpDevice<'static>> {
    let net_store = cortex_m::singleton!(: NetStorage = NetStorage::default()).unwrap();

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
