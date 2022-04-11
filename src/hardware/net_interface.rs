use crate::BoosterSettings;
use smoltcp_nal::smoltcp;

use super::external_mac::SmoltcpDevice;

const NUM_SOCKETS: usize = 4;

/// Containers for smoltcp-related network configurations
pub struct NetStorage {
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_SOCKETS],
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
    pub tx_storage: [u8; 4096],
    pub rx_storage: [u8; 1024],
}

impl Default for NetStorage {
    fn default() -> Self {
        Self {
            // Placeholder for the real IP address, which is initialized at runtime.
            ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
                smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
            )],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [smoltcp::iface::SocketStorage::EMPTY; NUM_SOCKETS],
            tx_storage: [0; 4096],
            rx_storage: [0; 1024],
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

    let tcp_socket = {
        let tx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut net_store.tx_storage[..]);
        let rx_buffer = smoltcp::socket::TcpSocketBuffer::new(&mut net_store.rx_storage[..]);

        smoltcp::socket::TcpSocket::new(rx_buffer, tx_buffer)
    };

    interface.add_socket(tcp_socket);

    interface
}
