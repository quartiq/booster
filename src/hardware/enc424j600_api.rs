use super::{Enc424j600, Ethernet};
use crate::BoosterSettings;
use cortex_m::peripheral::DWT;
use embedded_hal::blocking::delay::DelayUs;
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
