use super::SPI;
use crate::AsmDelay;
use crate::BoosterSettings;
use crate::NetworkStack;
use stm32f4xx_hal as hal;

/// Containers for smoltcp-related network configurations
pub struct NetStorage {
    pub socket_storage: [Option<smoltcp::socket::SocketSetItem<'static, 'static>>; 1],
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],
    pub neighbor_cache: [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
    pub tx_storage: [u8; 4096],
    pub rx_storage: [u8; 1024],
}

static mut NET_STORE: NetStorage = NetStorage {
    // Placeholder for the real IP address, which is initialized at runtime.
    ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
        smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
    )],
    neighbor_cache: [None; 8],
    routes_cache: [None; 8],
    socket_storage: [None; 1],
    tx_storage: [0; 4096],
    rx_storage: [0; 1024],
};

type Ethernet = NetworkStack<'static, 'static, 'static>;
type EthSpiInterface =
    enc424j600::SpiEth<SPI, hal::gpio::gpioa::PA4<hal::gpio::Output<hal::gpio::PushPull>>>;

pub fn setup(
    mut enc424j600: EthSpiInterface,
    settings: &BoosterSettings,
    mut delay: AsmDelay,
) -> (Ethernet, [u8; 6]) {
    use enc424j600::EthController;
    use smoltcp as net;

    match enc424j600.init_dev(&mut delay) {
        Ok(_) => {}
        Err(_) => {
            panic!("ENC424J600 PHY initialization failed");
        }
    }

    let mut eth_mac_addr: [u8; 6] = [0; 6];
    enc424j600.read_from_mac(&mut eth_mac_addr).unwrap();

    // Init Rx/Tx buffers
    enc424j600.init_rxbuf().unwrap();
    enc424j600.init_txbuf().unwrap();

    let eth_iface = unsafe {
        let device = enc424j600::smoltcp_phy::SmoltcpDevice::new(enc424j600);

        // TODO: Restore IP config from EEPROM
        NET_STORE.ip_addrs[0] = {
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
            // TODO: Restore gateway config from EEPROM
            let gateway = net::wire::Ipv4Address::from_bytes(&settings.gateway().octets());

            let mut routes = net::iface::Routes::new(&mut NET_STORE.routes_cache[..]);
            routes.add_default_ipv4_route(gateway).unwrap();
            routes
        };

        let neighbor_cache = net::iface::NeighborCache::new(&mut NET_STORE.neighbor_cache[..]);

        net::iface::EthernetInterfaceBuilder::new(device)
            .ethernet_addr(net::wire::EthernetAddress(eth_mac_addr))
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut NET_STORE.ip_addrs[..])
            .routes(routes)
            .finalize()
    };

    let socket_set = unsafe {
        let mut sockets = net::socket::SocketSet::new(&mut NET_STORE.socket_storage[..]);

        let mut tcp_socket = {
            let tx_buffer = net::socket::TcpSocketBuffer::new(&mut NET_STORE.tx_storage[..]);
            let rx_buffer = net::socket::TcpSocketBuffer::new(&mut NET_STORE.rx_storage[..]);

            net::socket::TcpSocket::new(rx_buffer, tx_buffer)
        };
        tcp_socket.set_keep_alive(Some(net::time::Duration::from_millis(1000)));

        let _handle = sockets.add(tcp_socket);
        sockets
    };

    (NetworkStack::new(eth_iface, socket_set), eth_mac_addr)
}
