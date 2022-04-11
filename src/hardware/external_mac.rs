//! Smoltcp device implementation for external ethernet MACs.
//!
//! # Design
//! TODO
use heapless::pool::Box;
use smoltcp_nal::smoltcp;

pub const MAX_MTU_SIZE: usize = 1024;
pub const NUM_PACKETS: usize = 16;
pub const RX_SIZE: usize = NUM_PACKETS;
pub const TX_SIZE: usize = NUM_PACKETS;

pub const POOL_SIZE_BYTES: usize = core::mem::size_of::<Packet>() * NUM_PACKETS;

pub struct Packet {
    payload: [u8; MAX_MTU_SIZE],
    length: usize,
}

impl Default for Packet {
    fn default() -> Self {
        Self {
            payload: [0; MAX_MTU_SIZE],
            length: 0,
        }
    }
}

struct PooledPacket {
    packet: Option<Box<Packet>>,
    packet_pool: &'static heapless::pool::Pool<Packet>,
}

impl PooledPacket {
    pub fn alloc(packet_pool: &'static heapless::pool::Pool<Packet>) -> Option<Self> {
        packet_pool.alloc().map(|packet| Self {
            packet: Some(packet.init(Packet::default())),
            packet_pool,
        })
    }
}

impl Drop for PooledPacket {
    fn drop(&mut self) {
        if let Some(packet) = self.packet.take() {
            self.packet_pool.free(packet)
        }
    }
}

pub trait ExternalMac {
    fn receive_packet(&mut self, packet: &mut Packet) -> bool;
    fn send_packet(&mut self, packet: &Packet);
}

impl ExternalMac for w5500::raw_device::RawDevice<w5500::bus::FourWire<super::Spi, super::SpiCs>> {
    fn receive_packet(&mut self, packet: &mut Packet) -> bool {
        let len = self.read_frame(&mut packet.payload[..]).unwrap();
        packet.length = len;

        len != 0
    }

    fn send_packet(&mut self, packet: &Packet) {
        self.write_frame(&packet.payload[..packet.length]).unwrap();
    }
}

/// Smoltcp phy::Device implementation to provide to the network stack.
pub struct SmoltcpDevice<'a> {
    tx: &'a heapless::mpmc::MpMcQueue<PooledPacket, TX_SIZE>,
    rx: heapless::spsc::Consumer<'static, PooledPacket, RX_SIZE>,
    packet_pool: &'static heapless::pool::Pool<Packet>,
}

/// ENC424J600 PHY management interface to handle packet ingress/egress manually.
pub struct Manager<'a, Mac: ExternalMac> {
    mac: Mac,
    packet_pool: &'static heapless::pool::Pool<Packet>,
    tx: &'a heapless::mpmc::MpMcQueue<PooledPacket, TX_SIZE>,
    rx: heapless::spsc::Producer<'a, PooledPacket, RX_SIZE>,
}

impl<'a, Mac: ExternalMac> Manager<'a, Mac> {
    /// Construct a new smoltcp device and MAC management interface.
    ///
    /// # Args
    /// * `mac` - The MAC to use for transmission and reception.
    ///
    /// # Returns
    /// A tuple of (smoltcp_device, manager) to both handle transmission/reception and generate
    /// data within the network stack.
    pub fn new(mac: Mac) -> (SmoltcpDevice<'a>, Manager<'a, Mac>) {
        let rx_queue = cortex_m::singleton!(: heapless::spsc::Queue<PooledPacket, RX_SIZE> = heapless::spsc::Queue::new()).unwrap();
        let tx_queue = cortex_m::singleton!(: heapless::mpmc::MpMcQueue<PooledPacket, TX_SIZE> = heapless::mpmc::MpMcQueue::new()).unwrap();
        let (rx_producer, rx_consumer) = rx_queue.split();

        // Create a static packet pool to allocate buffer space for ethernet frames into.
        let packet_pool =
            cortex_m::singleton!(: heapless::pool::Pool<Packet> = heapless::pool::Pool::new())
                .unwrap();
        let pool_storage =
            cortex_m::singleton!(: [u8; POOL_SIZE_BYTES] = [0; POOL_SIZE_BYTES] ).unwrap();
        packet_pool.grow(pool_storage);

        (
            SmoltcpDevice {
                tx: tx_queue,
                rx: rx_consumer,
                packet_pool,
            },
            Manager {
                mac,
                packet_pool,
                tx: tx_queue,
                rx: rx_producer,
            },
        )
    }

    /// Must be called periodically to handle sending and receiving frames with the PHY.
    pub fn process(&mut self) {
        self.ingress();
        self.egress();
    }

    fn ingress(&mut self) {
        // If there's no space to enqueue read packets, don't try to receive at all.
        if !self.rx.ready() {
            return;
        }

        if let Some(mut packet) = PooledPacket::alloc(&self.packet_pool) {
            // Note(unwrap): Packet reception should never fail.
            if self.mac.receive_packet(packet.packet.as_mut().unwrap()) {
                // Note(unsafe): We checked that there was sufficient queue space already, so this
                // should never fail.
                unsafe {
                    self.rx.enqueue_unchecked(packet);
                }
            }
        }
    }

    fn egress(&mut self) {
        // Egress as many packets from the queue as possible.
        while let Some(packet) = self.tx.dequeue() {
            if self.egress_packet(packet).is_err() {
                return;
            }
        }
    }

    fn egress_packet(&mut self, packet: PooledPacket) -> Result<(), ()> {
        Ok(self.mac.send_packet(packet.packet.as_ref().unwrap()))
    }
}

impl<'a, 'b: 'a> smoltcp::phy::Device<'a> for SmoltcpDevice<'b> {
    type RxToken = RxToken;
    type TxToken = TxToken<'a>;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.max_transmission_unit = MAX_MTU_SIZE;
        caps
    }

    fn receive(&mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        if let Some(tx_packet) = PooledPacket::alloc(self.packet_pool) {
            self.rx.dequeue().map(|rx_packet| {
                (
                    RxToken(rx_packet),
                    TxToken {
                        packet: tx_packet,
                        queue: &self.tx,
                    },
                )
            })
        } else {
            None
        }
    }

    fn transmit(&mut self) -> Option<Self::TxToken> {
        PooledPacket::alloc(self.packet_pool).map(|packet| TxToken {
            queue: self.tx,
            packet,
        })
    }
}

pub struct RxToken(PooledPacket);

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(
        mut self,
        _timestamp: smoltcp::time::Instant,
        f: F,
    ) -> Result<R, smoltcp::Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, smoltcp::Error>,
    {
        let packet = self.0.packet.as_mut().unwrap();
        let len = packet.length;
        f(&mut packet.payload[..len])
    }
}

pub struct TxToken<'a> {
    packet: PooledPacket,
    queue: &'a heapless::mpmc::MpMcQueue<PooledPacket, TX_SIZE>,
}

impl<'a> smoltcp::phy::TxToken for TxToken<'a> {
    fn consume<R, F>(
        self,
        _timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> Result<R, smoltcp::Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, smoltcp::Error>,
    {
        let TxToken {
            packet: mut pooled_packet,
            queue,
        } = self;

        let packet = pooled_packet.packet.as_mut().unwrap();
        assert!(len <= packet.length);
        packet.length = len;
        let result = f(&mut packet.payload[..len]);

        match queue.enqueue(pooled_packet) {
            Ok(_) => result,

            // We couldn't enqueue the packet.
            Err(_) => Err(smoltcp::Error::Exhausted),
        }
    }
}
