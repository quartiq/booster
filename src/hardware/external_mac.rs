//! Smoltcp device implementation for external ethernet MACs.
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
//!
//! # Design
//!
//! ## Ownership Issues
//!
//! Smoltcp must own a means to communicate with the PHY. However, smoltcp may have multiple RX/TX
//! tokens in flight. Because of this, when Smoltcp tries to consume an RX/TX token, it would have
//! to also own the means to transmit or review via the MAC.
//!
//! Because the `receive()` API of [smoltcp::phy::Device] requires provisioning two tokens, it's
//! not possible for each token to own a mutable reference to the underlying MAC. To get around
//! this, the MAC is not owned by smoltp at all, but rather Smoltcp is given a software device with
//! RX and TX FIFOs to the driver that handles ingress and egress on the MAC. With this design,
//! Smoltcp can allocate a packet buffer for a TX packet and populate it. When ready to transmit,
//! it then enqueues the prepared packet in the FIFO.
//!
//! Periodically, the driver for the MAC then checks the TX FIFO for any outbound packets and
//! transmits them if they are available. Similarly, it checks the MAC for any received ethernet
//! frames and enqueues them to an RX FIFO for ingression into Smoltcp.
//!
//! In this file, the [Manager] is the owner of the hardware interface to the MAC and reads and
//! writes frames to the device over SPI. It then enqueues/dequeues frames from the RX/TX FIFOs
//! respectively as packets become available.
//!
//! Similarly, the [SmoltcpDevice] is the software construct with the RX/TX FIFO endpoints that can
//! be passed to Smoltcp's interface and implements [smoltcp::phy::Device].
//!
//!
//! ## Ethernet Frame Buffers
//!
//! In order to avoid large copying of ethernet frames between [SmoltcpDevice] and [Manager], the
//! ethernet frames are allocated from a global [heapless::pool::Pool]. Because of the operation of
//! the pool, buffers are not actually copied when the frames are enqueued/dequeued from the FIFOs.
//! Instead, a [heapless::pool::Box] is used, which is a proxy to the underlying `static mut`
//! buffer. This ensures that the ethernet frames are not copied when transferring data between the
//! [SmoltcpDevice] and the [Manager]
use heapless::pool::Box;
use smoltcp_nal::smoltcp;

// The maximum size of each ethernet frame.
const DEFAULT_MTU_SIZE: usize = 1500;

// The number of ethernet frame buffers to maintain in RAM.
const NUM_PACKETS: usize = 16;

// The size of the RX FIFO. Note that it is intentionally smaller than the number of packets to
// ensure that packet reception does not start packet transmission.
const RX_SIZE: usize = NUM_PACKETS / 2;

// The size of the TX transmit queue. Note that it is intentionally smaller than the number of
// packets to ensure that packet transmission does not start reception.
const TX_SIZE: usize = NUM_PACKETS / 2;

const POOL_SIZE_BYTES: usize = core::mem::size_of::<Packet>() * NUM_PACKETS;

/// Static storage for staged ethernet frames.
static mut POOL_STORAGE: [u8; POOL_SIZE_BYTES] = [0; POOL_SIZE_BYTES];

#[derive(Debug)]
pub struct Packet {
    payload: [u8; DEFAULT_MTU_SIZE],
    length: usize,
}

impl Packet {
    const fn new() -> Self {
        Self {
            payload: [0; DEFAULT_MTU_SIZE],
            length: 0,
        }
    }
}

impl core::fmt::Display for Packet {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "({}) {:02x?}", self.length, &self.payload[..self.length])
    }
}

/// A convenience type for a pointed-to packet.
///
/// # Note
/// This type can be cheaply copied among FIFOs to ensure that the underlying Packet isn't copied.
struct PooledPacket {
    packet: Option<Box<Packet>>,
    packet_pool: &'static heapless::pool::Pool<Packet>,
}

impl PooledPacket {
    /// Allocate a packet from the packet pool.
    ///
    /// # Note
    /// Packets will automatically be returned to their pool when dropped.
    ///
    /// # Args
    /// * `packet_pool` - The pool to allocate from.
    ///
    /// # Returns
    /// Some(packet) if a packet was available.
    pub fn alloc(packet_pool: &'static heapless::pool::Pool<Packet>) -> Option<Self> {
        packet_pool.alloc().map(|packet| Self {
            packet: Some(packet.init(Packet::new())),
            packet_pool,
        })
    }
}

impl Drop for PooledPacket {
    fn drop(&mut self) {
        // Automatically deallocate pooled packets when they go out of scope.
        if let Some(packet) = self.packet.take() {
            self.packet_pool.free(packet)
        }
    }
}

pub trait ExternalMac {
    /// Receive a single ethernet frame.
    ///
    /// # Args
    /// * `packet` - The packet storage to receive the frame into.
    ///
    /// # Returns
    /// True if a frame was received.
    fn receive_packet(&mut self, packet: &mut Packet) -> bool;

    /// Send a single ethernet frame to the MAC and transmit it.
    ///
    /// # Args
    /// * `packet` - The ethernet frame to send.
    fn send_packet(&mut self, packet: &Packet);
}

#[cfg(feature = "phy_w5500")]
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

        // Note(unsafe): This function is only called once to access the packet pool, so the static
        // storage is only accessible once. It is intentionally not placed in a singleton to avoid
        // allocating initial state on the stack.
        packet_pool.grow(unsafe { &mut POOL_STORAGE });

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
        // Ingress all available packets.
        while self.ingress_packet() {}

        // Egress all pending packets.
        while self.egress_packet() {}
    }

    /// Ingress a single frame from the MAC.
    ///
    /// # Returns
    /// True if a frame was received and enqueued. False otherwise.
    fn ingress_packet(&mut self) -> bool {
        // If there's no space to enqueue read packets, don't try to receive at all.
        if !self.rx.ready() {
            log::warn!("RX queue full");
            return false;
        }

        match PooledPacket::alloc(self.packet_pool) {
            Some(mut packet) => {
                // Note(unwrap): Packet reception should never fail.
                if !self.mac.receive_packet(packet.packet.as_mut().unwrap()) {
                    return false;
                }

                // Note(unsafe): We checked that there was sufficient queue space already,
                // so this should never fail.
                unsafe {
                    self.rx.enqueue_unchecked(packet);
                }

                true
            }
            None => {
                log::warn!("RX alloc failed");
                false
            }
        }
    }

    /// Egress a single ethernet frame to the MAC.
    ///
    /// # Returns
    /// True if a frame was egressed. False otherwise.
    fn egress_packet(&mut self) -> bool {
        // Egress as many packets from the queue as possible.
        match self.tx.dequeue() {
            Some(packet) => {
                self.mac.send_packet(packet.packet.as_ref().unwrap());
                true
            }
            None => false,
        }
    }
}

impl<'a, 'b: 'a> smoltcp::phy::Device<'a> for SmoltcpDevice<'b> {
    type RxToken = RxToken;
    type TxToken = TxToken<'a>;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.max_transmission_unit = DEFAULT_MTU_SIZE;
        caps.medium = smoltcp::phy::Medium::Ethernet;
        caps
    }

    fn receive(&mut self) -> Option<(Self::RxToken, Self::TxToken)> {
        if let Some(tx_packet) = PooledPacket::alloc(self.packet_pool) {
            self.rx.dequeue().map(|rx_packet| {
                (
                    RxToken(rx_packet),
                    TxToken {
                        packet: tx_packet,
                        queue: self.tx,
                    },
                )
            })
        } else {
            log::warn!("Failed to alloc RxTxToken");
            None
        }
    }

    fn transmit(&mut self) -> Option<Self::TxToken> {
        if let Some(packet) = PooledPacket::alloc(self.packet_pool) {
            Some(TxToken {
                queue: self.tx,
                packet,
            })
        } else {
            log::warn!("Failed to alloc TxToken");
            None
        }
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
        assert!(len <= packet.payload.len());
        packet.length = len;
        let result = f(&mut packet.payload[..len]);

        match queue.enqueue(pooled_packet) {
            Ok(_) => result,

            // We couldn't enqueue the packet.
            Err(_) => Err(smoltcp::Error::Exhausted),
        }
    }
}
