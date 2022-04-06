use crate::{
    EthPhy, RAW_FRAME_LENGTH_MAX,
    rx::RxPacket, tx::TxPacket,
    Enc424j600,
};

struct PooledPacket<'a> {
    packet: Box<Packet>,
    packet_pool: &'a heapless::pool::Pool<Packet>,
}

impl<'a> PooledPacket<'a> {
    pub fn new(packet_pool: &'a heapless::pool::Pool<Packet>) -> Self {
        let packet = packet_pool.alloc().unwrap();

        Self { packet, packet_pool }
    }
}

impl<'a> Drop for PooledPacket<'a> {
    fn drop(&self) {
        self.packet_pool.free(self.packet)
    }
}

pub trait ExternalMac {
    type Error;
    fn receive_packet(&mut self, packet: &mut Packet) -> Result<bool, Self::Error>;
    fn send_packet(&mut self, packet: &Packet) -> Result<(), Self::Error>;
}

/// Smoltcp phy::Device implementation to provide to the network stack.
pub struct SmoltcpDevice<'a, const TX_N: usize, const RX_N: usize> {
    tx: &'a heapless::mpmc::MpMcQueue<PooledPacket<'a>, TX_N>,
    rx: heapless::spsc::Consumer<'a , PooledPacket<'a>, RX_N>,
    packet_pool: &'a heapless::pool::Pool<Packet>,
}

/// ENC424J600 PHY management interface to handle packet ingress/egress manually.
pub struct Manager<'a, Mac: ExternalMac, const TX_N: usize, const RX_N: usize> {
    mac: Mac,
    pending_transmission: Option<PooledPacket>,
    packet_pool: &'a heapless::pool::Pool<PooledPacket<'a>>,
    tx: &'a heapless::mpmc::MpMcQueue<PooledPacket<'a>, TX_N>,
    rx: heapless::spsc::MpMcQueue<PooledPacket<'a>, RX_N>,
}

impl<'a, Mac: ExternalMac, const TX_N: usize, const RX_N: usize> Manager<'a, Mac, TX_N, RX_N> {
    /// Construct a new smoltcp device and MAC management interface.
    ///
    /// # Args
    /// * `mac` - The MAC to use for transmission and reception.
    ///
    /// # Returns
    /// A tuple of (smoltcp_device, manager) to both handle transmission/reception and generate
    /// data within the network stack.
    pub fn new(mac: Mac) -> (Manager<'a, TX_N, RX_N>) {

        let rx_queue = cortex_m::singleton!(: heapless::spsc::Queue<Box<Packet>, RX_N> = heapless::spsc::Queue::new()).unwrap();
        let tx_queue = cortex_m::singleton!(: heapless::mpmc::MpmcQueue<Box<Packet>, TX_N> = heapless::mpmc::MpmcQueue::new()).unwrap();
        let (rx_consumer, rx_producer) = rx_queue.split();

        // Create a static packet pool to allocate buffer space for ethernet frames into.
        let packet_pool = cortex_m::singleton!(: heapless::pool::Pool<Packet> = heapless::pool::Pool::new()).unwrap();
        let pool_storage = cortex_m::singleton!(: [u8; {core::mem::size_of<Packet>() * (RX_N + TX_N)}] = [0; _] ).unwrap();
        packet_pool.grow(pool_storage);

        (
            SmoltcpDevice {
                tx: tx_queue,
                rx: rx_consumer,
                packet_pool,
            },
            Manager {
                mac,
                pending_transmission: None,
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

        let mut packet = PooledPacket::new(self.packet_pool);

        // Note(unwrap): Packet reception should never fail.
        if self.mac.receive_packet(&mut packet.packet).unwrap() {
            // Note(unwrap): We checked that there was sufficient queue space already, so this
            // should never fail.
            self.rx.enqueue(packet).unwrap();
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

    fn egress_packet(&mut self, packet: PooledPacket<'a>) -> Result<(), ()> {
        if self.mac.send_packet(&packet.packet).is_err() {

            // If we can't send the current packet, we have to store it until we can in the future.
            self.pending_transmission.replace(packet);
            return Err(());
        }

        Ok(())
    }
}

impl<'a, 'b: 'a, const TX_N: usize, const RX_N: usize> smoltcp::phy::Device<'a> for SmoltcpDevice<'b, TX_N, RX_N> {
    type RxToken = RxToken;
    type TxToken = TxToken<'a, TX_N>;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.max_transmission_unit = RAW_FRAME_LENGTH_MAX;
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

pub struct RxToken<'a>(PooledPacket<'a>);

impl<'a> smoltcp::phy::RxToken for RxToken<'a> {
    fn consume<R, F>(mut self, _timestamp: smoltcp::time::Instant, f: F) -> Result<R, smoltcp::Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, smoltcp::Error>,
    {
        // TODO: Implement
        let len = self.0.get_frame_length();
        let frame = self.0.get_mut_frame();
        f(&mut frame[..len])
    }
}

pub struct TxToken<'a, const N: usize> {
    queue: &'a heapless::mpmc::MpMcQueue<PooledPacket<'a>, N>,
    pool: &'a heapless::pool::Pool<PooledPacket<'a>>,
}

impl<'a, const N: usize> smoltcp::phy::TxToken for TxToken<'a, N> {
    fn consume<R, F>(
        self,
        _timestamp: smoltcp::time::Instant,
        len: usize,
        f: F,
    ) -> Result<R, smoltcp::Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, smoltcp::Error>,
    {
        // TODO: Implement
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
