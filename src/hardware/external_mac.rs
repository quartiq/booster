//! Smoltcp device implementation for external ethernet MACs.

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
//! Smoltcp can allocate a frame buffer for a TX frame and populate it. When ready to transmit,
//! it then enqueues the prepared frame in the FIFO.
//!
//! Periodically, the driver for the MAC then checks the TX FIFO for any outbound frames and
//! transmits them if they are available. Similarly, it checks the MAC for any received ethernet
//! frames and enqueues them to an RX FIFO for ingression into Smoltcp.
//!
//! In this file, the [Manager] is the owner of the hardware interface to the MAC and reads and
//! writes frames to the device over SPI. It then enqueues/dequeues frames from the RX/TX FIFOs
//! respectively as frames become available.
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
const NUM_FRAMES: usize = 16;

// The size of the RX FIFO. Note that it is intentionally smaller than the number of frames to
// ensure that frame reception does not stop transmission.
const RX_SIZE: usize = NUM_FRAMES / 2;

// The size of the TX transmit queue. Note that it is intentionally smaller than the number of
// frames to ensure that transmission does not stop reception.
const TX_SIZE: usize = NUM_FRAMES / 2;

const POOL_SIZE_BYTES: usize = core::mem::size_of::<Frame>() * NUM_FRAMES;

/// Static storage for staged ethernet frames.
static mut POOL_STORAGE: [u8; POOL_SIZE_BYTES] = [0; POOL_SIZE_BYTES];

#[derive(Debug)]
pub struct Frame {
    payload: [u8; DEFAULT_MTU_SIZE],
    length: usize,
}

impl Frame {
    const fn new() -> Self {
        Self {
            payload: [0; DEFAULT_MTU_SIZE],
            length: 0,
        }
    }
}

impl core::fmt::Display for Frame {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(f, "({}) {:02x?}", self.length, &self.payload[..self.length])
    }
}

/// A convenience type for a pointed-to Frame.
///
/// # Note
/// This type can be cheaply copied among FIFOs to ensure that the underlying Frame isn't copied.
struct PooledFrame {
    frame: Option<Box<Frame>>,
    frame_pool: &'static heapless::pool::Pool<Frame>,
}

impl PooledFrame {
    /// Allocate a frame from the frame pool.
    ///
    /// # Note
    /// Frames will automatically be returned to their pool when dropped.
    ///
    /// # Args
    /// * `frame_pool` - The pool to allocate from.
    ///
    /// # Returns
    /// Some(frame) if a frame was available.
    pub fn alloc(frame_pool: &'static heapless::pool::Pool<Frame>) -> Option<Self> {
        frame_pool.alloc().map(|frame| Self {
            frame: Some(frame.init(Frame::new())),
            frame_pool,
        })
    }
}

impl Drop for PooledFrame {
    fn drop(&mut self) {
        // Automatically deallocate pooled frames when they go out of scope.
        if let Some(frame) = self.frame.take() {
            self.frame_pool.free(frame)
        }
    }
}

pub trait ExternalMac {
    /// Receive a single ethernet frame.
    ///
    /// # Args
    /// * `frame` - The frame storage to receive the frame into.
    ///
    /// # Returns
    /// True if a frame was received.
    fn receive_frame(&mut self, frame: &mut Frame) -> bool;

    /// Send a single ethernet frame to the MAC and transmit it.
    ///
    /// # Args
    /// * `frame` - The ethernet frame to send.
    fn send_frame(&mut self, frame: &Frame);
}

#[cfg(feature = "phy_w5500")]
impl ExternalMac for w5500::raw_device::RawDevice<w5500::bus::FourWire<super::Spi, super::SpiCs>> {
    fn receive_frame(&mut self, frame: &mut Frame) -> bool {
        let len = self.read_frame(&mut frame.payload[..]).unwrap();
        frame.length = len;

        len != 0
    }

    fn send_frame(&mut self, frame: &Frame) {
        self.write_frame(&frame.payload[..frame.length]).unwrap();
    }
}

#[cfg(feature = "phy_enc424j600")]
use enc424j600::EthPhy;

#[cfg(feature = "phy_enc424j600")]
impl ExternalMac for enc424j600::Enc424j600<super::Spi, super::SpiCs> {
    fn receive_frame(&mut self, frame: &mut Frame) -> bool {
        match self.recv_packet(false) {
            Ok(rx_packet) => {
                rx_packet.write_frame_to(&mut frame.payload[..]);
                frame.length = rx_packet.get_frame_length();
                frame.length != 0
            }

            Err(enc424j600::Error::NoRxPacketError) => false,

            Err(other) => {
                panic!("Unexpected MAC error: {:?}", other);
            }
        }
    }

    fn send_frame(&mut self, frame: &Frame) {
        let mut tx_packet = enc424j600::tx::TxPacket::new();
        tx_packet.update_frame(&frame.payload[..], frame.length);
        self.send_packet(&tx_packet).unwrap();
    }
}

/// Smoltcp phy::Device implementation to provide to the network stack.
pub struct SmoltcpDevice<'a> {
    tx: &'a heapless::mpmc::MpMcQueue<PooledFrame, TX_SIZE>,
    rx: heapless::spsc::Consumer<'static, PooledFrame, RX_SIZE>,
    frame_pool: &'static heapless::pool::Pool<Frame>,
}

/// MAC management interface to handle frame transmission/reception manually.
pub struct Manager<'a, Mac: ExternalMac> {
    mac: Mac,
    frame_pool: &'static heapless::pool::Pool<Frame>,
    tx: &'a heapless::mpmc::MpMcQueue<PooledFrame, TX_SIZE>,
    rx: heapless::spsc::Producer<'a, PooledFrame, RX_SIZE>,
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
        let rx_queue = cortex_m::singleton!(:
            heapless::spsc::Queue<PooledFrame, RX_SIZE> = heapless::spsc::Queue::new())
        .unwrap();
        let tx_queue = cortex_m::singleton!(:
            heapless::mpmc::MpMcQueue<PooledFrame, TX_SIZE> = heapless::mpmc::MpMcQueue::new())
        .unwrap();
        let (rx_producer, rx_consumer) = rx_queue.split();

        // Create a static frame pool to allocate buffer space for ethernet frames into.
        let frame_pool =
            cortex_m::singleton!(: heapless::pool::Pool<Frame> = heapless::pool::Pool::new())
                .unwrap();

        // Note(unsafe): This function is only called once to access the frame pool, so the static
        // storage is only accessible once. It is intentionally not placed in a singleton to avoid
        // allocating initial state on the stack.
        frame_pool.grow(unsafe { &mut POOL_STORAGE });

        (
            SmoltcpDevice {
                tx: tx_queue,
                rx: rx_consumer,
                frame_pool,
            },
            Manager {
                mac,
                frame_pool,
                tx: tx_queue,
                rx: rx_producer,
            },
        )
    }

    /// Must be called periodically to handle sending and receiving frames with the PHY.
    pub fn process(&mut self) {
        // Ingress all available frames.
        while self.ingress_frame() {}

        // Egress all pending frames.
        while self.egress_frame() {}
    }

    /// Ingress a single frame from the MAC.
    ///
    /// # Returns
    /// True if a frame was received and enqueued. False otherwise.
    fn ingress_frame(&mut self) -> bool {
        // If there's no space to enqueue read frames, don't try to receive at all.
        if !self.rx.ready() {
            log::warn!("RX queue full");
            return false;
        }

        match PooledFrame::alloc(self.frame_pool) {
            Some(mut frame) => {
                // Note(unwrap): Frame reception should never fail.
                if !self.mac.receive_frame(frame.frame.as_mut().unwrap()) {
                    return false;
                }

                // Note(unsafe): We checked that there was sufficient queue space already,
                // so this should never fail.
                unsafe {
                    self.rx.enqueue_unchecked(frame);
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
    fn egress_frame(&mut self) -> bool {
        // Egress as many frames from the queue as possible.
        match self.tx.dequeue() {
            Some(frame) => {
                self.mac.send_frame(frame.frame.as_ref().unwrap());
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
        if let Some(tx_frame) = PooledFrame::alloc(self.frame_pool) {
            self.rx.dequeue().map(|rx_frame| {
                (
                    RxToken(rx_frame),
                    TxToken {
                        frame: tx_frame,
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
        if let Some(frame) = PooledFrame::alloc(self.frame_pool) {
            Some(TxToken {
                queue: self.tx,
                frame,
            })
        } else {
            log::warn!("Failed to alloc TxToken");
            None
        }
    }
}

/// A single token representing a received ethernet frame.
pub struct RxToken(PooledFrame);

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(
        mut self,
        _timestamp: smoltcp::time::Instant,
        f: F,
    ) -> Result<R, smoltcp::Error>
    where
        F: FnOnce(&mut [u8]) -> Result<R, smoltcp::Error>,
    {
        // Get the ethernet frame data and provide it to Smoltcp for processing.
        let frame = self.0.frame.as_mut().unwrap();
        let len = frame.length;
        f(&mut frame.payload[..len])
    }
}

/// A single token representing an allocated frame buffer.
pub struct TxToken<'a> {
    frame: PooledFrame,
    queue: &'a heapless::mpmc::MpMcQueue<PooledFrame, TX_SIZE>,
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
            frame: mut pooled_frame,
            queue,
        } = self;

        // Provide the frame buffer to Smoltcp to populate the data fields.
        let frame = pooled_frame.frame.as_mut().unwrap();
        assert!(len <= frame.payload.len());
        frame.length = len;
        let result = f(&mut frame.payload[..len]);

        // Enqueue the frame for transmission by the MAC.
        match queue.enqueue(pooled_frame) {
            Ok(_) => result,

            // We couldn't enqueue the frame.
            Err(_) => Err(smoltcp::Error::Exhausted),
        }
    }
}
