//! Smoltcp device implementation for external ethernet MACs.
//!
//! # Design
//!
//! Smoltcp must own a means to communicate with the PHY. However, smoltcp may have multiple RX/TX
//! tokens in flight. Because of this, when Smoltcp tries to consume an RX/TX token, it would have
//! to also own the means to transmit or recieve via the MAC.
//!
//! Because the `receive()` API of [smoltcp::phy::Device] requires provisioning two tokens, it's
//! not possible for each token to own a mutable reference to the underlying MAC. To get around
//! this, the MAC is not owned by smoltcp at all, but rather Smoltcp is given a software device with
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
//! # Example
//!
//! ```rust
//! struct Mac;
//!
//! impl smoltcp_mac::ExternalMac for Mac {
//!     type Error = ();
//!     fn receive_frame(&mut self, frame: &mut [u8]) -> Result<usize, Self::Error> {
//!         todo!();
//!     }
//!
//!     fn send_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
//!         todo!();
//!     }
//! }
//!
//! // Allocate static storage for ethernet frames.
//! const POOL_SIZE_BYTES: usize = core::mem::size_of::<smoltcp_mac::Frame>() * 16;
//! static mut POOL_STORAGE: [u8; POOL_SIZE_BYTES] = [0; POOL_SIZE_BYTES];
//!
//! // Construct the smoltcp device and the manager.
//! let (device, manager) = smoltcp_mac::new_default(mac, unsafe { &mut POOL_STORAGE });
//!
//! // Create the smoltcp interface and sockets.
//! let smoltcp_interface = smoltcp::iface::Interface::new(smoltcp::iface::Config::default(), &mut
//! device);
//! let mut sockets = smoltcp::socket::SocketSet::new(&[]);
//!
//! loop {
//!     manager.process().unwrap();
//!     smoltcp_interface.poll(smoltcp::time::Instant::now(), &mut device, &mut sockets).unwrap();
//! }
//!
//! ```
//!
//!
//! # Ethernet Frame Buffers
//!
//! In order to avoid large copying of ethernet frames between [SmoltcpDevice] and [Manager], the
//! ethernet frames are allocated from a global [heapless::pool::Pool]. Because of the operation of
//! the pool, buffers are not actually copied when the frames are enqueued/dequeued from the FIFOs.
//! Instead, a [heapless::pool::Box] is used, which is a proxy to the underlying `static mut`
//! buffer. This ensures that the ethernet frames are not copied when transferring data between the
//! [SmoltcpDevice] and the [Manager]
#![no_std]

use heapless::pool::Box;
use smoltcp_nal::smoltcp;

pub use heapless;
pub use smoltcp_nal;

pub trait ExternalMac {
    type Error;

    /// Receive a single ethernet frame.
    ///
    /// # Args
    /// * `frame` - The frame storage to receive the frame into.
    ///
    /// # Returns
    /// The number of bytes received into the ethernet frame. If zero, no frame was read.
    fn receive_frame(&mut self, frame: &mut [u8]) -> Result<usize, Self::Error>;

    /// Send a single ethernet frame to the MAC and transmit it.
    ///
    /// # Args
    /// * `frame` - The ethernet frame to send.
    fn send_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error>;
}

#[cfg(feature = "cortex-m")]
const DEFAULT_NUM_FRAMES: usize = 8;

// The maximum size of each ethernet frame.
const DEFAULT_MTU_SIZE: usize = 1500;

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
pub struct PooledFrame {
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

/// Smoltcp phy::Device implementation to provide to the network stack.
pub struct SmoltcpDevice<'tx, 'rx, const TX: usize, const RX: usize> {
    tx: &'tx heapless::mpmc::MpMcQueue<PooledFrame, TX>,
    rx: heapless::spsc::Consumer<'rx, PooledFrame, RX>,
    frame_pool: &'static heapless::pool::Pool<Frame>,
}

/// MAC management interface to handle frame transmission/reception manually.
pub struct Manager<'tx, 'rx, Mac: ExternalMac, const TX: usize, const RX: usize> {
    mac: Mac,
    frame_pool: &'static heapless::pool::Pool<Frame>,
    tx: &'tx heapless::mpmc::MpMcQueue<PooledFrame, TX>,
    rx: heapless::spsc::Producer<'rx, PooledFrame, RX>,
}

#[cfg(feature = "cortex-m")]
/// Construct a driver for usage with smoltcp using sensible defaults.
///
/// # Note
/// It is only safe to call this function once. Multiple calls will cause a panic.
///
/// # Args
/// * `mac` - The external MAC to use for frame transmission and reception
/// * `storage` - Raw binary storage needed for allocating ethernet frames from.
///
/// # Returns
/// (manager, device) See [`Manager::new()`]
pub fn new_default<Mac: ExternalMac>(
    mac: Mac,
    storage: &'static mut [u8],
) -> (
    SmoltcpDevice<'static, 'static, DEFAULT_NUM_FRAMES, DEFAULT_NUM_FRAMES>,
    Manager<'static, 'static, Mac, DEFAULT_NUM_FRAMES, DEFAULT_NUM_FRAMES>,
) {
    let rx_queue = cortex_m::singleton!(:
        heapless::spsc::Queue<PooledFrame, DEFAULT_NUM_FRAMES> = heapless::spsc::Queue::new())
    .unwrap();
    let tx_queue = cortex_m::singleton!(:
        heapless::mpmc::MpMcQueue<PooledFrame, DEFAULT_NUM_FRAMES> = heapless::mpmc::MpMcQueue::new())
    .unwrap();

    // Create a static frame pool to allocate buffer space for ethernet frames into.
    let frame_pool =
        cortex_m::singleton!(: heapless::pool::Pool<Frame> = heapless::pool::Pool::new()).unwrap();

    frame_pool.grow(storage);

    Manager::new(mac, frame_pool, tx_queue, rx_queue)
}

impl<'tx, 'rx, Mac: ExternalMac, const TX: usize, const RX: usize> Manager<'tx, 'rx, Mac, TX, RX> {
    /// Construct a new smoltcp device and MAC management interface.
    ///
    /// # Args
    /// * `mac` - The MAC to use for transmission and reception.
    ///
    /// # Returns
    /// A tuple of (smoltcp_device, manager) to both handle transmission/reception and generate
    /// data within the network stack.
    pub fn new(
        mac: Mac,
        pool: &'static heapless::pool::Pool<Frame>,
        tx_queue: &'tx heapless::mpmc::MpMcQueue<PooledFrame, TX>,
        rx_queue: &'rx mut heapless::spsc::Queue<PooledFrame, RX>,
    ) -> (SmoltcpDevice<'tx, 'rx, TX, RX>, Self) {
        let (rx_producer, rx_consumer) = rx_queue.split();

        (
            SmoltcpDevice {
                tx: tx_queue,
                rx: rx_consumer,
                frame_pool: pool,
            },
            Manager {
                mac,
                frame_pool: pool,
                tx: tx_queue,
                rx: rx_producer,
            },
        )
    }

    /// Must be called periodically to handle sending and receiving frames with the PHY.
    pub fn process(&mut self) -> Result<(), Mac::Error> {
        // Ingress all available frames.
        while self.ingress_frame()? {}

        // Egress all pending frames.
        while self.egress_frame()? {}

        Ok(())
    }

    /// Ingress a single frame from the MAC.
    ///
    /// # Returns
    /// True if a frame was received and enqueued. False otherwise.
    fn ingress_frame(&mut self) -> Result<bool, Mac::Error> {
        // If there's no space to enqueue read frames, don't try to receive at all.
        if !self.rx.ready() {
            return Ok(false);
        }

        let Some(mut frame) = PooledFrame::alloc(self.frame_pool) else { return Ok(false) };
        let Some(rx_frame) = &mut frame.frame else { return Ok(false) };

        rx_frame.length = self.mac.receive_frame(&mut rx_frame.payload[..])?;

        if rx_frame.length == 0 {
            return Ok(false);
        }

        // Note(unsafe): We checked that there was sufficient queue space already,
        // so this should never fail.
        unsafe {
            self.rx.enqueue_unchecked(frame);
        }

        Ok(true)
    }

    /// Egress a single ethernet frame to the MAC.
    ///
    /// # Returns
    /// True if a frame was egressed. False otherwise.
    fn egress_frame(&mut self) -> Result<bool, Mac::Error> {
        let Some(frame) = self.tx.dequeue() else { return Ok(false) };
        let Some(tx_frame) = &frame.frame else { return Ok(false) };

        self.mac.send_frame(&tx_frame.payload[..tx_frame.length])?;
        Ok(true)
    }
}

impl<'a, 'tx: 'a, 'rx: 'a, const TX: usize, const RX: usize> smoltcp::phy::Device<'a>
    for SmoltcpDevice<'tx, 'rx, TX, RX>
{
    type RxToken = RxToken;
    type TxToken = TxToken<'tx, TX>;

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
            None
        }
    }

    fn transmit(&mut self) -> Option<Self::TxToken> {
        PooledFrame::alloc(self.frame_pool).map(|frame| TxToken {
            queue: self.tx,
            frame,
        })
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
pub struct TxToken<'tx, const TX: usize> {
    frame: PooledFrame,
    queue: &'tx heapless::mpmc::MpMcQueue<PooledFrame, TX>,
}

impl<'tx, const TX: usize> smoltcp::phy::TxToken for TxToken<'tx, TX> {
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
