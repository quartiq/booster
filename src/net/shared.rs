//! Booster shared network stack management
//!
//! # Note
//! Portions of this file are taken from the
//! [`smoltcp-nal`](https://github.com/quartiq/smoltcp-nal/blob/main/src/shared.rs)
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.
use minimq::embedded_nal;
use shared_bus::{AtomicCheckMutex, BusMutex};

/// A manager for a shared network stack.
pub struct NetworkManager<S> {
    mutex: AtomicCheckMutex<S>,
}

/// A basic proxy that references a shared network stack.
pub struct NetworkStackProxy<'a, S> {
    mutex: &'a AtomicCheckMutex<S>,
}

impl<'a, S> NetworkStackProxy<'a, S> {
    /// Using the proxy, access the underlying network stack directly.
    ///
    /// # Args
    /// * `f` - A closure which will be provided the network stack as an argument.
    ///
    /// # Returns
    /// Any value returned by the provided closure
    pub fn lock<R, F: FnOnce(&mut S) -> R>(&mut self, f: F) -> R {
        self.mutex.lock(|stack| f(stack))
    }
}

// A simple forwarding macro taken from the `embedded-nal` to forward the embedded-nal API into the
// proxy structure.
macro_rules! forward {
    ($func:ident($($v:ident: $IT:ty),*) -> $T:ty) => {
        fn $func(&mut self, $($v: $IT),*) -> $T {
            self.mutex.lock(|stack| stack.$func($($v),*))
        }
    }
}

// Implement a TCP stack for the proxy if the underlying network stack implements it.
impl<'a, S> embedded_nal::TcpClientStack for NetworkStackProxy<'a, S>
where
    S: embedded_nal::TcpClientStack,
{
    type TcpSocket = S::TcpSocket;
    type Error = S::Error;

    forward! {socket() -> Result<S::TcpSocket, S::Error>}
    forward! {connect(socket: &mut S::TcpSocket, remote: embedded_nal::SocketAddr) -> embedded_nal::nb::Result<(), S::Error>}
    forward! {is_connected(socket: &S::TcpSocket) -> Result<bool, S::Error>}
    forward! {send(socket: &mut S::TcpSocket, buffer: &[u8]) -> embedded_nal::nb::Result<usize, S::Error>}
    forward! {receive(socket: &mut S::TcpSocket, buffer: &mut [u8]) -> embedded_nal::nb::Result<usize, S::Error>}
    forward! {close(socket: S::TcpSocket) -> Result<(), S::Error>}
}

impl<S> NetworkManager<S> {
    /// Construct a new manager for a shared network stack
    ///
    /// # Args
    /// * `stack` - The network stack that is being shared.
    pub fn new(stack: S) -> Self {
        Self {
            mutex: AtomicCheckMutex::create(stack),
        }
    }

    /// Acquire a proxy to the shared network stack.
    ///
    /// # Returns
    /// A proxy that can be used in place of the network stack. Note the requirements of
    /// concurrency listed in the description of this file for usage.
    pub fn acquire_stack(&'_ self) -> NetworkStackProxy<'_, S> {
        NetworkStackProxy { mutex: &self.mutex }
    }
}
