//! Networking data types.
//!
//! There may a standard for embedded networking in the future, see
//! [rust-embedded issue 348] and [RFC 2832]
//!
//! This is mostly ripped directly from [w5500::net].
//!
//! [rust-embedded issue 348]: https://github.com/rust-embedded/wg/issues/348
//! [std::net]: https://doc.rust-lang.org/std/net/index.html
//! [RFC 2832]: https://github.com/rust-lang/rfcs/pull/2832
// #![deny(unsafe_code, missing_docs, warnings)]

/// MAC address struct.  Can be instantiated with `MacAddress::new`.
///
/// This is an EUI-48 MAC address (previously called MAC-48).
#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Debug, Hash, Default)]
pub struct MacAddress {
    /// Octets of the MAC address.
    pub octets: [u8; 6],
}

impl MacAddress {
    /// Creates a new EUI-48 MAC address from six eight-bit octets.
    pub const fn from_bytes(octets: [u8; 6]) -> MacAddress {
        MacAddress { octets }
    }
}

impl ::core::fmt::Display for MacAddress {
    /// String formatter for MacAddress addresses.
    fn fmt(&self, fmt: &mut ::core::fmt::Formatter) -> ::core::fmt::Result {
        write!(
            fmt,
            "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            self.octets[0],
            self.octets[1],
            self.octets[2],
            self.octets[3],
            self.octets[4],
            self.octets[5],
        )
    }
}
