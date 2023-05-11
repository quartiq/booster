use super::Mac;
use enc424j600::EthPhy;

use smoltcp_nal::smoltcp;

impl smoltcp::phy::Device for Mac {
    type RxToken<'a> = RxToken where Self: 'a;
    type TxToken<'a> = TxToken<'a> where Self: 'a;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut caps = smoltcp::phy::DeviceCapabilities::default();
        caps.medium = smoltcp::phy::Medium::Ethernet;
        caps
    }

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        let mut buffer = [0u8; 1500];
        let len = match self {
            Mac::W5500(w5500) => w5500.read_frame(&mut buffer[..]).unwrap(),
            Mac::Enc424j600(mac) => match mac.recv_packet(false) {
                Ok(rx_packet) => {
                    rx_packet.write_frame_to(&mut buffer[..]);
                    rx_packet.get_frame_length()
                }
                Err(enc424j600::Error::NoRxPacketError) => 0,

                Err(_other) => panic!("{}", "Unexpected MAC error: {_other:?}"),
            },
        };

        if len != 0 {
            Some((
                RxToken {
                    frame_buffer: buffer,
                    length: len,
                },
                TxToken { mac: self },
            ))
        } else {
            None
        }
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        Some(TxToken { mac: self })
    }
}

pub struct RxToken {
    frame_buffer: [u8; 1500],
    length: usize,
}

impl smoltcp::phy::RxToken for RxToken {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        f(&mut self.frame_buffer[..self.length])
    }
}

pub struct TxToken<'a> {
    mac: &'a mut Mac,
}

impl<'a> smoltcp::phy::TxToken for TxToken<'a> {
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let mut buffer = [0u8; 1500];
        let result = f(&mut buffer);
        match self.mac {
            Mac::W5500(mac) => {
                mac.write_frame(&buffer[..len]).unwrap();
            }
            Mac::Enc424j600(mac) => {
                let mut tx_packet = enc424j600::tx::TxPacket::new();
                tx_packet.update_frame(&buffer[..len], len);
                mac.send_packet(&tx_packet).unwrap();
            }
        }

        result
    }
}
