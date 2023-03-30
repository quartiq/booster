pub type SmoltcpDevice<'a> = smoltcp_mac::SmoltcpDevice<'a, 'static, 8, 8>;
pub type Manager<'a, Mac> = smoltcp_mac::Manager<'a, 'static, Mac, 8, 8>;

pub struct WrappedW5500(
    pub w5500::raw_device::RawDevice<w5500::bus::FourWire<super::Spi, super::SpiCs>>,
);

#[cfg(feature = "phy_w5500")]
impl smoltcp_mac::ExternalMac for WrappedW5500 {
    type Error = ();

    fn receive_frame(&mut self, frame: &mut [u8]) -> Result<usize, Self::Error> {
        let len = self.0.read_frame(frame).unwrap();
        Ok(len)
    }

    fn send_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        self.0.write_frame(frame).unwrap();
        Ok(())
    }
}

#[cfg(feature = "phy_enc424j600")]
use enc424j600::EthPhy;

#[cfg(feature = "phy_enc424j600")]
impl smoltcp_mac::ExternalMac for enc424j600::Enc424j600<super::Spi, super::SpiCs> {
    type Error = ();

    fn receive_frame(&mut self, frame: &mut [u8]) -> Result<usize, Self::Error> {
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

    fn send_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        let mut tx_packet = enc424j600::tx::TxPacket::new();
        tx_packet.update_frame(&frame[..], frame.len());
        self.send_packet(&tx_packet).unwrap();
    }
}
