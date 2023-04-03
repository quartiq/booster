use super::Mac;
use enc424j600::EthPhy;

impl smoltcp_mac::ExternalMac for Mac {
    type Error = ();

    fn receive_frame(&mut self, frame: &mut [u8]) -> Result<usize, Self::Error> {
        let len = match self {
            Mac::W5500(mac) => mac.read_frame(frame).unwrap(),
            Mac::Enc424j600(mac) => match mac.recv_packet(false) {
                Ok(rx_packet) => {
                    rx_packet.write_frame_to(&mut frame[..]);
                    rx_packet.get_frame_length()
                }

                Err(enc424j600::Error::NoRxPacketError) => 0,

                Err(other) => {
                    panic!("Unexpected MAC error: {:?}", other);
                }
            },
        };

        Ok(len)
    }

    fn send_frame(&mut self, frame: &[u8]) -> Result<(), Self::Error> {
        match self {
            Mac::W5500(mac) => {
                mac.write_frame(frame).unwrap();
            }
            Mac::Enc424j600(mac) => {
                let mut tx_packet = enc424j600::tx::TxPacket::new();
                tx_packet.update_frame(&frame[..], frame.len());
                mac.send_packet(&tx_packet).unwrap();
            }
        }
        Ok(())
    }
}
