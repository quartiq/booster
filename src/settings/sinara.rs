//! Booster NGFW NVM Sinara settings

use crate::Error;
use core::convert::TryInto;

/// The sinara configuration board ID.
pub enum BoardId {
    Mainboard = 21,
    RfChannel = 22,
}

/// Serialization tool for serializing sinara configurations.
///
/// # Note
/// Sinara configurations use a big-endian encoding, so we can't rely on postcard or other
/// convenience libraries for serde functionality.
struct Serializer<'s> {
    input: &'s mut [u8],
    index: usize,
}

impl<'s> Serializer<'s> {
    /// Construct a new serializer into `input`.
    pub fn new(input: &'s mut [u8]) -> Self {
        Self { input, index: 0 }
    }

    /// Serialize data into the serialization buffer.
    ///
    /// # Args
    /// * `data` - The data to serialize.
    pub fn serialize_data(&mut self, data: &[u8]) -> Result<(), Error> {
        let remainder = &mut self.input[self.index..];
        if remainder.len() >= data.len() {
            let (head, _) = remainder.split_at_mut(data.len());
            head.copy_from_slice(data);
            self.index += data.len();
            Ok(())
        } else {
            Err(Error::Invalid)
        }
    }

    /// Serialize a byte of data.
    pub fn serialize_u8(&mut self, value: u8) -> Result<(), Error> {
        self.serialize_data(&[value])
    }

    /// Serialize 16-bit integer.
    pub fn serialize_u16(&mut self, value: u16) -> Result<(), Error> {
        self.serialize_data(&value.to_be_bytes())
    }

    /// Serialize 32-bit integer.
    pub fn serialize_u32(&mut self, value: u32) -> Result<(), Error> {
        self.serialize_data(&value.to_be_bytes())
    }

    /// Finish the serialization process.
    ///
    /// # Returns
    /// The serialized buffer.
    pub fn finish(self) -> Result<&'s [u8], Error> {
        if self.index != self.input.len() {
            Err(Error::Invalid)
        } else {
            Ok(self.input)
        }
    }
}

/// Utility for deserializing sinara configuration from raw data.
struct Deserializer<'de> {
    input: &'de [u8],
}

impl<'de> Deserializer<'de> {
    /// Construct a deserializer from the provided `input` data.
    pub fn new(input: &'de [u8]) -> Self {
        Self { input }
    }

    fn try_take(&mut self, count: usize) -> Result<&'de [u8], Error> {
        if self.input.len() >= count {
            let (head, tail) = self.input.split_at(count);
            self.input = tail;
            Ok(head)
        } else {
            Err(Error::Invalid)
        }
    }

    /// Deserialize a single byte from the serialized data.
    pub fn deserialize_u8(&mut self) -> Result<u8, Error> {
        Ok(self.try_take(1)?[0])
    }

    /// Deserialize a 16-bit integer from the serialized data.
    pub fn deserialize_u16(&mut self) -> Result<u16, Error> {
        let mut buf = [0u8; 2];
        buf.copy_from_slice(self.try_take(2)?);
        Ok(u16::from_be_bytes(buf))
    }

    /// Deserialize a 32-bit integer from the serialized data.
    pub fn deserialize_u32(&mut self) -> Result<u32, Error> {
        let mut buf = [0u8; 4];
        buf.copy_from_slice(self.try_take(4)?);
        Ok(u32::from_be_bytes(buf))
    }

    /// Deserialize sinara configuration board data.
    pub fn deserialize_board_data(&mut self) -> Result<[u8; 64], Error> {
        let mut buf = [0u8; 64];
        buf.copy_from_slice(self.try_take(64)?);
        Ok(buf)
    }

    /// Deserialize sinara padding data.
    pub fn deserialize_padding(&mut self) -> Result<[u8; 122], Error> {
        let mut buf = [0u8; 122];
        buf.copy_from_slice(self.try_take(122)?);
        Ok(buf)
    }
}

/// The standard EEPROM layout for all Sinara hardware. Booster-specific configuration is stored in
/// `board_data`.
pub struct SinaraConfiguration {
    crc32: u32,
    magic: u16,
    pub name: [u8; 10],
    pub board_id: u16,
    pub format_rev: u8,
    pub major: u8,
    pub minor: u8,
    pub variant: u8,
    pub port: u8,
    pub vendor: u8,
    pub vendor_data: [u8; 8],
    pub project_data: [u8; 16],
    pub user_data: [u8; 16],
    pub board_data: [u8; 64],
    _padding: [u8; 122],
    pub eui48: [u8; 6],
}

impl SinaraConfiguration {
    /// Attempt to deserialize sinara configuration data from the raw EEPROM content.
    ///
    /// # Returns
    /// The configuration if it was properly decoded. Otherwise, an error.
    pub fn try_deserialize(data: [u8; 256]) -> Result<SinaraConfiguration, Error> {
        let config = {
            let mut deserializer = Deserializer::new(&data);

            SinaraConfiguration {
                crc32: deserializer.deserialize_u32()?,
                magic: deserializer.deserialize_u16()?,
                name: deserializer.try_take(10)?.try_into().unwrap(),
                board_id: deserializer.deserialize_u16()?,
                format_rev: deserializer.deserialize_u8()?,
                major: deserializer.deserialize_u8()?,
                minor: deserializer.deserialize_u8()?,
                variant: deserializer.deserialize_u8()?,
                port: deserializer.deserialize_u8()?,
                vendor: deserializer.deserialize_u8()?,
                vendor_data: deserializer.try_take(8)?.try_into().unwrap(),
                project_data: deserializer.try_take(16)?.try_into().unwrap(),
                user_data: deserializer.try_take(16)?.try_into().unwrap(),
                board_data: deserializer.deserialize_board_data()?,
                _padding: deserializer.deserialize_padding()?,
                eui48: deserializer.try_take(6)?.try_into().unwrap(),
            }
        };

        if config.crc32 != config.calculate_crc32() || config.magic != 0x391e {
            Err(Error::Invalid)
        } else {
            Ok(config)
        }
    }

    /// Serialize the configuration into an EEPROM buffer.
    ///
    /// # Args
    /// * `buf` - The buffer to serialize into.
    pub fn serialize_into<'a>(&self, buf: &'a mut [u8; 128]) -> &'a [u8] {
        let mut serializer = Serializer::new(buf);

        serializer.serialize_u32(self.crc32).unwrap();
        serializer.serialize_u16(self.magic).unwrap();
        serializer.serialize_data(&self.name).unwrap();
        serializer.serialize_u16(self.board_id).unwrap();
        serializer.serialize_u8(self.format_rev).unwrap();
        serializer.serialize_u8(self.major).unwrap();
        serializer.serialize_u8(self.minor).unwrap();
        serializer.serialize_u8(self.variant).unwrap();
        serializer.serialize_u8(self.port).unwrap();
        serializer.serialize_u8(self.vendor).unwrap();
        serializer.serialize_data(&self.vendor_data).unwrap();
        serializer.serialize_data(&self.project_data).unwrap();
        serializer.serialize_data(&self.user_data).unwrap();
        serializer.serialize_data(&self.board_data).unwrap();
        serializer.finish().unwrap()
    }

    /// Generate a default sinara EEPROM configuration.
    ///
    /// # Args
    /// * `mainboard` - Specified true if the sinara configuration is for the booster mainboard.
    pub fn default(board_id: BoardId) -> SinaraConfiguration {
        let name = match board_id {
            BoardId::Mainboard => "Booster",
            BoardId::RfChannel => "Booster_Ch",
        };

        let mut name_info: [u8; 10] = [0; 10];
        name_info[..name.len()].copy_from_slice(name.as_bytes());

        let mut config = SinaraConfiguration {
            // Will be updated later.
            crc32: 0,

            magic: 0x391e,
            name: name_info,
            board_id: board_id as u16,
            format_rev: 0,
            major: 1,
            minor: 0,
            variant: 0,
            port: 0,

            // Specifies QUARTIQ
            vendor: 3,
            vendor_data: [0; 8],

            project_data: [0; 16],
            user_data: [0; 16],
            board_data: [0; 64],

            // Padding and EUI48 are DONT-CARE - this is a read-only memory region.
            _padding: [0xFF; 122],
            eui48: [0xFF; 6],
        };

        config.update_crc32();

        config
    }

    /// Update the internal CRC32 of the configuration.
    pub fn update_crc32(&mut self) {
        self.crc32 = self.calculate_crc32()
    }

    fn calculate_crc32(&self) -> u32 {
        let mut data = [0u8; 128];
        self.serialize_into(&mut data);

        // The first 4 bytes of the serialized structure are the CRC itself, so do not include them
        // in the calculation.
        let mut crc32 = crc_any::CRC::crc32();
        crc32.digest(&data[4..]);

        crc32.get_crc() as u32
    }
}
