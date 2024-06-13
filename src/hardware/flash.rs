use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use stm32f4xx_hal::flash::{FlashExt, LockedFlash};

pub struct Flash {
    flash: LockedFlash,
    base: u32,
}

impl Flash {
    pub fn new(flash: LockedFlash, base: usize) -> Self {
        Self {
            base: base as u32,
            flash,
        }
    }

    pub fn range(&self) -> core::ops::Range<u32> {
        0..(self.capacity() as u32)
    }
}

impl embedded_storage::nor_flash::ErrorType for Flash {
    type Error = <LockedFlash as embedded_storage::nor_flash::ErrorType>::Error;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        ReadNorFlash::read(&mut self.flash, self.base + offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.flash.capacity() - (self.base as usize)
    }
}

impl embedded_storage::nor_flash::NorFlash for Flash {
    const WRITE_SIZE: usize = stm32f4xx_hal::flash::UnlockedFlash::WRITE_SIZE;
    const ERASE_SIZE: usize = stm32f4xx_hal::flash::UnlockedFlash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let mut bank = self.flash.unlocked();
        NorFlash::erase(&mut bank, self.base + from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut bank = self.flash.unlocked();
        NorFlash::write(&mut bank, self.base + offset, bytes)
    }
}
