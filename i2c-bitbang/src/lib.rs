//! I2C Bitbanging Driver
//!
//! # Description
//! This is a minimalistic driver for accessing I2C devices via bitbanging.
//! Currently, cortex-m::asm is used to accomplish microsecond delays.
//!
//! # Special Notes
//! Most of the bitbanging mechanism is attributed to M-Labs:
//! * ARTIQ: https://github.com/m-labs/artiq/blob/master/artiq/firmware/libboard_misoc/i2c.rs
//! * zynq-rs: https://git.m-labs.hk/M-Labs/zynq-rs/src/branch/master/libboard_zynq/src/i2c/mod.rs
#![no_std]
#![deny(warnings)]

use embedded_hal::{
    blocking::i2c::{Read, Write, WriteRead},
    digital::v2::{InputPin, OutputPin},
};

/// I2C Bitbanging Driver
pub struct I2cBitBang<SCL, SDA> {
    scl: SCL,
    sda: SDA,
    cycles_half_period: u32,
}

#[derive(Copy, Clone, Debug)]
pub enum Error {
    ///
    GenericError,
    NoAck,
}

impl<SCL, SDA> I2cBitBang<SCL, SDA>
where
    SCL: OutputPin + InputPin,
    SDA: OutputPin + InputPin,
{
    pub fn new(mut scl: SCL, mut sda: SDA, i2c_freq: u32, cpu_freq: u32) -> Self {
        let cycles_half_period = cpu_freq / i2c_freq / 2;

        // Ensure SCL and SDA are high before transactions
        scl.set_high().ok();
        sda.set_high().ok();
        // Check the I2C bus is ready
        cortex_m::asm::delay(cycles_half_period);
        cortex_m::asm::delay(cycles_half_period);
        if let Some(true) = sda.is_low().ok() {
            // Try toggling SCL a few times
            for _bit in 0..8 {
                scl.set_low().ok();
                cortex_m::asm::delay(cycles_half_period);
                scl.set_high().ok();
                cortex_m::asm::delay(cycles_half_period);
            }
        }
        // TODO: check if SDA is stuck low
        // TODO: check if SCL is stuck low

        Self {
            scl,
            sda,
            cycles_half_period,
        }
    }

    #[inline]
    fn half_period(&self) {
        cortex_m::asm::delay(self.cycles_half_period);
    }

    fn start(&mut self) -> Result<(), Error> {
        // Ensure SCL and SDA are high before each transaction
        self.scl.set_high().ok();
        self.sda.set_high().ok();
        self.half_period();
        // TODO: check if SDA is stuck low
        // TODO: check if SCL is stuck low
        self.sda.set_low().ok();
        self.half_period();
        self.scl.set_low().ok();
        self.half_period();
        Ok(())
    }

    fn stop(&mut self) -> Result<(), Error> {
        self.half_period();
        self.scl.set_high().ok();
        self.half_period();
        self.sda.set_high().ok();
        self.half_period();
        // TODO: check if SDA is still high meaning lost arbitration
        Ok(())
    }

    fn send_byte(&mut self, byte: u8) -> Result<(), Error> {
        // MSB first
        for bit in (0..8).rev() {
            match byte & (1 << bit) {
                0 => self.sda.set_low().ok(),
                _ => self.sda.set_high().ok(),
            };
            self.half_period();
            self.scl.set_high().ok();
            self.half_period();
            self.scl.set_low().ok();
            self.half_period();
        }
        self.sda.set_high().ok();
        self.half_period();
        self.scl.set_high().ok();
        self.half_period();
        // Read ack/nack
        let ack = self.sda.is_low().ok();
        self.scl.set_low().ok();
        self.half_period();
        self.sda.set_low().ok();
        if let Some(false) = ack {
            return Err(Error::NoAck);
        }
        Ok(())
    }

    fn recv_byte(&mut self, ack: bool) -> Result<u8, Error> {
        self.sda.set_high().ok();
        let mut byte: u8 = 0;
        // MSB first
        for bit in (0..8).rev() {
            self.half_period();
            self.scl.set_high().ok();
            self.half_period();
            if let Some(true) = self.sda.is_high().ok() {
                byte |= 1 << bit
            }
            self.scl.set_low().ok();
        }
        // Send ack/nack
        match ack {
            true => self.sda.set_low().ok(),
            _ => self.sda.set_high().ok(),
        };
        self.half_period();
        self.scl.set_high().ok();
        self.half_period();
        self.scl.set_low().ok();
        self.sda.set_low().ok();
        Ok(byte)
    }

    /// Initiate START (can be repeated START), send bytes, and returns.
    /// To allow repeated START, this does not send STOP at the end.
    fn write_bytes(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Send START
        self.start()?;
        // Write slave address + R/W_n bit
        self.send_byte(addr << 1)?;
        // Write data bytes
        for byte in bytes {
            self.send_byte(*byte)?;
        }
        Ok(())
    }

    /// Initiate START (can be repeated START), receive as many bytes as the
    /// buffer can accommodate, and returns.
    /// Send ACK after each byte except the final byte, where NACK is sent.
    /// To allow repeated START, this does not send STOP at the end.
    fn read_bytes(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // Send START
        self.start()?;
        // Write slave address + R/W_n bit
        self.send_byte((addr << 1) | 1)?;
        // Read data bytes to buffer
        if let Some((last, buffer)) = buffer.split_last_mut() {
            for byte in buffer {
                self.recv_byte(true).map(|b| {
                    *byte = b;
                })?;
            }
            self.recv_byte(false).map(|b| {
                *last = b;
            })?;
        }
        Ok(())
    }
}

impl<SCL, SDA> Read for I2cBitBang<SCL, SDA>
where
    SCL: OutputPin + InputPin,
    SDA: OutputPin + InputPin,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.read_bytes(addr, buffer)?;
        // Send STOP
        self.stop()?;
        Ok(())
    }
}

impl<SCL, SDA> Write for I2cBitBang<SCL, SDA>
where
    SCL: OutputPin + InputPin,
    SDA: OutputPin + InputPin,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(addr, bytes)?;
        // Send STOP
        self.stop()?;
        Ok(())
    }
}

impl<SCL, SDA> WriteRead for I2cBitBang<SCL, SDA>
where
    SCL: OutputPin + InputPin,
    SDA: OutputPin + InputPin,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        self.write_bytes(addr, bytes)?;
        self.read(addr, buffer)?;
        // Send STOP
        self.stop()?;
        Ok(())
    }
}
