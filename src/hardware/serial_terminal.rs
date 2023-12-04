//! Booster NGFW Application
use super::flash::Flash;
use super::{platform, UsbBus};
use crate::settings::global_settings::BoosterMainBoardData;
use core::fmt::Write;
use embedded_storage::nor_flash::NorFlash;

pub struct SerialSettingsPlatform {
    pub metadata: &'static crate::hardware::metadata::ApplicationMetadata,
    pub settings: BoosterMainBoardData,
    pub storage: Flash,

    /// The interface to read/write data to/from serially (via text) to the user.
    pub interface: serial_settings::BestEffortInterface<usbd_serial::SerialPort<'static, UsbBus>>,
}

#[derive(Debug)]
pub enum Error<F> {
    Postcard(postcard::Error),
    Flash(F),
}

impl<F> From<postcard::Error> for Error<F> {
    fn from(e: postcard::Error) -> Self {
        Self::Postcard(e)
    }
}

impl serial_settings::Platform for SerialSettingsPlatform {
    type Interface = serial_settings::BestEffortInterface<usbd_serial::SerialPort<'static, UsbBus>>;

    type Settings = crate::settings::global_settings::BoosterMainBoardData;

    type Error = Error<<Flash as embedded_storage::nor_flash::ErrorType>::Error>;

    fn save(&mut self, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let serialized = postcard::to_slice(self.settings(), buffer)?;
        self.storage
            .erase(0, serialized.len() as u32)
            .map_err(Self::Error::Flash)?;
        self.storage
            .write(0, serialized)
            .map_err(Self::Error::Flash)?;
        Ok(())
    }

    /// Execute a platform specific command.
    fn cmd(&mut self, cmd: &str) {
        match cmd {
            "reboot" => {
                cortex_m::interrupt::disable();
                platform::shutdown_channels();
                cortex_m::peripheral::SCB::sys_reset();
            }
            "dfu" => {
                cortex_m::interrupt::disable();

                // Power off all output channels and reset the MCU.
                platform::shutdown_channels();

                platform::reset_to_dfu_bootloader();
            }
            "service" => {
                writeln!(
                    &mut self.interface,
                    "{:<20}: {} [{}]",
                    "Version", self.metadata.firmware_version, self.metadata.profile,
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Hardware Revision", self.metadata.hardware_version
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Rustc Version", self.metadata.rust_version
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Features", self.metadata.features
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Detected Phy", self.metadata.phy
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Panic Info", self.metadata.panic_info
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Watchdog Detected", self.metadata.watchdog
                )
                .unwrap();

                // Use this as a mechanism for the user to "acknowledge" the service state of
                // the device. This will allow RF channels to re-enable.
                platform::clear_reset_flags();
            }
            other => {
                writeln!(
                    self.interface_mut(),
                    "Invalid platform command: `{other}` is not in [`dfu`, `service`, `reboot`]"
                )
                .ok();
            }
        }
    }

    /// Return a mutable reference to the `Interface`.
    fn interface_mut(&mut self) -> &mut Self::Interface {
        &mut self.interface
    }

    /// Return a reference to the `Settings`
    fn settings(&self) -> &Self::Settings {
        &self.settings
    }

    /// Return a mutable reference to the `Settings`.
    fn settings_mut(&mut self) -> &mut Self::Settings {
        &mut self.settings
    }
}
