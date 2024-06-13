//! Booster NGFW Application

use heapless::{String, Vec};
use miniconf::{JsonCoreSlash, Postcard, TreeKey};
use sequential_storage::map;

use crate::hardware::{flash::Flash, platform, UsbBus};
use crate::settings::global_settings::BoosterMainBoardData;
use core::fmt::Write;

#[derive(Default, serde::Serialize, serde::Deserialize)]
pub struct SettingsItem {
    // We only make these owned vec/string to get around lifetime limitations.
    pub path: String<64>,
    pub data: Vec<u8, 256>,
}

impl map::StorageItem for SettingsItem {
    type Key = String<64>;
    type Error = postcard::Error;

    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(postcard::to_slice(self, buffer)?.len())
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, Self::Error> {
        postcard::from_bytes(buffer)
    }

    fn key(&self) -> Self::Key {
        self.path.clone()
    }
}

pub fn load_from_flash<T: for<'d> JsonCoreSlash<'d, Y>, const Y: usize>(
    structure: &mut T,
    storage: &mut Flash,
) {
    // Loop over flash and read settings
    let mut buffer = [0u8; 512];
    for path in T::iter_paths::<String<64>>("/") {
        let path = path.unwrap();

        // Try to fetch the setting from flash.
        let item = match map::fetch_item::<SettingsItem, _>(
            storage,
            storage.range(),
            &mut buffer,
            path.clone(),
        ) {
            Err(e) => {
                log::warn!("Failed to fetch `{path}` from flash: {e:?}");
                continue;
            }
            Ok(Some(item)) => item,
            _ => continue,
        };

        log::info!("Loading initial `{path}` from flash");

        let flavor = postcard::de_flavors::Slice::new(&item.data);
        if let Err(e) = structure.set_postcard_by_key(path.split('/').skip(1), flavor) {
            log::warn!("Failed to deserialize `{path}` from flash: {e:?}");
        }
    }
    log::info!("Loaded settings from Flash");
}

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

impl serial_settings::Platform<1> for SerialSettingsPlatform {
    type Interface = serial_settings::BestEffortInterface<usbd_serial::SerialPort<'static, UsbBus>>;

    type Settings = crate::settings::global_settings::BoosterMainBoardData;

    type Error = Error<<Flash as embedded_storage::nor_flash::ErrorType>::Error>;

    fn save(&mut self, buffer: &mut [u8], settings: &Self::Settings) -> Result<(), Self::Error> {
        for path in Self::Settings::iter_paths::<String<64>>("/") {
            let mut item = SettingsItem {
                path: path.unwrap(),
                ..Default::default()
            };

            item.data.resize(item.data.capacity(), 0).unwrap();

            let flavor = postcard::ser_flavors::Slice::new(&mut item.data);

            let len = match settings.get_postcard_by_key(item.path.split('/').skip(1), flavor) {
                Err(e) => {
                    log::warn!("Failed to save `{}` to flash: {e:?}", item.path);
                    continue;
                }
                Ok(slice) => slice.len(),
            };
            item.data.truncate(len);

            let range = self.storage.range();

            // Check if the settings has changed from what's currently in flash (or if it doesn't
            // yet exist).
            if map::fetch_item::<SettingsItem, _>(
                &mut self.storage,
                range.clone(),
                buffer,
                item.path.clone(),
            )
            .unwrap()
            .map(|old| old.data != item.data)
            .unwrap_or(true)
            {
                log::info!("Storing `{}` to flash", item.path);
                map::store_item(&mut self.storage, range, buffer, item).unwrap();
            }
        }

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
}
