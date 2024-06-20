//! Booster NGFW Application

use heapless::{String, Vec};
use miniconf::{JsonCoreSlash, Postcard, TreeKey};
use sequential_storage::map;

use crate::hardware::{flash::Flash, platform};
use core::fmt::Write;

pub fn load_from_flash<T: for<'d> JsonCoreSlash<'d, Y>, const Y: usize>(
    structure: &mut T,
    storage: &mut Flash,
) {
    // Loop over flash and read settings
    let mut buffer = [0u8; 512];
    for path in T::iter_paths::<String<64>>("/") {
        let path = path.unwrap();

        // Try to fetch the setting from flash.
        let item = match embassy_futures::block_on(map::fetch_item::<SettingsKey, SettingsItem, _>(
            storage,
            storage.range(),
            &mut sequential_storage::cache::NoCache::new(),
            &mut buffer,
            SettingsKey(path.clone()),
        )) {
            Err(e) => {
                log::warn!("Failed to fetch `{path}` from flash: {e:?}");
                continue;
            }
            Ok(Some(item)) => item,
            _ => continue,
        };

        // An empty vector may be saved to flash to "erase" a setting, since the H7 doesn't support
        // multi-write NOR flash. If we see an empty vector, ignore this entry.
        if item.0.is_empty() {
            continue;
        }

        log::info!("Loading initial `{path}` from flash");

        let flavor = postcard::de_flavors::Slice::new(&item.0);
        if let Err(e) = structure.set_postcard_by_key(path.split('/').skip(1), flavor) {
            log::warn!("Failed to deserialize `{path}` from flash: {e:?}");
        }
    }
    log::info!("Loaded settings from Flash");
}

#[derive(Default, serde::Serialize, serde::Deserialize, Clone, PartialEq, Eq)]
pub struct SettingsKey(String<64>);

impl map::Key for SettingsKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, map::SerializationError> {
        Ok(postcard::to_slice(self, buffer)
            .map_err(|_| map::SerializationError::BufferTooSmall)?
            .len())
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), map::SerializationError> {
        let original_length = buffer.len();
        let (result, remainder) = postcard::take_from_bytes(buffer)
            .map_err(|_| map::SerializationError::BufferTooSmall)?;
        Ok((result, original_length - remainder.len()))
    }
}

#[derive(Default, serde::Serialize, serde::Deserialize, Clone, PartialEq, Eq)]
pub struct SettingsItem(Vec<u8, 256>);

impl<'a> map::Value<'a> for SettingsItem {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, map::SerializationError> {
        if buffer.len() < self.0.len() {
            return Err(map::SerializationError::BufferTooSmall);
        }

        buffer[..self.0.len()].copy_from_slice(&self.0);
        Ok(self.0.len())
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, map::SerializationError> {
        let vec = Vec::from_slice(buffer).map_err(|_| map::SerializationError::BufferTooSmall)?;
        Ok(Self(vec))
    }
}

pub struct SerialSettingsPlatform {
    pub metadata: &'static crate::hardware::metadata::ApplicationMetadata,
    pub storage: Flash,

    /// The interface to read/write data to/from serially (via text) to the user.
    pub interface: serial_settings::BestEffortInterface<crate::hardware::SerialPort>,
}

impl SerialSettingsPlatform {
    pub fn save_item(&mut self, buffer: &mut [u8], key: String<64>, value: Vec<u8, 256>) {
        let path = SettingsKey(key);
        let range = self.storage.range();

        // Check if the settings has changed from what's currently in flash (or if it doesn't
        // yet exist).
        if embassy_futures::block_on(map::fetch_item::<SettingsKey, SettingsItem, _>(
            &mut self.storage,
            range.clone(),
            &mut sequential_storage::cache::NoCache::new(),
            buffer,
            path.clone(),
        ))
        .unwrap()
        .map(|old| old.0 != value)
        .unwrap_or(true)
        {
            log::info!("Storing `{}` to flash", path.0);
            embassy_futures::block_on(map::store_item(
                &mut self.storage,
                range,
                &mut sequential_storage::cache::NoCache::new(),
                buffer,
                path,
                &SettingsItem(value),
            ))
            .unwrap();
        }
    }
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

impl serial_settings::Platform<5> for SerialSettingsPlatform {
    type Interface = serial_settings::BestEffortInterface<crate::hardware::SerialPort>;

    type Settings = crate::settings::Settings;

    type Error = Error<<Flash as embedded_storage::nor_flash::ErrorType>::Error>;

    fn save(
        &mut self,
        buffer: &mut [u8],
        key: Option<&str>,
        settings: &Self::Settings,
    ) -> Result<(), Self::Error> {
        let mut save_setting = |path: String<64>| {
            let path = SettingsKey(path);

            let mut data = Vec::new();
            data.resize(data.capacity(), 0).unwrap();
            let flavor = postcard::ser_flavors::Slice::new(&mut data);
            let len = match settings.get_postcard_by_key(path.0.split('/').skip(1), flavor) {
                Err(e) => {
                    log::warn!("Failed to save `{}` to flash: {e:?}", path.0);
                    return;
                }
                Ok(slice) => slice.len(),
            };
            data.truncate(len);

            self.save_item(buffer, path.0, data)
        };

        if let Some(path) = key {
            save_setting(path.parse().unwrap());
        } else {
            for path in Self::Settings::iter_paths::<String<64>>("/") {
                save_setting(path.unwrap());
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

    fn clear(&mut self, buffer: &mut [u8], key: Option<&str>) {
        let mut erase_setting = |path| -> Result<(), Self::Error> {
            let path = SettingsKey(path);
            let range = self.storage.range();

            // Check if there's an entry for this item in our flash map. The item might be a
            // sentinel value indicating "erased". Because we can't write flash memory twice, we
            // instead append a sentry "erased" value to the map where the serialized value is
            // empty.
            let maybe_item =
                embassy_futures::block_on(map::fetch_item::<SettingsKey, SettingsItem, _>(
                    &mut self.storage,
                    range.clone(),
                    &mut sequential_storage::cache::NoCache::new(),
                    buffer,
                    path.clone(),
                ))
                .unwrap();

            // An entry may exist in the map with no data as a sentinel that this path was
            // previously erased. If we find this, there's no need to store a duplicate "item is
            // erased" sentinel in flash. We only need to logically erase the path from the map if
            // it existed there in the first place.
            if matches!(maybe_item, Some(item) if !item.0.is_empty()) {
                embassy_futures::block_on(map::store_item(
                    &mut self.storage,
                    range,
                    &mut sequential_storage::cache::NoCache::new(),
                    buffer,
                    path,
                    &SettingsItem(Vec::new()),
                ))
                .unwrap();
            }

            Ok(())
        };

        if let Some(key) = key {
            erase_setting(key.parse().unwrap()).unwrap();
        } else {
            for path in Self::Settings::iter_paths::<String<64>>("/") {
                erase_setting(path.unwrap()).unwrap();
            }
        }
    }

    /// Return a mutable reference to the `Interface`.
    fn interface_mut(&mut self) -> &mut Self::Interface {
        &mut self.interface
    }
}
