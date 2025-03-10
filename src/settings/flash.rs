//! Booster NGFW Application

use heapless::{String, Vec};
use miniconf::{Path, TreeDeserializeOwned, TreeKey, TreeSerialize};

use crate::hardware::{flash::Flash, metadata::ApplicationMetadata, platform};
use embassy_futures::block_on;
use embedded_io::Write;
use sequential_storage::{
    cache::NoCache,
    map::{fetch_item, store_item, SerializationError},
};
use serial_settings::{BestEffortInterface, Platform, Settings};

#[derive(Default, serde::Serialize, serde::Deserialize, Clone, PartialEq, Eq)]
pub struct SettingsKey(Vec<u8, 128>);

impl sequential_storage::map::Key for SettingsKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        Ok(postcard::to_slice(self, buffer)
            .map_err(|_| SerializationError::BufferTooSmall)?
            .len())
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        let original_length = buffer.len();
        let (result, remainder) =
            postcard::take_from_bytes(buffer).map_err(|_| SerializationError::BufferTooSmall)?;
        Ok((result, original_length - remainder.len()))
    }
}

pub struct SerialSettingsPlatform<C> {
    /// The interface to read/write data to/from serially (via text) to the user.
    pub interface: BestEffortInterface<crate::hardware::SerialPort>,

    pub _settings_marker: core::marker::PhantomData<C>,

    pub storage: Flash,

    pub metadata: &'static ApplicationMetadata,
}

impl<C> SerialSettingsPlatform<C>
where
    C: TreeDeserializeOwned + TreeSerialize + TreeKey,
{
    pub fn load(structure: &mut C, storage: &mut Flash) {
        // Loop over flash and read settings
        let mut buffer = [0u8; 512];
        for path in C::nodes::<Path<String<128>, '/'>, 8>() {
            let (path, _node) = path.unwrap();

            // Try to fetch the setting from flash.
            let value: &[u8] = match block_on(fetch_item(
                storage,
                storage.range(),
                &mut NoCache::new(),
                &mut buffer,
                &SettingsKey(path.clone().into_inner().into_bytes()),
            )) {
                Err(e) => {
                    log::warn!("Failed to fetch `{}` from flash: {e:?}", path.as_str());
                    continue;
                }
                Ok(Some(value)) => value,
                Ok(None) => continue,
            };

            // An empty vector may be saved to flash to "erase" a setting, since the H7 doesn't support
            // multi-write NOR flash. If we see an empty vector, ignore this entry.
            if value.is_empty() {
                continue;
            }

            log::info!("Loading initial `{}` from flash", path.as_str());

            let flavor = postcard::de_flavors::Slice::new(value);
            if let Err(e) = miniconf::postcard::set_by_key(structure, &path, flavor) {
                log::warn!(
                    "Failed to deserialize `{}` from flash: {e:?}",
                    path.as_str()
                );
            }
        }
    }
}

impl<C> Platform for SerialSettingsPlatform<C>
where
    C: Settings,
{
    type Interface = BestEffortInterface<crate::hardware::SerialPort>;
    type Settings = C;
    type Error =
        sequential_storage::Error<<Flash as embedded_storage::nor_flash::ErrorType>::Error>;

    fn fetch<'a>(
        &mut self,
        buf: &'a mut [u8],
        key: &[u8],
    ) -> Result<Option<&'a [u8]>, Self::Error> {
        let range = self.storage.range();
        block_on(fetch_item(
            &mut self.storage,
            range,
            &mut NoCache::new(),
            buf,
            &SettingsKey(Vec::try_from(key).unwrap()),
        ))
        .map(|v| v.filter(|v: &&[u8]| !v.is_empty()))
    }

    fn store(&mut self, buf: &mut [u8], key: &[u8], value: &[u8]) -> Result<(), Self::Error> {
        let range = self.storage.range();
        block_on(store_item(
            &mut self.storage,
            range,
            &mut NoCache::new(),
            buf,
            &SettingsKey(Vec::try_from(key).unwrap()),
            &value,
        ))
    }

    fn clear(&mut self, buf: &mut [u8], key: &[u8]) -> Result<(), Self::Error> {
        self.store(buf, key, b"")
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

                platform::start_dfu_reboot();
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
