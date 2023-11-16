//! Booster NGFW Application
use super::{platform, UsbBus};

pub struct SerialSettingsPlatform {
    metadata: &'static crate::hardware::metadata::ApplicationMetadata,
}

impl SerialSettingsPlatform {
    /// Construct a terminal for interacting with the user.
    pub fn new(metadata: &'static crate::hardware::metadata::ApplicationMetadata) -> Self {
        Self { metadata }
    }
}

impl serial_settings::Platform for SerialSettingsPlatform {
    type Interface = usbd_serial::SerialPort<'static, UsbBus>;

    type Settings = crate::settings::global_settings::BoosterMainBoardData;
    type Storage = crate::hardware::flash::Flash;

    fn dfu(&mut self, mut interface: impl embedded_io::Write) {
        cortex_m::interrupt::disable();

        // Power off all output channels and reset the MCU.
        platform::shutdown_channels();

        platform::reset_to_dfu_bootloader();
    }

    fn reset(&mut self, _interface: impl embedded_io::Write) {
        cortex_m::interrupt::disable();
        platform::shutdown_channels();
    }

    fn service(&mut self, mut interface: impl embedded_io::Write) {
        writeln!(
            interface,
            "{:<20}: {} [{}]",
            "Version", self.metadata.firmware_version, self.metadata.profile,
        )
        .unwrap();
        writeln!(
            interface,
            "{:<20}: {}",
            "Hardware Revision", self.metadata.hardware_version
        )
        .unwrap();
        writeln!(
            interface,
            "{:<20}: {}",
            "Rustc Version", self.metadata.rust_version
        )
        .unwrap();
        writeln!(interface, "{:<20}: {}", "Features", self.metadata.features).unwrap();
        writeln!(interface, "{:<20}: {}", "Detected Phy", self.metadata.phy).unwrap();
        writeln!(
            interface,
            "{:<20}: {}",
            "Panic Info", self.metadata.panic_info
        )
        .unwrap();
        writeln!(
            interface,
            "{:<20}: {}",
            "Watchdog Detected", self.metadata.watchdog
        )
        .unwrap();

        // Use this as a mechanism for the user to "acknowledge" the service state of
        // the device. This will allow RF channels to re-enable.
        platform::clear_reset_flags();
    }
}
