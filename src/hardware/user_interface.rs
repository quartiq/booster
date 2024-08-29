//! Booster NGFW User Interface code

use super::Channel;
use bit_field::BitField;
use stm32f4xx_hal as hal;

use debouncr::{Debouncer, Repeat10};

/// Represents an event indicated through the GPIO buttons.
pub enum ButtonEvent {
    InterlockReset,
    Standby,
}

pub type Button1 = hal::gpio::gpiof::PF14<hal::gpio::Input>;

type Button2 = hal::gpio::gpiof::PF15<hal::gpio::Input>;

/// Represents the two user input buttons on the front panel.
pub struct UserButtons {
    button1: Button1,
    button2: Button2,
    button1_state: Debouncer<u16, Repeat10>,
    button2_state: Debouncer<u16, Repeat10>,
}

impl UserButtons {
    /// Construct the user buttons.
    ///
    /// # Args:
    /// * `button1` - The primary user interface button.
    /// * `button2` - The secondary user interface button.
    ///
    /// # Returns
    /// The user interface button manager.
    pub fn new(button1: Button1, button2: Button2) -> Self {
        UserButtons {
            button1,
            button2,
            button1_state: debouncr::debounce_10(false),
            button2_state: debouncr::debounce_10(false),
        }
    }

    /// Check for a button update event.
    ///
    /// # Returns
    /// An option containing any event that is indicated by the button update.
    pub fn update(&mut self) -> Option<ButtonEvent> {
        // Prioritize entering standby.
        let button2_pressed = self.button2.is_low();
        if let Some(debouncr::Edge::Rising) = self.button2_state.update(button2_pressed) {
            return Some(ButtonEvent::Standby);
        }

        let button1_pressed = self.button1.is_low();
        if let Some(debouncr::Edge::Rising) = self.button1_state.update(button1_pressed) {
            return Some(ButtonEvent::InterlockReset);
        }
        None
    }
}

/// Represents LED colors on the front panel.
pub enum Color {
    Red,
    Yellow,
    Green,
}

type LedSpi = hal::spi::Spi<hal::pac::SPI2>;

pub struct UserLeds {
    red: u8,
    yellow: u8,
    green: u8,
    spi: LedSpi,
    spi_csn: hal::gpio::gpiob::PB12<hal::gpio::Output<hal::gpio::PushPull>>,
}

impl UserLeds {
    /// Construct a driver for the user front-panel LEDs.
    pub fn new(
        spi: LedSpi,
        csn: hal::gpio::gpiob::PB12<hal::gpio::Output<hal::gpio::PushPull>>,
        mut oen: hal::gpio::gpiob::PB8<hal::gpio::Output<hal::gpio::PushPull>>,
    ) -> Self {
        let mut leds = UserLeds {
            red: 0u8,
            yellow: 0u8,
            green: 0u8,
            spi,
            spi_csn: csn,
        };

        // Enable LED output.
        oen.set_low();

        leds.update();

        leds
    }

    /// Write the LED state to the LED outputs.
    pub fn update(&mut self) {
        self.spi_csn.set_low();
        self.spi
            .write(&[self.green, self.yellow, self.red])
            .unwrap();
        self.spi_csn.set_high();
    }

    /// Update the state of an LED.
    ///
    /// # Args
    /// * `color` - The LED color to modify.
    /// * `channel` - The channel to update the LED for.
    /// * `enabled` - Specified true if the LED should be illuminated.
    pub fn set_led(&mut self, color: Color, channel: Channel, enabled: bool) {
        // The LEDs annotate the channels in reverse ordering.
        let channel = Channel::Seven as usize - channel as usize;

        match color {
            Color::Green => self.green.set_bit(channel, enabled),
            Color::Yellow => self.yellow.set_bit(channel, enabled),
            Color::Red => self.red.set_bit(channel, enabled),
        };
    }
}
