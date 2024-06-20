//! Booster NGFW User Interface code

use super::Channel;
use bit_field::BitField;
use hal::hal_02::digital::v2::InputPin;
use stm32f4xx_hal as hal;

use debounced_pin::{Debounce, DebounceState, DebouncedInputPin};

/// Represents an event indicated through the GPIO buttons.
pub enum ButtonEvent {
    InterlockReset,
    Standby,
}

type Button1 = hal::gpio::gpiof::PF14<hal::gpio::Input>;

type Button2 = hal::gpio::gpiof::PF15<hal::gpio::Input>;

/// Represents the two user input buttons on the front panel.
pub struct UserButtons {
    button1: InputButton<Button1, <Button1 as InputPin>::Error>,
    button2: InputButton<Button2, <Button2 as InputPin>::Error>,
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
            button1: InputButton::new(button1),
            button2: InputButton::new(button2),
        }
    }

    /// Check for a button update event.
    ///
    /// # Returns
    /// An option containing any event that is indicated by the button update.
    pub fn update(&mut self) -> Option<ButtonEvent> {
        // Prioritize entering standby.
        if self.button2.update() {
            return Some(ButtonEvent::Standby);
        }

        if self.button1.update() {
            return Some(ButtonEvent::InterlockReset);
        }

        None
    }
}

/// A structure representing one of the input buttons.
struct InputButton<INPUT, E>
where
    INPUT: InputPin<Error = E>,
    E: core::fmt::Debug,
{
    button: DebouncedInputPin<INPUT, debounced_pin::ActiveLow>,
    was_active: bool,
}

impl<INPUT, E> InputButton<INPUT, E>
where
    INPUT: InputPin<Error = E>,
    E: core::fmt::Debug,
{
    /// Construct a new input button.
    pub fn new(button: INPUT) -> Self {
        InputButton {
            was_active: false,
            button: DebouncedInputPin::new(button, debounced_pin::ActiveLow),
        }
    }

    /// Periodically check the state of the input button.
    ///
    /// # Returns
    /// True if the debounced button state has encountered an activation.
    pub fn update(&mut self) -> bool {
        match self.button.update().unwrap() {
            DebounceState::Active => {
                let result = !self.was_active;
                self.was_active = true;
                result
            }
            DebounceState::NotActive => {
                self.was_active = false;
                false
            }
            _ => false,
        }
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
