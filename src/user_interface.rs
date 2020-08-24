//! Booster NGFW User Interface code
//!
//! # Copyright
//! Copyright (C) 2020 QUARTIQ GmbH - All Rights Reserved
//! Unauthorized usage, editing, or copying is strictly prohibited.
//! Proprietary and confidential.

use super::{hal, Channel};
use bit_field::BitField;
use embedded_hal::{
    blocking::spi::Write,
    digital::v2::{InputPin, OutputPin},
};

use debounced_pin::{Debounce, DebounceState, DebouncedInputPin};

/// Represents an event indicated through the GPIO buttons.
pub enum ButtonEvent {
    InterlockReset,
    Standby,
}

type Button1 = hal::gpio::gpiof::PF14<hal::gpio::Input<hal::gpio::Floating>>;
type Button2 = hal::gpio::gpiof::PF15<hal::gpio::Input<hal::gpio::Floating>>;

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
    pub fn new(button: INPUT) -> Self {
        InputButton {
            was_active: false,
            button: DebouncedInputPin::new(button, debounced_pin::ActiveLow),
        }
    }

    pub fn update(&mut self) -> bool {
        match self.button.update().unwrap() {
            DebounceState::Active => {
                let result = self.was_active == false;
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

pub enum Color {
    Red,
    Yellow,
    Green,
}

type LedSpi = hal::spi::Spi<
    hal::stm32::SPI2,
    (
        hal::gpio::gpiob::PB13<hal::gpio::Alternate<hal::gpio::AF5>>,
        hal::spi::NoMiso,
        hal::gpio::gpiob::PB15<hal::gpio::Alternate<hal::gpio::AF5>>,
    ),
>;

pub struct UserLeds {
    red: u8,
    yellow: u8,
    green: u8,
    spi: LedSpi,
    spi_csn: hal::gpio::gpiob::PB12<hal::gpio::Output<hal::gpio::PushPull>>,
    spi_oen: hal::gpio::gpiob::PB8<hal::gpio::Output<hal::gpio::PushPull>>,
}

impl UserLeds {
    pub fn new(
        spi: LedSpi,
        csn: hal::gpio::gpiob::PB12<hal::gpio::Output<hal::gpio::PushPull>>,
        oen: hal::gpio::gpiob::PB8<hal::gpio::Output<hal::gpio::PushPull>>,
    ) -> Self {
        let mut leds = UserLeds {
            red: 0u8,
            yellow: 0u8,
            green: 0u8,
            spi: spi,
            spi_oen: oen,
            spi_csn: csn,
        };

        leds.update();

        leds
    }

    pub fn update(&mut self) {
        self.spi_oen.set_high().unwrap();

        self.spi_csn.set_low().unwrap();
        self.spi
            .write(&[self.green, self.yellow, self.red])
            .unwrap();
        self.spi_csn.set_high().unwrap();

        self.spi_oen.set_low().unwrap();
    }

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