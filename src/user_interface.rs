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

use super::{Duration, Instant};

/// Represents an event indicated through the GPIO buttons.
pub enum ButtonEvent {
    EnableAllChannels,
    DisableChannels,
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

    /// Register a button update event.
    ///
    /// # Args
    /// * `instant` - The current time instant when the event occurred.
    ///
    /// # Returns
    /// An option containing any event that is indicated by the button update.
    pub fn event(&mut self, instant: Instant) -> Option<ButtonEvent> {
        if self.button1.check_short_press(instant) {
            return Some(ButtonEvent::EnableAllChannels);
        }

        if self.button2.check_short_press(instant) {
            return Some(ButtonEvent::DisableChannels);
        }

        None
    }

    /// Check if the buttons are indicating a device reset.
    ///
    /// # Args
    /// * `instant` - The current time instant.
    ///
    /// # Returns
    /// True if a device reset is being requested.
    pub fn check_reset(&mut self, instant: Instant) -> bool {
        self.button1.check_long_press(instant) && self.button2.check_long_press(instant)
    }
}

struct InputButton<INPUT, E>
where
    INPUT: InputPin<Error = E> + hal::gpio::ExtiPin,
    E: core::fmt::Debug,
{
    button: INPUT,
    press_start: Option<Instant>,
}

impl<INPUT, E> InputButton<INPUT, E>
where
    INPUT: InputPin<Error = E> + hal::gpio::ExtiPin,
    E: core::fmt::Debug,
{
    pub fn new(button: INPUT) -> Self {
        InputButton {
            button,
            press_start: None,
        }
    }

    pub fn check_short_press(&mut self, instant: Instant) -> bool {
        self.button.clear_interrupt_pending_bit();

        let released = self.button.is_high().unwrap();

        // If this event occurred too close to a previous event, ignore it as a spurious debounce
        // event.
        if self.debounce(instant) == false {
            if released {
                self.press_start = None;
            }

            return false;
        }

        // Check if the button is now released. This indicates a short press (regardless of
        // duration).
        if released {
            let result = self.press_start.is_some();

            self.press_start = None;

            result
        } else {
            // Otherwise, record the time of the button press.
            self.press_start = Some(instant);

            false
        }
    }

    /// Check if the button event is debounced from the start.
    fn debounce(&mut self, instant: Instant) -> bool {
        self.press_start.map_or(true, |start| {
            instant - start > Duration::from_cycles((168_000_000.0 * 0.08) as u32)
        })
    }

    pub fn check_long_press(&mut self, instant: Instant) -> bool {
        // Check if the button is still pressed.
        if self.button.is_low().unwrap() == false {
            return false;
        }

        self.press_start.map_or(false, |start| {
            // Check if the button is being held for greater than the long-press duration.
            instant - start > Duration::from_cycles((168_000_000.0 * 2.0) as u32)
        })
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
        self.spi_oen.set_low().unwrap();

        self.spi_csn.set_low().unwrap();
        self.spi
            .write(&[self.green, self.yellow, self.red])
            .unwrap();
        self.spi_csn.set_high().unwrap();

        self.spi_oen.set_high().unwrap();
    }

    pub fn set_led(&mut self, color: Color, channel: Channel, enabled: bool) {
        match color {
            Color::Green => self.green.set_bit(channel as usize, enabled),
            Color::Yellow => self.yellow.set_bit(channel as usize, enabled),
            Color::Red => self.red.set_bit(channel as usize, enabled),
        };
    }
}
