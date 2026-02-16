//! Display subsystem

use core::mem::discriminant;

use arduino_hal::{
    hal::port::{PB0, PB1, PB2, PB3, PD4, PD6},
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
    Delay,
};

use ag_lcd::{Blink, Cursor, LcdDisplay, Lines};

use crate::rtc::RTCTime;

const BLANK_LINE: &str = "                    ";

/// A page being displayed on the LCD
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Page {
    /// Nothing; [Display] always starts on this page
    Blank,

    /// Temperature Readouts
    Temps(f32, f32, f32, f32),

    /// Current time according to RTC, or an error if time is unavailable
    Time(Result<RTCTime, &'static str>),
}

impl Page {
    /// The number of informational page variants; used for periodic page swaps
    pub const COUNT: usize = core::mem::variant_count::<Self>() - 1;
}

/// Climate controller display subsystem
#[must_use]
pub struct Display {
    lcd: LcdDisplay<Pin<Output>, Delay>,
    last_page: Page,
    page: Page,
    needs_clear: bool,
}

impl Display {
    /// Construct the display
    pub fn new(
        d4: Pin<Input<Floating>, PD4>,
        d12: Pin<Input<Floating>, PD6>,
        led_rx: Pin<Input<Floating>, PB0>,
        sck: Pin<Input<Floating>, PB1>,
        mosi: Pin<Input<Floating>, PB2>,
        miso: Pin<Input<Floating>, PB3>,
    ) -> Self {
        let lcd_rs = d12.into_output().downgrade();
        let lcd_en = d4.into_output().downgrade();
        let lcd_d4 = led_rx.into_output().downgrade();
        let lcd_d5 = sck.into_output().downgrade();
        let lcd_d6 = mosi.into_output().downgrade();
        let lcd_d7 = miso.into_output().downgrade();

        Self {
            lcd: LcdDisplay::new(lcd_rs, lcd_en, arduino_hal::Delay::new())
                .with_half_bus(lcd_d4, lcd_d5, lcd_d6, lcd_d7)
                .with_lines(Lines::FourLines)
                .with_cols(20)
                .build(),
            last_page: Page::Blank,
            page: Page::Blank,
            needs_clear: false,
        }
    }

    /// Initialize the display
    pub fn init(&mut self) {
        self.lcd.set_blink(Blink::Off);
        self.lcd.set_cursor(Cursor::Off);
    }

    /// Set the page and associated data to be drawn on the next refresh
    pub fn set_page(&mut self, page: Page) {
        if discriminant(&page) != discriminant(&self.page) {
            self.needs_clear = true;
        }
        self.page = page;
    }

    /// Refresh the display, drawing the currently set page if it has changed since the last refresh
    pub fn refresh(&mut self) {
        if self.page == self.last_page {
            return;
        }

        if self.needs_clear {
            self.lcd.clear();
        }

        match self.page {
            Page::Blank => {}
            Page::Temps(habitat, coolant, condenser, evaporator) => {
                self.print_temp(0, "Habitat:    ", habitat, self.needs_clear);
                self.print_temp(1, "Coolant:    ", coolant, self.needs_clear);
                self.print_temp(2, "Condenser:  ", condenser, self.needs_clear);
                self.print_temp(3, "Evaporator: ", evaporator, self.needs_clear);
            }
            Page::Time(Ok(time)) => {
                // 1st row: |Today is DayName    |
                // 2nd row: |YYYY.MM.DD  HH:MM:SS|

                self.lcd.set_position(0, 0);
                self.lcd.print("Today is ");
                self.lcd.print(time.day.name());

                self.lcd.set_position(0, 1);
                self.lcd.print("20");
                self.print_bcd(time.year.bcd());
                self.lcd.print(".");
                self.print_bcd(time.month.bcd());
                self.lcd.print(".");
                self.print_bcd(time.date.bcd());
                self.lcd.print("  ");
                self.print_bcd(time.hours.bcd_24h());
                self.lcd.print(":");
                self.print_bcd(time.minutes.bcd());
                self.lcd.print(":");
                self.print_bcd(time.seconds.bcd());
            }
            Page::Time(Err(msg)) => {
                self.lcd.set_position(0, 0);
                self.lcd.print(msg);
                self.lcd.set_position(0, 1);
                self.lcd.print(BLANK_LINE);
                self.lcd.set_position(0, 2);
                self.lcd.print(BLANK_LINE);
            }
        }

        self.last_page = self.page;
        self.needs_clear = false;
    }

    fn print_temp(&mut self, row: u8, title: &str, temp: f32, first: bool) {
        if first {
            self.lcd.set_position(0, row);
            self.lcd.print(title);
        } else {
            self.lcd.set_position(title.len() as u8, row);
        }

        self.print_float(temp);

        self.lcd.print("F");
    }

    /// Prints a 2-digit BCD value
    fn print_bcd(&mut self, value: u8) {
        self.lcd.print(digit(value >> 4));
        self.lcd.print(digit(value & 0xf));
    }

    /// Prints a floating point value in the range `[-999.99, 999.99]` at the current position,
    /// padded so that the number always occupies 7 characters and ends at the hundredths position
    ///
    /// Leading zeroes are omitted and the sign always appears immediately before the first nonzero
    /// leading digit, or the decimal point if there is none
    fn print_float(&mut self, value: f32) {
        let sign = if value.is_sign_negative() { "-" } else { " " };
        let value = value.abs();
        let hundreds = (libm::floorf(value / 100.0) as u8).rem_euclid(10);
        let tens = (libm::floorf(value / 10.0) as u8).rem_euclid(10);
        let ones = (value as u8).rem_euclid(10);
        let tenths = ((value - libm::floorf(value)) * 10.0) as u8;
        let hundredths = ((value * 10.0 - libm::floorf(value * 10.0)) * 10.0) as u8;

        match (hundreds, tens, ones) {
            (0, 0, 0) => {
                self.lcd.print("   ");
                self.lcd.print(sign);
            }
            (0, 0, _) => {
                self.lcd.print("  ");
                self.lcd.print(sign);
                self.lcd.print(digit(ones));
            }
            (0, _, _) => {
                self.lcd.print(" ");
                self.lcd.print(sign);
                self.lcd.print(digit(tens));
                self.lcd.print(digit(ones));
            }
            (_, _, _) => {
                self.lcd.print(sign);
                self.lcd.print(digit(hundreds));
                self.lcd.print(digit(tens));
                self.lcd.print(digit(ones));
            }
        }

        self.lcd.print(".");
        self.lcd.print(digit(tenths));
        self.lcd.print(digit(hundredths));
    }
}

const fn digit(digit: u8) -> &'static str {
    match digit {
        0 => "0",
        1 => "1",
        2 => "2",
        3 => "3",
        4 => "4",
        5 => "5",
        6 => "6",
        7 => "7",
        8 => "8",
        9 => "9",
        _ => unreachable!(),
    }
}
