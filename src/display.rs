//! Display subsystem

use core::mem::discriminant;

use arduino_hal::{
    hal::port::{PB0, PB1, PB2, PB3, PD4, PD6},
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
};

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
///
/// Note: optimized for binary size at the cost of generic utility
#[must_use]
pub struct Display {
    rs: Pin<Output, PD6>,
    en: Pin<Output, PD4>,
    d4: Pin<Output, PB0>,
    d5: Pin<Output, PB1>,
    d6: Pin<Output, PB2>,
    d7: Pin<Output, PB3>,

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
        Self {
            rs: d12.into_output(),
            en: d4.into_output(),
            d4: led_rx.into_output(),
            d5: sck.into_output(),
            d6: mosi.into_output(),
            d7: miso.into_output(),

            page: Page::Blank,
            needs_clear: false,
        }
    }

    /// Initialize the display
    pub fn init(&mut self) {
        self.set_func(0x08); // 4-bit bus; two lines; 5x8 char size
        self.set_ctrl(0x04); // Display on; cursor/blink off
        self.set_mode(0x02); // Left-to-right layout; no display shift
        self.clear();
        self.home();
    }

    fn clear(&mut self) {
        self.command(0x01);
        arduino_hal::delay_us(3000);
    }

    fn home(&mut self) {
        self.command(0x02);
        arduino_hal::delay_us(3000);
    }

    fn set_mode(&mut self, mode: u8) {
        self.command(0x04 | mode);
        arduino_hal::delay_us(100);
    }

    fn set_ctrl(&mut self, ctrl: u8) {
        self.command(0x08 | ctrl);
        arduino_hal::delay_us(100);
    }

    fn set_func(&mut self, func: u8) {
        self.command(0x20 | func);
        arduino_hal::delay_us(100);
    }

    fn set_pos(&mut self, col: u8, row: u8) {
        const OFFSETS: [u8; 4] = [0x00, 0x40, 0x14, 0x54];
        let pos = col + OFFSETS[(row & 0x3) as usize];
        self.command(0x80 | pos);
        arduino_hal::delay_us(100);
    }

    fn command(&mut self, cmd: u8) {
        self.send8(cmd, false);
    }

    fn print(&mut self, text: &str) {
        for ch in text.chars() {
            self.write(ch as u8);
        }
    }

    fn write(&mut self, value: u8) {
        self.send8(value, true);
        arduino_hal::delay_us(100);
    }

    fn send8(&mut self, byte: u8, mode: bool) {
        if mode {
            self.rs.set_high();
        } else {
            self.rs.set_low();
        }

        self.send4(byte >> 4);
        self.send4(byte & 0xf);
    }

    fn send4(&mut self, half_byte: u8) {
        if half_byte & 0b1000 != 0 {
            self.d7.set_high();
        } else {
            self.d7.set_low();
        }
        if half_byte & 0b0100 != 0 {
            self.d6.set_high();
        } else {
            self.d6.set_low();
        }
        if half_byte & 0b0010 != 0 {
            self.d5.set_high();
        } else {
            self.d5.set_low();
        }
        if half_byte & 0b0001 != 0 {
            self.d4.set_high();
        } else {
            self.d4.set_low();
        }
        self.pulse();
    }

    fn pulse(&mut self) {
        self.en.set_high();
        self.en.set_low();
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
        if self.needs_clear {
            self.clear();
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

                self.set_pos(0, 0);
                self.print("Today is ");
                self.print(time.day.name());

                self.set_pos(0, 1);
                self.print("20");
                self.print_bcd(time.year.bcd());
                self.print(".");
                self.print_bcd(time.month.bcd());
                self.print(".");
                self.print_bcd(time.date.bcd());
                self.print("  ");
                self.print_bcd(time.hours.bcd_24h());
                self.print(":");
                self.print_bcd(time.minutes.bcd());
                self.print(":");
                self.print_bcd(time.seconds.bcd());
            }
            Page::Time(Err(msg)) => {
                self.set_pos(0, 0);
                self.print(msg);
                self.set_pos(0, 1);
                self.print(BLANK_LINE);
                self.set_pos(0, 2);
                self.print(BLANK_LINE);
            }
        }

        self.needs_clear = false;
    }

    fn print_temp(&mut self, row: u8, title: &str, temp: f32, first: bool) {
        if first {
            self.set_pos(0, row);
            self.print(title);
        } else {
            self.set_pos(title.len() as u8, row);
        }

        self.print_float(temp);

        self.print("F");
    }

    /// Prints a 2-digit BCD value
    fn print_bcd(&mut self, value: u8) {
        self.print(digit(value >> 4));
        self.print(digit(value & 0xf));
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
                self.print("   ");
                self.print(sign);
            }
            (0, 0, _) => {
                self.print("  ");
                self.print(sign);
                self.print(digit(ones));
            }
            (0, _, _) => {
                self.print(" ");
                self.print(sign);
                self.print(digit(tens));
                self.print(digit(ones));
            }
            (_, _, _) => {
                self.print(sign);
                self.print(digit(hundreds));
                self.print(digit(tens));
                self.print(digit(ones));
            }
        }

        self.print(".");
        self.print(digit(tenths));
        self.print(digit(hundredths));
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
