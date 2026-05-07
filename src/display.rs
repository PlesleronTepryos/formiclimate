//! Display subsystem

use arduino_hal::{
    hal::port::{PB0, PB1, PB2, PB3, PD2, PD3},
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
};

use crate::utils::{hexit, u16_to_f32};

/// Tool for building a page's layout before sending it to the display
#[must_use]
#[repr(C)]
pub struct PageBuilder {
    data: [u8; 80],
    pos: u8,
}

impl PageBuilder {
    /// Create a new blank page to write on
    #[inline(never)]
    pub const fn new() -> Self {
        Self {
            data: [b' '; 80],
            pos: 0,
        }
    }

    /// Write an arbitrary byte to the page, moving the cursor by 1
    ///
    /// Note: If the cursor reaches the end of a line, it automatically jumps to the next. If it
    /// reaches the end of the last line, subsequent bytes are dropped
    #[inline(never)]
    pub const fn byte(mut self, byte: u8) -> Self {
        if self.pos < 80 {
            self.data[self.pos as usize] = byte;
            self.pos += 1;
        }
        self
    }

    /// Write a hexadecimal digit in ascii to the page, moving the cursor by 1
    ///
    /// Note: If the cursor reaches the end of a line, it automatically jumps to the next. If it
    /// reaches the end of the last line, subsequent bytes are dropped
    pub const fn hexit(self, value: u8) -> Self {
        self.byte(hexit(value))
    }

    /// Write a pair of hexadecimal digits in ascii to the page, moving the cursor by 2
    ///
    /// Note: If the cursor reaches the end of a line, it automatically jumps to the next. If it
    /// reaches the end of the last line, subsequent bytes are dropped
    #[inline(never)]
    pub const fn hexit2(self, value: u8) -> Self {
        self.write(&[hexit(value >> 4), hexit(value & 0xf)])
    }

    /// Prints a floating point value in the range `[-999.99, 999.99]` at the current position,
    /// padded so that the cursor always moves by 7 characters
    ///
    /// Leading zeroes are omitted and the sign always appears immediately before the first nonzero
    /// leading digit, or the decimal point if there is none
    pub const fn decimal(self, value: f32) -> Self {
        let [b0, b1, b2, b3] = value.to_le_bytes();
        let sign = if b3 >> 7 == 1 { b'-' } else { b' ' };
        let abs_b3 = b3 & 0x7F;

        let mut out_bytes;
        self.write(if abs_b3 == 0x7F && b2 >> 7 == 1 {
            if b2 & 0x7f != 0 || b1 != 0 || b0 != 0 {
                b"    NaN"
            } else if sign == b'-' {
                b"   -Inf"
            } else {
                b"    Inf"
            }
        } else {
            let abs = f32::from_le_bytes([b0, b1, b2, abs_b3]);

            let trunc = abs as u16;
            let frac = ((abs - u16_to_f32(trunc)) * 100.0) as u8;

            let ones = (trunc % 10) as u8;
            let tens = (trunc / 10) as u8;

            out_bytes = [
                sign,
                b'0' + tens / 10,
                b'0' + tens % 10,
                b'0' + ones,
                b'.',
                b'0' + frac / 10,
                b'0' + frac % 10,
            ];

            let mut i = 1;
            while i < 4 && out_bytes[i] == b'0' {
                out_bytes[i - 1] = b' ';
                out_bytes[i] = sign;
                i += 1;
            }

            &out_bytes
        })
    }

    /// Prints an unsigned integer right-aligned in exactly 6 characters
    pub const fn uint(self, value: u16) -> Self {
        let mut out_bytes;
        self.write(if value == 0 {
            b"     0"
        } else {
            let ones_and_tens = (value % 100) as u8;
            let rest = value / 100;
            let hund_and_thou = (rest % 100) as u8;

            out_bytes = [
                b' ',
                b'0' + (rest / 100) as u8,
                b'0' + hund_and_thou / 10,
                b'0' + hund_and_thou % 10,
                b'0' + ones_and_tens / 10,
                b'0' + ones_and_tens % 10,
            ];

            let mut i = 1;
            while i < 5 && out_bytes[i] == b'0' {
                out_bytes[i] = b' ';
                i += 1;
            }

            &out_bytes
        })
    }

    /// Prints a signed integer right-aligned in exactly 6 characters
    pub const fn sint(self, value: i16) -> Self {
        let sign = if value < 0 { b'-' } else { b' ' };
        let abs_value = value.unsigned_abs();

        let mut out_bytes;
        self.write(if abs_value == 0 {
            b"     0"
        } else {
            let ones_and_tens = (abs_value % 100) as u8;
            let rest = abs_value / 100;
            let hund_and_thou = (rest % 100) as u8;

            out_bytes = [
                sign,
                b'0' + (rest / 100) as u8,
                b'0' + hund_and_thou / 10,
                b'0' + hund_and_thou % 10,
                b'0' + ones_and_tens / 10,
                b'0' + ones_and_tens % 10,
            ];

            let mut i = 1;
            while i < 5 && out_bytes[i] == b'0' {
                out_bytes[i - 1] = b' ';
                out_bytes[i] = sign;
                i += 1;
            }

            &out_bytes
        })
    }

    /// Write a series of bytes to the page, moving the cursor by `bytes.len()`
    ///
    /// Note: If the cursor reaches the end of a line, it automatically jumps to the next. If it
    /// reaches the end of the last line, remaining bytes are dropped
    #[inline(never)]
    pub const fn write(mut self, bytes: &[u8]) -> Self {
        let mut i = 0;
        while self.pos < 80 && i < bytes.len() {
            self.data[self.pos as usize] = bytes[i];
            self.pos += 1;
            i += 1;
        }
        self
    }

    /// Skips ahead by `n` characters
    #[inline(never)]
    pub const fn skip(mut self, n: u8) -> Self {
        self.pos += n;
        if self.pos > 80 {
            self.pos = 80;
        }
        self
    }

    /// Fills the remainder of the current line with blank space and moves to the next line; this is
    /// a no-op at the very beginning of a line
    #[inline(never)]
    pub const fn end_line(mut self) -> Self {
        self.pos = if self.pos <= 20 {
            20
        } else if self.pos <= 40 {
            40
        } else if self.pos <= 60 {
            60
        } else {
            80
        };
        self
    }

    /// Jumps cursor to the beginning of next line uncondtionally
    #[inline(never)]
    pub const fn next_line(mut self) -> Self {
        self.pos = if self.pos < 20 {
            20
        } else if self.pos < 40 {
            40
        } else if self.pos < 60 {
            60
        } else {
            80
        };
        self
    }

    /// Finalize the page
    pub const fn finish(self) -> PageData {
        PageData {
            data: self.data,
            _filler: 0,
        }
    }
}

impl Default for PageBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// A complete page ready to be sent to the display
#[must_use]
#[repr(C)]
pub struct PageData {
    data: [u8; 80],
    _filler: u8,
}

impl PageData {
    /// A completely blank page
    pub const BLANK: Self = Self {
        data: [b' '; 80],
        _filler: 0,
    };

    /// Rewrite the existing page data
    pub const fn rewrite(self) -> PageBuilder {
        PageBuilder {
            data: self.data,
            pos: 0,
        }
    }
}

/// Climate controller display subsystem
///
/// Note: optimized for binary size at the cost of generic utility
#[must_use]
pub struct Display {
    rs: Pin<Output, PD2>,
    en: Pin<Output, PD3>,
    d4: Pin<Output, PB0>,
    d5: Pin<Output, PB1>,
    d6: Pin<Output, PB2>,
    d7: Pin<Output, PB3>,

    page: PageData,
}

impl Display {
    /// Construct the display
    #[expect(clippy::similar_names, reason = "I didn't name the pins")]
    pub fn new(
        pd2: Pin<Input<Floating>, PD2>,
        pd3: Pin<Input<Floating>, PD3>,
        pb0: Pin<Input<Floating>, PB0>,
        pb1: Pin<Input<Floating>, PB1>,
        pb2: Pin<Input<Floating>, PB2>,
        pb3: Pin<Input<Floating>, PB3>,
    ) -> Self {
        Self {
            rs: pd2.into_output(),
            en: pd3.into_output(),
            d4: pb0.into_output(),
            d5: pb1.into_output(),
            d6: pb2.into_output(),
            d7: pb3.into_output(),

            page: PageData::BLANK,
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
        self.command(0x80 | (col + OFFSETS[(row & 0x3) as usize]));
        arduino_hal::delay_us(100);
    }

    fn command(&mut self, cmd: u8) {
        self.send8(cmd, false);
    }

    fn write(&mut self, value: u8) {
        self.send8(value, true);
        arduino_hal::delay_us(100);
    }

    #[inline(never)]
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

    /// Write an entire page worth of data to the display
    ///
    /// # Performance
    /// Execution time is variable based on how much of the new page is different from the last and
    /// how the differences are arranged. Roughly speaking, this function takes
    /// `([characters changed] + [runs of unchanged characters]) * 100`us
    ///
    /// At worst, this will take ~8-9ms in either of two cases:
    /// - if every single character in the new page is different from the last (80 new characters)
    /// - if every other character is different (40 new characters with 40 unchanged runs between)
    ///
    /// Any other situation will take less time, down to ~400us with a completely identical page
    pub fn write_page(&mut self, page: PageData) {
        let mut i = 0;
        let mut col = 0;
        let mut row = 0;

        let mut skip = true;

        while i < 80 {
            let byte = page.data[i];
            if byte == self.page.data[i] {
                skip = true;
            } else {
                if skip {
                    self.set_pos(col, row);
                    skip = false;
                }
                self.write(byte);
            }

            i += 1;
            col += 1;

            if col == 20 {
                col = 0;
                row += 1;
                skip = true;
            }
        }

        self.page = page;
    }
}
