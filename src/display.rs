//! Display subsystem

use arduino_hal::{
    hal::port::{PB0, PB1, PB2, PB3, PD2, PD3},
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
};

/// Tool for building a page's layout before sending it to the display
#[must_use]
#[repr(C)]
pub struct PageBuilder {
    data: [u8; 80],
    pos: u8,
}

impl PageBuilder {
    /// Create a new blank page to write on
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
    pub const fn hexit2(self, value: u8) -> Self {
        self.write(&[hexit(value >> 4), hexit(value & 0xf)])
    }

    /// Prints a floating point value in the range `[-999.99, 999.99]` at the current position,
    /// padded so that the cursor always moves by 7 characters
    ///
    /// Leading zeroes are omitted and the sign always appears immediately before the first nonzero
    /// leading digit, or the decimal point if there is none
    pub const fn decimal(self, value: f32) -> Self {
        if value.is_nan() {
            return self.write(b"    NaN");
        }

        let sign = if value.is_sign_negative() { b'-' } else { b' ' };

        if value.is_infinite() {
            return self.write(b"   ").byte(sign).write(b"Inf");
        }

        let mut value = value.abs();

        let mut out_bytes = [
            sign,
            hexit({
                let mut hundreds = 0;
                while value >= 100.0 {
                    hundreds += 1;
                    value -= 100.0;
                }
                hundreds
            }),
            hexit({
                let mut tens = 0;
                while value >= 10.0 {
                    tens += 1;
                    value -= 10.0;
                }
                tens
            }),
            hexit({
                let mut ones = 0;
                while value >= 1.0 {
                    ones += 1;
                    value -= 1.0;
                }
                ones
            }),
            b'.',
            hexit({
                let mut tenths = 0;
                while value >= 0.1 {
                    tenths += 1;
                    value -= 0.1;
                }
                tenths
            }),
            hexit({
                let mut hundredths = 0;
                while value >= 0.01 {
                    hundredths += 1;
                    value -= 0.01;
                }
                hundredths
            }),
        ];

        let mut i = 1;
        while i < 4 && out_bytes[i] == b'0' {
            out_bytes[i - 1] = b' ';
            out_bytes[i] = sign;
            i += 1;
        }

        self.write(&out_bytes)
    }

    /// Write a series of bytes to the page, moving the cursor by `bytes.len()`
    ///
    /// Note: If the cursor reaches the end of a line, it automatically jumps to the next. If it
    /// reaches the end of the last line, remaining bytes are dropped
    pub const fn write(mut self, bytes: &[u8]) -> Self {
        let mut i = 0;
        while self.pos < 80 && i < bytes.len() {
            self.data[self.pos as usize] = bytes[i];
            self.pos += 1;
            i += 1;
        }
        self
    }

    /// Fills the remainder of the current line with blank space and moves to the next line; this is
    /// a no-op at the very beginning of a line
    pub const fn end_line(mut self) -> Self {
        self.pos = self.pos.next_multiple_of(20);
        self
    }

    /// Jumps cursor to the beginning of next line uncondtionally
    pub const fn next_line(mut self) -> Self {
        if self.pos < 80 {
            self.pos = (self.pos + 1).next_multiple_of(20);
        }
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
    /// In the worst case, this will take ~8-9ms in either of two cases:
    /// - if every single character in the new page is different from the last (80 new characters)
    /// - if every other character is different (40 new characters with 40 unchanged runs between)
    ///
    /// Any other situation will take less time, down to ~400us with a completely identical page
    pub fn write_page(&mut self, page: PageData) {
        let mut i = 0;
        let mut col = 0;
        let mut row = 0;

        self.set_pos(col, row);
        let mut skip = false;

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
                self.set_pos(col, row);
                skip = false;
            }
        }

        self.page = page;
    }
}

const fn hexit(hexit: u8) -> u8 {
    match hexit & 0xf {
        0x0 => b'0',
        0x1 => b'1',
        0x2 => b'2',
        0x3 => b'3',
        0x4 => b'4',
        0x5 => b'5',
        0x6 => b'6',
        0x7 => b'7',
        0x8 => b'8',
        0x9 => b'9',
        0xa => b'A',
        0xb => b'B',
        0xc => b'C',
        0xd => b'D',
        0xe => b'E',
        0xf => b'F',
        _ => unreachable!(),
    }
}
