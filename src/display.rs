//! Display subsystem

use arduino_hal::{
    hal::port::{PB0, PB1, PB2, PB3, PD2, PD3},
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
};

/// A complete page ready to be sent to the display
#[derive(Clone)]
#[must_use]
#[repr(C)]
pub struct PageData {
    data: [u8; 80],
}

impl PageData {
    const BLANK: Self = Self { data: [b' '; 80] };

    /// Create a new blank page (all spaces)
    pub const fn blank() -> Self {
        Self::BLANK
    }

    /// Create a new page from the given character data
    pub const fn new(data: [u8; 80]) -> Self {
        Self { data }
    }

    /// Reset the page to blank (all spaces)
    pub const fn clear(&mut self) {
        *self = Self::BLANK;
    }

    /// Position after padding the current line to its end (no-op if already at a line boundary)
    #[must_use]
    pub const fn end_line_pos(pos: usize) -> usize {
        if pos <= 20 {
            20
        } else if pos <= 40 {
            40
        } else if pos <= 60 {
            60
        } else if pos <= 80 {
            80
        } else {
            panic!("invalid positon")
        }
    }

    /// Position at the start of the next line
    #[must_use]
    pub const fn next_line_pos(pos: usize) -> usize {
        if pos < 20 {
            20
        } else if pos < 40 {
            40
        } else if pos < 60 {
            60
        } else if pos < 80 {
            80
        } else {
            panic!("invalid positon")
        }
    }

    /// Write a single byte at `pos`
    pub const fn write_byte(&mut self, pos: usize, byte: u8) {
        self.data[pos] = byte;
    }

    /// Write the first `n` bytes of `bytes` starting at `pos`
    pub const fn write_bytes(&mut self, pos: usize, bytes: &[u8], n: usize) {
        let mut i = 0;
        while i < n {
            self.data[pos + i] = bytes[i];
            i += 1;
        }
    }

    /// Return the underlying character data
    #[must_use]
    pub const fn into_data(self) -> [u8; 80] {
        self.data
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

    page_a: PageData,
    page_b: PageData,
    which: bool,
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

            page_a: PageData::BLANK,
            page_b: PageData::BLANK,
            which: false,
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

    const fn front(&self) -> &PageData {
        if self.which {
            &self.page_a
        } else {
            &self.page_b
        }
    }

    const fn back(&self) -> &PageData {
        if self.which {
            &self.page_b
        } else {
            &self.page_a
        }
    }

    /// Returns a mutable reference to the page not currently on display
    pub const fn back_mut(&mut self) -> &mut PageData {
        if self.which {
            &mut self.page_b
        } else {
            &mut self.page_a
        }
    }

    /// Present the back buffer to the display, sending only characters that differ from the
    /// currently displayed page
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
    pub fn swap(&mut self) {
        let mut i = 0;
        let mut col = 0;
        let mut row = 0;

        let mut skip = true;

        self.which = !self.which;

        while i < 80 {
            let byte = self.front().data[i];
            if byte == self.back().data[i] {
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
    }
}

/// Build a [`PageData`] with compile-time cursor tracking and bounds checking
///
/// # Invocation forms
///
/// - `page!(cmds...)`: creates and returns a new [`PageData`]
/// - `page!(rewrite <page>; cmds...)`: clears and rewrites an existing [`PageData`]
///
/// # Commands (separated by `;`)
///
/// - `write b"..."`: write a fixed-length byte string literal
/// - `write N <expr>`: write the first `N` characters of any `&[u8]` expression
/// - `decimal <expr>`: render an `f32` in 7 characters (eg. `-999.99`)
/// - `uint <expr>`: render a `u16` in 5 characters (eg. `65535`)
/// - `sint <expr>`: render an `i16` in 6 characters (eg. `-32768`)
/// - `byte <expr>`: write a single character
/// - `byte b'' if <expr>`: write the given byte only if `expr` is `true`, otherwise leave blank
/// - `hexit2 <expr>`: render a `u8` as two hexadecimal characters (also works for BCD values)
/// - `skip N`: skip ahead by `N` characters (`N` must be an integer literal)
/// - `end_line`: jump to the end of the current line (no-op if already at the end)
/// - `next_line`: jump to the start of the next line (skips entire line if already at the start)
/// - `end_page`: jump to the end of the whole page
/// - `if NAME (<cond>) { cmds... } else { cmds... }`: conditional; `NAME` should be a unique
///   `const` identifier within this invocation of `page!()`; `cond` may be an expression or a `let`
///   pattern; both branches must advance the cursor by the same number of characters
/// - `match NAME (<expr>) { <pat> => { cmds... } , ... }`: pattern match; `NAME` should be a unique
///   `const` identifier within this invocation of `page!()`; all arms must advance the cursor by
///   the same number of characters
#[macro_export]
macro_rules! page {
    // Command parsing
    (@s $d:ident [$pe:expr]) => {};
    (@s $d:ident [$pe:expr] write $bytes:literal; $($r:tt)*) => {
        const { assert!($pe + $bytes.len() <= 80usize, "page overflow") };
        $d.write_bytes($pe, $bytes, $bytes.len());
        $crate::page!(@s $d [$pe + $bytes.len()] $($r)*);
    };
    (@s $d:ident [$pe:expr] write 2 $bytes:expr; $($r:tt)*) => {
        const { assert!($pe + 2 <= 80usize, "page overflow") };
        {
            let [__v0, __v1] = *$bytes;
            $d.write_byte($pe, __v0);
            $d.write_byte($pe + 1, __v1);
        }
        $crate::page!(@s $d [$pe + 2] $($r)*);
    };
    (@s $d:ident [$pe:expr] write 3 $bytes:expr; $($r:tt)*) => {
        const { assert!($pe + 3 <= 80usize, "page overflow") };
        {
            let [__v0, __v1, __v2] = *$bytes;
            $d.write_byte($pe, __v0);
            $d.write_byte($pe + 1, __v1);
            $d.write_byte($pe + 2, __v2);
        }
        $crate::page!(@s $d [$pe + 3] $($r)*);
    };
    (@s $d:ident [$pe:expr] write $n:literal $bytes:expr; $($r:tt)*) => {
        const { assert!($pe + $n <= 80usize, "page overflow") };
        $d.write_bytes($pe, $bytes, $n);
        $crate::page!(@s $d [$pe + $n] $($r)*);
    };
    (@s $d:ident [$pe:expr] decimal $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe] write 7 &$crate::utils::f32_to_bytes($v); $($r)*);
    };
    (@s $d:ident [$pe:expr] uint $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe] write 5 &$crate::utils::u16_to_bytes($v); $($r)*);
    };
    (@s $d:ident [$pe:expr] sint $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe] write 6 &$crate::utils::i16_to_bytes($v); $($r)*);
    };
    (@s $d:ident [$pe:expr] byte b' '; $($r:tt)*) => {
        $crate::page!(@s $d [$pe] skip 1; $($r)*);
    };
    (@s $d:ident [$pe:expr] byte $b:literal if $cond:expr; $($r:tt)*) => {
        const { assert!($pe < 80usize, "page overflow") };
        if $cond {
            $d.write_byte($pe, $b);
        }
        $crate::page!(@s $d [$pe + 1] $($r)*);
    };
    (@s $d:ident [$pe:expr] byte $b:expr; $($r:tt)*) => {
        const { assert!($pe < 80usize, "page overflow") };
        $d.write_byte($pe, $b);
        $crate::page!(@s $d [$pe + 1] $($r)*);
    };
    (@s $d:ident [$pe:expr] hexit2 $v:expr; $($r:tt)*) => {
        const { assert!($pe + 2 <= 80usize, "page overflow") };
        {
            let __v = $v;
            $d.write_byte($pe, $crate::utils::hexit(__v >> 4));
            $d.write_byte($pe + 1, $crate::utils::hexit(__v & 0xf));
        }
        $crate::page!(@s $d [$pe + 2] $($r)*);
    };
    (@s $d:ident [$pe:expr] skip 1; $($r:tt)*) => {
        const { assert!($pe < 80usize, "page overflow") };
        $crate::page!(@s $d [$pe + 1] $($r)*);
    };
    (@s $d:ident [$pe:expr] skip $n:literal; $($r:tt)*) => {
        const { assert!($pe + $n <= 80usize, "page overflow") };
        $crate::page!(@s $d [$pe + $n] $($r)*);
    };
    (@s $d:ident [$pe:expr] end_line; $($r:tt)*) => {
        $crate::page!(@s $d [$crate::display::PageData::end_line_pos($pe)] $($r)*);
    };
    (@s $d:ident [$pe:expr] next_line; $($r:tt)*) => {
        const { assert!($pe < 80usize, "past last line") };
        $crate::page!(@s $d [$crate::display::PageData::next_line_pos($pe)] $($r)*);
    };
    (@s $d:ident [$pe:expr] end_page; $($r:tt)*) => {
        $crate::page!(@s $d [80] $($r)*);
    };
    (@s $d:ident [$pe:expr] if $name:ident ($($cond:tt)*) { $($if_t:tt)* } else { $($if_f:tt)* } $($r:tt)*) => {{
        const $name: usize = $crate::page!(@c [0usize] $($if_t)*);
        const {
            assert!($crate::page!(@c [0usize] $($if_f)*) == $name, "if/else branches must have equal length");
            assert!(($pe) + $name <= 80usize, "if/else block exceeds end of page");
        };
        if $($cond)* {
            $crate::page!(@s $d [$pe] $($if_t)*);
        } else {
            $crate::page!(@s $d [$pe] $($if_f)*);
        }
        $crate::page!(@s $d [($pe) + $name] $($r)*);
    }};
    (@s $d:ident [$pe:expr] match $name:ident ($e:expr) { $first_p:pat => { $($first_arm:tt)* } $(,)? $( $p:pat => { $($arm:tt)* } $(,)? )* } $($r:tt)*) => {{
        const $name: usize = $crate::page!(@c [0usize] $($first_arm)*);
        const {
            $( assert!($crate::page!(@c [0usize] $($arm)*) == $name, "match arms must have equal length"); )*
            assert!(($pe) + $name <= 80usize, "match block exceeds end of page");
        };
        match $e {
            $first_p => { $crate::page!(@s $d [$pe] $($first_arm)*); },
            $( $p => { $crate::page!(@s $d [$pe] $($arm)*); } ),*
        }
        $crate::page!(@s $d [($pe) + $name] $($r)*);
    }};

    // Fixed-width fields; these commands should only be used by the code generated by the
    // `interactive!()` macro; intentionally undocumented
    (@s $d:ident [$pe:expr] field bool $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 5] if __BOOL ($v) { skip 1; write b"On"; } else { write b"Off"; } $($r)*);
    };
    (@s $d:ident [$pe:expr] field u8 $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 3] uint $v as u16; $($r)*);
    };
    (@s $d:ident [$pe:expr] field u16 $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 3] uint $v; $($r)*);
    };
    (@s $d:ident [$pe:expr] field u32 $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 3] uint $v as u16; $($r)*);
    };
    (@s $d:ident [$pe:expr] field i8 $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 2] sint $v as i16; $($r)*);
    };
    (@s $d:ident [$pe:expr] field i16 $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 2] sint $v; $($r)*);
    };
    (@s $d:ident [$pe:expr] field i32 $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 2] sint $v as i16; $($r)*);
    };
    (@s $d:ident [$pe:expr] field f32 $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe] decimal $v; byte b'F'; $($r)*);
    };
    (@s $d:ident [$pe:expr] field Month $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 5] write 3 $v.abbrev(); $($r)*);
    };
    (@s $d:ident [$pe:expr] field Date $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 4] hexit2 $v.bcd(); write 2 $v.suffix(); $($r)*);
    };
    (@s $d:ident [$pe:expr] field Duty $v:expr; $($r:tt)*) => {
        $crate::page!(@s $d [$pe + 3] uint $v.0; $($r)*);
    };

    // Cursor advance calculation
    (@c [$pe:expr]) => { $pe };
    (@c [$pe:expr] write $bytes:literal; $($r:tt)*) => {
        $crate::page!(@c [$pe + $bytes.len()] $($r)*)
    };
    (@c [$pe:expr] write $n:literal $_b:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + $n] $($r)*)
    };
    (@c [$pe:expr] decimal $_v:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + 7] $($r)*)
    };
    (@c [$pe:expr] uint $_v:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + 5] $($r)*)
    };
    (@c [$pe:expr] sint $_v:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + 6] $($r)*)
    };
    (@c [$pe:expr] byte $_b:literal if $_cond:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + 1] $($r)*)
    };
    (@c [$pe:expr] byte $_b:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + 1] $($r)*)
    };
    (@c [$pe:expr] hexit2 $_v:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + 2] $($r)*)
    };
    (@c [$pe:expr] skip 1; $($r:tt)*) => {
        $crate::page!(@c [$pe + 1] $($r)*)
    };
    (@c [$pe:expr] skip $n:literal; $($r:tt)*) => {
        $crate::page!(@c [$pe + $n] $($r)*)
    };
    (@c [$pe:expr] end_line; $($r:tt)*) => {
        $crate::page!(@c [$crate::display::PageData::end_line_pos($pe)] $($r)*)
    };
    (@c [$pe:expr] next_line; $($r:tt)*) => {
        $crate::page!(@c [$crate::display::PageData::next_line_pos($pe)] $($r)*)
    };
    (@c [$pe:expr] end_page; $($r:tt)*) => {
        $crate::page!(@c [80] $($r)*)
    };
    (@c [$pe:expr] if $_name:ident ($($cond:tt)*) { $($if_t:tt)* } else { $($if_f:tt)* } $($r:tt)*) => {
        $crate::page!(@c [$pe + $crate::page!(@c [0usize] $($if_t)*)] $($r)*)
    };
    (@c [$pe:expr] match $_name:ident ($_e:expr) { $_first_p:pat => { $($first_arm:tt)* } $(,)? $( $_p:pat => { $($_arm:tt)* } $(,)? )* } $($r:tt)*) => {
        $crate::page!(@c [$pe + $crate::page!(@c [0usize] $($first_arm)*)] $($r)*)
    };
    (@c [$pe:expr] field $_t:tt $_v:expr; $($r:tt)*) => {
        $crate::page!(@c [$pe + 8] $($r)*)
    };

    (rewrite $page:expr; $($tok:tt)*) => {{
        let __page = $page;
        __page.clear();
        $crate::page!(@s __page [0usize] $($tok)*);
    }};

    ($($tok:tt)*) => {{
        let mut __page = $crate::display::PageData::blank();
        $crate::page!(@s __page [0usize] $($tok)*);
        __page
    }};
}
