//! Various utility functions and size-optimized implementations of standard library functions

use core::f32;

/// Natural log approximation adapted from [<https://quadst.rip/ln-approx>]
///
/// This saves nearly 500 bytes of program memory as opposed to `libm::logf` and has a max relative
/// error of ~6.06e-5, which is more than suitable for this application
#[must_use]
pub const fn ln(x: f32) -> f32 {
    let bx = x.to_bits();
    let t = i16_to_f32((bx >> 23) as i16 - 127);
    let y = f32::from_bits(1_065_353_216 | (bx & 8_388_607));
    -1.741_793_9
        + (2.821_202_6 + (-1.469_956_8 + (0.447_179_55 - 0.056_570_85 * y) * y) * y) * y
        + f32::consts::LN_2 * t
}

/// Reciprocal approximation adapted from [<https://news.ycombinator.com/item?id=42573188>]
#[must_use]
pub const fn recip(x: f32) -> f32 {
    let bx = x.to_bits();
    let y = f32::from_bits(0x7ef3_11c2 - bx);
    let y = y * (2.0 - x * y);
    // let y = y * (2.0 - x * y);
    y * (2.0 - x * y)
}

/// Efficently checks if an [`f32`] is not `Inf`, `-Inf`, or `NaN`
#[must_use]
pub const fn is_finite(x: f32) -> bool {
    let [_, _, b2, b3] = x.to_le_bytes();
    b3 & 0x7f != 0x7f || b2 >> 7 != 1
}

/// This saves ~250 additional bytes over the builtin `x as f32` conversion
#[must_use]
pub const fn i16_to_f32(x: i16) -> f32 {
    match x {
        0 => 0.0,
        i16::MIN => -32768.0,
        x => {
            let sign: u32 = if x < 0 { 1 } else { 0 };
            let abs = (if x < 0 { -x } else { x }) as u16 as u32;

            let mut bit_pos = 0u32;
            let mut tmp = abs >> 1;
            while tmp != 0 {
                tmp >>= 1;
                bit_pos += 1;
            }

            let exp = bit_pos + 127;
            let mantissa = (abs ^ (1 << bit_pos)) << (23 - bit_pos);
            f32::from_bits((sign << 31) | (exp << 23) | mantissa)
        }
    }
}

/// This saves space over the builtin `x as f32` conversion, for the same reason as [`i16_to_f32`]
#[must_use]
pub const fn u16_to_f32(x: u16) -> f32 {
    match x {
        0 => 0.0,
        x => {
            let abs = x as u32;

            let mut bit_pos = 0u32;
            let mut tmp = abs >> 1;
            while tmp != 0 {
                tmp >>= 1;
                bit_pos += 1;
            }

            let exp = bit_pos + 127;
            let mantissa = (abs ^ (1 << bit_pos)) << (23 - bit_pos);
            f32::from_bits((exp << 23) | mantissa)
        }
    }
}

/// Convert the lower 4 bits of a [u8] to a hexadecimal character
///
/// Also works for decimal values
#[must_use]
pub const fn hexit(hexit: u8) -> u8 {
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
