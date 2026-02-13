//! Faithful implementation of Arduino `millis()`

use core::cell::Cell;

use arduino_hal::{clock::Clock, pac::TC0, DefaultClock};
use avr_device::interrupt::Mutex;

const MICROS_PER_SECOND: u32 = 1_000_000;
const CYCLES_PER_MICRO: u32 = DefaultClock::FREQ / MICROS_PER_SECOND;
const MICROS_PER_OVF: u32 = 64 * 256 / CYCLES_PER_MICRO;

const MILLIS_INC: u32 = MICROS_PER_OVF / 1000;
const FRACT_INC: u32 = MICROS_PER_OVF.rem_euclid(1000) >> 3;
const FRACT_MAX: u32 = 1000 >> 3;

static OVERFLOWS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
static MILLIS: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));
static FRACT: Mutex<Cell<u32>> = Mutex::new(Cell::new(0));

#[expect(clippy::allow_attributes, reason = "expect somehow doesn't work")]
#[allow(missing_docs, reason = "macro expansion breaks doc comments")]
mod internal {
    use super::{FRACT, FRACT_INC, FRACT_MAX, MILLIS, MILLIS_INC, OVERFLOWS};

    #[avr_device::interrupt(atmega32u4)]
    fn TIMER0_OVF() {
        avr_device::interrupt::free(|cs| {
            let millis_cell = MILLIS.borrow(cs);
            let fract_cell = FRACT.borrow(cs);
            let mut millis = millis_cell.get();
            let mut fract = fract_cell.get();

            millis += MILLIS_INC;
            fract += FRACT_INC;
            if fract >= FRACT_MAX {
                fract -= FRACT_MAX;
                millis += 1;
            }

            fract_cell.set(fract);
            millis_cell.set(millis);
            OVERFLOWS.borrow(cs).update(|x| x + 1);
        });
    }
}

/// Initialize Timer0 to track milliseconds
pub fn init_millis(tc0: &TC0) {
    tc0.tccr0a().write(|w| w.wgm0().pwm_fast());
    tc0.tccr0b().write(|w| w.cs0().prescale_64());
    tc0.timsk0().write(|w| w.toie0().set_bit());

    avr_device::interrupt::free(|cs| {
        MILLIS.borrow(cs).set(0);
    });
}

/// Milliseconds since last reset
#[must_use]
pub fn millis() -> u32 {
    avr_device::interrupt::free(|cs| MILLIS.borrow(cs).get())
}
