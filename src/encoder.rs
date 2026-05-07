//! Abstraction for rotary encoder user interface

use core::cell::{Cell, OnceCell};

use arduino_hal::{
    hal::port::{PB4, PE2, PE6},
    pac::{EXINT, TC4},
    port::{
        mode::{Floating, Input, PullUp},
        Pin,
    },
};
use avr_device::interrupt::{CriticalSection, Mutex};

const COOLDOWN: u8 = 5;

static QUEUED_CLICKS: Mutex<Cell<i8>> = Mutex::new(Cell::new(0));
static QUEUED_PRESSES: Mutex<Cell<u8>> = Mutex::new(Cell::new(0));

static ENC_A: Mutex<OnceCell<Pin<Input<PullUp>, PE2>>> = Mutex::new(OnceCell::new());
static BTN: Mutex<OnceCell<Pin<Input<PullUp>, PB4>>> = Mutex::new(OnceCell::new());
static TC4: Mutex<OnceCell<TC4>> = Mutex::new(OnceCell::new());
static EXINT: Mutex<OnceCell<EXINT>> = Mutex::new(OnceCell::new());

fn start_cooldown(cs: CriticalSection<'_>) {
    EXINT.borrow(cs).get().inspect(|exint| {
        exint.pcmsk0().reset();
    });

    TC4.borrow(cs).get().inspect(|tc4| {
        tc4.tccr4b().write(|w| {
            w.psr4().set_bit();
            w.cs4().prescale_16384()
        });
    });
}

#[expect(clippy::allow_attributes, reason = "expect somehow doesn't work")]
#[allow(missing_docs, reason = "macro expansion breaks doc comments")]
mod interrupts {
    use arduino_hal::{
        hal::port::PB4,
        port::{
            mode::{Input, PullUp},
            Pin,
        },
    };

    use super::{start_cooldown, BTN, ENC_A, EXINT, QUEUED_CLICKS, QUEUED_PRESSES, TC4};

    #[avr_device::interrupt(atmega32u4)]
    fn INT6() {
        avr_device::interrupt::free(|cs| {
            ENC_A.borrow(cs).get().inspect(|enc_a| {
                QUEUED_CLICKS
                    .borrow(cs)
                    .update(|s| s + if enc_a.is_low() { 1 } else { -1 });
            });

            //start_cooldown(cs);
        });
    }

    #[avr_device::interrupt(atmega32u4)]
    fn PCINT0() {
        avr_device::interrupt::free(|cs| {
            start_cooldown(cs);
        });
    }

    #[avr_device::interrupt(atmega32u4)]
    fn TIMER4_OVF() {
        avr_device::interrupt::free(|cs| {
            TC4.borrow(cs).get().inspect(|tc4| {
                tc4.tccr4b().write(|w| w.cs4().no_clock());
            });

            EXINT.borrow(cs).get().inspect(|exint| {
                exint.pcmsk0().write(|w| w.set(0b0001_0000)); // Sensitize PB4
            });

            if BTN
                .borrow(cs)
                .get()
                .is_some_and(Pin::<Input<PullUp>, PB4>::is_low)
            {
                QUEUED_PRESSES.borrow(cs).update(|p| p + 1);
            }
        });
    }
}

/// The direction in which the [`Encoder`] was clicked
#[derive(Clone, Copy)]
pub enum Click {
    /// Counter-clockwise; anti-clockwise; widdershins; tuathal
    CCW,

    /// Clockwise; turnwise; sunwise; deasil
    CW,
}

/// Rotary encoder with push button
#[must_use]
pub struct Encoder {
    _enc_b: Pin<Input<PullUp>, PE6>,
}

impl Encoder {
    /// Create and initialize encoder/button input handling
    pub fn new(
        enc_a: Pin<Input<Floating>, PE2>,
        enc_b: Pin<Input<Floating>, PE6>,
        btn: Pin<Input<Floating>, PB4>,
        exint: EXINT,
        tc4: TC4,
    ) -> Self {
        exint.eimsk().reset();
        exint.eicra().reset();
        exint.eicrb().write(|w| w.isc6().set(0b10)); // Enable INT6 on falling edge
        exint.eimsk().write(|w| w.int().set(0b0100_0000)); // Sensitize PE6

        exint.pcmsk0().reset();
        exint.pcicr().write(|w| w.pcie0().set_bit()); // Enable PCINT0
        exint.pcmsk0().write(|w| w.set(0b0001_0000)); // Sensitize PB4

        tc4.ocr4c().write(|w| w.set(COOLDOWN));
        tc4.timsk4().write(|w| w.toie4().set_bit());

        avr_device::interrupt::free(|cs| {
            let _ = ENC_A.borrow(cs).set(enc_a.into_pull_up_input());
            let _ = BTN.borrow(cs).set(btn.into_pull_up_input());
            let _ = TC4.borrow(cs).set(tc4);
            let _ = EXINT.borrow(cs).set(exint);
        });

        Self {
            _enc_b: enc_b.into_pull_up_input(),
        }
    }

    /// Returns `true` if the button has been pressed since the last call to `was_pressed`
    ///
    /// Note: button presses are accumulated, so this method will return `true` exactly as many
    /// times as the button has been pressed up to the moment of invocation
    #[must_use]
    pub fn was_pressed(&self) -> bool {
        avr_device::interrupt::free(|cs| {
            let presses_cell = QUEUED_PRESSES.borrow(cs);
            match presses_cell.get() {
                0 => false,
                p => {
                    presses_cell.set(p - 1);
                    true
                }
            }
        })
    }

    /// Returns which direction the encoder was last clicked, if any
    #[must_use]
    pub fn next_click(&self) -> Option<Click> {
        avr_device::interrupt::free(|cs| {
            let clicks_cell = QUEUED_CLICKS.borrow(cs);
            match clicks_cell.get() {
                p @ i8::MIN..=-1 => {
                    clicks_cell.set(p + 1);
                    Some(Click::CW)
                }
                0 => None,
                p @ 1..=i8::MAX => {
                    clicks_cell.set(p - 1);
                    Some(Click::CCW)
                }
            }
        })
    }
}
