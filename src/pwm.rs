//! Abstractions for PWM-controlled devices

use arduino_hal::{
    clock::Clock,
    hal::port::{PB5, PB6, PB7},
    pac::TC1,
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
    DefaultClock,
};

/// Initialize PWM clock at specified frequency
pub fn init_pwm(tc1: &TC1, hz: u16) {
    let top = (DefaultClock::FREQ / (hz as u32 * 2)) as u16;

    tc1.tccr1a().write(|w| {
        w.com1a().match_clear();
        w.com1b().match_clear();
        w.com1c().match_clear();
        w.wgm1().set(0b10)
    });

    tc1.tccr1b().write(|w| {
        w.wgm1().set(0b1);
        w.cs1().direct()
    });

    tc1.icr1().write(|w| w.set(top));

    tc1.ocr1a().write(|w| w.set(0));
    tc1.ocr1b().write(|w| w.set(0));
    tc1.ocr1c().write(|w| w.set(0));
}

/// 3-channel PWM controller built atop [TC1]
///
/// Output pins are:
/// - [PB5]: channel A
/// - [PB6]: channel B
/// - [PB7]: channel C
pub struct PWMController {
    tc1: TC1,
    _ch_a: Pin<Output, PB5>,
    _ch_b: Pin<Output, PB6>,
    _ch_c: Pin<Output, PB7>,

    hz: u16,
    top: u16,

    duty_a: f32,
    duty_b: f32,
    duty_c: f32,
}

impl PWMController {
    /// Create and initialize PWM controller, taking ownership of timer/pins to prevent conflicts
    #[must_use]
    pub fn new(
        tc1: TC1,
        d9: Pin<Input<Floating>, PB5>,
        d10: Pin<Input<Floating>, PB6>,
        d11: Pin<Input<Floating>, PB7>,
        hz: u16,
    ) -> Self {
        let top = (DefaultClock::FREQ / (hz as u32 * 2)) as u16;

        let ch_a = d9.into_output();
        let ch_b = d10.into_output();
        let ch_c = d11.into_output();

        tc1.tccr1a().write(|w| {
            w.com1a().match_clear();
            w.com1b().match_clear();
            w.com1c().match_clear();
            w.wgm1().set(0b10)
        });

        tc1.tccr1b().write(|w| {
            w.wgm1().set(0b10);
            w.cs1().direct()
        });

        tc1.icr1().write(|w| w.set(top));

        tc1.ocr1a().write(|w| w.set(0));
        tc1.ocr1b().write(|w| w.set(0));
        tc1.ocr1c().write(|w| w.set(0));

        Self {
            tc1,

            _ch_a: ch_a,
            _ch_b: ch_b,
            _ch_c: ch_c,

            hz,
            top,

            duty_a: 0.0,
            duty_b: 0.0,
            duty_c: 0.0,
        }
    }

    /// Change PWM frequency and reset timer to minimize interruptions
    pub fn set_hz(&mut self, hz: u16) {
        self.hz = hz;
        self.top = (DefaultClock::FREQ / (self.hz as u32 * 2)) as u16;

        self.tc1.icr1().write(|w| w.set(self.top));

        let top = self.top as f32;

        self.tc1
            .ocr1a()
            .write(|w| w.set((top * self.duty_a) as u16));
        self.tc1
            .ocr1b()
            .write(|w| w.set((top * self.duty_b) as u16));
        self.tc1
            .ocr1c()
            .write(|w| w.set((top * self.duty_c) as u16));

        self.tc1.tcnt1().reset();
    }

    /// Set duty percentage of channel A
    pub fn set_duty_a(&mut self, duty: f32) {
        let duty = duty.clamp(0.0, 1.0);
        self.tc1
            .ocr1a()
            .write(|w| w.set((self.top as f32 * duty) as u16));
        self.duty_a = duty;
    }

    /// Set duty percentage of channel B
    pub fn set_duty_b(&mut self, duty: f32) {
        let duty = duty.clamp(0.0, 1.0);
        self.tc1
            .ocr1b()
            .write(|w| w.set((self.top as f32 * duty) as u16));
        self.duty_b = duty;
    }

    /// Set duty percentage of channel C
    pub fn set_duty_c(&mut self, duty: f32) {
        let duty = duty.clamp(0.0, 1.0);
        self.tc1
            .ocr1c()
            .write(|w| w.set((self.top as f32 * duty) as u16));
        self.duty_c = duty;
    }
}
