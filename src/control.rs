//! Abstractions for PWM-controlled devices

use arduino_hal::{
    clock::Clock,
    hal::port::{PB5, PB6, PB7},
    pac::TC1,
    port::{
        mode::{Floating, Input, Output},
        Pin, PinOps,
    },
    DefaultClock,
};

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

    duty_a: u16,
    duty_b: u16,
    duty_c: u16,
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

            duty_a: 0,
            duty_b: 0,
            duty_c: 0,
        }
    }

    /// Change PWM frequency and reset timer to minimize interruptions
    pub fn set_hz(&mut self, hz: u16) {
        self.hz = hz;
        self.top = (DefaultClock::FREQ / (self.hz as u32 * 2)) as u16;

        self.tc1.icr1().write(|w| w.set(self.top));

        let da = ((self.top as u32 * self.duty_a as u32) >> 8) as u16;
        let db = ((self.top as u32 * self.duty_b as u32) >> 8) as u16;
        let dc = ((self.top as u32 * self.duty_c as u32) >> 8) as u16;

        self.tc1.ocr1a().write(|w| w.set(da));
        self.tc1.ocr1b().write(|w| w.set(db));
        self.tc1.ocr1c().write(|w| w.set(dc));

        self.tc1.tcnt1().reset();
    }

    /// Set PWM duty of channel A in the range `0..=256`
    ///
    /// Values exceeding `256` will be clamped
    pub fn set_duty_a(&mut self, duty: u16) {
        let duty = if duty > 256 { 256 } else { duty };

        let d = ((self.top as u32 * duty as u32) >> 8) as u16;
        self.tc1.ocr1a().write(|w| w.set(d));

        self.duty_a = duty;
    }

    /// Set PWM duty of channel B in the range `0..=256`
    ///
    /// Values exceeding `256` will be clamped
    pub fn set_duty_b(&mut self, duty: u16) {
        let duty = if duty > 256 { 256 } else { duty };

        let d = ((self.top as u32 * duty as u32) >> 8) as u16;
        self.tc1.ocr1b().write(|w| w.set(d));

        self.duty_b = duty;
    }

    /// Set PWM duty of channel C in the range `0..=256`
    ///
    /// Values exceeding `256` will be clamped
    pub fn set_duty_c(&mut self, duty: u16) {
        let duty = if duty > 256 { 256 } else { duty };

        let d = ((self.top as u32 * duty as u32) >> 8) as u16;
        self.tc1.ocr1c().write(|w| w.set(d));

        self.duty_c = duty;
    }

    /// Gets PWM duty of channel A in the range `0..=256`
    #[must_use]
    pub const fn duty_a(&self) -> u16 {
        self.duty_a
    }

    /// Gets PWM duty of channel B in the range `0..=256`
    #[must_use]
    pub const fn duty_b(&self) -> u16 {
        self.duty_b
    }

    /// Gets PWM duty of channel C in the range `0..=256`
    #[must_use]
    pub const fn duty_c(&self) -> u16 {
        self.duty_c
    }
}

enum RelayState {
    VerifiedOff,
    VerifiedOn,
    TurnedOff(u32),
    TurnedOn(u32),
    BlipOff(u32),
    BlipOn(u32),
}

/// Relay state machine with error detection/correction
pub struct Relay<PIN> {
    pin: Pin<Output, PIN>,
    state: RelayState,
    verify_off_delay: u16,
    verify_on_delay: u16,
    blip_delay: u16,
}

impl<PIN> Relay<PIN>
where
    PIN: PinOps,
{
    /// Bind relay to pin and initialize in off position
    pub const fn new(
        pin: Pin<Output, PIN>,
        verify_off_delay: u16,
        verify_on_delay: u16,
        blip_delay: u16,
    ) -> Self {
        Self {
            pin,
            state: RelayState::VerifiedOff,
            verify_off_delay,
            verify_on_delay,
            blip_delay,
        }
    }

    /// Checks if the relay is on, regardless of whether its state has been verified
    pub const fn is_on(&self) -> bool {
        matches!(self.state, RelayState::TurnedOn(_) | RelayState::VerifiedOn)
    }

    /// Checks if the relay is off, regardless of whether its state has been verified
    pub const fn is_off(&self) -> bool {
        matches!(
            self.state,
            RelayState::TurnedOff(_) | RelayState::VerifiedOff
        )
    }

    /// Attempts to switch the relay on only if it in the verified off state, returning whether that
    /// succeeded
    pub fn turn_on(&mut self, now: u32) -> bool {
        if matches!(self.state, RelayState::VerifiedOff) {
            self.pin.set_high();
            self.state = RelayState::TurnedOn(now);
            true
        } else {
            false
        }
    }

    /// Attempts to switch the relay off only if it in the verified on state, returning whether that
    /// succeeded
    pub fn turn_off(&mut self, now: u32) -> bool {
        if matches!(self.state, RelayState::VerifiedOn) {
            self.pin.set_low();
            self.state = RelayState::TurnedOff(now);
            true
        } else {
            false
        }
    }

    /// Runs either of the given callbacks to verify the state of the relay after its preset
    /// verification delay. If the callback fails, the relay is temporarily toggled to the opposite
    /// state
    pub fn verify_when_ready<F, G>(&mut self, now: u32, verify_on: F, verify_off: G)
    where
        F: FnOnce() -> bool,
        G: FnOnce() -> bool,
    {
        if let RelayState::TurnedOff(when) = self.state {
            if now - when >= self.verify_off_delay as u32 * 1000 {
                self.state = if verify_off() {
                    RelayState::VerifiedOff
                } else {
                    self.pin.set_high();
                    RelayState::BlipOn(now)
                }
            }
        } else if let RelayState::TurnedOn(when) = self.state {
            if now - when >= self.verify_on_delay as u32 * 1000 {
                self.state = if verify_on() {
                    RelayState::VerifiedOn
                } else {
                    self.pin.set_low();
                    RelayState::BlipOff(now)
                }
            }
        }
    }

    /// Restores the state of the relay after its preset blip delay
    pub fn restore_when_ready(&mut self, now: u32) {
        if let RelayState::BlipOff(when) = self.state {
            if now - when >= self.blip_delay as u32 * 1000 {
                self.pin.set_high();
                self.state = RelayState::TurnedOn(now);
            }
        } else if let RelayState::BlipOn(when) = self.state {
            if now - when >= self.blip_delay as u32 * 1000 {
                self.pin.set_low();
                self.state = RelayState::TurnedOff(now);
            }
        }
    }

    /// Forces the relay into the verified on state
    ///
    /// Note: should only be used for manual relay control
    pub fn force_on(&mut self) {
        self.state = RelayState::VerifiedOn;
        self.pin.set_high();
    }

    /// Forces the relay into the verified off state
    ///
    /// Note: should only be used for manual relay control
    pub fn force_off(&mut self) {
        self.state = RelayState::VerifiedOff;
        self.pin.set_low();
    }
}
