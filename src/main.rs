//! Bespoke climate control system for formicarium
//!
//! Schematics perhaps forthcoming, but don't count on it
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(variant_count)]

use arduino_hal::{
    entry,
    hal::port::{PB4, PC6, PC7, PD4, PD5, PD6, PD7, PE2, PE6, PF6, PF7},
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
    I2c, Peripherals,
};
use panic_halt as _;

pub mod display;
pub mod millis;
pub mod pwm;
pub mod rtc;
pub mod sens;

use crate::{
    display::{Display, Page},
    millis::{init_millis, millis},
    pwm::PWMController,
    rtc::DS1307,
    sens::Sensorium,
};

const PWM_HZ: u16 = 25_000;

const SAMPLE_INTERVAL: u32 = 1;
const UPDATE_INTERVAL: u32 = 10;
const DISPLAY_INTERVAL: u32 = 100;
const PAGE_SWAP_INTERVAL: u32 = 10000;

const GRACE_PERIOD: u32 = 2000;

const TARGET_TEMP: f32 = 57.5;

/// Formicarium climate control stystem state machine
///
/// # Pin Configuration
///
/// `PORTB`:
/// - `PB0`: LCD D4
/// - `PB1`: LCD D5
/// - `PB2`: LCD D6
/// - `PB3`: LCD D7
/// - `PB4`: unused
/// - `PB5`: PWM channel A (condenser fan)
/// - `PB6`: PWM channel B (enclosure fan)
/// - `PB7`: PWM channel C (circulation pump)
///
/// `PORTC`:
/// - `PC6`: unused
/// - `PC7`: RTC square wave input
///
/// `PORTD`:
/// - `PD0`: I2C SCL
/// - `PD1`: I2C SDA
/// - `PD2`: LCD RS
/// - `PD3`: LCD enable
/// - `PD4`: relay 0 (compressor)
/// - `PD5`: relay 1 (heater)[^1]
/// - `PD6`: relay 2 (wired but unused)
/// - `PD7`: relay 3 (master 120V)
///
/// `PORTE`:
/// - `PE2`: unused[^2]
/// - `PE6`: unused
///
/// `PORTF`:
/// - `PF0`: thermistor (evaporator)
/// - `PF1`: thermistor (condenser)
/// - `PF4`: thermistor (formicarium)
/// - `PF5`: thermistor (coolant loop)
/// - `PF6`: unused
/// - `PF7`: unused
///
/// [^1]: board modified to break `PD5` out to the factory NC pin that would be A7\
/// [^2]: board modified to break `PE2` out to the factory NC pin that would be A6
#[must_use]
pub struct ClimateController {
    sensorium: Sensorium,

    compressor: Pin<Output, PD4>,
    heater: Pin<Output, PD5>,
    _relay2: Pin<Output, PD6>,
    master_120vac: Pin<Output, PD7>,

    pwm: PWMController,

    rtc: DS1307,
    _sqw: Pin<Input<Floating>, PC7>,

    _pb4: Pin<Input<Floating>, PB4>,
    _pc6: Pin<Input<Floating>, PC6>,
    _pe2: Pin<Input<Floating>, PE2>,
    _pe6: Pin<Input<Floating>, PE6>,
    _pf6: Pin<Input<Floating>, PF6>,
    _pf7: Pin<Input<Floating>, PF7>,

    display: Display,

    next_sample: u32,
    next_update: u32,
    next_display: u32,

    target_temp: Option<f32>,
}

impl ClimateController {
    /// Construct and initialize climate controller and interface with hardware
    pub fn new(periphs: Peripherals) -> Self {
        let pins = arduino_hal::hal::Pins::new(
            periphs.PORTB,
            periphs.PORTC,
            periphs.PORTD,
            periphs.PORTE,
            periphs.PORTF,
        );

        // Disable USB controller to prevent the production of spurious interrupts
        periphs.USB_DEVICE.usbcon().reset();

        init_millis(&periphs.TC0);

        Self {
            sensorium: Sensorium::new(periphs.ADC, pins.pf5, pins.pf4, pins.pf1, pins.pf0),

            compressor: pins.pd4.into_output(),
            heater: pins.pd5.into_output(),
            _relay2: pins.pd6.into_output(),
            master_120vac: pins.pd7.into_output(),

            pwm: PWMController::new(periphs.TC1, pins.pb5, pins.pb6, pins.pb7, PWM_HZ),

            rtc: DS1307::new(I2c::new(
                periphs.TWI,
                pins.pd1.into_pull_up_input(),
                pins.pd0.into_pull_up_input(),
                50_000,
            )),
            // Note: d13 is connected via a 1k resistor to d8, which, when d8 is set high, acts as a
            // pull-up for the ds1307's open-drain oscillator output
            _sqw: pins.pc7,

            _pb4: pins.pb4,
            _pc6: pins.pc6,
            _pe2: pins.pe2,
            _pe6: pins.pe6,
            _pf6: pins.pf6,
            _pf7: pins.pf7,

            display: Display::new(pins.pd2, pins.pd3, pins.pb0, pins.pb1, pins.pb2, pins.pb3),

            next_sample: 0,
            next_update: 0,
            next_display: 0,

            target_temp: None,
        }
    }

    /// Set the system's target temperature and begin the state machine
    pub fn begin(&mut self, target: f32) {
        if self.target_temp.is_some() {
            return;
        }

        self.target_temp = Some(target);

        self.master_120vac.set_high();

        arduino_hal::delay_ms(500);

        self.display.init();
    }

    fn set_condenser_fan_duty(&mut self, duty: f32) {
        self.pwm.set_duty_a(duty);
    }

    fn set_habitat_fan_duty(&mut self, duty: f32) {
        self.pwm.set_duty_b(duty);
    }

    fn set_coolant_pump_duty(&mut self, duty: f32) {
        self.pwm.set_duty_c(duty);
    }

    fn try_sample(&mut self, now: u32) {
        if now < self.next_sample {
            return;
        }
        self.next_sample += SAMPLE_INTERVAL;

        self.sensorium.sample();
    }

    fn try_update(&mut self, now: u32) {
        if now < self.next_update {
            return;
        }
        self.next_update += UPDATE_INTERVAL;

        if now < GRACE_PERIOD {
            return;
        }

        if let Some(target) = self.target_temp {
            let habitat_temp = self.sensorium.habitat_temp().fahrenheit();
            let coolant_temp = self.sensorium.coolant_temp().fahrenheit();
            let condenser_temp = self.sensorium.condenser_temp().fahrenheit();
            let evaporator_temp = self.sensorium.evaporator_temp().fahrenheit();

            let ht_delta = habitat_temp - target;
            let ce_delta = coolant_temp - evaporator_temp;

            if self.heater.is_set_high() {
                if habitat_temp > target {
                    self.heater.set_low();
                }
            } else if habitat_temp < target - 1.0 {
                self.heater.set_high();
            }

            if self.compressor.is_set_high() {
                if coolant_temp < target - 20.0 {
                    self.compressor.set_low();
                }
            } else if coolant_temp > target - 5.0 {
                self.compressor.set_high();
            }

            if condenser_temp > 80.0 {
                self.set_condenser_fan_duty(normalize(condenser_temp, 75.0, 100.0));
            } else {
                self.set_condenser_fan_duty(0.0);
            }

            // FIXME: Habitat fan does not handle PWM well, needs all/nothing control
            if ht_delta > 0.05 && coolant_temp < target {
                self.set_habitat_fan_duty(1.0);
            } else if ht_delta < -0.05 {
                self.set_habitat_fan_duty(0.0);
            }

            if ce_delta > 5.0 {
                self.set_coolant_pump_duty(normalize(ce_delta, -5.0, 15.0));
            } else {
                self.set_coolant_pump_duty(0.0);
            }
        }
    }

    fn try_display(&mut self, now: u32) {
        if now < self.next_display {
            return;
        }
        self.next_display += DISPLAY_INTERVAL;

        let page = match (now / PAGE_SWAP_INTERVAL).rem_euclid(Page::COUNT as u32) {
            0 => Page::Temps(
                self.sensorium.habitat_temp().fahrenheit(),
                self.sensorium.coolant_temp().fahrenheit(),
                self.sensorium.condenser_temp().fahrenheit(),
                self.sensorium.evaporator_temp().fahrenheit(),
            ),
            1 => Page::Time(self.rtc.get_time().map_err(|_| "Can't get RTC time!")),
            _ => unreachable!(),
        };

        self.display.set_page(page);
        self.display.refresh();
    }
}

#[entry]
fn main() -> ! {
    let periphs = Peripherals::take().unwrap();
    let mut controller = ClimateController::new(periphs);

    // Safety: not called inside avr_device::interrupt::free
    unsafe { avr_device::interrupt::enable() };

    controller.begin(TARGET_TEMP);

    loop {
        let now = millis();

        controller.try_sample(now);
        controller.try_update(now);
        controller.try_display(now);
    }
}

/// Clamp `value` to the range `[min,max]` then map that to `[0.0,1.0]`
#[must_use]
pub fn normalize(value: f32, min: f32, max: f32) -> f32 {
    (value.clamp(min, max) - min) / (max - min)
}
