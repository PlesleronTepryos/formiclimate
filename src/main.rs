//! Bespoke climate control system for formicarium
//!
//! Schematics perhaps forthcoming, but don't count on it
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(variant_count)]

use arduino_hal::{
    adc::AdcSettings,
    entry,
    hal::port::{PB4, PC6, PC7, PD7, PE6, PF0, PF1, PF4, PF5, PF6, PF7},
    pac::{ADC, TC1, TWI},
    pins,
    port::{
        mode::{Floating, Input},
        Pin,
    },
    Adc, I2c, Peripherals, Pins,
};
use panic_halt as _;

pub mod display;
pub mod millis;
pub mod ntc;
pub mod pwm;
pub mod relay;
pub mod rtc;

use crate::{
    display::{Display, Page},
    millis::{init_millis, millis},
    ntc::Thermistor,
    pwm::PWMController,
    relay::Relay,
    rtc::DS1307,
};

const PWM_HZ: u16 = 25_000;

const SAMPLE_INTERVAL: u32 = 1;
const UPDATE_INTERVAL: u32 = 10;
const DISPLAY_INTERVAL: u32 = 100;
const PAGE_SWAP_INTERVAL: u32 = 10000;

const GRACE_PERIOD: u32 = 2000;

const TARGET_TEMP: f32 = 57.5;

/// The control system's complete sensory apparatus
#[must_use]
pub struct Sensorium {
    adc: Adc,

    coolant_temp: Thermistor<PF5>,
    habitat_temp: Thermistor<PF4>,
    condenser_temp: Thermistor<PF1>,
    evaporator_temp: Thermistor<PF0>,

    sens: f32,
    sens_steps: u8,
}

impl Sensorium {
    /// Construct sensorium
    pub fn new(
        adc: ADC,
        a2: Pin<Input<Floating>, PF5>,
        a3: Pin<Input<Floating>, PF4>,
        a4: Pin<Input<Floating>, PF1>,
        a5: Pin<Input<Floating>, PF0>,
    ) -> Self {
        let mut adc = Adc::new(adc, AdcSettings::default());

        Self {
            coolant_temp: Thermistor::new(
                a2.into_analog_input(&mut adc),
                10_000.0,
                3_380.0,
                9_820.0,
            ),
            habitat_temp: Thermistor::new(
                a3.into_analog_input(&mut adc),
                20_000.0,
                3_950.0,
                21_440.0,
            ),
            condenser_temp: Thermistor::new(
                a4.into_analog_input(&mut adc),
                50_000.0,
                3_950.0,
                46_200.0,
            ),
            evaporator_temp: Thermistor::new(
                a5.into_analog_input(&mut adc),
                10_000.0,
                3_380.0,
                9_860.0,
            ),

            adc,

            sens: 1.0,
            sens_steps: 10,
        }
    }

    /// Take a measurement sample on all sensors
    pub fn sample(&mut self) {
        self.coolant_temp.sample(&mut self.adc, self.sens);
        self.habitat_temp.sample(&mut self.adc, self.sens);
        self.condenser_temp.sample(&mut self.adc, self.sens);
        self.evaporator_temp.sample(&mut self.adc, self.sens);

        if self.sens_steps > 0 {
            self.sens *= 0.5;
            self.sens_steps -= 1;
        }
    }
}

/// Formicarium climate control stystem state machine
#[must_use]
pub struct ClimateController {
    sensorium: Sensorium,

    next_sample: u32,
    next_update: u32,
    next_display: u32,

    target_temp: Option<f32>,

    compressor: Relay<PC6>,
    heater: Relay<PD7>,
    _relay2: Relay<PE6>,
    master_120vac: Relay<PB4>,

    pwm: PWMController,

    rtc: DS1307,
    sqw: Pin<Input<Floating>, PC7>,

    _aux2: Pin<Input<Floating>, PF7>,
    _aux3: Pin<Input<Floating>, PF6>,

    display: Display,
}

impl ClimateController {
    /// Construct and initialize state machine and interface with hardware
    pub fn new(pins: Pins, adc: ADC, tc1: TC1, twi: TWI) -> Self {
        Self {
            sensorium: Sensorium::new(adc, pins.a2, pins.a3, pins.a4, pins.a5),

            next_sample: 0,
            next_update: 0,
            next_display: 0,

            target_temp: None,

            compressor: Relay::new(pins.d5),
            heater: Relay::new(pins.d6),
            _relay2: Relay::new(pins.d7),
            master_120vac: Relay::new(pins.d8),

            pwm: PWMController::new(tc1, pins.d9, pins.d10, pins.d11, PWM_HZ),

            rtc: DS1307::new(I2c::new(
                twi,
                pins.d2.into_pull_up_input(),
                pins.d3.into_pull_up_input(),
                50_000,
            )),
            // Note: d13 is connected via a 1k resistor to d8, which, when d8 is set high, acts as a
            // pull-up for the ds1307's open-drain oscillator output
            sqw: pins.d13,

            _aux2: pins.a0,
            _aux3: pins.a1,

            display: Display::new(
                pins.d4,
                pins.d12,
                pins.led_rx,
                pins.sck,
                pins.mosi,
                pins.miso,
            ),
        }
    }

    /// Set the system's target temperature and begin the state machine
    pub fn begin(&mut self, target: f32) {
        if self.target_temp.is_some() {
            return;
        }

        self.target_temp = Some(target);

        self.master_120vac.activate();

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
            let habitat_temp = self.sensorium.habitat_temp.fahrenheit();
            let coolant_temp = self.sensorium.coolant_temp.fahrenheit();
            let condenser_temp = self.sensorium.condenser_temp.fahrenheit();
            let evaporator_temp = self.sensorium.evaporator_temp.fahrenheit();

            let ht_delta = habitat_temp - target;
            let ce_delta = coolant_temp - evaporator_temp;

            if self.heater.is_active() {
                if habitat_temp > target {
                    self.heater.deactivate();
                }
            } else if habitat_temp < target - 1.0 {
                self.heater.activate();
            }

            if self.compressor.is_active() {
                if coolant_temp < target - 20.0 {
                    self.compressor.deactivate();
                }
            } else if coolant_temp > target - 5.0 {
                self.compressor.activate();
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
                self.sensorium.habitat_temp.fahrenheit(),
                self.sensorium.coolant_temp.fahrenheit(),
                self.sensorium.condenser_temp.fahrenheit(),
                self.sensorium.evaporator_temp.fahrenheit(),
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
    let pins = pins!(periphs);

    // Disable USB controller to prevent the production of spurious interrupts
    periphs.USB_DEVICE.usbcon().reset();

    init_millis(&periphs.TC0);

    let mut controller = ClimateController::new(pins, periphs.ADC, periphs.TC1, periphs.TWI);

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
