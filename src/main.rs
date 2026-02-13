//! Bespoke climate control system for formicarium
//!
//! Schematics perhaps forthcoming, but don't count on it
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use arduino_hal::{
    adc::AdcSettings,
    entry,
    hal::port::{PB4, PC6, PC7, PD6, PD7, PE6, PF0, PF1, PF4, PF5, PF6, PF7},
    pac::{ADC, TC1},
    pins,
    port::{
        mode::{Floating, Input, Output},
        Pin,
    },
    Adc, Delay, Peripherals, Pins,
};
use panic_halt as _;

use ag_lcd::{Blink, Cursor, Display, LcdDisplay, Lines};

pub mod millis;
pub mod ntc;
pub mod pwm;
pub mod relay;
pub mod rtc;

use crate::{
    millis::{init_millis, millis},
    ntc::Thermistor,
    pwm::PWMController,
    relay::Relay,
};

const PWM_HZ: u16 = 25_000;

const SAMPLE_INTERVAL: u32 = 1;
const UPDATE_INTERVAL: u32 = 10;
const DISPLAY_INTERVAL: u32 = 100;
const BLINK_INTERVAL: u32 = 1000;

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
                100_000.0,
                3_950.0,
                10_0500.0,
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

/// A page being displayed on the LCD
pub enum Page {
    /// No display
    None,

    /// Temperature Readouts
    Temps,
}

/// Formicarium climate control stystem state machine
#[must_use]
pub struct ClimateController {
    sensorium: Sensorium,

    next_sample: u32,
    next_update: u32,
    next_display: u32,
    next_blink: u32,

    target_temp: Option<f32>,

    compressor: Relay<PC6>,
    heater: Relay<PD7>,
    _relay2: Relay<PE6>,
    master_120vac: Relay<PB4>,

    pwm: PWMController,

    blink: Pin<Output, PC7>,
    _aux1: Pin<Input<Floating>, PD6>,
    _aux2: Pin<Input<Floating>, PF7>,
    _aux3: Pin<Input<Floating>, PF6>,

    display: LcdDisplay<Pin<Output>, Delay>,

    last_page: Page,
}

impl ClimateController {
    /// Construct and initialize state machine and interface with hardware
    pub fn new(pins: Pins, adc: ADC, tc1: TC1) -> Self {
        let rs = pins.mosi.into_output().downgrade();
        let rw = pins.d2.into_output().downgrade();
        let en = pins.led_rx.into_output().downgrade();
        let d4 = pins.d3.into_output().downgrade();
        let d5 = pins.sck.into_output().downgrade();
        let d6 = pins.d4.into_output().downgrade();
        let d7 = pins.miso.into_output().downgrade();

        let delay = arduino_hal::Delay::new();

        Self {
            sensorium: Sensorium::new(adc, pins.a2, pins.a3, pins.a4, pins.a5),

            next_sample: 0,
            next_update: 0,
            next_display: 0,
            next_blink: 0,

            target_temp: None,

            compressor: Relay::new(pins.d5),
            heater: Relay::new(pins.d6),
            _relay2: Relay::new(pins.d7),
            master_120vac: Relay::new(pins.d8),

            pwm: PWMController::new(tc1, pins.d9, pins.d10, pins.d11, PWM_HZ),

            blink: pins.d13.into_output(),
            _aux1: pins.d12,
            _aux2: pins.a0,
            _aux3: pins.a1,

            display: LcdDisplay::new(rs, en, delay)
                // .with_full_bus(d0, d1, d2, d3, d4, d5, d6, d7)
                .with_half_bus(d4, d5, d6, d7)
                .with_display(Display::On)
                .with_blink(Blink::Off)
                .with_cursor(Cursor::Off)
                .with_cols(20)
                .with_lines(Lines::FourLines)
                .with_rw(rw) // optional (set to GND if not provided)
                .with_reliable_init(20000)
                .build(),

            last_page: Page::None,
        }
    }

    /// Set the system's target temperature and begin the state machine
    pub fn begin(&mut self, target: f32) {
        if self.target_temp.is_some() {
            return;
        }

        self.target_temp = Some(target);

        self.master_120vac.activate();
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

        self.print_temps();
    }

    fn try_blink(&mut self, now: u32) {
        if now < self.next_blink {
            return;
        }
        self.next_blink += BLINK_INTERVAL;

        self.blink.toggle();
    }

    fn print_temps(&mut self) {
        let first_print = !matches!(self.last_page, Page::Temps);

        self.print_temp(
            0,
            self.sensorium.habitat_temp.fahrenheit(),
            "Habitat",
            first_print,
        );
        self.print_temp(
            1,
            self.sensorium.coolant_temp.fahrenheit(),
            "Coolant",
            first_print,
        );
        self.print_temp(
            2,
            self.sensorium.condenser_temp.fahrenheit(),
            "Condenser",
            first_print,
        );
        self.print_temp(
            3,
            self.sensorium.evaporator_temp.fahrenheit(),
            "Evaporator",
            first_print,
        );

        self.last_page = Page::Temps;
    }

    fn print_temp(&mut self, row: u8, temp: f32, title: &str, first: bool) {
        if first {
            self.display.set_position(0, row);
            self.display.print(title);
            self.display.print(": ");
        } else {
            self.display.set_position(title.len() as u8 + 2, row);
        }

        let hundreds = (libm::floorf(temp / 100.0) as u8).rem_euclid(10);
        if hundreds > 0 {
            self.display.print(digit(hundreds));
        }

        let tens = (libm::floorf(temp / 10.0) as u8).rem_euclid(10);
        if tens > 0 || hundreds > 0 {
            self.display.print(digit(tens));
        }

        let ones = (temp as u8).rem_euclid(10);
        if ones > 0 || tens > 0 || hundreds > 0 {
            self.display.print(digit(ones));
        }

        self.display.print(".");

        let tenths = ((temp - libm::floorf(temp)) * 10.0) as u8;
        self.display.print(digit(tenths));

        let hundredths = ((temp * 10.0 - libm::floorf(temp * 10.0)) * 10.0) as u8;
        self.display.print(digit(hundredths));

        self.display.print("F ");
    }
}

#[entry]
fn main() -> ! {
    let periphs = Peripherals::take().unwrap();
    let pins = pins!(periphs);

    // Disable USB controller to prevent the production of spurious interrupts
    periphs.USB_DEVICE.usbcon().reset();

    init_millis(&periphs.TC0);

    let mut controller = ClimateController::new(pins, periphs.ADC, periphs.TC1);

    // Safety: not called inside avr_device::interrupt::free
    unsafe { avr_device::interrupt::enable() };

    controller.begin(TARGET_TEMP);

    loop {
        let now = millis();

        controller.try_sample(now);
        controller.try_update(now);
        controller.try_display(now);
        controller.try_blink(now);
    }
}

/// Clamp `value` to the range `[min,max]` then map that to `[0.0,1.0]`
#[must_use]
pub fn normalize(value: f32, min: f32, max: f32) -> f32 {
    (value.clamp(min, max) - min) / (max - min)
}

const fn digit(digit: u8) -> &'static str {
    match digit {
        0 => "0",
        1 => "1",
        2 => "2",
        3 => "3",
        4 => "4",
        5 => "5",
        6 => "6",
        7 => "7",
        8 => "8",
        9 => "9",
        _ => unreachable!(),
    }
}

/* Ok so I know this is a bit ambitious, but now that I've got my cooling system pretty much finalized hardware-wise, I'd like to make it as "smart" as possible on the software side. Not like IoT type stuff, but rather continuously monitored self-tuning to maximize power efficiency and temperature stability. It's currently running pretty simply, just switching the compressor on and off to keep the water loop chilled, and switching the enclosure radiator fan on and off to regulate the enclosure temp. I would use PWM for that fan but for some reason it stalls out below ~70% duty. I have four temperature readouts: one in the enclosure, one in the water loop, one on the evaporator, and one on the condenser. The condenser fan, water circulation pump */
