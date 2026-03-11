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
    pac::TC0,
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
    display::{Display, PageBuilder},
    millis::{init_millis, millis},
    pwm::PWMController,
    rtc::{Month, RTCTime, DS1307},
    sens::Sensorium,
};

const PWM_HZ: u16 = 25_000;

const SAMPLE_INTERVAL: u32 = 1;
const UPDATE_INTERVAL: u32 = 10;
const DISPLAY_INTERVAL: u32 = 100;
const CONFIG_INTERVAL: u32 = 1000;
const PAGE_SWAP_INTERVAL: u32 = 10000;

const CALIBRATION_PERIOD: u32 = 2000;

const DEFAULT_CONFIG: ControllerConfig = ControllerConfig {
    day_temp: 75.0,
    night_temp: 70.0,
    diapause_temp: 57.5,

    diapause_start: Month::November.nth(1, false),
    diapause_length: 125,
    diapause_ramp_days: 14,

    min_effective_coolant_delta: 8.0,
};

macro_rules! extract {
    ($data:ident, $offset:ident, bool) => {{
        let b0 = $data[$offset];
        $offset += 1;
        b0 != 0
    }};
    ($data:ident, $offset:ident, u8) => {{
        let b0 = $data[$offset];
        $offset += 1;
        b0
    }};
    ($data:ident, $offset:ident, u16) => {{
        let [b0, b1] = [$data[$offset], $data[$offset + 1]];
        $offset += 2;
        u16::from_le_bytes([b0, b1])
    }};
    ($data:ident, $offset:ident, u32) => {{
        let [b0, b1, b2, b3] = [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ];
        $offset += 4;
        u32::from_le_bytes([b0, b1, b2, b3])
    }};
    ($data:ident, $offset:ident, i8) => {{
        let b0 = $data[$offset];
        $offset += 1;
        b0 as i8
    }};
    ($data:ident, $offset:ident, i16) => {{
        let [b0, b1] = [$data[$offset], $data[$offset + 1]];
        $offset += 2;
        i16::from_le_bytes([b0, b1])
    }};
    ($data:ident, $offset:ident, i32) => {{
        let [b0, b1, b2, b3] = [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ];
        $offset += 4;
        i32::from_le_bytes([b0, b1, b2, b3])
    }};
    ($data:ident, $offset:ident, f32) => {{
        let [b0, b1, b2, b3] = [
            $data[$offset],
            $data[$offset + 1],
            $data[$offset + 2],
            $data[$offset + 3],
        ];
        $offset += 4;
        f32::from_le_bytes([b0, b1, b2, b3])
    }};
}

macro_rules! inject {
    ($name:ident, $data:ident, $offset:ident, bool) => {
        $data[$offset] = $name as u8;
        $offset += 1;
    };
    ($name:ident, $data:ident, $offset:ident, u8) => {
        $data[$offset] = $name;
        $offset += 1;
    };
    ($name:ident, $data:ident, $offset:ident, u16) => {
        let [b0, b1] = $name.to_le_bytes();
        $data[$offset] = b0;
        $data[$offset + 1] = b1;
        $offset += 2;
    };
    ($name:ident, $data:ident, $offset:ident, u32) => {
        let [b0, b1, b2, b3] = $name.to_le_bytes();
        $data[$offset] = b0;
        $data[$offset + 1] = b1;
        $data[$offset + 2] = b2;
        $data[$offset + 3] = b3;
        $offset += 4;
    };
    ($name:ident, $data:ident, $offset:ident, i8) => {
        $data[$offset] = $name;
        $offset += 1;
    };
    ($name:ident, $data:ident, $offset:ident, i16) => {
        let [b0, b1] = $name.to_le_bytes();
        $data[$offset] = b0;
        $data[$offset + 1] = b1;
        $offset += 2;
    };
    ($name:ident, $data:ident, $offset:ident, i32) => {
        let [b0, b1, b2, b3] = $name.to_le_bytes();
        $data[$offset] = b0;
        $data[$offset + 1] = b1;
        $data[$offset + 2] = b2;
        $data[$offset + 3] = b3;
        $offset += 4;
    };
    ($name:ident, $data:ident, $offset:ident, f32) => {
        let [b0, b1, b2, b3] = $name.to_le_bytes();
        $data[$offset] = b0;
        $data[$offset + 1] = b1;
        $data[$offset + 2] = b2;
        $data[$offset + 3] = b3;
        $offset += 4;
    };
}

macro_rules! portable {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $($field_vis:vis $field_name:ident : $field_type:tt),* $(,)?
        }
    ) => {
        $(#[$meta])*
        $vis struct $name {
            $($field_vis $field_name : $field_type,)*
        }

        impl $name {
            const SIGNATURE: u32 = {
                let bytes = concat!($(stringify!($field_type)),*).as_bytes();
                let mut sig: u32 = 0;
                let mut i = 0;
                while i < bytes.len() {
                    let byte = bytes[i];
                    let un = (byte >> 4) as u32;
                    let ln = (byte & 0xf) as u32;
                    sig ^= (1 << (un + 16)) | (1 << ln);
                    sig = sig.rotate_left(1);
                    i += 1;
                }
                sig
            };

            #[expect(unused_assignments, reason = "macro expansion leaves trailing offset increment")]
            const fn from_data(data: [u8; 56]) -> Result<Self, [u8; 56]> {
                let [struct_data @ .., s0, s1, s2, s3] = data;
                let data_sig = u32::from_le_bytes([s0, s1, s2, s3]);
                if data_sig == Self::SIGNATURE {
                    let mut offset = 0;
                    Ok(Self{$($field_name: extract!(struct_data, offset, $field_type)),*})
                } else {
                    Err(data)
                }
            }

            #[expect(unused_assignments, reason = "macro expansion leaves trailing offset increment")]
            const fn into_data(self) -> [u8;56] {
                let Self {$($field_name),*} = self;
                let mut data = [0;56];
                let mut offset = 0;
                $(inject!($field_name, data, offset, $field_type);)*
                data
            }
        }
    };
}

portable!(
    /// Portable configuration for the [`ClimateController`]
    ///
    /// Ramping into diapause begins on [`diapause_start`](Self::diapause_start); ramping out begins
    /// [`diapause_length`](Self::diapause_length) days after [`diapause_start`](Self::diapause_start)
    #[derive(Clone, Copy)]
    #[repr(C)]
    pub struct ControllerConfig {
        day_temp: f32,
        night_temp: f32,
        diapause_temp: f32,

        diapause_start: u16,
        diapause_length: u16,
        diapause_ramp_days: u8,

        min_effective_coolant_delta: f32,
    }
);

impl ControllerConfig {
    /// Calculate the target temperature for the given time based on the current configuration
    #[must_use]
    pub const fn calculate_target(&self, time: RTCTime) -> f32 {
        let leap_day = time.year.is_leap();

        let day_of_year = time.month.offset(leap_day) + time.date.bin() as u16;

        let days_since_diapause_start = (day_of_year as i16 - self.diapause_start as i16)
            .rem_euclid(365 + leap_day as i16) as u16;

        if days_since_diapause_start >= self.diapause_ramp_days as u16
            && days_since_diapause_start < self.diapause_length
        {
            return self.diapause_temp;
        }

        let hour = time.hours.bin();
        let clock_hour = if hour > 12 { hour - 12 } else { hour };
        let secs_of_hour = time.seconds.bin() as u16 + time.minutes.bin() as u16 * 60;

        let diurnal_cycle_temp = if clock_hour >= 6 {
            let prog = (secs_of_hour + (clock_hour - 6) as u16 * 3600) as f32 / 21_600.0;
            let smooth = (3.0 - prog * 2.0) * prog * prog;

            let (a_temp, b_temp) = if hour < 12 {
                (self.night_temp, self.day_temp)
            } else {
                (self.day_temp, self.night_temp)
            };

            a_temp * (1.0 - smooth) + b_temp * smooth
        } else if hour < 12 {
            self.night_temp
        } else {
            self.day_temp
        };

        if days_since_diapause_start >= self.diapause_length + self.diapause_ramp_days as u16 {
            return diurnal_cycle_temp;
        }

        let descending = days_since_diapause_start < self.diapause_length;

        let prog = (secs_of_hour as u32
            + hour as u32 * 3600
            + (days_since_diapause_start - if descending { 0 } else { self.diapause_length })
                as u32
                * 86_400) as f32
            / (self.diapause_ramp_days as u32 * 86_400) as f32;

        let (a_temp, b_temp) = if descending {
            (diurnal_cycle_temp, self.diapause_temp)
        } else {
            (self.diapause_temp, diurnal_cycle_temp)
        };

        a_temp * (1.0 - prog) + b_temp * prog
    }

    const fn diapause_status(&self, time: RTCTime) -> &'static str {
        let leap_day = time.year.is_leap();

        let day_of_year = time.month.nth(time.date.bin(), leap_day);

        let days_since_diapause_start = (day_of_year as i16 - self.diapause_start as i16)
            .rem_euclid(365 + leap_day as i16) as u16;

        if days_since_diapause_start < self.diapause_ramp_days as u16 {
            "[Entering Diapause]"
        } else if days_since_diapause_start < self.diapause_length {
            "[In Diapause]"
        } else if days_since_diapause_start < self.diapause_length + self.diapause_ramp_days as u16
        {
            "[Exiting Diapause]"
        } else {
            let hour = time.hours.bin();

            if hour < 6 {
                "[Night]"
            } else if hour < 12 {
                "[Morning]"
            } else if hour < 18 {
                "[Day]"
            } else {
                "[Evening]"
            }
        }
    }
}

#[derive(Clone, Copy)]
enum Target {
    Unset,
    Static(f32),
    Dynamic(f32),
}

impl Target {
    const fn value(self) -> Option<f32> {
        match self {
            Self::Unset => None,
            Self::Static(v) | Self::Dynamic(v) => Some(v),
        }
    }
}

enum PageId {
    TimeAndTarget,
    TempReadings,
}

impl PageId {
    const fn next(&mut self) {
        *self = match *self {
            Self::TimeAndTarget => Self::TempReadings,
            Self::TempReadings => Self::TimeAndTarget,
        };
    }
}

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

    tc0: TC0,

    next_sample: u32,
    next_update: u32,
    next_display: u32,
    next_config: u32,
    next_page_swap: u32,

    target_temp: Target,

    config: ControllerConfig,

    page_id: PageId,
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
            // Note: pc7 is connected via a 1k resistor to pd7, which, when pd7 is set high, acts as
            // a pull-up for the ds1307's open-drain oscillator output
            _sqw: pins.pc7,

            _pb4: pins.pb4,
            _pc6: pins.pc6,
            _pe2: pins.pe2,
            _pe6: pins.pe6,
            _pf6: pins.pf6,
            _pf7: pins.pf7,

            display: Display::new(pins.pd2, pins.pd3, pins.pb0, pins.pb1, pins.pb2, pins.pb3),

            tc0: periphs.TC0,

            next_sample: 0,
            next_update: 0,
            next_display: 0,
            next_config: 0,
            next_page_swap: PAGE_SWAP_INTERVAL,

            target_temp: Target::Unset,

            config: DEFAULT_CONFIG,

            page_id: PageId::TimeAndTarget,
        }
    }

    /// Start the operation of the climate controller
    pub fn begin(&mut self) {
        arduino_hal::delay_ms(500);

        self.master_120vac.set_high();

        self.load_config();

        self.display.init();

        init_millis(&self.tc0);
    }

    fn load_config(&mut self) {
        if let Ok(data) = self.rtc.get_ram() {
            if let Ok(config) = ControllerConfig::from_data(data) {
                self.config = config;
            }
        }
    }

    fn save_config(&mut self) {
        let _ = self.rtc.set_ram(self.config.into_data());
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

    fn update(&mut self) {
        if let Some(target) = self.target_temp.value() {
            let habitat_temp = self.sensorium.habitat_temp().fahrenheit();
            let coolant_temp = self.sensorium.coolant_temp().fahrenheit();
            let condenser_temp = self.sensorium.condenser_temp().fahrenheit();
            let evaporator_temp = self.sensorium.evaporator_temp().fahrenheit();

            let ce_delta = coolant_temp - evaporator_temp;

            if self.heater.is_set_high() {
                if habitat_temp > target {
                    self.heater.set_low();
                }
            } else if habitat_temp < target - 0.25 {
                self.heater.set_high();
            }

            if self.compressor.is_set_high() {
                if coolant_temp < target - self.config.min_effective_coolant_delta - 10.0 {
                    self.compressor.set_low();
                }
            // If the habitat temp exceeds the target by too much, the coolant is too warm to
            // extract heat effectively
            } else if habitat_temp > target + 0.25 {
                self.compressor.set_high();
                self.config.min_effective_coolant_delta =
                    (target - coolant_temp).max(self.config.min_effective_coolant_delta + 0.5);
            } else if coolant_temp > target - self.config.min_effective_coolant_delta {
                self.compressor.set_high();
                self.config.min_effective_coolant_delta -= 0.1;
            }

            // Fail-safe: max out the condenser fan in case of condenser temp sensor failure to
            // avoid overheating the compressor
            if condenser_temp.is_nan() {
                self.set_condenser_fan_duty(1.0);
            } else {
                self.set_condenser_fan_duty(normalize(condenser_temp, 80.0, 90.0));
            }

            // FIXME: Habitat fan does not handle PWM well, needs all/nothing control
            if habitat_temp > target + 0.05 && coolant_temp < target {
                self.set_habitat_fan_duty(1.0);
            } else if habitat_temp < target {
                self.set_habitat_fan_duty(0.0);
            }

            if ce_delta > 5.0 {
                self.set_coolant_pump_duty(normalize(ce_delta, -5.0, 15.0));
            } else {
                self.set_coolant_pump_duty(0.0);
            }
        }
    }

    fn config(&mut self) {
        match self.target_temp {
            Target::Static(_) => {
                if let Ok(time) = self.rtc.get_time() {
                    self.load_config();
                    self.target_temp = Target::Dynamic(self.config.calculate_target(time));
                }
            }
            Target::Unset | Target::Dynamic(_) => {
                self.target_temp = if let Ok(time) = self.rtc.get_time() {
                    Target::Dynamic(self.config.calculate_target(time))
                } else {
                    // If the RTC does not respond, fail-safe by holding the current habitat temperature
                    Target::Static(self.sensorium.habitat_temp().fahrenheit())
                }
            }
        }

        self.save_config();
    }

    fn display(&mut self) {
        let page = match self.page_id {
            PageId::TimeAndTarget => {
                let mut page = PageBuilder::new();

                page = match self.rtc.get_time() {
                    Ok(time) => page
                        .write(b"Today is ")
                        .write(time.day.name().as_bytes())
                        .end_line()
                        .write(b"20")
                        .hexit2(time.year.bcd())
                        .byte(b'.')
                        .hexit2(time.month.bcd())
                        .byte(b'.')
                        .hexit2(time.date.bcd())
                        .write(b"  ")
                        .hexit2(time.hours.bcd_24h())
                        .byte(b':')
                        .hexit2(time.minutes.bcd())
                        .byte(b':')
                        .hexit2(time.seconds.bcd())
                        .end_line()
                        .write(self.config.diapause_status(time).as_bytes())
                        .end_line(),
                    Err(_) => page
                        .write(b"Can't get RTC time!")
                        .end_line()
                        .next_line()
                        .next_line(),
                };

                if let Some(target) = self.target_temp.value() {
                    page.write(b"Setpoint:   ").decimal(target).byte(b'F')
                } else {
                    page.write(b"Calibrating...").end_line()
                }
                .finish()
            }
            PageId::TempReadings => PageBuilder::new()
                .write(b"Habitat:    ")
                .decimal(self.sensorium.habitat_temp().fahrenheit())
                .byte(b'F')
                .write(b"Coolant:    ")
                .decimal(self.sensorium.coolant_temp().fahrenheit())
                .byte(b'F')
                .write(b"Condenser:  ")
                .decimal(self.sensorium.condenser_temp().fahrenheit())
                .byte(b'F')
                .write(b"Evaporator: ")
                .decimal(self.sensorium.evaporator_temp().fahrenheit())
                .byte(b'F')
                .finish(),
        };

        self.display.write_page(page);
    }

    fn periodic(&mut self) {
        let now = millis();

        if now >= self.next_sample {
            self.sensorium.sample();
            self.next_sample += SAMPLE_INTERVAL;
        }

        if now >= self.next_update {
            self.update();
            self.next_update += UPDATE_INTERVAL;
        }

        if now >= CALIBRATION_PERIOD && now >= self.next_config {
            self.config();
            self.next_config += CONFIG_INTERVAL;
        }

        if now >= self.next_display {
            self.display();
            self.next_display += DISPLAY_INTERVAL;
        }

        if now >= self.next_page_swap {
            self.page_id.next();
            self.next_page_swap += PAGE_SWAP_INTERVAL;
        }
    }
}

#[entry]
fn main() -> ! {
    let periphs = Peripherals::take().unwrap();
    let mut controller = ClimateController::new(periphs);

    // Safety: not called inside avr_device::interrupt::free
    unsafe { avr_device::interrupt::enable() };

    controller.begin();

    loop {
        controller.periodic();
    }
}

/// Clamp `value` to the range `[min,max]` then map that to `[0.0,1.0]`
#[must_use]
pub fn normalize(value: f32, min: f32, max: f32) -> f32 {
    (value.clamp(min, max) - min) / (max - min)
}
