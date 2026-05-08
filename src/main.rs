//! Bespoke climate control system for formicarium
//!
//! Schematics perhaps forthcoming, but don't count on it
#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![feature(macro_metavar_expr)]

use arduino_hal::{
    entry,
    hal::port::{PC6, PC7, PD4, PD5, PD6, PD7, PF6, PF7},
    pac::TC0,
    port::{
        mode::{Floating, Input},
        Pin,
    },
    I2c, Peripherals,
};
use panic_halt as _;

mod codegen;
pub mod control;
pub mod display;
pub mod encoder;
pub mod millis;
pub mod rtc;
pub mod sens;
pub mod utils;

use crate::{
    control::{PWMController, Relay},
    display::{Display, PageBuilder, PageData},
    encoder::{Click, Encoder},
    millis::{init_millis, millis},
    rtc::{Date, Month, RTCTime, DS1307},
    sens::Sensorium,
    utils::{i16_to_f32, is_finite, recip, u16_to_f32},
};

const PWM_HZ: u16 = 31_250;

const SAMPLE_INTERVAL: u32 = 1;
const UPDATE_INTERVAL: u32 = 10;
const DISPLAY_INTERVAL: u32 = 100;
const CONFIG_INTERVAL: u32 = 1000;

const CALIBRATION_PERIOD: u32 = 2000;

crate::codegen::portable!(
    /// Portable configuration for the [`ClimateController`]
    ///
    /// The diapause season runs from `diapause_start_month`/`diapause_start_day` to
    /// `diapause_end_month`/`diapause_end_day`, with `diapause_ramp_days` days of linear
    /// interpolation at each end
    #[derive(Clone)]
    pub struct ControllerConfig {
        day_temp as DayTemp: f32 = 75.0,
        night_temp as NightTemp: f32 = 70.0,
        diapause_temp as DiapauseTemp: f32 = 57.5,

        diapause_start_month as DiapauseStartMonth: Month = Month::November,
        diapause_start_day as DiapauseStartDay: Date = Date::from_bin(1),
        diapause_end_month as DiapauseEndMonth: Month = Month::March,
        diapause_end_day as DiapauseEndDay: Date = Date::from_bin(20),
        diapause_ramp_days as DiapauseRampDays: u8 = 14,

        min_effective_subcooling as MinSubcooling: f32 = 8.0,
    }
    exit = b"";
    info = b"  Press To Confirm  ";
    ConfigBuffer
);

impl ControllerConfig {
    const fn calc_diapause_window(&self, time: RTCTime) -> (u16, u16, u16) {
        let leap_day = time.year.is_leap();

        let day_of_year = time.month.nth(time.date.bin(), leap_day);

        let start_doy = self
            .diapause_start_month
            .nth(self.diapause_start_day.bin(), leap_day);
        let end_doy = self
            .diapause_end_month
            .nth(self.diapause_end_day.bin(), leap_day);
        let year_len = 365 + leap_day as u16;

        let diapause_duration =
            end_doy + if end_doy < start_doy { year_len } else { 0 } - start_doy;
        let days_since_start =
            day_of_year + if day_of_year < start_doy { year_len } else { 0 } - start_doy;
        let ramp = if diapause_duration < (self.diapause_ramp_days * 2) as u16 {
            diapause_duration / 2
        } else {
            self.diapause_ramp_days as u16
        };

        (diapause_duration, days_since_start, ramp)
    }

    /// Calculate the target temperature for the given time based on the current configuration
    #[must_use]
    pub const fn calculate_target(&self, time: RTCTime) -> f32 {
        const INV_24: f32 = 1.0 / 24.0;
        const INV_21600: f32 = 1.0 / 21_600.0;
        const INV_86400: f32 = 1.0 / 86_400.0;

        let (diapause_duration, days_since_start, ramp) = self.calc_diapause_window(time);

        if days_since_start >= ramp && days_since_start < diapause_duration.saturating_sub(ramp) {
            return self.diapause_temp;
        }

        let hour = time.hours.bin();
        let clock_hour = if hour < 12 { hour } else { hour - 12 };
        let secs_of_hour = time.seconds.bin() as u16 + time.minutes.bin() as u16 * 60;

        let diurnal_cycle_temp = if clock_hour >= 6 {
            let prog =
                i16_to_f32((secs_of_hour + (clock_hour - 6) as u16 * 3600) as i16) * INV_21600;
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

        if days_since_start >= diapause_duration {
            return diurnal_cycle_temp;
        }

        let descending = days_since_start < ramp;
        let ramp_start_offset = if descending {
            0
        } else {
            diapause_duration.saturating_sub(ramp)
        };

        let prog = (u16_to_f32(secs_of_hour) * INV_86400
            + u16_to_f32(hour as u16) * INV_24
            + u16_to_f32(days_since_start - ramp_start_offset))
            * recip(u16_to_f32(ramp));

        let (a_temp, b_temp) = if descending {
            (diurnal_cycle_temp, self.diapause_temp)
        } else {
            (self.diapause_temp, diurnal_cycle_temp)
        };

        a_temp * (1.0 - prog) + b_temp * prog
    }

    const fn diapause_status(&self, time: RTCTime) -> &'static str {
        let (diapause_duration, days_since_start, ramp) = self.calc_diapause_window(time);

        if days_since_start < ramp {
            "[Ramp Down]"
        } else if days_since_start < diapause_duration.saturating_sub(ramp) {
            "[Diapause]"
        } else if days_since_start < diapause_duration {
            "[Ramp Up]"
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

crate::codegen::revolving_enum!(
    #[derive(Clone, Copy)]
    enum PageId {
        TimeAndTarget,
        TempReadings,
        Configuration,
        ManualControl,
    }
);

struct SelectIndex {
    idx: u8,
    window: u8,
    len: u8,
}

impl SelectIndex {
    const fn new_config() -> Self {
        Self {
            idx: 0,
            window: 0,
            len: ControllerConfig::FIELD_COUNT + 1,
        }
    }

    const fn new_control() -> Self {
        Self {
            idx: 0,
            window: 0,
            len: ControlState::FIELD_COUNT + 1,
        }
    }

    const fn inc(&mut self) {
        if self.idx < self.len - 1 {
            self.idx += 1;
            if self.window + 3 < self.idx {
                self.window += 1;
            }
        }
    }

    const fn dec(&mut self) {
        if self.idx > 0 {
            self.idx -= 1;
            if self.idx < self.window {
                self.window = self.idx;
            }
        }
    }

    const fn generate_select_page(&self, names: &[[u8; 18]]) -> PageData {
        let mut page = PageBuilder::new();
        let mut row = 0;
        while row < 4 {
            let display_idx = self.window + row;
            page = page
                .write(if display_idx == self.idx {
                    b"> "
                } else {
                    b"  "
                })
                .write(&names[display_idx as usize])
                .end_line();
            row += 1;
        }
        page.finish()
    }
}

crate::codegen::interactive!(
    /// Manual control for the [`ClimateController`]
    #[derive(Clone)]
    pub struct ControlState {
        compressor as Compressor: bool = false,
        heater as Heater: bool = false,

        duty_a as CondenserFan: Duty = Duty(0),
        duty_b as HabitatFan: Duty = Duty(0),
        duty_c as CoolantPump: Duty = Duty(0),
    }
    exit = b"[Return To Auto]  ";
    info = b"    Live Control    ";
    ControlBuffer
);

#[derive(Clone, Copy)]
struct Duty(u16);

impl Duty {
    const fn next(self) -> Self {
        Self(self.0 + (self.0 < 256) as u16)
    }

    const fn prev(self) -> Self {
        Self(self.0.saturating_sub(1))
    }
}

enum UIMode<'a> {
    Normal(&'a mut PageId),
    Select(&'a mut PageId, &'a mut SelectIndex),
    Edit(&'a mut ConfigBuffer),
    Control(&'a mut ControlBuffer),
}

struct UIState {
    page: PageId,
    select_idx: Option<SelectIndex>,
    edit_buffer: Option<ConfigBuffer>,
    control_buffer: Option<ControlBuffer>,
}

impl UIState {
    const fn new() -> Self {
        Self {
            page: PageId::TimeAndTarget,
            select_idx: None,
            edit_buffer: None,
            control_buffer: None,
        }
    }

    const fn mode(&mut self) -> UIMode<'_> {
        if let Some(buf) = self.control_buffer.as_mut() {
            UIMode::Control(buf)
        } else if let Some(buf) = self.edit_buffer.as_mut() {
            UIMode::Edit(buf)
        } else if let Some(select_idx) = self.select_idx.as_mut() {
            UIMode::Select(&mut self.page, select_idx)
        } else {
            UIMode::Normal(&mut self.page)
        }
    }

    const fn handle_click(&mut self, click: Click) -> Option<ControlBuffer> {
        match self.mode() {
            UIMode::Normal(page) => {
                *page = match click {
                    Click::CW => page.next(),
                    Click::CCW => page.prev(),
                };
                None
            }
            UIMode::Select(_, select_idx) => {
                match click {
                    Click::CW => select_idx.inc(),
                    Click::CCW => select_idx.dec(),
                }
                None
            }
            UIMode::Edit(buffer) => {
                buffer.adjust(click);
                None
            }
            UIMode::Control(buffer) => {
                buffer.adjust(click);
                Some(*buffer)
            }
        }
    }

    const fn handle_press(
        &mut self,
        config: &mut ControllerConfig,
        control: &ControlState,
    ) -> (bool, bool) {
        if self.control_buffer.take().is_some() {
            (false, false)
        } else if let Some(buffer) = self.edit_buffer.take() {
            config.set_buffer(buffer);
            (true, false)
        } else if let Some(ref mut select_idx) = self.select_idx {
            if matches!(self.page, PageId::Configuration) {
                self.edit_buffer = config.get_buffer(select_idx.idx);
                if self.edit_buffer.is_none() {
                    self.select_idx = None;
                }
            } else if matches!(self.page, PageId::ManualControl) {
                self.control_buffer = control.get_buffer(select_idx.idx);
                if self.control_buffer.is_none() {
                    self.select_idx = None;
                }
            }
            (false, false)
        } else if matches!(self.page, PageId::Configuration) {
            self.select_idx = Some(SelectIndex::new_config());
            (false, false)
        } else if matches!(self.page, PageId::ManualControl) {
            self.select_idx = Some(SelectIndex::new_control());
            (false, true)
        } else {
            (false, false)
        }
    }

    const fn is_in_manual_mode(&self) -> bool {
        matches!(self.page, PageId::ManualControl) && self.select_idx.is_some()
    }
}

/// Condition of the habitat with respect to target temperature
#[derive(Clone, Copy)]
#[repr(u8)]
enum HabitatCondition {
    TooCold,
    Cool,
    JustRight,
    Warm,
    TooHot,
}

impl HabitatCondition {
    const fn test(habitat: f32, target: f32) -> Self {
        let delta = habitat - target;
        if delta < -0.25 {
            Self::TooCold
        } else if delta < -0.05 {
            Self::Cool
        } else if delta < 0.05 {
            Self::JustRight
        } else if delta < 0.25 {
            Self::Warm
        } else {
            Self::TooHot
        }
    }

    const fn is_different(self, other: Self) -> bool {
        !matches!(
            (self, other),
            (Self::TooCold, Self::TooCold)
                | (Self::Cool, Self::Cool)
                | (Self::JustRight, Self::JustRight)
                | (Self::Warm, Self::Warm)
                | (Self::TooHot, Self::TooHot)
        )
    }

    const fn from_u8(i: u8) -> Self {
        match i {
            0 => Self::TooCold,
            1 => Self::Cool,
            2 => Self::JustRight,
            3 => Self::Warm,
            _ => Self::TooHot,
        }
    }

    const fn next_toward(self, other: Self) -> Self {
        let sd = self as u8;
        let od = other as u8;

        if sd == od {
            other
        } else if sd < od {
            Self::from_u8(sd + 1)
        } else {
            Self::from_u8(sd - 1)
        }
    }
}

/// Formicarium climate control system state machine
///
/// # Pin Configuration
///
/// `PORTB`:
/// - `PB0`: LCD D4
/// - `PB1`: LCD D5
/// - `PB2`: LCD D6
/// - `PB3`: LCD D7
/// - `PB4`: rotary encoder push button
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
/// - `PE2`: rotary encoder A[^2]
/// - `PE6`: rotary encoder B
///
/// `PORTF`:
/// - `PF0`: thermistor (unused)
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

    compressor: Relay<PD4>,
    heater: Relay<PD5>,
    _relay2: Relay<PD6>,
    master_120vac: Relay<PD7>,

    pwm: PWMController,

    rtc: DS1307,
    _sqw: Pin<Input<Floating>, PC7>,

    encoder: Encoder,

    _pc6: Pin<Input<Floating>, PC6>,
    _pf6: Pin<Input<Floating>, PF6>,
    _pf7: Pin<Input<Floating>, PF7>,

    display: Display,

    tc0: TC0,

    next_sample: u32,
    next_update: u32,
    next_display: u32,
    next_config: u32,

    target_temp: Target,

    config: ControllerConfig,
    config_changed: bool,

    control_state: ControlState,

    ui_state: UIState,

    last_condition: HabitatCondition,
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

            compressor: Relay::new(pins.pd4.into_output(), 0, 120, 1),
            heater: Relay::new(pins.pd5.into_output(), 60, 0, 1),
            _relay2: Relay::new(pins.pd6.into_output(), 0, 0, 0),
            master_120vac: Relay::new(pins.pd7.into_output(), 0, 0, 0),

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

            encoder: Encoder::new(pins.pe2, pins.pe6, pins.pb4, periphs.EXINT, periphs.TC4),

            _pc6: pins.pc6,
            _pf6: pins.pf6,
            _pf7: pins.pf7,

            display: Display::new(pins.pd2, pins.pd3, pins.pb0, pins.pb1, pins.pb2, pins.pb3),

            tc0: periphs.TC0,

            next_sample: 0,
            next_update: 0,
            next_display: 0,
            next_config: 0,

            target_temp: Target::Unset,

            config: ControllerConfig::DEFAULT,
            config_changed: false,

            control_state: ControlState::DEFAULT,

            ui_state: UIState::new(),

            last_condition: HabitatCondition::JustRight,
        }
    }

    /// Start the operation of the climate controller
    pub fn begin(&mut self) {
        arduino_hal::delay_ms(500);

        self.master_120vac.turn_on(0);

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
        if self.config_changed {
            let _ = self.rtc.set_ram(self.config.clone().into_data());
            self.config_changed = false;
        }
    }

    fn set_condenser_fan_duty(&mut self, duty: u16) {
        self.pwm.set_duty_a(duty);
    }

    fn set_habitat_fan_duty(&mut self, duty: u16) {
        self.pwm.set_duty_b(duty);
    }

    fn set_coolant_pump_duty(&mut self, duty: u16) {
        self.pwm.set_duty_c(duty);
    }

    const fn tune_subcooling(&mut self, delta: f32) {
        self.config.min_effective_subcooling += delta;
        self.config_changed = true;
    }

    #[inline(never)]
    fn update(&mut self, now: u32) {
        let Some(target) = self.target_temp.value() else {
            return;
        };

        let habitat = self.sensorium.habitat_temp().fahrenheit();
        let coolant = self.sensorium.coolant_temp().fahrenheit();
        let condenser = self.sensorium.condenser_temp().fahrenheit();

        let new_condition = self
            .last_condition
            .next_toward(HabitatCondition::test(habitat, target));
        if new_condition.is_different(self.last_condition) {
            let mut defer = false;

            match new_condition {
                HabitatCondition::TooCold => {
                    self.heater.turn_on(now);
                }
                HabitatCondition::Cool => {
                    self.set_habitat_fan_duty(0);
                }
                HabitatCondition::JustRight => {
                    self.heater.turn_off(now);
                }
                HabitatCondition::Warm => {
                    self.set_habitat_fan_duty(256);
                }
                HabitatCondition::TooHot => {
                    if self.compressor.turn_on(now) {
                        self.tune_subcooling(0.5);
                    } else {
                        defer = true;
                    }
                }
            }

            if !defer {
                self.last_condition = new_condition;
            }
        }

        let subcooling = target - coolant;
        if subcooling < self.config.min_effective_subcooling {
            if self.compressor.turn_on(now) {
                self.tune_subcooling(-0.1);
            }
        } else if subcooling > self.config.min_effective_subcooling + 10.0 {
            self.compressor.turn_off(now);
        }

        // Fail-safe: max out the condenser fan in case of condenser temp sensor failure to
        // avoid overheating the compressor
        self.set_condenser_fan_duty(if is_finite(condenser) {
            if condenser < 80.0 {
                0
            } else if condenser <= 90.0 {
                ((condenser - 80.0) * 25.6) as u16
            } else {
                256
            }
        } else {
            256
        });

        self.set_coolant_pump_duty(if self.compressor.is_on() {
            256
        } else if self.pwm.duty_b() > 0 {
            192
        } else {
            0
        });

        // Verify that the compressor has in fact switched on by checking if the condenser is hot
        self.compressor
            .verify_when_ready(now, || condenser >= 80.0, || true);
        self.compressor.restore_when_ready(now);

        // Verfy that the heater is not stuck on when switched off by checking if the target was
        // significantly overshot
        self.heater.verify_when_ready(
            now,
            || true,
            || {
                self.target_temp
                    .value()
                    .is_some_and(|target| habitat < target + 1.0)
            },
        );
        self.heater.restore_when_ready(now);
    }

    fn config(&mut self) {
        match self.target_temp {
            Target::Unset => {
                self.target_temp = if let Ok(time) = self.rtc.get_time() {
                    Target::Dynamic(self.config.calculate_target(time))
                } else {
                    // If the RTC does not respond, fail-safe by holding the current habitat temperature
                    Target::Static(self.sensorium.habitat_temp().fahrenheit())
                }
            }
            Target::Static(_) => {
                if let Ok(time) = self.rtc.get_time() {
                    self.load_config();
                    self.target_temp = Target::Dynamic(self.config.calculate_target(time));
                }
            }
            Target::Dynamic(_) => {
                if let Ok(time) = self.rtc.get_time() {
                    self.target_temp = Target::Dynamic(self.config.calculate_target(time));
                }
            }
        }

        self.save_config();
    }

    #[inline(never)]
    fn display(&mut self) {
        let page = match self.ui_state.mode() {
            UIMode::Normal(page) => match page {
                PageId::TimeAndTarget => {
                    let page = if let Ok(time) = self.rtc.get_time() {
                        PageBuilder::new()
                            .write(time.day.abbrev().as_bytes())
                            .skip(1)
                            .write(b"20")
                            .hexit2(time.year.bcd())
                            .byte(b'.')
                            .hexit2(time.month.bcd())
                            .byte(b'.')
                            .hexit2(time.date.bcd())
                            .end_line()
                            .hexit2(time.hours.bcd_24h())
                            .byte(b':')
                            .hexit2(time.minutes.bcd())
                            .byte(b':')
                            .hexit2(time.seconds.bcd())
                            .skip(1)
                            .write(self.config.diapause_status(time).as_bytes())
                            .end_line()
                    } else {
                        PageBuilder::new()
                            .write(b"RTC not responding")
                            .end_line()
                            .next_line()
                    };

                    if let Some(target) = self.target_temp.value() {
                        page.write(b"Setpoint:   ").decimal(target).byte(b'F')
                    } else {
                        page.write(b"Calibrating...").end_line()
                    }
                    .write(b"Habitat:    ")
                    .decimal(self.sensorium.habitat_temp().fahrenheit())
                    .byte(b'F')
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
                    .finish(),
                PageId::Configuration => PageBuilder::new()
                    .write(b"> [Press To Config] ")
                    .write(b"  ...")
                    .finish(),
                PageId::ManualControl => PageBuilder::new()
                    .write(b"> [Press To Control]")
                    .write(b"  ...")
                    .finish(),
            },
            UIMode::Select(page, select_idx) => select_idx.generate_select_page(match *page {
                PageId::Configuration => &ControllerConfig::NAMES,
                PageId::ManualControl => &ControlState::NAMES,
                _ => unreachable!(),
            }),
            UIMode::Edit(buffer) => buffer.generate_edit_page(),
            UIMode::Control(buffer) => buffer.generate_edit_page(),
        };

        self.display.write_page(page);
    }

    fn periodic(&mut self) {
        let now = millis();

        if now >= self.next_sample {
            self.sensorium.sample();
            self.next_sample += SAMPLE_INTERVAL;
        }

        if !self.ui_state.is_in_manual_mode() && now >= self.next_update {
            self.update(now);
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

        if let Some(click) = self.encoder.next_click() {
            if let Some(buf) = self.ui_state.handle_click(click) {
                match buf {
                    ControlBuffer::Compressor(value) => {
                        if value {
                            self.compressor.force_on();
                        } else {
                            self.compressor.force_off();
                        }
                    }
                    ControlBuffer::Heater(value) => {
                        if value {
                            self.heater.force_on();
                        } else {
                            self.heater.force_off();
                        }
                    }
                    ControlBuffer::CondenserFan(value) => self.pwm.set_duty_a(value.0),
                    ControlBuffer::HabitatFan(value) => self.pwm.set_duty_b(value.0),
                    ControlBuffer::CoolantPump(value) => self.pwm.set_duty_c(value.0),
                }
                self.control_state.set_buffer(buf);
            }
        }

        if self.encoder.was_pressed() {
            let entering_manual;
            (self.config_changed, entering_manual) = self
                .ui_state
                .handle_press(&mut self.config, &self.control_state);
            if entering_manual {
                self.control_state.compressor = self.compressor.is_on();
                self.control_state.heater = self.heater.is_on();
                self.control_state.duty_a = Duty(self.pwm.duty_a());
                self.control_state.duty_b = Duty(self.pwm.duty_b());
                self.control_state.duty_c = Duty(self.pwm.duty_c());
            }
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
