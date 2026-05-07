//! Sensor abstractions for climate controller

use core::{cell::Cell, f32};

use arduino_hal::{
    adc::AdcSettings,
    hal::port::{PF0, PF1, PF4, PF5},
    pac::ADC,
    port::{
        mode::{Analog, Floating, Input},
        Pin,
    },
    Adc,
};

use crate::utils::{ln, recip, u16_to_f32};

/// The control system's complete sensory apparatus
#[must_use]
pub struct Sensorium {
    adc: Adc,
    coolant_pin: Pin<Analog, PF5>,
    habitat_pin: Pin<Analog, PF4>,
    condenser_pin: Pin<Analog, PF1>,
    _pin3: Pin<Analog, PF0>,

    coolant_temp: Thermistor,
    habitat_temp: Thermistor,
    condenser_temp: Thermistor,
    _temp3: Thermistor,

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
            coolant_pin: a2.into_analog_input(&mut adc),
            habitat_pin: a3.into_analog_input(&mut adc),
            condenser_pin: a4.into_analog_input(&mut adc),
            _pin3: a5.into_analog_input(&mut adc),

            coolant_temp: Thermistor::new(10_000.0, 3_380.0, 9_820.0),
            habitat_temp: Thermistor::new(20_000.0, 3_950.0, 21_440.0),
            condenser_temp: Thermistor::new(50_000.0, 3_950.0, 46_200.0),
            _temp3: Thermistor::new(10_000.0, 3_380.0, 9_860.0),

            adc,

            sens: 1.0,
            sens_steps: 10,
        }
    }

    /// Take a measurement sample on all sensors
    pub fn sample(&mut self) {
        let coolant_sample = self.coolant_pin.analog_read(&mut self.adc);
        let habitat_sample = self.habitat_pin.analog_read(&mut self.adc);
        let condenser_sample = self.condenser_pin.analog_read(&mut self.adc);

        self.coolant_temp.sample(coolant_sample, self.sens);
        self.habitat_temp.sample(habitat_sample, self.sens);
        self.condenser_temp.sample(condenser_sample, self.sens);

        if self.sens_steps > 0 {
            self.sens *= 0.5;
            self.sens_steps -= 1;
        }
    }

    /// Access coolant temperature (read-only)
    pub const fn coolant_temp(&self) -> &Thermistor {
        &self.coolant_temp
    }

    /// Access habitat temperature (read-only)
    pub const fn habitat_temp(&self) -> &Thermistor {
        &self.habitat_temp
    }

    /// Access condenser temperature (read-only)
    pub const fn condenser_temp(&self) -> &Thermistor {
        &self.condenser_temp
    }
}

/// Abstraction for NTC Thermistor measurement
///
/// The expected wiring is a voltage divider with the measurement pin in the middle, a resistor to
/// GND, and the thermistor to VCC.
///
/// Note: the GND side resistor value should be roughly equal to the thermistor's value at the
/// middle of the expected operating temperature range. This is both to maximize accuracy and so
/// that sampled voltages very close to GND or VCC can correctly be rejected as invalid due to the
/// thermistor failing open or short respectively. Invalid samples increment a counter, and if that
/// counter exceeds a threshold, the calculated temperature will be reported as NaN until enough
/// valid samples are taken to decrement the counter below the threshold.
#[must_use]
pub struct Thermistor {
    b: f32,
    sh_h_fixed: f32,

    sample: f32,
    kelvin: Cell<Option<f32>>,

    bad_samples: u8,
}

impl Thermistor {
    /// Initialize sampling and temperature calculation
    pub const fn new(r0: f32, b: f32, r_bias: f32) -> Self {
        const INV_25C: f32 = 1.0 / (273.15 + 25.0);
        Self {
            b,
            sh_h_fixed: ln(r_bias) - ln(r0) + b * INV_25C,

            sample: 0.0,
            kelvin: Cell::new(None),

            bad_samples: 0,
        }
    }

    /// Sample the voltage produced by the divider circuit
    ///
    /// The first sample is taken as a baseline, with the following 10 samples progressively
    /// decreasing in sensitivity to quickly settle fluctuations. After that, all samples go through
    /// a low-sensitivity IIR filter to mitigate noise
    pub fn sample(&mut self, value: u16, sens: f32) {
        if (8..1016).contains(&value) {
            self.bad_samples = self.bad_samples.saturating_sub(1);
        } else {
            self.bad_samples = self.bad_samples.saturating_add(1);
            return;
        }

        self.sample = self.sample * (1.0 - sens) + u16_to_f32(value) * sens;

        self.kelvin.set(None);
    }

    /// Return the measured temperature in kelvin
    pub fn kelvin(&self) -> f32 {
        if self.bad_samples >= 16 {
            return f32::NAN;
        }

        if let Some(kelvin) = self.kelvin.get() {
            return kelvin;
        }

        let kelvin = self.b * recip(ln(1023.0 * recip(self.sample) - 1.0) + self.sh_h_fixed);

        self.kelvin.set(Some(kelvin));

        kelvin
    }

    /// Return the measured temperature in celsius
    pub fn celsius(&self) -> f32 {
        self.kelvin() - 273.15
    }

    /// Return the measured temperature in fahrenheit
    pub fn fahrenheit(&self) -> f32 {
        self.celsius() * 1.8 + 32.0
    }
}
