//! NTC thermistor abstractions

use core::cell::Cell;

use arduino_hal::{
    adc::AdcChannel,
    hal::Atmega,
    pac::ADC,
    port::{mode::Analog, Pin, PinOps},
    Adc,
};

/// Abstraction for NTC Thermistor measurement
///
/// The expected wiring is a voltage divider with the measurement pin in the middle and the
/// thermistor on the VCC side. The GND side resistor value should be roughly equal to the
/// thermistor's value at the middle of the expected operating temperature range for maximum
/// accuracy
pub struct Thermistor<PIN> {
    pin: Pin<Analog, PIN>,
    r0: f32,
    b: f32,
    r_bias: f32,

    sample: f32,
    kelvin: Cell<Option<f32>>,
}

impl<PIN> Thermistor<PIN>
where
    PIN: PinOps,
{
    /// Bind a specified thermistor to an analog pin
    pub const fn new(pin: Pin<Analog, PIN>, r0: f32, b: f32, r_bias: f32) -> Self {
        Self {
            pin,
            r0,
            b,
            r_bias,

            sample: 0.0,
            kelvin: Cell::new(None),
        }
    }

    /// Sample the voltage produced by the divider circuit
    ///
    /// The first sample is taken as a baseline, with the following 10 samples progressively
    /// decreasing in sensitivity to quickly settle fluctuations. After that, all samples go through
    /// a low-sensitivity IIR filter to mitigate noise
    pub fn sample(&mut self, adc: &mut Adc, sens: f32)
    where
        Pin<Analog, PIN>: AdcChannel<Atmega, ADC>,
    {
        let val = self.pin.analog_read(adc) as f32;

        self.sample = self.sample * (1.0 - sens) + val * sens;

        self.kelvin.set(None);
    }

    /// Return the measured temperature in kelvin
    pub fn kelvin(&self) -> f32 {
        if let Some(kelvin) = self.kelvin.get() {
            return kelvin;
        }

        let ohms = self.r_bias * (1023.0 / self.sample - 1.0);
        let kelvin = self.b / (libm::logf(ohms / self.r0) + self.b / (273.15 + 25.0));

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
