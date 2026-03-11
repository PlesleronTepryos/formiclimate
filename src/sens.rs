//! Sensor abstractions for climate controller

use core::{cell::Cell, f32};

use arduino_hal::{
    adc::{AdcChannel, AdcSettings},
    hal::{
        port::{PF0, PF1, PF4, PF5},
        Atmega,
    },
    pac::ADC,
    port::{
        mode::{Analog, Floating, Input},
        Pin, PinOps,
    },
    Adc,
};

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

    /// Access coolant temperature (read-only)
    pub const fn coolant_temp(&self) -> &Thermistor<PF5> {
        &self.coolant_temp
    }

    /// Access habitat temperature (read-only)
    pub const fn habitat_temp(&self) -> &Thermistor<PF4> {
        &self.habitat_temp
    }

    /// Access condenser temperature (read-only)
    pub const fn condenser_temp(&self) -> &Thermistor<PF1> {
        &self.condenser_temp
    }

    /// Access evaporator temperature (read-only)
    pub const fn evaporator_temp(&self) -> &Thermistor<PF0> {
        &self.evaporator_temp
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
pub struct Thermistor<PIN> {
    pin: Pin<Analog, PIN>,
    r0: f32,
    b: f32,
    r_bias: f32,

    sample: f32,
    kelvin: Cell<Option<f32>>,

    bad_samples: u8,
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

            bad_samples: 0,
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
        let val = self.pin.analog_read(adc);

        if (8..1016).contains(&val) {
            self.bad_samples = self.bad_samples.saturating_sub(1);
        } else {
            self.bad_samples = self.bad_samples.saturating_add(1);
            return;
        }

        self.sample = self.sample * (1.0 - sens) + val as f32 * sens;

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

        let ohms = self.r_bias * (1023.0 / self.sample - 1.0);
        let kelvin = self.b / (ln(ohms / self.r0) + self.b / (273.15 + 25.0));

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

/// Natural log approximation adapted from [<https://quadst.rip/ln-approx>]
///
/// This saves nearly 500 bytes of program memory as opposed to `libm::logf` and has a max relative
/// error of ~6.06e-5, which is more than suitable for this application
const fn ln(x: f32) -> f32 {
    let bx = x.to_bits();
    let t = i16_to_f32((bx >> 23) as i16 - 127);
    let y = f32::from_bits(1_065_353_216 | (bx & 8_388_607));
    -1.741_793_9
        + (2.821_202_6 + (-1.469_956_8 + (0.447_179_55 - 0.056_570_85 * y) * y) * y) * y
        + f32::consts::LN_2 * t
}

/// This saves ~250 additional bytes over the builtin `x as f32` conversion
const fn i16_to_f32(x: i16) -> f32 {
    match x {
        0 => 0.0,
        i16::MIN => -32768.0,
        x => {
            let sign: u32 = if x < 0 { 1 } else { 0 };
            let abs = (if x < 0 { -x } else { x }) as u16 as u32;

            let mut bit_pos = 0u32;
            let mut tmp = abs >> 1;
            while tmp != 0 {
                tmp >>= 1;
                bit_pos += 1;
            }

            let exp = bit_pos + 127;
            let mantissa = (abs ^ (1 << bit_pos)) << (23 - bit_pos);
            f32::from_bits((sign << 31) | (exp << 23) | mantissa)
        }
    }
}
