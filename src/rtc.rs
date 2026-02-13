//! ds1307 RTC abstractions and API

use arduino_hal::{prelude::*, I2c};

type I2cResult<T = ()> = Result<T, arduino_hal::i2c::Error>;

const DS1307_ADDR: u8 = 0x68;

/// ds1307 real-time clock module; interfaced via I2C
///
/// No internal state; can be freely constructed/destructed if the I2c bus must be shared
#[must_use]
pub struct DS1307 {
    i2c: I2c,
}

// Specialized methods
impl DS1307 {
    /// Connect to ds1307 by taking ownership of the I2C bus
    pub const fn new(i2c: I2c) -> Self {
        Self { i2c }
    }

    /// Disconnect to release the I2C bus
    #[must_use]
    pub const fn release(self) -> I2c {
        self.i2c
    }

    /// Corrects any illogical values in time data on-chip
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn validate(&mut self) -> I2cResult {
        let mut buf = [0u8; 7];
        self.i2c.write_read(DS1307_ADDR, &[0], &mut buf)?;

        let seconds = Seconds::try_from_bcd(buf[0]).unwrap_or_default();
        let minutes = Minutes::try_from_bcd(buf[1]).unwrap_or_default();
        let hours = Hours::try_from_bcd(buf[2]).unwrap_or_default();
        let day = Day::try_from_bcd(buf[3]).unwrap_or_default();
        let mut date = Date::try_from_bcd(buf[4]).unwrap_or_default();
        let month = Month::try_from_bcd(buf[5]).unwrap_or_default();
        let year = Year::try_from_bcd(buf[6]).unwrap_or_default();

        if date.bin() > month.length(year.is_leap()) {
            date = Date(1);
        }

        let valid_buf = RTCTime {
            seconds,
            minutes,
            hours,
            day,
            date,
            month,
            year,
        }
        .as_write();

        self.i2c.write(DS1307_ADDR, &valid_buf)
    }

    /// Zero out the time and date to the earliest valid value
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn clear_clock(&mut self) -> I2cResult {
        let buf = RTCTime::default().as_write();
        self.i2c.write(DS1307_ADDR, &buf)
    }

    /// Set the clock halt bit to disable timekeeping
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn halt_clock(&mut self) -> I2cResult {
        let seconds = self.get_seconds()?;
        let buf = [0, seconds.bcd() | 0b1000_0000];
        self.i2c.write(DS1307_ADDR, &buf)
    }

    /// Clear the clock halt bit to enable timekeeping
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn start_clock(&mut self) -> I2cResult {
        let seconds = self.get_seconds()?;
        let buf = [0, seconds.bcd() & 0b0111_1111];
        self.i2c.write(DS1307_ADDR, &buf)
    }

    /// Enable square wave output
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn sqw_enable(&mut self) -> I2cResult {
        let mut control = [0u8];
        self.i2c.read(DS1307_ADDR, &mut control)?;
        self.i2c.write(DS1307_ADDR, &[7, control[0] | 0b0001_0000])
    }

    /// Disable square wave output
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn sqw_disable(&mut self) -> I2cResult {
        let mut control = [0u8];
        self.i2c.read(DS1307_ADDR, &mut control)?;
        self.i2c.write(DS1307_ADDR, &[7, control[0] & 0b1110_1111])
    }

    /// Get square wave output frequency
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn sqw_get_freq(&mut self) -> I2cResult<Freq> {
        let mut control = [0u8];
        self.i2c
            .read(DS1307_ADDR, &mut control)
            .map(|()| Freq::from_bits(control[0] & 0x3))
    }

    /// Set square wave output frequency
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn sqw_set_freq(&mut self, freq: Freq) -> I2cResult {
        let mut control = [0u8];
        self.i2c.read(DS1307_ADDR, &mut control)?;
        self.i2c
            .write(DS1307_ADDR, &[7, (control[0] & 0xfc) | freq as u8])
    }

    /// Get entire RAM block
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_ram(&mut self) -> I2cResult<[u8; 56]> {
        let mut buf = [0u8; 56];
        self.i2c
            .write_read(DS1307_ADDR, &[8], &mut buf)
            .map(|()| buf)
    }

    /// Set entire RAM block
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    #[expect(clippy::manual_memcpy, reason = "no_std")]
    pub fn set_ram(&mut self, ram: [u8; 56]) -> I2cResult {
        let mut buf = [0u8; 57];
        for i in 1..57 {
            buf[i] = ram[i - 1];
        }
        self.i2c.write(DS1307_ADDR, &buf)
    }
}

// Time getters
impl DS1307 {
    /// Get complete date and time reading
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_time(&mut self) -> I2cResult<RTCTime> {
        let mut buf = [0u8; 7];
        self.i2c
            .write_read(DS1307_ADDR, &[0], &mut buf)
            .map(|()| RTCTime::from_bcd(buf))
    }

    /// Get seconds
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_seconds(&mut self) -> I2cResult<Seconds> {
        let mut buf = [0u8];
        self.i2c
            .write_read(DS1307_ADDR, &[0], &mut buf)
            .map(|()| Seconds::from_bcd(buf[0]))
    }

    /// Get minutes
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_minutes(&mut self) -> I2cResult<Minutes> {
        let mut buf = [0u8];
        self.i2c
            .write_read(DS1307_ADDR, &[1], &mut buf)
            .map(|()| Minutes::from_bcd(buf[0]))
    }

    /// Get hours
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_hours(&mut self) -> I2cResult<Hours> {
        let mut buf = [0u8];
        self.i2c
            .write_read(DS1307_ADDR, &[2], &mut buf)
            .map(|()| Hours::from_bcd(buf[0]))
    }

    /// Get day
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_day(&mut self) -> I2cResult<Day> {
        let mut buf = [0u8];
        self.i2c
            .write_read(DS1307_ADDR, &[3], &mut buf)
            .map(|()| Day::from_bcd(buf[0]))
    }

    /// Get date
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_date(&mut self) -> I2cResult<Date> {
        let mut buf = [0u8];
        self.i2c
            .write_read(DS1307_ADDR, &[4], &mut buf)
            .map(|()| Date::from_bcd(buf[0]))
    }

    /// Get month
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_month(&mut self) -> I2cResult<Month> {
        let mut buf = [0u8];
        self.i2c
            .write_read(DS1307_ADDR, &[5], &mut buf)
            .map(|()| Month::from_bcd(buf[0]))
    }

    /// Get year
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn get_year(&mut self) -> I2cResult<Year> {
        let mut buf = [0u8];
        self.i2c
            .write_read(DS1307_ADDR, &[6], &mut buf)
            .map(|()| Year::from_bcd(buf[0]))
    }
}

// Time setters
impl DS1307 {
    /// Set complete date and time reading
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_time(&mut self, time: RTCTime) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &time.as_write())
    }

    /// Set seconds
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_seconds(&mut self, seconds: Seconds) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &[0, seconds.bcd()])
    }

    /// Set minutes
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_minutes(&mut self, minutes: Minutes) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &[1, minutes.bcd()])
    }

    /// Set hours
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_hours(&mut self, hours: Hours) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &[2, hours.bcd_24h()])
    }

    /// Set day
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_day(&mut self, day: Day) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &[3, day.bcd()])
    }

    /// Set date
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_date(&mut self, date: Date) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &[4, date.bcd()])
    }

    /// Set month
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_month(&mut self, month: Month) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &[5, month.bcd()])
    }

    /// Set year
    ///
    /// # Errors
    /// Returns an error if the something goes wrong on the I2C bus
    pub fn set_year(&mut self, year: Year) -> I2cResult {
        self.i2c.write(DS1307_ADDR, &[6, year.bcd()])
    }
}

/// Square wave freqency selection
#[expect(missing_docs, reason = "self-explanatory variants")]
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Freq {
    Hz1,
    Hz4096,
    Hz8192,
    Hz32768,
}

impl Freq {
    /// Construct from binary representation; panics if out of range
    #[must_use]
    pub const fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::Hz1,
            1 => Self::Hz4096,
            2 => Self::Hz8192,
            3 => Self::Hz32768,
            _ => panic!(),
        }
    }
}

/// Complete time reading; layout identical to [DS1307] internally
#[expect(missing_docs, reason = "self-explanatory variants")]
#[derive(Debug, Clone, Copy, Default)]
#[repr(C)]
pub struct RTCTime {
    pub seconds: Seconds,
    pub minutes: Minutes,
    pub hours: Hours,
    pub day: Day,
    pub date: Date,
    pub month: Month,
    pub year: Year,
}

impl RTCTime {
    /// Construct from BCD representation
    ///
    /// # Errors
    /// Returns an error if any value is out of range or is invalid BCD
    pub const fn try_from_bcd(bytes: [u8; 7]) -> Result<Self, [u8; 7]> {
        let seconds = Seconds::try_from_bcd(bytes[0]);
        let minutes = Minutes::try_from_bcd(bytes[1]);
        let hours = Hours::try_from_bcd(bytes[2]);
        let day = Day::try_from_bcd(bytes[3]);
        let date = Date::try_from_bcd(bytes[4]);
        let month = Month::try_from_bcd(bytes[5]);
        let year = Year::try_from_bcd(bytes[6]);

        if let (Ok(seconds), Ok(minutes), Ok(hours), Ok(day), Ok(date), Ok(month), Ok(year)) =
            (seconds, minutes, hours, day, date, month, year)
        {
            Ok(Self {
                seconds,
                minutes,
                hours,
                day,
                date,
                month,
                year,
            })
        } else {
            Err(bytes)
        }
    }

    /// Construct from BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bytes: [u8; 7]) -> Self {
        if let Ok(v) = Self::try_from_bcd(bytes) {
            return v;
        }
        panic!();
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn bcd(self) -> [u8; 7] {
        [
            self.seconds.bcd(),
            self.minutes.bcd(),
            self.hours.bcd_24h(),
            self.day.bcd(),
            self.date.bcd(),
            self.month.bcd(),
            self.year.bcd(),
        ]
    }

    const fn as_write(self) -> [u8; 8] {
        let time = self.bcd();
        let mut buf = [0u8; 8];
        let mut i = 1;
        while i < 8 {
            buf[i] = time[i - 1];
            i += 1;
        }
        buf
    }
}

/// Seconds encoded as 2 digit BCD
///
/// Note: bit 7 is allowed to be set, but this will not reflect in the value of seconds
#[derive(Debug, Clone, Copy, Default)]
#[repr(transparent)]
pub struct Seconds(u8);

impl Seconds {
    /// Construct from BCD representation
    ///
    /// # Errors
    /// Returns an error if the value is out of range or is invalid BCD
    pub const fn try_from_bcd(bcd: u8) -> Result<Self, u8> {
        if bcd & 0x7f <= 0x59 && bcd & 0xf <= 9 {
            Ok(Self(bcd))
        } else {
            Err(bcd)
        }
    }

    /// Construct from BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bcd: u8) -> Self {
        if let Ok(v) = Self::try_from_bcd(bcd) {
            return v;
        }
        panic!();
    }

    /// Construct from binary representation; panics if out of range
    #[must_use]
    pub const fn from_bin(value: u8) -> Self {
        assert!(value <= 59, "value out of range");

        let mut ones = value;
        let mut tens = 0;
        while ones > 9 {
            ones -= 10;
            tens += 1;
        }

        Self((tens << 4) + ones)
    }

    /// Returns value as binary
    #[must_use]
    pub const fn bin(self) -> u8 {
        decode_bcd7b(self.0)
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn bcd(self) -> u8 {
        self.0
    }
}

/// Minutes encoded as 2 digit BCD
#[derive(Debug, Clone, Copy, Default)]
#[repr(transparent)]
pub struct Minutes(u8);

impl Minutes {
    /// Construct from BCD representation
    ///
    /// # Errors
    /// Returns an error if the value is out of range or is invalid BCD
    pub const fn try_from_bcd(bcd: u8) -> Result<Self, u8> {
        if bcd <= 0x59 && bcd & 0xf <= 9 {
            Ok(Self(bcd))
        } else {
            Err(bcd)
        }
    }

    /// Construct from BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bcd: u8) -> Self {
        if let Ok(v) = Self::try_from_bcd(bcd) {
            return v;
        }
        panic!();
    }

    /// Construct from binary representation; panics if out of range
    #[must_use]
    pub const fn from_bin(value: u8) -> Self {
        assert!(value <= 59, "value out of range");

        let mut ones = value;
        let mut tens = 0;
        while ones > 9 {
            ones -= 10;
            tens += 1;
        }

        Self((tens << 4) + ones)
    }

    /// Returns value as binary
    #[must_use]
    pub const fn bin(self) -> u8 {
        decode_bcd7b(self.0)
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn bcd(self) -> u8 {
        self.0
    }
}

/// Hours encoded as 2 digit BCD
///
/// 12/24-hour format detected and handled automatically
///
/// Internally normalized to 24-hour format
#[derive(Debug, Clone, Copy, Default)]
#[repr(transparent)]
pub struct Hours(u8);

impl Hours {
    /// Construct from 12/24-hour BCD representation
    ///
    /// # Errors
    /// Returns an error if the value is out of range or is invalid BCD
    pub const fn try_from_bcd(bcd: u8) -> Result<Self, u8> {
        match bcd >> 6 {
            // 24-hour format check
            0 if bcd <= 0x24 && bcd & 0xf <= 9 => Ok(Self(bcd)),

            // 12-hour format check
            1 if bcd != 0 && bcd & 0x1f <= 0x12 && bcd & 0xf <= 9 => {
                // AM hours are unchanged except 12AM becomes 0
                if bcd & 0x20 == 0 {
                    if bcd == 0x12 {
                        Ok(Self(0))
                    } else {
                        Ok(Self(bcd))
                    }
                // 8PM & 9PM require a half-carry (+6) to convert to 24-hour format
                } else if bcd & 0xf >= 8 {
                    Ok(Self((bcd & 0x1f) + 0x18))
                // Other PM hours require no carry except for 12PM which is left unchanged
                } else if bcd & 0x1f != 0x12 {
                    Ok(Self((bcd & 0x1f) + 0x12))
                } else {
                    Ok(Self(bcd))
                }
            }

            _ => Err(bcd),
        }
    }

    /// Construct from 12/24-hour BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bcd: u8) -> Self {
        if let Ok(v) = Self::try_from_bcd(bcd) {
            return v;
        }
        panic!();
    }

    /// Construct from 24-hour binary representation; panics if out of range
    #[must_use]
    pub const fn from_bin(value: u8) -> Self {
        assert!(value <= 23, "value out of range");

        let mut ones = value;
        let mut tens = 0;
        while ones > 9 {
            ones -= 10;
            tens += 1;
        }

        Self((tens << 4) + ones)
    }

    /// Returns value as binary
    #[must_use]
    pub const fn bin(self) -> u8 {
        decode_bcd6b(self.0)
    }

    /// Returns value as 24-hour BCD
    #[must_use]
    pub const fn bcd_24h(self) -> u8 {
        self.0
    }

    /// Returns value as 12-hour BCD
    #[must_use]
    pub const fn bcd_12h(self) -> u8 {
        unimplemented!()
    }
}

/// Day of the week
#[expect(missing_docs, reason = "self-explanatory variants")]
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Day {
    Sunday = 1,
    Monday = 2,
    Tuesday = 3,
    Wednesday = 4,
    Thursday = 5,
    Friday = 6,
    Saturday = 7,
}

impl Day {
    /// Construct from BCD representation
    ///
    /// # Errors
    /// Returns an error if the value is out of range or is invalid BCD
    pub const fn try_from_bcd(bcd: u8) -> Result<Self, u8> {
        match bcd {
            1 => Ok(Self::Sunday),
            2 => Ok(Self::Monday),
            3 => Ok(Self::Tuesday),
            4 => Ok(Self::Wednesday),
            5 => Ok(Self::Thursday),
            6 => Ok(Self::Friday),
            7 => Ok(Self::Saturday),
            _ => Err(bcd),
        }
    }

    /// Construct from BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bcd: u8) -> Self {
        if let Ok(v) = Self::try_from_bcd(bcd) {
            return v;
        }
        panic!();
    }

    /// Name of [Day] as text
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Sunday => "Sunday",
            Self::Monday => "Monday",
            Self::Tuesday => "Tuesday",
            Self::Wednesday => "Wednesday",
            Self::Thursday => "Thursday",
            Self::Friday => "Friday",
            Self::Saturday => "Saturday",
        }
    }

    /// Returns value as binary
    #[must_use]
    pub const fn bin(self) -> u8 {
        self.bcd()
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn bcd(self) -> u8 {
        match self {
            Self::Sunday => 1,
            Self::Monday => 2,
            Self::Tuesday => 3,
            Self::Wednesday => 4,
            Self::Thursday => 5,
            Self::Friday => 6,
            Self::Saturday => 7,
        }
    }
}

impl Default for Day {
    fn default() -> Self {
        Self::Sunday
    }
}

/// Day of the month encoded as 2 digit BCD
#[derive(Debug, Clone, Copy)]
#[repr(transparent)]
pub struct Date(u8);

impl Date {
    /// Construct from BCD representation
    ///
    /// # Errors
    /// Returns an error if the value is out of range or is invalid BCD
    pub const fn try_from_bcd(bcd: u8) -> Result<Self, u8> {
        if bcd != 0 && bcd <= 0x31 && bcd & 0xf <= 9 {
            Ok(Self(bcd))
        } else {
            Err(bcd)
        }
    }

    /// Construct from BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bcd: u8) -> Self {
        if let Ok(v) = Self::try_from_bcd(bcd) {
            return v;
        }
        panic!();
    }

    /// Construct from binary representation; panics if out of range
    #[must_use]
    pub const fn from_bin(value: u8) -> Self {
        assert!(value != 0 && value <= 31, "value out of range");

        let mut ones = value;
        let mut tens = 0;
        while ones > 9 {
            ones -= 10;
            tens += 1;
        }

        Self((tens << 4) + ones)
    }

    /// Returns value as binary
    #[must_use]
    pub const fn bin(self) -> u8 {
        decode_bcd6b(self.0)
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn bcd(self) -> u8 {
        self.0
    }
}

impl Default for Date {
    fn default() -> Self {
        Self(1)
    }
}

/// Month of the year
#[expect(missing_docs, reason = "self-explanatory variants")]
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Month {
    January = 1,
    February = 2,
    March = 3,
    April = 4,
    May = 5,
    June = 6,
    July = 7,
    August = 8,
    September = 9,
    October = 10,
    November = 11,
    December = 12,
}

impl Month {
    /// Construct from BCD representation
    ///
    /// # Errors
    /// Returns an error if the value is out of range or is invalid BCD
    pub const fn try_from_bcd(bcd: u8) -> Result<Self, u8> {
        match bcd {
            1 => Ok(Self::January),
            2 => Ok(Self::February),
            3 => Ok(Self::March),
            4 => Ok(Self::April),
            5 => Ok(Self::May),
            6 => Ok(Self::June),
            7 => Ok(Self::July),
            8 => Ok(Self::August),
            9 => Ok(Self::September),
            0x10 => Ok(Self::October),
            0x11 => Ok(Self::November),
            0x12 => Ok(Self::December),
            _ => Err(bcd),
        }
    }

    /// Construct from BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bcd: u8) -> Self {
        if let Ok(v) = Self::try_from_bcd(bcd) {
            return v;
        }
        panic!();
    }

    /// Construct from binary representation; panics if out of range
    #[must_use]
    pub const fn from_bin(value: u8) -> Self {
        assert!(value != 0 && value <= 12, "value out of range");
        Self::from_bcd(value + if value > 9 { 6 } else { 1 })
    }

    /// Name of [Month] as text
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::January => "January",
            Self::February => "February",
            Self::March => "March",
            Self::April => "April",
            Self::May => "May",
            Self::June => "June",
            Self::July => "July",
            Self::August => "August",
            Self::September => "September",
            Self::October => "October",
            Self::November => "November",
            Self::December => "December",
        }
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn length(self, leap: bool) -> u8 {
        match self {
            Self::January
            | Self::March
            | Self::May
            | Self::July
            | Self::August
            | Self::October
            | Self::December => 31,
            Self::February => 28 + leap as u8,
            Self::April | Self::June | Self::September | Self::November => 30,
        }
    }

    /// Returns value as binary
    #[must_use]
    pub const fn bin(self) -> u8 {
        self as u8
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn bcd(self) -> u8 {
        match self {
            Self::January => 1,
            Self::February => 2,
            Self::March => 3,
            Self::April => 4,
            Self::May => 5,
            Self::June => 6,
            Self::July => 7,
            Self::August => 8,
            Self::September => 9,
            Self::October => 0x10,
            Self::November => 0x11,
            Self::December => 0x12,
        }
    }
}

impl Default for Month {
    fn default() -> Self {
        Self::January
    }
}

/// Year encoded as 2 digit BCD
#[derive(Debug, Clone, Copy, Default)]
#[repr(transparent)]
pub struct Year(u8);

impl Year {
    /// Construct from BCD representation
    ///
    /// # Errors
    /// Returns an error if the value is out of range or is invalid BCD
    pub const fn try_from_bcd(bcd: u8) -> Result<Self, u8> {
        if bcd <= 0x99 && bcd & 0xf <= 9 {
            Ok(Self(bcd))
        } else {
            Err(bcd)
        }
    }

    /// Construct from BCD representation; panics if invalid or out of range
    #[must_use]
    pub const fn from_bcd(bcd: u8) -> Self {
        if let Ok(v) = Self::try_from_bcd(bcd) {
            return v;
        }
        panic!();
    }

    /// Construct from binary representation; panics if out of range
    #[must_use]
    pub const fn from_bin(value: u8) -> Self {
        assert!(value <= 99, "value out of range");

        let mut ones = value;
        let mut tens = 0;
        while ones > 9 {
            ones -= 10;
            tens += 1;
        }

        Self((tens << 4) + ones)
    }

    /// Returns value as binary
    #[must_use]
    pub const fn bin(self) -> u8 {
        decode_bcd8b(self.0)
    }

    /// Returns value as BCD
    #[must_use]
    pub const fn bcd(self) -> u8 {
        self.0
    }

    /// Whether the year is a leap year
    ///
    /// Note: does not account for 100 year or 400 year correction
    #[must_use]
    pub const fn is_leap(self) -> bool {
        self.0 & 0x1 == 0 && ((self.0 & 0x10 == 0) ^ (self.0 & 0x2 != 0))
    }
}

const fn decode_bcd8b(byte: u8) -> u8 {
    let ones = byte & 0b0000_1111;
    let tens = (byte & 0b1111_0000) >> 4;
    ones + tens * 10
}

const fn decode_bcd7b(byte: u8) -> u8 {
    let ones = byte & 0b0000_1111;
    let tens = (byte & 0b0111_0000) >> 4;
    ones + tens * 10
}

const fn decode_bcd6b(byte: u8) -> u8 {
    let ones = byte & 0b0000_1111;
    let tens = (byte & 0b0011_0000) >> 4;
    ones + tens * 10
}
