//! A platform agnostic driver to interface with the L3GD20 (gyroscope)
//!
//! This driver was built using [`embedded-hal`] traits.
//!
//! [`embedded-hal`]: https://docs.rs/embedded-hal/~0.1
//!
//! # Examples
//!
//! You should find at least one example in the [f3] crate.
//!
//! [f3]: https://docs.rs/f3/~0.5

#![deny(missing_docs)]
#![deny(warnings)]
#![feature(unsize)]
#![no_std]

extern crate embedded_hal as hal;

use core::marker::Unsize;
use core::mem;

use hal::blocking::spi::{Transfer, Write};
use hal::spi::{Mode, Phase, Polarity};
use hal::digital::OutputPin;

/// SPI mode
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

/// L3GD20 driver
pub struct L3gd20<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS, E> L3gd20<SPI, CS>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    CS: OutputPin,
{
    /// Creates a new driver from a SPI peripheral and a NCS pin
    pub fn new(spi: SPI, cs: CS) -> Result<Self, E> {
        let mut l3gd20 = L3gd20 { spi, cs };

        // power up and enable all the axes
        l3gd20.write_register(Register::CTRL_REG1, 0b00_00_1_111)?;

        Ok(l3gd20)
    }

    /// Temperature measurement + gyroscope measurements
    pub fn all(&mut self) -> Result<Measurements, E> {
        let bytes: [u8; 9] = self.read_registers(Register::OUT_TEMP)?;

        Ok(Measurements {
            gyro: I16x3 {
                x: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
                y: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
                z: (bytes[7] as u16 + ((bytes[8] as u16) << 8)) as i16,
            },
            temp: bytes[1] as i8,
        })
    }

    /// Gyroscope measurements
    pub fn gyro(&mut self) -> Result<I16x3, E> {
        let bytes: [u8; 7] = self.read_registers(Register::OUT_X_L)?;

        Ok(I16x3 {
            x: (bytes[1] as u16 + ((bytes[2] as u16) << 8)) as i16,
            y: (bytes[3] as u16 + ((bytes[4] as u16) << 8)) as i16,
            z: (bytes[5] as u16 + ((bytes[6] as u16) << 8)) as i16,
        })
    }

    /// Temperature sensor measurement
    pub fn temp(&mut self) -> Result<i8, E> {
        Ok(self.read_register(Register::OUT_TEMP)? as i8)
    }

    /// Reads the WHO_AM_I register; should return `0xD4`
    pub fn who_am_i(&mut self) -> Result<u8, E> {
        self.read_register(Register::WHO_AM_I)
    }

    /// Read `STATUS_REG` of sensor
    pub fn status(&mut self) -> Result<Status, E> {
        let sts = self.read_register(Register::STATUS_REG)?;
        Ok(
            Status{
                overrun:   (sts & 1 << 7) != 0,
                z_overrun: (sts & 1 << 6) != 0,
                y_overrun: (sts & 1 << 5) != 0,
                x_overrun: (sts & 1 << 4) != 0,
                new_data:  (sts & 1 << 3) != 0,
                z_new:     (sts & 1 << 2) != 0,
                y_new:     (sts & 1 << 1) != 0,
                x_new:     (sts & 1 << 0) != 0,
            })
    }

    /// Get the current Output Data Rate
    pub fn odr(&mut self) -> Result<ODR, E> {
        // Read control register
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        // Extract ODR value, converting to enum (ROI: 0b1100_0000)
        let odr = match (reg1 >> 6) & 0x03 {
            x if x == ODR::Hz95  as u8 => ODR::Hz95,
            x if x == ODR::Hz190 as u8 => ODR::Hz190,
            x if x == ODR::Hz380 as u8 => ODR::Hz380,
            x if x == ODR::Hz760 as u8 => ODR::Hz760,
            _ => unreachable!(),
        };
        Ok(odr)
    }

    /// Set the Output Data Rate
    pub fn set_odr(&mut self, odr: ODR) -> Result<&mut Self, E> {
        // New configuration
        let bits = (odr as u8) << 6;
        // Mask to only affect ODR configuration
        let mask = 0b1100_0000;
        // Apply change
        self.change_config(Register::CTRL_REG1, mask, bits)
    }

    /// Get current Bandwidth
    pub fn bandwidth(&mut self) -> Result<Bandwidth, E> {
        let reg1 = self.read_register(Register::CTRL_REG1)?;
        // Shift and mask bandwidth of register, (ROI: 0b0011_0000)
        let bw = match (reg1 >> 4) & 0x03 {
            x if x == Bandwidth::Low     as u8 => Bandwidth::Low,
            x if x == Bandwidth::Medium  as u8 => Bandwidth::Medium,
            x if x == Bandwidth::High    as u8 => Bandwidth::High,
            x if x == Bandwidth::Maximum as u8 => Bandwidth::Maximum,
            _ => unreachable!(),
        };
        Ok(bw)
    }

    /// Set low-pass cut-off frequency (i.e. bandwidth)
    ///
    /// See `Bandwidth` for further explanation
    pub fn set_bandwidth(&mut self, bw: Bandwidth) -> Result<&mut Self, E> {
        let bits = (bw as u8) << 4;
        let mask = 0b0011_0000;
        self.change_config(Register::CTRL_REG1, mask, bits)
    }

    /// Get the current Full Scale Selection
    ///
    /// This is the sensitivity of the sensor, see `Scale` for more information
    pub fn scale(&mut self) -> Result<Scale, E> {
        let scl = self.read_register(Register::CTRL_REG4)?;
        // Extract scale value from register, ensure that we mask with
        // `0b0000_0011` to extract `FS1-FS2` part of register
        let scale = match (scl >> 4) & 0x03 {
            x if x == Scale::Dps250  as u8 => Scale::Dps250,
            x if x == Scale::Dps500  as u8 => Scale::Dps500,
            x if x == Scale::Dps2000 as u8 => Scale::Dps2000,
            // Special case for Dps2000
            0x02 => Scale::Dps2000,
            _ => unreachable!(),
        };
        Ok(scale)
    }

    /// Set the Full Scale Selection
    ///
    /// This sets the sensitivity of the sensor, see `Scale` for more
    /// information
    pub fn set_scale(&mut self, scale: Scale) -> Result<&mut Self, E> {
        let bits = (scale as u8) << 4;
        let mask = 0b0011_0000;
        self.change_config(Register::CTRL_REG4, mask, bits)
    }

    /// Get the current FIFO configuration
    ///
    /// # Note
    /// The queue may not be enabled, this method only returns the current
    /// configuration which doesn't say anything about the queue being enabled.
    pub fn fifo_config(&mut self) -> Result<FifoConfig, E> {
        let bits = self.read_register(Register::FIFO_CTRL_REG)?;
        let mode = match bits >> 5 & 0x07 {
            x if x == FifoMode::Bypass         as u8 => FifoMode::Bypass,
            x if x == FifoMode::Fifo           as u8 => FifoMode::Fifo,
            x if x == FifoMode::Stream         as u8 => FifoMode::Stream,
            x if x == FifoMode::StreamToFifo   as u8 => FifoMode::StreamToFifo,
            x if x == FifoMode::BypassToStream as u8 => FifoMode::BypassToStream,
            // TODO: Change panic to proper Error
            // Since not all combinations are used for `FifoMode` we _could_
            // end up in the default arm (which should not panic?)
            _ => panic!("Unknown 'FifoMode'"),
        };
        let lvl = bits & 0x1F;
        Ok(FifoConfig {
            mode: mode,
            watermark: lvl,
        })
    }

    /// Set the FIFO mode
    ///
    /// See `FifoMode` for values and meaning.
    pub fn set_fifo_mode(&mut self, mode: FifoMode) -> Result<&mut Self, E> {
        let bits = (mode as u8) << 5;
        let mask = 0b1110_0000;
        self.change_config(Register::FIFO_CTRL_REG, mask, bits)
    }

    /// Set the watermark threshold for the FIFO queue
    ///
    /// The threshold is used to create interrupts with FIFO status.
    ///
    /// # Note
    /// Only the bottom 5 bits are used so values above `32` are masked out.
    pub fn set_fifo_watermark(&mut self, threshold: u8) -> Result<&mut Self, E> {
        let bits = threshold & 0x1F;
        let mask = 0b0001_1111;
        self.change_config(Register::FIFO_CTRL_REG, mask, bits)
    }

    /// Get the current FIFO queue status
    ///
    /// See `FifoStatus` for more information
    pub fn fifo_status(&mut self) -> Result<FifoStatus, E> {
        let bits = self.read_register(Register::FIFO_SRC_REG)?;
        Ok(FifoStatus {
            above_watermark: bits & (1 << 7) != 0,
            overrun:         bits & (1 << 6) != 0,
            empty:           bits & (1 << 5) != 0,
            num_elements:    bits & 0x1F,
        })
    }

    /// Get the current configuration for `INT2`
    ///
    /// See `Int2Config` for more information.
    pub fn int2_config(&mut self) -> Result<Int2Config, E> {
        let bits = self.read_register(Register::CTRL_REG3)?;
        if bits & 0x0F == 0  {
            // If none of the lower 4 bits are active then we treat that as
            // not in use
            Ok(Int2Config::NotUsed)
        } else if bits & (1 << 3) != 0 {
            Ok(Int2Config::DataReady)
        } else {
            Ok(Int2Config::Fifo{
                watermark: bits & (1 << 2) != 0,
                overrun:   bits & (1 << 1) != 0,
                empty:     bits & (1 << 0) != 0,
            })
        }
    }

    /// Set the configuration for `INT2`
    ///
    /// See `Int2Config` for more information.
    pub fn set_int2_config(&mut self, cfg: Int2Config) -> Result<&mut Self, E> {
        let bits = match cfg {
            Int2Config::NotUsed => 0x00,
            Int2Config::DataReady => 1 << 3,
            Int2Config::Fifo {watermark, overrun, empty} => {
                let mut bits = 0x00;
                if watermark {
                    bits |= 1 << 2;
                }
                if overrun {
                    bits |= 1 << 1;
                }
                if empty {
                    bits |= 1 << 0;
                }
                bits
            },
        };
        let mask = 0b0000_1111;
        self.change_config(Register::CTRL_REG3, mask, bits)
    }

    /// Check if FIFO queue is enabled
    pub fn fifo_enabled(&mut self) -> Result<bool, E> {
        let bits = self.read_register(Register::CTRL_REG5)?;
        Ok(bits & (1 << 6) != 0)
    }

    /// Enable or disable FIFO
    ///
    /// See functions like `fifo_status`, `fifo_config`, `set_fifo_mode` and
    /// `set_fifo_watermark` for configuration of FIFO queue.
    pub fn enable_fifo(&mut self, enable: bool) -> Result<&mut Self, E> {
        let bits = if enable { 1 << 6 } else { 0x00 };
        let mask = 0b0100_0000;
        self.change_config(Register::CTRL_REG5, mask, bits)
    }

    /// Get current output selection configuration
    ///
    /// See `OutSel` for more information.
    pub fn out_sel(&mut self) -> Result<OutSel, E> {
        let bits = self.read_register(Register::CTRL_REG5)?;
        // Is the high pass filter enabled?
        let high_en = bits & (1 << 4) != 0;
        let mode = match bits & 0x03 {
            x if x == OutSel::LowPass  as u8  => OutSel::LowPass,
            x if x == OutSel::HighPass as u8 => OutSel::HighPass,
            // Full configuration is used, but return depends on status of the
            // high-pass filter
            x if x == OutSel::Full as u8 || x == OutSel::DoubleLow as u8 => {
                if high_en {
                    OutSel::Full
                } else {
                    OutSel::DoubleLow
                }
            }
            _ => unreachable!(),
        };
        Ok(mode)
    }

    /// Set output selection configuration
    ///
    /// See `OutSel` for more information.
    ///
    /// # Note
    /// If `OutSel::Full` configuration is chosen then the high-pass filter
    /// will be automatically enabled.
    pub fn set_out_sel(&mut self, selection: OutSel) -> Result<&mut Self, E> {
        let mut bits = selection as u8;
        match selection {
            OutSel::Full => bits |= 1 << 4,
            _ =>            bits &= !(1 << 4),
        }
        let mask = 0b0001_0011;
        self.change_config(Register::CTRL_REG5, mask, bits)
    }

    /// Get the current high-pass configuration
    ///
    /// See `HighPassConfig`, `HighPassMode` and `HighPassCutOff` for further
    /// information.
    pub fn highpass_config(&mut self) -> Result<HighPassConfig, E> {
        let bits = self.read_register(Register::CTRL_REG2)?;
        let mode = match (bits >> 4) & 0x03 {
            x if x == HighPassMode::NormalReset as u8 => HighPassMode::NormalReset,
            x if x == HighPassMode::Reference   as u8 => HighPassMode::Reference,
            x if x == HighPassMode::Normal      as u8 => HighPassMode::Normal,
            x if x == HighPassMode::AutoReset   as u8 => HighPassMode::AutoReset,
            _ => unreachable!(),
        };
        let cut_off = match bits & 0x0F {
            x if x == HighPassCutOff::CutOff0 as u8 => HighPassCutOff::CutOff0,
            x if x == HighPassCutOff::CutOff1 as u8 => HighPassCutOff::CutOff1,
            x if x == HighPassCutOff::CutOff2 as u8 => HighPassCutOff::CutOff2,
            x if x == HighPassCutOff::CutOff3 as u8 => HighPassCutOff::CutOff3,
            x if x == HighPassCutOff::CutOff4 as u8 => HighPassCutOff::CutOff4,
            x if x == HighPassCutOff::CutOff5 as u8 => HighPassCutOff::CutOff5,
            x if x == HighPassCutOff::CutOff6 as u8 => HighPassCutOff::CutOff6,
            x if x == HighPassCutOff::CutOff7 as u8 => HighPassCutOff::CutOff7,
            x if x == HighPassCutOff::CutOff8 as u8 => HighPassCutOff::CutOff8,
            x if x == HighPassCutOff::CutOff9 as u8 => HighPassCutOff::CutOff9,
            // TODO: Replace panic with proper Error!
            _ => panic!("Unknown 'HighPassCutOff' value"),
        };
        Ok(HighPassConfig {
            mode: mode,
            cut_off: cut_off,
        })
    }

    /// Set the high-pass mode
    ///
    /// See `HighPassMode` for more information.
    pub fn set_highpass_config(&mut self, mode: HighPassMode) -> Result<&mut Self, E> {
        let bits = (mode as u8) << 4;
        let mask = 0b0011_0000;
        self.change_config(Register::CTRL_REG2, mask, bits)
    }

    /// Set cut off frequency of the high-pass filter
    ///
    /// See `HighPassCutOff` for more information.
    pub fn set_highpass_cutoff(&mut self, cut: HighPassCutOff) -> Result<&mut Self, E> {
        let bits = cut as u8;
        let mask = 0b0000_1111;
        self.change_config(Register::CTRL_REG2, mask, bits)
    }

    fn read_register(&mut self, reg: Register) -> Result<u8, E> {
        self.cs.set_low();

        let mut buffer = [reg.addr() | SINGLE | READ, 0];
        self.spi.transfer(&mut buffer)?;

        self.cs.set_high();

        Ok(buffer[1])
    }

    fn read_registers<B>(&mut self, reg: Register) -> Result<B, E>
    where
        B: Unsize<[u8]>,
    {
        self.cs.set_low();

        let mut buffer: B = unsafe { mem::uninitialized() };
        {
            let slice: &mut [u8] = &mut buffer;
            slice[0] = reg.addr() | MULTI | READ;
            self.spi.transfer(slice)?;
        }

        self.cs.set_high();

        Ok(buffer)
    }

    fn write_register(
        &mut self,
        reg: Register,
        byte: u8,
    ) -> Result<(), E> {
        self.cs.set_low();

        let buffer = [reg.addr() | SINGLE | WRITE, byte];
        self.spi.write(&buffer)?;

        self.cs.set_high();

        Ok(())
    }

    /// Change configuration in register
    ///
    /// Helper function to update a particular part of a register without
    /// affecting other parts of the register that might contain desired
    /// configuration. This allows the `L3gd20` struct to be used like
    /// a builder interface when configuring specific parameters.
    fn change_config(&mut self, reg: Register, mask: u8, new_value: u8) -> Result<&mut Self, E> {
        // Read current value of register
        let current = self.read_register(reg)?;
        // Use supplied mask so we don't affect more than necessary
        let masked  = current & !mask;
        // Use `or` to apply the new value without affecting other parts
        let new_reg = masked | new_value;
        self.write_register(reg, new_reg)?;
        Ok(self)
    }
}

#[allow(dead_code)]
#[allow(non_camel_case_types)]
#[derive(Clone, Copy)]
enum Register {
    WHO_AM_I = 0x0F,
    CTRL_REG1 = 0x20,
    CTRL_REG2 = 0x21,
    CTRL_REG3 = 0x22,
    CTRL_REG4 = 0x23,
    CTRL_REG5 = 0x24,
    REFERENCE = 0x25,
    OUT_TEMP = 0x26,
    STATUS_REG = 0x27,
    OUT_X_L = 0x28,
    OUT_X_H = 0x29,
    OUT_Y_L = 0x2A,
    OUT_Y_H = 0x2B,
    OUT_Z_L = 0x2C,
    OUT_Z_H = 0x2D,
    FIFO_CTRL_REG = 0x2E,
    FIFO_SRC_REG = 0x2F,
    INT1_CFG = 0x30,
    INT1_SRC = 0x31,
    INT1_TSH_XH = 0x32,
    INT1_TSH_XL = 0x33,
    INT1_TSH_YH = 0x34,
    INT1_TSH_YL = 0x35,
    INT1_TSH_ZH = 0x36,
    INT1_TSH_ZL = 0x37,
    INT1_DURATION = 0x38,
}

/// Output Data Rate
#[derive(Debug, Clone, Copy)]
pub enum ODR {
    /// 95 Hz data rate
    Hz95  = 0x00,
    /// 190 Hz data rate
    Hz190 = 0x01,
    /// 380 Hz data rate
    Hz380 = 0x02,
    /// 760 Hz data rate
    Hz760 = 0x03,
}

/// Full scale selection
#[derive(Debug, Clone, Copy)]
pub enum Scale {
    /// 250 Degrees Per Second
    Dps250  = 0x00,
    /// 500 Degrees Per Second
    Dps500  = 0x01,
    /// 2000 Degrees Per Second
    Dps2000 = 0x03,
}

/// Bandwidth of sensor
///
/// The bandwidth of the sensor is equal to the cut-off for the low-pass
/// filter. The cut-off depends on the `ODR` of the sensor, for specific
/// information consult the data sheet (table 21).
#[derive(Debug, Clone, Copy)]
pub enum Bandwidth {
    /// Lowest possible cut-off for any `ODR` configuration
    Low     = 0x00,
    /// Medium cut-off, can be the same as `High` for some `ODR` configurations
    Medium  = 0x01,
    /// High cut-off
    High    = 0x02,
    /// Maximum cut-off for any `ODR` configuration
    Maximum = 0x03,
}

/// Configuration mode for device FIFO queue
#[derive(Debug, Clone, Copy)]
pub enum FifoMode {
    /// Bypass mode, FIFO queue not in use
    ///
    /// Only the first register is written to and when new data is available
    /// it overwrites the old data.
    Bypass         = 0x00,
    /// Place all measurements into FIFO queue
    ///
    /// Once the queue is full no further measurements are added and
    /// the queue has to be reset by changing the mode back to `Bypass`.
    Fifo           = 0x01,
    /// Place, and replace, all measurements into FIFO queue
    ///
    /// Once the queue is full older data will be evacuated in favour of
    /// new measurements.
    Stream         = 0x02,
    /// Set mode to `Stream` until trigger event, then `Fifo` mode
    ///
    /// The queue will first be operated the same way as `Stream` mode, when
    /// an interrupt is triggered on `INT1_CFG` the mode changes to `Fifo`.
    StreamToFifo   = 0x03,
    /// Set mode to `Bypass` until trigger event, then `Stream` mode
    ///
    /// The queue will first be operated the same way as `Bypass` mode, when
    /// an interrupt is triggered on `INT1_CFG` the mode changes to `Stream`.
    BypassToStream = 0x04,
}

/// Interrupt line 2 configuration
#[derive(Debug, Clone, Copy)]
pub enum Int2Config {
    /// Interrupt on Data Ready
    ///
    /// In this mode `INT2` is used to notify when data is ready. This is most
    /// useful in `FifoMode::Bypass` to simply be notified every time new
    /// data is available to be read.
    DataReady,
    /// Interrupt on FIFO configuration
    ///
    /// In this mode `INT2` is used to notify about FIFO events. The interrupt
    /// can be used not notify when FIFO queue is above `watermark`, if the
    /// queue has been `overrun` and/or if the queue is `empty`.
    Fifo {
        /// Create interrupt when FIFO watermark is exceed
        watermark: bool,
        /// Create interrupt when FIFO is overrun
        overrun: bool,
        /// Create interrupt when FIFO is empty
        empty: bool,
    },
    /// There is no active configuration for `INT2`
    NotUsed,
}

/// Output selection
///
/// Output selection decides how data is filtered before it is placed in
/// output registers (or FIFO queue). Refer to Figure 18 of data sheet.
#[derive(Debug, Clone, Copy)]
pub enum OutSel {
    /// Data is only passed through low-pass filter
    LowPass   = 0x00,
    /// Data is first passed through low-pass filter then a high-pass filter
    HighPass  = 0x01,
    /// Data is passed through two low-pass filters
    DoubleLow = 0x02,
    /// Data is passed through low-pass filter, then high-pass filter then
    /// another low-pass filter
    Full      = 0x03,
}

/// High-pass filter mode
///
/// See table 25 in data sheet for further information.
#[derive(Debug, Clone, Copy)]
pub enum HighPassMode {
    // TODO: Is there any better way to explain the modes below?
    /// Normal mode (reset reading `HP_RESET_FILTER`)
    NormalReset = 0x00,
    /// Reference signal for filtering
    Reference   = 0x01,
    /// Normal mode
    Normal      = 0x02,
    /// Auto reset on interrupt event
    AutoReset   = 0x03,
}

/// High-pass filter cut off frequency
///
/// The cut off is dependent on `ODR` so refer to table 26 of the data sheet
/// for actual numbers.
#[allow(missing_docs)]
#[derive(Debug, Clone, Copy)]
pub enum HighPassCutOff {
    CutOff0 = 0x00,
    CutOff1 = 0x01,
    CutOff2 = 0x02,
    CutOff3 = 0x03,
    CutOff4 = 0x04,
    CutOff5 = 0x05,
    CutOff6 = 0x06,
    CutOff7 = 0x07,
    CutOff8 = 0x08,
    CutOff9 = 0x09,
}

const READ: u8 = 1 << 7;
const WRITE: u8 = 0 << 7;
const MULTI: u8 = 1 << 6;
const SINGLE: u8 = 0 << 6;

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}

impl Scale {
    /// Convert a measurement to degrees
    pub fn degrees(&self, val: i16) -> f32 {
        match *self {
            Scale::Dps250  => val as f32 * 0.00875,
            Scale::Dps500  => val as f32 * 0.0175,
            Scale::Dps2000 => val as f32 * 0.07,
        }
    }

    /// Convert a measurement to radians
    pub fn radians(&self, val: i16) -> f32 {
        // TODO: Use `to_radians` or other built in method
        // NOTE: `to_radians` is only exported in `std` (07.02.18)
        self.degrees(val) * (core::f32::consts::PI / 180.0)
    }
}

/// XYZ triple
#[derive(Debug)]
pub struct I16x3 {
    /// X component
    pub x: i16,
    /// Y component
    pub y: i16,
    /// Z component
    pub z: i16,
}

/// Several measurements
#[derive(Debug)]
pub struct Measurements {
    /// Gyroscope measurements
    pub gyro: I16x3,
    /// Temperature sensor measurement
    pub temp: i8,
}

/// Sensor status
#[derive(Debug, Clone, Copy)]
pub struct Status {
    /// Overrun (data has overwritten previously unread data)
    /// has occurred on at least one axis
    pub overrun: bool,
    /// Overrun occurred on Z-axis
    pub z_overrun: bool,
    /// Overrun occurred on Y-axis
    pub y_overrun: bool,
    /// Overrun occurred on X-axis
    pub x_overrun: bool,
    /// New data is available for either X, Y, Z - axis
    pub new_data: bool,
    /// New data is available on Z-axis
    pub z_new: bool,
    /// New data is available on Y-axis
    pub y_new: bool,
    /// New data is available on X-axis
    pub x_new: bool,
}

/// Status of device FIFO
#[derive(Debug, Clone, Copy)]
pub struct FifoStatus {
    /// Does the FIFO queue have more elements than the watermark?
    pub above_watermark: bool,
    /// Is the queue completely filled?
    pub overrun: bool,
    /// Is the queue empty?
    pub empty: bool,
    /// How many elements are stored in the queue
    ///
    /// Maximum queue size is 32
    pub num_elements: u8,
}

/// FIFO configuration for device
#[derive(Debug, Clone, Copy)]
pub struct FifoConfig {
    /// Operating mode of FIFO queue
    pub mode: FifoMode,
    /// Watermark threshold level
    ///
    /// # Note
    /// Only the bottom 5 bits will be considered, meaning a maximum value
    /// of `32`.
    pub watermark: u8,
}

/// Configuration of the High-pass filter
#[derive(Debug, Clone, Copy)]
pub struct HighPassConfig {
    /// Mode selection for High-pass filter
    pub mode: HighPassMode,
    /// Cut off frequency selection for High-pass filter
    pub cut_off: HighPassCutOff,
}
