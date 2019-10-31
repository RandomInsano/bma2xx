//! Driver for the Bosch BMA222E and family
//! ==========================================
//! Currently, the BMA222E and BMA253 are explicitly supported. Other chips can
//! be supported fairly easily by implementing the Bma2xx trait with the correct
//! values from the related data sheet.
//!
//! The register maps in this family are very similar and so it
//! should be possible to support the following chips with minimal
//! changes to the code here:
//!
//! * BMA222 - 8bit resolution
//! * BMA250 - 10bit resolution
//! * BMA253 - 12bit resolution
//! * BMA255 - 12bit resolution
//! * BMA280 - 14bit resolution
//!
//! More info on the product line from Bosch's website:
//! https://www.bosch-sensortec.com/bst/products/all_products/bma222e
//!
//! ### What's currently supported
//!
//! 1. Accessing the chip via I2C and via SPI
//! 2. Reading X, Y, Z axis and whether they've updated since last poll
//! 3. Reading the event FIFO for X, Y, Z axis (32 element deep queue version of #1)
//! 4. Changing FIFO mode (stream, fifo, bypass) and reading how full the FIFO is
//! 5. Checking what data is in the EEPROM and how many writes it has left
//! 6. Software reset
//! 7. Reading temperature
//! 8. Tap detection
//!
//! ### What's *not* currently supported
//!
//! If anything here seems meaningful to you, feel free to reach out
//! and help implement these features. I just didn't need any of them
//! personally.
//!
//! 1. Any chip other than the BMA222E and BMA253
//! 2. Changing which axis are thrown on the FIFO
//! 3. Enabling interrupts for slope, tap, FIFO full, orientation, etc
//! 4. Acceleration data filters
//! 5. Power modes (normal, deep sleep, low power, suspend)
//! 6. Lower power sleep duration
//! 7. Whether to shadow acceleration data or not
//! 8. Configuration of the INT1 and INT2 pins nor their sources
//! 9. Interrupt thresholds for acceleration, slope, etc
//! 10. FIFO watermark interrupt level
//! 11. Self-test
//! 12. Actually programming the EEPROM or setting the values
//! 13. Setting the offset compensation
//! 14. Offset compensation
//! 15. And many more!
//!
//!
//! ### Example usage:
//! ```
//! fn main() {
//!     // Get an instance of an i2c struct here with one of the [device crates](https://github.com/rust-embedded/awesome-embedded-rust#device-crates).
//!
//!     use bma2xx::{interface, Accelerometer, Bma222e};
//!     let interface = interface::I2CInterface(i2c, interface::I2C_DEFAULT_ADDRESS);
//!     let mut accel = Accelerometer::new(interface, Bma222e{});
//!
//!     accel.reset().unwrap();
//!
//!     examples(accel).unwrap();
//! }
//!
//! fn examples(accel: Bma222e) -> Result<(), ()> {
//!     let chip_id = accel.who_am_i()?;
//!     println!("About to Begin. ID is {} and should be {}", chip_id, bma222e::IDENTIFIER)?;
//!
//!     let temp = accel.temperature()?;
//!     println!("Temperature: {}", temp)?;
//!
//!     let writes_remaining = accel.eeprom_writes_remaining()?;
//!     println!("EEPROM Writes Remaining: {}", writes_remaining)?;
//!
//!     let nvm_data = accel.eeprom_data()?;
//!     hprint!("EEPROM Data: ")?;
//!     for byte in &nvm_data {
//!         hprint!("{:02X} ", byte)?;
//!     }
//!     println!()?;
//!
//!
//!     // FIFO-related goodness!
//!     // TODO: Find out why the output data wasn't consistent
//!     let fifo_size = accel.fifo_size()?;
//!     println!("Events in the FIFO: {}", fifo_size)?;
//!     let mut events = [AxisData {value: 0, changed: false}; 3];
//!     accel.fifo_set_mode(FIFOConfig::BYPASS)?;
//!     accel.fifo_get(&mut events)?;
//!
//!     let x = accel.axis_x()?;
//!     let y = accel.axis_y()?;
//!     let z = accel.axis_z()?;
//!     println!("X:{0:03} Y:{1:03} Z:{2:03}",
//!         x.value,
//!         y.value,
//!         z.value)?;
//!
//!     Ok(())    
//! }
//! ```

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]

extern crate bitfield;
extern crate embedded_hal as hal;

pub mod interface;
pub mod register;

use core::fmt;
use crate::interface::DigitalInterface;
use crate::register::Reg;
use bitfield::{Bit, BitRange};

const MAX_NVM_LENGTH: usize = 8;
const RESET_MAGIC_VALUE: u8 = 0xb6;

/// Implements chip-specific values.
pub trait Bma2xx {
    /// The chip ID that should be the result of read(Reg::BGW_CHIPID).
    const CHIPID: u8;
    /// The number of registers belonging to the non-volatile memory.
    const NVM_LENGTH: usize;
    /// The first register belonging to the non-volatile memory.
    const NVM_START: Reg;
    /// The width in bits of the acceleration data.
    const DATA_WIDTH: usize;
}

/// The BMA253 digital triaxial acceleration sensor.
pub struct Bma253 {}

impl Bma2xx for Bma253 {
    const CHIPID: u8 = 0xfa;
    const NVM_LENGTH: usize = 5;
    const NVM_START: Reg = Reg::OFC_OFFSET_X;
    const DATA_WIDTH: usize = 12;
}

/// The BMA222E digital triaxial acceleration sensor.
pub struct Bma222e {}

impl Bma2xx for Bma222e {
    const CHIPID: u8 = 0xf8;
    const NVM_LENGTH: usize = 5;
    const NVM_START: Reg = Reg::OFC_OFFSET_X;
    const DATA_WIDTH: usize = 8;
}

/// Extension trait for a typical BMA2XX-like accelerometer.
pub trait AccelerometerExt {
    /// An error that can be thrown by the implementation
    type Error;

    /// Soft-reset the chip
    fn reset(&mut self) -> Result<(), Self::Error>;

    /// Read the temperature of the chip and return it in half Kelvins above 23
    /// degrees Celsius.
    fn temperature(&mut self) -> Result<u8, Self::Error>;

    /// Return the chip's identifier. While there are many devices in this
    /// family, this crate was written for the BMA222E and you should
    /// compare this value to REG_CHIPID.
    fn who_am_i(&mut self) -> Result<u8, Self::Error>;

    /// Set the accelerometer g-range.
    fn set_range(&mut self, range: RangeConfig) -> Result<(), Self::Error>;

    /// Configure tap sensing.
    fn configure_tap_sensing(&mut self, cfg: TapSensingConfig) -> Result<(), Self::Error>;

    /// Set the FIFO mode for events
    fn fifo_set_mode(&mut self, mode: FIFOConfig) -> Result<(), Self::Error>;

    /// How many events are stored in the FIFO
    fn fifo_size(&mut self) -> Result<u8, Self::Error>;

    /// Did the event FIFO overflow?
    fn fifo_overflow(&mut self) -> Result<bool, Self::Error>;

    /// Pull accelerometer data from the FIFO.
    fn fifo_get(&mut self, data: &mut [AxisData]) -> Result<(), Self::Error>;

    /// Grab an axis and create an AxisData
    fn element_get(&mut self, reg: Reg) -> Result<AxisData, Self::Error>;

    /// Grab data for X axis
    fn axis_x(&mut self) -> Result<AxisData, Self::Error>;

    /// Grab data for Y axis
    fn axis_y(&mut self) -> Result<AxisData, Self::Error>;

    /// Grab data for Z axis
    fn axis_z(&mut self) -> Result<AxisData, Self::Error>;

    /// How many writes are left in the device's EEPROM. Each write will decrement
    /// this counter and the chip will deny writes at zero, so use any writes
    /// sparingly. The counter is only four bits wide (16 total writes)
    fn eeprom_writes_remaining(&mut self) -> Result<u8, Self::Error>;

    /// Pull values from internal EEPROM. Due to limitations in Rust, this always returns an array
    /// of length MAX_NVM_LENGTH even though some implementations use only a part of it.
    fn eeprom_data(&mut self) -> Result<[u8; MAX_NVM_LENGTH], Self::Error>;
}

/// A usable BMA2xx accelerometer.
pub struct Accelerometer<DigitalInterface, Bma2xx> {
    interface: DigitalInterface,
    _device: Bma2xx,
}

impl<I, B> Accelerometer<I, B> {
    /// Combine a digital interface and a device specification to create a
    /// usable accelerometer.
    pub fn new(interface: I, device: B) -> Accelerometer<I, B> {
        Accelerometer {
            interface: interface,
            _device: device,
        }
    }
}

/// Various FIFO operating modes
pub enum FIFOConfig {
    /// Don't use the FIFO. Reads from the FIFO are the immediate value
    BYPASS = 0,
    /// Collect readings and don't drop readings if buffer is full
    FIFO = 1,
    /// Collect readings but discard the oldest
    STREAM = 2,
}

// TODO: This need to go into FIFO_CONFIG
/*
pub enum FIFOAxis {
    ALL = 0,
    X = 1,
    Y = 2,
    Z = 3
}
*/

#[derive(Copy, Clone, Debug)]
/// Holds accelerometer data
pub struct AxisData {
    /// What the value was when the accelerometer was sampled
    pub value: u16,
    /// Whether the data has changed since the last sample
    pub changed: bool,
}

impl fmt::Display for AxisData {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Value: {} Changed?: {}", self.value, self.changed)
    }
}

#[allow(missing_docs)]
#[derive(Copy, Clone, Debug)]
pub enum TapShockDuration {
    _50ms = 0,
    _75ms = 1,
}

impl From<TapShockDuration> for bool {
    fn from(d: TapShockDuration) -> bool {
        match d {
            TapShockDuration::_50ms => false,
            TapShockDuration::_75ms => true,
        }
    }
}

#[allow(missing_docs)]
#[derive(Copy, Clone, Debug)]
pub enum TapQuietDuration {
    _30ms = 0,
    _20ms = 1,
}

impl From<TapQuietDuration> for bool {
    fn from(d: TapQuietDuration) -> bool {
        match d {
            TapQuietDuration::_30ms => false,
            TapQuietDuration::_20ms => true,
        }
    }
}

#[allow(missing_docs)]
#[derive(Copy, Clone, Debug)]
pub enum DoubleTapDuration {
    _50ms = 0,
    _100ms = 1,
    _150ms = 2,
    _200ms = 3,
    _250ms = 4,
    _375ms = 5,
    _500ms = 6,
    _700ms = 7,
}

#[allow(missing_docs)]
#[derive(Copy, Clone, Debug)]
pub enum TapWakeupSamples {
    _2samples = 0,
    _4samples = 1,
    _8samples = 2,
    _16samples = 3,
}

#[allow(missing_docs)]
#[derive(Copy, Clone, Debug)]
pub enum RangeConfig {
    _2g = 3,
    _4g = 5,
    _8g = 8,
    _16g = 12,
}

/// Configuration for tap sensing.
#[derive(Copy, Clone, Debug)]
pub struct TapSensingConfig {
    /// Whether interrupts for a single tap are enabled.
    pub single_tap_enabled: bool,
    /// Whether interrupts for a double tap are enabled.
    pub double_tap_enabled: bool,
    /// Acceleration needed to register as a tap, in units of range/32.
    pub tap_threshold: u8,
    /// The number of samples to process after a low-power mode wake-up.
    pub tap_wakeup_samples: TapWakeupSamples,
    /// The maximum duration the shock can last for it to register as a tap.
    pub tap_shock_duration: TapShockDuration,
    /// The minimum time for the acceleration to stay quiet after the shock for
    /// a tap to register.
    pub tap_quiet_duration: TapQuietDuration,
    /// The maximum time between taps to register as a double tap.
    pub double_tap_duration: DoubleTapDuration,
    /// Whether to map single tap interrupts to interrupt pin 1.
    pub map_single_to_int1: bool,
    /// Whether to map double tap interrupts to interrupt pin 1.
    pub map_double_to_int1: bool,
    /// Whether to map single tap interrupts to interrupt pin 2.
    pub map_single_to_int2: bool,
    /// Whether to map double tap interrupts to interrupt pin 2.
    pub map_double_to_int2: bool,
}

impl Default for TapSensingConfig {
    fn default() -> TapSensingConfig {
        TapSensingConfig {
            single_tap_enabled: false,
            double_tap_enabled: false,
            tap_threshold: 10,
            tap_wakeup_samples: TapWakeupSamples::_2samples,
            tap_shock_duration: TapShockDuration::_50ms,
            tap_quiet_duration: TapQuietDuration::_30ms,
            double_tap_duration: DoubleTapDuration::_250ms,
            map_single_to_int1: false,
            map_double_to_int1: false,
            map_single_to_int2: false,
            map_double_to_int2: false,
        }
    }
}

impl<I, E, B> AccelerometerExt for Accelerometer<I, B>
where
    I: DigitalInterface<Error = E>,
    B: Bma2xx,
{
    type Error = E;

    fn reset(&mut self) -> Result<(), E> {
        self.interface.write(Reg::BGW_SOFTRESET, RESET_MAGIC_VALUE)
    }

    fn temperature(&mut self) -> Result<u8, E> {
        let value = self.interface.read(Reg::ACCD_TEMP)?;
        Ok(value)
    }

    fn who_am_i(&mut self) -> Result<u8, E> {
        Ok(self.interface.read(Reg::BGW_CHIPID)?)
    }

    fn set_range(&mut self, range: RangeConfig) -> Result<(), Self::Error> {
        let mut pmu_range = 0;
        pmu_range.set_bit_range(0, 3, range as u8);
        self.interface.write(Reg::PMU_RANGE, pmu_range)
    }

    fn configure_tap_sensing(&mut self, cfg: TapSensingConfig) -> Result<(), E> {
        let mut int_en_0 = self.interface.read(Reg::INT_EN_0)?;
        int_en_0.set_bit(4, cfg.double_tap_enabled);
        int_en_0.set_bit(5, cfg.single_tap_enabled);
        self.interface.write(Reg::INT_EN_0, int_en_0)?;
        let mut int_8 = 0u8;
        int_8.set_bit_range(0, 2, cfg.double_tap_duration as u8);
        int_8.set_bit(6, cfg.tap_shock_duration.into());
        int_8.set_bit(7, cfg.tap_quiet_duration.into());
        self.interface.write(Reg::INT_8, int_8)?;
        let mut int_9 = 0u8;
        int_9.set_bit_range(0, 4, cfg.tap_threshold & 0x1f);
        int_9.set_bit_range(6, 7, cfg.tap_wakeup_samples as u8);
        self.interface.write(Reg::INT_9, int_9)?;
        let mut int_map_0 = self.interface.read(Reg::INT_MAP_0)?;
        int_map_0.set_bit(4, cfg.map_double_to_int1);
        int_map_0.set_bit(5, cfg.map_single_to_int1);
        self.interface.write(Reg::INT_MAP_0, int_map_0)?;
        let mut int_map_2 = self.interface.read(Reg::INT_MAP_2)?;
        int_map_2.set_bit(4, cfg.map_double_to_int2);
        int_map_2.set_bit(5, cfg.map_single_to_int2);
        self.interface.write(Reg::INT_MAP_2, int_map_2)
    }

    fn fifo_set_mode(&mut self, mode: FIFOConfig) -> Result<(), E> {
        self.interface.write(Reg::FIFO_CONFIG_1, (mode as u8) << 6)
    }

    fn fifo_size(&mut self) -> Result<u8, E> {
        let value = self.interface.read(Reg::FIFO_STATUS)?;
        Ok(value & 0b0111_1111)
    }

    fn fifo_overflow(&mut self) -> Result<bool, E> {
        let value = self.interface.read(Reg::FIFO_STATUS)?;
        Ok(value & 0b1000_0000 != 0)
    }

    fn fifo_get(&mut self, data: &mut [AxisData]) -> Result<(), E> {
        // Implementation detail here: Ideally you would use a longer read buffer
        // to avoid register selection overhead on I2C, but we can't dynamically
        // size the array length in Rust yet (as of 1.32)
        let mut out = [0u8; 6];

        // TODO: Other chips in this family have 6, 10 or 12 bits of accuracy. This
        // needs to change to support the least significant bits existing in the
        // second byte
        // TODO: Support different number of X, Y, and Z elements
        let mut index: usize = 0;
        for item in data {
            self.interface.read_multiple(Reg::FIFO_DATA, &mut out)?;

            item.changed = out[index] & 0x01 != 0;
            item.value = u16::from(out[index + 1]);

            if index == 3 {
                index = 0;
            } else {
                index += 1;
            }
        }

        Ok(())
    }

    fn element_get(&mut self, reg: Reg) -> Result<AxisData, E> {
        let mut out = [0u8; 2];

        self.interface.read_multiple(reg, &mut out)?;

        let unmasked = (out[1] as u16) << 8 | out[0] as u16;
        let mask = 0xffffu16 << (16 - B::DATA_WIDTH);
        let item = AxisData {
            value: unmasked & mask,
            changed: out[0] & 0x01 != 0,
        };

        Ok(item)
    }

    fn axis_x(&mut self) -> Result<AxisData, E> {
        Ok(self.element_get(Reg::ACCD_X_LSB)?)
    }

    fn axis_y(&mut self) -> Result<AxisData, E> {
        Ok(self.element_get(Reg::ACCD_Y_LSB)?)
    }

    fn axis_z(&mut self) -> Result<AxisData, E> {
        Ok(self.element_get(Reg::ACCD_Z_LSB)?)
    }

    fn eeprom_writes_remaining(&mut self) -> Result<u8, E> {
        // The top four bits store the value. Using magic values here

        let value = self.interface.read(Reg::TRIM_NVM_CTRL)?;
        let value = (value | 0b1111_0000) >> 4;

        Ok(value)
    }

    fn eeprom_data(&mut self) -> Result<[u8; MAX_NVM_LENGTH], E> {
        let mut out = [0u8; MAX_NVM_LENGTH];
        self.interface
            .read_multiple(B::NVM_START, &mut out[..B::NVM_LENGTH])?;

        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(1 + 1, 2);
    }
}
