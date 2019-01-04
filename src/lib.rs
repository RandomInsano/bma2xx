//! Driver for the Bosch BMA222E and family
//! ==========================================
//! Currently this only support I2C configuration as I can't remove
//! the itty bitty package from its home on a populated PCB. If you
//! want to play with one, you can find them in Xfinity XR11 remote
//! controls. 😊
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
//! Specifically, these chips should work with the library now, but
//! you wouldn't benefit from the enhanced resolution.
//! 
//! More info on the product line from Bosch's website:
//! https://www.bosch-sensortec.com/bst/products/all_products/bma222e
//!
//! ### What's currently supported
//!
//! 1. Accessing the chip via I2C
//! 2. Reading X, Y, Z axis and whether they've updated since last poll
//! 3. Reading the event FIFO for X, Y, Z axis (32 element deep queue version of #1)
//! 4. Changing FIFO mode (stream, fifo, bypass) and reading how full the FIFO is
//! 5. Checking what data is in the EEPROM and how many writes it has left
//! 6. Software reset
//! 7. Reading temperature
//!
//! ### What's *not* currently supported
//!
//! If anything here seems meaningful to you, feel free to reach out
//! and help implement these features. I just didn't need any of them
//! personally.
//!
//! 1. Any chip other than the BMA222E
//! 2. Accessing the chip via SPI
//! 3. Changing which axis are thrown on the FIFO
//! 4. Enabling interrupts for slope, tap, FIFO full, orientation, etc
//! 5. Tap detection
//! 6. Acceleration data filters
//! 7. Power modes (normal, deep sleep, low power, suspend)
//! 8. Lower power sleep duration
//! 9. Whether to shadow acceleration data or not
//! 10. Configuration of the INT1 and INT2 pins nor their sources
//! 11. Interrupt thresholds for acceleration, slope, etc
//! 12. FIFO watermark interrupt level
//! 13. Self-test
//! 14. Actually programming the EEPROM or setting the values
//! 15. Setting the offset compensation
//! 16. Offset compensation
//! 17. And many more!
//!
//!
//! ### Example usage:
//! ```
//! fn main() {
//!     // Get an instance of an i2c struct here with one of the [device crates](https://github.com/rust-embedded/awesome-embedded-rust#device-crates).
//!
//!     let mut accel = Bma222e::new(i2c);
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

extern crate embedded_hal as hal;

use core::fmt;
use hal::blocking::i2c::{Write, WriteRead};

/// Create an instance of the accelerometer
pub struct Bma222e<I2C> {
    device: I2C,
}

/// The address on the bus. TODO: Support alt addresses
pub const ADDRESS: u8 = 0x19;

/// This identifier changes based on the product in the range.
pub const IDENTIFIER: u8 = 0xF8;

/// How long the NVM register bank is (differs between BMA222 and BMA222E)
const EEPROM_LENGTH: usize = 5;

const REG_CHIPID: u8 = 0x00;
const REG_XAXIS: u8 = 0x02;
const REG_YAXIS: u8 = 0x04;
const REG_ZAXIS: u8 = 0x06;
const REG_TEMPERATURE: u8 = 0x08;
const REG_FIFO_STATUS: u8 = 0x0E;
const REG_RESET: u8 = 0x14;
const REG_EEPROM_CONTROL: u8 = 0x33;
const REG_EEPROM_START: u8 = 0x38;
const REG_FIFO_CONFIG: u8 = 0x3E;
const REG_FIFO_DATA: u8 = 0x3F;

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

#[derive(Copy, Clone)]
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

impl<I2C, E> Bma222e<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Create a new instance for fun and profit!
    pub fn new(dev: I2C) -> Self {
        Self { device: dev }
    }

    /// Helper to load / unload buffers to send along
    fn write(&mut self, register: u8, data: &[u8]) -> Result<(), E> {
        let mut input = [0u8; 16]; // Can't dynamically size this, so 16 it is!
        assert!(
            data.len() < 16 - 1,
            "write() can only take buffers up to 15 bytes ☹️"
        );

        input[1..=data.len()].copy_from_slice(data);
        input[0] = register;

        self.device.write(ADDRESS, &input)?;

        Ok(())
    }

    /// Helper to load / unload buffers to send along
    fn read(&mut self, register: u8, data: &mut [u8]) -> Result<(), E> {
        self.device.write_read(ADDRESS, &[register], data)?;

        Ok(())
    }

    /// Helper to grab single byte registers
    fn single_read(&mut self, register: u8) -> Result<u8, E> {
        let mut out = [0u8; 1];
        self.read(register, &mut out)?;
        Ok(out[0])
    }

    /// Soft-reset the chip
    pub fn reset(&mut self) -> Result<(), E> {
        // Magic value for the reset is 0xB6
        self.write(ADDRESS, &[REG_RESET, 0xB6])?;
        Ok(())
    }

    /// Read the temperature of the chip
    pub fn temperature(&mut self) -> Result<u8, E> {
        let value = self.single_read(REG_TEMPERATURE)?;
        Ok(value.wrapping_add(23))
    }

    /// Return the chip's identifier. While there are many devices in this
    /// family, this crate was written for the BMA222E and you should
    /// compare this value to REG_CHIPID.
    pub fn who_am_i(&mut self) -> Result<u8, E> {
        Ok(self.single_read(REG_CHIPID)?)
    }

    /// Set the FIFO mode for events
    pub fn fifo_set_mode(&mut self, mode: FIFOConfig) -> Result<(), E> {
        let value = (mode as u8) << 6;

        self.write(REG_FIFO_CONFIG, &[value])?;

        Ok(())
    }

    /// How many events are stored in the FIFO
    pub fn fifo_size(&mut self) -> Result<u8, E> {
        let value = self.single_read(REG_FIFO_STATUS)?;
        Ok(value & 0b0111_1111)
    }

    /// Did the event FIFO overflow?
    pub fn fifo_overflow(&mut self) -> Result<bool, E> {
        let value = self.single_read(REG_FIFO_STATUS)?;
        Ok(value & 0b1000_0000 != 0)
    }

    /// Pull accelerometer data from the FIFO.
    pub fn fifo_get(&mut self, data: &mut [AxisData]) -> Result<(), E> {
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
            self.read(REG_FIFO_DATA, &mut out)?;

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

    /// Grab an axis and create an AxisData
    pub fn element_get(&mut self, register: u8) -> Result<AxisData, E> {
        let mut out = [0u8; 2];

        self.read(register, &mut out)?;

        let item = AxisData {
            value: u16::from(out[1]),
            changed: out[0] & 0x01 != 0,
        };

        Ok(item)
    }

    /// Grab data for X axis
    pub fn axis_x(&mut self) -> Result<AxisData, E> {
        Ok(self.element_get(REG_XAXIS)?)
    }

    /// Grab data for Y axis
    pub fn axis_y(&mut self) -> Result<AxisData, E> {
        Ok(self.element_get(REG_YAXIS)?)
    }

    /// Grab data for Z axis
    pub fn axis_z(&mut self) -> Result<AxisData, E> {
        Ok(self.element_get(REG_ZAXIS)?)
    }

    /// How many writes are left in the device's EEPROM. Each write will decrement
    /// this counter and the chip will deny writes at zero, so use any writes
    /// sparingly. The counter is only four bits wide (16 total writes)
    pub fn eeprom_writes_remaining(&mut self) -> Result<u8, E> {
        // The top four bits store the value. Using magic values here

        let value = self.single_read(REG_EEPROM_CONTROL)?;
        let value = (value | 0b1111_0000) >> 4;

        Ok(value)
    }

    /// Pull values from internal EEPROM
    pub fn eeprom_data(&mut self) -> Result<[u8; EEPROM_LENGTH], E> {
        let mut out = [0u8; EEPROM_LENGTH];
        self.read(REG_EEPROM_START, &mut out)?;

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
