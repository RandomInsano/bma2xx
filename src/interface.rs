//! This module defines a trait for interfacing with the registers on a bma2xx, as well as
//! implementations for both I2C and SPI.
use crate::register::Reg;
use hal::blocking::{i2c, spi};
use hal::digital::v2::OutputPin;

/// Represents a digital interface with a bma2xx.
pub trait DigitalInterface {
    /// The type of errors the underlying digital interface generates.
    type Error;
    /// Read data.len() bytes from register into data.
    fn read_multiple(&mut self, register: Reg, data: &mut [u8]) -> Result<(), Self::Error>;
    /// Write data.len() bytes in data to a register.
    fn write_multiple(&mut self, register: Reg, data: &[u8]) -> Result<(), Self::Error>;
    /// Read a single byte from a register.
    fn read(&mut self, register: Reg) -> Result<u8, Self::Error>;
    /// Write a singly byte to a register.
    fn write(&mut self, register: Reg, data: u8) -> Result<(), Self::Error>;
}

/// Default I2C device address when SDO pin is pulled to GND.
pub const I2C_DEFAULT_ADDRESS: u8 = 0x18;

/// Alternate I2C device address when SDO pin is pulled to VDDIO.
pub const I2C_ALTERNATE_ADDRESS: u8 = 0x19;

/// Represents digital interface with a bma2xx device over I2C.
pub struct I2CInterface<I2C> {
    device: I2C,
    address: u8,
}

impl<I2C, E> I2CInterface<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    /// Create a new digital interface based on an I2C device.
    pub fn new(device: I2C, address: u8) -> I2CInterface<I2C> {
        I2CInterface {
            device: device,
            address: address,
        }
    }
}

impl<I2C, E> DigitalInterface for I2CInterface<I2C>
where
    I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
{
    type Error = E;

    fn read_multiple(&mut self, register: Reg, data: &mut [u8]) -> Result<(), E> {
        self.device
            .write_read(self.address, &[register as u8], data)
    }

    fn write_multiple(&mut self, register: Reg, data: &[u8]) -> Result<(), E> {
        let mut input = [0u8; 16]; // Can't dynamically size this, so 16 it is!
        assert!(
            data.len() < 16 - 1,
            "write_multiple() can only take buffers up to 15 bytes"
        );

        input[1..=data.len()].copy_from_slice(data);
        input[0] = register as u8;

        self.device.write(self.address, &input)?;

        Ok(())
    }

    #[inline]
    fn read(&mut self, register: Reg) -> Result<u8, E> {
        let mut out = [0];
        self.device
            .write_read(self.address, &[register as u8], &mut out)?;
        Ok(out[0])
    }

    #[inline]
    fn write(&mut self, register: Reg, data: u8) -> Result<(), E> {
        self.device.write(self.address, &[register as u8, data])
    }
}

const SPI_READ_BIT: u8 = 0x80;
const SPI_WRITE_MASK: u8 = 0x7f;

/// An error caused by the SPI digital interface.
#[derive(Debug, Copy, Clone)]
pub enum SPIError<E, E2> {
    /// SPI bus I/O error
    BusError(E),
    /// Error setting the nCS pin
    NCSError(E2),
}

/// Represents digital interface with a bma2xx device over SPI.
pub struct SPIInterface<SPI, NSS> {
    device: SPI,
    nss: NSS,
}

impl<SPI, NSS, E, EO> SPIInterface<SPI, NSS>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    NSS: OutputPin<Error = EO>,
{
    /// Create a new digital interface based on a SPI device and a
    /// notSlaveSelect OutputPin.
    pub fn new(device: SPI, nss: NSS) -> Result<SPIInterface<SPI, NSS>, SPIError<E, EO>> {
        let mut result = SPIInterface {
            device: device,
            nss: nss,
        };

        result.nss.set_high().map_err(SPIError::NCSError)?;
        Ok(result)
    }

    #[inline]
    fn transfer<'w>(&mut self, buffer: &'w mut [u8]) -> Result<&'w [u8], SPIError<E, EO>> {
        self.nss.set_low().map_err(SPIError::NCSError)?;
        let result = self.device.transfer(buffer).map_err(SPIError::BusError);
        self.nss.set_high().map_err(SPIError::NCSError)?;
        result
    }

    #[inline]
    fn write(&mut self, buffer: &[u8]) -> Result<(), SPIError<E, EO>> {
        self.nss.set_low().map_err(SPIError::NCSError)?;
        let result = self.device.write(buffer).map_err(SPIError::BusError);
        self.nss.set_high().map_err(SPIError::NCSError)?;
        result
    }
}

impl<SPI, NSS, E, EO> DigitalInterface for SPIInterface<SPI, NSS>
where
    SPI: spi::Transfer<u8, Error = E> + spi::Write<u8, Error = E>,
    NSS: OutputPin<Error = EO>,
{
    type Error = SPIError<E, EO>;

    fn write_multiple(&mut self, register: Reg, data: &[u8]) -> Result<(), Self::Error> {
        let mut input = [0; 16]; // Can't dynamically size this, so 16 it is!
        assert!(
            data.len() < 16 - 1,
            "write_multiple() can only take buffers up to 15 bytes"
        );

        input[1..data.len() + 1].copy_from_slice(data);
        input[0] = register as u8 & SPI_WRITE_MASK;

        self.write(&input[..data.len() + 1])?;

        Ok(())
    }

    fn read_multiple(&mut self, reg: Reg, data: &mut [u8]) -> Result<(), Self::Error> {
        let mut buf = [0; 16]; // Can't dynamically size this, so 16 it is!
        assert!(
            data.len() < 16 - 1,
            "read_multiple() can only take buffers up to 15 bytes"
        );

        buf[0] = reg as u8 | SPI_READ_BIT;

        let out = self.transfer(&mut buf[..data.len() + 1])?;
        data.copy_from_slice(&out[1..data.len() + 1]);
        Ok(())
    }

    #[inline]
    fn read(&mut self, reg: Reg) -> Result<u8, Self::Error> {
        let mut buf = [reg as u8 | SPI_READ_BIT, 0];
        let out = self.transfer(&mut buf)?;
        Ok(out[1])
    }

    #[inline]
    fn write(&mut self, reg: Reg, data: u8) -> Result<(), Self::Error> {
        let mut buf = [reg as u8 & SPI_WRITE_MASK, data];
        self.write(&mut buf)
    }
}
