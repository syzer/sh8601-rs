//! Driver implementation for Waveshare ESP32-S3 1.8" AMOLED
//! Uses QSPI interface and I2C-based GPIO expander or GPIO for reset.

use crate::{DisplayInterface, ResetInterface};
use esp_hal::{
    delay::Delay,
    i2c::master::{Error as I2cError, I2c},
    spi::{
        master::{Address, Command, SpiDmaBus},
        DataMode, Error as SpiError,
    },
    Blocking,
};

const CMD_RAMWR: u32 = 0x2C;
const CMD_RAMWRC: u32 = 0x3C;
const QSPI_PIXEL_OPCODE: u8 = 0x32;
const QSPI_CONTROL_OPCODE: u8 = 0x02;
pub const DMA_CHUNK_SIZE: usize = 16380;

/// QSPI implementation of DisplayInterface for SH8601
pub struct Ws18AmoledDriver {
    pub qspi: SpiDmaBus<'static, Blocking>,
}

impl Ws18AmoledDriver {
    pub fn new(qspi: SpiDmaBus<'static, Blocking>) -> Self {
        Ws18AmoledDriver { qspi }
    }
}

impl DisplayInterface for Ws18AmoledDriver {
    type Error = SpiError;

    fn send_command(&mut self, cmd: u8) -> Result<(), Self::Error> {
        let address_value = (cmd as u32) << 8;

        self.qspi.half_duplex_write(
            DataMode::Single,
            Command::_8Bit(QSPI_CONTROL_OPCODE as u16, DataMode::Single),
            Address::_24Bit(address_value, DataMode::Single),
            0,
            &[],
        )?;
        Ok(())
    }

    fn send_command_with_data(&mut self, cmd: u8, data: &[u8]) -> Result<(), Self::Error> {
        let address_value = (cmd as u32) << 8;

        self.qspi.half_duplex_write(
            DataMode::Single,
            Command::_8Bit(QSPI_CONTROL_OPCODE as u16, DataMode::Single),
            Address::_24Bit(address_value, DataMode::Single),
            0,
            data,
        )?;
        Ok(())
    }

    fn send_pixels(&mut self, pixels: &[u8]) -> Result<(), Self::Error> {
        let ramwr_addr_val = (CMD_RAMWR as u32) << 8;
        let ramwrc_addr_val = (CMD_RAMWRC as u32) << 8;

        let mut chunks = pixels.chunks(DMA_CHUNK_SIZE).enumerate();

        while let Some((index, chunk)) = chunks.next() {
            if index == 0 {
                self.qspi.half_duplex_write(
                    DataMode::Quad,
                    Command::_8Bit(QSPI_PIXEL_OPCODE as u16, DataMode::Single),
                    Address::_24Bit(ramwr_addr_val, DataMode::Single),
                    0,
                    chunk,
                )?;
            } else {
                self.qspi.half_duplex_write(
                    DataMode::Quad,
                    Command::_8Bit(QSPI_PIXEL_OPCODE as u16, DataMode::Single),
                    Address::_24Bit(ramwrc_addr_val, DataMode::Single),
                    0,
                    chunk,
                )?;
            }
        }
        Ok(())
    }
}

/// I2C-controlled GPIO Reset Pin
pub struct I2cGpioResetDriver {
    i2c: I2c<'static, Blocking>,
}

impl I2cGpioResetDriver {
    pub fn new(i2c: I2c<'static, Blocking>) -> Self {
        I2cGpioResetDriver { i2c }
    }
}

impl ResetInterface for I2cGpioResetDriver {
    type Error = I2cError;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();
        self.i2c.write(0x20, &[0x03, 0x00])?; // Configure as output
        self.i2c.write(0x20, &[0x01, 0b0000_0010])?; // Drive low
        delay.delay_millis(20);
        self.i2c.write(0x20, &[0x01, 0b0000_0111])?; // Drive high
        delay.delay_millis(150);
        Ok(())
    }
}
