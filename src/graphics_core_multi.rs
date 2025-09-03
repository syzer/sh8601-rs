use crate::{ControllerInterface, DrawTarget, ResetInterface, Sh8601Driver};
use embedded_graphics_core::{pixelcolor::*, prelude::*};

// =========== RGB888 Implementation (Current) ===========
impl<IFACE, RST> DrawTarget for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    type Color = Rgb888;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0
                && coord.x < self.config.width as i32
                && coord.y >= 0
                && coord.y < self.config.height as i32
            {
                let x = coord.x as u32;
                let y = coord.y as u32;
                let index = ((y * self.config.width as u32 + x) * 3) as usize;

                if index + 2 < self.framebuffer.len() {
                    let storage = color.into_storage();
                    self.framebuffer[index] = (storage >> 16) as u8;     // R
                    self.framebuffer[index + 1] = (storage >> 8) as u8;  // G
                    self.framebuffer[index + 2] = storage as u8;         // B
                }
            }
        }
        Ok(())
    }
}

// =========== RGB565 Driver Type ===========
/// A version of Sh8601Driver that works with RGB565 colors
pub struct Sh8601Rgb565Driver<IFACE, RST> {
    inner: Sh8601Driver<IFACE, RST>,
}

impl<IFACE, RST> Sh8601Rgb565Driver<IFACE, RST> {
    pub fn new(inner: Sh8601Driver<IFACE, RST>) -> Self {
        Self { inner }
    }
    
    // Delegate other methods to the inner driver
    pub fn flush(&mut self) -> Result<(), <IFACE as ControllerInterface>::Error> {
        self.inner.flush()
    }
}

impl<IFACE, RST> DrawTarget for Sh8601Rgb565Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    type Color = Rgb565;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0
                && coord.x < self.inner.config.width as i32
                && coord.y >= 0
                && coord.y < self.inner.config.height as i32
            {
                let x = coord.x as u32;
                let y = coord.y as u32;
                let index = ((y * self.inner.config.width as u32 + x) * 2) as usize;

                if index + 1 < self.inner.framebuffer.len() {
                    let storage = color.into_storage();
                    // Store RGB565 in little-endian format
                    self.inner.framebuffer[index] = storage as u8;           // Low byte
                    self.inner.framebuffer[index + 1] = (storage >> 8) as u8; // High byte
                }
            }
        }
        Ok(())
    }
}

impl<IFACE, RST> OriginDimensions for Sh8601Rgb565Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    fn size(&self) -> Size {
        self.inner.size()
    }
}

// =========== Gray8 Driver Type ===========
/// A version of Sh8601Driver that works with Gray8 colors
pub struct Sh8601Gray8Driver<IFACE, RST> {
    inner: Sh8601Driver<IFACE, RST>,
}

impl<IFACE, RST> DrawTarget for Sh8601Gray8Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    type Color = Gray8;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0
                && coord.x < self.inner.config.width as i32
                && coord.y >= 0
                && coord.y < self.inner.config.height as i32
            {
                let x = coord.x as u32;
                let y = coord.y as u32;
                let index = (y * self.inner.config.width as u32 + x) as usize;

                if index < self.inner.framebuffer.len() {
                    self.inner.framebuffer[index] = color.luma();
                }
            }
        }
        Ok(())
    }
}
