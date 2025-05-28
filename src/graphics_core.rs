use crate::{ControllerInterface, DrawTarget, ResetInterface, Sh8601Driver};
use embedded_graphics_core::{pixelcolor::Rgb888, prelude::*};

impl<IFACE, RST> DrawTarget for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    type Color = Rgb888;
    // Drawing to the framebuffer in memory is infallible.
    // Errors happen during flush with SPI comms.
    type Error = core::convert::Infallible;

    /// Draws a single pixel to the internal framebuffer.
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
                    let r = (color.into_storage() >> 16) as u8; // 8-bit Red
                    let g = (color.into_storage() >> 8) as u8; // 8-bit Green
                    let b = color.into_storage() as u8; // 8-bit Blue

                    self.framebuffer[index] = r as u8;
                    self.framebuffer[index + 1] = g as u8;
                    self.framebuffer[index + 2] = b as u8;
                }
            }
        }
        Ok(())
    }
}

// =========== embedded-graphics OriginDimensions Implementation ===========

impl<IFACE, RST> OriginDimensions for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    fn size(&self) -> Size {
        Size::new((self.config.width) as u32, (self.config.height) as u32)
    }
}
