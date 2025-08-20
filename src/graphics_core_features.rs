use crate::{ControllerInterface, DrawTarget, ResetInterface, Sh8601Driver};
use embedded_graphics_core::prelude::*;

// Feature flags to choose color mode at compile time
#[cfg(feature = "rgb565")]
use embedded_graphics_core::pixelcolor::Rgb565 as DisplayColor;

#[cfg(feature = "rgb888")]
use embedded_graphics_core::pixelcolor::Rgb888 as DisplayColor;

#[cfg(feature = "gray8")]  
use embedded_graphics_core::pixelcolor::Gray8 as DisplayColor;

// Default to RGB888 if no feature is specified
#[cfg(not(any(feature = "rgb565", feature = "rgb888", feature = "gray8")))]
use embedded_graphics_core::pixelcolor::Rgb888 as DisplayColor;

impl<IFACE, RST> DrawTarget for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    type Color = DisplayColor;
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
                
                #[cfg(any(feature = "rgb888", not(any(feature = "rgb565", feature = "rgb888", feature = "gray8"))))]
                {
                    let index = ((y * self.config.width as u32 + x) * 3) as usize;
                    if index + 2 < self.framebuffer.len() {
                        let storage = color.into_storage();
                        self.framebuffer[index] = (storage >> 16) as u8;     // R
                        self.framebuffer[index + 1] = (storage >> 8) as u8;  // G
                        self.framebuffer[index + 2] = storage as u8;         // B
                    }
                }
                
                #[cfg(feature = "rgb565")]
                {
                    let index = ((y * self.config.width as u32 + x) * 2) as usize;
                    if index + 1 < self.framebuffer.len() {
                        let storage = color.into_storage();
                        self.framebuffer[index] = storage as u8;           // Low byte
                        self.framebuffer[index + 1] = (storage >> 8) as u8; // High byte
                    }
                }
                
                #[cfg(feature = "gray8")]
                {
                    let index = (y * self.config.width as u32 + x) as usize;
                    if index < self.framebuffer.len() {
                        self.framebuffer[index] = color.luma();
                    }
                }
            }
        }
        Ok(())
    }
}

impl<IFACE, RST> OriginDimensions for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
{
    fn size(&self) -> Size {
        Size::new((self.config.width) as u32, (self.config.height) as u32)
    }
}
