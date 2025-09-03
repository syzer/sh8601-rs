use crate::{ControllerInterface, DrawTarget, ResetInterface, Sh8601Driver, ColorMode};
use embedded_graphics_core::{pixelcolor::*, prelude::*};

/// Trait for converting embedded-graphics colors to raw bytes for the framebuffer
pub trait ColorConverter<C: PixelColor> {
    fn color_to_bytes(color: C) -> &'static [u8];
    fn bytes_per_pixel() -> usize;
    fn color_mode() -> ColorMode;
}

/// RGB888 converter
pub struct Rgb888Converter;
impl ColorConverter<Rgb888> for Rgb888Converter {
    fn color_to_bytes(color: Rgb888) -> &'static [u8] {
        // Note: This would need to return actual bytes, probably via a thread-local buffer
        // This is a simplified example
        &[]
    }
    
    fn bytes_per_pixel() -> usize { 3 }
    fn color_mode() -> ColorMode { ColorMode::Rgb888 }
}

/// RGB565 converter  
pub struct Rgb565Converter;
impl ColorConverter<Rgb565> for Rgb565Converter {
    fn color_to_bytes(color: Rgb565) -> &'static [u8] {
        // Convert RGB565 to 2-byte little-endian format
        &[]
    }
    
    fn bytes_per_pixel() -> usize { 2 }
    fn color_mode() -> ColorMode { ColorMode::Rgb565 }
}

/// Generic DrawTarget implementation that works with any supported color type
impl<IFACE, RST, C, CONV> DrawTarget for Sh8601Driver<IFACE, RST>
where
    IFACE: ControllerInterface,
    RST: ResetInterface,
    C: PixelColor,
    CONV: ColorConverter<C>,
{
    type Color = C;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let bytes_per_pixel = CONV::bytes_per_pixel();
        
        for Pixel(coord, color) in pixels.into_iter() {
            if coord.x >= 0
                && coord.x < self.config.width as i32
                && coord.y >= 0
                && coord.y < self.config.height as i32
            {
                let x = coord.x as u32;
                let y = coord.y as u32;
                let index = ((y * self.config.width as u32 + x) * bytes_per_pixel as u32) as usize;

                // Convert color based on the specific format
                match CONV::color_mode() {
                    ColorMode::Rgb888 => {
                        if index + 2 < self.framebuffer.len() {
                            let rgb888: Rgb888 = color.into(); // This would need proper conversion
                            let storage = rgb888.into_storage();
                            self.framebuffer[index] = (storage >> 16) as u8;     // R
                            self.framebuffer[index + 1] = (storage >> 8) as u8;  // G  
                            self.framebuffer[index + 2] = storage as u8;         // B
                        }
                    }
                    ColorMode::Rgb565 => {
                        if index + 1 < self.framebuffer.len() {
                            let rgb565: Rgb565 = color.into(); // This would need proper conversion
                            let storage = rgb565.into_storage();
                            self.framebuffer[index] = storage as u8;           // Low byte
                            self.framebuffer[index + 1] = (storage >> 8) as u8; // High byte
                        }
                    }
                    ColorMode::Rgb666 => {
                        // Implementation for RGB666 format
                        todo!("RGB666 support")
                    }
                    ColorMode::Gray8 => {
                        if index < self.framebuffer.len() {
                            // Convert color to grayscale
                            let gray: Gray8 = color.into(); // This would need proper conversion
                            self.framebuffer[index] = gray.luma();
                        }
                    }
                }
            }
        }
        Ok(())
    }
}
