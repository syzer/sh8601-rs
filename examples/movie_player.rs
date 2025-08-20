#![no_std]
#![no_main]

use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    DMA_CHUNK_SIZE,
};

use embedded_graphics::{
    pixelcolor::Rgb888,
    prelude::*,
    image::{Image, ImageRaw},
};

extern crate alloc;
use esp_alloc as _;
use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    i2c::master::{Config as I2cConfig, I2c},
    main,
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    time::Rate,
};
use esp_println::println;

// --- Touch (FT3168) ---
use embedded_hal_bus::{i2c, util::AtomicCell};
use ft3x68_rs::{Ft3x68Driver, FT3168_DEVICE_ADDRESS, ResetInterface, TouchState};

// Minimal reset driver for FT3168 via TCA9554 (addr 0x20).
pub struct TouchReset<I2C> {
    i2c: I2C,
}

impl<I2C> TouchReset<I2C> {
    pub fn new(i2c: I2C) -> Self { Self { i2c } }
}

impl<I2C> ResetInterface for TouchReset<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    type Error = <I2C as embedded_hal::i2c::ErrorType>::Error;

    fn reset(&mut self) -> Result<(), Self::Error> {
        // TCA9554 registers: 0x03=CONFIG (1=input), 0x01=OUTPUT
        // Only toggle the configured TOUCH_RST_BIT, preserve other pins.
        const TCA9554_ADDR: u8 = 0x20;        // adjust if your expander addr differs
        const TOUCH_RST_BIT: u8 = 1 << 2;     // adjust if reset is on a different pin

        // Ensure the RESET pin is an output without changing others.
        let mut cfg = [0u8];
        self.i2c.write_read(TCA9554_ADDR, &[0x03], &mut cfg)?; // read CONFIG
        let new_cfg = cfg[0] & !TOUCH_RST_BIT;                 // set bit as output (0)
        if new_cfg != cfg[0] {
            self.i2c.write(TCA9554_ADDR, &[0x03, new_cfg])?;
        }

        // Drive low then high: read-modify-write OUTPUT register.
        let mut out = [0u8];
        self.i2c.write_read(TCA9554_ADDR, &[0x01], &mut out)?; // read OUTPUT
        let low = out[0] & !TOUCH_RST_BIT;
        self.i2c.write(TCA9554_ADDR, &[0x01, low])?;           // reset low

        let d = esp_hal::delay::Delay::new();
        d.delay_millis(20);

        let high = low | TOUCH_RST_BIT;
        self.i2c.write(TCA9554_ADDR, &[0x01, high])?;          // reset high
        d.delay_millis(200);
        Ok(())
    }
}

const W: u32 = 368;
const H: u32 = 448;

struct Movie {
    data: &'static [u8],
    frames: usize,
}

impl Movie {
    const fn new(data: &'static [u8], frames: usize) -> Self {
        Self { data, frames }
    }

    fn get_frame(&self, index: usize) -> Option<&[u8]> {
        if index >= self.frames {
            return None;
        }
        
        let frame_size = (W * H * 2) as usize; // RGB565 = 2 bytes per pixel (input data)
        let start = index * frame_size;
        let end = start + frame_size;
        
        if end <= self.data.len() {
            Some(&self.data[start..end])
        } else {
            None
        }
    }

    // Convert RGB565 frame data to RGB888 for display
    fn get_frame_rgb888(&self, index: usize, buffer: &mut [u8]) -> Option<()> {
        if let Some(rgb565_data) = self.get_frame(index) {
            let width = W as usize;
            let height = H as usize;
            let expected_size = width.checked_mul(height)?.checked_mul(3)?; // RGB888 = 3 bytes per pixel (output)
            if buffer.len() < expected_size {
                return None;
            }

            let total_pixels = width.checked_mul(height)?;

            // Convert RGB565 to RGB888
            for i in 0..total_pixels {
                let rgb565_idx = i.checked_mul(2)?;
                let rgb888_idx = i.checked_mul(3)?;

                if rgb565_idx + 1 < rgb565_data.len() && rgb888_idx + 2 < buffer.len() {
                    // Read RGB565 (little-endian)
                    let rgb565 = u16::from_le_bytes([
                        rgb565_data[rgb565_idx], 
                        rgb565_data[rgb565_idx + 1]
                    ]);

                    // Extract RGB565 components
                    let r5 = ((rgb565 >> 11) & 0x1F) as u8;
                    let g6 = ((rgb565 >> 5) & 0x3F) as u8; 
                    let b5 = (rgb565 & 0x1F) as u8;

                    // Convert to RGB888 (expand bit depth)
                    let r8 = (r5 * 255 / 31) as u8;
                    let g8 = (g6 * 255 / 63) as u8;
                    let b8 = (b5 * 255 / 31) as u8;

                    // Store RGB888
                    buffer[rgb888_idx] = r8;
                    buffer[rgb888_idx + 1] = g8;
                    buffer[rgb888_idx + 2] = b8;
                }
            }
            Some(())
        } else {
            None
        }
    }
}

// Include the movie data (tiny test versions with 5 frames)
static MOVIE_COW_DATA: &[u8] = include_bytes!("../assets/rgb/cow_tiny_rgb565.raw");
static MOVIE_NO_COW_DATA: &[u8] = include_bytes!("../assets/rgb/no_cow_tiny_rgb565.raw");

esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let delay = Delay::new();

    // --- DMA Buffers for SPI ---
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_CHUNK_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // SPI Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40_u32))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sio0(peripherals.GPIO4)
    .with_sio1(peripherals.GPIO5)
    .with_sio2(peripherals.GPIO6)
    .with_sio3(peripherals.GPIO7)
    .with_cs(peripherals.GPIO12)
    .with_sck(peripherals.GPIO11)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    // I2C Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO15)
    .with_scl(peripherals.GPIO14);

    // Share the I2C bus between display reset expander and touch controller
    let i2c_cell = AtomicCell::new(i2c);
    let i2c_for_lcd = i2c::AtomicDevice::new(&i2c_cell);
    let i2c_for_touch = i2c::AtomicDevice::new(&i2c_cell);

    // Initialize I2C GPIO Reset Pin for the WaveShare 1.8" AMOLED display (use shared bus)
    let reset = ResetDriver::new(i2c_for_lcd);

    // Initialize display driver for the Waveshare 1.8" AMOLED display
    let ws_driver = Ws18AmoledDriver::new(lcd_spi);

    // Set up the display size
    const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);

    // Calculate framebuffer size based on the display size and color mode
    const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb888);

    // Instantiate and Initialize Display
    println!("Initializing SH8601 Display...");
    let display_res = Sh8601Driver::new_heap::<_, FB_SIZE>(
        ws_driver,
        reset,
        ColorMode::Rgb888,
        DISPLAY_SIZE,
        delay,
    );
    let mut display = match display_res {
        Ok(d) => {
            println!("Display initialized successfully.");
            d
        }
        Err(e) => {
            println!("Error initializing display: {:?}", e);
            loop {}
        }
    };

    // --- Initialize FT3168 touch ---
    println!("Initializing touch...");
    let touch_reset = TouchReset::new(i2c::AtomicDevice::new(&i2c_cell));
    let mut touch = Ft3x68Driver::new(
        i2c_for_touch,
        FT3168_DEVICE_ADDRESS, // usually 0x38
        touch_reset,
        Delay::new(),
    );
    if let Err(e) = touch.initialize() {
        println!("FT3168 init failed: {:?}", e);
    } else {
        // optional: enable gesture mode
        let _ = touch.set_gesture_mode(true);
        println!("FT3168 ready");
    }

    // Movie setup - INSIDE the main function where it belongs
    let movies = [
        Movie::new(MOVIE_COW_DATA, 5),      // 5 frames for tiny test movies
        Movie::new(MOVIE_NO_COW_DATA, 5),   // 5 frames for tiny test movies
    ];
    
    let mut current_movie = 0;
    let mut current_frame = 0;
    let mut frame_counter = 0u32;
    let mut prev_pressed = false;

    // Buffer for RGB888 conversion (allocated once, reused for each frame)
    extern crate alloc;
    use alloc::vec::Vec;
    let width = W as usize;
    let height = H as usize;
    let buffer_size = width * height * 3; // RGB888 = 3 bytes per pixel
    let mut rgb888_buffer = Vec::with_capacity(buffer_size);
    rgb888_buffer.resize(buffer_size, 0);

    loop {
        // Check touch input for movie switching
        match touch.touch1() {
            Ok(touch_state) => {
                let is_pressed = matches!(touch_state, TouchState::Pressed(_));
                
                // Detect rising edge (just pressed)
                if is_pressed && !prev_pressed {
                    current_movie = (current_movie + 1) % movies.len();
                    current_frame = 0; // Reset to start of new movie
                    frame_counter = 0;
                    println!("Switching to movie {}", current_movie);
                }
                prev_pressed = is_pressed;
            }
            Err(_e) => {
                // ignore transient I2C errors
            }
        }

        // Animate at ~24fps (every 4th loop iteration at 10ms = ~25fps)
        if frame_counter % 4 == 0 {
            // Convert RGB565 frame to RGB888 and display
            if movies[current_movie].get_frame_rgb888(current_frame, &mut rgb888_buffer).is_some() {
                // Create ImageRaw from converted RGB888 data
                let raw_image = ImageRaw::<Rgb888>::new(&rgb888_buffer, W);
                let image = Image::new(&raw_image, Point::new(0, 0));
                
                // Draw the image
                if let Err(_e) = image.draw(&mut display) {
                    println!("Error drawing frame");
                } else {
                    let _ = display.flush();
                }
            }
            
            current_frame = (current_frame + 1) % movies[current_movie].frames;
        }

        frame_counter = frame_counter.wrapping_add(1);
        delay.delay_millis(10);
    }
}