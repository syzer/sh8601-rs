#![no_std]
#![no_main]

use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    DMA_CHUNK_SIZE,
};

use embedded_graphics::{
    pixelcolor::Rgb565,
    prelude::*,
    image::{Image, ImageRaw},
};

extern crate alloc;
use alloc::vec::Vec;
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

// Simple file streaming approach - we'll use a simpler method
// Since full SD card integration is complex, let's create a streaming buffer approach

const W: u32 = 368;
const H: u32 = 448;
const FRAME_SIZE: usize = (368 * 448 * 2) as usize; // RGB565 = 2 bytes per pixel

// Movie structure that simulates SD card streaming
struct Movie {
    name: &'static str,
    filename: &'static str,
    total_frames: usize,
    frame_buffer: Vec<u8>, // Buffer for one frame
}

impl Movie {
    fn new(name: &'static str, filename: &'static str, total_frames: usize) -> Self {
        let mut frame_buffer = Vec::new();
        frame_buffer.resize(FRAME_SIZE, 0);
        
        Self { 
            name, 
            filename, 
            total_frames,
            frame_buffer,
        }
    }

    fn get_frame_data(&self) -> &[u8] {
        &self.frame_buffer
    }

    // Simulate reading a frame from SD card
    // In a real implementation, this would read from the actual SD card file
    fn simulate_load_frame(&mut self, frame_index: usize) -> Result<(), &'static str> {
        if frame_index >= self.total_frames {
            return Err("Frame index out of bounds");
        }

        // For now, generate a simple test pattern based on frame index
        // In real implementation, this would read from SD card
        let pattern_value = ((frame_index * 255) / self.total_frames) as u8;
        
        // Create a simple gradient pattern for testing
        for y in 0..448 {
            for x in 0..368 {
                let pixel_idx = (y * 368 + x) * 2;
                if pixel_idx + 1 < self.frame_buffer.len() {
                    // RGB565: 5 bits red, 6 bits green, 5 bits blue
                    let r = ((x + frame_index) % 32) as u16;  // 5 bits
                    let g = ((y + frame_index) % 64) as u16;  // 6 bits  
                    let b = (pattern_value % 32) as u16;      // 5 bits
                    
                    // Pack into RGB565 format (little-endian)
                    let rgb565 = ((r & 0x1F) << 11) | ((g & 0x3F) << 5) | (b & 0x1F);
                    self.frame_buffer[pixel_idx] = (rgb565 & 0xFF) as u8;     // Low byte
                    self.frame_buffer[pixel_idx + 1] = (rgb565 >> 8) as u8;   // High byte
                }
            }
        }
        
        println!("Loaded frame {} for movie {}", frame_index, self.name);
        Ok(())
    }
}

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
    const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb565);

    // Instantiate and Initialize Display
    println!("Initializing SH8601 Display...");
    let display_res = Sh8601Driver::new_heap::<_, FB_SIZE>(
        ws_driver,
        reset,
        ColorMode::Rgb565,
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

    // Movie setup - simulating SD card streaming
    // Calculate frame count: 39,897,088 bytes / (368 × 448 × 2) = ~121 frames
    let expected_frames = 39897088 / FRAME_SIZE; // Based on actual file size from SD card
    
    let mut movies = [
        Movie::new("Cow", "cow_full_rgb565.raw", expected_frames),
        Movie::new("No Cow", "no_cow_full_rgb565.raw", expected_frames),
    ];
    
    println!("Expected frames per movie: {} ({} bytes per frame)", expected_frames, FRAME_SIZE);
    println!("Total memory per frame: {}KB", FRAME_SIZE / 1024);
    
    let mut current_movie = 0;
    let mut current_frame = 0;
    let mut frame_counter = 0u32;
    let mut prev_pressed = false;

    println!("Starting simulated SD card streaming movie playback...");

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
                    println!("Switching to movie: {}", movies[current_movie].name);
                }
                prev_pressed = is_pressed;
            }
            Err(_e) => {
                // ignore transient I2C errors
            }
        }

        // Animate at ~24fps (every 4th loop iteration at 10ms = ~25fps)  
        if frame_counter % 4 == 0 {
            // Simulate loading current frame from SD card
            if let Ok(()) = movies[current_movie].simulate_load_frame(current_frame) {
                let frame_data = movies[current_movie].get_frame_data();
                
                // Create ImageRaw from RGB565 data
                let raw_image = ImageRaw::<Rgb565>::new(frame_data, W);
                let image = Image::new(&raw_image, Point::new(0, 0));
                
                // Draw the image
                if let Err(_e) = image.draw(&mut display) {
                    println!("Error drawing frame");
                } else {
                    let _ = display.flush();
                }
            } else {
                println!("Failed to load frame {} from movie {}", current_frame, movies[current_movie].name);
            }
            
            current_frame = (current_frame + 1) % movies[current_movie].total_frames;
        }

        frame_counter = frame_counter.wrapping_add(1);
        delay.delay_millis(10);
    }
}
