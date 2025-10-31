#![no_std]
#![no_main]

use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    DMA_CHUNK_SIZE, tile_decoder::TileDecoder,
};

use alloc::collections::BTreeMap;

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
    time::{Instant, Rate},
};
use esp_println::println;

// I2C bus sharing for display reset
use embedded_hal_bus::{i2c, util::AtomicCell};

esp_app_desc!();

const W: u32 = 368;
const H: u32 = 448;

// Ping-pong buffers in internal SRAM (DRAM0) - avoids PSRAM for hot path
const LINES: usize = 16;  // Process 16 lines at a time (tunable: 8-32)
const PIX: usize = (W as usize) * LINES;

// Wrapper struct for 64-byte DMA alignment (cache line alignment)
#[repr(align(64))]
struct AlignedU16Array<const N: usize> {
    data: [u16; N],
}

// RGB565 ping-pong buffers in DRAM0 (internal SRAM) - 64-byte aligned for DMA
// Used for streaming framebuffer chunks to display
#[allow(static_mut_refs)]
#[link_section = ".dram0.bss"]
static mut A: AlignedU16Array<PIX> = AlignedU16Array {
    data: [0; PIX],
};

#[allow(static_mut_refs)]
#[link_section = ".dram0.bss"]
static mut B: AlignedU16Array<PIX> = AlignedU16Array {
    data: [0; PIX],
};

// Byte swapping removed: tiles are now decoded with correct byte order for SPI
// Tile decoder swaps bytes once during decode (big-endian tile -> little-endian framebuffer)
// SPI sends LSB-first, so little-endian u16 becomes MSB-first on wire
// No per-chunk swapping needed in hot path!

#[main]
fn main() -> ! {
    // Configure CPU frequency to 240 MHz for maximum performance
    // Note: CPU frequency is configured via build-time flags or Config if available
    // For esp-hal, CPU typically runs at max (240 MHz) by default
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // CRITICAL: Initialize PSRAM allocator BEFORE any heap allocations
    // This must happen first, before any Vec or other heap-allocated structures
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // Include tile-based video file (dirty tile encoding - only changed 16×16 tiles per frame)
    static TILE_VIDEO: &[u8] = include_bytes!("../assets/tiles/Yasin_no_cow.tiles");
    
    // Initialize tile decoder
    let decoder = match TileDecoder::new(TILE_VIDEO, W, H) {
        Ok(d) => {
            println!("Tile decoder initialized: {} frames", d.frame_count());
            d
        }
        Err(e) => {
            println!("Error initializing tile decoder: {}", e);
            println!("Make sure to convert MJPEG to tiles first:");
            println!("  just convert-tiles assets/mjpeg/Yasin_no_cow.mjpeg assets/tiles/Yasin_no_cow.tiles");
            loop {}
        }
    };
    
    let num_frames = decoder.frame_count();
    if num_frames == 0 {
        println!("Error: Tile file has no frames");
        loop {}
    }
    
    let mut frame_idx: usize = 0;

    let delay = Delay::new();

    // --- DMA Buffers for SPI ---
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_CHUNK_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // SPI Configuration for Waveshare ESP32-S3 1.8inch AMOLED Display
    // Hardware is configured for QSPI. Pinout obtained from the schematic.
    // Schematic:
    // https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf
    // Using DMA for more efficient SPI communication.
    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(80_u32))
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

    // I2C Configuration for Waveshare ESP32-S3 1.8inch AMOLED Display
    // Display uses an I2C IO Expander (TCA9554PWR) to control the LCD_RESET and LCD_DC lines.
    // Pinout:
    // SDA -> GPIO15
    // SCL -> GPIO14
    // Schematic:
    // https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO15)
    .with_scl(peripherals.GPIO14);

    // I2C for display reset (via TCA9554PWR GPIO expander)
    let i2c_cell = AtomicCell::new(i2c);
    let i2c_for_lcd = i2c::AtomicDevice::new(&i2c_cell);

    // Initialize I2C GPIO Reset Pin for the WaveShare 1.8" AMOLED display
    let reset = ResetDriver::new(i2c_for_lcd);

    // Initialize display driver for the Waveshare 1.8" AMOLED display
    let ws_driver = Ws18AmoledDriver::new(lcd_spi);

    // Set up the display size
    const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);

    // Calculate framebuffer size - using RGB565 for 33% less data transfer
    // We'll convert RGB888 -> RGB565 in chunks using static buffers
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
        Ok(mut d) => {
            println!("Display initialized successfully.");
            // Set maximum brightness
            if let Err(e) = d.set_brightness(1023) {
                println!("Warning: Could not set brightness: {:?}", e);
            } else {
                println!("Display brightness set to maximum (1023)");
            }
            d
        }
        Err(e) => {
            println!("Error initializing display: {:?}", e);
            loop {}
        }
    };

    // Play tile-based animation (dirty tile encoding - only changed tiles per frame)
    println!("Starting tile-based playback...");
    println!("Decoding only changed 16×16 tiles per frame for optimal performance");
    
    // Get framebuffer for tile decoding (RGB565, already in correct format)
    let framebuffer = display.framebuffer_mut();
    let fb_u16: &mut [u16] = unsafe {
        core::slice::from_raw_parts_mut(
            framebuffer.as_mut_ptr() as *mut u16,
            framebuffer.len() / 2
        )
    };
    
    // Previous frame tiles cache (for tile decoder state)
    let mut prev_tiles = BTreeMap::new();
    
    // FPS tracking
    let mut t0 = Instant::now();
    let mut frames: u32 = 0;
    let mut frame_count = 0;
    
    loop {
        // Clear framebuffer for first frame (all tiles encoded)
        // Subsequent frames only update changed tiles
        if frame_idx == 0 {
            fb_u16.fill(0);
        }
        
        // Decode frame tiles into framebuffer
        match decoder.decode_frame(frame_idx, fb_u16, &mut prev_tiles) {
            Ok(tiles_decoded) => {
                // Begin frame streaming - sets window once
                if let Err(_e) = display.begin_frame(0, 0, (W - 1) as u16, (H - 1) as u16) {
                    // Error handling removed from hot path - just skip frame
                    frame_idx = (frame_idx + 1) % num_frames;
                    continue;
                }
                
                // Stream framebuffer to display in chunks (using existing ping-pong buffers)
                // HOT PATH: No println! or error handling here for maximum performance
                let mut use_a = true;
                let mut y = 0;
                let mut is_first_chunk = true;
                
                while y < (H as usize) {
                    // Calculate how many lines we'll process in this chunk
                    let got_lines = ((H as usize) - y).min(LINES);
                    let end_pix = got_lines * (W as usize);
                    let fb_start = y * (W as usize);
                    let fb_end = fb_start + end_pix;
                    
                    unsafe {
                        // Get RGB565 data from framebuffer for this stripe
                        // Framebuffer already has correct byte order (swapped once during tile decode)
                        // SPI sends LSB-first, so native little-endian u16 becomes MSB-first on wire
                        let src = &fb_u16[fb_start..fb_end];
                        
                        // Copy to ping-pong buffer (no byte swapping needed - done once at decode time)
                        let dst = if use_a { &mut A.data[..] } else { &mut B.data[..] };
                        dst[..end_pix].copy_from_slice(src);
                        
                        // Write u16 slice directly - more efficient than byte slice
                        // Error handling removed from hot path for performance
                        if display.write_pixels_dma_u16(&dst[..end_pix], is_first_chunk).is_err() {
                            break;
                        }
                    }
                    
                    // While DMA runs, prepare for next chunk
                    use_a = !use_a; // Ping-pong swap
                    is_first_chunk = false; // All chunks after first use RAMWRC
                    y += got_lines;
                }
                
                // End frame (error handling removed from hot path)
                let _ = display.end_frame();
                
                frame_count += 1;
                frames += 1;
                
                // Calculate FPS every second
                let elapsed_ms = t0.elapsed().as_millis();
                if elapsed_ms >= 1000 {
                    let fps = (frames as f32) * 1000.0 / (elapsed_ms as f32);
                    println!("Frame {} ({} tiles, {} total, {:.1} FPS)", frame_idx, tiles_decoded, frame_count, fps);
                    frames = 0;
                    t0 = Instant::now();
                } else if frame_count == 1 || (frame_count % 60 == 0) {
                    // Print frame info occasionally (before 1 second has elapsed)
                    println!("Frame {} ({} tiles, {} total)", frame_idx, tiles_decoded, frame_count);
                }
            }
            Err(_e) => {
                // Error handling moved outside hot path - only print occasionally
                if frame_idx % 10 == 0 {
                    println!("Decode error on frame {}", frame_idx);
                }
            }
        }
        
        // Advance to next frame
        frame_idx = (frame_idx + 1) % num_frames;
    }
}
