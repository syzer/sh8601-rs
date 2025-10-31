#![no_std]
#![no_main]

use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    DMA_CHUNK_SIZE, residual_decoder::ResidualDecoder,
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
    time::{Instant, Rate},
};
use esp_println::println;
use embedded_hal_bus::{i2c, util::AtomicCell};

esp_app_desc!();

const W: u32 = 368;
const H: u32 = 448;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // PSRAM allocator first
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // --- Residual video (XOR + Zero-RLE + LZ4HC) ---
    static RESIDUAL_VIDEO: &[u8] = include_bytes!("../assets/residuals/Yasin_no_cow.residuals");
    let decoder = match ResidualDecoder::new(RESIDUAL_VIDEO, W, H) {
        Ok(d) => { println!("Residual decoder initialized: {} frames", d.frame_count()); d }
        Err(e) => { println!("Residual decoder init error: {}", e); loop {} }
    };
    let num_frames = decoder.frame_count();
    if num_frames == 0 { println!("No frames"); loop {} }

    let delay = Delay::new();

    // --- SPI + I2C setup (QSPI @ 80 MHz) ---
    let (rx_buffer, rx_desc, tx_buffer, tx_desc) = dma_buffers!(DMA_CHUNK_SIZE);
    let dma_rx = DmaRxBuf::new(rx_desc, rx_buffer).unwrap();
    let dma_tx = DmaTxBuf::new(tx_desc, tx_buffer).unwrap();

    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default().with_frequency(Rate::from_mhz(80)).with_mode(Mode::_0),
    ).unwrap()
     .with_sio0(peripherals.GPIO4)
     .with_sio1(peripherals.GPIO5)
     .with_sio2(peripherals.GPIO6)
     .with_sio3(peripherals.GPIO7)
     .with_cs(peripherals.GPIO12)
     .with_sck(peripherals.GPIO11)
     .with_dma(peripherals.DMA_CH0)
     .with_buffers(dma_rx, dma_tx);

    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default().with_frequency(Rate::from_khz(400)))
        .unwrap()
        .with_sda(peripherals.GPIO15)
        .with_scl(peripherals.GPIO14);

    let i2c_cell = AtomicCell::new(i2c);
    let i2c_for_lcd = i2c::AtomicDevice::new(&i2c_cell);

    let reset = ResetDriver::new(i2c_for_lcd);
    let ws = Ws18AmoledDriver::new(lcd_spi);

    const DS: DisplaySize = DisplaySize::new(W as u16, H as u16);
    const FB_SIZE: usize = framebuffer_size(DS, ColorMode::Rgb565);

    println!("Initializing SH8601 Display...");
    let mut display = match Sh8601Driver::new_heap::<_, FB_SIZE>(ws, reset, ColorMode::Rgb565, DS, delay) {
        Ok(mut d) => { let _ = d.set_brightness(1023); println!("Display OK"); d }
        Err(e) => { println!("Display init error: {:?}", e); loop {} }
    };

    // --- Framebuffer (RGB565 LE; residual decoder applies XOR directly) ---
    let fb = display.framebuffer_mut();
    let fb_u16: &mut [u16] = unsafe {
        core::slice::from_raw_parts_mut(fb.as_mut_ptr() as *mut u16, (W as usize)*(H as usize))
    };
    
    // Initialize framebuffer to zero (required for XOR residual decoding)
    fb_u16.fill(0);

    // FPS
    let mut t0 = Instant::now();
    let mut frames = 0u32;
    let mut frame_idx = 0usize;

    println!("Starting residual video playback (XOR + Zero-RLE + LZ4HC)...");

    loop {
        // Decode frame: LZ4 decompress → Zero-RLE expand → XOR apply to framebuffer
        // Much simpler than tiles: no dirty tracking, no coalescing, just decode and flush!
        if let Err(e) = decoder.decode_frame(frame_idx, fb_u16) {
            if frame_idx % 10 == 0 { println!("Decode error @ frame {}: {}", frame_idx, e); }
            frame_idx = (frame_idx + 1) % num_frames;
            continue;
        }

        // Flush entire framebuffer to display
        if let Err(e) = display.flush() {
            println!("Display flush error: {:?}", e);
        }

        frames += 1;
        let dt_ms = t0.elapsed().as_millis();
        if dt_ms >= 1000 {
            let fps = (frames as f32) * 1000.0 / (dt_ms as f32);
            println!("Frame {}  {:.1} FPS", frame_idx, fps);
            frames = 0;
            t0 = Instant::now();
        }

        frame_idx = (frame_idx + 1) % num_frames;
    }
}
