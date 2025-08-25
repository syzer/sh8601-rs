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
use core::convert::Infallible;
use embedded_hal::digital::OutputPin;
use embedded_hal::i2c::I2c as I2cTrait;
use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::{SdCard, VolumeManager, VolumeIdx, TimeSource, Timestamp, Mode as SdMode, ShortFileName};

extern crate alloc;
use esp_alloc as _;
use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    delay::Delay,
    dma::{DmaRxBuf, DmaTxBuf},
    dma_buffers,
    i2c::master::{Config as I2cConfig, I2c as EspI2c},
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
    I2C: I2cTrait,
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

// --- TCA9554 single-bit output wrapper (for SD CS on EXIO7) ---
struct Tca9554Pin<'a, B> {
    bus: i2c::AtomicDevice<'a, B>,
    addr: u8,
    bit: u8,
}
impl<'a, B> Tca9554Pin<'a, B>
where
    B: I2cTrait,
{
    fn new(dev: i2c::AtomicDevice<'a, B>, addr: u8, bit: u8) -> Self { Self { bus: dev, addr, bit } }
    fn ensure_output(&mut self) {
        let mut cfg = [0u8];
        let _ = self.bus.write_read(self.addr, &[0x03], &mut cfg); // CONFIG
        let new_cfg = cfg[0] & !(1 << self.bit); // set as output
        if new_cfg != cfg[0] {
            let _ = self.bus.write(self.addr, &[0x03, new_cfg]);
        }
    }
    fn write_level(&mut self, high: bool) {
        let mut out = [0u8];
        let _ = self.bus.write_read(self.addr, &[0x01], &mut out); // OUTPUT
        let mut val = out[0];
        if high { val |= 1 << self.bit; } else { val &= !(1 << self.bit); }
        let _ = self.bus.write(self.addr, &[0x01, val]);
    }
}

impl<'a, B> embedded_hal::digital::ErrorType for Tca9554Pin<'a, B>
where
    B: I2cTrait,
{
    type Error = Infallible;
}

impl<'a, B> OutputPin for Tca9554Pin<'a, B>
where
    B: I2cTrait,
{
    fn set_low(&mut self) -> Result<(), Self::Error> { self.ensure_output(); self.write_level(false); Ok(()) }
    fn set_high(&mut self) -> Result<(), Self::Error> { self.ensure_output(); self.write_level(true); Ok(()) }
}

// Minimal time source for embedded-sdmmc (we don't care about real timestamps now)
struct DummyTime;
impl TimeSource for DummyTime {
    fn get_timestamp(&self) -> Timestamp { Timestamp::from_fat(0, 0) }
}

// Display/native resolution
const W: u32 = 368;
const H: u32 = 448;

// RAW movie geometry (can be set to W/H if your RAWs match the panel)
// Set these to match your files. For legacy C6 assets you used 172x320.
const RAW_W: u32 = W;
const RAW_H: u32 = H;

// Derived sizes
const FRAME_SZ: usize = (RAW_W as usize) * (RAW_H as usize) * 2; // RGB565

// Stream in 64-line chunks (multiple of 512B sector size)
const BYTES_PER_LINE: usize = (RAW_W as usize) * 2;
const LINES_PER_CHUNK: usize = 64;
const CHUNK_BYTES: usize = BYTES_PER_LINE * LINES_PER_CHUNK; // e.g. 64 lines

// Ping-pong tiles for DMA-ish streaming from SD
static mut TILE_A: [u8; CHUNK_BYTES] = [0u8; CHUNK_BYTES];
static mut TILE_B: [u8; CHUNK_BYTES] = [0u8; CHUNK_BYTES];

esp_app_desc!();

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    println!("APP: esp_hal init OK");


    // keep this comment
    // esp_alloc::heap_allocator!(peripherals.PSRAM, esp_hal::psram);
    // Use PSRAM heap (moves framebuffer out of DRAM)
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    println!("APP: heap allocator ready (PSRAM)");

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
    let i2c = EspI2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
        .unwrap()
        .with_sda(peripherals.GPIO15)
        .with_scl(peripherals.GPIO14);

    // Share the I2C bus between display reset expander and touch controller
    let i2c_cell = AtomicCell::new(i2c);

    // -------- SD card over SPI (uses SPI3 with pins from schematic) --------
    // Pins confirmed from Waveshare schematic (ESP32-S3-Touch-AMOLED-1.8.pdf):
    // MOSI=GPIO1, SCK=GPIO2, MISO=GPIO3, CS=EXIO7 via TCA9554 @0x20.
    let sd_spi = Spi::new(
        peripherals.SPI3,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(24_u32)) // start conservative; can try 40–60 later
            .with_mode(Mode::_0),
    )
        .unwrap()
        .with_sck(peripherals.GPIO2)
        .with_mosi(peripherals.GPIO1)
        .with_miso(peripherals.GPIO3);

    // CS via IO expander EXIO7
    let mut sd_cs = Tca9554Pin::new(i2c::AtomicDevice::new(&i2c_cell), 0x20, 7);
    let _ = sd_cs.set_high(); // deassert CS
    println!("APP: SD bus + CS ready (CS=HIGH)");

    // Shareable SPI bus wrapper so LCD can still use SPI2 separately
    let sd_bus = core::cell::RefCell::new(sd_spi);
    let sd_dev = RefCellDevice::new(&sd_bus, sd_cs, Delay::new()).unwrap();

    println!("APP: Probing SD (SPI3 24MHz, Mode0)...");
    let mut sd_delay_for_card = Delay::new();
    let sd = SdCard::new(sd_dev, &mut sd_delay_for_card);
    println!("APP: SdCard::new done");
    match sd.num_bytes() {
        Ok(sz) => println!("SD size: {} bytes", sz),
        Err(e) => println!("SD: num_bytes failed: {:?}", e),
    }
    let volume_mgr = VolumeManager::new(sd, DummyTime);
    let volume = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => { println!("open_volume failed: {:?}", e); loop { Delay::new().delay_millis(1000); } }
    };
    let root_dir = match volume.open_root_dir() {
        Ok(d) => d,
        Err(e) => { println!("open_root_dir failed: {:?}", e); loop { Delay::new().delay_millis(1000); } }
    };
    // Build playlist of 368x448 RGB565 movies only
    const FRAME_SZ_368X448: usize = (368usize * 448usize) * 2; // 329,728 bytes per frame
    let mut movies: alloc::vec::Vec<ShortFileName> = alloc::vec::Vec::new();
    let mut kept = 0usize;
    let mut skipped = 0usize;
    let _ = root_dir.iterate_dir(|e| {
        if !e.attributes.is_directory() && e.name.extension() == b"RAW" {
            let sz = e.size as usize;
            if sz >= FRAME_SZ_368X448 && (sz % FRAME_SZ_368X448) == 0 {
                // Accept any file that matches the size/multiple, ignore name geometry
                movies.push(e.name.clone());
                kept += 1;
            } else {
                skipped += 1;
            }
        }
    });
    if movies.is_empty() {
        println!("No 368x448 .RAW movies found in root (kept={}, skipped={})", kept, skipped);
        loop { Delay::new().delay_millis(1000); }
    }
    println!("Found {} movie(s) at 368x448 (skipped {})", movies.len(), skipped);

    let i2c_for_lcd = i2c::AtomicDevice::new(&i2c_cell);
    let i2c_for_touch = i2c::AtomicDevice::new(&i2c_cell);

    // Initialize I2C GPIO Reset Pin for the WaveShare 1.8" AMOLED display (use shared bus)
    let reset = ResetDriver::new(i2c_for_lcd);

    // Initialize display driver for the Waveshare 1.8" AMOLED display
    let ws_driver = Ws18AmoledDriver::new(lcd_spi);

    println!("APP: SPI2/QSPI + I2C setup done");

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

    // Prefer a file whose base starts with "NO"; else first one
    let mut idx: usize = 0;
    for (i, name) in movies.iter().enumerate() {
        if name.base_name().starts_with(b"NO") { idx = i; break; }
    }

    println!("Playing movie: {} ({}x{} RGB565)", movies[idx], RAW_W, RAW_H);
    let mut prev_pressed = false;

    loop {
        let Ok(file) = root_dir.open_file_in_dir(&movies[idx], SdMode::ReadOnly) else {
            println!("open_file_in_dir failed; sleeping");
            loop { Delay::new().delay_millis(1000); }
        };
        let mut advance = false;
        let mut eof = false;
        let mut y: i32;

        loop {
            // Prefetch first tile
            y = 0;
            let want0 = core::cmp::min(CHUNK_BYTES, FRAME_SZ);
            let n0 = match file.read(unsafe { &mut TILE_A[..want0] }) { Ok(n) => n, Err(e) => { println!("Read error: {:?}", e); advance = true; 0 } };
            if n0 == 0 { eof = true; break; }
            if n0 % BYTES_PER_LINE != 0 { println!("Unaligned tile ({} B)", n0); advance = true; break; }

            let mut use_a = true;
            let mut cur_len = n0;
            let mut remaining = FRAME_SZ.saturating_sub(n0);

            // Draw and ping‑pong until frame done
            loop {
                let (ptr, len) = if use_a { (unsafe { &TILE_A[..cur_len] }, cur_len) } else { (unsafe { &TILE_B[..cur_len] }, cur_len) };
                let y0 = y as u16;
                // Draw chunk at (0, y)
                let tile_h = (len / BYTES_PER_LINE) as u32;
                let raw = ImageRaw::<Rgb565>::new(ptr, RAW_W);
                // Draw only the first `tile_h` rows by slicing the buffer accordingly
                // ImageRaw consumes the whole slice; ensure `len` equals RAW_W * tile_h * 2
                // which is true by construction of our tile.
                let img = Image::new(&raw, Point::new(0, y0 as i32));
                let _ = img.draw(&mut display);
                let _ = display.flush();
                y += tile_h as i32;

                if remaining == 0 { break; }
                // Read next tile into the other buffer
                let want = core::cmp::min(CHUNK_BYTES, remaining);
                let dst = if use_a { unsafe { &mut TILE_B[..want] } } else { unsafe { &mut TILE_A[..want] } };
                let n = match file.read(dst) { Ok(n) => n, Err(e) => { println!("Read error: {:?}", e); advance = true; 0 } };
                if n == 0 { eof = true; break; }
                if n % BYTES_PER_LINE != 0 { println!("Unaligned tile ({} B)", n); advance = true; break; }
                remaining -= n;
                cur_len = n;
                use_a = !use_a;

                // Touch edge detect to switch movies
                match touch.touch1() {
                    Ok(ts) => {
                        let is_pressed = matches!(ts, TouchState::Pressed(_));
                        if is_pressed && !prev_pressed { advance = true; }
                        prev_pressed = is_pressed;
                    }
                    Err(_) => {}
                }
                if advance { break; }
            }

            if advance { break; }

            // If we reach here, one frame was drawn. Continue to next frame until EOF.
        }

        let _ = file.close();
        if eof || advance {
            idx = if advance { (idx + 1) % movies.len() } else { idx };
            println!("Next movie: {}", movies[idx]);
        }
    }
}