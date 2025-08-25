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

use zune_jpeg::JpegDecoder;

// --- Touch (FT3168) ---
use embedded_hal_bus::{i2c, util::AtomicCell};
use ft3x68_rs::{Ft3x68Driver, FT3168_DEVICE_ADDRESS, ResetInterface};

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
const RAW_W: u32 = 368;
const RAW_H: u32 = 448;

// Tunables
const LCD_SPI_MHZ: u32 = 80; // SPI2 (QSPI panel link)
const SD_SPI_MHZ: u32  = 48; // SPI3 (SD card link)
// const SD_SPI_MHZ: u32  = 28; // SPI3 (SD card link)

esp_app_desc!();

/// Find [start,end) byte ranges for JPEG frames in an MJPEG AVI chunk
fn index_jpegs(buf: &[u8], max_frames: usize) -> alloc::vec::Vec<(usize,usize)> {
    let mut out = alloc::vec::Vec::new();
    let mut i = 0;
    while i + 3 < buf.len() && out.len() < max_frames {
        if buf[i] == 0xFF && buf[i+1] == 0xD8 && buf[i+2] == 0xFF { // SOI + next marker
            let start = i;
            // search for EOI 0xFF 0xD9
            let mut j = i+3;
            while j + 1 < buf.len() {
                if buf[j] == 0xFF && buf[j+1] == 0xD9 {
                    let end = j+2;
                    out.push((start,end));
                    i = end; // continue after this frame
                    break;
                }
                j += 1;
            }
        }
        i += 1;
    }
    out
}

/// Pack RGB888 bytes into RGB565 words (big-endian 16bpp expected by driver)
fn rgb888_to_rgb565_be(src: &[u8], dst: &mut [u16]) {
    // src len must be 3*w*h, dst len must be w*h
    for (pix, chunk) in dst.iter_mut().zip(src.chunks_exact(3)) {
        let r = chunk[0] as u16;
        let g = chunk[1] as u16;
        let b = chunk[2] as u16;
        // 5-6-5 packing
        let v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
        *pix = v; // store as native u16; driver will send big-endian on wire
    }
}

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
            .with_frequency(Rate::from_mhz(LCD_SPI_MHZ))
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
            .with_frequency(Rate::from_mhz(SD_SPI_MHZ)) // start conservative; can try 40–60 later
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

    println!("APP: Probing SD (SPI3 {}MHz, Mode0)...", SD_SPI_MHZ);
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
    // Find exactly one file: VALEN~37.AVI (skip names starting with '_')
    let mut selected: Option<ShortFileName> = None;
    let mut scanned = 0usize;
    let _ = root_dir.iterate_dir(|e| {
        if !e.attributes.is_directory() && e.name.extension() == b"AVI" {
            scanned += 1;
            if selected.is_some() {
                return; // already picked one
            }
            let base = e.name.base_name();
            if base.get(0).copied() != Some(b'_') {
                selected = Some(e.name.clone());
            }
        }
    });

    let Some(chosen) = selected else {
        println!(".AVI not found in root (scanned {} entries)", scanned);
        loop { Delay::new().delay_millis(1000); }
    };

    println!("Playing movie (MJPEG AVI): {}", chosen);

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

    // Open and preload some/all bytes (cap to PSRAM budget for now)
    let Ok(file) = root_dir.open_file_in_dir(&chosen, SdMode::ReadOnly) else {
        println!("open_file_in_dir failed; sleeping");
        loop { Delay::new().delay_millis(1000); }
    };

    // Cap preload (debug): 4 MiB if available, otherwise 512 KiB
    // let mut preload_cap = 4 * 1024 * 1024usize;
    let mut preload_cap = 256 * 1024usize;
    if preload_cap > 4*1024*1024 { preload_cap = 4*1024*1024; }
    // Try to allocate PSRAM buffer
    let mut avi_buf: alloc::vec::Vec<u8> = alloc::vec::Vec::with_capacity(preload_cap);
    unsafe { avi_buf.set_len(0); }

    let mut total = 0usize;
    loop {
        let mut chunk = [0u8; 8192];
        match file.read(&mut chunk) {
            Ok(0) => break,
            Ok(n) => {
                let want = core::cmp::min(n, preload_cap.saturating_sub(total));
                if want == 0 { break; }
                avi_buf.extend_from_slice(&chunk[..want]);
                total += want;
                if total % (256*1024) == 0 { println!("  preloading... {} KiB", total/1024); }
                if total >= preload_cap { break; }
            }
            Err(e) => { println!("SD read error: {:?}", e); break; }
        }
    }
    let _ = file.close();
    println!("Movie preload done: {} bytes into PSRAM", avi_buf.len());

    // Find JPEG frames in the preloaded data
    let frames = index_jpegs(&avi_buf, 512);
    println!("Indexed {} JPEG frame(s)", frames.len());
    if frames.is_empty() { loop { Delay::new().delay_millis(1000); } }

    // One-time RGB565 buffer for decoded frames
    let mut rgb565: alloc::vec::Vec<u16> = alloc::vec::Vec::new();
    rgb565.resize((RAW_W as usize)*(RAW_H as usize), 0);

    // Playback loop
    let mut fidx = 0usize;
    loop {
        let (s,e) = frames[fidx];
        let jpeg = &avi_buf[s..e];

        // Decode JPEG → RGB24 using zune-jpeg
        let mut dec = JpegDecoder::new(jpeg);
        let pixels = match dec.decode() {
            Ok(p) => p,
            Err(_e) => { println!("JPEG decode failed; skipping frame"); fidx = (fidx+1)%frames.len(); continue; }
        };
        let info = match dec.info() { Some(i) => i, None => { println!("JPEG info missing"); fidx = (fidx+1)%frames.len(); continue; } };
        if info.width as u32 != RAW_W || info.height as u32 != RAW_H { println!("Unexpected frame size {}x{}", info.width, info.height); }

        // Convert RGB888 → RGB565 (big-endian on wire handled by driver)
        rgb888_to_rgb565_be(&pixels, &mut rgb565);

        // Present using embedded-graphics
        let bytes: &[u8] = unsafe { core::slice::from_raw_parts(rgb565.as_ptr() as *const u8, rgb565.len()*2) };
        let raw = ImageRaw::<Rgb565>::new(bytes, RAW_W);
        let img = Image::new(&raw, Point::new(0,0));
        let _ = img.draw(&mut display);
        let _ = display.flush();

        fidx = (fidx+1) % frames.len();
    }
}