#![no_std]
#![no_main]

use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    DMA_CHUNK_SIZE,
};

use embedded_graphics::{
    mono_font::{
        ascii::{FONT_10X20},
        MonoTextStyle,
    },
    pixelcolor::{Rgb888},
    prelude::*,
    primitives::{PrimitiveStyleBuilder},
    text::{Alignment, LineHeight, Text, TextStyleBuilder},
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
// Adjust the bitmask if your wiring differs; here we toggle P2.

// NOTE: Verify TOUCH_RST_BIT and TCA9554_ADDR against the Waveshare schematic.
// Never write whole-byte constants to 0x01/0x03 or you will clobber LCD DC/BL/EN lines and blank the screen.
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

const W: u32 = 368;
const H: u32 = 448;

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Puts the image into the firmware. One 368×448 RGB888 image is ~496 KB; RGB565 is ~330 KB.
    // If you’ll show many images, don’t embed—load from SD or SPI flash.
    static IMG1: &[u8] = include_bytes!("../assets/rgb/pic_1_368x448.rgb");
    static IMG2: &[u8] = include_bytes!("../assets/rgb/pic_2_368x448.rgb");
    static IMG3: &[u8] = include_bytes!("../assets/rgb/pic_3_368x448.rgb");

    // Image playlist and state
    let images: [&[u8]; 3] = [IMG1, IMG2, IMG3];
    let mut img_idx: usize = 0;
    let mut prev_pressed = false; // simple edge detection

    // Prebuild ImageRaw objects once at startup (avoid per-touch construction)
    let raws: [ImageRaw<'static, Rgb888>; 3] = [
        ImageRaw::<Rgb888>::new(images[0], W),
        ImageRaw::<Rgb888>::new(images[1], W),
        ImageRaw::<Rgb888>::new(images[2], W),
    ];

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let delay = Delay::new();

    // --- DMA Buffers for SPI ---
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_CHUNK_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // SPI Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    // Hardware is configured for QSPI. Pinout obtained from the schematic.
    // Schematic:
    // https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf
    // Using DMA for more efficient SPI communication.
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

    // Instantiare and Initialize Display
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

    let character_style = MonoTextStyle::new(&FONT_10X20, Rgb888::WHITE);

    let text_style = TextStyleBuilder::new()
        .line_height(LineHeight::Pixels(300))
        .alignment(Alignment::Center)
        .build();

    let text = "Cow Enabled!";

    let style = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb888::RED)
        .stroke_width(3)
        .fill_color(Rgb888::GREEN)
        .build();

    // Draw overlay text
    Text::with_text_style(text, Point::new(100, 100), character_style, text_style)
        .draw(&mut display)
        .unwrap();

    // Draw initial image (index 0)
    {
        if let Err(_e) = Image::new(&raws[img_idx], Point::new(0, 0)).draw(&mut display) {
            println!("Error drawing image");
        }
        let _ = display.flush().ok();
    }
    loop {

        // Poll touch and on a new press advance to next image
        match touch.touch1() {
            Ok(TouchState::Pressed(p)) => {
                if !prev_pressed {
                    // edge: Released -> Pressed
                    img_idx = (img_idx + 1) % images.len();
                    if let Err(_e) = Image::new(&raws[img_idx], Point::new(0, 0)).draw(&mut display) {
                        println!("Error drawing image");
                    }
                    // bit shit, but will do7
                    if let Err(e) = display.flush() {
                        println!("Error flushing display {:?}", e);
                    }
                }
                prev_pressed = true;
            }
            Ok(TouchState::Released) => {
                prev_pressed = false;
            }
            Err(_e) => {
                // ignore transient I2C errors
            }
        }

        delay.delay_millis(10);
    }
}
