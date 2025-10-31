#![no_std]
#![no_main]

use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetDriver, Sh8601Driver, Ws18AmoledDriver,
    DMA_CHUNK_SIZE, tile_decoder::TileDecoder,
};

use alloc::collections::BTreeMap;
use heapless::Vec;

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
const TILE_SIZE: usize = 16; // 16x16 tiles for v1 stream
const TILES_X: usize = ((W as usize) + TILE_SIZE - 1) / TILE_SIZE;
const TILES_Y: usize = ((H as usize) + TILE_SIZE - 1) / TILE_SIZE;
const MAX_DIRTY_TILES_CONST: usize = TILES_X * TILES_Y; // 23*28 = 644 for 368x448@16

// Small ping-pong line buffers in DRAM0 for DMA bursts (full display width)
#[repr(align(64))]
struct AlignedU16<const N: usize> { data: [u16; N] }

#[allow(static_mut_refs)]
#[link_section = ".dram0.bss"]
static mut A: AlignedU16<{W as usize}> = AlignedU16 { data: [0; W as usize] };

#[allow(static_mut_refs)]
#[link_section = ".dram0.bss"]
static mut B: AlignedU16<{W as usize}> = AlignedU16 { data: [0; W as usize] };

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // PSRAM allocator first
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    // --- Input stream (tile v1) ---
    static TILE_VIDEO: &[u8] = include_bytes!("../assets/tiles/Yasin_no_cow.tiles");
    let decoder = match TileDecoder::new(TILE_VIDEO, W, H) {
        Ok(d) => { println!("Tile decoder initialized: {} frames", d.frame_count()); d }
        Err(e) => { println!("Tile decoder init error: {}", e); loop {} }
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

    // --- Framebuffer (RGB565 LE; SH8601 wants MSB first, SPI LSB-first flips it on the wire) ---
    let fb = display.framebuffer_mut();
    let fb_u16: &mut [u16] = unsafe {
        core::slice::from_raw_parts_mut(fb.as_mut_ptr() as *mut u16, (W as usize)*(H as usize))
    };

    // Previous-tile cache for the decoder (v1)
    let mut prev_tiles = BTreeMap::<(u8,u8), alloc::vec::Vec<u8>>::new();

    // Reusable dirty tile list (outside loop, cleared each frame)
    // Capacity == worst-case tiles to avoid dropping rectangles on keyframes.
    const MAX_DIRTY_TILES: usize = MAX_DIRTY_TILES_CONST;
    let mut dirty: Vec<(u8, u8), MAX_DIRTY_TILES> = Vec::new();

    // FPS
    let mut t0 = Instant::now();
    let mut frames = 0u32;
    let mut frame_idx = 0usize;

    loop {
        // Clear dirty list for this frame (reuse allocation)
        dirty.clear();

        // Decode tiles into FB and collect coords per-frame
        let res = decoder.decode_frame_with_callback(
            frame_idx,
            &mut prev_tiles,
            |tx, ty, tile| {
                // Blit tile into fb_u16
                let x0 = (tx as usize) * TILE_SIZE;
                let y0 = (ty as usize) * TILE_SIZE;
                for yy in 0..TILE_SIZE {
                    let y = y0 + yy;
                    if y >= H as usize { break; }
                    let fb_row = &mut fb_u16[y * (W as usize) ..];
                    let src_row = &tile[yy * TILE_SIZE .. (yy+1) * TILE_SIZE];
                    let len = core::cmp::min(TILE_SIZE, (W as usize) - x0);
                    fb_row[x0 .. x0 + len].copy_from_slice(&src_row[..len]);
                }
                if dirty.push((tx, ty)).is_err() {
                    // Vec full (shouldn't happen with 400 capacity), but handle gracefully
                }
            }
        );

        if res.is_err() {
            if frame_idx % 10 == 0 { println!("Decode error @ frame {}", frame_idx); }
            frame_idx = (frame_idx + 1) % num_frames;
            continue;
        }

        // Quick wins: sort dirty tiles (heapless Vec is fast, no heap allocations)
        dirty.sort_unstable_by(|a,b| a.1.cmp(&b.1).then(a.0.cmp(&b.0)));

        // Coalesce horizontally then vertically into rectangles
        let fb_w = W as usize;
        let mut i = 0usize;

        while i < dirty.len() {
            // Step 1: Coalesce horizontally (same ty, consecutive tx)
            let (start_tx, ty) = dirty[i];
            let mut end_tx = start_tx;
            let mut j = i + 1;
            while j < dirty.len() && dirty[j].1 == ty && dirty[j].0 == end_tx + 1 {
                end_tx += 1;
                j += 1;
            }

            // Step 2: Coalesce vertically - check if next horizontal runs can merge
            let mut end_ty = ty;
            let mut k = j;
            
            // Look ahead for adjacent rows with identical x_start/x_end
            while k < dirty.len() {
                // Check if next run starts on adjacent row (ty + 1)
                if dirty[k].1 != end_ty + 1 {
                    break;
                }
                
                // Check if next run has same x bounds
                let (next_start_tx, next_ty) = dirty[k];
                if next_start_tx != start_tx {
                    break;
                }
                
                // Check if this run extends to same end_tx
                let mut next_end_tx = next_start_tx;
                let mut next_k = k + 1;
                while next_k < dirty.len() && dirty[next_k].1 == next_ty && dirty[next_k].0 == next_end_tx + 1 {
                    next_end_tx += 1;
                    next_k += 1;
                }
                
                if next_end_tx != end_tx {
                    break; // Different width, can't merge
                }
                
                // This row matches - merge it
                end_ty = next_ty;
                k = next_k;
            }

            // Calculate rectangle bounds (coalesced horizontally and vertically)
            let x_start = (start_tx as usize) * TILE_SIZE;
            let x_end   = ((end_tx as usize + 1) * TILE_SIZE - 1).min(fb_w - 1);
            let y_start = (ty as usize) * TILE_SIZE;
            let y_end   = ((end_ty as usize + 1) * TILE_SIZE - 1).min(H as usize - 1);

            // One set_window() per coalesced rectangle
            if display.set_window(x_start as u16, y_start as u16, x_end as u16, y_end as u16).is_err() {
                i = k; continue;
            }

            let rect_w = x_end - x_start + 1;
            let rect_h = y_end - y_start + 1;
            let mut use_a = true;

            // Stream rows via DMA ping-pong (A/B)
            // RAMWR on first row, RAMWRC on the rest
            for row in 0..rect_h {
                let y = y_start + row;
                if y > y_end { break; }

                let src = &fb_u16[y * fb_w + x_start .. y * fb_w + x_start + rect_w];
                unsafe {
                    let dst = if use_a { &mut A.data[..rect_w] } else { &mut B.data[..rect_w] };
                    dst.copy_from_slice(src);
                    
                    // First row of rectangle = RAMWR, subsequent rows = RAMWRC
                    let is_first_chunk = row == 0;
                    let _ = display.write_pixels_dma_u16(dst, is_first_chunk);
                }
                use_a = !use_a; // Ping-pong: swap buffers for next row
            }
            // Ensure DMA is idle before changing window for next rectangle.
            // If the driver exposes a blocking flush/idle API, prefer that; otherwise this is a no-op.
            #[allow(unused_must_use)]
            {
                // Try common method names in sh8601-rs:
                #[cfg(any())]
                display.flush_dma();
                #[cfg(any())]
                display.wait_for_dma_idle();
            }

            i = k; // Skip all merged rows
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