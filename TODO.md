# TODO: Optimize Tile Playback Performance (SH8601 / ESP32-S3)

Current: ~3 FPS  
Goal: ≥ 6 FPS stable on 368×448 (RGB565, tile-encoded)

---

## 🔧 1. Replace Full-Frame Flush
- [X] Stop calling `begin_frame(0,0,W,H)` + `end_frame()` each frame.
  - [X] No full-frame flush - removed `begin_frame`/`end_frame` calls
- [X] Implement **per-rectangle push** (lines 250-284):
  - [X] `display.set_window(x,y,x+w-1,y+h-1)` per coalesced rectangle
  - [X] Copy into DRAM0 ping-pong buffers (A/B) in `.dram0.bss`
  - [X] `display.write_pixels_dma_u16(tile_buf, is_first_chunk)` immediately
  - [X] RAMWR on first chunk, RAMWRC on subsequent chunks
- [X] Advanced coalescing (lines 201-242):
  - [X] Horizontal: merge consecutive tiles on same row
  - [X] Vertical: merge adjacent rows with identical x-bounds
  - [X] Result: minimal `set_window()` calls, larger DMA bursts

---

## ⚙️ 2. Remove `BTreeMap` from Hot Path
- [ ] Use flat **previous-frame buffer** in PSRAM (`Box<[u8]>`).
- [ ] Maintain a small **dirty bitmap** in DRAM0.
- [ ] Encoder should mark changed tiles directly → no per-tile `BTreeMap` lookup.

---

## ⚡ 3. Optimize DMA Path
- [X] Keep **two 8KB (4096 px)** buffers in `.dram0.bss` for ping-pong DMA.
  - [X] Upgraded from 512B to 8KB per buffer (16× larger!)
  - [X] Can send ~5 full scanlines (368px × 5) in a single chunk
  - [X] Reduces DMA overhead by ~16× compared to 256px buffers
- [X] **Upgraded to 32×32 tiles** (from 16×16):
  - [X] 4× fewer tiles overall (12×14 = 168 vs 23×28 = 644)
  - [X] Larger bursts per tile (2KB vs 512B)
  - [X] Less overhead (fewer dirty checks, fewer set_window calls)
  - [X] 1.6MB for 5 frames (fits in flash, ~320KB per frame)
- [X] Align each buffer to **64 bytes** for cache line safety.
- [X] Avoid allocations and `println!` inside the render loop.
- [X] Call `write_pixels_dma_u16()` immediately after copying tile data.
- [X] Use `heapless::Vec` for dirty tiles (no heap allocations).
- [X] Reuse dirty vec across frames with `clear()`.

---

## 🧠 4. Endianness & Data Path
- [ ] Ensure tile decoder emits **native u16 RGB565**.
- [ ] Confirm SPI driver transmits **MSB-first** on wire.
- [ ] If not, do a single `swap_bytes()` per tile buffer (not per pixel).

---

## 🧩 5. Coalescing Strategy
- [X] Merge horizontally adjacent dirty tiles on same `tile_y`.
- [X] Merge vertically adjacent rows with identical x-bounds into rectangles.
- [X] Send them in a single `set_window()` + `RAMWR`/`RAMWRC` burst per rectangle.
- [X] RAMWR on first chunk, RAMWRC on subsequent chunks.
- [X] Handle rectangles wider than tile buffer by chunking within rows.
- [X] Reduces per-tile command overhead significantly.

---

## 🕐 6. FPS & Logging
- [X] Track FPS using `Instant` (already done).
- [ ] Print once per second only — remove frame-by-frame logs.

---

## 🧱 7. Memory Layout
- [X] Keep large data (frame cache, tiles) in **PSRAM**.
  - [X] Framebuffer (368×448×2 = 330KB) allocated via `new_heap()` in PSRAM
  - [X] `prev_tiles` BTreeMap in PSRAM (heap-allocated)
  - [X] TILE_VIDEO constant in flash (`.rodata`)
- [X] Keep DMA buffers in **DRAM0** for fast access.
  - [X] Two 8KB DMA buffers (4096 pixels each) in `.dram0.bss`
  - [X] Total: 16KB for ping-pong buffering (upgraded from 1KB → 16× larger!)
  - [X] `dirty` heapless::Vec (stack-allocated, ~2.6KB max) in DRAM
- [X] Mark hot functions with `#[link_section = ".iram1.text"]`.
  - [X] `blit_tile_hot()` - tile blitting to framebuffer
  - [X] `render_rectangles_hot()` - coalescing and DMA streaming
  - [X] `write_pixels_dma_u16()` in driver (already in IRAM)

---

## 🧮 8. Encoder Tweaks
- [ ] Verify tile encoder marks only **changed tiles**.
- [X] Consider larger tile size (e.g. 32×32) → **Done: using 32×32 tiles**
- [ ] For static backgrounds, raise diff-threshold to minimize tile churn.

---

## 🔬 9. Residual Encoding (Alternative Approach - Experimental)

**See `RESIDUAL_FORMAT.md` for full details**

### Concept:
Encode **XOR residuals** instead of tiles:
1. `residual = curr_frame ^ prev_frame` (byte-wise XOR on RGB565)
2. Zero-RLE encode (most pixels = 0x0000 for unchanged areas)
3. LZ4HC compress final result

### Expected Benefits:
- **3-6× compression** on cartoons (large flat/unchanged areas)
- **Simpler decoder**: LZ4 decompress → Zero-RLE expand → XOR apply
- **No tile overhead**: No dirty tracking, coalescing, or multiple set_window() calls
- **Cache-friendly**: Single linear pass over framebuffer

### Expected Drawbacks:
- **Full-frame decode**: Must decode entire frame even for small changes
- **LZ4 CPU cost**: Decompression overhead (~400 MB/s on ESP32-S3)
- **Works best for**: Large moving objects, not sparse UI changes

### Implementation Status:
- [X] Created `convert_mjpeg_to_residuals.py` encoder (XOR + Zero-RLE + LZ4HC)
- [X] Created `RESIDUAL_FORMAT.md` documentation
- [X] Install lz4: `pip install lz4` ✓
- [X] Test encode: `just convert-residuals assets/mjpeg/Yasin_no_cow.mjpeg` ✓
- [X] Implement Rust decoder in `src/residual_decoder.rs` ✓
- [X] Create example: `examples/ws_18in_amoled_residuals.rs` (130 lines vs 298 for tiles!)
- [X] Add justfile commands: `just run-residuals` and `just release-residuals`
- [ ] **TEST ON DEVICE:** Benchmark decode time and FPS
- [ ] **Decision:** Compare both, keep winner (or keep both as options)

### 🎉 Encoding Results (5 frames, 368×448):

| Metric | Tiles (32×32) | Residuals | Winner |
|--------|---------------|-----------|--------|
| **File size** | 1.6 MB | 0.83 MB | **Residuals (1.9× smaller)** |
| **Avg/frame** | ~320 KB | ~174 KB | **Residuals** |
| **Frame 1** | - | 203 KB (1.6× - keyframe) | - |
| **Frame 2** | - | 104 KB (3.2× - best) | - |
| **Frames 3-5** | - | 176-202 KB (1.6-1.9×) | - |

**Residuals win on file size!** Now need to test decode speed on device.

### Quick Comparison Command:
```bash
# Encode both formats (5 frames)
just convert-tiles assets/mjpeg/Yasin_no_cow.mjpeg
just convert-residuals assets/mjpeg/Yasin_no_cow.mjpeg

# Compare sizes
ls -lh assets/tiles/Yasin_no_cow.tiles
ls -lh assets/residuals/Yasin_no_cow.residuals
```

---

## ✅ Expected Result
After implementing the above:
- Dirty-tile DMA only (~20–40 KB/frame typical).
- No BTreeMap overhead.
- Fewer SPI commands.
- 5–7 FPS typical, 8+ FPS peak on 80 MHz QSPI (368×448 RGB565).
- **OR** residual encoding if it proves faster/smaller.

---