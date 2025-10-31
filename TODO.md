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
- [X] Keep **two 16×16 (256 px)** buffers in `.dram0.bss` for ping-pong DMA.
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
- [X] Keep small DMA buffers and code in **DRAM0 / IRAM**.
  - [X] Two 16×16 tile buffers (512 bytes each) in `.dram0.bss`
  - [X] `dirty` heapless::Vec (stack-allocated, ~2.6KB max) in DRAM
- [X] Mark hot functions with `#[link_section = ".iram1.text"]`.
  - [X] `blit_tile_hot()` - tile blitting to framebuffer
  - [X] `render_rectangles_hot()` - coalescing and DMA streaming
  - [X] `write_pixels_dma_u16()` in driver (already in IRAM)

---

## 🧮 8. Encoder Tweaks
- [ ] Verify tile encoder marks only **changed tiles**.
- [ ] Consider larger tile size (e.g. 32×16) if scenes have full-screen motion.
- [ ] For static backgrounds, raise diff-threshold to minimize tile churn.

---

## ✅ Expected Result
After implementing the above:
- Dirty-tile DMA only (~20–40 KB/frame typical).
- No BTreeMap overhead.
- Fewer SPI commands.
- 5–7 FPS typical, 8+ FPS peak on 80 MHz QSPI (368×448 RGB565).

---