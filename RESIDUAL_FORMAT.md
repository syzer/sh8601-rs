# Residual Encoding Format (XOR + Zero-RLE + LZ4HC)

## Overview

Alternative to tile-based encoding for animation. Encodes **XOR residuals** between consecutive frames, optimized for cartoon/flat content with large unchanged areas.

## Rationale

**Problem with tiles:** 
- Overhead from dirty tile tracking, coalescing, multiple `set_window()` calls
- Works well for sparse changes, less efficient for large moving objects

**Residual approach:**
- XOR produces mostly zeros for unchanged pixels
- Zero-RLE compresses zeros efficiently
- LZ4HC provides final size reduction
- Decoder is simpler: decompress → expand → XOR

## File Format

### Header (12 bytes)
```
Offset  Size    Description
0x00    4       Magic: "RSDL"
0x04    2       Version: 1 (u16, little-endian)
0x06    2       Frame count (u16, little-endian)
0x08    2       Width (u16, little-endian)
0x0A    2       Height (u16, little-endian)
```

### Frame Offsets Table
```
Offset  Size              Description
0x0C    4 × frame_count   u32 offsets to each frame (little-endian)
```

### Frame Data
For each frame at offset:
```
Offset  Size              Description
+0      4                 Compressed size (u32, little-endian)
+4      compressed_size   LZ4HC(Zero-RLE(XOR residual))
```

**First frame:** Stored as full frame (no XOR, just Zero-RLE + LZ4HC)
**Subsequent frames:** XOR with previous decoded frame

## Encoding Process

### 1. Compute XOR Residual
```python
residual[i] = curr_frame[i] ^ prev_frame[i]  # byte-wise XOR
```
For unchanged pixels: `curr ^ prev = 0`

### 2. Zero-RLE Encoding (on u16 words)

**Format:**
- `[type_byte] [data...]`

**Type byte:**
- `0x00-0x7F`: Literal run of N u16 words (1-127)
- `0x80-0xFE`: Zero run of (type_byte & 0x7F) u16 words (1-127)
- `0xFF + u16`: Extended zero run of u16 count (for runs > 127)

**Example:**
```
Input (u16):  [0x0000, 0x0000, 0x0000, 0x1234, 0x5678, 0x0000]
Encoded:      [0x83, 0x02, 0x12, 0x34, 0x56, 0x78, 0x81]
              ^      ^                           ^
              |      |                           |
              3 zeros  2 literals                1 zero
```

### 3. LZ4HC Compression
```python
lz4.block.compress(rle_data, mode='high_compression', compression=12)
```
Uses highest compression for offline encoding (level 12).

## Decoding Process (Rust)

```rust
// 1. LZ4 decompress
let rle_data = lz4_flex::decompress(compressed_frame, max_size)?;

// 2. Zero-RLE expand
let residual = zero_rle_decode(&rle_data)?;

// 3. XOR apply
for i in 0..framebuffer.len() {
    framebuffer[i] ^= residual[i];
}
```

## Expected Performance

### Compression (cartoons with flat areas):
- **XOR residual:** 70-90% zeros (unchanged pixels)
- **Zero-RLE:** 5-10× compression on zeros
- **LZ4HC:** Additional 2-3× on patterns
- **Combined:** 3-6× total compression

### Decode Speed:
- **LZ4 decompress:** ~400 MB/s on ESP32-S3
- **Zero-RLE expand:** Mostly `memset(0)`, very fast
- **XOR apply:** Single pass, cache-friendly

### Memory:
- **Temp buffer:** One frame worth (~330 KB in PSRAM)
- **Working space:** LZ4 scratch space (~64 KB)
- **No DRAM0 needed:** Direct framebuffer updates

## Comparison vs Tiles

| Aspect | Tiles (32×32) | Residuals |
|--------|---------------|-----------|
| **Best for** | Sparse changes | Large moving objects |
| **Overhead** | Tile tracking, coalescing | Decompress + XOR |
| **Complexity** | High (dirty map, rectangles) | Low (linear decode) |
| **Decode time** | Varies (dirty count) | Constant (full frame) |
| **Memory** | 16KB DRAM0 + BTreeMap | ~64KB PSRAM temp |

## Usage

### Encode:
```bash
python3 convert_mjpeg_to_residuals.py \
    input.mjpeg \
    output.residuals \
    368 448 \
    5  # max frames
```

### Decode (Rust):
```rust
let decoder = ResidualDecoder::new(RESIDUAL_DATA, W, H)?;
for frame_idx in 0..decoder.frame_count() {
    decoder.decode_frame(frame_idx, &mut framebuffer)?;
    display.flush()?;
}
```

## Trade-offs

### Choose Residuals if:
- ✓ Content has large moving objects (characters walking, etc.)
- ✓ Many pixels change per frame
- ✓ Want simpler decoder logic
- ✓ CPU cycles available for LZ4 + XOR

### Choose Tiles if:
- ✓ Content has sparse, localized changes (UI elements, small movements)
- ✓ Want to skip unchanged regions entirely
- ✓ Display commands are expensive (many `set_window()` calls OK)
- ✓ Bandwidth is the bottleneck

## Testing Both

Run comparison:
```bash
# Encode both formats
just convert-tiles assets/mjpeg/video.mjpeg '' '368' '448' '5' '32'
python3 convert_mjpeg_to_residuals.py assets/mjpeg/video.mjpeg assets/residuals/video.residuals 368 448 5

# Compare file sizes
ls -lh assets/tiles/video.tiles assets/residuals/video.residuals

# Measure FPS on device
# (implement both decoders and compare)
```

## Implementation Status

- [X] Python encoder (`convert_mjpeg_to_residuals.py`)
- [ ] Rust decoder (`src/residual_decoder.rs`)
- [ ] Benchmark comparison
- [ ] Document winner in TODO.md

