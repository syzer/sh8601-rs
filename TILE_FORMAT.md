# Tile-Based Video Format for Embedded Displays

## Overview

Tile-based encoding is a content-side optimization that achieves **2-10× effective FPS improvement** for animated content (especially cartoons) by only encoding changed 16×16 pixel tiles per frame.

## Format Specification

```
Header (8 bytes):
  - Magic: "TILE" (4 bytes)
  - Version: u16 (little-endian, currently 1)
  - Frame count: u16 (little-endian)

Frame Offsets Table (4 × frame_count bytes):
  - Array of u32 offsets (little-endian) pointing to each frame's data

Per Frame:
  - num_tiles: u16 (little-endian) - number of changed tiles
  - For each tile:
    - tile_x: u8 (0-22 for 368px width)
    - tile_y: u8 (0-27 for 448px height)
    - tile_data_len: u16 (little-endian) - compressed tile size
    - tile_data: bytes - RLE-compressed 16×16 RGB565 tile (512 bytes uncompressed)
```

## Tile Encoding

- **Tile size**: 16×16 pixels
- **Pixel format**: RGB565 (big-endian, MSB first)
- **Uncompressed size**: 512 bytes per tile (16×16×2)
- **Compression**: Simple RLE (Run-Length Encoding)
  - `0xFF <value> <count>`: RLE run
  - `0xFE <count> <data...>`: Literal run

## Usage

### 1. Convert MJPEG to Tile Format

```bash
# Convert existing MJPEG file
just convert-tiles assets/mjpeg/video.mjpeg assets/tiles/video.tiles 368 448
```

Or directly:
```bash
python3 convert_mjpeg_to_tiles.py input.mjpeg output.tiles 368 448
```

### 2. Use in Rust Code

```rust
use sh8601_rs::tile_decoder::TileDecoder;
use alloc::collections::BTreeMap;

// Include tile file
static TILE_VIDEO: &[u8] = include_bytes!("../assets/tiles/video.tiles");

// Create decoder
let decoder = TileDecoder::new(TILE_VIDEO, 368, 448)?;

// Framebuffer (368×448×2 bytes for RGB565)
let mut framebuffer = [0u16; 368 * 448];

// Previous frame tiles cache
let mut prev_tiles = BTreeMap::new();

// Decode frame
let tiles_decoded = decoder.decode_frame(frame_idx, &mut framebuffer, &mut prev_tiles)?;
```

## Performance Benefits

### For Cartoons/Animation:
- **Typical savings**: Only 5-20% of tiles change per frame
- **Data reduction**: 80-95% less data per frame
- **Effective FPS**: 2-10× improvement (same bandwidth, higher frame rate)

### For Full-Motion Video:
- **Typical savings**: 40-60% of tiles change per frame  
- **Data reduction**: 40-60% less data per frame
- **Effective FPS**: 1.5-2.5× improvement

## Example Workflow

1. **Prepare video**: Convert MP4 to MJPEG first
   ```bash
   just convert-mjpeg assets/mp4/video.mp4 assets/mjpeg/video.mjpeg
   ```

2. **Convert to tiles**: Extract and encode changed tiles
   ```bash
   just convert-tiles assets/mjpeg/video.mjpeg assets/tiles/video.tiles
   ```

3. **Embed in firmware**: Include tile file in Rust code
   ```rust
   static VIDEO: &[u8] = include_bytes!("../assets/tiles/video.tiles");
   ```

4. **Decode on-device**: Use tile decoder in embedded code

## Format Limitations

- **First frame**: All tiles must be encoded (no previous frame to diff against)
- **Frame dependency**: Each frame depends on previous frame state
- **Memory**: Requires storage for previous frame's tile cache
- **Best for**: Content with minimal per-frame changes (cartoons, UI animations)

## Future Improvements

- [ ] LZ4 compression option (faster decompression)
- [ ] 8×8 tile option for finer granularity
- [ ] Delta encoding within tiles (only encode changed pixels)
- [ ] Keyframe support (periodic full frames)

