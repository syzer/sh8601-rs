#!/usr/bin/env python3
"""
Convert MJPEG video to tile-based format with dirty tile encoding.
Only sends 16×16 RGB565 tiles that changed between frames.
Uses RLE compression for tile data.

Format:
- Header: "TILE" magic, version (u16), frame_count (u16)
- Frame offsets table: u32[frame_count]
- Per frame:
  - num_tiles (u16): number of changed tiles
  - For each tile:
    - tile_x (u8): tile X coordinate (0-22 for 368 width)
    - tile_y (u8): tile Y coordinate (0-27 for 448 height)
    - tile_data_len (u16): compressed tile data length
    - tile_data (bytes): RLE-compressed 16×16 RGB565 tile (512 bytes uncompressed)

Tile size: 16×16 pixels × 2 bytes (RGB565) = 512 bytes per tile
Frame size: 368×448 = 23×28 tiles
"""

import struct
import sys
from PIL import Image
import io

TILE_SIZE = 32  # Default: 32×32 tiles (can be overridden via command line)
MAGIC = b"TILE"
VERSION = 1


def simple_rle_encode(data: bytes) -> bytes:
    """Simple RLE compression for tile data."""
    if len(data) == 0:
        return b""
    
    result = bytearray()
    i = 0
    while i < len(data):
        # Count consecutive identical bytes
        value = data[i]
        count = 1
        while i + count < len(data) and data[i + count] == value and count < 255:
            count += 1
        
        if count >= 3:  # RLE encode if 3+ repeats
            result.extend([0xFF, value, count])
            i += count
        else:  # Literal run
            # Find next RLE opportunity or end
            literal_start = i
            literal_count = 0
            while i + literal_count < len(data) and literal_count < 255:
                if i + literal_count + 2 < len(data):
                    if data[i + literal_count] == data[i + literal_count + 1] == data[i + literal_count + 2]:
                        break  # Found RLE opportunity
                literal_count += 1
            
            result.extend([0xFE, literal_count])
            result.extend(data[literal_start:literal_start + literal_count])
            i += literal_count
    
    return bytes(result)


def simple_rle_decode(data: bytes, expected_len: int) -> bytes:
    """Simple RLE decompression."""
    result = bytearray(expected_len)
    result_idx = 0
    i = 0
    
    while i < len(data) and result_idx < expected_len:
        if data[i] == 0xFF:  # RLE run
            if i + 2 >= len(data):
                break
            value = data[i + 1]
            count = data[i + 2]
            for _ in range(min(count, expected_len - result_idx)):
                result[result_idx] = value
                result_idx += 1
            i += 3
        elif data[i] == 0xFE:  # Literal run
            if i + 1 >= len(data):
                break
            literal_count = data[i + 1]
            if i + 2 + literal_count > len(data):
                literal_count = len(data) - i - 2
            if result_idx + literal_count > expected_len:
                literal_count = expected_len - result_idx
            result[result_idx:result_idx + literal_count] = data[i + 2:i + 2 + literal_count]
            result_idx += literal_count
            i += 2 + literal_count
        else:
            # Raw byte (shouldn't happen, but handle gracefully)
            if result_idx < expected_len:
                result[result_idx] = data[i]
                result_idx += 1
            i += 1
    
    # Pad if needed
    while result_idx < expected_len:
        result[result_idx] = 0
        result_idx += 1
    
    return bytes(result[:expected_len])


def rgb888_to_rgb565(r: int, g: int, b: int) -> int:
    """Convert RGB888 to RGB565 (big-endian format)."""
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def extract_frame_jpeg(mjpeg_data: bytes, frame_num: int, frame_offsets: list) -> bytes:
    """Extract a single JPEG frame from MJPEG data."""
    if frame_num >= len(frame_offsets):
        return None
    
    frame_start = frame_offsets[frame_num]
    frame_end = frame_offsets[frame_num + 1] if frame_num + 1 < len(frame_offsets) else len(mjpeg_data)
    
    return mjpeg_data[frame_start:frame_end]


def decode_jpeg_frame(jpeg_data: bytes) -> Image.Image:
    """Decode JPEG frame to PIL Image."""
    return Image.open(io.BytesIO(jpeg_data))


def get_tile_rgb565(image: Image.Image, tile_x: int, tile_y: int, width: int, height: int) -> bytes:
    """Extract TILE_SIZE×TILE_SIZE tile as RGB565 big-endian bytes."""
    tile_bytes = TILE_SIZE * TILE_SIZE * 2  # TILE_SIZE×TILE_SIZE×2 bytes
    tile = bytearray(tile_bytes)
    
    x_start = tile_x * TILE_SIZE
    y_start = tile_y * TILE_SIZE
    
    for y in range(TILE_SIZE):
        for x in range(TILE_SIZE):
            px_x = x_start + x
            px_y = y_start + y
            
            if px_x < width and px_y < height:
                pixel = image.getpixel((px_x, px_y))
                if isinstance(pixel, int):  # Grayscale
                    r = g = b = pixel
                else:  # RGB/RGBA
                    r, g, b = pixel[0], pixel[1], pixel[2]
            else:
                r = g = b = 0  # Padding
            
            rgb565 = rgb888_to_rgb565(r, g, b)
            # Big-endian (MSB first)
            idx = (y * TILE_SIZE + x) * 2
            tile[idx] = (rgb565 >> 8) & 0xFF
            tile[idx + 1] = rgb565 & 0xFF
    
    return bytes(tile)


def tiles_differ(tile1: bytes, tile2: bytes) -> bool:
    """Check if two tiles differ."""
    return tile1 != tile2


def convert_mjpeg_to_tiles(input_mjpeg: bytes, output_file: str, width: int, height: int, max_frames: int = None):
    """Convert MJPEG to tile-based format."""
    # Find frame offsets
    frame_offsets = []
    i = 0
    while i < len(input_mjpeg) - 1:
        if input_mjpeg[i] == 0xFF and input_mjpeg[i + 1] == 0xD8:
            frame_offsets.append(i)
            # Limit to max_frames if specified
            if max_frames is not None and len(frame_offsets) >= max_frames:
                break
            j = i + 2
            while j < len(input_mjpeg) - 1:
                if input_mjpeg[j] == 0xFF and input_mjpeg[j + 1] == 0xD9:
                    i = j + 2
                    break
                j += 1
            if j >= len(input_mjpeg) - 1:
                break
        else:
            i += 1
    
    num_frames = len(frame_offsets)
    if max_frames is not None:
        print(f"Found {num_frames} frames in MJPEG file (limited to {max_frames})")
    else:
        print(f"Found {num_frames} frames in MJPEG file")
    print(f"Frame size: {width}×{height}, Tiles: {width//TILE_SIZE}×{height//TILE_SIZE}")
    
    tiles_x = (width + TILE_SIZE - 1) // TILE_SIZE
    tiles_y = (height + TILE_SIZE - 1) // TILE_SIZE
    
    with open(output_file, 'wb') as f:
        # Write header
        f.write(MAGIC)
        f.write(struct.pack('<H', VERSION))  # version
        f.write(struct.pack('<H', num_frames))  # frame_count
        
        # Reserve space for frame offsets (will fill later)
        frame_offset_pos = f.tell()
        f.write(struct.pack(f'<{num_frames}I', *([0] * num_frames)))  # placeholder
        
        # Previous frame tiles for comparison
        prev_tiles = {}  # (tile_x, tile_y) -> tile_data
        
        frame_offset_table = []
        
        for frame_idx in range(num_frames):
            frame_offset_table.append(f.tell())
            
            print(f"Processing frame {frame_idx + 1}/{num_frames}...", end='\r')
            
            # Extract and decode JPEG frame
            jpeg_data = extract_frame_jpeg(input_mjpeg, frame_idx, frame_offsets)
            if jpeg_data is None:
                break
            
            try:
                img = decode_jpeg_frame(jpeg_data)
                # MJPEG should already be correct size (scaled/cropped during conversion)
                # Only resize if significantly different (handles minor rounding issues)
                if abs(img.size[0] - width) > 2 or abs(img.size[1] - height) > 2:
                    # Crop/scale to exact size if needed (avoid black bars)
                    if img.size[0] != width or img.size[1] != height:
                        # Use thumbnail for better quality when downscaling, or crop if upscaling
                        img.thumbnail((width, height), Image.LANCZOS)
                        # If still not exact, create new image and paste (centered crop)
                        if img.size != (width, height):
                            new_img = Image.new('RGB', (width, height), (0, 0, 0))
                            paste_x = (width - img.size[0]) // 2
                            paste_y = (height - img.size[1]) // 2
                            new_img.paste(img, (paste_x, paste_y))
                            img = new_img
            except Exception as e:
                print(f"\nError decoding frame {frame_idx}: {e}")
                # Write empty frame (0 tiles)
                f.write(struct.pack('<H', 0))
                continue
            
            # Find changed tiles
            changed_tiles = []
            current_tiles = {}
            
            for ty in range(tiles_y):
                for tx in range(tiles_x):
                    tile_data = get_tile_rgb565(img, tx, ty, width, height)
                    current_tiles[(tx, ty)] = tile_data
                    
                    # First frame: encode all tiles. Subsequent frames: only changed tiles
                    prev_tile = prev_tiles.get((tx, ty)) if frame_idx > 0 else None
                    if frame_idx == 0 or prev_tile is None or tiles_differ(tile_data, prev_tile):
                        # Tile changed or first frame, compress it
                        compressed = simple_rle_encode(tile_data)
                        changed_tiles.append((tx, ty, compressed))
            
            # Update previous frame tiles for next frame comparison
            prev_tiles = current_tiles
            
            # Write frame: num_tiles (u16), then tile data
            f.write(struct.pack('<H', len(changed_tiles)))
            
            for tx, ty, compressed_data in changed_tiles:
                f.write(struct.pack('<BB', tx, ty))  # tile coordinates
                f.write(struct.pack('<H', len(compressed_data)))  # compressed size
                f.write(compressed_data)  # tile data
            
            if (frame_idx + 1) % 10 == 0 or frame_idx == num_frames - 1:
                avg_tiles = sum(len(prev_tiles) for _ in range(frame_idx + 1)) / (frame_idx + 1)
                print(f"\nFrame {frame_idx + 1}: {len(changed_tiles)}/{tiles_x * tiles_y} tiles changed")
        
        # Write frame offsets table
        f.seek(frame_offset_pos)
        f.write(struct.pack(f'<{len(frame_offset_table)}I', *frame_offset_table))
        
        print(f"\nConverted {num_frames} frames to tile format")
        print(f"Output file: {output_file}")
        print(f"File size: {f.tell()} bytes")


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <input.mjpeg> <output.tiles> <width> <height> [max_frames] [tile_size]")
        print(f"Example: {sys.argv[0]} video.mjpeg video.tiles 368 448 20 32")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    width = int(sys.argv[3])
    height = int(sys.argv[4])
    max_frames = int(sys.argv[5]) if len(sys.argv) > 5 else None
    tile_size_arg = int(sys.argv[6]) if len(sys.argv) > 6 else None
    
    # Update global TILE_SIZE if provided
    if tile_size_arg:
        TILE_SIZE = tile_size_arg
    print(f"Using tile size: {TILE_SIZE}×{TILE_SIZE}")
    
    with open(input_file, 'rb') as f:
        mjpeg_data = f.read()
    
    convert_mjpeg_to_tiles(mjpeg_data, output_file, width, height, max_frames)

