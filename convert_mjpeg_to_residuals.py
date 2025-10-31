#!/usr/bin/env python3
"""
Convert MJPEG to residual-encoded format with XOR + Zero-RLE + LZ4HC.

Approach:
1. Compute XOR residual: curr_frame ^ prev_frame (RGB565)
2. Zero-RLE: encode spans of 0x0000 as (count)
3. LZ4HC compression for final size reduction

Format:
- Header: "RSDL" magic, version (u16), frame_count (u16), width (u16), height (u16)
- Frame offsets table: u32[frame_count]
- Per frame:
  - compressed_size (u32): size of compressed data
  - compressed_data (bytes): LZ4HC(zero_rle(xor_residual))

First frame is stored as full frame (no residual).
"""

import struct
import sys
from PIL import Image
import io
import lz4.block

MAGIC = b"RSDL"
VERSION = 1


def rgb888_to_rgb565(r: int, g: int, b: int) -> int:
    """Convert RGB888 to RGB565."""
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)


def decode_jpeg_frame(jpeg_data: bytes) -> Image.Image:
    """Decode JPEG frame to PIL Image."""
    return Image.open(io.BytesIO(jpeg_data))


def frame_to_rgb565(image: Image.Image, width: int, height: int) -> bytes:
    """Convert PIL Image to RGB565 big-endian bytes."""
    data = bytearray(width * height * 2)
    idx = 0
    
    for y in range(height):
        for x in range(width):
            if x < image.width and y < image.height:
                pixel = image.getpixel((x, y))
                if isinstance(pixel, int):  # Grayscale
                    r = g = b = pixel
                else:  # RGB/RGBA
                    r, g, b = pixel[0], pixel[1], pixel[2]
            else:
                r = g = b = 0  # Padding
            
            rgb565 = rgb888_to_rgb565(r, g, b)
            # Big-endian (MSB first)
            data[idx] = (rgb565 >> 8) & 0xFF
            data[idx + 1] = rgb565 & 0xFF
            idx += 2
    
    return bytes(data)


def compute_xor_residual(curr_frame: bytes, prev_frame: bytes) -> bytes:
    """Compute XOR residual between two frames."""
    if prev_frame is None or len(prev_frame) != len(curr_frame):
        return curr_frame
    
    residual = bytearray(len(curr_frame))
    for i in range(len(curr_frame)):
        residual[i] = curr_frame[i] ^ prev_frame[i]
    
    return bytes(residual)


def zero_rle_encode(data: bytes) -> bytes:
    """
    Zero-RLE encoding on 16-bit words.
    Format: [type_byte] [data...]
    - type_byte & 0x80 == 0: literal run, length = type_byte (1-127 u16 words)
    - type_byte & 0x80 != 0: zero run, length = (type_byte & 0x7F) (1-127 u16 words)
    
    Special encoding for longer runs:
    - 0xFF + u16_count: zero run of count u16 words (for runs > 127)
    """
    if len(data) % 2 != 0:
        raise ValueError("Data length must be even for u16 encoding")
    
    encoded = bytearray()
    i = 0
    num_words = len(data) // 2
    
    while i < num_words:
        # Check for zero run
        if data[i*2] == 0 and data[i*2+1] == 0:
            zero_count = 0
            j = i
            while j < num_words and data[j*2] == 0 and data[j*2+1] == 0:
                zero_count += 1
                j += 1
            
            # Encode zero runs
            while zero_count > 0:
                if zero_count > 32767:  # Use extended encoding for very long runs
                    encoded.append(0xFF)
                    encoded.extend(struct.pack('<H', 32767))
                    zero_count -= 32767
                elif zero_count > 127:
                    encoded.append(0xFF)
                    encoded.extend(struct.pack('<H', min(zero_count, 32767)))
                    zero_count -= min(zero_count, 32767)
                else:
                    encoded.append(0x80 | zero_count)
                    zero_count = 0
            
            i = j
        else:
            # Literal run
            lit_start = i
            while i < num_words and not (data[i*2] == 0 and data[i*2+1] == 0) and (i - lit_start) < 127:
                i += 1
            
            lit_count = i - lit_start
            encoded.append(lit_count)  # Literal marker (no 0x80 bit)
            encoded.extend(data[lit_start*2 : i*2])
    
    return bytes(encoded)


def zero_rle_decode(encoded: bytes) -> bytes:
    """Decode zero-RLE encoded data."""
    decoded = bytearray()
    i = 0
    
    while i < len(encoded):
        type_byte = encoded[i]
        i += 1
        
        if type_byte == 0xFF:
            # Extended zero run
            if i + 1 >= len(encoded):
                break
            count = struct.unpack('<H', encoded[i:i+2])[0]
            i += 2
            decoded.extend(b'\x00\x00' * count)
        elif type_byte & 0x80:
            # Zero run
            count = type_byte & 0x7F
            decoded.extend(b'\x00\x00' * count)
        else:
            # Literal run
            count = type_byte
            byte_count = count * 2
            if i + byte_count > len(encoded):
                break
            decoded.extend(encoded[i:i+byte_count])
            i += byte_count
    
    return bytes(decoded)


def convert_mjpeg_to_residuals(input_mjpeg: bytes, output_file: str, width: int, height: int, max_frames: int = None):
    """Convert MJPEG to residual-encoded format."""
    # Find frame offsets
    frame_offsets = []
    i = 0
    while i < len(input_mjpeg) - 1:
        if input_mjpeg[i] == 0xFF and input_mjpeg[i+1] == 0xD8:  # JPEG SOI
            frame_offsets.append(i)
        i += 1
    
    if not frame_offsets:
        print("No JPEG frames found")
        return
    
    num_frames = len(frame_offsets)
    if max_frames is not None and max_frames > 0:
        num_frames = min(num_frames, max_frames)
        frame_offsets = frame_offsets[:num_frames]
    
    print(f"Found {len(frame_offsets)} frames in MJPEG file" + 
          (f" (limited to {num_frames})" if max_frames else ""))
    print(f"Frame size: {width}×{height}")
    
    # Process frames
    prev_frame = None
    compressed_frames = []
    
    for frame_idx, offset in enumerate(frame_offsets):
        # Extract JPEG frame
        if frame_idx + 1 < len(frame_offsets):
            jpeg_data = input_mjpeg[offset:frame_offsets[frame_idx + 1]]
        else:
            jpeg_data = input_mjpeg[offset:]
        
        # Decode to RGB565
        print(f"Processing frame {frame_idx + 1}/{num_frames}...", end='')
        img = decode_jpeg_frame(jpeg_data)
        curr_frame = frame_to_rgb565(img, width, height)
        
        # Compute residual (XOR with previous frame)
        if frame_idx == 0:
            residual = curr_frame  # First frame is full
            print(" (keyframe)", end='')
        else:
            residual = compute_xor_residual(curr_frame, prev_frame)
        
        # Zero-RLE encode
        rle_encoded = zero_rle_encode(residual)
        
        # LZ4HC compress
        lz4_compressed = lz4.block.compress(rle_encoded, mode='high_compression', compression=12, store_size=False)
        
        compressed_frames.append(lz4_compressed)
        
        compression_ratio = len(curr_frame) / len(lz4_compressed) if lz4_compressed else 0
        print(f" {len(curr_frame)}B → {len(rle_encoded)}B (RLE) → {len(lz4_compressed)}B (LZ4) = {compression_ratio:.1f}x")
        
        prev_frame = curr_frame
    
    # Write output file
    with open(output_file, 'wb') as f:
        # Header
        f.write(MAGIC)
        f.write(struct.pack('<H', VERSION))
        f.write(struct.pack('<H', num_frames))
        f.write(struct.pack('<H', width))
        f.write(struct.pack('<H', height))
        
        # Frame offsets table (placeholder)
        offsets_pos = f.tell()
        for _ in range(num_frames):
            f.write(struct.pack('<I', 0))
        
        # Write frames and record offsets
        frame_offsets_table = []
        for frame_data in compressed_frames:
            frame_offset = f.tell()
            frame_offsets_table.append(frame_offset)
            
            # Write compressed size + data
            f.write(struct.pack('<I', len(frame_data)))
            f.write(frame_data)
        
        # Go back and write actual offsets
        f.seek(offsets_pos)
        for offset in frame_offsets_table:
            f.write(struct.pack('<I', offset))
        
        # Return to end for file size
        f.seek(0, 2)
        file_size = f.tell()
        
        print(f"\nConverted {num_frames} frames to residual format")
        print(f"Output file: {output_file}")
        print(f"File size: {file_size} bytes ({file_size / (1024*1024):.2f} MB)")
        print(f"Average per frame: {file_size / num_frames:.0f} bytes")


if __name__ == '__main__':
    if len(sys.argv) < 4:
        print(f"Usage: {sys.argv[0]} <input.mjpeg> <output.residuals> <width> <height> [max_frames]")
        print(f"Example: {sys.argv[0]} video.mjpeg video.residuals 368 448 5")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    width = int(sys.argv[3])
    height = int(sys.argv[4])
    max_frames = int(sys.argv[5]) if len(sys.argv) > 5 else None
    
    with open(input_file, 'rb') as f:
        mjpeg_data = f.read()
    
    convert_mjpeg_to_residuals(mjpeg_data, output_file, width, height, max_frames)

