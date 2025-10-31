#!/bin/bash
# Convert MP4 video to MQOI animated format
# Usage: ./convert_mp4_to_mqoi.sh input.mp4 [output.mqoi] [fps] [width] [height]

set -euo pipefail

INPUT="${1:-}"
OUTPUT="${2:-}"
FPS="${3:-24}"          # Default 24 fps
WIDTH="${4:-368}"       # Default display width
HEIGHT="${5:-448}"      # Default display height

if [ -z "$INPUT" ]; then
    echo "Usage: $0 <input.mp4> [output.mqoi] [fps] [width] [height]"
    echo "  Example: $0 assets/mp4/video.mp4 output.mqoi 15 368 448"
    exit 1
fi

if [ ! -f "$INPUT" ]; then
    echo "Error: Input file '$INPUT' not found"
    exit 1
fi

# Determine output filename
if [ -z "$OUTPUT" ] || [ "$OUTPUT" == "''" ] || [ "$OUTPUT" == '""' ]; then
    BASENAME=$(basename "$INPUT" .mp4)
    OUTPUT_DIR="assets/mqoi"
    mkdir -p "$OUTPUT_DIR"
    OUTPUT="$OUTPUT_DIR/${BASENAME}.mqod"
fi

OUTPUT_DIR=$(dirname "$OUTPUT")
TEMP_DIR=$(mktemp -d)
trap "rm -rf '$TEMP_DIR'" EXIT

echo "Converting $INPUT to $OUTPUT"
echo "  FPS: $FPS"
echo "  Size: ${WIDTH}x${HEIGHT}"

# Extract frames from MP4 using ffmpeg
echo "Extracting frames..."
ffmpeg -i "$INPUT" \
    -vf "fps=$FPS,scale=${WIDTH}:${HEIGHT}:force_original_aspect_ratio=decrease,pad=${WIDTH}:${HEIGHT}:(ow-iw)/2:(oh-ih)/2:color=black" \
    -y \
    "$TEMP_DIR/frame_%06d.png" 2>/dev/null || {
    echo "Error: ffmpeg not found or failed. Install with: brew install ffmpeg"
    exit 1
}

# Count frames
FRAME_COUNT=$(ls -1 "$TEMP_DIR"/frame_*.png 2>/dev/null | wc -l | tr -d ' ')
if [ "$FRAME_COUNT" -eq 0 ]; then
    echo "Error: No frames extracted"
    exit 1
fi

echo "Extracted $FRAME_COUNT frames"

# Convert each frame to QOI
echo "Converting frames to QOI..."
FRAME_NUM=0
for PNG in "$TEMP_DIR"/frame_*.png; do
    FRAME_QOI="$TEMP_DIR/frame_$(printf "%06d" $FRAME_NUM).qoi"
    
    # Convert PNG to PPM, then to QOI
    magick "$PNG" -depth 8 ppm:- | pamtoqoi > "$FRAME_QOI"
    
    FRAME_NUM=$((FRAME_NUM + 1))
    if [ $((FRAME_NUM % 10)) -eq 0 ]; then
        echo "  Converted $FRAME_NUM frames..."
    fi
done

echo "Converted all frames to QOI"

# Pack frames into MQOI format with delta encoding
echo "Packing into MQOI format with delta encoding..."
python3 <<EOF
import struct
import sys
import glob
import os
from PIL import Image

temp_dir = "$TEMP_DIR"
output_file = "$OUTPUT"
width = $WIDTH
height = $HEIGHT

# Get sorted frame files (QOI and PNG)
qoi_frames = sorted(glob.glob(os.path.join(temp_dir, "frame_*.qoi")))
png_frames = sorted(glob.glob(os.path.join(temp_dir, "frame_*.png")))

if not qoi_frames:
    print("Error: No QOI frames found")
    sys.exit(1)

if len(qoi_frames) != len(png_frames):
    print(f"Error: Mismatch between QOI ({len(qoi_frames)}) and PNG ({len(png_frames)}) frames")
    sys.exit(1)

def rgb888_to_rgb565(r, g, b):
    """Convert RGB888 to RGB565"""
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

with open(output_file, "wb") as out:
    # Write magic: "MQOΔ" = b"MQO\xCE\x94" (UTF-8) or b"MQO\xCE" (as per example)
    out.write(b"MQO\xCE")
    
    # Write version (u16, little-endian)
    out.write(struct.pack("<H", 1))
    
    # Write frame count (u16, little-endian)
    out.write(struct.pack("<H", len(qoi_frames)))
    
    # Calculate offsets
    offsets = []
    header_size = 4 + 2 + 2 + 4 * len(qoi_frames)  # magic + version + count + offsets
    pos = header_size
    
    # Frame 0: full QOI, subsequent frames: delta encoded
    # Calculate size of first frame (full QOI)
    frame0_size = os.path.getsize(qoi_frames[0])
    offsets.append(pos)
    pos += frame0_size
    
    # Calculate sizes of delta frames
    for i in range(1, len(png_frames)):
        # Load previous and current frame
        img_prev = Image.open(png_frames[i - 1]).convert("RGB")
        img_curr = Image.open(png_frames[i]).convert("RGB")
        
        pix_prev = img_prev.load()
        pix_curr = img_curr.load()
        
        # Find changed pixels
        diff = []
        for y in range(height):
            for x in range(width):
                if pix_prev[x, y] != pix_curr[x, y]:
                    idx = y * width + x
                    r, g, b = pix_curr[x, y]
                    rgb565 = rgb888_to_rgb565(r, g, b)
                    diff.append((idx, rgb565))
        
        # Delta frame size: 1 byte marker + 4 bytes count + 6 bytes per changed pixel
        delta_size = 1 + 4 + 6 * len(diff)
        offsets.append(pos)
        pos += delta_size
    
    # Write offsets (u32, little-endian)
    for o in offsets:
        out.write(struct.pack("<I", o))
    
    # Write frame 0: full QOI
    print("Writing frame 0 (full QOI)...")
    with open(qoi_frames[0], "rb") as f:
        out.write(f.read())
    
    # Write delta frames
    for i in range(1, len(png_frames)):
        if i % 10 == 0:
            print(f"  Processing delta frame {i}/{len(png_frames) - 1}...")
        
        # Load previous and current frame
        img_prev = Image.open(png_frames[i - 1]).convert("RGB")
        img_curr = Image.open(png_frames[i]).convert("RGB")
        
        pix_prev = img_prev.load()
        pix_curr = img_curr.load()
        
        # Find changed pixels
        diff = []
        for y in range(height):
            for x in range(width):
                if pix_prev[x, y] != pix_curr[x, y]:
                    idx = y * width + x
                    r, g, b = pix_curr[x, y]
                    rgb565 = rgb888_to_rgb565(r, g, b)
                    diff.append((idx, rgb565))
        
        # Write delta frame
        out.write(b"\x01")  # Delta frame marker
        out.write(struct.pack("<I", len(diff)))  # Count of changed pixels
        
        for idx, rgb565 in diff:
            out.write(struct.pack("<I", idx))  # Pixel index (u32)
            out.write(struct.pack("<H", rgb565))  # RGB565 color (u16)

print(f"\n✅ Created {output_file}")
print(f"   Frames: {len(qoi_frames)}")
print(f"   Frame 0: Full QOI")
print(f"   Frames 1-{len(qoi_frames) - 1}: Delta encoded")
print(f"   File size: {os.path.getsize(output_file):,} bytes")
EOF

echo ""
echo "✅ Conversion complete: $OUTPUT"

