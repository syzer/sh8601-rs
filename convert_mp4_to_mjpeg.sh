#!/bin/bash
# Convert MP4 video to MJPEG format
# Usage: ./convert_mp4_to_mjpeg.sh input.mp4 [output.mjpeg] [fps] [width] [height] [transpose] [quality]

set -euo pipefail

INPUT="${1:-}"
OUTPUT="${2:-}"
FPS="${3:-20}"          # Default 20 fps
WIDTH="${4:-368}"       # Default display width
HEIGHT="${5:-448}"      # Default display height
TRANSPOSE="${6:-0}"     # Default no rotation: 0=none, 1=90° clockwise, 2=180°, 3=90° counter-clockwise
QUALITY="${7:-9}"       # Default JPEG quality 9 (higher = lower quality, faster decode)

if [ -z "$INPUT" ]; then
    echo "Usage: $0 <input.mp4> [output.mjpeg] [fps] [width] [height] [transpose] [quality]"
    echo "  Example: $0 assets/mp4/video.mp4 output.mjpeg 24 368 448 0 9"
    echo "  transpose: 0=none, 1=90° clockwise, 2=180°, 3=90° counter-clockwise"
    echo "  quality: JPEG quality 2-31 (lower = better quality, higher = faster decode)"
    echo "           Recommended: 9-11 for embedded playback"
    exit 1
fi

if [ ! -f "$INPUT" ]; then
    echo "Error: Input file '$INPUT' not found"
    exit 1
fi

# Determine output filename
# Handle empty string, quotes from justfile, or unset
# Also handle case where OUTPUT might be a number (misaligned arguments)
if [ -z "$OUTPUT" ] || [ "$OUTPUT" == "''" ] || [ "$OUTPUT" == '""' ] || [[ "$OUTPUT" =~ ^[0-9]+$ ]]; then
    # If OUTPUT looks like a number, arguments are misaligned
    if [[ "$OUTPUT" =~ ^[0-9]+$ ]]; then
        # Shift arguments: OUTPUT is actually FPS
        FPS="$OUTPUT"
        WIDTH="${4:-368}"
        HEIGHT="${5:-448}"
        TRANSPOSE="${6:-0}"
    fi
    BASENAME=$(basename "$INPUT" .mp4)
    OUTPUT_DIR="assets/mjpeg"
    mkdir -p "$OUTPUT_DIR"
    OUTPUT="$OUTPUT_DIR/${BASENAME}.mjpeg"
fi

OUTPUT_DIR=$(dirname "$OUTPUT")
mkdir -p "$OUTPUT_DIR"

echo "Converting $INPUT to $OUTPUT"
echo "  FPS: $FPS"
echo "  Size: ${WIDTH}x${HEIGHT}"
echo "  Transpose: $TRANSPOSE"
echo "  JPEG Quality: ${QUALITY} (higher = lower quality, faster decode)"

# Build video filter
# Force exact dimensions: scale to fill (may crop), then crop to exact size
# This avoids black bars by cropping instead of padding
# Order matters: first scale to fill, then crop to exact size
SCALE_FILTER="scale=${WIDTH}:${HEIGHT}:force_original_aspect_ratio=increase"
CROP_FILTER="crop=${WIDTH}:${HEIGHT}"
VF_FILTER="fps=${FPS},${SCALE_FILTER},${CROP_FILTER}"

if [ "$TRANSPOSE" != "0" ]; then
    VF_FILTER="transpose=${TRANSPOSE},${VF_FILTER}"
fi

# Convert using ffmpeg
# -q:v quality range: 2-31 (lower = better quality, higher = faster decode/smaller files)
# Quality 7 = high quality, 9-11 = balanced for embedded playback (recommended)
if ! ffmpeg -y -i "$INPUT" \
    -pix_fmt yuvj420p \
    -q:v "${QUALITY}" \
    -vf "${VF_FILTER}" \
    "$OUTPUT" 2>&1 | grep -v "^frame=" | grep -v "^size="; then
    echo "Error: ffmpeg conversion failed"
    exit 1
fi

FILE_SIZE=$(stat -f%z "$OUTPUT" 2>/dev/null || stat -c%s "$OUTPUT" 2>/dev/null || echo "?")
echo ""
echo "✅ Conversion complete: $OUTPUT"
if [ "$FILE_SIZE" != "?" ]; then
    echo "   File size: $(numfmt --to=iec-i --suffix=B $FILE_SIZE 2>/dev/null || echo "${FILE_SIZE} bytes")"
fi

