# Justfile for SH8601-RS project

# Default recipe to list available commands
default:
    @just --list

# Convert and resize images for SH8601 display (368x448 RGB format)
resizeImages:
    #!/usr/bin/env bash
    set -euo pipefail
    shopt -s nullglob

    mkdir -p assets/rgb

    files=(assets/png/*.{png,PNG,jpg,JPG,jpeg,JPEG})
    if [[ ${#files[@]} -eq 0 ]]; then
        echo "No images found in assets/png/ (looking for .png/.jpg/.jpeg)."
        exit 1
    fi

    idx=1
    for f in "${files[@]}"; do
        base="$(basename "$f")"
        out="assets/rgb/pic_${idx}_368x448.rgb"
        echo "Converting $base -> ${out}"
        magick "$f" -resize 368x448\! -strip -depth 8 rgb:"$out"
        if [[ -f "$out" ]]; then
            echo "  ✓ $(du -h "$out" | cut -f1)  $out"
        else
            echo "  ✗ Failed to create $out" >&2
            exit 1
        fi
        idx=$((idx+1))
    done

    echo "✅ Done. Converted ${#files[@]} image(s) to assets/rgb/"

# Convert MP4 video to MJPEG format
# Usage: just convert-mjpeg input.mp4 [output.mjpeg] [fps] [width] [height] [transpose] [quality]
# transpose: 0=none, 1=90° clockwise, 2=180°, 3=90° counter-clockwise
# quality: JPEG quality 2-31 (lower = better quality, higher = faster decode). Default 9 for embedded
#   Recommended: 9-11 for embedded playback (faster decode, smaller files)
convert-mjpeg input output='' fps='20' width='368' height='448' transpose='0' quality='9':
    ./convert_mp4_to_mjpeg.sh {{input}} {{output}} {{fps}} {{width}} {{height}} {{transpose}} {{quality}}

# Convert MP4 video to MQOI animated format (with delta encoding)
# Usage: just convert-mp4 input.mp4 [output.mqoi] [fps] [width] [height]
convert-mqoi input output='' fps='24' width='368' height='448':
    ./convert_mp4_to_mqoi.sh {{input}} {{output}} {{fps}} {{width}} {{height}}

# Convert MJPEG to tile-based format (dirty tile encoding)
# Usage: just convert-tiles input.mjpeg [output.tiles] [width] [height] [max_frames] [tile_size]
# Now using 32×32 tiles for 4× fewer tiles and larger DMA bursts
# Can achieve 2-10× effective FPS improvement
# max_frames: Limit conversion to first N frames (default: 5 for 32×32 tiles, keeps ~1.6MB)
convert-tiles input output='' width='368' height='448' max_frames='5' tile_size='32':
    #!/usr/bin/env bash
    if [ -z "{{output}}" ]; then
        OUTPUT="{{input}}.tiles"
    else
        OUTPUT="{{output}}"
    fi
    python3 convert_mjpeg_to_tiles.py "{{input}}" "$OUTPUT" {{width}} {{height}} {{max_frames}} {{tile_size}}

# Convert MJPEG to residual format (XOR + Zero-RLE + LZ4HC)
# Experimental: may be better for large moving objects vs tiles
convert-residuals input output='' width='368' height='448' max_frames='5':
    #!/usr/bin/env bash
    if [ -z "{{output}}" ]; then
        base=$(basename "{{input}}" .mjpeg)
        OUTPUT="assets/residuals/${base}.residuals"
    else
        OUTPUT="{{output}}"
    fi
    mkdir -p assets/residuals
    # Use the Python from PATH (should be asdf-managed)
    python convert_mjpeg_to_residuals.py "{{input}}" "$OUTPUT" {{width}} {{height}} {{max_frames}}

# Convert all MJPEG files from assets/mjpeg to assets/tiles
# Uses default settings: 32×32 tiles, 5 frames max, 368×448 resolution
convert-all-tiles width='368' height='448' max_frames='5' tile_size='32':
    #!/usr/bin/env bash
    set -euo pipefail
    
    mkdir -p assets/tiles
    
    if [ ! -d "assets/mjpeg" ]; then
        echo "Error: assets/mjpeg directory not found"
        exit 1
    fi
    
    # Find all .mjpeg files
    shopt -s nullglob
    mjpeg_files=(assets/mjpeg/*.mjpeg)
    
    if [ ${#mjpeg_files[@]} -eq 0 ]; then
        echo "No .mjpeg files found in assets/mjpeg/"
        exit 0
    fi
    
    echo "Found ${#mjpeg_files[@]} MJPEG file(s) to convert"
    echo "Settings: {{tile_size}}×{{tile_size}} tiles, max {{max_frames}} frames, {{width}}×{{height}}"
    echo ""
    
    for input in "${mjpeg_files[@]}"; do
        base=$(basename "$input" .mjpeg)
        output="assets/tiles/${base}.tiles"
        
        echo "Converting: $input -> $output"
        python3 convert_mjpeg_to_tiles.py \
            "$input" \
            "$output" \
            {{width}} {{height}} \
            {{max_frames}} \
            {{tile_size}}
        
        if [ -f "$output" ]; then
            size=$(du -h "$output" | cut -f1)
            echo "  ✓ Created: $output ($size)"
        else
            echo "  ✗ Failed to create: $output"
        fi
        echo ""
    done
    
    echo "Conversion complete! Output files in assets/tiles/"
    ls -lh assets/tiles/*.tiles

# Build and run the tile-based example on the ESP32-S3
run:
    cargo run --example ws_18in_amoled --features "waveshare_18_amoled"

# Build and run the residual-based example on the ESP32-S3 (RELEASE mode for performance)
run-residuals:
    cargo run --example ws_18in_amoled_residuals --features "waveshare_18_amoled" --release

# Debug mode for residuals (slow, only for debugging)
debug-residuals:
    cargo run --example ws_18in_amoled_residuals --features "waveshare_18_amoled"

release:
    cargo run --example ws_18in_amoled --features "waveshare_18_amoled" --release

# Clean build artifacts
clean:
    cargo clean

# Check code formatting and linting
check:
    cargo fmt --check
    cargo clippy -- -D warnings

# Format code
fmt:
    cargo fmt

# Build the library
build:
    cargo build

# Build for release
build-release:
    cargo build --release

# List available PNG images
list-images:
    @echo "Available PNG images in assets/png/:"
    @ls -la assets/png/ | grep -E "\\.png$" || echo "No PNG files found"

# Show current RGB file info
show-current:
    @if [ -f "assets/pic_368x448.rgb" ]; then \
        echo "Current RGB file: assets/pic_368x448.rgb"; \
        echo "Size: $(du -h assets/pic_368x448.rgb | cut -f1)"; \
        echo "Modified: $(stat -f "%Sm" assets/pic_368x448.rgb)"; \
    else \
        echo "No RGB file found. Run 'just resizeImage <image>' to create one."; \
    fi
