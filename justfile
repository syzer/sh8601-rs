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

# Build and run the example on the ESP32-S3
run:
    cargo run --example ws_18in_amoled --features "waveshare_18_amoled"

release:
    cargo run --example ws_18in_amoled --features "waveshare_18_amoled" --release

# Run the movie player example with RGB565 (33% memory savings)
run-movies-rgb565:
    cargo run --example movie_player --features "waveshare_18_amoled,rgb565"

# Run the movie player example with RGB888 (default)
run-movies-rgb888:
    cargo run --example movie_player --features "waveshare_18_amoled,rgb888"

# Run the movie player example (defaults to RGB565 for efficiency)
run-movies:
    cargo run --example movie_player --features "waveshare_18_amoled,rgb565"

run-movies-release:
    cargo run --example movie_player --features "waveshare_18_amoled,rgb565" --release

run-movies-rgb888-release:
    cargo run --example movie_player --features "waveshare_18_amoled,rgb888" --release

resizeMovies:
    #!/usr/bin/env bash
    set -euo pipefail
    
    mkdir -p assets/rgb
    
    for f in assets/mp4/*.mp4; do
        if [[ ! -f "$f" ]]; then
            echo "No MP4 files found in assets/mp4/"
            exit 1
        fi
        base="$(basename "$f" .mp4)"
        out="assets/rgb/${base}_368x448_rgb565.raw"
        echo "Converting $f -> $out (resized to 368x448, RGB565 format)"
        ffmpeg -y -i "$f" -vf "scale=368:448" -pix_fmt rgb565le -f rawvideo "$out"
        if [[ -f "$out" ]]; then
            echo "  ✓ $(du -h "$out" | cut -f1)  $out"
        else
            echo "  ✗ Failed to create $out" >&2
            exit 1
        fi
    done
    
    echo "✅ Done. Converted MP4(s) to RGB565 format in assets/rgb/"

resizeMoviesTiny:
    #!/usr/bin/env bash
    set -euo pipefail
    
    mkdir -p assets/rgb
    
    for f in assets/mp4/*.mp4; do
        if [[ ! -f "$f" ]]; then
            echo "No MP4 files found in assets/mp4/"
            exit 1
        fi
        base="$(basename "$f" .mp4)"
        out="assets/rgb/${base}_tiny_rgb565.raw"
        echo "Converting $f -> $out (first 5 frames, RGB565 format for testing)"
        ffmpeg -y -i "$f" -vf "scale=368:448" -vframes 5 -pix_fmt rgb565le -f rawvideo "$out"
        if [[ -f "$out" ]]; then
            echo "  ✓ $(du -h "$out" | cut -f1)  $out"
        else
            echo "  ✗ Failed to create $out" >&2
            exit 1
        fi
    done
    
    echo "✅ Done. Converted MP4(s) to tiny RGB565 test files in assets/rgb/"

resizeMoviesSmall:
    #!/usr/bin/env bash
    set -euo pipefail
    
    mkdir -p assets/rgb
    
    for f in assets/mp4/*.mp4; do
        if [[ ! -f "$f" ]]; then
            echo "No MP4 files found in assets/mp4/"
            exit 1
        fi
        base="$(basename "$f" .mp4)"
        out="assets/rgb/${base}_small_rgb565.raw"
        echo "Converting $f -> $out (first 15 frames, RGB565 format for testing)"
        ffmpeg -y -i "$f" -vf "scale=368:448" -vframes 15 -pix_fmt rgb565le -f rawvideo "$out"
        if [[ -f "$out" ]]; then
            echo "  ✓ $(du -h "$out" | cut -f1)  $out"
        else
            echo "  ✗ Failed to create $out" >&2
            exit 1
        fi
    done
    
    echo "✅ Done. Converted MP4(s) to small RGB565 test files in assets/rgb/"

resizeMoviesSmallRgb888:
    #!/usr/bin/env bash
    set -euo pipefail
    
    mkdir -p assets/rgb
    
    for f in assets/mp4/*.mp4; do
        if [[ ! -f "$f" ]]; then
            echo "No MP4 files found in assets/mp4/"
            exit 1
        fi
        base="$(basename "$f" .mp4)"
        out="assets/rgb/${base}_small_rgb888.raw"
        echo "Converting $f -> $out (first 15 frames, RGB888 format for testing)"
        ffmpeg -y -i "$f" -vf "scale=368:448" -vframes 15 -pix_fmt rgb24 -f rawvideo "$out"
        if [[ -f "$out" ]]; then
            echo "  ✓ $(du -h "$out" | cut -f1)  $out"
        else
            echo "  ✗ Failed to create $out" >&2
            exit 1
        fi
    done
    
    echo "✅ Done. Converted MP4(s) to small RGB888 test files in assets/rgb/"

resizeMoviesMedium:
    #!/usr/bin/env bash
    set -euo pipefail
    
    mkdir -p assets/rgb
    
    for f in assets/mp4/*.mp4; do
        if [[ ! -f "$f" ]]; then
            echo "No MP4 files found in assets/mp4/"
            exit 1
        fi
        base="$(basename "$f" .mp4)"
        out="assets/rgb/${base}_medium_rgb565.raw"
        echo "Converting $f -> $out (60 frames, RGB565 format - should fit in flash)"
        ffmpeg -y -i "$f" -vf "scale=368:448" -vframes 60 -pix_fmt rgb565le -f rawvideo "$out"
        if [[ -f "$out" ]]; then
            echo "  ✓ $(du -h "$out" | cut -f1)  $out"
        else
            echo "  ✗ Failed to create $out" >&2
            exit 1
        fi
    done
    
    echo "✅ Done. Converted MP4(s) to medium RGB565 files in assets/rgb/"

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
