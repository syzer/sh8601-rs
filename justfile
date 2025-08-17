# Justfile for SH8601-RS project

# Default recipe to list available commands
default:
    @just --list

# Convert and resize image for SH8601 display (368x448 RGB format)
resizeImage input="Cool_Dude.png":
    #!/usr/bin/env bash
    set -euo pipefail

    # Check if input file exists
    if [[ ! -f "assets/png/{{input}}" ]]; then
        echo "Error: File assets/png/{{input}} not found!"
        echo "Available files in assets/png/:"
        ls -la assets/png/ || echo "assets/png/ directory not found"
        exit 1
    fi

    echo "Converting {{input}} to 368x448 RGB format..."
    magick assets/png/{{input}} -resize 368x448\! -strip -depth 8 rgb:assets/pic_368x448.rgb

    echo "✅ Successfully converted {{input}} to assets/pic_368x448.rgb"
    echo "File size: $(du -h assets/pic_368x448.rgb | cut -f1)"

# Build and run the example on the ESP32-S3
run:
    cargo run --example ws_18in_amoled --features "waveshare_18_amoled"

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
