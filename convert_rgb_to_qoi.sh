#!/bin/bash
# Convert RGB888 raw files to QOI format using ImageMagick and Netpbm

set -euo pipefail

WIDTH=368
HEIGHT=448

mkdir -p assets/qoi

for i in 1 2 3; do
    RGB_FILE="assets/rgb/pic_${i}_368x448.rgb"
    QOI_FILE="assets/qoi/pic_${i}_368x448.qoi"
    
    if [ ! -f "$RGB_FILE" ]; then
        echo "Warning: $RGB_FILE not found, skipping"
        continue
    fi
    
    # Convert RGB to PPM (using ImageMagick)
    PPM_FILE=$(mktemp).ppm
    magick -size ${WIDTH}x${HEIGHT} -depth 8 rgb:"$RGB_FILE" "$PPM_FILE"
    
    # Check if pamtoqoi is available
    if command -v pamtoqoi &> /dev/null; then
        # Use pamtoqoi if available
        pamtoqoi "$PPM_FILE" > "$QOI_FILE"
    else
        # Fallback: Use Python with qoi-conv if available
        if python3 -c "import qoi_conv" 2>/dev/null; then
            python3 -c "
import qoi_conv
import sys
from PIL import Image

img = Image.open('$PPM_FILE')
qoi_conv.encode('$QOI_FILE', img)
"
        else
            echo "Error: Need either 'pamtoqoi' or Python 'qoi-conv' package"
            echo "  Install pamtoqoi: brew install netpbm"
            echo "  Or install qoi-conv: python3 -m pip install --user qoi-conv"
            rm -f "$PPM_FILE"
            exit 1
        fi
    fi
    
    # Get file sizes
    RGB_SIZE=$(stat -f%z "$RGB_FILE" 2>/dev/null || stat -c%s "$RGB_FILE" 2>/dev/null || echo "?")
    QOI_SIZE=$(stat -f%z "$QOI_FILE" 2>/dev/null || stat -c%s "$QOI_FILE" 2>/dev/null || echo "?")
    
    echo "Converted $RGB_FILE ($RGB_SIZE bytes) -> $QOI_FILE ($QOI_SIZE bytes)"
    
    if [ "$RGB_SIZE" != "?" ] && [ "$QOI_SIZE" != "?" ]; then
        RATIO=$(echo "scale=2; $RGB_SIZE / $QOI_SIZE" | bc)
        echo "  Compression ratio: ${RATIO}x"
    fi
    
    rm -f "$PPM_FILE"
done

echo ""
echo "✅ All conversions complete!"

