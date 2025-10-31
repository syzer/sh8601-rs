//! Residual decoder for XOR + Zero-RLE + LZ4HC encoded animation
//!
//! Format:
//! - Header: "RSDL" magic, version, frame_count, width, height
//! - Frame offsets table
//! - Per frame: compressed_size + LZ4HC(Zero-RLE(XOR residual))
//!
//! First frame is keyframe (full), subsequent frames are XOR residuals.

extern crate alloc;
use alloc::vec::Vec;

/// Residual decoder for animation
pub struct ResidualDecoder<'a> {
    data: &'a [u8],
    frame_count: usize,
    offsets_table_start: usize,
    width: u32,
    height: u32,
}

impl<'a> ResidualDecoder<'a> {
    /// Create a new residual decoder
    pub fn new(data: &'a [u8], width: u32, height: u32) -> Result<Self, &'static str> {
        if data.len() < 12 {
            return Err("Residual file too short (missing header)");
        }
        
        // Check magic "RSDL"
        if &data[0..4] != b"RSDL" {
            return Err("Invalid residual file magic");
        }
        
        let version = u16::from_le_bytes([data[4], data[5]]);
        if version != 1 {
            return Err("Unsupported residual format version");
        }
        
        let frame_count = u16::from_le_bytes([data[6], data[7]]) as usize;
        let file_width = u16::from_le_bytes([data[8], data[9]]) as u32;
        let file_height = u16::from_le_bytes([data[10], data[11]]) as u32;
        
        if file_width != width || file_height != height {
            return Err("Residual file dimensions mismatch");
        }
        
        if data.len() < 12 + frame_count * 4 {
            return Err("Residual file too short (missing offsets table)");
        }
        
        Ok(Self {
            data,
            frame_count,
            offsets_table_start: 12,
            width,
            height,
        })
    }
    
    /// Get frame count
    pub fn frame_count(&self) -> usize {
        self.frame_count
    }
    
    /// Decode frame and apply to framebuffer via XOR
    /// For frame 0 (keyframe), replaces framebuffer
    /// For frame > 0, XORs residual with framebuffer
    #[inline(always)]
    #[link_section = ".iram1.text"]
    pub fn decode_frame(&self, frame_idx: usize, framebuffer: &mut [u16]) -> Result<(), &'static str> {
        if frame_idx >= self.frame_count {
            return Err("Frame index out of bounds");
        }
        
        let expected_fb_size = (self.width * self.height) as usize;
        if framebuffer.len() < expected_fb_size {
            return Err("Framebuffer too small");
        }
        
        // Read frame offset
        let offset_pos = self.offsets_table_start + frame_idx * 4;
        let frame_offset = u32::from_le_bytes([
            self.data[offset_pos],
            self.data[offset_pos + 1],
            self.data[offset_pos + 2],
            self.data[offset_pos + 3],
        ]) as usize;
        
        if frame_offset + 4 > self.data.len() {
            return Err("Frame offset out of bounds");
        }
        
        // Read compressed size
        let compressed_size = u32::from_le_bytes([
            self.data[frame_offset],
            self.data[frame_offset + 1],
            self.data[frame_offset + 2],
            self.data[frame_offset + 3],
        ]) as usize;
        
        let compressed_data_start = frame_offset + 4;
        if compressed_data_start + compressed_size > self.data.len() {
            return Err("Compressed data out of bounds");
        }
        
        let compressed_data = &self.data[compressed_data_start..compressed_data_start + compressed_size];
        
        // LZ4 decompress
        let max_decompressed_size = expected_fb_size * 2 + 4096; // RGB565 bytes + overhead
        let rle_data = lz4_flex::decompress(compressed_data, max_decompressed_size)
            .map_err(|_| "LZ4 decompression failed")?;
        
        // Zero-RLE decode
        let residual_bytes = zero_rle_decode(&rle_data)?;
        
        if residual_bytes.len() != expected_fb_size * 2 {
            return Err("Decoded residual size mismatch");
        }
        
        // Apply XOR to framebuffer (byte-wise, then convert to u16)
        // For frame 0 (keyframe), this replaces the framebuffer
        // For frame > 0, this applies the residual via XOR
        let fb_bytes = unsafe {
            core::slice::from_raw_parts_mut(
                framebuffer.as_mut_ptr() as *mut u8,
                expected_fb_size * 2
            )
        };
        
        for i in 0..fb_bytes.len() {
            fb_bytes[i] ^= residual_bytes[i];
        }
        
        Ok(())
    }
}

/// Zero-RLE decoder
/// Format:
/// - 0x00-0x7F: literal run of N u16 words (1-127)
/// - 0x80-0xFE: zero run of (type_byte & 0x7F) u16 words (1-127)
/// - 0xFF + u16: extended zero run
#[inline(always)]
#[link_section = ".iram1.text"]
fn zero_rle_decode(encoded: &[u8]) -> Result<Vec<u8>, &'static str> {
    let mut decoded = Vec::new();
    let mut i = 0;
    
    while i < encoded.len() {
        let type_byte = encoded[i];
        i += 1;
        
        if type_byte == 0xFF {
            // Extended zero run
            if i + 1 >= encoded.len() {
                return Err("Truncated extended zero run");
            }
            let count = u16::from_le_bytes([encoded[i], encoded[i + 1]]) as usize;
            i += 2;
            
            // Append count * 2 zero bytes (count u16 words)
            decoded.resize(decoded.len() + count * 2, 0);
        } else if type_byte & 0x80 != 0 {
            // Zero run
            let count = (type_byte & 0x7F) as usize;
            decoded.resize(decoded.len() + count * 2, 0);
        } else {
            // Literal run
            let count = type_byte as usize;
            let byte_count = count * 2;
            
            if i + byte_count > encoded.len() {
                return Err("Truncated literal run");
            }
            
            decoded.extend_from_slice(&encoded[i..i + byte_count]);
            i += byte_count;
        }
    }
    
    Ok(decoded)
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_zero_rle_decode() {
        // Test zero run
        let encoded = vec![0x83]; // 3 zero u16 words = 6 bytes
        let decoded = zero_rle_decode(&encoded).unwrap();
        assert_eq!(decoded, vec![0, 0, 0, 0, 0, 0]);
        
        // Test literal run
        let encoded = vec![0x02, 0x12, 0x34, 0x56, 0x78]; // 2 u16 words
        let decoded = zero_rle_decode(&encoded).unwrap();
        assert_eq!(decoded, vec![0x12, 0x34, 0x56, 0x78]);
        
        // Test mixed
        let encoded = vec![0x81, 0x01, 0xAB, 0xCD]; // 1 zero + 1 literal
        let decoded = zero_rle_decode(&encoded).unwrap();
        assert_eq!(decoded, vec![0, 0, 0xAB, 0xCD]);
    }
}

