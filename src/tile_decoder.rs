//! Tile-based video decoder for dirty tile encoding
//! 
//! Format:
//! - Header: "TILE" magic, version (u16), frame_count (u16)
//! - Frame offsets table: u32[frame_count]
//! - Per frame:
//!   - num_tiles (u16): number of changed tiles
//!   - For each tile:
//!     - tile_x (u8), tile_y (u8): tile coordinates
//!     - tile_data_len (u16): compressed tile data length
//!     - tile_data (bytes): RLE-compressed 16×16 RGB565 tile

extern crate alloc;

const TILE_SIZE: usize = 16;  // 16×16 tiles
const TILE_PIXELS: usize = TILE_SIZE * TILE_SIZE;
const TILE_BYTES: usize = TILE_PIXELS * 2;  // RGB565 = 2 bytes per pixel

/// Tile decoder for dirty tile format
pub struct TileDecoder<'a> {
    data: &'a [u8],
    frame_count: usize,
    offsets_table_start: usize,
    width: u32,
    height: u32,
    tiles_x: usize,
    tiles_y: usize,
}

impl<'a> TileDecoder<'a> {
    /// Create a new tile decoder from tile format data
    pub fn new(data: &'a [u8], width: u32, height: u32) -> Result<Self, &'static str> {
        if data.len() < 8 {
            return Err("Tile file too short (missing header)");
        }
        
        // Check magic
        if &data[0..4] != b"TILE" {
            return Err("Invalid tile file magic");
        }
        
        let version = u16::from_le_bytes([data[4], data[5]]);
        if version != 1 {
            return Err("Unsupported tile format version");
        }
        
        let frame_count = u16::from_le_bytes([data[6], data[7]]) as usize;
        
        if data.len() < 8 + frame_count * 4 {
            return Err("Tile file too short (missing frame offsets)");
        }
        
        // Verify frame offsets table exists
        let offsets_start = 8;
        if data.len() < offsets_start + frame_count * 4 {
            return Err("Frame offset table incomplete");
        }
        
        let tiles_x = ((width as usize) + TILE_SIZE - 1) / TILE_SIZE;
        let tiles_y = ((height as usize) + TILE_SIZE - 1) / TILE_SIZE;
        
        Ok(Self {
            data,
            frame_count,
            offsets_table_start: offsets_start,
            width,
            height,
            tiles_x,
            tiles_y,
        })
    }
    
    /// Get number of frames
    pub fn frame_count(&self) -> usize {
        self.frame_count
    }
    
    /// Get frame offset from table
    fn get_frame_offset(&self, frame_idx: usize) -> Result<usize, &'static str> {
        if frame_idx >= self.frame_count {
            return Err("Frame index out of range");
        }
        
        let off = self.offsets_table_start + frame_idx * 4;
        if off + 4 > self.data.len() {
            return Err("Frame offset out of bounds");
        }
        
        Ok(u32::from_le_bytes([
            self.data[off],
            self.data[off + 1],
            self.data[off + 2],
            self.data[off + 3],
        ]) as usize)
    }
    
    /// Decode a frame's tiles with a callback for each tile
    /// Calls callback(tile_x, tile_y, tile_data) for each decoded tile
    /// Returns number of tiles decoded
    pub fn decode_frame_with_callback<F>(
        &self,
        frame_idx: usize,
        prev_frame_tiles: &mut alloc::collections::BTreeMap<(u8, u8), alloc::vec::Vec<u8>>,
        mut callback: F,
    ) -> Result<usize, &'static str>
    where
        F: FnMut(u8, u8, &[u16]),
    {
        let frame_offset = self.get_frame_offset(frame_idx)?;
        if frame_offset >= self.data.len() {
            return Err("Frame offset out of range");
        }
        
        let frame_data = &self.data[frame_offset..];
        
        if frame_data.len() < 2 {
            return Err("Frame too short");
        }
        
        let num_tiles = u16::from_le_bytes([frame_data[0], frame_data[1]]) as usize;
        let mut pos = 2;
        
        let mut tiles_decoded = 0;
        
        for _ in 0..num_tiles {
            if pos + 4 >= frame_data.len() {
                return Err("Tile header out of bounds");
            }
            
            let tile_x = frame_data[pos];
            let tile_y = frame_data[pos + 1];
            let tile_data_len = u16::from_le_bytes([frame_data[pos + 2], frame_data[pos + 3]]) as usize;
            pos += 4;
            
            if pos + tile_data_len > frame_data.len() {
                return Err("Tile data out of bounds");
            }
            
            let compressed_tile = &frame_data[pos..pos + tile_data_len];
            pos += tile_data_len;
            
            // Decode RLE-compressed tile
            let mut tile_data = alloc::vec::Vec::<u8>::with_capacity(TILE_BYTES);
            self.decode_rle(compressed_tile, &mut tile_data)?;
            
            // Convert to u16 array (native little-endian for SPI LSB-first)
            // Tiles are stored as big-endian (MSB, LSB), but SPI sends LSB-first
            // So we swap bytes once here: (MSB, LSB) -> (LSB, MSB) for native little-endian
            let mut tile_u16 = alloc::vec::Vec::<u16>::with_capacity(TILE_PIXELS);
            for i in 0..(tile_data.len() / 2).min(TILE_PIXELS) {
                let msb = tile_data[i * 2];
                let lsb = tile_data[i * 2 + 1];
                // Swap bytes: (MSB, LSB) -> (LSB, MSB) for native little-endian u16
                // SPI will send LSB first, so this becomes MSB-first on the wire
                tile_u16.push(((lsb as u16) << 8) | (msb as u16));
            }
            while tile_u16.len() < TILE_PIXELS {
                tile_u16.push(0);
            }
            
            // Call callback with decoded tile data
            callback(tile_x, tile_y, &tile_u16);
            
            // Store raw tile data bytes for delta encoding (v1 doesn't use it, but store for compatibility)
            prev_frame_tiles.insert((tile_x, tile_y), tile_data);
            
            tiles_decoded += 1;
        }
        
        Ok(tiles_decoded)
    }
    
    /// Decode a frame's tiles into the framebuffer
    /// Returns number of tiles decoded
    pub fn decode_frame(
        &self,
        frame_idx: usize,
        framebuffer: &mut [u16],
        prev_frame_tiles: &mut alloc::collections::BTreeMap<(u8, u8), alloc::vec::Vec<u16>>,
    ) -> Result<usize, &'static str> {
        let frame_offset = self.get_frame_offset(frame_idx)?;
        if frame_offset >= self.data.len() {
            return Err("Frame offset out of range");
        }
        
        let frame_data = &self.data[frame_offset..];
        
        if frame_data.len() < 2 {
            return Err("Frame too short");
        }
        
        let num_tiles = u16::from_le_bytes([frame_data[0], frame_data[1]]) as usize;
        let mut pos = 2;
        
        let mut tiles_decoded = 0;
        
        for _ in 0..num_tiles {
            if pos + 4 >= frame_data.len() {
                return Err("Tile header out of bounds");
            }
            
            let tile_x = frame_data[pos];
            let tile_y = frame_data[pos + 1];
            let tile_data_len = u16::from_le_bytes([frame_data[pos + 2], frame_data[pos + 3]]) as usize;
            pos += 4;
            
            if pos + tile_data_len > frame_data.len() {
                return Err("Tile data out of bounds");
            }
            
            let compressed_tile = &frame_data[pos..pos + tile_data_len];
            pos += tile_data_len;
            
            // Decode RLE-compressed tile
            let mut tile_data = alloc::vec::Vec::<u8>::with_capacity(TILE_BYTES);
            self.decode_rle(compressed_tile, &mut tile_data)?;
            
            // Convert to u16 array (native little-endian for SPI LSB-first)
            // Tiles are stored as big-endian (MSB, LSB), but SPI sends LSB-first
            // So we swap bytes once here: (MSB, LSB) -> (LSB, MSB) for native little-endian
            let mut tile_u16 = alloc::vec::Vec::<u16>::with_capacity(TILE_PIXELS);
            for i in 0..(tile_data.len() / 2).min(TILE_PIXELS) {
                let msb = tile_data[i * 2];
                let lsb = tile_data[i * 2 + 1];
                // Swap bytes: (MSB, LSB) -> (LSB, MSB) for native little-endian u16
                // SPI will send LSB first, so this becomes MSB-first on the wire
                tile_u16.push(((lsb as u16) << 8) | (msb as u16));
            }
            while tile_u16.len() < TILE_PIXELS {
                tile_u16.push(0);
            }
            
            // Copy tile to framebuffer
            let fb_width = self.width as usize;
            let x_start = (tile_x as usize) * TILE_SIZE;
            let y_start = (tile_y as usize) * TILE_SIZE;
            
            for ty in 0..TILE_SIZE {
                let y = y_start + ty;
                if y >= self.height as usize {
                    break;
                }
                for tx in 0..TILE_SIZE {
                    let x = x_start + tx;
                    if x >= fb_width {
                        break;
                    }
                    let fb_idx = y * fb_width + x;
                    let tile_idx = ty * TILE_SIZE + tx;
                    if fb_idx < framebuffer.len() && tile_idx < tile_u16.len() {
                        framebuffer[fb_idx] = tile_u16[tile_idx];
                    }
                }
            }
            
            // Store tile for next frame comparison
            prev_frame_tiles.insert((tile_x, tile_y), tile_u16);
            tiles_decoded += 1;
        }
        
        Ok(tiles_decoded)
    }
    
    /// Simple RLE decoder
    fn decode_rle(&self, data: &[u8], output: &mut alloc::vec::Vec<u8>) -> Result<(), &'static str> {
        let mut i = 0;
        let max_output = TILE_BYTES;
        
        while i < data.len() && output.len() < max_output {
            if data[i] == 0xFF {  // RLE run
                if i + 2 >= data.len() {
                    break;
                }
                let value = data[i + 1];
                let count = data[i + 2] as usize;
                for _ in 0..count.min(max_output - output.len()) {
                    output.push(value);
                }
                i += 3;
            } else if data[i] == 0xFE {  // Literal run
                if i + 1 >= data.len() {
                    break;
                }
                let literal_count = data[i + 1] as usize;
                if i + 2 + literal_count > data.len() {
                    break;
                }
                let to_copy = literal_count.min(max_output - output.len());
                output.extend_from_slice(&data[i + 2..i + 2 + to_copy]);
                i += 2 + literal_count;
            } else {
                // Raw byte (shouldn't happen, but handle gracefully)
                if output.len() < max_output {
                    output.push(data[i]);
                }
                i += 1;
            }
        }
        
        // Pad to expected size
        while output.len() < max_output {
            output.push(0);
        }
        
        Ok(())
    }
}

