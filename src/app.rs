//! Load an IDF-format application image from flash and jump to its
//! entry point.
//!
//! Image layout (`esp_image_header_t` + N × `esp_image_segment_header_t`
//! + segment data, in flash, packed):
//!
//! ```text
//! offset 0x00..0x18   esp_image_header_t (24 B): magic, segment_count,
//!                     spi_mode, spi_size_freq, entry_addr, wp_pin,
//!                     pin_drv[3], chip_id, ...
//! offset 0x18..0x20   segment[0] header (8 B): load_addr, data_len
//! offset 0x20..       segment[0] data
//! ...                 next segment header right after, no padding
//! end                 1 B XOR checksum padded to 16 B alignment +
//!                     optional 32 B SHA256 (if header.hash_appended)
//! ```
//!
//! Reference: IDF v5.3 `bootloader_support/include/esp_app_format.h`.

#![allow(dead_code)]

use crate::flash;

const ESP_IMAGE_HEADER_MAGIC: u8 = 0xE9;

#[derive(Debug, Clone, Copy)]
pub struct ImageHeader {
    pub magic: u8,
    pub segment_count: u8,
    pub entry_addr: u32,
    pub chip_id: u16,
    /// `1` ⇒ a 32-byte SHA256 of the rest of the image is appended at
    /// the end (after segments + 16-byte-aligned checksum). The IDF
    /// 2nd-stage bootloader and our `verify` pass use this to detect
    /// flash corruption / partial writes before jumping.
    pub hash_appended: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum AppLoadError {
    Flash(flash::FlashError),
    BadMagic(u8),
    /// Segment data_len too large for any plausible HP SRAM target.
    SegmentTooLarge(u32),
    /// Non-aligned data_len that ROM SPI flash read can't handle.
    UnalignedSegment(u32),
}

impl From<flash::FlashError> for AppLoadError {
    fn from(e: flash::FlashError) -> Self {
        Self::Flash(e)
    }
}

/// Read the 24-byte image header from flash.
pub fn read_header(flash_offset: u32) -> Result<ImageHeader, AppLoadError> {
    let mut buf = [0u32; 6]; // 24 bytes
    // SAFETY: 4-byte aligned.
    unsafe { flash::read(flash_offset, buf.as_mut_ptr() as *mut u8, 24)? };
    let bytes: &[u8; 24] = unsafe { &*(buf.as_ptr() as *const [u8; 24]) };
    let magic = bytes[0];
    if magic != ESP_IMAGE_HEADER_MAGIC {
        return Err(AppLoadError::BadMagic(magic));
    }
    Ok(ImageHeader {
        magic,
        segment_count: bytes[1],
        entry_addr: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        chip_id: u16::from_le_bytes([bytes[12], bytes[13]]),
        hash_appended: bytes[23] != 0,
    })
}

/// Compute the total on-flash size of the app image — image header +
/// each segment header + segment data, padded with the 1-byte XOR
/// checksum to a 16-byte boundary, plus the trailing 32-byte SHA256
/// when `hash_appended` is set.
///
/// Walks segment headers in flash without copying segment data.
pub fn compute_image_total_size(
    flash_offset: u32,
    header: &ImageHeader,
) -> Result<u32, AppLoadError> {
    const IMAGE_HEADER_SIZE: u32 = 24;
    const SEGMENT_HEADER_SIZE: u32 = 8;
    const HASH_LEN: u32 = 32;

    let mut total: u32 = IMAGE_HEADER_SIZE;
    let mut pos = flash_offset + IMAGE_HEADER_SIZE;
    for _ in 0..header.segment_count {
        let mut hdr = [0u32; 2];
        // SAFETY: 4-byte aligned by [u32; 2].
        unsafe { flash::read(pos, hdr.as_mut_ptr() as *mut u8, 8)? };
        let data_len = hdr[1];
        if data_len > 0x80_0000 {
            return Err(AppLoadError::SegmentTooLarge(data_len));
        }
        total += SEGMENT_HEADER_SIZE + data_len;
        pos += SEGMENT_HEADER_SIZE + data_len;
    }
    total += 1; // 1-byte XOR checksum
    total = (total + 15) & !15; // round up to 16-byte alignment
    if header.hash_appended {
        total += HASH_LEN;
    }
    Ok(total)
}

/// Load all segments described by the header into their `load_addr`s.
/// Returns the entry address that the caller should jump to.
///
/// SAFETY: copies into arbitrary memory pointed-to by each segment's
/// `load_addr`. The caller is responsible for ensuring those address
/// ranges (a) are valid HP SRAM and (b) don't collide with anything
/// the bootloader currently uses (incl. its own stack at 0x4FF20000).
pub unsafe fn load_segments<F: FnMut(usize, u32, u32)>(
    flash_offset: u32,
    header: &ImageHeader,
    mut on_segment: F,
) -> Result<(), AppLoadError> {
    // Walk past the image header.
    let mut pos = flash_offset + 24; // sizeof(esp_image_header_t)

    for i in 0..header.segment_count as usize {
        // Read segment header (8 B): load_addr, data_len.
        let mut hdr = [0u32; 2];
        // SAFETY: 4-byte aligned by [u32; 2].
        unsafe { flash::read(pos, hdr.as_mut_ptr() as *mut u8, 8)? };
        let load_addr = hdr[0];
        let data_len = hdr[1];
        pos += 8;

        if data_len > 0x80_0000 {
            return Err(AppLoadError::SegmentTooLarge(data_len));
        }
        if data_len % 4 != 0 {
            return Err(AppLoadError::UnalignedSegment(data_len));
        }

        on_segment(i, load_addr, data_len);

        // Stream the segment data straight into its target HP SRAM.
        // ROM `esp_rom_spiflash_read` requires 4-byte aligned dest
        // and 4-byte aligned len — both satisfied here (load_addr is
        // aligned per IDF image format, data_len is enforced above).
        // SAFETY: load_addr is the LMA chosen by the app's linker;
        // bootloader's caller validates it doesn't clobber us.
        unsafe { flash::read(pos, load_addr as *mut u8, data_len as usize)? };
        pos += data_len;
    }

    Ok(())
}

/// Jump to the loaded app's entry point. Never returns.
///
/// Before the jump we (1) writeback L1 D-cache + L2 — segment stores
/// went through D-cache, instruction fetch only sees backing memory —
/// and (2) invalidate L1 I-cache + L2 so the CPU re-fetches the freshly
/// written code rather than any stale lines. Skipping either step
/// leaves the chip silently jumping into uninitialised memory.
///
/// SAFETY: caller must have populated the segments at their load_addrs
/// and ensured that the bootloader's code/data above 0x4FF20000 is no
/// longer needed (we don't restore anything). The app is responsible
/// for setting up its own stack via its `_start` (riscv-rt does this).
pub unsafe fn jump_to_entry(entry_addr: u32) -> ! {
    // ROM cache helpers. Bit map (per IDF v5.3 cache_ll_p4.h):
    //   bit 0 = L1 I-cache core0
    //   bit 1 = L1 I-cache core1
    //   bit 4 = L1 D-cache
    //   bit 5 = L2 cache
    const ROM_CACHE_WRITEBACK_ALL: usize = 0x4FC0_0414;
    const ROM_CACHE_INVALIDATE_ALL: usize = 0x4FC0_0404;
    const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
    const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    const CACHE_MAP_L2: u32 = 1 << 5;
    type CacheAll = unsafe extern "C" fn(map: u32) -> i32;

    // SAFETY: ROM functions, well-known signatures.
    unsafe {
        let wb: CacheAll = core::mem::transmute(ROM_CACHE_WRITEBACK_ALL);
        let _ = wb(CACHE_MAP_L1_DCACHE | CACHE_MAP_L2);
        let inv: CacheAll = core::mem::transmute(ROM_CACHE_INVALIDATE_ALL);
        let _ = inv(CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1 | CACHE_MAP_L2);
        // Synchronise the i-fetch stream — required after self-modifying-
        // style writes to executable memory.
        core::arch::asm!("fence.i", options(nomem, nostack));
        core::arch::asm!(
            "jr {0}",
            in(reg) entry_addr,
            options(noreturn)
        );
    }
}
