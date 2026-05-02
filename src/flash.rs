//! Read flash via ROM functions.
//!
//! At ROM-handoff the MSPI flash controller is already brought up (the
//! ROM had to read our bootloader from flash@0x2000 to load it here).
//! We can keep using ROM SPI flash routines without re-init — the ROM
//! state survives our `init_phase2_full()` because flash MSPI sits on
//! `MSPI_0/1` while our PSRAM-side init touches `MSPI_2/3`.
//!
//! Reference: IDF v5.3 `rom_p4/ld/esp32p4.rom.ld`. Address verified
//! to be `0x4FC00158` for `esp_rom_spiflash_read`.

#![allow(dead_code)]

const ESP_ROM_SPIFLASH_READ: usize = 0x4FC0_0158;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    /// Length is not a multiple of 4 bytes — ROM requires word reads.
    Unaligned,
    /// ROM returned non-zero status.
    Rom(i32),
}

/// Read `len` bytes from flash physical address `addr` into `dest`.
///
/// `dest` must be 4-byte aligned, `len` must be a multiple of 4.
/// (ROM enforces both via internal asserts on its slow path.)
///
/// SAFETY:
/// - `dest` must be a valid mutable pointer to `len` bytes.
/// - Caller must ensure no concurrent flash access (we are single-hart
///   at boot, so this is trivially satisfied).
pub unsafe fn read(addr: u32, dest: *mut u8, len: usize) -> Result<(), FlashError> {
    if len % 4 != 0 || (dest as usize) % 4 != 0 {
        return Err(FlashError::Unaligned);
    }
    type RomFn = unsafe extern "C" fn(target: u32, dest: *mut u32, len: i32) -> i32;
    let f: RomFn = unsafe { core::mem::transmute(ESP_ROM_SPIFLASH_READ) };
    let r = unsafe { f(addr, dest as *mut u32, len as i32) };
    if r == 0 {
        Ok(())
    } else {
        Err(FlashError::Rom(r))
    }
}

/// Read a fixed-size array from flash. Convenience wrapper over [`read`]
/// for when you have an aligned `[u32; N]` or `[u8; N]` (N divisible by 4).
pub fn read_into<const N: usize>(addr: u32, buf: &mut [u8; N]) -> Result<(), FlashError> {
    // SAFETY: buf is a valid &mut to N bytes; we just took a pointer
    // through `as_mut_ptr`. N is a const so alignment is enforced by
    // the caller's choice of array.
    unsafe { read(addr, buf.as_mut_ptr(), N) }
}

/// Read a 32-bit word from flash. Always succeeds barring ROM error.
pub fn read_u32(addr: u32) -> Result<u32, FlashError> {
    let mut buf = [0u32; 1];
    let ptr = buf.as_mut_ptr() as *mut u8;
    // SAFETY: 4-byte aligned by the array, len is 4.
    unsafe { read(addr, ptr, 4)? };
    Ok(buf[0])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_rejects_unaligned_len() {
        let mut buf = [0u8; 7];
        let r = unsafe { read(0x8000, buf.as_mut_ptr(), 7) };
        assert_eq!(r, Err(FlashError::Unaligned));
    }
}
