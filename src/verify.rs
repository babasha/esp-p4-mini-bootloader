//! SHA256 verification of the on-flash app image.
//!
//! When `esp_image_header_t.hash_appended == 1`, espflash / IDF append
//! a 32-byte SHA256 digest after the image's segments + checksum. We
//! recompute the digest by stream-reading the same byte range and
//! compare. Mismatch ⇒ refuse to jump (corruption / partial write).
//!
//! Hashing in software with the `sha2` crate (~10 KB code). The chip's
//! ROM SHA hardware accelerator (`ets_sha_*` @ 0x4FC0_0614+) is faster
//! but its `SHA_CTX` layout is chip-specific and not in our reference
//! headers; if SHA cost ever shows up in boot time we can swap.

#![allow(dead_code)]

use sha2::{Digest, Sha256};

use crate::app::{AppLoadError, ImageHeader};
use crate::flash;

const HASH_LEN: usize = 32;

#[derive(Debug, Clone, Copy)]
pub enum VerifyError {
    /// Reading flash failed during digest streaming.
    Flash(flash::FlashError),
    /// `hash_appended == 0` — image declares no SHA, nothing to verify.
    NoHash,
    /// Computed digest doesn't match the appended one.
    Mismatch {
        expected: [u8; HASH_LEN],
        actual: [u8; HASH_LEN],
    },
    /// Total image size couldn't be determined (segment walk failed).
    TotalSize(AppLoadError),
}

impl From<flash::FlashError> for VerifyError {
    fn from(e: flash::FlashError) -> Self {
        Self::Flash(e)
    }
}

impl From<AppLoadError> for VerifyError {
    fn from(e: AppLoadError) -> Self {
        match e {
            AppLoadError::Flash(f) => Self::Flash(f),
            other => Self::TotalSize(other),
        }
    }
}

/// Stream-hash the image and compare to the appended SHA256.
///
/// `flash_offset` points at the `esp_image_header_t` (i.e. the start of
/// the factory partition for our singleapp layout).
pub fn verify_app_sha256(
    flash_offset: u32,
    header: &ImageHeader,
) -> Result<[u8; HASH_LEN], VerifyError> {
    if !header.hash_appended {
        return Err(VerifyError::NoHash);
    }

    let total = crate::app::compute_image_total_size(flash_offset, header)?;
    let payload_len = (total as usize) - HASH_LEN;

    // Stream-read the payload in 1 KB aligned chunks. 1 KB balances
    // stack pressure (we live in 64 KB SRAM with the riscv-rt stack)
    // against ROM read overhead per call.
    const CHUNK_LEN: usize = 1024;
    // 4-byte aligned via [u32; N].
    let mut buf32 = [0u32; CHUNK_LEN / 4];
    let buf_ptr = buf32.as_mut_ptr() as *mut u8;

    // payload_len is 16-byte aligned by IDF image format, so every
    // chunk len is automatically 4-byte aligned for ROM SPI flash read.
    let mut hasher = Sha256::new();
    let mut read = 0usize;
    while read < payload_len {
        let want = core::cmp::min(CHUNK_LEN, payload_len - read);
        // SAFETY: 4-byte aligned dest by [u32; _], `want` is 4-aligned.
        unsafe { flash::read(flash_offset + read as u32, buf_ptr, want)? };
        let slice: &[u8] = unsafe { core::slice::from_raw_parts(buf_ptr, want) };
        hasher.update(slice);
        read += want;
    }
    let actual_digest: [u8; HASH_LEN] = hasher.finalize().into();

    // Read the appended digest.
    let mut expected = [0u8; HASH_LEN];
    let mut expect_buf = [0u32; HASH_LEN / 4];
    // SAFETY: 4-byte aligned, len 32.
    unsafe {
        flash::read(
            flash_offset + payload_len as u32,
            expect_buf.as_mut_ptr() as *mut u8,
            HASH_LEN,
        )?;
    }
    let src: &[u8] =
        unsafe { core::slice::from_raw_parts(expect_buf.as_ptr() as *const u8, HASH_LEN) };
    expected.copy_from_slice(src);

    if actual_digest != expected {
        return Err(VerifyError::Mismatch {
            expected,
            actual: actual_digest,
        });
    }
    Ok(actual_digest)
}
