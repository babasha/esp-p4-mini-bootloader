//! Parse the IDF partition table at flash @ 0x8000.
//!
//! Layout: `esp_partition_info_t[]` (32-byte entries) terminated by an
//! entry whose `type == 0xFF`. Each valid entry has `magic == 0x50AA`.
//! Optional MD5 entry `magic == 0xEBEB` may appear once and is skipped.
//!
//! Reference: IDF v5.3 `bootloader_support/include/esp_flash_partitions.h`.

// Public API surface — callers may want every constant, even if the
// bootloader itself only uses a subset right now.
#![allow(dead_code)]

use crate::flash;

pub const PARTITION_TABLE_OFFSET: u32 = 0x8000;
/// Per IDF: max 0xC00 bytes (96 entries × 32 B). We read 1 KB which fits
/// any realistic singleapp/OTA layout (32 entries) and saves stack.
pub const PARTITION_TABLE_READ_LEN: usize = 0x400;

const ESP_PARTITION_MAGIC: u16 = 0x50AA;
const ESP_PARTITION_MAGIC_MD5: u16 = 0xEBEB;

pub const PART_TYPE_APP: u8 = 0x00;
pub const PART_TYPE_DATA: u8 = 0x01;
pub const PART_TYPE_END: u8 = 0xFF;

pub const PART_SUBTYPE_FACTORY: u8 = 0x00;

#[derive(Debug, Clone, Copy)]
pub struct PartitionEntry {
    pub kind: u8,
    pub subtype: u8,
    pub offset: u32,
    pub size: u32,
    /// Raw label bytes (NUL-terminated within 16 bytes).
    pub label: [u8; 16],
}

impl PartitionEntry {
    /// Decode 32 bytes into a typed entry. Returns `None` if magic is wrong
    /// (caller decides whether that means terminator vs corruption).
    pub fn parse(bytes: &[u8; 32]) -> Option<Self> {
        let magic = u16::from_le_bytes([bytes[0], bytes[1]]);
        if magic != ESP_PARTITION_MAGIC {
            return None;
        }
        let kind = bytes[2];
        let subtype = bytes[3];
        let offset = u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let size = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        let mut label = [0u8; 16];
        label.copy_from_slice(&bytes[12..28]);
        Some(Self {
            kind,
            subtype,
            offset,
            size,
            label,
        })
    }

    /// Label as a `&str`, trimming NULs. Falls back to empty on non-UTF8.
    pub fn label_str(&self) -> &str {
        let n = self.label.iter().position(|&b| b == 0).unwrap_or(16);
        core::str::from_utf8(&self.label[..n]).unwrap_or("")
    }
}

/// Read the partition table once and find the factory app entry. Returns
/// its (offset, size) on success, or an error.
#[derive(Debug, Clone, Copy)]
pub enum PartitionError {
    Flash(flash::FlashError),
    /// Walked all entries up to the terminator without finding factory.
    NoFactoryApp,
}

impl From<flash::FlashError> for PartitionError {
    fn from(e: flash::FlashError) -> Self {
        Self::Flash(e)
    }
}

/// Walk the partition table, calling `f` for each valid entry. Stops on
/// the first terminator (non-magic entry). MD5 markers are skipped.
fn for_each_entry<F: FnMut(&PartitionEntry) -> bool>(mut f: F) -> Result<(), flash::FlashError> {
    const N: usize = PARTITION_TABLE_READ_LEN / 4;
    let mut buf32 = [0u32; N];
    // SAFETY: 4-byte aligned by [u32; N]; len is N*4 = PARTITION_TABLE_READ_LEN.
    unsafe {
        flash::read(
            PARTITION_TABLE_OFFSET,
            buf32.as_mut_ptr() as *mut u8,
            PARTITION_TABLE_READ_LEN,
        )?;
    }
    // SAFETY: same buffer, reinterpret as bytes for entry walk.
    let bytes: &[u8] =
        unsafe { core::slice::from_raw_parts(buf32.as_ptr() as *const u8, PARTITION_TABLE_READ_LEN) };

    let mut entry = [0u8; 32];
    for chunk_start in (0..PARTITION_TABLE_READ_LEN).step_by(32) {
        entry.copy_from_slice(&bytes[chunk_start..chunk_start + 32]);
        let magic = u16::from_le_bytes([entry[0], entry[1]]);
        match magic {
            ESP_PARTITION_MAGIC => {
                if let Some(e) = PartitionEntry::parse(&entry) {
                    if !f(&e) {
                        return Ok(());
                    }
                }
            }
            ESP_PARTITION_MAGIC_MD5 => { /* skip */ }
            _ => break, // terminator
        }
    }
    Ok(())
}

pub fn find_factory_app() -> Result<PartitionEntry, PartitionError> {
    let mut found: Option<PartitionEntry> = None;
    for_each_entry(|e| {
        if e.kind == PART_TYPE_APP && e.subtype == PART_SUBTYPE_FACTORY {
            found = Some(*e);
            false // stop walking
        } else {
            true // keep walking
        }
    })?;
    found.ok_or(PartitionError::NoFactoryApp)
}

/// Find a partition entry by label (NUL-trimmed). Returns the first
/// match; partition labels are unique by IDF convention.
pub fn find_by_label(label: &str) -> Result<PartitionEntry, PartitionError> {
    let mut found: Option<PartitionEntry> = None;
    for_each_entry(|e| {
        if e.label_str() == label {
            found = Some(*e);
            false
        } else {
            true
        }
    })?;
    found.ok_or(PartitionError::NoFactoryApp)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_real_partition_entry_decodes_offset_and_label() {
        // Real bytes captured from flash@0x8000 by Phase 2 read.
        let entry: [u8; 32] = [
            0xAA, 0x50, 0x01, 0x02, 0x00, 0x90, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x6E, 0x76,
            0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00,
        ];
        let p = PartitionEntry::parse(&entry).unwrap();
        assert_eq!(p.kind, PART_TYPE_DATA);
        assert_eq!(p.subtype, 0x02);
        assert_eq!(p.offset, 0x9000);
        assert_eq!(p.size, 0x6000);
        assert_eq!(p.label_str(), "nvs");
    }

    #[test]
    fn parse_rejects_bad_magic() {
        let mut entry = [0u8; 32];
        entry[0] = 0x55;
        entry[1] = 0x55;
        assert!(PartitionEntry::parse(&entry).is_none());
    }
}
