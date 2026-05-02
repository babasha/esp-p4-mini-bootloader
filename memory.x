/*
 * ESP32-P4 mini-bootloader memory map.
 *
 * The ROM bootloader loads us from flash @ 0x2000 according to the
 * standard `esp_image_header_t` format. We live in the lower half of
 * HP SRAM (0x4FF20000+) — the same area the IDF v5.3 bootloader uses,
 * which leaves the upper portion (0x4FF50000..0x4FF80000) free for the
 * app payload. L2 cache backing is at 0x4FF80000+ (NEVER touch).
 *
 * 64 KB total: enough room for code + small stack. IDF bootloader
 * fits in ~22 KB so Rust should fit similarly.
 *
 * App image is loaded by us into 0x4FF50000+ (via flash read + memcpy);
 * once we jump to the app's entry the bootloader region (0x4FF20000)
 * becomes free DRAM the app can re-use if it wants.
 *
 * No app_desc here — that's only checked by the IDF 2nd-stage bootloader
 * for app images. The ROM 1st-stage just verifies the magic byte and
 * loads our segments.
 */
MEMORY
{
    RAM : ORIGIN = 0x4FF20000, LENGTH = 0x00010000
}

REGION_ALIAS("REGION_TEXT", RAM);
REGION_ALIAS("REGION_RODATA", RAM);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

SECTIONS
{
    /DISCARD/ :
    {
        *(.eh_frame)
        *(.eh_frame_hdr)
    }
}
