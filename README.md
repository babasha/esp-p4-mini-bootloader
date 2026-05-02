# esp-p4-mini-bootloader

All-Rust 1st-stage bootloader for the ESP32-P4. Replaces the IDF v5.3
`bootloader.bin` C blob at flash @ 0x2000.

> **Status (2026-05-02):** end-to-end on Waveshare ESP32-P4-ETH. Chip POR ⇒
> ROM ⇒ this bootloader ⇒ Rust app ⇒ PSRAM 32 MB smoke test PASS.

## Why this exists

The IDF v5.3 2nd-stage bootloader's `unpack_load_app` for ESP32-P4 (in
`bootloader_utility.c:745`, `SOC_MMU_DI_VADDR_SHARED` path) hard-asserts
that the application image has exactly **two** segments with `load_addr`
in the `SOC_DROM_LOW..SOC_DROM_HIGH` range — i.e. two segments of cache-
mapped flash-XIP code/rodata. That's the standard IDF C application
layout.

Pure-Rust no_std applications running on HP SRAM (e.g. anything built
with `riscv-rt` and a single RAM region in `memory.x`) have a single
segment whose LMA is in HP SRAM. They never satisfy `rom_index == 2`.
The IDF bootloader correctly detects them as a valid image (header magic
matches, partition table parses, app descriptor verifies, segment loads
into HP SRAM) and then asserts because we don't have flash-XIP segments,
silently rebooting in a loop.

`esp-bootloader-esp-idf` doesn't yet support ESP32-P4 either (verified
v0.4.0, v0.5.0, `main` as of 2026-04-30). Result: no off-the-shelf path
for booting Rust apps from flash on P4 today.

This crate is that path.

## What it does

```
ROM     loads us from flash @ 0x2000 per the standard esp_image_header_t
        format (single segment @ 0x4FF20000, ~22 KB)
        ├─ wdt::disable_all        WDT_EN + flashboot_mod_en cleared
mini-bl ├─ init_phase2_full        chip-wide bring-up (PMU/MPLL/cache/MMU)
        ├─ psram::init             32 MB Hex PSRAM up at 0x48000000
        ├─ partition::find_factory  walks PT @ 0x8000, magic 0x50AA
        ├─ app::read_header         parses esp_image_header_t @ 0x10000
        ├─ verify::verify_app_sha256  stream-SHA256 vs appended digest
        ├─ app::load_segments       memcpy flash → segment.load_addr
        ├─ Cache_WriteBack_All(L1D|L2)
        ├─ Cache_Invalidate_All(L1I|L2)
        ├─ fence.i
        └─ jr entry_addr
app                                runs natively from HP SRAM and/or PSRAM
```

About 22 KB of Rust at flash @ 0x2000. Boot to user `main()` is ~10 ms.

## Quick start

This is a binary crate, not a library. Clone, build, flash:

```sh
git clone https://github.com/babasha/esp-p4-mini-bootloader
cd esp-p4-mini-bootloader

# Build the bootloader image
cargo build --release --target riscv32imafc-unknown-none-elf

# Convert ELF to esp-idf image format
espflash save-image \
    --chip esp32p4 -m dio -f 80mhz -s 2mb --ignore-app-descriptor \
    target/riscv32imafc-unknown-none-elf/release/mini-bootloader \
    mini-bootloader.bin
```

Then flash all three blobs to your board (bootloader at 0x2000, partition
table at 0x8000, your Rust app at 0x10000). Use `espflash flash` with
`--bootloader mini-bootloader.bin --partition-table example-partitions.csv`
or write each blob individually with `espflash write-bin`.

The included `example-partitions.csv` is a 2 MB single-app layout:

```
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x100000,
```

Your application must:
- be a `riscv32imafc-unknown-none-elf` ELF
- include a 256-byte `esp_app_desc_t` at the start of segment 0 (we read
  `min_efuse_blk_rev_full` from it; bytes can be zero — see app_desc.rs
  in the example app at `examples/`)
- be in standard esp-idf image format (espflash produces this by default)

## Reading UART after flashing (Waveshare ESP32-P4-ETH)

`espflash --monitor` is broken on `--ram --no-stub` boots for P4 (chip
resets every ~70 ms). Use raw pyserial. To trigger a clean POR while
listening, **pulse RTS** (not DTR — DTR has no effect on this hardware):

```python
import serial, time
s = serial.Serial(); s.port='COM5'; s.baudrate=115200
s.dsrdtr=False; s.rtscts=False; s.dtr=False; s.rts=False; s.timeout=0.05
s.open()
s.rts = True; time.sleep(0.1); s.rts = False  # pulse → POR
# now read s ...
```

## Status & TODOs

Working:
- [x] ROM loads us from flash @ 0x2000
- [x] WDT disable (incl. flashboot_mod_en separate enable path)
- [x] Chip-wide init via [`esp-p4-bootloader`]
- [x] PSRAM init via [`esp-p4-psram`]
- [x] ROM `esp_rom_spiflash_read` (@ 0x4FC00158) wrapped, alignment-safe
- [x] Partition table parser (`esp_partition_info_t`, magic 0x50AA)
- [x] App image header parse + segment load
- [x] D-cache writeback + I-cache invalidate + fence.i + jr (mandatory)
- [x] SHA256 verify of appended digest (refuses jump on mismatch)
- [x] Heartbeat fallback when app load fails (preserves serial recovery)

Not yet:
- [ ] OTA partition selection (currently hardcoded `factory`)
- [ ] Secure boot V2 signature verify
- [ ] Flash encryption transparent decrypt
- [ ] Anti-rollback (`secure_version` from app_desc)

PRs welcome.

[`esp-p4-bootloader`]: https://github.com/babasha/esp-p4-bootloader
[`esp-p4-psram`]:      https://github.com/babasha/esp-p4-psram

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE).
