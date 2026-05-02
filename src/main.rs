#![no_std]
#![no_main]

//! ESP32-P4 mini-bootloader — Phase 1 skeleton.
//!
//! The chip ROM bootloader loads this image from flash @ 0x2000 (per the
//! standard `esp_image_header_t` format) and jumps to our `_start`. We
//! then bring the chip up to a state where the app image can be loaded
//! and executed.
//!
//! Phase 1 (this file) does only:
//!   1. UART hello
//!   2. `bootloader::init_phase2_full()` — full chip-wide init
//!   3. UART confirmation + heartbeat
//!
//! Future phases will add:
//!   - ROM SPI flash read of partition table @ 0x8000
//!   - Locate factory app partition
//!   - Load app image segments via memcpy from flash
//!   - Jump to app entry_addr
//!
//! See task list and `MEMORY.md` reference `project_smartbox_bootloader`
//! for the multi-session plan.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use panic_halt as _;
use riscv_rt::entry;

mod app;
mod flash;
mod partition;
mod verify;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

struct Uart0;

impl Uart0 {
    fn write_byte(&mut self, byte: u8) {
        while txfifo_count() >= UART_TXFIFO_CAPACITY {
            spin_loop();
        }
        unsafe { write_volatile(UART_FIFO_REG, byte as u32) };
    }
}

impl Write for Uart0 {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            if b == b'\n' {
                self.write_byte(b'\r');
            }
            self.write_byte(b);
        }
        Ok(())
    }
}

fn txfifo_count() -> u32 {
    (unsafe { read_volatile(UART_STATUS_REG) } & UART_TXFIFO_CNT_MASK) >> UART_TXFIFO_CNT_SHIFT
}

#[entry]
fn main() -> ! {
    let mut uart = Uart0;
    let _ = writeln!(uart, "\n=== mini-bootloader alive @ 0x4FF20000 ===");

    // Disable WDTs FIRST — ROM `wdt_flashboot_mod_en` would otherwise
    // fire ~1 s into our boot.
    bootloader::wdt::disable_all();

    bootloader::init_phase2_full();
    let _ = writeln!(uart, "mini-bootloader: init_phase2_full() done");

    // Phase 5b: PSRAM up. Apps can place segments at LMA in 0x48000000+
    // (PSRAM virtual window) and we'll memcpy them through cache to PSRAM
    // during load_segments. PSRAM failure is non-fatal — HP-SRAM-only
    // apps continue to boot; PSRAM-segment apps will fault on first
    // 0x48xxxxxx access (which is preferable to skipping the load and
    // jumping into garbage). Idempotent re-init by the app is harmless.
    match psram::init() {
        Ok(regs) => {
            let _ = writeln!(
                uart,
                "psram::init() OK vendor=0x{:02X} density=0x{:X} ({} MB)",
                regs.mr1.vendor_id(),
                regs.mr2.density(),
                psram::PsramSize::from_mr2_density(regs.mr2.density())
                    .map(|s| s.bytes() / (1024 * 1024))
                    .unwrap_or(0)
            );
        }
        Err(e) => {
            let _ = writeln!(uart, "psram::init() FAILED: {} (continuing)", e);
        }
    }

    // Phase 3: parse the partition table and find the factory app.
    let factory = match partition::find_factory_app() {
        Ok(p) => {
            let _ = writeln!(
                uart,
                "factory app: label={:?} offset=0x{:X} size=0x{:X} ({} KB)",
                p.label_str(),
                p.offset,
                p.size,
                p.size / 1024,
            );
            p
        }
        Err(e) => {
            let _ = writeln!(uart, "partition::find_factory_app ERR {:?}", e);
            heartbeat(&mut uart);
        }
    };

    // Phase 4: load the app image segments and jump.
    let header = match app::read_header(factory.offset) {
        Ok(h) => {
            let _ = writeln!(
                uart,
                "app header: magic=0x{:02X} segs={} entry=0x{:08X} chip=0x{:04X} hash={}",
                h.magic, h.segment_count, h.entry_addr, h.chip_id, h.hash_appended
            );
            h
        }
        Err(e) => {
            let _ = writeln!(uart, "app::read_header ERR {:?}", e);
            heartbeat(&mut uart);
        }
    };

    // Phase 5: verify the appended SHA256. Refuse to jump on mismatch
    // (corruption / partial flash / tamper). Skip silently if the
    // image was built without `hash_appended` (rare for espflash).
    if header.hash_appended {
        match verify::verify_app_sha256(factory.offset, &header) {
            Ok(d) => {
                let _ = writeln!(
                    uart,
                    "sha256: OK ({:02x}{:02x}{:02x}{:02x}…{:02x}{:02x}{:02x}{:02x})",
                    d[0], d[1], d[2], d[3], d[28], d[29], d[30], d[31]
                );
            }
            Err(verify::VerifyError::Mismatch { expected, actual }) => {
                let _ = writeln!(uart, "sha256: MISMATCH — refusing to boot");
                let _ = write!(uart, "  expected:");
                for b in &expected {
                    let _ = write!(uart, " {:02x}", b);
                }
                let _ = writeln!(uart);
                let _ = write!(uart, "  actual:  ");
                for b in &actual {
                    let _ = write!(uart, " {:02x}", b);
                }
                let _ = writeln!(uart);
                heartbeat(&mut uart);
            }
            Err(e) => {
                let _ = writeln!(uart, "sha256: ERR {:?} — refusing to boot", e);
                heartbeat(&mut uart);
            }
        }
    } else {
        let _ = writeln!(uart, "sha256: skipped (image declares hash_appended=0)");
    }

    // SAFETY: app's segment LMAs are above 0x4FF50000 (boot-test) and
    // our bootloader runs at 0x4FF20000..0x4FF30000, so no collision.
    // After this returns, all segments are sitting at their load_addrs.
    if let Err(e) = unsafe {
        app::load_segments(factory.offset, &header, |i, la, dl| {
            let _ = writeln!(
                &mut Uart0,
                "  seg{}: load=0x{:08X} len={} ({} KB)",
                i,
                la,
                dl,
                dl / 1024
            );
        })
    } {
        let _ = writeln!(uart, "app::load_segments ERR {:?}", e);
        heartbeat(&mut uart);
    }

    let _ = writeln!(
        uart,
        "mini-bootloader: jumping to app @ 0x{:08X}",
        header.entry_addr
    );
    drain_uart();

    // SAFETY: segments are loaded; app's own _start sets up its stack.
    // Bootloader region (0x4FF20000+) becomes free DRAM for the app.
    unsafe { app::jump_to_entry(header.entry_addr) }
}

fn drain_uart() {
    // Wait for FIFO to flush before handoff.
    while txfifo_count() > 0 {
        spin_loop();
    }
}

fn heartbeat(uart: &mut Uart0) -> ! {
    let mut tick: u32 = 0;
    loop {
        use core::fmt::Write;
        let _ = writeln!(uart, "bl-tick {}", tick);
        for _ in 0..2_000_000 {
            spin_loop();
        }
        tick = tick.wrapping_add(1);
    }
}
