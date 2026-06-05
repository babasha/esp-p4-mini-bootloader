#![no_std]
#![no_main]

//! ESP32-P4 mini-bootloader.
//!
//! The chip ROM bootloader loads this image from flash @ 0x2000 (per
//! the standard `esp_image_header_t` format) and jumps to our `_start`.
//! From there we:
//!
//! 1. Disable the ROM watchdogs and bring the chip fully up via
//!    [`bootloader::init_phase2_full`].
//! 2. Read the HP-system reset cause and surface any pending crash dump
//!    / boot-history / bootstat record on UART before we touch the
//!    PSRAM or app paths (so a recurring crash that faults during
//!    those steps still gets last-boot diagnostics out).
//! 3. Bring up PSRAM (non-fatal — HP-SRAM-only apps still boot).
//! 4. Pick the active app slot from the otadata partition (A/B with
//!    automatic rollback after `MAX_BOOT_ATTEMPTS` failed boots),
//!    falling back to `factory` if both OTA slots are empty / corrupt.
//! 5. Read the app image header, verify its appended SHA-256, load
//!    segments through cache, arm the early-boot watchdog, and jump
//!    to the entry point.
//!
//! On any unrecoverable failure (no app, header bad, hash mismatch,
//! load error) we drop into [`heartbeat`] — a UART-only spin loop —
//! so the device is recoverable over serial.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::panic::PanicInfo;
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

use riscv_rt::entry;

mod app;
mod boot_history_report;
mod bootstat_report;
mod crashdump_report;
mod flash;
mod partition;
mod slot_select;
mod verify;

/// Global UART log ring — every `write_str` to [`Uart0`] tees a copy
/// here so the panic handler has a `uart_tail` to attach to the crash
/// dump. Single-hart, no IRQs at boot, so plain `static mut` access is
/// race-free.
static mut UART_RING: p4_crashdump::UartRing<256> = p4_crashdump::UartRing::new();

/// Early-boot watchdog timeout. The bootloader arms TIMG1 with this
/// budget right before jumping to the app. The app has until then to
/// either start feeding [`bootloader::wdt::feed_timg1`] periodically,
/// install its own watchdog and call [`bootloader::wdt::disable_timg1`],
/// or — for a one-shot success path — call `disable_timg1` after
/// `mark_boot_succeeded`. If the app hangs (deadlock, infinite loop,
/// panic-handler halt-spin) the WDT fires `SYS_RESET` and on the next
/// boot the otadata `boot_attempts` counter takes over to roll back
/// to the last stable slot.
///
/// 30 s is a deliberately generous default: typical app boot to
/// network-up is well under 10 s on this hardware, and DHCP / SNTP
/// lease delays can occasionally push past 15 s. Tighten once the
/// boot path is stable.
const WDT_TIMEOUT_MS: u32 = 30_000;

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
                // SAFETY: single-hart at boot, no IRQs preempting us.
                unsafe { (*addr_of_mut!(UART_RING)).push(b'\r') };
            }
            self.write_byte(b);
            unsafe { (*addr_of_mut!(UART_RING)).push(b) };
        }
        Ok(())
    }
}

/// Crashdump-aware panic handler. Captures CSRs + GP registers,
/// renders the panic message into the dump's fixed message buffer,
/// pulls the last 256 bytes of UART log from [`UART_RING`], writes
/// the encoded blob to the crashdump partition, and halts. The next
/// boot the mini-bootloader will print the dump on UART so support
/// can root-cause from a serial console (or, in a wired-up app, the
/// app forwards it to the cloud).
#[panic_handler]
fn panic(info: &PanicInfo<'_>) -> ! {
    let mut tail = [0u8; p4_crashdump::UART_TAIL_LEN];
    // SAFETY: single-hart at boot — nothing else can be borrowing the ring.
    let n = unsafe { (*addr_of!(UART_RING)).copy_chronological(&mut tail) };

    // SAFETY:
    // - Bootloader is single-hart, runs with IRQs disabled.
    // - Flash MSPI is up: the ROM brought it up to load us, and we
    //   never tear it down.
    // - CRASHDUMP_PARTITION_OFFSET is sector-aligned by the canonical CSV.
    unsafe {
        p4_crashdump::record_panic(
            info,
            crashdump_report::CRASHDUMP_PARTITION_OFFSET,
            &tail[..n],
            0, // boot_count: bootloader does not track one
            0, // timestamp_us: no clock yet
        )
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

    // Read the HP-system reset cause once, early. The latch is sticky
    // until POR or an explicit `_CLR` write — safe at any point during
    // boot. We don't auto-clear so the app can also observe the cause
    // after handoff and refine telemetry.
    let reset_cause = bootloader::reset_cause::read_hpcore0();
    let _ = writeln!(
        uart,
        "reset cause: {:?} (raw 0x{:02X})",
        reset_cause,
        reset_cause.raw()
    );

    // If the previous boot wrote a crash dump, surface it on UART now —
    // before we touch PSRAM or load the app, so a recurring crash that
    // faults during PSRAM init or app load still gets its previous-boot
    // dump printed. Does NOT erase the partition; the app erases after
    // forwarding (network upload, host pull, …).
    let crash_was_present = unsafe {
        p4_crashdump::is_present(crashdump_report::CRASHDUMP_PARTITION_OFFSET)
    };
    crashdump_report::print_if_present(&mut uart, crashdump_report::CRASHDUMP_PARTITION_OFFSET);

    // Append one entry to the boot-history ring. The bootloader has no
    // clock, so timestamp / previous-uptime stay zero — the app should
    // append a refined entry once SYSTIMER is up. Reason precedence:
    // a crashdump being present trumps the reset-cause signal because
    // it's the most specific (we already know the previous boot
    // panicked); otherwise we map the latched HP reset cause through
    // the chip-agnostic taxonomy in [`p4_boot_history::reason`].
    let boot_reason = if crash_was_present {
        p4_boot_history::reason::CRASH_RECOVERY
    } else {
        boot_history_report::map_cause_to_reason(reset_cause)
    };
    boot_history_report::record_and_print(
        &mut uart,
        boot_history_report::BOOTLOG_PARTITION_OFFSET,
        boot_reason,
    );

    // Append a bootstat record. Unlike boot-history (per-event ring),
    // bootstat carries cumulative counters (boot_count / crash_count /
    // uptime_hours) plus the last reset reason — what the service-token
    // MQTT hello forwards for fleet health. Uses the authoritative HP
    // reset cause, with crashdump-presence overriding to PanicReset
    // (the chip surfaces panic-driven resets as HpCoreSoftware, so
    // without that override we'd miscount panics as clean SW resets).
    bootstat_report::record_and_print(
        &mut uart,
        bootstat_report::BOOTSTAT_PARTITION_OFFSET,
        reset_cause,
        crash_was_present,
    );

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

    // Phase 3: pick the app partition to boot, honouring otadata A/B
    // state. On WDT-reset of a `pending` slot the boot-attempt counter
    // increments here; after MAX_BOOT_ATTEMPTS the bootloader auto-
    // rolls back to `stable`. Falls back to the `factory` partition if
    // the chosen OTA slot is empty / corrupt.
    let (chosen, _selection_outcome) = match slot_select::pick(&mut uart) {
        Ok(pair) => pair,
        Err(e) => {
            let _ = writeln!(uart, "slot_select::pick ERR {:?}", e);
            heartbeat(&mut uart);
        }
    };

    // Phase 4: load the app image segments and jump.
    let header = match app::read_header(chosen.offset) {
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
        match verify::verify_app_sha256(chosen.offset, &header) {
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
        app::load_segments(chosen.offset, &header, |i, la, dl| {
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
        "mini-bootloader: arming TIMG1 WDT ({} ms) — app must feed or call \
         bootloader::wdt::disable_timg1()",
        WDT_TIMEOUT_MS
    );
    let _ = writeln!(
        uart,
        "mini-bootloader: jumping to app @ 0x{:08X}",
        header.entry_addr
    );
    drain_uart();

    // Arm the early-boot watchdog *after* draining UART so the boot log
    // is fully flushed first — feed_timg1 is the app's responsibility
    // from this point on, and we don't want a slow UART FIFO drain
    // eating into the timeout budget. If the app hangs / panics /
    // never starts, TIMG1 fires SYS_RESET in WDT_TIMEOUT_MS and the
    // chip cold-restarts through us.
    bootloader::wdt::enable_timg1_reset(WDT_TIMEOUT_MS);

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
