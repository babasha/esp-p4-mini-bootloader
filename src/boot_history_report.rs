//! Boot-history append + summary print.
//!
//! Called once early in boot. Reads the existing ring from flash,
//! appends a fresh entry (the bootloader has no clock, so timestamp /
//! previous-uptime stay zero — the app is expected to append a refined
//! second entry once SYSTIMER and the PMU reset-cause are readable),
//! writes the ring back, and prints a one-line summary plus the last
//! few entries on UART.

use core::fmt::Write;

use bootloader::reset_cause::Cause;
use p4_boot_history::{reason, Entry, FlashError, History};

/// Map an HP-system reset-cause variant to the canonical
/// [`p4_boot_history::reason`] tag. Unknown causes fall back to
/// [`reason::UNKNOWN`] — the raw 6-bit value is still recoverable via
/// `Cause::Unknown(_).raw()` if richer telemetry is needed.
pub fn map_cause_to_reason(cause: Cause) -> u32 {
    match cause {
        Cause::PowerOn => reason::POWER_ON,
        Cause::UsbUartChip
        | Cause::UsbJtagChip
        | Cause::HpJtag => reason::EXTERNAL_RESET,
        Cause::DigitalSystemSoftware
        | Cause::HpCoreSoftware
        | Cause::PmuHpSystemPowerDown
        | Cause::HpCoreLockup => reason::SOFTWARE_RESET,
        Cause::HpWatchdogSystem
        | Cause::LpWatchdogSystem
        | Cause::HpWatchdogCore
        | Cause::LpWatchdogCore
        | Cause::LpWatchdogChip
        | Cause::SuperWatchdog => reason::WATCHDOG_RESET,
        Cause::BrownOut => reason::BROWNOUT_RESET,
        // Glitch (0x13) and eFuse CRC (0x14) are hardware-integrity
        // events — collapsed onto INTRUSION here to match
        // `esp-p4-bootstat::ResetReason::Intrusion` so cross-blob fleet
        // queries see the same signal.
        Cause::Glitch | Cause::EfuseCrc => reason::INTRUSION,
        Cause::Unknown(_) => reason::UNKNOWN,
    }
}

/// Flash byte offset of the `bootlog` partition (must match the
/// canonical CSV — see `example-partitions.csv`).
pub const BOOTLOG_PARTITION_OFFSET: u32 = 0x12000;

/// Number of recent entries to print on UART. Kept small so a noisy
/// device with 64 events doesn't flood the console.
const PRINT_RECENT_N: usize = 5;

/// Read history, append one entry for this boot, write back, and
/// summarise on `uart`. Quiet on the no-flash-error path (one summary
/// line). Failures are reported but not fatal — the bootloader still
/// continues to load the app.
///
/// `boot_reason` is what to record for *this* boot. Pass
/// [`reason::CRASH_RECOVERY`] when the crashdump partition magic was
/// valid on entry (so we know the previous boot panicked); otherwise
/// [`reason::UNKNOWN`] is the right default — the app will append a
/// refined second entry once it has read the PMU reset-cause register.
pub fn record_and_print<W: Write>(uart: &mut W, partition_offset: u32, boot_reason: u32) {
    // SAFETY: single-hart at boot, post-init_phase2_full so flash
    // MSPI is up.
    let mut history = unsafe { p4_boot_history::load_from_flash_or_empty(partition_offset) };

    let entry = Entry::new(0, 0, boot_reason);
    history.append(entry);

    // SAFETY: same conditions as the read above; partition offset is
    // sector-aligned per the canonical CSV.
    match unsafe { p4_boot_history::store_to_flash(partition_offset, &history) } {
        Ok(()) => {
            let _ = writeln!(
                uart,
                "boot-history: appended seq #{} (now {} entries)",
                history.iter_recent().next().map(|e| e.seq).unwrap_or(0),
                history.len(),
            );
        }
        Err(FlashError::Rom(s)) => {
            let _ = writeln!(uart, "boot-history: flash write failed (rom={})", s);
            return;
        }
        Err(e) => {
            let _ = writeln!(uart, "boot-history: write failed: {}", e);
            return;
        }
    }

    print_recent(uart, &history);
}

fn print_recent<W: Write>(uart: &mut W, history: &History) {
    let total = history.len();
    let take = core::cmp::min(total, PRINT_RECENT_N);
    if take == 0 {
        return;
    }
    let _ = writeln!(uart, "boot-history: last {} of {} entries:", take, total);
    for entry in history.iter_recent().take(take) {
        let _ = writeln!(
            uart,
            "  seq={:5}  ts_us={:>16}  prev_uptime_us={:>13}  reason={} ({})",
            entry.seq,
            entry.timestamp_us,
            entry.previous_uptime_us,
            entry.reason,
            reason_name(entry.reason),
        );
    }
}

fn reason_name(r: u32) -> &'static str {
    match r {
        reason::UNKNOWN => "unknown",
        reason::POWER_ON => "power-on",
        reason::EXTERNAL_RESET => "external",
        reason::SOFTWARE_RESET => "software",
        reason::WATCHDOG_RESET => "watchdog",
        reason::BROWNOUT_RESET => "brownout",
        reason::DEEP_SLEEP_WAKE => "deep-sleep-wake",
        reason::CRASH_RECOVERY => "crash-recovery",
        reason::INTRUSION => "intrusion",
        _ => "?",
    }
}
