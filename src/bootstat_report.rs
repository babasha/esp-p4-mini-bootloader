//! Boot-status append + summary print.
//!
//! Called once early in boot, right after the boot-history append.
//! Reads the latest bootstat record (or fresh if blank), bumps
//! `boot_count`, records the reset reason — derived from the
//! authoritative HP reset-cause register (already read in main) and,
//! when applicable, refined by crashdump-presence to PanicReset —
//! appends a fresh record, and prints a one-line summary on UART.
//!
//! The bootloader has no clock, so the previous run's `last_uptime_seconds`
//! (set by the app's hourly tick) is carried forward as-is — that's the
//! best "the run lasted at least N seconds" approximation we can make
//! without an authoritative clock.

use core::fmt::Write;

use bootloader::reset_cause::Cause;
use p4_bootstat::{FlashError, ResetReason};

/// Flash byte offset of the `bootstat` partition (must match the
/// canonical CSV — see `example-partitions.csv`).
pub const BOOTSTAT_PARTITION_OFFSET: u32 = 0x13000;

/// Map the HP reset-cause register's typed variant to the normalized
/// [`ResetReason`] used in bootstat records. Same chip table as
/// `boot_history_report::map_cause_to_reason`, but the bootstat
/// taxonomy is finer-grained (separate brownout / watchdog /
/// intrusion buckets) so the mapping isn't quite the same.
fn map_cause_to_reason(cause: Cause) -> ResetReason {
    match cause {
        Cause::PowerOn => ResetReason::PowerOn,
        // Host-driven chip resets — clean external resets, not a crash.
        Cause::UsbUartChip | Cause::UsbJtagChip => ResetReason::ExternalReset,
        Cause::HpJtag => ResetReason::JtagReset,
        // Deliberate software resets.
        Cause::DigitalSystemSoftware | Cause::HpCoreSoftware => ResetReason::SoftwareReset,
        // PMU-driven wake from a power-down — closest match in the
        // taxonomy is DeepSleepWake; not a crash.
        Cause::PmuHpSystemPowerDown => ResetReason::DeepSleepWake,
        // Every watchdog flavour collapses to the single WatchdogTimer
        // bucket; the raw byte preserves the subtype for whoever cares.
        // CPU lockup (hardware-detected wedge) is semantically a
        // watchdog reset for crash-counting purposes.
        Cause::HpWatchdogSystem
        | Cause::LpWatchdogSystem
        | Cause::HpWatchdogCore
        | Cause::LpWatchdogCore
        | Cause::LpWatchdogChip
        | Cause::SuperWatchdog
        | Cause::HpCoreLockup => ResetReason::WatchdogTimer,
        Cause::BrownOut => ResetReason::Brownout,
        // Glitch detector and eFuse CRC are hardware-integrity events;
        // surface them on the security-flavoured Intrusion variant.
        Cause::Glitch | Cause::EfuseCrc => ResetReason::Intrusion,
        Cause::Unknown(_) => ResetReason::Unknown,
    }
}

/// Read the latest record, append a new one for this boot, and
/// summarise on `uart`. Quiet on success (one line); failures are
/// reported but not fatal — diagnostics is best-effort, never load-
/// bearing for boot.
///
/// `cause` comes from the already-read HP reset-cause register (see
/// `bootloader::reset_cause::read_hpcore0`).
///
/// `crashdump_was_present` overrides the cause-based mapping with
/// `PanicReset` when set: a valid crashdump magic on flash means the
/// previous boot definitely panicked, regardless of how the chip
/// reports the reset (panic handlers typically trigger a soft-reset,
/// which surfaces as `HpCoreSoftware` — without this override we'd
/// undercount panics).
pub fn record_and_print<W: Write>(
    uart: &mut W,
    partition_offset: u32,
    cause: Cause,
    crashdump_was_present: bool,
) {
    // SAFETY: single-hart at boot, post-init_phase2_full so flash
    // MSPI is up.
    let prev = unsafe { p4_bootstat::load_latest_or_fresh(partition_offset) };

    let reason = if crashdump_was_present {
        ResetReason::PanicReset
    } else {
        map_cause_to_reason(cause)
    };
    let raw = cause.raw();

    let next = prev.record_boot(reason, raw, prev.last_uptime_seconds);

    // SAFETY: same conditions as the read above; partition offset is
    // sector-aligned per the canonical CSV.
    match unsafe { p4_bootstat::append_record(partition_offset, &next) } {
        Ok(()) => {
            let _ = writeln!(
                uart,
                "bootstat: boot #{} reason={} (raw 0x{:02X}) crashes={} uptime={}h last_run={}s",
                next.boot_count,
                next.last_reason,
                next.last_reason_raw,
                next.crash_count,
                next.uptime_hours,
                next.last_uptime_seconds,
            );
            if next.last_panic_pc != 0 {
                let _ = writeln!(uart, "bootstat: last panic PC = 0x{:08X}", next.last_panic_pc);
            }
        }
        Err(FlashError::Rom(s)) => {
            let _ = writeln!(uart, "bootstat: flash write failed (rom={})", s);
        }
        Err(e) => {
            let _ = writeln!(uart, "bootstat: write failed: {}", e);
        }
    }
}
