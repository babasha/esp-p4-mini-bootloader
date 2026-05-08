//! Boot-time crashdump reporter.
//!
//! On every boot we do a magic-only quick check at the start of the
//! crashdump partition. If the magic matches we decode + print the full
//! dump on UART for support to capture. We deliberately do **not**
//! erase the partition here — that's the app's job, after it has
//! successfully forwarded the dump (network upload / second-channel
//! retrieval). Otherwise a transient upload failure would lose the
//! crash data permanently.

use core::fmt::{self, Write};

use p4_crashdump::{reason, CrashDump};

/// Flash byte offset of the `crashdump` partition (must match the
/// canonical CSV — see `example-partitions.csv`).
pub const CRASHDUMP_PARTITION_OFFSET: u32 = 0x10000;

/// If a crashdump is present at `partition_offset`, decode and print
/// it on `uart`. Quiet on the no-crash path.
pub fn print_if_present<W: Write>(uart: &mut W, partition_offset: u32) {
    // SAFETY: single-hart at boot, flash MSPI is up post-init_phase2_full.
    if !unsafe { p4_crashdump::is_present(partition_offset) } {
        return;
    }
    // SAFETY: same conditions as is_present.
    match unsafe { p4_crashdump::load_from_flash(partition_offset) } {
        Ok(dump) => print_dump(uart, &dump, partition_offset),
        Err(e) => {
            let _ = writeln!(
                uart,
                "crashdump: magic OK at 0x{:05X} but decode failed: {}",
                partition_offset, e
            );
        }
    }
}

fn print_dump<W: Write>(uart: &mut W, dump: &CrashDump, partition_offset: u32) {
    let _ = writeln!(uart, "");
    let _ = writeln!(uart, "=== crashdump from previous boot ===");
    let _ = writeln!(uart, "  partition    : 0x{:05X}", partition_offset);
    let _ = writeln!(uart, "  timestamp_us : {}", dump.timestamp_us);
    let _ = writeln!(
        uart,
        "  reason_tag   : {} ({})",
        dump.reason_tag,
        reason_name(dump.reason_tag)
    );
    let _ = writeln!(uart, "  boot_count   : {}", dump.boot_count);
    let _ = writeln!(
        uart,
        "  mcause       : 0x{:08X} ({})",
        dump.mcause,
        McauseDecode(dump.mcause)
    );
    let _ = writeln!(uart, "  mepc         : 0x{:08X}", dump.mepc);
    let _ = writeln!(uart, "  mtval        : 0x{:08X}", dump.mtval);
    let _ = writeln!(uart, "  mstatus      : 0x{:08X}", dump.mstatus);

    let msg = dump.message_str();
    if !msg.is_empty() {
        let _ = writeln!(uart, "  message      : \"{}\"", msg);
    }

    let _ = writeln!(uart, "  registers (x1..x31):");
    print_registers(uart, &dump.registers);

    let used = dump.uart_tail_used();
    let _ = writeln!(uart, "  uart tail    : {} bytes", used.len());
    if !used.is_empty() {
        let _ = writeln!(uart, "  ----- BEGIN UART TAIL -----");
        print_escaped_bytes(uart, used);
        let _ = writeln!(uart, "");
        let _ = writeln!(uart, "  ----- END UART TAIL -----");
    }
    let _ = writeln!(uart, "=== end crashdump ===");
    let _ = writeln!(
        uart,
        "note: app should forward the dump and call p4_crashdump::erase(0x{:05X}) when done",
        partition_offset
    );
}

fn print_registers<W: Write>(uart: &mut W, regs: &[u32; 31]) {
    // Four registers per line: " x1=0x.. x2=0x.. x3=0x.. x4=0x.."
    for chunk_start in (0..31).step_by(4) {
        let _ = write!(uart, "   ");
        for i in chunk_start..core::cmp::min(chunk_start + 4, 31) {
            // `regs[i]` is `x{i+1}` (we skip x0).
            let _ = write!(uart, " x{:<2}=0x{:08X}", i + 1, regs[i]);
        }
        let _ = writeln!(uart, "");
    }
}

fn print_escaped_bytes<W: Write>(uart: &mut W, bytes: &[u8]) {
    for &b in bytes {
        let res: fmt::Result = match b {
            b'\n' => writeln!(uart, ""),
            b'\r' => Ok(()),
            b'\t' => uart.write_str("    "),
            0x20..=0x7E => uart.write_char(b as char),
            _ => write!(uart, "\\x{:02X}", b),
        };
        let _ = res;
    }
}

fn reason_name(tag: u32) -> &'static str {
    match tag {
        reason::PANIC => "panic",
        reason::TRAP => "trap",
        reason::WATCHDOG => "watchdog",
        reason::BROWNOUT => "brownout",
        reason::OTHER => "other",
        _ => "unknown",
    }
}

/// Adapter that turns an `mcause` value into a human-readable hint.
/// Display string is short on purpose — full decoding requires the chip-
/// specific PLIC/CLIC source table which we don't ship here.
struct McauseDecode(u32);

impl fmt::Display for McauseDecode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let v = self.0;
        // bit 31: 1 = interrupt, 0 = exception.
        let is_interrupt = (v & 0x8000_0000) != 0;
        let code = v & 0x7FFF_FFFF;
        if is_interrupt {
            // Standard machine-mode interrupt codes (RISC-V Privileged
            // Architecture v1.10). External / non-standard sources land
            // in code ≥ 16 — we don't try to name them.
            let name = match code {
                3 => "machine software intr",
                7 => "machine timer intr",
                11 => "machine external intr",
                _ => "platform/clic intr",
            };
            write!(f, "interrupt: {}", name)
        } else {
            // Standard exception codes (table 3.6 of priv spec).
            let name = match code {
                0 => "instr addr misaligned",
                1 => "instr access fault",
                2 => "illegal instruction",
                3 => "breakpoint",
                4 => "load addr misaligned",
                5 => "load access fault",
                6 => "store/AMO addr misaligned",
                7 => "store/AMO access fault",
                8 => "ecall from U-mode",
                9 => "ecall from S-mode",
                11 => "ecall from M-mode",
                12 => "instr page fault",
                13 => "load page fault",
                15 => "store/AMO page fault",
                _ => "exception",
            };
            write!(f, "exception: {}", name)
        }
    }
}
