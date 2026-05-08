//! UART-based recovery / re-provisioning shell.
//!
//! Entered when the recovery GPIO is held LOW at boot. The shell speaks
//! line-oriented ASCII at 115200 8N1 (whatever rate the ROM left on UART0
//! after handoff — the same rate as the boot logs). The intent is that a
//! field installer can connect any serial terminal and re-write the
//! `identity` partition without re-flashing firmware.
//!
//! Protocol (one command per line, `\n` or `\r\n` terminated):
//!
//!   info                      print current identity (token redacted)
//!   set board <id>            stage new board_id (≤ 32 bytes UTF-8)
//!   set house <id>            stage new house_id (≤ 32 bytes UTF-8)
//!   set token <token>         stage new house_token (≤ 192 bytes UTF-8)
//!   commit                    erase identity sector + write staged blob
//!   wipe                      erase identity sector (unprovisioned state)
//!   reboot                    print "power-cycle to apply" and halt
//!   help                      print this menu
//!
//! `commit` uses [`p4_cfg::encode`] so the on-flash format and CRC match
//! exactly what `tools/cfg-gen.py` produces. After a successful commit,
//! the chip should be power-cycled — recovery mode does not chain into a
//! normal boot to keep the state machine simple.

use core::fmt::{self, Write};
use core::hint::spin_loop;
use core::ptr::{read_volatile, write_volatile};

use crate::flash;

const UART0_BASE: usize = 0x500C_A000;
const UART_FIFO_REG: *mut u32 = UART0_BASE as *mut u32;
const UART_STATUS_REG: *const u32 = (UART0_BASE + 0x1C) as *const u32;
const UART_RXFIFO_CNT_MASK: u32 = 0xFF;
const UART_TXFIFO_CNT_SHIFT: u32 = 16;
const UART_TXFIFO_CNT_MASK: u32 = 0xFF << UART_TXFIFO_CNT_SHIFT;
const UART_TXFIFO_CAPACITY: u32 = 128;

/// Flash byte offset of the `identity` partition. Must match the
/// canonical CSV (see `example-partitions.csv`). Hard-coded here so
/// recovery does not depend on a working partition table parse — that
/// way a corrupt PT can't lock the installer out.
const IDENTITY_PARTITION_OFFSET: u32 = 0xD000;

const LINE_BUF_LEN: usize = 256;

struct Uart;

impl Uart {
    fn write_byte(&mut self, byte: u8) {
        let mask = UART_TXFIFO_CNT_MASK;
        let shift = UART_TXFIFO_CNT_SHIFT;
        while (unsafe { read_volatile(UART_STATUS_REG) } & mask) >> shift >= UART_TXFIFO_CAPACITY {
            spin_loop();
        }
        unsafe { write_volatile(UART_FIFO_REG, byte as u32) };
    }
}

impl Write for Uart {
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

fn read_byte_blocking() -> u8 {
    while (unsafe { read_volatile(UART_STATUS_REG) } & UART_RXFIFO_CNT_MASK) == 0 {
        spin_loop();
    }
    (unsafe { read_volatile(UART_FIFO_REG as *const u32) } & 0xFF) as u8
}

/// Read one line from UART into `buf`. Returns the populated slice. Echo
/// is on (so a serial terminal user sees what they type). `\r` and `\n`
/// both terminate; backspace (`0x08`) and DEL (`0x7F`) erase one byte.
fn read_line<'a>(buf: &'a mut [u8; LINE_BUF_LEN]) -> &'a str {
    let mut uart = Uart;
    let mut len = 0;
    loop {
        let b = read_byte_blocking();
        match b {
            b'\r' | b'\n' => {
                let _ = uart.write_str("\n");
                break;
            }
            0x08 | 0x7F => {
                if len > 0 {
                    len -= 1;
                    let _ = uart.write_str("\x08 \x08");
                }
            }
            0x20..=0x7E => {
                if len < buf.len() {
                    buf[len] = b;
                    len += 1;
                    uart.write_byte(b);
                }
            }
            _ => {
                // Ignore other control bytes (Ctrl-C, escape sequences, …).
            }
        }
    }
    // SAFETY: we only stored bytes 0x20..=0x7E above, all valid ASCII/UTF-8.
    core::str::from_utf8(&buf[..len]).unwrap_or("")
}

struct State {
    board_id: [u8; p4_cfg::BOARD_ID_LEN],
    house_id: [u8; p4_cfg::HOUSE_ID_LEN],
    house_token: [u8; p4_cfg::HOUSE_TOKEN_LEN],
    /// Did `set` modify any field since the last `commit` / `wipe`?
    dirty: bool,
    /// Did the existing partition decode cleanly when we entered?
    decoded_ok: bool,
}

impl State {
    fn from_flash() -> Self {
        let mut s = Self {
            board_id: [0; p4_cfg::BOARD_ID_LEN],
            house_id: [0; p4_cfg::HOUSE_ID_LEN],
            house_token: [0; p4_cfg::HOUSE_TOKEN_LEN],
            dirty: false,
            decoded_ok: false,
        };
        // SAFETY: single-hart at boot, flash MSPI is up post-init_phase2_full.
        if let Ok(id) = unsafe { p4_cfg::load_from_flash(IDENTITY_PARTITION_OFFSET) } {
            s.board_id = *id.board_id_raw();
            s.house_id = *id.house_id_raw();
            s.house_token = *id.house_token_raw();
            s.decoded_ok = true;
        }
        s
    }
}

fn copy_field(dst: &mut [u8], value: &str) -> Result<(), &'static str> {
    let bytes = value.as_bytes();
    if bytes.len() > dst.len() {
        return Err("value too long");
    }
    if bytes.contains(&0) {
        return Err("interior NUL not allowed");
    }
    // Zero-fill so trailing bytes from a previous longer value are gone.
    for d in dst.iter_mut() {
        *d = 0;
    }
    dst[..bytes.len()].copy_from_slice(bytes);
    Ok(())
}

fn trim_nul(buf: &[u8]) -> &str {
    let n = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
    core::str::from_utf8(&buf[..n]).unwrap_or("<non-utf8>")
}

fn cmd_info(uart: &mut Uart, st: &State) {
    let _ = writeln!(uart, "  source:    {}", if st.decoded_ok { "flash (decoded OK)" } else { "flash unreadable / unprovisioned (staging from blank)" });
    let _ = writeln!(uart, "  board_id:  \"{}\"", trim_nul(&st.board_id));
    let _ = writeln!(uart, "  house_id:  \"{}\"", trim_nul(&st.house_id));
    let token_len = trim_nul(&st.house_token).len();
    let _ = writeln!(uart, "  token_len: {} bytes (redacted)", token_len);
    let _ = writeln!(uart, "  dirty:     {}", st.dirty);
}

fn cmd_help(uart: &mut Uart) {
    let _ = writeln!(uart, "commands:");
    let _ = writeln!(uart, "  info                       show staged identity (token redacted)");
    let _ = writeln!(uart, "  set board <id>             stage new board_id (≤ 32 bytes)");
    let _ = writeln!(uart, "  set house <id>             stage new house_id (≤ 32 bytes)");
    let _ = writeln!(uart, "  set token <token>          stage new house_token (≤ 192 bytes)");
    let _ = writeln!(uart, "  commit                     erase + write identity partition");
    let _ = writeln!(uart, "  wipe                       erase identity partition");
    let _ = writeln!(uart, "  reboot                     print power-cycle hint and halt");
    let _ = writeln!(uart, "  help                       this menu");
}

fn cmd_commit(uart: &mut Uart, st: &mut State) {
    // Encode via p4_cfg so we share the exact byte format (incl. CRC32)
    // with tools/cfg-gen.py — no chance of drift.
    let board = trim_nul(&st.board_id);
    let house = trim_nul(&st.house_id);
    let token = trim_nul(&st.house_token);
    let blob = match p4_cfg::encode(board, house, token) {
        Ok(b) => b,
        Err(e) => {
            let _ = writeln!(uart, "encode failed: {}", e);
            return;
        }
    };

    let _ = writeln!(uart, "erasing sector at 0x{:05X} …", IDENTITY_PARTITION_OFFSET);
    // SAFETY: single-hart at boot, sector is identity partition only.
    if let Err(e) = unsafe { flash::erase_sector(IDENTITY_PARTITION_OFFSET) } {
        let _ = writeln!(uart, "erase failed: {:?}", e);
        return;
    }

    // ROM write requires word-aligned src + word-multiple length.
    let mut padded = [0u32; p4_cfg::BLOB_LEN_ALIGNED / 4];
    // SAFETY: padded is BLOB_LEN_ALIGNED bytes; blob is BLOB_LEN ≤ that.
    unsafe {
        core::ptr::copy_nonoverlapping(
            blob.as_ptr(),
            padded.as_mut_ptr() as *mut u8,
            p4_cfg::BLOB_LEN,
        );
    }
    // SAFETY: aligned by [u32; N], length is a multiple of 4 by construction.
    if let Err(e) = unsafe {
        flash::write(
            IDENTITY_PARTITION_OFFSET,
            padded.as_ptr() as *const u8,
            p4_cfg::BLOB_LEN_ALIGNED,
        )
    } {
        let _ = writeln!(uart, "write failed: {:?}", e);
        return;
    }

    // Read-back verification — confirms erase + write actually landed
    // before the installer power-cycles and discovers a half-write.
    // SAFETY: single-hart, identity partition is ours.
    match unsafe { p4_cfg::load_from_flash(IDENTITY_PARTITION_OFFSET) } {
        Ok(id) => {
            let _ = writeln!(uart, "commit OK — verified read-back:");
            let _ = writeln!(uart, "  board_id: \"{}\"", id.board_id());
            let _ = writeln!(uart, "  house_id: \"{}\"", id.house_id());
            let _ = writeln!(uart, "  token_len: {} bytes", id.house_token().len());
            st.dirty = false;
            st.decoded_ok = true;
        }
        Err(e) => {
            let _ = writeln!(uart, "commit wrote but verify failed: {}", e);
        }
    }
}

fn cmd_wipe(uart: &mut Uart, st: &mut State) {
    let _ = writeln!(uart, "wiping sector at 0x{:05X} …", IDENTITY_PARTITION_OFFSET);
    // SAFETY: single-hart at boot, identity partition only.
    match unsafe { flash::erase_sector(IDENTITY_PARTITION_OFFSET) } {
        Ok(()) => {
            for b in st.board_id.iter_mut() {
                *b = 0;
            }
            for b in st.house_id.iter_mut() {
                *b = 0;
            }
            for b in st.house_token.iter_mut() {
                *b = 0;
            }
            st.dirty = false;
            st.decoded_ok = false;
            let _ = writeln!(uart, "wipe OK — partition is unprovisioned");
        }
        Err(e) => {
            let _ = writeln!(uart, "wipe failed: {:?}", e);
        }
    }
}

fn cmd_set(uart: &mut Uart, st: &mut State, rest: &str) {
    // `rest` is the substring after "set ", trimmed of leading spaces.
    // Split into <field> <value...> — the value may contain spaces
    // (a JWT token sometimes has '+' / '.' / '_' but no spaces, but
    // we permit them harmlessly).
    let trimmed = rest.trim_start();
    let (field, value) = match trimmed.find(|c: char| c.is_ascii_whitespace()) {
        Some(i) => (&trimmed[..i], trimmed[i..].trim_start()),
        None => (trimmed, ""),
    };
    let res = match field {
        "board" => copy_field(&mut st.board_id, value),
        "house" => copy_field(&mut st.house_id, value),
        "token" => copy_field(&mut st.house_token, value),
        "" => Err("usage: set board|house|token <value>"),
        _ => Err("unknown field (expected: board, house, token)"),
    };
    match res {
        Ok(()) => {
            st.dirty = true;
            let _ = writeln!(uart, "  staged ({} bytes)", value.len());
        }
        Err(e) => {
            let _ = writeln!(uart, "error: {}", e);
        }
    }
}

/// Enter the recovery shell. Never returns — the only exit paths are
/// `reboot` (halt-loop, installer power-cycles) or a hard reset.
pub fn run() -> ! {
    let mut uart = Uart;
    let _ = writeln!(uart, "");
    let _ = writeln!(uart, "=== recovery mode ===");
    let _ = writeln!(uart, "type 'help' for commands. all input is line-buffered.");

    let mut st = State::from_flash();
    if !st.decoded_ok {
        let _ = writeln!(
            uart,
            "note: existing identity partition unreadable / blank — staging from empty",
        );
    }

    let mut buf = [0u8; LINE_BUF_LEN];
    loop {
        let _ = uart.write_str("recovery> ");
        let line = read_line(&mut buf);
        let cmd = line.trim();
        if cmd.is_empty() {
            continue;
        }
        // Split into first word + rest. We do this manually to keep
        // a `&str` borrow into `buf` short-lived.
        let (head, rest) = match cmd.find(|c: char| c.is_ascii_whitespace()) {
            Some(i) => (&cmd[..i], cmd[i..].trim_start()),
            None => (cmd, ""),
        };
        match head {
            "info" => cmd_info(&mut uart, &st),
            "help" | "?" => cmd_help(&mut uart),
            "set" => cmd_set(&mut uart, &mut st, rest),
            "commit" => cmd_commit(&mut uart, &mut st),
            "wipe" => cmd_wipe(&mut uart, &mut st),
            "reboot" => {
                let _ = writeln!(uart, "power-cycle the board to apply changes.");
                drain_uart();
                halt();
            }
            other => {
                let _ = writeln!(uart, "unknown command: '{}' (try 'help')", other);
            }
        }
    }
}

fn drain_uart() {
    let mask = UART_TXFIFO_CNT_MASK;
    let shift = UART_TXFIFO_CNT_SHIFT;
    while (unsafe { read_volatile(UART_STATUS_REG) } & mask) >> shift > 0 {
        spin_loop();
    }
}

fn halt() -> ! {
    loop {
        spin_loop();
    }
}
