//! Minimal GPIO read helper for the recovery-mode GPIO check.
//!
//! Programs `IO_MUX_GPIOn` to: function = GPIO matrix, input enable,
//! internal pull-up, then samples `GPIO_IN_REG` (or `GPIO_IN1_REG`).
//! No output / matrix routing — we only need the synchronised input
//! level.
//!
//! Register addresses are from IDF v5.3 `soc/esp32p4/include/soc/`:
//!   * `io_mux_reg.h` — `IO_MUX_GPIOn_REG = 0x500E_1004 + 4*N`
//!   * `gpio_reg.h`   — `GPIO_IN_REG = 0x500E_2044`,
//!                       `GPIO_IN1_REG = 0x500E_2048`
//!
//! ESP32-P4 has 57 GPIOs (0..=56). Pins 0..31 read from `GPIO_IN_REG`,
//! 32..56 from `GPIO_IN1_REG`.

#![allow(dead_code)]

use core::ptr::{read_volatile, write_volatile};

const IO_MUX_GPIO0_REG: usize = 0x500E_1004;
const GPIO_IN_REG: usize = 0x500E_2044;
const GPIO_IN1_REG: usize = 0x500E_2048;
const GPIO_ENABLE_REG: usize = 0x500E_2020;
const GPIO_ENABLE1_REG: usize = 0x500E_2024;

const FUN_WPD: u32 = 1 << 7;
const FUN_WPU: u32 = 1 << 8;
const FUN_IE: u32 = 1 << 9;
const MCU_SEL_SHIFT: u32 = 12;
const MCU_SEL_MASK: u32 = 0x7 << MCU_SEL_SHIFT;
const MCU_SEL_GPIO: u32 = 1; // GPIO matrix function

#[derive(Clone, Copy, Debug)]
pub enum Pull {
    Up,
    Down,
    None,
}

/// Configure `gpio_num` (0..=56) as a digital input with the requested
/// internal pull. Idempotent.
pub fn configure_input(gpio_num: u8, pull: Pull) {
    let reg = (IO_MUX_GPIO0_REG + 4 * gpio_num as usize) as *mut u32;

    // Disable matrix-driven output for this pin (we only read).
    if gpio_num < 32 {
        let bit = 1u32 << gpio_num;
        unsafe {
            let v = read_volatile(GPIO_ENABLE_REG as *const u32) & !bit;
            write_volatile(GPIO_ENABLE_REG as *mut u32, v);
        }
    } else {
        let bit = 1u32 << (gpio_num - 32);
        unsafe {
            let v = read_volatile(GPIO_ENABLE1_REG as *const u32) & !bit;
            write_volatile(GPIO_ENABLE1_REG as *mut u32, v);
        }
    }

    // Program IO_MUX: clear pulls + drv/IE + MCU_SEL, then set what we want.
    unsafe {
        let mut v = read_volatile(reg);
        v &= !(FUN_WPU | FUN_WPD | FUN_IE | MCU_SEL_MASK);
        v |= FUN_IE; // input enable
        v |= match pull {
            Pull::Up => FUN_WPU,
            Pull::Down => FUN_WPD,
            Pull::None => 0,
        };
        v |= MCU_SEL_GPIO << MCU_SEL_SHIFT;
        write_volatile(reg, v);
    }
}

/// Read the synchronised input level of `gpio_num`. Returns `true`
/// when the pad reads high.
#[inline]
pub fn read(gpio_num: u8) -> bool {
    if gpio_num < 32 {
        let v = unsafe { read_volatile(GPIO_IN_REG as *const u32) };
        (v & (1 << gpio_num)) != 0
    } else {
        let v = unsafe { read_volatile(GPIO_IN1_REG as *const u32) };
        (v & (1 << (gpio_num - 32))) != 0
    }
}
