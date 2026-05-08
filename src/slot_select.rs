//! Pick the app partition to boot, honouring the otadata A/B-OTA state.
//!
//! Flow:
//!
//! 1. Read otadata @ [`OTADATA_PARTITION_OFFSET`]; on any decode error
//!    treat it as [`p4_ota::OtaData::fresh`] (boot slot A, no rollback).
//! 2. Run [`p4_ota::select_slot`] with [`MAX_BOOT_ATTEMPTS`] to compute
//!    the [`p4_ota::BootAction`]. Persist the new otadata if the
//!    decision changed it (Pending counter bump or Rollback).
//! 3. Look up the chosen slot's partition by label (`ota_0` / `ota_1`).
//! 4. Read the candidate's app header; if the magic is wrong (slot
//!    blank or corrupt) fall back to a `factory` partition lookup.
//!    This is the rescue path for fresh devices that haven't done an
//!    OTA push yet — they ship with the app in `factory` only.
//! 5. If no factory either, return [`SlotSelectError::NoBootableApp`]
//!    so main can heartbeat instead of jumping into garbage.
//!
//! The boot-attempt counter is incremented **before** control jumps to
//! the app, so a hard hang (caught by the TIMG1 WDT we arm just before
//! jump) is guaranteed to land in the counter on the next boot. After
//! [`MAX_BOOT_ATTEMPTS`] failures the bootloader auto-rolls back to
//! `stable`, which in steady state is the slot that last ran
//! `mark_succeeded` at the app level — i.e. the previous-known-good
//! firmware.

use core::fmt::Write;

use p4_ota::{BootAction, OtaData, Slot, MAX_ATTEMPTS};

use crate::{app, partition};

/// Flash offset of the `otadata` partition (must match the canonical
/// CSV — see `example-partitions.csv`).
pub const OTADATA_PARTITION_OFFSET: u32 = 0xF000;

/// Boot-attempt limit before automatic rollback. Three matches the IDF
/// default and gives the WDT three chances to recover from a transient
/// hang before we declare the slot bad.
pub const MAX_BOOT_ATTEMPTS: u8 = MAX_ATTEMPTS;

/// What `pick` decided on top of the otadata flow — useful for
/// telemetry / boot-history annotation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SelectionOutcome {
    /// Booted the requested OTA slot; otadata unchanged.
    OtaStable(Slot),
    /// OTA slot was on probation, attempt counter incremented.
    OtaPending { slot: Slot, attempt: u8 },
    /// OTA slot used up its attempts, rolled back to `stable` slot.
    OtaRollback { from: Slot, to: Slot },
    /// OTA slot lookup or header check failed; fell back to factory.
    FactoryFallback { skipped: Slot },
    /// otadata was unreadable / blank; came up on `OtaData::fresh()`.
    /// Distinct from `OtaStable(A)` so logs can show the cold-init.
    OtaFresh,
}

#[derive(Debug, Clone, Copy)]
#[allow(dead_code)] // `Partition`'s inner is only read via #[derive(Debug)].
pub enum SlotSelectError {
    /// Neither the active OTA slot nor a factory partition has a
    /// loadable app image.
    NoBootableApp,
    /// Other fatal partition-table read failures — the underlying
    /// partition crate ran out of options.
    Partition(partition::PartitionError),
}

impl From<partition::PartitionError> for SlotSelectError {
    fn from(e: partition::PartitionError) -> Self {
        Self::Partition(e)
    }
}

/// Decide which partition to boot and persist any otadata state change.
/// Returns the `(partition, outcome)` pair so the caller can both load
/// segments from `partition` and record `outcome` in boot-history.
pub fn pick<W: Write>(
    uart: &mut W,
) -> Result<(partition::PartitionEntry, SelectionOutcome), SlotSelectError> {
    // Step 1: read otadata. Track whether decode succeeded so the
    // `OtaFresh` outcome distinguishes a blank partition from a
    // genuine "boot slot A, no pending" steady state.
    let (state, decoded_ok) = match unsafe { p4_ota::load_from_flash(OTADATA_PARTITION_OFFSET) } {
        Ok(s) => (s, true),
        Err(e) => {
            let _ = writeln!(uart, "otadata: decode failed ({}) — using fresh", e);
            (OtaData::fresh(), false)
        }
    };

    // Step 2: run the policy.
    let action = p4_ota::select_slot(&state, MAX_BOOT_ATTEMPTS);
    let (chosen_slot, outcome, persist) = match action {
        BootAction::Stable(slot) => {
            let outcome = if decoded_ok {
                SelectionOutcome::OtaStable(slot)
            } else {
                SelectionOutcome::OtaFresh
            };
            (slot, outcome, None)
        }
        BootAction::Pending {
            slot,
            attempt,
            new_state,
        } => {
            let _ = writeln!(
                uart,
                "otadata: slot {} pending, attempt {}/{}",
                slot, attempt, MAX_BOOT_ATTEMPTS
            );
            (
                slot,
                SelectionOutcome::OtaPending { slot, attempt },
                Some(new_state),
            )
        }
        BootAction::Rollback {
            from,
            to,
            new_state,
        } => {
            let _ = writeln!(
                uart,
                "otadata: slot {} exceeded {} attempts — rolling back to {}",
                from, MAX_BOOT_ATTEMPTS, to
            );
            (
                to,
                SelectionOutcome::OtaRollback { from, to },
                Some(new_state),
            )
        }
    };

    if let Some(new_state) = persist {
        // SAFETY: single-hart at boot, post-init_phase2_full so flash
        // MSPI is up; partition offset is sector-aligned per the CSV.
        match unsafe { p4_ota::store_to_flash(OTADATA_PARTITION_OFFSET, &new_state) } {
            Ok(()) => {}
            Err(e) => {
                // We failed to persist the increment / rollback. Boot
                // the chosen slot anyway — the alternative is to halt
                // and rolling back manually via UART is worse — but
                // log loudly so it's visible.
                let _ = writeln!(uart, "otadata: store_to_flash FAILED ({}) — booting anyway", e);
            }
        }
    }

    // Step 3: look up the chosen slot's partition.
    let label = chosen_slot.label();
    match partition::find_by_label(label) {
        Ok(part) => {
            // Step 4: validate the app image header.
            match app::read_header(part.offset) {
                Ok(_) => {
                    let _ = writeln!(
                        uart,
                        "boot slot: {} @ 0x{:X} ({} KB)",
                        label,
                        part.offset,
                        part.size / 1024
                    );
                    Ok((part, outcome))
                }
                Err(e) => {
                    let _ = writeln!(
                        uart,
                        "slot {} header invalid ({:?}) — falling back to factory",
                        label, e
                    );
                    fall_back_to_factory(uart, chosen_slot)
                }
            }
        }
        Err(e) => {
            let _ = writeln!(
                uart,
                "slot {} lookup failed ({:?}) — falling back to factory",
                label, e
            );
            fall_back_to_factory(uart, chosen_slot)
        }
    }
}

fn fall_back_to_factory<W: Write>(
    uart: &mut W,
    skipped: Slot,
) -> Result<(partition::PartitionEntry, SelectionOutcome), SlotSelectError> {
    match partition::find_factory_app() {
        Ok(part) => match app::read_header(part.offset) {
            Ok(_) => {
                let _ = writeln!(
                    uart,
                    "factory app: label={:?} offset=0x{:X} ({} KB) — booting as fallback",
                    part.label_str(),
                    part.offset,
                    part.size / 1024
                );
                Ok((part, SelectionOutcome::FactoryFallback { skipped }))
            }
            Err(e) => {
                let _ = writeln!(uart, "factory app header invalid: {:?}", e);
                Err(SlotSelectError::NoBootableApp)
            }
        },
        Err(_) => Err(SlotSelectError::NoBootableApp),
    }
}
