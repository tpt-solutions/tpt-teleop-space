//! Simulated board bring-up (Phase 16: COTS dev board bring-up, simulated).
//!
//! A model stands in for a dev board: SRAM that can carry stuck-at faults,
//! a watchdog kick register, and NVRAM that persists across a power cycle.
//! The checks run the shipped code against that model: a march test over the
//! SRAM, the watchdog feed loop, and a snapshot restored after a simulated
//! reset. A real board needs the same checks run on the part.

use std::cell::Cell;

use tpt_t_space_core::snapshot::SnapshotStore;
use tpt_t_space_edge::reg::{Access, RegCell};
use tpt_t_space_edge::wdt::{KICK_MAGIC, Watchdog, WatchdogConfig};
use tpt_t_space_sim::checkpoint::Context;

use crate::watchdog::{self, Scenario};

/// Simulated SRAM: stored bits, plus stuck-at masks that override reads.
#[derive(Debug, Clone)]
pub struct SimSram {
    cells: Vec<u8>,
    stuck0: Vec<u8>,
    stuck1: Vec<u8>,
}

impl SimSram {
    /// Zeroed SRAM of `len` bytes, with no faults.
    #[must_use]
    pub fn new(len: usize) -> Self {
        Self {
            cells: vec![0; len],
            stuck0: vec![0; len],
            stuck1: vec![0; len],
        }
    }

    /// Bytes in the array.
    #[must_use]
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// Whether the array is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    /// Stick bit `bit` of byte `addr` at `value`, whatever is written.
    pub fn stick(&mut self, addr: usize, bit: u8, value: bool) {
        let mask = 1u8 << (bit % 8);
        if value {
            self.stuck1[addr] |= mask;
        } else {
            self.stuck0[addr] |= mask;
        }
    }

    fn write(&mut self, addr: usize, value: u8) {
        self.cells[addr] = value;
    }

    fn read(&self, addr: usize) -> u8 {
        (self.cells[addr] & !self.stuck0[addr]) | self.stuck1[addr]
    }
}

/// The first wrong read a march test found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fault {
    /// Byte address.
    pub addr: usize,
    /// Value the pattern expected.
    pub expected: u8,
    /// Value the array returned.
    pub got: u8,
}

fn expect(ram: &SimSram, addr: usize, want: u8) -> Result<(), Fault> {
    let got = ram.read(addr);
    if got == want {
        Ok(())
    } else {
        Err(Fault {
            addr,
            expected: want,
            got,
        })
    }
}

/// March C- over `ram`: catches every stuck-at fault in every byte.
///
/// # Errors
/// The first [`Fault`] the march reads back.
pub fn march(ram: &mut SimSram) -> Result<(), Fault> {
    let n = ram.len();
    for a in 0..n {
        ram.write(a, 0x00);
    }
    for a in 0..n {
        expect(ram, a, 0x00)?;
        ram.write(a, 0xFF);
    }
    for a in 0..n {
        expect(ram, a, 0xFF)?;
        ram.write(a, 0x00);
    }
    for a in (0..n).rev() {
        expect(ram, a, 0x00)?;
        ram.write(a, 0xFF);
    }
    for a in (0..n).rev() {
        expect(ram, a, 0xFF)?;
        ram.write(a, 0x00);
    }
    for a in 0..n {
        expect(ram, a, 0x00)?;
    }
    Ok(())
}

/// One bring-up check and what it showed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Check {
    /// What was checked.
    pub name: &'static str,
    /// Whether it passed.
    pub pass: bool,
    /// What the check observed, for the report.
    pub evidence: String,
}

/// Run the bring-up checks on a simulated board.
///
/// `stuck` lists SRAM faults to inject as `(byte, bit, value)`.
/// `nvram_flip` corrupts one NVRAM byte between save and restore, as a
/// failing part would.
#[must_use]
pub fn bring_up(stuck: &[(usize, u8, bool)], nvram_flip: Option<usize>) -> Vec<Check> {
    vec![
        kick_register_check(),
        sram_check(stuck),
        nvram_check(nvram_flip),
        feed_loop_check(),
    ]
}

fn kick_register_check() -> Check {
    let kick = Cell::new(0u32);
    let _wdt = Watchdog::arm(
        WatchdogConfig::loop_1hz(),
        RegCell::mirror(&kick, Access::ReadWrite),
    );
    let read = kick.get();
    Check {
        name: "watchdog kick register",
        pass: read == KICK_MAGIC,
        evidence: format!("wrote {KICK_MAGIC:#010x} on arm, reads back {read:#010x}"),
    }
}

fn sram_check(stuck: &[(usize, u8, bool)]) -> Check {
    const SRAM_BYTES: usize = 4096;
    let mut ram = SimSram::new(SRAM_BYTES);
    for &(addr, bit, value) in stuck {
        if addr < SRAM_BYTES {
            ram.stick(addr, bit, value);
        }
    }
    match march(&mut ram) {
        Ok(()) => Check {
            name: "SRAM march test",
            pass: true,
            evidence: format!("{SRAM_BYTES} bytes, March C-, no mismatch"),
        },
        Err(f) => Check {
            name: "SRAM march test",
            pass: false,
            evidence: format!(
                "byte {:#06x} read {:#04x}, expected {:#04x}",
                f.addr, f.got, f.expected
            ),
        },
    }
}

fn nvram_check(flip: Option<usize>) -> Check {
    let ctx = Context {
        t_s: 42,
        mission_state: 2,
        safe_mode: false,
        bundles_delivered: 3,
    };
    let mut nvram = vec![0u8; 128];
    let mut store = SnapshotStore::new(nvram.len(), 64);
    if store.save(&mut nvram, &ctx).is_err() {
        return Check {
            name: "NVRAM snapshot across reset",
            pass: false,
            evidence: "snapshot did not save".to_owned(),
        };
    }
    // Power cycle: volatile state is gone, NVRAM is kept (and may be damaged).
    if let Some(at) = flip {
        if let Some(byte) = nvram.get_mut(at) {
            *byte ^= 0xFF;
        }
    }
    let restored = SnapshotStore::new(nvram.len(), 64).restore::<Context>(&nvram);
    Check {
        name: "NVRAM snapshot across reset",
        pass: restored == Some(ctx),
        evidence: match restored {
            Some(_) => "restored the saved context after the power cycle".to_owned(),
            None => "snapshot rejected after the power cycle (CRC or header)".to_owned(),
        },
    }
}

fn feed_loop_check() -> Check {
    let r = watchdog::run(Scenario::Nominal { jitter_ms: 200 }, 60_000, 7);
    Check {
        name: "watchdog feed loop, 60 s",
        pass: r.bites == 0 && r.too_early == 0,
        evidence: format!(
            "{} feeds accepted, {} too early, {} bites",
            r.feeds, r.too_early, r.bites
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_clean_board_passes_every_check() {
        let checks = bring_up(&[], None);
        assert!(checks.iter().all(|c| c.pass), "{checks:?}");
    }

    #[test]
    fn march_finds_a_stuck_at_zero_and_a_stuck_at_one() {
        let mut ram = SimSram::new(64);
        ram.stick(10, 3, false);
        let f = march(&mut ram).expect_err("stuck-at-0 caught");
        assert_eq!(f.addr, 10);
        let mut ram = SimSram::new(64);
        ram.stick(20, 7, true);
        let f = march(&mut ram).expect_err("stuck-at-1 caught");
        assert_eq!(f.addr, 20);
    }

    #[test]
    fn a_stuck_bit_fails_the_sram_check_and_names_the_byte() {
        let checks = bring_up(&[(0x123, 5, true)], None);
        let sram = checks
            .iter()
            .find(|c| c.name == "SRAM march test")
            .expect("sram check");
        assert!(!sram.pass);
        assert!(sram.evidence.contains("0x0123"), "{}", sram.evidence);
    }

    #[test]
    fn a_damaged_nvram_image_is_rejected_after_the_reset() {
        let checks = bring_up(&[], Some(30));
        let nvram = checks
            .iter()
            .find(|c| c.name == "NVRAM snapshot across reset")
            .expect("nvram check");
        assert!(!nvram.pass, "{nvram:?}");
    }
}
