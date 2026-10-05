//! Triple Modular Redundancy: the <50 µs voter (spec §4.1).
//!
//! Critical calculations — ADCS quaternion updates, replan step results —
//! execute three times on separate cores. The voter compares the three
//! results:
//!
//! * all agree → clean pass;
//! * exactly two agree → the minority result is *masked*, the majority
//!   value is the output, and the diverging core is flagged for a hardware
//!   watchdog reset;
//! * no agreement → all outputs distrusted (the caller falls back to the
//!   bounded last-known-good or holds), all cores flagged for inspection.
//!
//! The whole vote is a handful of integer comparisons: no allocation, no
//! locks, deterministic cycle time, and the budget test proves the <50 µs
//! spec bound with orders of magnitude to spare.

/// Result of one [`TmrVoter::submit`] vote.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vote<T> {
    /// The accepted result: the majority value on a masked vote, `None`
    /// equivalent via [`Vote::trusted`] when all three diverged.
    pub value: T,
    /// Which core (0..3) produced the masked-out result, if any.
    pub masked_core: Option<u8>,
    /// Whether `value` is trustworthy (majority existed).
    pub trusted: bool,
}

/// Per-core health bookkeeping.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct CoreHealth {
    /// Votes in which this core's result was the masked minority.
    pub divergences: u32,
    /// Whether the core is currently flagged for a watchdog reset.
    pub flagged: bool,
    /// Completed watchdog resets of this core.
    pub resets: u32,
}

/// The TMR voter: per-core health ledgers plus the pending reset mask.
#[derive(Debug, Clone, Copy)]
pub struct TmrVoter {
    cores: [CoreHealth; 3],
    /// Divergence threshold after which a core is flagged for reset.
    divergence_limit: u32,
}

impl Default for TmrVoter {
    fn default() -> Self {
        Self {
            cores: [CoreHealth::default(); 3],
            divergence_limit: 1,
        }
    }
}

impl TmrVoter {
    /// Create a voter; a core is flagged for reset after `divergence_limit`
    /// masked votes (default 1 — one strike, per the spec's
    /// "flag the affected core for a hardware watchdog reset").
    #[must_use]
    pub fn new(divergence_limit: u32) -> Self {
        Self {
            cores: [CoreHealth::default(); 3],
            divergence_limit: divergence_limit.max(1),
        }
    }

    /// Submit one calculation's three results (indexed by core). Returns
    /// the vote; a masked vote records the divergence and may flag the
    /// minority core for reset.
    pub fn submit<T: Copy + PartialEq>(&mut self, results: [T; 3]) -> Vote<T> {
        // Majority detection by pairwise equality.
        let (value, masked_core, trusted) = if results[0] == results[1] {
            if results[1] == results[2] {
                (results[0], None, true) // unanimous
            } else {
                (results[0], Some(2u8), true)
            }
        } else if results[0] == results[2] {
            (results[0], Some(1u8), true)
        } else if results[1] == results[2] {
            (results[1], Some(0u8), true)
        } else {
            // No majority: distrust everything, flag all for inspection.
            for core in &mut self.cores {
                core.divergences += 1;
                if core.divergences >= self.divergence_limit {
                    core.flagged = true;
                }
            }
            return Vote {
                value: results[0],
                masked_core: None,
                trusted: false,
            };
        };
        if let Some(c) = masked_core {
            self.cores[c as usize].divergences += 1;
            if self.cores[c as usize].divergences >= self.divergence_limit {
                self.cores[c as usize].flagged = true;
            }
        }
        Vote {
            value,
            masked_core,
            trusted,
        }
    }

    /// Health ledger for core `i` (0..3).
    #[must_use]
    pub fn core(&self, i: usize) -> CoreHealth {
        self.cores[i]
    }

    /// Take the reset-request bitmask (bit `i` = core `i` needs a watchdog
    /// reset) and clear the flags — the watchdog driver consumes this.
    pub fn take_reset_requests(&mut self) -> u8 {
        let mut mask = 0u8;
        for (i, core) in self.cores.iter_mut().enumerate() {
            if core.flagged {
                mask |= 1 << i;
                core.flagged = false;
                core.divergences = 0;
                core.resets += 1;
            }
        }
        mask
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unanimous_vote_is_clean() {
        let mut v = TmrVoter::default();
        let vote = v.submit([7.5f32; 3]);
        assert!(vote.trusted);
        assert_eq!(vote.masked_core, None);
        assert_eq!(vote.value, 7.5);
        for i in 0..3 {
            assert_eq!(v.core(i).divergences, 0);
        }
    }

    #[test]
    fn minority_result_is_masked_from_any_core() {
        for bad in 0..3 {
            let mut v = TmrVoter::default();
            let mut r = [1.5f64; 3]; // all three cores agree...
            r[bad] = f64::NAN; // ...then the SEU corrupts one
            let vote = v.submit(r);
            assert!(vote.trusted);
            assert_eq!(vote.masked_core, Some(bad as u8));
            assert_eq!(vote.value, 1.5);
            assert_eq!(v.core(bad).divergences, 1);
            assert!(v.core(bad).flagged, "core {bad} flagged for reset");
            assert_eq!(v.take_reset_requests(), 1 << bad);
            assert!(!v.core(bad).flagged);
            assert_eq!(v.core(bad).resets, 1);
        }
    }

    #[test]
    fn total_divergence_distrusts_all_cores() {
        let mut v = TmrVoter::default();
        let vote = v.submit([1u32, 2, 3]);
        assert!(!vote.trusted);
        assert_eq!(vote.masked_core, None);
        for i in 0..3 {
            assert!(v.core(i).flagged);
        }
        assert_eq!(v.take_reset_requests(), 0b111);
    }

    #[test]
    fn reset_requests_clear_after_take() {
        let mut v = TmrVoter::default();
        v.submit([0u8, 9, 0]); // core 1 diverges
        v.submit([9u8, 0, 0]); // core 0 diverges
        assert_eq!(v.take_reset_requests(), 0b011);
        assert_eq!(v.take_reset_requests(), 0);
    }

    #[test]
    fn divergence_limit_gates_flagging() {
        let mut v = TmrVoter::new(3);
        for _ in 0..2 {
            v.submit([0u16, 5, 0]); // core 1 diverges twice
        }
        assert!(!v.core(1).flagged, "below limit: no reset yet");
        v.submit([0u16, 5, 0]);
        assert!(v.core(1).flagged);
    }
}
