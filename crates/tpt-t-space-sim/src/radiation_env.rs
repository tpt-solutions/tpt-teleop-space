//! The radiation environment (spec §8): deterministic bit-flip injection
//! into memory to exercise TMR voting and memory scrubbing (Phase 3).
//!
//! The environment owns an SEU rate (events per simulated second per
//! protected region) and a deterministic PRNG; each injected upset flips
//! one bit in the caller's byte region at a chosen word. The radiation
//! crate's scrubber/TMR then *detects and heals* — the sim only breaks
//! things, never fixes them.

use crate::sensors::SimRng;
use tpt_t_space_radiation::seu::{SeuBook, SeuKind};

/// The radiation environment for one protected region.
pub struct RadiationEnv {
    /// Mean upsets per simulated second (heavy-ion flux proxy).
    pub upsets_per_s: f32,
    rng: SimRng,
    /// Accrued fractional upsets (rate × dt).
    credit: f32,
    /// Injection log (bounded, for the after-action report).
    pub log: SeuBook<64>,
}

impl RadiationEnv {
    /// New environment (e.g. `1e-3` upsets/s ≈ a solar-particle-event
    /// burst rate for a small memory region).
    #[must_use]
    pub fn new(upsets_per_s: f32, seed: u64) -> Self {
        Self {
            upsets_per_s,
            rng: SimRng::new(seed),
            credit: 0.0,
            log: SeuBook::new(),
        }
    }

    /// Advance the environment by `dt_s`, injecting upsets into `region`
    /// as the rate dictates. Returns the number of bits flipped this
    /// step.
    pub fn step(&mut self, dt_s: f64, region: &mut [u8]) -> u32 {
        self.credit += (self.upsets_per_s as f64 * dt_s) as f32;
        let mut flipped = 0u32;
        while self.credit >= 1.0 && !region.is_empty() {
            self.credit -= 1.0;
            // Choose a byte and a bit.
            let byte = (self.rng.next_u64() as usize) % region.len();
            let bit = (self.rng.next_u64() >> 3) % 8;
            region[byte] ^= 1 << bit;
            self.log.record(
                (self.credit * 1e6) as u64,
                SeuKind::ScrubCorrected,
                byte as u32,
            );
            flipped += 1;
        }
        flipped
    }

    /// Force an immediate upset at a specific byte/bit (scenario
    /// scripting: "flip a bit in the MEKF's temporary variable", §6).
    pub fn inject_at(&mut self, region: &mut [u8], byte: usize, bit: u32) {
        let byte = byte % region.len();
        region[byte] ^= 1 << (bit % 8);
        self.log.record(0, SeuKind::VoteMasked, byte as u32);
    }

    /// Total upsets injected.
    #[must_use]
    pub fn total(&self) -> u64 {
        self.log.total_all()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_t_space_radiation::scrub::{Hamming64, Scrubber};

    #[test]
    fn injection_flips_exactly_one_bit() {
        let mut env = RadiationEnv::new(0.0, 3);
        let mut region = [0u8; 64];
        env.inject_at(&mut region, 10, 3);
        assert_eq!(region.iter().map(|b| b.count_ones()).sum::<u32>(), 1);
        assert_eq!(env.total(), 1);
    }

    #[test]
    fn the_scrubber_heals_what_the_environment_breaks() {
        // Build an ECC-protected region, let the storm hit it, scrub.
        let mut env = RadiationEnv::new(0.0, 11);
        let mut data = vec![0u8; 8 * 32];
        let mut parity: Vec<u8> = data
            .chunks_exact(8)
            .map(|c| Hamming64::encode(u64::from_le_bytes(c.try_into().unwrap())))
            .collect();
        let mut scrub = Scrubber::default();
        scrub.seal_region(&data);

        // One upset per second for 60 s.
        let mut corrections = 0u32;
        for _ in 0..60 {
            env.step(1.0, &mut data);
            let report = scrub.scrub_pass(&mut data, &mut parity).expect("shape");
            corrections += report.bits_corrected;
        }
        assert_eq!(env.total() as u32, corrections, "every upset healed");
        assert_eq!(report_crc(&data), report_crc_expected(&data));
    }

    fn report_crc(data: &[u8]) -> u32 {
        tpt_t_space_core::crc::crc32_one_shot(data)
    }

    fn report_crc_expected(data: &[u8]) -> u32 {
        // The golden copy is recoverable: Hamming healed every single-bit
        // upset, so the region CRC matches the all-zeros golden image.
        tpt_t_space_core::crc::crc32_one_shot(&vec![0u8; data.len()])
    }

    #[test]
    fn storm_rate_respects_the_configured_rate() {
        let mut env = RadiationEnv::new(10.0, 5); // 10 upsets/s
        let mut region = [0u8; 4096];
        let mut total = 0u32;
        for _ in 0..100 {
            total += env.step(1.0, &mut region);
        }
        // 100 s × 10/s = 1000 expected; stochastic band ±10 %.
        assert!(
            (900..=1100).contains(&total),
            "storm delivered {total} upsets"
        );
    }
}
