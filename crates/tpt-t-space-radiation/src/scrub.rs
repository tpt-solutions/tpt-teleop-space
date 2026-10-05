//! Memory scrubbing: Hamming(72,64) SECDED per 64-bit word plus a CRC32
//! region walk (spec §4.1).
//!
//! A low-priority background task continuously reads protected memory,
//! checks each word's error-correcting code, and corrects single-bit errors
//! *before they accumulate into multi-bit failures*. Each 64-bit data word
//! carries an 8-bit check symbol: 7 Hamming parity bits + 1 overall parity
//! bit, giving single-error correction / double-error detection (SECDED).
//!
//! On top of the per-word ECC, [`Scrubber::seal_region`] snapshots a CRC32
//! over the whole protected region and [`Scrubber::verify_region`] compares
//! against it — the backstop that catches multi-bit corruption an
//! accumulate-then-burst upset can sneak past the word codes.
//!
//! Everything is allocation-free and panic-free: a corrupt word is corrected
//! or reported, never unreferenced.

use tpt_t_space_core::crc::crc32;

/// Outcome of checking one word's ECC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EccStatus {
    /// Word and check bits agree.
    Clean,
    /// A single-bit error was found and corrected (bit index 0..=71 across
    /// the 64 data bits, then the 8 check bits).
    Corrected(u8),
    /// Two (or more) errors detected — uncorrectable at this layer.
    Uncorrectable,
}

/// Hamming(72,64) SECDED codec for one 64-bit word.
///
/// Classic extended-Hamming layout over a 71-bit code space (positions
/// 1..=71): power-of-two positions (1,2,4,8,16,32,64) hold the seven group
/// parity bits; the remaining 64 positions hold the data. The check symbol
/// stores the seven group parities in bits 0..6 plus bit 7, the overall
/// parity over all 71 code bits. A single upset anywhere lands at odd
/// overall parity with a syndrome naming its exact position — unique for
/// every slot — so one-bit errors are corrected and two-bit errors are
/// always detected (never silently miscorrected).
#[derive(Debug, Clone, Copy, Default)]
pub struct Hamming64;

/// Position (1..=71) of data bit `b` in the code space: the `b+1`-th
/// non-power-of-two position.
const fn data_position(b: usize) -> u8 {
    let mut pos = 1usize;
    let mut seen = 0usize;
    while seen <= b {
        let is_pow2 = pos & (pos - 1) == 0;
        if !is_pow2 {
            if seen == b {
                return pos as u8;
            }
            seen += 1;
        }
        pos += 1;
    }
    0
}

impl Hamming64 {
    /// Compute the check symbol for a data word.
    #[must_use]
    pub fn encode(word: u64) -> u8 {
        let mut check = 0u8;
        for g in 0..7u32 {
            // Parity over the data positions belonging to group g. The
            // group's own parity bit (position 2^g) is stored as exactly
            // this value, so the group's total parity is even at rest and
            // the received syndrome names the flipped position directly.
            let ones = (word & hamming_group_mask(g)).count_ones();
            check |= ((ones & 1) as u8) << g;
        }
        // Overall parity over data + the 7 group bits (71 bits).
        let ones = word.count_ones() + (check & 0x7F).count_ones();
        check |= ((ones & 1) as u8) << 7;
        check
    }

    /// Check a data word against its stored check symbol, correcting a
    /// single-bit error in place if possible.
    #[must_use]
    pub fn check(word: u64, stored: u8) -> (u64, EccStatus) {
        let computed = Self::encode(word);
        let group_syndrome = (computed ^ stored) & 0x7F;
        // Odd total bit count across the 71 code bits = odd number of
        // upsets (correctable class); even = multi-bit (detected).
        let odd_weight =
            (word.count_ones() + (stored & 0x7F).count_ones() + (u32::from(stored) >> 7)) & 1 == 1;
        if !odd_weight {
            return if group_syndrome == 0 {
                (word, EccStatus::Clean)
            } else {
                (word, EccStatus::Uncorrectable)
            };
        }
        if group_syndrome == 0 {
            // Odd weight with clean groups: the overall parity bit itself
            // flipped; data is intact, the symbol is recomputed.
            return (word, EccStatus::Corrected(64 + 7));
        }
        if group_syndrome <= 71 {
            // A power-of-two syndrome names the group parity bit itself.
            if group_syndrome & (group_syndrome - 1) == 0 {
                let g = group_syndrome.trailing_zeros() as u8;
                return (word, EccStatus::Corrected(64 + g));
            }
            // Otherwise it names a data position — find which data bit.
            let mut b = 0usize;
            while b < 64 {
                if data_position(b) == group_syndrome {
                    return (word ^ (1u64 << b), EccStatus::Corrected(b as u8));
                }
                b += 1;
            }
        }
        (word, EccStatus::Uncorrectable)
    }
}

/// Mask selecting the data bits whose code position has bit `g` set
/// (data bits live at the non-power-of-two positions 1..=71).
const fn hamming_group_mask(g: u32) -> u64 {
    let mut mask = 0u64;
    let mut b = 0usize;
    while b < 64 {
        let pos = data_position(b) as u32;
        if (pos >> g) & 1 == 1 {
            mask |= 1u64 << b;
        }
        b += 1;
    }
    mask
}

/// Why a scrub pass did not run to completion over the requested range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SealError {
    /// Data and parity slices disagree in length (data must be a multiple
    /// of 8 bytes with one parity byte per word).
    RegionShapeMismatch,
}

impl core::fmt::Display for SealError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("region shape mismatch: expect 8n data bytes + n parity bytes")
    }
}

impl std::error::Error for SealError {}

/// Per-pass report of the scrubber.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ScrubReport {
    /// 64-bit words walked this pass.
    pub words_checked: u32,
    /// Single-bit errors corrected in place.
    pub bits_corrected: u32,
    /// Words found uncorrectable (multi-bit); the owner of the region is
    /// responsible for re-seeding it from a golden copy.
    pub words_uncorrectable: u32,
    /// Whether the region CRC32 matched the sealed reference (when sealed).
    pub crc_ok: Option<bool>,
}

/// The background memory scrubber state: counters across passes plus the
/// sealed region CRC reference.
#[derive(Debug, Clone, Copy, Default)]
pub struct Scrubber {
    passes: u64,
    corrections_total: u64,
    uncorrectable_total: u64,
    sealed_crc: Option<u32>,
}

impl Scrubber {
    /// Seal a region: compute and remember its CRC32 reference. Call after
    /// loading golden content (boot, commanded rewrite).
    pub fn seal_region(&mut self, data: &[u8]) {
        self.sealed_crc = Some(crc32(0, data));
    }

    /// Scrub one pass over `data` with its per-word check symbols
    /// (`parity.len() == data.len() / 8`). Single-bit errors are corrected
    /// in place; the report aggregates what was found.
    pub fn scrub_pass(
        &mut self,
        data: &mut [u8],
        parity: &mut [u8],
    ) -> Result<ScrubReport, SealError> {
        if data.len() % 8 != 0 || parity.len() != data.len() / 8 {
            return Err(SealError::RegionShapeMismatch);
        }
        let mut report = ScrubReport::default();
        for (w, p) in data.chunks_exact_mut(8).zip(parity.iter_mut()) {
            let mut word = u64::from_le_bytes(w.try_into().expect("8-byte chunk"));
            let stored = *p;
            let (fixed, status) = Hamming64::check(word, stored);
            match status {
                EccStatus::Clean => {}
                EccStatus::Corrected(bit) => {
                    report.bits_corrected += 1;
                    if bit < 64 {
                        word = fixed;
                        w.copy_from_slice(&word.to_le_bytes());
                    }
                    // Corrected check-bit (bit >= 64): data untouched, the
                    // recomputed symbol below heals the stored parity.
                    *p = Hamming64::encode(word);
                }
                EccStatus::Uncorrectable => {
                    report.words_uncorrectable += 1;
                }
            }
            report.words_checked += 1;
        }
        // CRC backstop (only meaningful when the region is sealed and
        // nothing was uncorrectable at word level).
        if let Some(expect) = self.sealed_crc {
            report.crc_ok = Some(crc32(0, data) == expect);
        }
        self.passes += 1;
        self.corrections_total += report.bits_corrected as u64;
        self.uncorrectable_total += report.words_uncorrectable as u64;
        Ok(report)
    }

    /// Total passes completed.
    #[must_use]
    pub fn passes(&self) -> u64 {
        self.passes
    }

    /// Lifetime single-bit corrections.
    #[must_use]
    pub fn corrections_total(&self) -> u64 {
        self.corrections_total
    }

    /// Lifetime uncorrectable words.
    #[must_use]
    pub fn uncorrectable_total(&self) -> u64 {
        self.uncorrectable_total
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn region(words: usize) -> (Vec<u8>, Vec<u8>) {
        let mut data = vec![0u8; words * 8];
        for (i, b) in data.iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(31).wrapping_add(7);
        }
        let parity: Vec<u8> = data
            .chunks_exact(8)
            .map(|c| Hamming64::encode(u64::from_le_bytes(c.try_into().unwrap())))
            .collect();
        (data, parity)
    }

    #[test]
    fn clean_region_reports_clean() {
        let (mut data, mut parity) = region(64);
        let mut s = Scrubber::default();
        s.seal_region(&data);
        let r = s.scrub_pass(&mut data, &mut parity).expect("shape");
        assert_eq!(r.words_checked, 64);
        assert_eq!(r.bits_corrected, 0);
        assert_eq!(r.words_uncorrectable, 0);
        assert_eq!(r.crc_ok, Some(true));
    }

    #[test]
    fn single_bit_flips_are_corrected_everywhere() {
        // Sweep one flipped bit through every byte/bit of word 5.
        for byte in 0..8usize {
            for bit in 0..8u32 {
                let (mut data, mut parity) = region(16);
                data[5 * 8 + byte] ^= 1 << bit;
                let mut s = Scrubber::default();
                let r = s.scrub_pass(&mut data, &mut parity).expect("shape");
                assert_eq!(r.bits_corrected, 1, "byte {byte} bit {bit}");
                assert_eq!(r.words_uncorrectable, 0);
                // The word content is healed.
                let golden = {
                    let (d, _) = region(16);
                    d
                };
                assert_eq!(data[5 * 8..6 * 8], golden[5 * 8..6 * 8]);
            }
        }
    }

    #[test]
    fn flipped_parity_bit_is_healed() {
        let (mut data, mut parity) = region(8);
        parity[2] ^= 0x40; // one check bit flips
        let mut s = Scrubber::default();
        let r = s.scrub_pass(&mut data, &mut parity).expect("shape");
        assert_eq!(r.bits_corrected, 1);
        // Recheck: region is clean now.
        let r2 = s.scrub_pass(&mut data, &mut parity).expect("shape");
        assert_eq!(r2.bits_corrected, 0);
    }

    #[test]
    fn double_bit_flip_is_detected_not_silently_accepted() {
        let (mut data, mut parity) = region(8);
        data[3] ^= 0x01;
        data[6] ^= 0x01; // two bits in the same word
        let mut s = Scrubber::default();
        let r = s.scrub_pass(&mut data, &mut parity).expect("shape");
        assert_eq!(r.words_uncorrectable, 1);
        assert_eq!(r.bits_corrected, 0);
    }

    #[test]
    fn crc_backstop_catches_multi_word_corruption() {
        let (mut data, mut parity) = region(32);
        let mut s = Scrubber::default();
        s.seal_region(&data);
        // Corrupt two words with *pairs* of bits each: Hamming sees even
        // weight per word (uncorrectable), and the CRC flags the region.
        data[0] ^= 0b0000_0011;
        data[17 * 8 + 4] ^= 0b1100_0000;
        let r = s.scrub_pass(&mut data, &mut parity).expect("shape");
        assert_eq!(r.words_uncorrectable, 2);
        assert_eq!(r.crc_ok, Some(false));
    }

    #[test]
    fn shape_mismatch_is_reported() {
        let (mut data, mut parity) = region(4);
        parity.pop();
        let mut s = Scrubber::default();
        assert_eq!(
            s.scrub_pass(&mut data, &mut parity),
            Err(SealError::RegionShapeMismatch)
        );
    }

    #[test]
    fn counters_accumulate_across_passes() {
        let (mut data, mut parity) = region(8);
        let mut s = Scrubber::default();
        data[0] ^= 1;
        s.scrub_pass(&mut data, &mut parity).unwrap();
        data[9] ^= 1;
        s.scrub_pass(&mut data, &mut parity).unwrap();
        let clean = s.scrub_pass(&mut data, &mut parity).unwrap();
        assert_eq!(clean.bits_corrected, 0);
        assert_eq!(s.passes(), 3);
        assert_eq!(s.corrections_total(), 2);
    }
}
