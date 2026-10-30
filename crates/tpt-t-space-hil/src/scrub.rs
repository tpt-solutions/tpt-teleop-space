//! Simulated radiation campaign against the memory scrubber (Phase 16:
//! radiation test campaign, simulated).
//!
//! Upsets come from the sim's [`RadiationEnv`] at a chosen rate, and the
//! scrubber and Hamming code are the shipped ones. A beam test measures the
//! real upset rate. This sweeps the rate and the scrub period instead, to
//! show where single-bit correction stops being enough.

use tpt_t_space_radiation::scrub::{Hamming64, Scrubber};
use tpt_t_space_sim::radiation_env::RadiationEnv;

/// One campaign: a region, an upset rate, and a scrub period.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Campaign {
    /// Mean upsets per second over the whole region.
    pub upsets_per_s: f32,
    /// Time between scrub passes, seconds. Must be positive.
    pub scrub_period_s: f64,
    /// Protected region size, bytes. A multiple of 8.
    pub region_bytes: usize,
    /// Mission length, seconds.
    pub mission_s: f64,
    /// Seed for the upset stream.
    pub seed: u64,
}

/// What one campaign found.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Outcome {
    /// Upsets injected.
    pub upsets: u64,
    /// Single-bit errors the scrubber corrected.
    pub bits_corrected: u64,
    /// Words with two or more upsets since the last scrub: detected, not fixed.
    pub uncorrectable_words: u64,
    /// Passes where the region CRC failed after correction.
    pub crc_mismatches: u64,
    /// Times the owner re-seeded the region from its golden copy.
    pub reseeds: u64,
    /// Scrub passes run.
    pub passes: u64,
    /// Whether the region matched its golden copy at mission end.
    pub intact_at_end: bool,
}

/// Run one campaign.
///
/// # Panics
/// If `scrub_period_s` is not positive, or `region_bytes` is not a multiple
/// of 8.
#[must_use]
pub fn run(c: &Campaign) -> Outcome {
    assert!(c.scrub_period_s > 0.0, "scrub period must be positive");
    assert!(c.region_bytes % 8 == 0, "region must be whole words");
    let golden: Vec<u8> = (0..c.region_bytes)
        .map(|i| (i as u8).wrapping_mul(31).wrapping_add(7))
        .collect();
    let mut data = golden.clone();
    let mut parity = parity_of(&golden);
    let mut scrubber = Scrubber::default();
    scrubber.seal_region(&golden);
    let mut env = RadiationEnv::new(c.upsets_per_s, c.seed);
    let mut out = Outcome::default();
    let mut t = 0.0_f64;
    while t < c.mission_s {
        let dt = c.scrub_period_s.min(c.mission_s - t);
        out.upsets += u64::from(env.step(dt, &mut data));
        let Ok(rep) = scrubber.scrub_pass(&mut data, &mut parity) else {
            break;
        };
        out.passes += 1;
        out.bits_corrected += u64::from(rep.bits_corrected);
        out.uncorrectable_words += u64::from(rep.words_uncorrectable);
        if rep.crc_ok == Some(false) {
            out.crc_mismatches += 1;
        }
        if rep.words_uncorrectable > 0 || rep.crc_ok == Some(false) {
            // The owner re-seeds a damaged region from its golden copy.
            data.copy_from_slice(&golden);
            parity.copy_from_slice(&parity_of(&golden));
            out.reseeds += 1;
        }
        t += dt;
    }
    out.intact_at_end = data == golden;
    out
}

/// Totals for one (rate, period) cell of a sweep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Row {
    /// Upset rate, per second.
    pub upsets_per_s: f32,
    /// Scrub period, seconds.
    pub scrub_period_s: f64,
    /// Campaigns run (one per seed).
    pub seeds: u32,
    /// Totals across the seeds.
    pub totals: Outcome,
    /// Seeds that ended intact.
    pub intact_runs: u32,
}

/// Sweep every rate against every scrub period, `seeds` campaigns each.
#[must_use]
pub fn sweep(
    rates: &[f32],
    periods_s: &[f64],
    region_bytes: usize,
    mission_s: f64,
    seeds: u32,
) -> Vec<Row> {
    let mut rows = Vec::new();
    for &rate in rates {
        for &period in periods_s {
            let mut row = Row {
                upsets_per_s: rate,
                scrub_period_s: period,
                seeds,
                totals: Outcome::default(),
                intact_runs: 0,
            };
            for seed in 0..seeds {
                let o = run(&Campaign {
                    upsets_per_s: rate,
                    scrub_period_s: period,
                    region_bytes,
                    mission_s,
                    seed: u64::from(seed) + 1,
                });
                row.totals.upsets += o.upsets;
                row.totals.bits_corrected += o.bits_corrected;
                row.totals.uncorrectable_words += o.uncorrectable_words;
                row.totals.crc_mismatches += o.crc_mismatches;
                row.totals.reseeds += o.reseeds;
                row.totals.passes += o.passes;
                row.intact_runs += u32::from(o.intact_at_end);
            }
            rows.push(row);
        }
    }
    rows
}

/// The Hamming check symbol of every 8-byte word in `data`.
fn parity_of(data: &[u8]) -> Vec<u8> {
    data.chunks_exact(8)
        .map(|w| {
            let mut word = 0u64;
            for (k, b) in w.iter().enumerate() {
                word |= u64::from(*b) << (8 * k);
            }
            Hamming64::encode(word)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frequent_scrubbing_at_a_low_rate_keeps_the_region_intact() {
        let o = run(&Campaign {
            upsets_per_s: 0.01,
            scrub_period_s: 1.0,
            region_bytes: 1024,
            mission_s: 200.0,
            seed: 3,
        });
        assert!(o.upsets > 0, "the environment must inject something");
        assert_eq!(o.uncorrectable_words, 0, "{o:?}");
        assert!(o.intact_at_end, "{o:?}");
    }

    #[test]
    fn rare_scrubbing_at_a_high_rate_accumulates_double_hits() {
        let o = run(&Campaign {
            upsets_per_s: 0.5,
            scrub_period_s: 200.0,
            region_bytes: 1024,
            mission_s: 200.0,
            seed: 3,
        });
        assert!(o.uncorrectable_words > 0, "{o:?}");
        assert!(o.reseeds > 0, "{o:?}");
    }

    #[test]
    fn a_sweep_reports_every_cell_and_is_deterministic() {
        let a = sweep(&[0.01, 0.1], &[1.0, 100.0], 256, 100.0, 2);
        let b = sweep(&[0.01, 0.1], &[1.0, 100.0], 256, 100.0, 2);
        assert_eq!(a.len(), 4);
        assert_eq!(a, b);
    }
}
