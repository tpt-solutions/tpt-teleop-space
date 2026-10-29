//! Fault-injection campaigns (Phase 20): sweep one fault parameter over many
//! seeds and report how the system behaved.
//!
//! * [`seu_campaign`] sweeps the single-event-upset rate against the real
//!   TMR voter. Each vote is classified as clean, masked (a corrupted core
//!   was outvoted), detected (no majority, so the voter refused to trust
//!   the result), or silent (a wrong value was trusted: the failure TMR
//!   exists to prevent).
//! * [`blackout_campaign`] sweeps the link-down length against the real DTN
//!   store. Telemetry is queued through the blackout, and the report counts
//!   what was accepted, dropped for lack of NVRAM sectors, and delivered
//!   after the link returns.
//!
//! Every trial is deterministic in its seed, so a row can be reproduced.
//! A dust-storm sweep is not offered: the power model cannot attenuate the
//! array partway, only shadow it, so such a sweep would not be faithful.

use tpt_t_space_dtn::bundle::{BundleStore, EndpointId, Priority};
use tpt_t_space_dtn::link::DtnAgent;
use tpt_t_space_radiation::tmr::TmrVoter;

use crate::sensors::SimRng;

/// The word each core computes; a corrupted core differs from it.
const TRUTH: u32 = 0xDEAD_BEEF;
/// Vote period, seconds: one TMR vote per second.
const VOTE_PERIOD_S: f64 = 1.0;

/// Outcome counts for one SEU rate.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SeuSummary {
    /// Upsets per second per core.
    pub rate_per_s: f64,
    /// Seeds run.
    pub trials: u32,
    /// Votes cast across all trials.
    pub votes: u64,
    /// Votes with no upset.
    pub clean: u64,
    /// Votes where an upset was outvoted and the true value returned.
    pub masked: u64,
    /// Votes the voter refused to trust.
    pub detected: u64,
    /// Votes that trusted a wrong value.
    pub silent: u64,
}

/// Run `seeds` trials of `votes` votes each at every rate in `rates_per_s`.
#[must_use]
pub fn seu_campaign(rates_per_s: &[f64], seeds: u32, votes: u32) -> Vec<SeuSummary> {
    rates_per_s
        .iter()
        .map(|&rate| {
            let p_hit = 1.0 - (-rate * VOTE_PERIOD_S).exp();
            let mut s = SeuSummary {
                rate_per_s: rate,
                trials: seeds,
                votes: 0,
                clean: 0,
                masked: 0,
                detected: 0,
                silent: 0,
            };
            for seed in 0..seeds {
                let mut rng = SimRng::new(u64::from(seed) + 1);
                let mut voter = TmrVoter::default();
                for _ in 0..votes {
                    s.votes += 1;
                    let mut cores = [TRUTH; 3];
                    let mut hit = false;
                    for core in &mut cores {
                        if unit(&mut rng) < p_hit {
                            *core ^= 1 << (rng.next_u64() % 32);
                            hit = true;
                        }
                    }
                    let vote = voter.submit(cores);
                    match (vote.trusted, vote.value == TRUTH, hit) {
                        (true, true, false) => s.clean += 1,
                        (true, true, true) => s.masked += 1,
                        (false, _, _) => s.detected += 1,
                        (true, false, _) => s.silent += 1,
                    }
                }
            }
            s
        })
        .collect()
}

/// Outcome counts for one blackout length.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BlackoutSummary {
    /// Link-down length, hours.
    pub blackout_h: f64,
    /// Seeds run.
    pub trials: u32,
    /// Telemetry bundles accepted into NVRAM.
    pub accepted: u64,
    /// Bundles refused because every sector was full.
    pub dropped: u64,
    /// Bundles delivered after the link returned.
    pub delivered: u64,
}

/// Run `seeds` trials at each blackout length. Telemetry arrives about every
/// `bundle_every_s` seconds (each interval jittered by the seed).
#[must_use]
pub fn blackout_campaign(hours: &[f64], seeds: u32, bundle_every_s: f64) -> Vec<BlackoutSummary> {
    hours
        .iter()
        .map(|&h| {
            let mut s = BlackoutSummary {
                blackout_h: h,
                trials: seeds,
                accepted: 0,
                dropped: 0,
                delivered: 0,
            };
            for seed in 0..seeds {
                let mut rng = SimRng::new(u64::from(seed) + 1);
                let mut agent = DtnAgent::new(BundleStore::new(16, 1024));
                let blackout_s = h * 3_600.0;
                let mut t = 0.0_f64;
                while t < blackout_s {
                    let payload = [0x5A_u8; 48];
                    let now_us = (t * 1e6) as u64;
                    match agent.submit(EndpointId::EARTH_OPS, &payload, Priority::Normal, 2, now_us)
                    {
                        Ok(_) => s.accepted += 1,
                        Err(_) => s.dropped += 1,
                    }
                    t += bundle_every_s * (0.5 + unit(&mut rng));
                }
                agent.set_link(true);
                while let Some(Ok(egress)) = agent.poll() {
                    if egress.final_flit && agent.deliver(egress.sector).is_ok() {
                        s.delivered += 1;
                    }
                }
            }
            s
        })
        .collect()
}

/// A uniform draw in `[0, 1)`.
fn unit(rng: &mut SimRng) -> f64 {
    (rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    // REQ: TMR-001
    #[test]
    fn no_upsets_means_every_vote_is_clean() {
        let rows = seu_campaign(&[0.0], 2, 50);
        let r = rows[0];
        assert_eq!(r.votes, 100);
        assert_eq!((r.clean, r.masked, r.detected, r.silent), (100, 0, 0, 0));
    }

    // REQ: TMR-001
    #[test]
    fn high_upset_rates_are_masked_or_detected_and_never_silent_at_low_rates() {
        let rows = seu_campaign(&[0.05, 2.0], 4, 200);
        let low = rows[0];
        assert!(low.masked > 0, "single upsets must be outvoted: {low:?}");
        assert_eq!(low.silent, 0, "a single upset is never trusted: {low:?}");
        let high = rows[1];
        assert!(
            high.detected > 0,
            "multiple upsets must be detected: {high:?}"
        );
        assert_eq!(
            high.clean + high.masked + high.detected + high.silent,
            high.votes
        );
    }

    #[test]
    fn short_blackouts_deliver_everything_and_long_ones_drop() {
        let rows = blackout_campaign(&[0.25, 4.0], 2, 60.0);
        let short = rows[0];
        assert_eq!(short.dropped, 0, "{short:?}");
        assert_eq!(short.delivered, short.accepted, "{short:?}");
        let long = rows[1];
        assert!(
            long.dropped > 0,
            "16 sectors cannot hold four hours at 1/min: {long:?}"
        );
        assert_eq!(
            long.delivered, long.accepted,
            "accepted bundles all survive: {long:?}"
        );
    }

    #[test]
    fn campaigns_are_deterministic_in_their_seeds() {
        assert_eq!(seu_campaign(&[0.5], 3, 30), seu_campaign(&[0.5], 3, 30));
        assert_eq!(
            blackout_campaign(&[1.0], 2, 90.0),
            blackout_campaign(&[1.0], 2, 90.0)
        );
    }
}
