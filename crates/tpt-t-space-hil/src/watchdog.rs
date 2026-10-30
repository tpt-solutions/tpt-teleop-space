//! Simulated feed loop for the watchdog (Phase 16: watchdog wiring and
//! feed-loop validation, simulated).
//!
//! The watchdog logic is the shipped [`Watchdog`], driven through a register
//! mirror standing in for the kick register. Only the loop's timing is
//! modelled. Ticks are milliseconds, matching [`WatchdogConfig::loop_1hz`].

use std::cell::Cell;

use tpt_t_space_edge::reg::{Access, RegCell};
use tpt_t_space_edge::wdt::{FeedError, ResetCause, Watchdog, WatchdogConfig};
use tpt_t_space_sim::sensors::SimRng;

/// Nominal feed period of the 1 Hz loop, ms.
const PERIOD_MS: u64 = 1_000;

/// How the control loop behaves during a run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Scenario {
    /// Feeds once a second, each feed late by up to `jitter_ms`.
    Nominal {
        /// Largest scheduling delay, ms.
        jitter_ms: u64,
    },
    /// Once, at `at_ms`, the loop freezes for `for_ms` on top of its normal
    /// period, so the gap to the next feed is one period plus `for_ms`.
    Stall {
        /// When the freeze starts, ms.
        at_ms: u64,
        /// Extra time the freeze adds, ms.
        for_ms: u64,
    },
    /// The loop spins and tries to feed every `every_ms`.
    Runaway {
        /// Attempt interval, ms.
        every_ms: u64,
    },
}

/// What the watchdog saw over one run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Report {
    /// Feeds the watchdog accepted.
    pub feeds: u32,
    /// Feeds refused as too early (a runaway loop).
    pub too_early: u32,
    /// Bites recorded.
    pub bites: u32,
    /// Reset cause latched by the watchdog at the end of the run.
    pub cause: ResetCause,
    /// Feed-latency EWMA, `0..=1`.
    pub latency: f32,
}

/// Run `scenario` for `duration_ms` of loop time.
#[must_use]
pub fn run(scenario: Scenario, duration_ms: u64, seed: u64) -> Report {
    let cfg = WatchdogConfig::loop_1hz();
    let kick = Cell::new(0u32);
    let mut wdt = Watchdog::arm(cfg, RegCell::mirror(&kick, Access::ReadWrite));
    let mut rng = SimRng::new(seed);
    let mut report = Report {
        feeds: 0,
        too_early: 0,
        bites: 0,
        cause: ResetCause::PowerOn,
        latency: 0.0,
    };
    let mut stalled = false;
    // The first feed comes after one period: a feed at boot is inside the
    // window's minimum interval and would be refused as too early.
    let mut t = PERIOD_MS;
    while t < duration_ms {
        match wdt.feed(t) {
            Ok(()) => report.feeds += 1,
            Err(FeedError::TooEarly { .. }) => report.too_early += 1,
            Err(FeedError::BitLate) => report.bites += 1,
        }
        t = match scenario {
            Scenario::Nominal { jitter_ms } => t + PERIOD_MS + rng.next_u64() % (jitter_ms + 1),
            Scenario::Runaway { every_ms } => t + every_ms,
            Scenario::Stall { at_ms, for_ms } => {
                let next = t + PERIOD_MS;
                if !stalled && next >= at_ms {
                    stalled = true;
                    at_ms + for_ms
                } else {
                    next
                }
            }
        };
    }
    report.cause = wdt.cause();
    report.latency = wdt.latency();
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_loop_that_feeds_on_time_never_bites() {
        let r = run(Scenario::Nominal { jitter_ms: 200 }, 60_000, 7);
        assert_eq!(r.bites, 0);
        assert_eq!(r.too_early, 0);
        // About one feed per 1.1 s over 60 s.
        assert!(r.feeds >= 50, "{r:?}");
        assert_eq!(r.cause, ResetCause::PowerOn);
    }

    #[test]
    fn a_stall_past_the_deadline_bites_once_and_latches_the_cause() {
        // Deadline is 2500 ms; a gap of 1 s + 4 s misses it.
        let r = run(
            Scenario::Stall {
                at_ms: 10_000,
                for_ms: 4_000,
            },
            30_000,
            7,
        );
        assert_eq!(r.bites, 1);
        assert_eq!(r.cause, ResetCause::WatchdogBite);
    }

    #[test]
    fn a_stall_inside_the_window_is_survived() {
        // One period plus 1 s is a 2 s gap, inside the 2.5 s deadline.
        let r = run(
            Scenario::Stall {
                at_ms: 10_000,
                for_ms: 1_000,
            },
            30_000,
            7,
        );
        assert_eq!(r.bites, 0);
        assert_eq!(r.cause, ResetCause::PowerOn);
    }

    #[test]
    fn a_runaway_loop_is_refused_as_too_early() {
        let r = run(Scenario::Runaway { every_ms: 300 }, 10_000, 7);
        assert!(r.too_early > 0, "{r:?}");
        assert_eq!(r.bites, 0);
    }
}
