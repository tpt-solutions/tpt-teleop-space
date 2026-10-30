//! Simulated sensor timing against the control loop (Phase 16: HIL validation
//! of ADCS timing, simulated).
//!
//! The ADCS control loop lives in the nightly satellite crate, so this rig
//! does not run it. It models the sensor feeds and their transport to a
//! control loop that reads the newest sample each tick, and checks that each
//! sample is fresh enough when read.
//!
//! The transport numbers are **assumptions**, not measurements. Replace
//! [`Transport::assumed_spi`] with the latencies measured on the flight
//! harness before reading anything into the stale counts.

use tpt_t_space_sim::sensors::SimRng;

/// Latency model for one sensor's path to the control loop, microseconds.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Transport {
    /// Fixed part of the latency.
    pub nominal_us: f64,
    /// Scale of the half-normal jitter added to it.
    pub jitter_us: f64,
    /// Chance a sample takes the slow path.
    pub tail_prob: f64,
    /// Extra latency on the slow path.
    pub tail_us: f64,
}

impl Transport {
    /// Placeholder numbers for a serial sensor bus. Not measured.
    #[must_use]
    pub fn assumed_spi() -> Self {
        Self {
            nominal_us: 40.0,
            jitter_us: 10.0,
            tail_prob: 0.01,
            tail_us: 400.0,
        }
    }
}

/// One sensor feed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feed {
    /// Sensor name.
    pub name: &'static str,
    /// Sample rate, Hz.
    pub rate_hz: f64,
    /// Oldest sample the control loop accepts, microseconds.
    pub max_age_us: f64,
    /// Chance a sample is lost before it leaves the sensor.
    pub drop_prob: f64,
}

/// The ADCS feeds, at the rates and freshness limits of the mission design.
pub const ADCS_FEEDS: [Feed; 3] = [
    Feed {
        name: "star tracker",
        rate_hz: 10.0,
        max_age_us: 150_000.0,
        drop_prob: 0.02,
    },
    Feed {
        name: "gyro",
        rate_hz: 100.0,
        max_age_us: 20_000.0,
        drop_prob: 0.001,
    },
    Feed {
        name: "sun sensor",
        rate_hz: 20.0,
        max_age_us: 100_000.0,
        drop_prob: 0.005,
    },
];

/// What one feed looked like to the control loop.
#[derive(Debug, Clone, PartialEq)]
pub struct FeedReport {
    /// Sensor name.
    pub name: &'static str,
    /// Samples produced.
    pub samples: u32,
    /// Samples lost before transport.
    pub dropped: u32,
    /// Control ticks on which the newest sample was too old, or none had
    /// arrived.
    pub stale_ticks: u32,
    /// Control ticks run.
    pub ticks: u32,
    /// 99th-percentile sample age at read, microseconds.
    pub p99_age_us: f64,
    /// Oldest sample age seen at a read, microseconds.
    pub max_age_us: f64,
}

/// Run every feed against a control loop at `loop_hz` for `duration_s`.
#[must_use]
pub fn run(
    feeds: &[Feed],
    transport: Transport,
    loop_hz: f64,
    duration_s: f64,
    seed: u64,
) -> Vec<FeedReport> {
    feeds
        .iter()
        .enumerate()
        .map(|(i, feed)| {
            let mut rng = SimRng::new(seed ^ ((i as u64 + 1) << 32));
            run_feed(feed, transport, loop_hz, duration_s, &mut rng)
        })
        .collect()
}

fn run_feed(
    feed: &Feed,
    transport: Transport,
    loop_hz: f64,
    duration_s: f64,
    rng: &mut SimRng,
) -> FeedReport {
    // Arrivals: (time the sample reaches the loop, time it was taken), µs.
    let mut arrivals: Vec<(f64, f64)> = Vec::new();
    let mut samples = 0u32;
    let mut dropped = 0u32;
    let period_us = 1e6 / feed.rate_hz;
    let mut taken_us = 0.0_f64;
    while taken_us < duration_s * 1e6 {
        samples += 1;
        if unit(rng) < feed.drop_prob {
            dropped += 1;
        } else {
            let mut latency =
                transport.nominal_us + transport.jitter_us * f64::from(rng.gaussian(1.0)).abs();
            if unit(rng) < transport.tail_prob {
                latency += transport.tail_us;
            }
            arrivals.push((taken_us + latency, taken_us));
        }
        taken_us += period_us;
    }
    arrivals.sort_by(|a, b| a.0.total_cmp(&b.0));

    let tick_us = 1e6 / loop_hz;
    let ticks = (duration_s * loop_hz) as u32;
    let mut newest_taken: Option<f64> = None;
    let mut next = 0usize;
    let mut ages = Vec::with_capacity(ticks as usize);
    let mut stale_ticks = 0u32;
    for k in 0..ticks {
        let now_us = f64::from(k) * tick_us;
        while next < arrivals.len() && arrivals[next].0 <= now_us {
            let taken = arrivals[next].1;
            newest_taken = Some(newest_taken.map_or(taken, |n| n.max(taken)));
            next += 1;
        }
        match newest_taken {
            Some(taken) => {
                let age = now_us - taken;
                ages.push(age);
                if age > feed.max_age_us {
                    stale_ticks += 1;
                }
            }
            None => stale_ticks += 1,
        }
    }
    ages.sort_by(f64::total_cmp);
    let p99_age_us = ages
        .get(((ages.len() as f64 * 0.99) as usize).min(ages.len().saturating_sub(1)))
        .copied()
        .unwrap_or(0.0);
    let max_age_us = ages.last().copied().unwrap_or(0.0);
    FeedReport {
        name: feed.name,
        samples,
        dropped,
        stale_ticks,
        ticks,
        p99_age_us,
        max_age_us,
    }
}

/// A uniform draw in `[0, 1)`.
fn unit(rng: &mut SimRng) -> f64 {
    (rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    const QUIET: Transport = Transport {
        nominal_us: 40.0,
        jitter_us: 0.0,
        tail_prob: 0.0,
        tail_us: 0.0,
    };

    #[test]
    fn a_quiet_bus_with_no_drops_keeps_every_feed_fresh() {
        let lossless = ADCS_FEEDS.map(|f| Feed {
            drop_prob: 0.0,
            ..f
        });
        let rows = run(&lossless, QUIET, 1_000.0, 5.0, 11);
        for r in &rows {
            // Only the first tick of each feed can predate its first sample.
            assert!(r.stale_ticks <= 1, "{r:?}");
            assert!(r.max_age_us < 200_000.0, "{r:?}");
        }
    }

    #[test]
    fn a_slow_tail_makes_the_fast_gyro_stale() {
        let slow = Transport {
            tail_prob: 0.5,
            tail_us: 30_000.0,
            ..QUIET
        };
        let rows = run(&ADCS_FEEDS, slow, 1_000.0, 5.0, 11);
        let gyro = rows.iter().find(|r| r.name == "gyro").expect("gyro");
        assert!(gyro.stale_ticks > 0, "{gyro:?}");
    }

    #[test]
    fn drops_are_counted_against_samples() {
        let rows = run(&ADCS_FEEDS, QUIET, 1_000.0, 10.0, 11);
        let star = rows
            .iter()
            .find(|r| r.name == "star tracker")
            .expect("star");
        assert_eq!(star.samples, 100);
        assert!(star.dropped <= star.samples);
    }
}
