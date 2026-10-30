//! Simulated light-time DTN rig (Phase 16: DTN link testing over a light-time
//! rig, simulated).
//!
//! Commands go from ground to rover, and acknowledgements come back over the
//! same link profile. Each one waits for the link to be up before it departs,
//! and takes one light time to arrive. The rover stores each command through
//! the shipped [`DtnAgent`]. No radio is involved, and the in-flight delay
//! is taken as given.

use tpt_t_space_dtn::bundle::{BundleStore, EndpointId, Priority};
use tpt_t_space_dtn::link::DtnAgent;
use tpt_t_space_sim::dtn_link::LinkProfile;

/// Results of one command series.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Report {
    /// Commands sent.
    pub sent: u32,
    /// Commands the rover stored under custody.
    pub stored: u32,
    /// Shortest round trip seen, seconds.
    pub min_rtt_s: f64,
    /// Mean round trip, seconds.
    pub mean_rtt_s: f64,
    /// Longest round trip, seconds.
    pub max_rtt_s: f64,
    /// Round trips that waited for the link at either end.
    pub delayed: u32,
    /// The round trip with the link always up: twice the one-way delay.
    pub nominal_rtt_s: f64,
}

/// The first instant at or after `t_s` when the link is up.
fn next_up(profile: &LinkProfile, t_s: f64) -> f64 {
    if profile.link_up(t_s) {
        t_s
    } else {
        t_s + profile.seconds_until_return(t_s)
    }
}

/// Send `commands` commands, one every `spacing_s` seconds, over `profile`.
#[must_use]
pub fn run(profile: LinkProfile, commands: u32, spacing_s: f64) -> Report {
    let one_way = profile.delay_s(0.0);
    // Sized to hold every command, so the measurement is the link, not NVRAM.
    let mut agent = DtnAgent::new(BundleStore::new((commands as usize).max(1), 256));
    let mut rtts = Vec::with_capacity(commands as usize);
    let mut stored = 0u32;
    let mut delayed = 0u32;
    for i in 0..commands {
        let sent = f64::from(i) * spacing_s;
        let depart = next_up(&profile, sent);
        let arrive = depart + one_way;
        let payload = i.to_le_bytes();
        let accepted = agent.submit(
            EndpointId::SURFACE_ASSET,
            &payload,
            Priority::Expedited,
            1,
            (arrive * 1e6) as u64,
        );
        stored += u32::from(accepted.is_ok());
        let ack_depart = next_up(&profile, arrive);
        let ack_arrive = ack_depart + one_way;
        rtts.push(ack_arrive - sent);
        if depart > sent || ack_depart > arrive {
            delayed += 1;
        }
    }
    let mean = if rtts.is_empty() {
        0.0
    } else {
        rtts.iter().sum::<f64>() / rtts.len() as f64
    };
    Report {
        sent: commands,
        stored,
        min_rtt_s: if rtts.is_empty() {
            0.0
        } else {
            rtts.iter().copied().fold(f64::INFINITY, f64::min)
        },
        mean_rtt_s: mean,
        max_rtt_s: rtts.iter().copied().fold(0.0_f64, f64::max),
        delayed,
        nominal_rtt_s: 2.0 * one_way,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_always_up_link_gives_the_nominal_round_trip() {
        let profile = LinkProfile {
            blackout_fraction: 0.0,
            ..LinkProfile::mars_surface()
        };
        let r = run(profile, 10, 600.0);
        assert_eq!(r.stored, 10);
        assert_eq!(r.delayed, 0);
        assert!((r.max_rtt_s - r.nominal_rtt_s).abs() < 1e-6, "{r:?}");
    }

    #[test]
    fn occultation_delays_some_round_trips_and_never_shortens_them() {
        let r = run(LinkProfile::mars_surface(), 24, 3_600.0);
        assert_eq!(r.stored, 24);
        assert!(r.delayed > 0, "{r:?}");
        assert!(r.min_rtt_s >= r.nominal_rtt_s - 1e-6, "{r:?}");
        assert!(r.max_rtt_s > r.nominal_rtt_s, "{r:?}");
    }
}
