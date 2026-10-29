//! Light-time-aware operator cues (Phase 20).
//!
//! The operator sends a command now but acts on it later. Three numbers
//! matter, and this module computes them so every display shows the same
//! ones:
//!
//! * when the command **arrives** at the asset (`sent + one-way delay`),
//! * when its **result is known** (the confirmation has to come back, so
//!   `sent + two-way delay`),
//! * how far the **predicted** view has drifted from the **confirmed** state
//!   the asset last reported, against the predictive display's bubble.

use crate::display::PredictedPose;

/// Timing of one command over a link with a fixed one-way delay.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommandTiming {
    /// When the operator sent the command, µs on the operator clock.
    pub sent_us: u64,
    /// When it reaches the asset, µs.
    pub arrives_us: u64,
    /// When the confirmation can be seen, µs.
    pub result_known_us: u64,
}

/// Timing for a command sent at `sent_us` over a link with `one_way_us` delay.
#[must_use]
pub fn command_timing(sent_us: u64, one_way_us: u64) -> CommandTiming {
    CommandTiming {
        sent_us,
        arrives_us: sent_us.saturating_add(one_way_us),
        result_known_us: sent_us.saturating_add(one_way_us.saturating_mul(2)),
    }
}

/// How far a prediction is from the confirmed state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Divergence {
    /// Straight-line distance between the predicted and confirmed positions, metres.
    pub distance_m: f64,
    /// Whether the confirmed position lies within the predicted 1σ bubble.
    pub within_bubble: bool,
}

/// Compare a prediction with the position the asset later confirmed.
#[must_use]
pub fn divergence(predicted: &PredictedPose, confirmed_xyz_m: [f64; 3]) -> Divergence {
    let d = [
        predicted.xyz_m[0] - confirmed_xyz_m[0],
        predicted.xyz_m[1] - confirmed_xyz_m[1],
        predicted.xyz_m[2] - confirmed_xyz_m[2],
    ];
    let distance_m = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
    Divergence {
        distance_m,
        within_bubble: distance_m <= f64::from(predicted.sigma_m),
    }
}

/// One line of operator text: time until the command arrives, and until
/// its result is known, measured from `now_us`.
#[must_use]
pub fn cue_text(timing: &CommandTiming, now_us: u64) -> String {
    format!(
        "arrives in {} · result known in {}",
        clock(timing.arrives_us.saturating_sub(now_us)),
        clock(timing.result_known_us.saturating_sub(now_us)),
    )
}

/// `h:mm:ss` (or `m:ss` under an hour) for a duration in µs.
fn clock(us: u64) -> String {
    let s = us / 1_000_000;
    if s >= 3_600 {
        format!("{}:{:02}:{:02}", s / 3_600, (s / 60) % 60, s % 60)
    } else {
        format!("{}:{:02}", s / 60, s % 60)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose(xyz: [f64; 3], sigma: f32) -> PredictedPose {
        PredictedPose {
            xyz_m: xyz,
            heading_deg: 0.0,
            sigma_m: sigma,
            state_age_us: 0,
        }
    }

    #[test]
    fn mars_command_arrives_in_twelve_minutes_and_is_known_in_twenty_four() {
        let one_way = 720_000_000; // 720 s
        let t = command_timing(1_000_000_000, one_way);
        assert_eq!(t.arrives_us - t.sent_us, 720_000_000);
        assert_eq!(t.result_known_us - t.sent_us, 1_440_000_000);
    }

    #[test]
    fn cue_text_counts_down_from_now() {
        let t = command_timing(0, 720_000_000);
        assert_eq!(cue_text(&t, 0), "arrives in 12:00 · result known in 24:00");
        assert_eq!(
            cue_text(&t, 300_000_000),
            "arrives in 7:00 · result known in 19:00"
        );
    }

    #[test]
    fn cue_text_switches_to_hours() {
        let t = command_timing(0, 4 * 3_600_000_000);
        assert_eq!(
            cue_text(&t, 0),
            "arrives in 4:00:00 · result known in 8:00:00"
        );
    }

    #[test]
    fn divergence_is_measured_against_the_bubble() {
        let p = pose([10.0, 0.0, 0.0], 2.0);
        let inside = divergence(&p, [11.0, 1.0, 0.0]);
        assert!((inside.distance_m - 2f64.sqrt()).abs() < 1e-9);
        assert!(inside.within_bubble);
        let outside = divergence(&p, [16.0, 0.0, 0.0]);
        assert!((outside.distance_m - 6.0).abs() < 1e-9);
        assert!(!outside.within_bubble);
    }
}
