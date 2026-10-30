//! Timeline scenario files (Phase 20): a shareable script that declares a
//! link geometry, faults, and telemetry at fixed simulated times, then
//! replays them deterministically against the real DTN store, TMR voter and
//! black box. The same file always produces the same report, so a file can
//! be sent with a bug report as the reproduction.
//!
//! ```text
//! link mars-surface        # mars-surface, leo-relay, or none (always up)
//! at 0     telemetry 4 48  # 4 bundles of 48 bytes queued for Earth
//! at 600   blackout 2      # link forced down for 2 h from here
//! at 1800  seu 1 20        # flip bit 20 of core 1 and vote
//! at 7200  safe-mode       # power-survival Safe Mode entry, into the box
//! at 9000  drain           # if the link is up, hand the backlog to it
//! ```
//!
//! Steps are replayed in time order; steps at the same time keep file order.
//! Without a `link` line the Mars surface geometry applies.

use tpt_t_space_core::blackbox::{BlackBox, Event as BoxEvent};
use tpt_t_space_core::safe_mode::SafeModeCause;
use tpt_t_space_dtn::bundle::{BundleStore, EndpointId, Priority};
use tpt_t_space_dtn::link::DtnAgent;
use tpt_t_space_radiation::tmr::TmrVoter;

use crate::dtn_link::LinkProfile;

/// NVRAM sectors and bytes per sector, as in the blackout campaign.
const SECTORS: usize = 16;
const SECTOR_BYTES: usize = 1024;
/// The word each TMR core computes; a flipped core differs from it.
const TRUTH: u32 = 0xDEAD_BEEF;
const HOUR_S: f64 = 3_600.0;

/// Link geometry the script runs under.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Link {
    /// Always up: no occultation.
    None,
    /// Mars surface → orbiter → Earth (spec §6).
    MarsSurface,
    /// LEO relay passes.
    LeoRelay,
}

impl Link {
    fn parse(name: &str) -> Option<Self> {
        match name {
            "none" => Some(Self::None),
            "mars-surface" => Some(Self::MarsSurface),
            "leo-relay" => Some(Self::LeoRelay),
            _ => None,
        }
    }

    fn profile(self) -> Option<LinkProfile> {
        match self {
            Self::None => None,
            Self::MarsSurface => Some(LinkProfile::mars_surface()),
            Self::LeoRelay => Some(LinkProfile::leo_relay()),
        }
    }
}

/// One thing that happens at a step's time.
#[derive(Debug, Clone, PartialEq)]
pub enum Action {
    /// Queue telemetry bundles for Earth.
    Telemetry {
        /// Bundles to queue.
        count: u32,
        /// Payload size of each bundle, bytes.
        bytes: usize,
    },
    /// Force the link down for `hours` from this step.
    Blackout {
        /// Length of the forced outage, hours.
        hours: f64,
    },
    /// Flip one bit in one TMR core, then vote.
    Seu {
        /// The core to upset, 0..3.
        core: usize,
        /// The bit to flip, 0..32.
        bit: u32,
    },
    /// If the link is up, transmit the backlog and release custody.
    Drain,
    /// Enter Safe Mode on power survival; logged to the black box.
    SafeMode,
}

/// An action at a simulated time, in seconds.
#[derive(Debug, Clone, PartialEq)]
pub struct Step {
    /// Simulation time, seconds.
    pub t_s: f64,
    /// What happens.
    pub action: Action,
}

/// A parsed scenario file.
#[derive(Debug, Clone, PartialEq)]
pub struct Script {
    /// Link geometry.
    pub link: Link,
    /// Steps, sorted by time.
    pub steps: Vec<Step>,
}

/// What a replay did. `lines` is the timeline; the counts summarize it.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Report {
    /// One line per step, in replay order.
    pub lines: Vec<String>,
    /// Telemetry bundles accepted into NVRAM.
    pub accepted: u64,
    /// Telemetry bundles refused (full or oversized).
    pub dropped: u64,
    /// Bundles delivered over the link.
    pub delivered: u64,
    /// Upsets outvoted by TMR.
    pub masked: u32,
    /// Upsets the voter refused to trust.
    pub detected: u32,
    /// Upsets that produced a wrong trusted value.
    pub silent: u32,
    /// Records held in the black box at the end.
    pub blackbox_records: usize,
}

/// Parse a scenario file. Errors name the offending line.
///
/// # Errors
/// Returns a message naming the line for any unknown key, action, or
/// malformed value.
pub fn parse(text: &str) -> Result<Script, String> {
    let mut link = Link::MarsSurface;
    let mut steps = Vec::new();
    for (n, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let at = |msg: String| format!("line {}: {msg}", n + 1);
        let words: Vec<&str> = line.split_whitespace().collect();
        match words.as_slice() {
            ["link", name] => {
                link = Link::parse(name).ok_or_else(|| at(format!("unknown link `{name}`")))?;
            }
            ["at", t, rest @ ..] => {
                let t_s = t
                    .parse::<f64>()
                    .ok()
                    .filter(|t| t.is_finite() && *t >= 0.0)
                    .ok_or_else(|| at(format!("`{t}` is not a time in seconds")))?;
                steps.push(Step {
                    t_s,
                    action: action(rest).map_err(at)?,
                });
            }
            _ => return Err(at("expected `link NAME` or `at TIME ACTION`".to_owned())),
        }
    }
    steps.sort_by(|a, b| a.t_s.total_cmp(&b.t_s));
    Ok(Script { link, steps })
}

fn action(words: &[&str]) -> Result<Action, String> {
    match words {
        ["telemetry", count, bytes] => Ok(Action::Telemetry {
            count: count
                .parse()
                .map_err(|_| format!("`{count}` is not a count"))?,
            bytes: bytes
                .parse()
                .map_err(|_| format!("`{bytes}` is not a byte count"))?,
        }),
        ["blackout", hours] => hours
            .parse::<f64>()
            .ok()
            .filter(|h| h.is_finite() && *h > 0.0)
            .map(|hours| Action::Blackout { hours })
            .ok_or_else(|| format!("`{hours}` is not a positive duration in hours")),
        ["seu", core, bit] => {
            let core: usize = core
                .parse()
                .ok()
                .filter(|c| *c < 3)
                .ok_or_else(|| format!("`{core}` is not a TMR core (0, 1 or 2)"))?;
            let bit: u32 = bit
                .parse()
                .ok()
                .filter(|b| *b < 32)
                .ok_or_else(|| format!("`{bit}` is not a bit (0..32)"))?;
            Ok(Action::Seu { core, bit })
        }
        ["drain"] => Ok(Action::Drain),
        ["safe-mode"] => Ok(Action::SafeMode),
        _ => Err(format!("unknown action `{}`", words.join(" "))),
    }
}

/// Replay a script and report what happened.
///
/// # Errors
/// Returns a message if the black box cannot hold an entry; every other
/// failure (a full store, an oversized bundle) is counted in the report.
pub fn run(script: &Script) -> Result<Report, String> {
    let profile = script.link.profile();
    let mut agent = DtnAgent::new(BundleStore::new(SECTORS, SECTOR_BYTES));
    let mut voter = TmrVoter::default();
    let mut recorder = BlackBox::<16>::new();
    let mut report = Report::default();
    // Forced-down window set by `blackout`, in seconds.
    let mut down_until = 0.0_f64;

    for step in &script.steps {
        let now_us = (step.t_s * 1e6) as u64;
        let line = match &step.action {
            Action::Telemetry { count, bytes } => {
                let payload = vec![0x5A_u8; *bytes];
                let (mut ok, mut lost) = (0u64, 0u64);
                for _ in 0..*count {
                    match agent.submit(EndpointId::EARTH_OPS, &payload, Priority::Normal, 2, now_us)
                    {
                        Ok(_) => ok += 1,
                        Err(_) => lost += 1,
                    }
                }
                report.accepted += ok;
                report.dropped += lost;
                format!("{ok} bundle(s) of {bytes} B accepted, {lost} refused")
            }
            Action::Blackout { hours } => {
                down_until = down_until.max(step.t_s + hours * HOUR_S);
                format!("link forced down for {hours} h")
            }
            Action::Seu { core, bit } => {
                let mut cores = [TRUTH; 3];
                cores[*core] ^= 1 << bit;
                let vote = voter.submit(cores);
                let outcome = match (vote.trusted, vote.value == TRUTH) {
                    (true, true) => {
                        report.masked += 1;
                        "masked"
                    }
                    (true, false) => {
                        report.silent += 1;
                        "SILENT (wrong value trusted)"
                    }
                    (false, _) => {
                        report.detected += 1;
                        "detected (no majority)"
                    }
                };
                format!("upset on core {core}, bit {bit}: {outcome}")
            }
            Action::Drain => {
                let up = step.t_s >= down_until && profile.is_none_or(|p| p.link_up(step.t_s));
                agent.set_link(up);
                if up {
                    let mut sent = 0u64;
                    while let Some(Ok(egress)) = agent.poll() {
                        if egress.final_flit && agent.deliver(egress.sector).is_ok() {
                            sent += 1;
                        }
                    }
                    report.delivered += sent;
                    format!("link up: {sent} bundle(s) delivered")
                } else {
                    format!("link down: {} bundle(s) held", agent.pending())
                }
            }
            Action::SafeMode => {
                recorder.record(
                    step.t_s as u32,
                    BoxEvent::SafeMode(SafeModeCause::PowerSurvivalThreshold),
                );
                "safe mode entered (power survival)".to_owned()
            }
        };
        report.lines.push(format!("{:>10.1} s  {line}", step.t_s));
    }
    report.blackbox_records = recorder.len();
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = "\
link none
at 0     telemetry 4 48
at 600   blackout 2
at 700   telemetry 2 48
at 1800  seu 1 20
at 7200  safe-mode
at 8000  drain
";

    #[test]
    fn steps_are_sorted_and_the_link_is_read() {
        let s = parse("at 50 drain\nat 10 safe-mode\nlink leo-relay\n").expect("parses");
        assert_eq!(s.link, Link::LeoRelay);
        assert_eq!(s.steps[0].action, Action::SafeMode);
        assert_eq!(s.steps[1].action, Action::Drain);
    }

    #[test]
    fn errors_name_the_line() {
        assert!(
            parse("link moon\n")
                .unwrap_err()
                .contains("line 1: unknown link `moon`")
        );
        assert!(
            parse("\nat soon drain\n")
                .unwrap_err()
                .starts_with("line 2:")
        );
        assert!(
            parse("at 0 seu 3 0\n")
                .unwrap_err()
                .contains("not a TMR core")
        );
        assert!(parse("at 0 blackout -1\n").is_err());
        assert!(
            parse("at 0 explode\n")
                .unwrap_err()
                .contains("unknown action")
        );
    }

    #[test]
    fn a_single_upset_is_masked_and_never_silent() {
        let s = parse("link none\nat 0 seu 0 31\nat 1 seu 2 0\n").expect("parses");
        let r = run(&s).expect("runs");
        assert_eq!((r.masked, r.detected, r.silent), (2, 0, 0));
    }

    #[test]
    fn telemetry_queued_in_a_blackout_is_delivered_on_drain() {
        let s = parse(SAMPLE).expect("parses");
        let r = run(&s).expect("runs");
        assert_eq!(r.accepted, 6);
        assert_eq!(r.dropped, 0);
        // The blackout ends at 7800 s, so the drain at 8000 s finds the link up.
        assert_eq!(r.delivered, 6);
        assert_eq!(r.blackbox_records, 1);
    }

    #[test]
    fn a_drain_inside_a_forced_blackout_holds_the_backlog() {
        let s = parse("link none\nat 0 blackout 1\nat 10 telemetry 3 48\nat 20 drain\n")
            .expect("parses");
        let r = run(&s).expect("runs");
        assert_eq!(r.delivered, 0);
        assert!(r.lines.last().expect("a line").contains("3 bundle(s) held"));
    }

    #[test]
    fn oversized_telemetry_is_refused_and_counted() {
        let s = parse("link none\nat 0 telemetry 2 999999\n").expect("parses");
        let r = run(&s).expect("runs");
        assert_eq!((r.accepted, r.dropped), (0, 2));
    }

    #[test]
    fn replays_are_deterministic() {
        let s = parse(SAMPLE).expect("parses");
        assert_eq!(run(&s).expect("runs"), run(&s).expect("runs"));
    }
}
