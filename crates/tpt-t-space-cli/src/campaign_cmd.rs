//! `tpts campaign seu|blackout [--seeds N]` — run a seeded fault-injection
//! sweep and print one row per parameter value.

use std::process::ExitCode;

use tpt_t_space_sim::campaign::{blackout_campaign, seu_campaign};

use crate::{CliResult, flag_value, positional};

/// SEU rates swept, upsets per second per core.
const SEU_RATES: [f64; 5] = [0.0, 0.01, 0.05, 0.2, 1.0];
/// Blackout lengths swept, hours.
const BLACKOUT_HOURS: [f64; 5] = [0.25, 1.0, 2.0, 4.0, 8.0];

pub fn run(args: &[String]) -> CliResult {
    let seeds: u32 = match flag_value(args, "--seeds")? {
        None => 8,
        Some(v) => v
            .parse()
            .ok()
            .filter(|n| *n > 0)
            .ok_or_else(|| format!("--seeds: `{v}` is not a positive count"))?,
    };
    match positional(args, &["--seeds"]) {
        Some("seu") => {
            println!(
                "{:>12} {:>7} {:>8} {:>8} {:>9} {:>9} {:>7}",
                "upsets/s", "votes", "clean", "masked", "detected", "silent", "silent%"
            );
            for r in seu_campaign(&SEU_RATES, seeds, 600) {
                let pct = 100.0 * r.silent as f64 / r.votes.max(1) as f64;
                println!(
                    "{:>12.3} {:>7} {:>8} {:>8} {:>9} {:>9} {:>6.2}%",
                    r.rate_per_s, r.votes, r.clean, r.masked, r.detected, r.silent, pct
                );
            }
            println!("\n{seeds} seed(s) × 600 votes per rate; one vote per second.");
            Ok(ExitCode::SUCCESS)
        }
        Some("blackout") => {
            println!(
                "{:>10} {:>9} {:>9} {:>10} {:>9}",
                "blackout", "accepted", "dropped", "delivered", "dropped%"
            );
            for r in blackout_campaign(&BLACKOUT_HOURS, seeds, 60.0) {
                let pct = 100.0 * r.dropped as f64 / (r.accepted + r.dropped).max(1) as f64;
                println!(
                    "{:>8.2} h {:>9} {:>9} {:>10} {:>8.1}%",
                    r.blackout_h, r.accepted, r.dropped, r.delivered, pct
                );
            }
            println!(
                "\n{seeds} seed(s); telemetry about every minute (jittered), 16 NVRAM sectors."
            );
            Ok(ExitCode::SUCCESS)
        }
        Some(other) => Err(format!(
            "unknown campaign `{other}` (available: seu, blackout)"
        )),
        None => Err("campaign needs a name (available: seu, blackout)".to_owned()),
    }
}
