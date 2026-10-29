//! `tpts campaign seu|blackout [--seeds N]` — run a seeded fault-injection
//! sweep and print one row per parameter value.
//!
//! `tpts campaign run FILE` runs a sweep described by a scenario file, so a
//! shared file reproduces the same numbers. The format is one `key value…`
//! pair per line; `#` starts a comment:
//!
//! ```text
//! seeds 8
//! votes 600
//! seu-rates 0 0.05 0.2 1
//! blackout-hours 0.25 1 4
//! bundle-every-s 60
//! ```
//!
//! Every key is optional and defaults to the values used by `campaign seu`
//! and `campaign blackout`.

use std::process::ExitCode;

use tpt_t_space_sim::campaign::{blackout_campaign, seu_campaign};

use crate::{CliResult, flag_value, positional};

/// SEU rates swept, upsets per second per core.
const SEU_RATES: [f64; 5] = [0.0, 0.01, 0.05, 0.2, 1.0];
/// Blackout lengths swept, hours.
const BLACKOUT_HOURS: [f64; 5] = [0.25, 1.0, 2.0, 4.0, 8.0];

/// What one campaign run sweeps.
#[derive(Debug, Clone, PartialEq)]
pub struct Sweep {
    /// Seeds per parameter value.
    pub seeds: u32,
    /// Votes per seed in the SEU sweep.
    pub votes: u32,
    /// SEU rates, upsets per second per core.
    pub seu_rates: Vec<f64>,
    /// Blackout lengths, hours.
    pub blackout_hours: Vec<f64>,
    /// Mean telemetry interval, seconds.
    pub bundle_every_s: f64,
}

impl Default for Sweep {
    fn default() -> Self {
        Self {
            seeds: 8,
            votes: 600,
            seu_rates: SEU_RATES.to_vec(),
            blackout_hours: BLACKOUT_HOURS.to_vec(),
            bundle_every_s: 60.0,
        }
    }
}

pub fn run(args: &[String]) -> CliResult {
    match positional(args, &["--seeds"]) {
        Some("run") => {
            let path = args
                .iter()
                .skip_while(|a| a.as_str() != "run")
                .nth(1)
                .ok_or("campaign run needs a scenario file")?;
            let text = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
            let sweep = parse(&text)?;
            print_seu(&sweep.seu_rates, sweep.seeds, sweep.votes);
            println!();
            print_blackout(&sweep.blackout_hours, sweep.seeds, sweep.bundle_every_s);
            Ok(ExitCode::SUCCESS)
        }
        Some("seu") => {
            let seeds = seeds_flag(args)?;
            print_seu(&SEU_RATES, seeds, 600);
            Ok(ExitCode::SUCCESS)
        }
        Some("blackout") => {
            let seeds = seeds_flag(args)?;
            print_blackout(&BLACKOUT_HOURS, seeds, 60.0);
            Ok(ExitCode::SUCCESS)
        }
        Some(other) => Err(format!("unknown campaign `{other}` (available: seu, blackout, run)")),
        None => Err("campaign needs a name (available: seu, blackout, run)".to_owned()),
    }
}

fn seeds_flag(args: &[String]) -> Result<u32, String> {
    match flag_value(args, "--seeds")? {
        None => Ok(Sweep::default().seeds),
        Some(v) => v
            .parse()
            .ok()
            .filter(|n| *n > 0)
            .ok_or_else(|| format!("--seeds: `{v}` is not a positive count")),
    }
}

/// Parse a scenario file. Errors name the offending line.
pub fn parse(text: &str) -> Result<Sweep, String> {
    let mut sweep = Sweep::default();
    for (n, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or_default().trim();
        if line.is_empty() {
            continue;
        }
        let at = |msg: String| format!("line {}: {msg}", n + 1);
        let mut words = line.split_whitespace();
        let key = words.next().unwrap_or_default();
        let values: Vec<&str> = words.collect();
        if values.is_empty() {
            return Err(at(format!("`{key}` needs a value")));
        }
        match key {
            "seeds" => sweep.seeds = one(&values, at)?,
            "votes" => sweep.votes = one(&values, at)?,
            "bundle-every-s" => sweep.bundle_every_s = one(&values, at)?,
            "seu-rates" => sweep.seu_rates = many(&values, at)?,
            "blackout-hours" => sweep.blackout_hours = many(&values, at)?,
            other => return Err(at(format!("unknown key `{other}`"))),
        }
    }
    if sweep.seeds == 0 || sweep.votes == 0 {
        return Err("seeds and votes must be positive".to_owned());
    }
    Ok(sweep)
}

fn one<T: std::str::FromStr>(values: &[&str], at: impl Fn(String) -> String) -> Result<T, String> {
    match values {
        [v] => v.parse().map_err(|_| at(format!("`{v}` is not a valid value"))),
        _ => Err(at("expected exactly one value".to_owned())),
    }
}

fn many<T: std::str::FromStr>(values: &[&str], at: impl Fn(String) -> String) -> Result<Vec<T>, String> {
    values
        .iter()
        .map(|v| v.parse().map_err(|_| at(format!("`{v}` is not a valid value"))))
        .collect()
}

fn print_seu(rates: &[f64], seeds: u32, votes: u32) {
    println!(
        "{:>12} {:>7} {:>8} {:>8} {:>9} {:>9} {:>7}",
        "upsets/s", "votes", "clean", "masked", "detected", "silent", "silent%"
    );
    for r in seu_campaign(rates, seeds, votes) {
        let pct = 100.0 * r.silent as f64 / r.votes.max(1) as f64;
        println!(
            "{:>12.3} {:>7} {:>8} {:>8} {:>9} {:>9} {:>6.2}%",
            r.rate_per_s, r.votes, r.clean, r.masked, r.detected, r.silent, pct
        );
    }
    println!("\n{seeds} seed(s) × {votes} votes per rate; one vote per second.");
}

fn print_blackout(hours: &[f64], seeds: u32, every_s: f64) {
    println!(
        "{:>10} {:>9} {:>9} {:>10} {:>9}",
        "blackout", "accepted", "dropped", "delivered", "dropped%"
    );
    for r in blackout_campaign(hours, seeds, every_s) {
        let pct = 100.0 * r.dropped as f64 / (r.accepted + r.dropped).max(1) as f64;
        println!(
            "{:>8.2} h {:>9} {:>9} {:>10} {:>8.1}%",
            r.blackout_h, r.accepted, r.dropped, r.delivered, pct
        );
    }
    println!("\n{seeds} seed(s); telemetry about every {every_s} s (jittered), 16 NVRAM sectors.");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_empty_file_is_the_default_sweep() {
        assert_eq!(parse("").expect("parses"), Sweep::default());
    }

    #[test]
    fn keys_override_defaults_and_comments_are_ignored() {
        let s = parse("# repro for bug 42\nseeds 3\nseu-rates 0.1 0.5 # two rates\nblackout-hours 2\n")
            .expect("parses");
        assert_eq!(s.seeds, 3);
        assert_eq!(s.seu_rates, vec![0.1, 0.5]);
        assert_eq!(s.blackout_hours, vec![2.0]);
        assert_eq!(s.votes, 600);
    }

    #[test]
    fn errors_name_the_line() {
        assert!(parse("seeds 2\nseeds two\n").unwrap_err().starts_with("line 2:"));
        assert!(parse("warp 9\n").unwrap_err().contains("unknown key `warp`"));
        assert!(parse("seeds\n").unwrap_err().contains("needs a value"));
        assert!(parse("seeds 0\n").unwrap_err().contains("must be positive"));
    }
}
