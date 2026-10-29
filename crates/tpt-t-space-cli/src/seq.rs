//! `tpts seq "<steps>"` — lint an uplink sequence and dry-run it over the
//! spec §6 Mars terrain before it is sent.
//!
//! Steps are separated by `;` or `,`: `drive <metres>`, `turn <degrees>`
//! (positive = left), `capture <instrument-id>`. The dry run consults the
//! same DEM hazard model the rover uses, with the pose advancing as the
//! sequence would move it from the origin.

use std::process::ExitCode;

use tpt_t_space_predictive::dem::{DemConfig, DemGrid};
use tpt_t_space_sim::scenario::terrain;
use tpt_t_space_teleop::lint::{Issue, SequenceLimits, dry_run};
use tpt_t_space_teleop::sequencing::{HazardCheck, Step};

use crate::{CliResult, positional};

/// Spacing between terrain samples along a drive, metres.
const SAMPLE_M: f32 = 0.1;

pub fn run(args: &[String]) -> CliResult {
    let text =
        positional(args, &[]).ok_or("seq needs a sequence, e.g. \"drive 1; turn 90; drive 3\"")?;
    let steps = parse(text)?;
    let dem = terrain();
    let mut oracle = TerrainOracle {
        dem: &dem,
        cfg: DemConfig::rover(),
        x_m: 0.0,
        y_m: 0.0,
        yaw_rad: 0.0,
    };
    let report = dry_run(&steps, &SequenceLimits::rover_default(), &mut oracle);

    println!("steps              {}", report.steps);
    println!("commanded drive    {:.1} m", report.drive_m);
    println!(
        "final pose         ({:.1}, {:.1}) m, yaw {:.0}°",
        oracle.x_m,
        oracle.y_m,
        oracle.yaw_rad.to_degrees()
    );
    if report.is_clean() {
        println!("\nPASS: sequence is clean for uplink.");
        return Ok(ExitCode::SUCCESS);
    }
    println!();
    for issue in &report.issues {
        println!("{}", describe(issue));
    }
    println!(
        "\nFAIL: {} issue(s); fix the sequence before uplink.",
        report.issues.len()
    );
    Ok(ExitCode::FAILURE)
}

/// Parse `drive 1; turn 90, capture 2` into steps.
fn parse(text: &str) -> Result<Vec<Step>, String> {
    text.split([';', ','])
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(|token| {
            let mut words = token.split_whitespace();
            let verb = words.next().unwrap_or_default();
            let arg = words
                .next()
                .ok_or_else(|| format!("`{token}` needs a value"))?;
            match verb {
                "drive" => arg
                    .parse()
                    .map(|metres| Step::Drive { metres })
                    .map_err(|_| format!("`{arg}` is not a distance in metres")),
                "turn" => arg
                    .parse()
                    .map(|degrees| Step::Turn { degrees })
                    .map_err(|_| format!("`{arg}` is not an angle in degrees")),
                "capture" => arg
                    .parse()
                    .map(|instrument_id| Step::Capture { instrument_id })
                    .map_err(|_| format!("`{arg}` is not an instrument id")),
                other => Err(format!("unknown step `{other}` (drive, turn, capture)")),
            }
        })
        .collect()
}

fn describe(issue: &Issue) -> String {
    match *issue {
        Issue::Empty => "empty sequence: nothing to uplink".to_owned(),
        Issue::TooManySteps { count, max } => {
            format!("{count} steps exceed the uplink budget of {max}")
        }
        Issue::BadDrive { index, metres } => {
            format!("step {index}: drive {metres} m is not a positive distance")
        }
        Issue::DriveTooLong { index, metres, max } => {
            format!("step {index}: drive {metres} m exceeds {max} m")
        }
        Issue::BadTurn {
            index,
            degrees,
            max,
        } => format!("step {index}: turn {degrees}° exceeds ±{max}°"),
        Issue::HazardRefused { index, step } => {
            format!("step {index}: {step:?} is refused by the hazard model")
        }
    }
}

/// The DEM hazard model as the rover's oracle, tracking the pose.
struct TerrainOracle<'a> {
    dem: &'a DemGrid<96, 96>,
    cfg: DemConfig,
    x_m: f32,
    y_m: f32,
    yaw_rad: f32,
}

impl HazardCheck for TerrainOracle<'_> {
    fn drive_safe(&mut self, metres: f32) -> bool {
        let (dx, dy) = (self.yaw_rad.cos(), self.yaw_rad.sin());
        let samples = (metres / SAMPLE_M).ceil().max(1.0) as u32;
        (1..=samples).all(|k| {
            let d = metres.min(k as f32 * SAMPLE_M);
            !self
                .dem
                .sample(self.x_m + dx * d, self.y_m + dy * d, &self.cfg)
                .hazard
        })
    }

    fn turn_safe(&mut self, _degrees: f32) -> bool {
        !self.dem.sample(self.x_m, self.y_m, &self.cfg).hazard
    }

    fn advance(&mut self, forward_m: f32, turned_deg: f32) {
        self.yaw_rad += turned_deg.to_radians();
        self.x_m += forward_m * self.yaw_rad.cos();
        self.y_m += forward_m * self.yaw_rad.sin();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_all_three_step_kinds_and_both_separators() {
        let steps = parse("drive 1.5; turn -30, capture 2").expect("parses");
        assert_eq!(
            steps,
            vec![
                Step::Drive { metres: 1.5 },
                Step::Turn { degrees: -30.0 },
                Step::Capture { instrument_id: 2 },
            ]
        );
    }

    #[test]
    fn rejects_unknown_verbs_and_bad_values() {
        assert!(parse("roll 3").unwrap_err().contains("unknown step"));
        assert!(parse("drive far").unwrap_err().contains("not a distance"));
        assert!(parse("turn").unwrap_err().contains("needs a value"));
    }

    #[test]
    fn oracle_refuses_the_hazard_band_and_follows_the_pose() {
        let dem = terrain();
        let mut oracle = TerrainOracle {
            dem: &dem,
            cfg: DemConfig::rover(),
            x_m: 0.0,
            y_m: 0.0,
            yaw_rad: 0.0,
        };
        assert!(oracle.drive_safe(1.0), "short of the 25° band");
        assert!(!oracle.drive_safe(5.0), "a 5 m drive runs into the band");
        oracle.advance(1.0, 90.0);
        assert!((oracle.y_m - 1.0).abs() < 1e-4 && oracle.x_m.abs() < 1e-4);
    }
}
