//! `tpts orbit` and `tpts scenario mars` — run the simulator scenarios and
//! write replay files for the visualizer.

use std::fs::File;
use std::io::BufWriter;
use std::process::ExitCode;

use tpt_t_space_sim::profile::{self, Profile};
use tpt_t_space_sim::replay::Replay;
use tpt_t_space_sim::scenario::{Event, leo_survey, mars_sample};

use crate::{CliResult, flag_value, positional};

/// `tpts orbit [--orbits N] [--step S] [--out FILE]`.
pub fn orbit(args: &[String]) -> CliResult {
    let orbits: u32 = parse_flag(args, "--orbits", 2)?;
    let step: f64 = parse_flag(args, "--step", 30.0)?;
    if orbits == 0 {
        return Err("--orbits must be at least 1".to_owned());
    }
    if step <= 0.0 || !step.is_finite() {
        return Err("--step must be a positive number of seconds".to_owned());
    }
    let out = flag_value(args, "--out")?.unwrap_or("orbit.replay");

    let (replay, events) = leo_survey(orbits, step);
    print_timeline(&events);
    print_budget_summary(&replay);
    write_replay(&replay, out)?;
    Ok(ExitCode::SUCCESS)
}

/// `tpts scenario mars [--out FILE]`.
pub fn scenario(args: &[String]) -> CliResult {
    match positional(args, &["--out"]) {
        Some("mars") => {}
        Some(other) => return Err(format!("unknown scenario `{other}` (available: mars)")),
        None => return Err("scenario needs a name (available: mars)".to_owned()),
    }
    let out = flag_value(args, "--out")?.unwrap_or("mars.replay");

    let run = mars_sample().map_err(|e| e.to_string())?;
    print_timeline(&run.events);
    println!();
    println!("blackout           {:.2} h", run.blackout_s / 3_600.0);
    println!("detour replan      {:.2} ms (budget 50 ms)", run.replan_ms);
    println!("max lateral clear  {:.1} m", run.max_clearance_m);
    println!(
        "SEU masked         core {:?}, reset mask {:#05b}",
        run.seu_masked_core, run.reset_mask
    );
    println!(
        "safe mode          entered={} beacon={} bps",
        run.safe_mode_entered, run.safe_mode_beacon_bps
    );
    println!(
        "downlink           {} bundles in {} flits",
        run.delivered_bundles, run.flits
    );
    println!("sim time           day {:.2}", run.sim_days);
    write_replay(&run.replay, out)?;
    Ok(ExitCode::SUCCESS)
}

/// `tpts profile [NAME]`: list the mission-profile presets, or show one.
pub fn profile(args: &[String]) -> CliResult {
    match positional(args, &[]) {
        None => {
            println!(
                "{:<14} {:>10} {:>10} {:>10}  summary",
                "name", "light (s)", "battery", "array"
            );
            for p in profile::all() {
                println!(
                    "{:<14} {:>10.2} {:>7.0} Wh {:>7.0} W  {}",
                    p.name, p.link.one_way_delay_s, p.battery_wh, p.array_w, p.summary
                );
            }
            Ok(ExitCode::SUCCESS)
        }
        Some(name) => {
            let p = profile::by_name(name).ok_or_else(|| {
                format!("unknown profile `{name}` (run `tpts profile` to list them)")
            })?;
            print_profile(&p);
            Ok(ExitCode::SUCCESS)
        }
    }
}

fn print_profile(p: &Profile) {
    println!("{}: {}", p.name, p.summary);
    println!("downlink");
    println!("  one-way light time   {:.2} s", p.link.one_way_delay_s);
    println!(
        "  occultation          {:.0} s every {:.0} s ({:.0} % down)",
        p.link.blackout_period_s * f64::from(p.link.blackout_fraction),
        p.link.blackout_period_s,
        f64::from(p.link.blackout_fraction) * 100.0
    );
    println!("power");
    println!("  battery              {:.0} Wh", p.battery_wh);
    println!("  array                {:.0} W clear-sky", p.array_w);
    println!(
        "  survival floor       {:.0} W ({:.0} % of rating)",
        p.array_w * p.survival_fraction,
        p.survival_fraction * 100.0
    );
    println!("thermal");
    println!(
        "  battery              {:.0} … {:.0} °C",
        p.battery.min_c, p.battery.max_c
    );
    println!(
        "  electronics          {:.0} … {:.0} °C",
        p.electronics.min_c, p.electronics.max_c
    );
    match p.rover {
        Some(r) => println!(
            "traverse
  tilt limit           {:.0}°
  lateral accel        {:.1} m/s²
  speed                {:.1} … {:.1} m/s",
            r.tilt_limit_rad.to_degrees(),
            r.lateral_accel_max,
            r.speed_range.0,
            r.speed_range.1
        ),
        None => println!("traverse               none (not a surface vehicle)"),
    }
}

fn parse_flag<T: std::str::FromStr>(args: &[String], name: &str, default: T) -> Result<T, String> {
    match flag_value(args, name)? {
        None => Ok(default),
        Some(v) => v
            .parse()
            .map_err(|_| format!("{name}: `{v}` is not a valid value")),
    }
}

fn print_timeline(events: &[Event]) {
    println!("{:<14} {:<10} detail", "T+", "phase");
    for e in events {
        let t = e.t_s as u64;
        println!(
            "{:<14} {:<10} {}",
            format!("{:02}:{:02}:{:02}", t / 3_600, (t / 60) % 60, t % 60),
            e.phase,
            e.detail
        );
    }
}

fn print_budget_summary(replay: &Replay) {
    let (mut p_min, mut p_max) = (f32::INFINITY, f32::NEG_INFINITY);
    let (mut s_min, mut s_max) = (f32::INFINITY, f32::NEG_INFINITY);
    for b in &replay.budgets {
        p_min = p_min.min(b.power_w);
        p_max = p_max.max(b.power_w);
        s_min = s_min.min(b.soc_pct);
        s_max = s_max.max(b.soc_pct);
    }
    println!();
    println!("samples            {}", replay.orbit.len());
    println!("harvest            {p_min:.0} … {p_max:.0} W");
    println!("battery SoC        {s_min:.1} … {s_max:.1} %");
}

fn write_replay(replay: &Replay, path: &str) -> Result<(), String> {
    let file = File::create(path).map_err(|e| format!("{path}: {e}"))?;
    let mut out = BufWriter::new(file);
    replay
        .write_to(&mut out)
        .and_then(|()| std::io::Write::flush(&mut out))
        .map_err(|e| format!("{path}: {e}"))?;
    println!("\nreplay written to {path}");
    Ok(())
}
