//! `tpts hil` — run the simulated Phase 16 bring-up rig and print what it
//! found.
//!
//! Every line is simulation evidence. None of it closes a hardware item in
//! `todo.md`; those stay open until the same checks run on real parts.

use std::process::ExitCode;

use tpt_t_space_hil::bringup;
use tpt_t_space_hil::environment::{self, Cycle};
use tpt_t_space_hil::headroom;
use tpt_t_space_hil::link;
use tpt_t_space_hil::scrub;
use tpt_t_space_hil::timing;
use tpt_t_space_hil::watchdog::{self, Scenario};
use tpt_t_space_sim::dtn_link::LinkProfile;

use crate::CliResult;

pub fn run(args: &[String]) -> CliResult {
    if !args.is_empty() {
        return Err("hil takes no arguments".to_owned());
    }
    println!("tpts hil: SIMULATED Phase 16 bring-up. Not hardware evidence.\n");
    let board_ok = board();
    watchdog_loop();
    radiation();
    timing();
    light_time();
    headroom_report();
    thermal();
    Ok(if board_ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    })
}

fn verdict(pass: bool) -> &'static str {
    if pass { "PASS" } else { "FAIL" }
}

/// Bring-up checks on a clean board, then the same checks with a stuck bit
/// injected, which must fail.
fn board() -> bool {
    println!("[1] Board bring-up (simulated dev board)");
    let clean = bringup::bring_up(&[], None);
    for c in &clean {
        println!("    {}  {:<28} {}", verdict(c.pass), c.name, c.evidence);
    }
    let faulted = bringup::bring_up(&[(0x0200, 4, true)], None);
    if let Some(c) = faulted.iter().find(|c| c.name == "SRAM march test") {
        let outcome = if c.pass { "MISSED" } else { "caught" };
        println!(
            "    fault injection: stuck-at-1 at 0x0200 bit 4 -> {outcome}  {}",
            c.evidence
        );
    }
    println!();
    clean.iter().all(|c| c.pass)
}

fn watchdog_loop() {
    println!("[2] Watchdog feed loop (real watchdog, simulated 1 Hz loop)");
    let cases = [
        (
            "nominal, 60 s, 200 ms jitter",
            Scenario::Nominal { jitter_ms: 200 },
            60_000,
        ),
        (
            "freeze 1 s past its period",
            Scenario::Stall {
                at_ms: 10_000,
                for_ms: 1_000,
            },
            30_000,
        ),
        (
            "freeze 4 s past its period",
            Scenario::Stall {
                at_ms: 10_000,
                for_ms: 4_000,
            },
            30_000,
        ),
        (
            "runaway loop, feeds every 300 ms",
            Scenario::Runaway { every_ms: 300 },
            30_000,
        ),
    ];
    for (name, scenario, duration_ms) in cases {
        let r = watchdog::run(scenario, duration_ms, 7);
        println!(
            "    {name:<34} feeds {:>3}  too early {:>3}  bites {}  cause {:?}",
            r.feeds, r.too_early, r.bites, r.cause
        );
    }
    println!();
}

fn radiation() {
    println!("[3] Radiation campaign (real scrubber, simulated upsets; 1 KiB, 200 s, 4 seeds)");
    println!(
        "    {:>8} {:>8} {:>8} {:>10} {:>14} {:>8} {:>8}",
        "upsets/s", "scrub s", "upsets", "corrected", "uncorrectable", "reseeds", "intact"
    );
    for row in scrub::sweep(&[0.01, 0.1, 0.5], &[1.0, 10.0, 100.0], 1024, 200.0, 4) {
        println!(
            "    {:>8.2} {:>8.0} {:>8} {:>10} {:>14} {:>8} {:>5}/{}",
            row.upsets_per_s,
            row.scrub_period_s,
            row.totals.upsets,
            row.totals.bits_corrected,
            row.totals.uncorrectable_words,
            row.totals.reseeds,
            row.intact_runs,
            row.seeds,
        );
    }
    println!();
}

fn timing() {
    println!("[4] Sensor timing vs a 1 kHz control loop, 10 s (transport latencies ASSUMED)");
    for r in timing::run(
        &timing::ADCS_FEEDS,
        timing::Transport::assumed_spi(),
        1_000.0,
        10.0,
        11,
    ) {
        println!(
            "    {:<13} samples {:>5}  dropped {:>3}  stale ticks {:>5}  p99 age {:>8.2} ms",
            r.name,
            r.samples,
            r.dropped,
            r.stale_ticks,
            r.p99_age_us / 1_000.0,
        );
    }
    println!();
}

fn light_time() {
    println!("[5] Light-time DTN link, 24 commands (ack returns over the same link)");
    let cases = [
        ("Mars surface", LinkProfile::mars_surface(), 3_600.0),
        ("LEO relay", LinkProfile::leo_relay(), 600.0),
    ];
    for (name, profile, spacing_s) in cases {
        let r = link::run(profile, 24, spacing_s);
        println!(
            "    {name:<13} stored {}/{}  rtt min {:>8.2} s  mean {:>8.1} s  max {:>8.1} s  nominal {:>8.2} s  delayed {}",
            r.stored, r.sent, r.min_rtt_s, r.mean_rtt_s, r.max_rtt_s, r.nominal_rtt_s, r.delayed,
        );
    }
    println!();
}

fn headroom_report() {
    println!("[6] Target headroom (HOST timings: how much slower a target could be)");
    if cfg!(debug_assertions) {
        println!(
            "    debug build: times are inflated; rerun with --release for representative numbers"
        );
    }
    for h in headroom::measure(5) {
        match h.slowdown_limit() {
            Some(limit) => println!(
                "    {:<28} host {:>10.3} µs  budget {:>8.0} µs  target may be up to {:>8.0}× slower",
                h.name, h.host_us, h.budget_us, limit,
            ),
            None => println!(
                "    {:<28} not measured here (budget {:.0} µs, nightly crate)",
                h.name, h.budget_us,
            ),
        }
    }
    println!();
}

fn thermal() {
    println!("[7] Thermal cycling, battery channel (thermal only: no vibration or vacuum)");
    let cases = [
        (
            "inside limits, 0 to 30 °C",
            Cycle {
                low_c: 0.0,
                high_c: 30.0,
                period_s: 3_600,
                cycles: 3,
            },
        ),
        (
            "cold plateau, -35 to 30 °C",
            Cycle {
                low_c: -35.0,
                high_c: 30.0,
                period_s: 3_600,
                cycles: 3,
            },
        ),
        (
            "hot plateau, 10 to 55 °C",
            Cycle {
                low_c: 10.0,
                high_c: 55.0,
                period_s: 3_600,
                cycles: 3,
            },
        ),
    ];
    for (name, cycle) in cases {
        let r = environment::run(cycle);
        let first = r
            .first_trip_s
            .map_or_else(|| "none".to_owned(), |t| format!("{t} s"));
        println!(
            "    {name:<28} low trips {}  high trips {}  first trip {first}",
            r.low_trips, r.high_trips,
        );
    }
    println!();
}
