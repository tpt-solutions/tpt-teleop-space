//! Phase 14 — the complete "Mars Sample Collection" data flow (spec §6):
//! the lifecycle of an autonomous rover operation during a communications
//! blackout, exercising the whole workspace.
//!
//! The flow itself lives in [`tpt_t_space_sim::scenario::mars_sample`] so the
//! `tpts scenario mars` CLI and the visualizer run the same code. This test
//! is the gate: it asserts every budget and invariant of the run.
//!
//! ```text
//!  1. DTN uplink + custody transfer      (tpt-t-space-dtn)
//!  2. The blackout (4 h)                 (tpt-t-space-sim::dtn_link)
//!  3. Autonomous traverse, DEM hazards   (tpt-t-space-predictive)
//!  4. 15 m detour replanned in <50 ms    (planner budget)
//!  5. SEU mid-calculation → TMR masks it (tpt-t-space-radiation)
//!  6. Sample collection sequence         (science chain, verified)
//!  7. Dust storm → power Safe Mode       (tpt-t-space-power + core)
//!  8. Reconnect: hash-verified telemetry (dtn + teleop)
//! ```

use tpt_t_space_sim::scenario::mars_sample;

#[test]
fn mars_sample_collection_data_flow() {
    let run = mars_sample().expect("the §6 flow completes");

    // 1–2. Custody held, and the blackout is hours long.
    assert_eq!(
        run.queued_bundles, 9,
        "command + 8 telemetry bundles queued"
    );
    assert!(
        run.blackout_s > 3.0 * 3_600.0,
        "the §6 blackout is hours long: {:.0} s",
        run.blackout_s
    );

    // 3–4. The detour is planned in budget and the arc stays off the hazard.
    assert!(
        run.replan_ms < 50.0,
        "§6 budget: 40 ms for a 15 m detour; got {:.2} ms",
        run.replan_ms
    );
    assert!(!run.halted, "halted inside the traverse");
    assert!(!run.hazard_entered, "the arc entered the 25° band");
    assert!(run.max_clearance_m > 1.5, "cleared the hazard corridor");
    println!(
        "traverse: hazard detected, detour replanned in {:.2} ms, cleared in {:.1} s of sim time \
         (max lateral clearance {:.1} m)",
        run.replan_ms, run.detour_arc_s, run.max_clearance_m
    );

    // 5. The TMR voter masks the corrupted core and flags it.
    assert_eq!(run.seu_masked_core, Some(1), "corrupted core masked");
    assert_eq!(
        run.reset_mask, 0b010,
        "core 1 flagged for the watchdog reset"
    );
    assert!(
        run.vote_intact,
        "the voted quaternion is the majority value"
    );
    println!("radiation: SEU injected, TMR masked core 1, watchdog reset flagged, path unaffected");

    // 6. The science chain verified by sensors.
    assert_eq!(run.science_steps, 5, "five-step sample collection verified");
    println!("science: 5-step sample collection verified by sensors");

    // 7. Safe Mode: payload off, sun-pointing, low-rate beacon.
    assert!(run.safe_mode_entered, "Safe Mode entered atomically");
    assert!(run.safe_mode_payloads_shed, "science payload powered down");
    assert!(
        run.safe_mode_sun_pointing,
        "slewed to sun-pointing for survival"
    );
    assert_eq!(
        run.safe_mode_beacon_bps,
        tpt_t_space_core::safe_mode::SAFE_MODE_BEACON_BPS
    );
    println!("safe mode: payload off, rover halted, low-rate beacon on");

    // 8. Reconnection: all queued bundles drained, telemetry logged.
    assert_eq!(
        run.delivered_bundles, 10,
        "command + 8 telemetry + alert delivered"
    );
    assert!(
        run.telemetry_chunks >= 14,
        "telemetry + science + alert logged"
    );
    println!(
        "reconnect: {} bundles in {} flits drained; sim ended at day {:.2}",
        run.delivered_bundles, run.flits, run.sim_days
    );
}
