//! Manual replan benchmark (spec §4.4: thousands of candidate trajectories
//! evaluated in <50 ms; the §6 example budgets 40 ms for a 15 m detour).
//! A plain binary timed with `std::time::Instant` — no harness dependency.
//!
//! ```sh
//! cargo bench --manifest-path crates/tpt-t-space-predictive/Cargo.toml
//! # enforce the spec gate (exit 1 on miss):
//! TPT_PRED_STRICT=1 cargo bench --manifest-path crates/tpt-t-space-predictive/Cargo.toml
//! ```

use std::time::Instant;

use tpt_t_space_predictive::dem::{DemConfig, DemGrid, StereoRig};
use tpt_t_space_predictive::planner::{Pose, RoverLimits, replan};

const W: usize = 96;
const H: usize = 96;

fn scene() -> DemGrid<W, H> {
    let mut dem = DemGrid::new(DemConfig::rover());
    // Terrain: rolling hills + the §6 hazard (a 25° slope band ahead).
    let hazard_slope = 25.0_f32.to_radians();
    for gy in -40..40 {
        for gx in -40..40 {
            let x = gx as f32 * 0.25;
            let y = gy as f32 * 0.25;
            let h = 0.4 * (x * 0.1).sin() + 0.3 * (y * 0.15).cos();
            let h = if (2.0..5.0).contains(&x) && y.abs() < 1.5 {
                hazard_slope.tan() * (x - 2.0) // the hazard band
            } else {
                h
            };
            dem.ingest_point(x, y, h);
        }
    }
    dem.finish();
    dem
}

fn main() {
    let dem = scene();
    let cfg = DemConfig::rover();
    let limits = RoverLimits::flight_rules();
    let pose = Pose::default();

    // Warmup.
    let _ = replan(&dem, &cfg, &limits, &pose);

    const REPS: u32 = 50;
    let start = Instant::now();
    let mut plans = 0u32;
    let mut detours = 0u32;
    for k in 0..REPS {
        // Slight pose jitter per rep (the rover advances).
        let p = Pose {
            x_m: (k % 8) as f32 * 0.1,
            y_m: ((k / 8) % 8) as f32 * 0.1 - 0.35,
            yaw_rad: 0.0,
        };
        if let Ok(plan) = replan(&dem, &cfg, &limits, &pose_of(p)) {
            plans += 1;
            if plan.best.curvature.abs() > 0.05 {
                detours += 1;
            }
        }
    }
    let elapsed = start.elapsed();
    let per_replan_ms = elapsed.as_secs_f64() * 1000.0 / REPS as f64;

    let candidates = limits.speed_steps * (2 * limits.curvature_steps + 1);
    println!(
        "predictive replan: {REPS} cycles in {elapsed:.3?} → {per_replan_ms:.3} ms/replan \
         (budget 50 ms, §6 example 40 ms); {candidates} candidates each"
    );
    println!("plans: {plans}/{REPS}, detours chosen: {detours}");

    if per_replan_ms > 50.0 {
        println!("FAIL: replan exceeds the 50 ms spec budget");
        if std::env::var("TPT_PRED_STRICT").as_deref() == Ok("1") {
            std::process::exit(1);
        }
    } else {
        println!("PASS: within the 50 ms spec budget");
        if std::env::var("TPT_PRED_STRICT").as_deref() == Ok("1") {
            std::process::exit(0);
        }
    }
}

fn pose_of(p: Pose) -> Pose {
    // (keeps the loop body honest: planner sees a fresh pose per rep)
    let _ = StereoRig {
        baseline_m: 0.5,
        focal_px: 800.0,
        height_m: 1.5,
        disparity_range: (4.0, 200.0),
    };
    p
}
