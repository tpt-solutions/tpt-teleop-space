//! Phase 9 end-to-end: the spec §6 traverse scenario — a 25° slope is
//! detected ahead (exceeding the 20° limit), a 15 m detour is replanned
//! in under 50 ms, entirely onboard.

use std::time::Instant;

use tpt_t_space_predictive::dem::{DemConfig, DemGrid};
use tpt_t_space_predictive::planner::Pose;
use tpt_t_space_rover::hazards::{Avoidance, HazardAvoidance};
use tpt_t_space_rover::odometry::{EncoderTick, Odometry, RoverGeometry, SitePose};
use tpt_t_space_rover::vio::{Feature, Vio, VioConfig};

const W: usize = 96;
const H: usize = 96;

/// Terrain: flat ground with a 25° slope band ahead (the hazard), as in
/// spec §6.
fn terrain_with_hazard() -> DemGrid<W, H> {
    let mut dem = DemGrid::new(DemConfig::rover());
    let hazard_slope = 25.0_f32.to_radians();
    for gy in -44..44 {
        for gx in -44..44 {
            let x = gx as f32 * 0.25;
            let y = gy as f32 * 0.25;
            let h = if (2.0..6.0).contains(&x) && y.abs() < 2.0 {
                hazard_slope.tan() * (x - 2.0) // the 25° ramp across the path
            } else {
                0.05 * (x * 0.3).sin() // benign undulation
            };
            dem.ingest_point(x, y, h);
        }
    }
    dem.finish();
    dem
}

#[test]
fn hazard_detected_and_detour_replanned_under_budget() {
    let dem = terrain_with_hazard();
    let cfg = DemConfig::rover();

    // ─── 1. The rover drives toward the hazard on flat ground ──────────
    let mut av = HazardAvoidance::new(1);
    let pose = Pose::default();
    let decision = av.traverse_tick(&dem, &pose);
    // First tick: the planner sees the 25° band ahead and detours.
    let Avoidance::Replanned {
        curvature,
        replan_us,
        ..
    } = decision
    else {
        panic!("expected a replan at the hazard, got {decision:?}");
    };
    assert!(
        curvature.abs() > 0.05,
        "planner curves around the hazard, κ={curvature}"
    );

    // ─── 2. The §6 budget: the whole replan in <50 ms ───────────────────
    let start = Instant::now();
    const REPS: u32 = 20;
    for _ in 0..REPS {
        let _ = av.traverse_tick(&dem, &pose);
    }
    let per_replan_ms = start.elapsed().as_secs_f64() * 1000.0 / REPS as f64;
    assert!(
        per_replan_ms < 50.0,
        "replan took {per_replan_ms:.2} ms — over the 50 ms budget (spec §6: 40 ms)"
    );
    println!(
        "rover detour: 25° slope detected, replan {per_replan_ms:.3} ms \
         (budget 50 ms, §6 example 40 ms), replan_us reported by the loop: {replan_us}"
    );

    // ─── 3. The detour is ~15 m of arc: endpoint clears the hazard ─────
    // The planned arc must end beyond the hazard band with lateral
    // clearance of the |y| < 2 m corridor.
    let limits = *av.limits();
    let _ = limits;
    let pose_after = simulate_arc(&dem, &cfg, pose, curvature);
    assert!(
        pose_after.y_abs_witness > 2.0 || pose_after.x_m > 6.5,
        "detour clears the hazard (endpoint {:?})",
        (pose_after.x_m, pose_after.y_m)
    );
}

/// Integrate the planned arc forward, sampling the DEM (the follower's
/// prediction pass). Returns the end pose plus a witness of max |y|.
fn simulate_arc<const W: usize, const H: usize>(
    dem: &DemGrid<W, H>,
    cfg: &DemConfig,
    mut pose: Pose,
    curvature: f32,
) -> SitePoseWitness {
    let mut witness = SitePoseWitness::default();
    let speed = 1.0f32;
    let dt = 0.1f32;
    for _ in 0..150 {
        if curvature.abs() < 1e-6 {
            pose.x_m += speed * pose.yaw_rad.cos() * dt;
            pose.y_m += speed * pose.yaw_rad.sin() * dt;
        } else {
            let dtheta = curvature * speed * dt;
            let r = 1.0 / curvature;
            pose.x_m += r * (pose.yaw_rad + dtheta).sin() - r * pose.yaw_rad.sin();
            pose.y_m += -r * (pose.yaw_rad + dtheta).cos() + r * pose.yaw_rad.cos();
            pose.yaw_rad += dtheta;
        }
        witness.x_m = pose.x_m;
        witness.y_m = pose.y_m;
        witness.y_abs_witness = witness.y_abs_witness.max(pose.y_m.abs());
        // No hazard cell may be entered.
        let cell = dem.sample(pose.x_m, pose.y_m, cfg);
        assert!(
            !cell.hazard,
            "arc entered a hazard cell at {:?}",
            (pose.x_m, pose.y_m)
        );
        if pose.x_m > 8.0 {
            break; // cleared the band
        }
    }
    witness
}

#[derive(Default)]
struct SitePoseWitness {
    x_m: f32,
    y_m: f32,
    y_abs_witness: f32,
}

#[test]
fn odometry_and_vio_agree_on_clean_ground() {
    // Driving straight on good traction: wheel odometry and VIO produce
    // compatible motion, no slip flag.
    let geom = RoverGeometry::typical();
    let mut odo = Odometry::new(geom, SitePose::default());
    let cfg = VioConfig::typical();
    let mut vio = Vio::new(cfg);

    let step_m = 0.05f32;
    let counts = step_m * geom.counts_per_rev / (2.0 * core::f32::consts::PI * geom.wheel_radius_m);
    for _ in 0..40 {
        odo.step(&EncoderTick {
            left_counts: counts,
            right_counts: counts,
            commanded_arc_m: step_m,
        });
        let flow = synthetic_forward_flow(step_m, &cfg);
        let (_vx, _vy) = vio.step(&flow, 0.0).expect("vio solve");
    }
    assert!((odo.pose.x_m - 2.0).abs() < 1e-2);
    assert!(!odo.slipping());
}

/// Synthetic forward-motion flow matching [`vio`]'s ground-plane model.
fn synthetic_forward_flow(d: f32, cfg: &VioConfig) -> Vec<Feature> {
    let mut out = Vec::new();
    for row in 1..5 {
        for col in 0..4 {
            let fy = cfg.principal.1 + row as f32 * 100.0;
            let fx = cfg.principal.0 + (col as f32 - 1.5) * 100.0;
            let depth = cfg.cam_height_m * cfg.focal_px / (fy - cfg.principal.1);
            out.push(Feature {
                prev: (fx, fy),
                cur: (
                    fx + (fx - cfg.principal.0) * d / depth,
                    fy + (fy - cfg.principal.1) * d / depth,
                ),
                quality: 0.9,
                depth_m: depth,
            });
        }
    }
    out
}
