//! The rover meets a 25° slope band across its corridor and replans a
//! detour from its DEM. Prints the chosen curvature and the planner's
//! measured time against the §6 budget.
//!
//! Run with: `cargo run -p tpt-t-space-predictive --release --example replan`

use tpt_t_space_predictive::dem::{DemConfig, DemGrid};
use tpt_t_space_predictive::planner::{Pose, RoverLimits, replan};

const N: usize = 96;

fn main() {
    let cfg = DemConfig::rover();
    let mut dem: DemGrid<N, N> = DemGrid::new(cfg);
    let slope = 25.0_f32.to_radians().tan();
    for gy in -44..44 {
        for gx in -44..44 {
            let (x, y) = (gx as f32 * 0.25, gy as f32 * 0.25);
            let h = if (2.0..6.0).contains(&x) && y.abs() < 2.0 {
                slope * (x - 2.0)
            } else {
                0.05 * (x * 0.3).sin()
            };
            dem.ingest_point(x, y, h);
        }
    }
    dem.finish();

    let pose = Pose {
        x_m: 0.0,
        y_m: 0.0,
        yaw_rad: 0.0,
    };
    match replan(&dem, &cfg, &RoverLimits::flight_rules(), &pose) {
        Ok(plan) => {
            println!("curvature   : {:.3} 1/m", plan.best.curvature);
            println!("speed       : {:.2} m/s", plan.best.speed);
            println!(
                "replan time : {} µs (budget: 40 ms for a 15 m detour)",
                plan.micros
            );
        }
        Err(e) => println!("no safe path: {e:?}"),
    }
}
