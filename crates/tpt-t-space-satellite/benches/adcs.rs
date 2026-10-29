//! Manual ADCS cycle benchmark (spec §4.3: the control loop executes in
//! <200 µs at 100 Hz+ on a pinned real-time core). Plain binary timed
//! with `std::time::Instant`.
//!
//! ```sh
//! cd crates/tpt-t-space-satellite && cargo bench
//! # enforce the spec gate (exit 1 on miss):
//! cd crates/tpt-t-space-satellite && TPT_ADCS_STRICT=1 cargo bench
//! ```

use std::time::Instant;

use tpt_t_space_satellite::adcs::{Adcs, AdcsConfig, Sensors, Targets};
use tpt_t_space_satellite::mekf::SensorNoise;

fn main() {
    let mut adcs = Adcs::new(
        [1.0, 0.0, 0.0, 0.0],
        SensorNoise::typical(),
        AdcsConfig::leo(),
    );
    let sensors = Sensors {
        gyro: Some([0.0, 0.0, 0.0]),
        startracker: Some([1.0, 0.0, 0.0, 0.0]),
        sun_body: Some([1.0, 0.0, 0.0]),
        sun_inertial: [1.0, 0.0, 0.0],
        mag_body: [0.0, 30.0, 0.0],
    };
    let targets = Targets::hold_identity();

    // Warmup.
    for _ in 0..200 {
        adcs.cycle(0.01, &sensors, &targets);
    }

    const CYCLES: u32 = 200_000; // 2000 simulated seconds at 100 Hz
    let start = Instant::now();
    for _ in 0..CYCLES {
        adcs.cycle(0.01, &sensors, &targets);
    }
    let elapsed = start.elapsed();
    let per_cycle_us = elapsed.as_nanos() as f64 / (CYCLES as f64) / 1000.0;

    println!(
        "satellite ADCS: {CYCLES} cycles in {elapsed:.3?} → {per_cycle_us:.2} µs/cycle \
         (budget 200 µs @ 100 Hz)"
    );
    println!(
        "pointing mode: {:?}, masked TMR votes: {}, wheel speeds: {:?} rpm",
        adcs.mode(),
        adcs.masked_votes,
        adcs.wheels().speeds_rpm()
    );

    if per_cycle_us > 200.0 {
        println!("FAIL: ADCS cycle exceeds the 200 µs spec budget");
        if std::env::var("TPT_ADCS_STRICT").as_deref() == Ok("1") {
            std::process::exit(1);
        }
    } else {
        println!("PASS: within the 200 µs spec budget");
        if std::env::var("TPT_ADCS_STRICT").as_deref() == Ok("1") {
            std::process::exit(0);
        }
    }
}
