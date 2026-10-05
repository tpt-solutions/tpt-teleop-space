//! Manual TMR vote benchmark (spec §4.1: the vote-and-reset-trigger cycle
//! completes in <50 µs with zero heap allocation). No benchmark harness
//! dependency — a plain binary timed with `std::time::Instant`, in keeping
//! with the zero-bloat policy.
//!
//! ```sh
//! cargo bench --manifest-path crates/tpt-t-space-radiation/Cargo.toml
//! # enforce the spec gate (exit 1 on miss):
//! TPT_RAD_STRICT=1 cargo bench --manifest-path crates/tpt-t-space-radiation/Cargo.toml
//! ```
//!
//! The gate is informational by default because absolute numbers are
//! host-dependent; CI runs it without the env var and prints the numbers.

use std::time::Instant;

use tpt_t_space_radiation::seu::{SeuBook, SeuKind};
use tpt_t_space_radiation::tmr::TmrVoter;

/// The per-cycle TMR pattern: three redundant quaternion updates (one per
/// core), one vote, bookkeeping, and reset-mask drain.
fn main() {
    let mut voter = TmrVoter::default();
    let mut book = SeuBook::<64>::new();
    // The "calculations": a quaternion-ish 4-float update per core. Core 1
    // suffers an SEU partway through the run.
    let mut cores: [[f32; 4]; 3] = [[1.0, 0.0, 0.0, 0.0]; 3];

    const CYCLES: u32 = 1_000_000;
    // Warmup.
    for k in 0..2_000u32 {
        let vote = voter.submit(cores);
        if !vote.trusted {
            let _ = voter.take_reset_requests();
        }
        let _ = k;
    }

    let start = Instant::now();
    let mut masked = 0u32;
    for k in 0..CYCLES {
        // Advance each core's calculation (identical work per core).
        for q in &mut cores {
            q[1] += 1e-6;
            q[2] -= 1e-6;
            q[3] += 2e-6;
        }
        if k == CYCLES / 2 {
            // The upset: a flipped mantissa bit sends core 1 astray.
            cores[1][2] = f32::from_bits(cores[1][2].to_bits() ^ (1 << 20));
        }
        let vote = voter.submit(cores);
        if !vote.trusted {
            // Distrusted: hold last good (not modeled here).
        } else if vote.masked_core.is_some() {
            masked += 1;
            book.record(
                k as u64,
                SeuKind::VoteMasked,
                vote.masked_core.unwrap_or(0) as u32,
            );
            let mask = voter.take_reset_requests();
            if mask != 0 {
                // Watchdog would reset the flagged core here; re-converge.
                cores[mask.trailing_zeros() as usize] = cores[0];
            }
        }
    }
    let elapsed = start.elapsed();
    let per_cycle_us = elapsed.as_nanos() as f64 / (CYCLES as f64) / 1000.0;

    println!(
        "radiation TMR: {CYCLES} vote cycles in {:.3?} → {:.4} µs/cycle (budget 50 µs)",
        elapsed, per_cycle_us
    );
    println!(
        "masked votes: {masked}, SEU book totals: {}, reset cycles: {}",
        book.total_all(),
        voter.core(0).resets + voter.core(1).resets + voter.core(2).resets
    );

    if per_cycle_us > 50.0 {
        println!("FAIL: TMR cycle exceeds the 50 µs spec budget");
        if std::env::var("TPT_RAD_STRICT").as_deref() == Ok("1") {
            std::process::exit(1);
        }
    } else {
        println!("PASS: within the 50 µs spec budget");
        if std::env::var("TPT_RAD_STRICT").as_deref() == Ok("1") {
            std::process::exit(0);
        }
    }
}
