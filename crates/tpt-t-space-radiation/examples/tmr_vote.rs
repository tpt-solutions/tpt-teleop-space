//! Three cores compute the same result; a cosmic-ray bit flip corrupts one.
//! The TMR voter returns the majority value and flags the bad core for the
//! watchdog reset.
//!
//! Run with: `cargo run -p tpt-t-space-radiation --example tmr_vote`

use tpt_t_space_radiation::tmr::TmrVoter;

fn main() {
    let mut voter = TmrVoter::default();
    let good = [0.5_f32, -1.25, 3.0];
    let mut hit = good;
    // One mantissa bit flipped on core 1.
    hit[1] = f32::from_bits(hit[1].to_bits() ^ (1 << 20));

    let vote = voter.submit([good, hit, good]);
    println!("voted value : {:?}", vote.value);
    println!("trusted     : {}", vote.trusted);
    println!("masked core : {:?}", vote.masked_core);
    println!("reset mask  : {:#05b}", voter.take_reset_requests());
}
