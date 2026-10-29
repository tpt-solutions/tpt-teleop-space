//! Record a short mission episode in the black box, then print the encoded
//! image as hex. Feed the output to `tpts blackbox` to decode it.
//!
//! Run with: `cargo run -q -p tpt-t-space-core --example blackbox_dump`

use tpt_t_space_core::blackbox::{BlackBox, Event, encoded_capacity};
use tpt_t_space_core::machine::MissionState;
use tpt_t_space_core::safe_mode::SafeModeCause;

fn main() {
    let mut bb = BlackBox::<8>::new();
    bb.record(
        0,
        Event::Transition {
            from: MissionState::Cruise,
            to: MissionState::Approach,
        },
    );
    bb.record(
        420,
        Event::Transition {
            from: MissionState::Approach,
            to: MissionState::Operations,
        },
    );
    bb.record(
        3_600,
        Event::SafeMode(SafeModeCause::PowerSurvivalThreshold),
    );
    bb.record(3_660, Event::Marker(2));

    let mut image = vec![0u8; encoded_capacity(8)];
    let n = bb.encode(&mut image).expect("buffer sized for the box");
    let hex: String = image[..n].iter().map(|b| format!("{b:02X}")).collect();
    println!("{hex}");
}
