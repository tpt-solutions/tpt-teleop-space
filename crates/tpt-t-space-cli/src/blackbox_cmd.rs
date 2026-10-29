//! `tpts blackbox <hex|file>` — decode a Safe Mode black-box image written
//! by `tpt_t_space_core::blackbox`, oldest event first.

use std::process::ExitCode;

use tpt_t_space_core::blackbox::{Event, decode_each};

use crate::{CliResult, positional};

pub fn run(args: &[String]) -> CliResult {
    let input = positional(args, &[]).ok_or("blackbox needs a hex string or a file path")?;
    let bytes = crate::ccsds::load(input)?;
    println!("{:<12} event", "T+");
    match decode_each(&bytes, |r| {
        println!("{:<12} {}", format!("{}s", r.t_s), describe(r.event));
    }) {
        Ok(count) => {
            println!("\n{count} record(s)");
            Ok(ExitCode::SUCCESS)
        }
        Err(e) => {
            println!("cannot decode: {e}");
            Ok(ExitCode::FAILURE)
        }
    }
}

fn describe(event: Event) -> String {
    match event {
        Event::Transition { from, to } => format!("transition {from} -> {to}"),
        Event::SafeMode(cause) => format!("safe mode entered: {cause:?}"),
        Event::Marker(code) => format!("marker {code}"),
    }
}
