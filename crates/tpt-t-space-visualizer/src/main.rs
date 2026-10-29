//! The visualizer binary.
//!
//! `tpt-t-space-visualizer [REPLAY]` — open a replay file written by
//! `tpts orbit` or `tpts scenario mars`, or the sim's demo replay when no
//! file is given.

use std::path::PathBuf;

use eframe::egui;
use tpt_t_space_visualizer::{ViewerApp, load_replay};

fn main() -> eframe::Result {
    let path = std::env::args_os().nth(1).map(PathBuf::from);
    let replay = match load_replay(path.as_deref()) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("tpt-t-space-visualizer: {e}");
            std::process::exit(2);
        }
    };
    let source = path.as_deref().and_then(|p| p.file_name()).map_or_else(
        || "demo replay".to_owned(),
        |n| n.to_string_lossy().into_owned(),
    );
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1024.0, 640.0])
            .with_title("tpt-teleop-space — visualizer"),
        ..Default::default()
    };
    eframe::run_native(
        "tpt-teleop-space",
        options,
        Box::new(move |_cc| Ok(Box::new(ViewerApp::new(replay, source)))),
    )
}
