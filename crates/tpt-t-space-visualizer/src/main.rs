//! The visualizer binary.
//!
//! `tpt-t-space-visualizer [REPLAY] [--ccsds FILE]` — open a replay file
//! written by `tpts orbit` or `tpts scenario mars`, or the demo when no file
//! is given. `--ccsds FILE` follows a CCSDS downlink capture as it grows.

use eframe::egui;
use tpt_t_space_visualizer::{ViewerApp, load, parse_args};

fn fail(msg: impl std::fmt::Display) -> ! {
    eprintln!("tpt-t-space-visualizer: {msg}");
    std::process::exit(2);
}

fn main() -> eframe::Result {
    let opts = parse_args(std::env::args().skip(1)).unwrap_or_else(|e| fail(e));
    let loaded = load(opts.replay.as_deref()).unwrap_or_else(|e| fail(e));
    let source = opts
        .replay
        .as_deref()
        .and_then(|p| p.file_name())
        .map_or_else(
            || "demo replay".to_owned(),
            |n| n.to_string_lossy().into_owned(),
        );
    let mut app = ViewerApp::new(loaded.replay, source);
    if let Some(mars) = loaded.mars {
        app = app.with_mars(mars);
    }
    if let Some(path) = opts.downlink {
        app = app.following(path).unwrap_or_else(|e| fail(e));
    }
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1024.0, 640.0])
            .with_title("tpt-teleop-space — visualizer"),
        ..Default::default()
    };
    eframe::run_native(
        "tpt-teleop-space",
        options,
        Box::new(move |_cc| Ok(Box::new(app))),
    )
}
