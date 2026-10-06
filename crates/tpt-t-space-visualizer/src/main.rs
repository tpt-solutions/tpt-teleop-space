//! The visualizer binary.
fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1024.0, 640.0])
            .with_title("tpt-teleop-space — visualizer"),
        ..Default::default()
    };
    eframe::run_native(
        "tpt-teleop-space",
        options,
        Box::new(|_cc| Ok(Box::new(tpt_t_space_visualizer::ViewerApp::default()))),
    )
}
