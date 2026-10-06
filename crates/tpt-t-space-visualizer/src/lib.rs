//! `tpt-t-space-visualizer` — the egui renderer (spec §8).
//!
//! Four panes over the simulator's data model:
//! * **Orbit**: 3D trajectory + ground track,
//! * **Attitude**: quaternion + sun-vector alignment,
//! * **Terrain**: rover DEM with planned vs. actual path,
//! * **Budgets**: power/thermal traces over time.
//!
//! This crate is excluded from the workspace (the egui chain sits
//! outside the MIT-chain deny scope); it consumes only the workspace's
//! public types.

#![warn(missing_docs)]

use eframe::egui;

/// A shared model the panes render (populated by the sim loop or a
/// replay file).
#[derive(Debug, Clone)]
pub struct SimModel {
    /// Spacecraft trajectory samples, ECI metres (subsampled).
    pub trajectory: Vec<[f64; 3]>,
    /// Attitude quaternion history `[w, x, y, z]`.
    pub attitude: Vec<[f32; 4]>,
    /// Sun direction history (body frame).
    pub sun_body: Vec<[f32; 3]>,
    /// Rover DEM heights, row-major.
    pub dem: Vec<f32>,
    /// DEM dimensions.
    pub dem_dims: (usize, usize),
    /// Rover planned path (DEM cells).
    pub planned_path: Vec<(usize, usize)>,
    /// Rover actual path (DEM cells).
    pub actual_path: Vec<(usize, usize)>,
    /// Power budget trace (W).
    pub power_w: Vec<f32>,
    /// Battery SoC trace (%).
    pub soc_pct: Vec<f32>,
    /// Thermal trace (°C).
    pub temp_c: Vec<f32>,
}

impl Default for SimModel {
    fn default() -> Self {
        Self {
            trajectory: default_trajectory(),
            attitude: vec![[1.0, 0.0, 0.0, 0.0]; 64],
            sun_body: vec![[1.0, 0.0, 0.0]; 64],
            dem: vec![0.0; 64 * 64],
            dem_dims: (64, 64),
            planned_path: (0..64).map(|k| (k, 30)).collect(),
            actual_path: (0..40).map(|k| (k, 30 + (k / 8))).collect(),
            power_w: (0..128).map(|k| 400.0 + 60.0 * (k as f32 * 0.1).sin()).collect(),
            soc_pct: (0..128).map(|k| 90.0 - k as f32 * 0.2).collect(),
            temp_c: (0..128).map(|k| 15.0 + 8.0 * (k as f32 * 0.07).sin()).collect(),
        }
    }
}

/// A nominal LEO ring for the default view.
fn default_trajectory() -> Vec<[f64; 3]> {
    let r = 7.2e6;
    (0..180)
        .map(|k| {
            let t = k as f64 / 180.0 * std::f64::consts::TAU;
            [
                r * t.cos(),
                r * t.sin(),
                r * 0.35 * (2.0 * t).sin(),
            ]
        })
        .collect()
}

/// The viewer application.
pub struct ViewerApp {
    model: SimModel,
    selected_pane: Pane,
    zoom: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Pane {
    Orbit,
    Attitude,
    Terrain,
    Budgets,
}

impl Default for ViewerApp {
    fn default() -> Self {
        Self {
            model: SimModel::default(),
            selected_pane: Pane::Orbit,
            zoom: 1.0,
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("tabs").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("tpt-teleop-space visualizer");
                ui.separator();
                for (pane, label) in [
                    (Pane::Orbit, "Orbit"),
                    (Pane::Attitude, "Attitude"),
                    (Pane::Terrain, "Terrain"),
                    (Pane::Budgets, "Budgets"),
                ] {
                    if ui
                        .selectable_label(self.selected_pane == pane, label)
                        .clicked()
                    {
                        self.selected_pane = pane;
                    }
                }
                ui.separator();
                ui.add(egui::Slider::new(&mut self.zoom, 0.2..=5.0).text("zoom"));
            });
        });
        egui::CentralPanel::default().show(ctx, |ui| match self.selected_pane {
            Pane::Orbit => self.draw_orbit(ui),
            Pane::Attitude => self.draw_attitude(ui),
            Pane::Terrain => self.draw_terrain(ui),
            Pane::Budgets => self.draw_budgets(ui),
        });
    }
}

impl ViewerApp {
    fn draw_orbit(&mut self, ui: &mut egui::Ui) {
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let to_screen = |p: [f64; 3]| {
            let scale = rect.width().min(rect.height()) * 0.4 * self.zoom / 7.2e6;
            let (sin_y, cos_y) = 0.5_f32.sin_cos(); // fixed isometric-ish view
            egui::pos2(
                rect.center().x + (p[0] as f32 * cos_y - p[1] as f32 * sin_y) * scale,
                rect.center().y + (p[1] as f32 * sin_y + p[2] as f32) * scale,
            )
        };
        // Earth disc.
        painter.circle_filled(rect.center(), 18.0 * self.zoom, egui::Color32::from_rgb(30, 60, 120));
        // Trajectory polyline.
        for pair in self.model.trajectory.windows(2) {
            painter.line_segment(
                [to_screen(pair[0]), to_screen(pair[1])],
                egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(120, 220, 160)),
            );
        }
        // Ground track: project to the equatorial plane.
        for pair in self.model.trajectory.windows(2) {
            let g0 = [pair[0][0], pair[0][1], 0.0];
            let g1 = [pair[1][0], pair[1][1], 0.0];
            painter.line_segment(
                [to_screen(g0), to_screen(g1)],
                egui::Stroke::new(0.8_f32, egui::Color32::from_rgba_unmultiplied(200, 200, 120, 90)),
            );
        }
        painter.text(
            rect.left_bottom() + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{} trajectory samples", self.model.trajectory.len()),
            egui::TextStyle::Small.resolve(ui.style()),
            egui::Color32::GRAY,
        );
    }

    fn draw_attitude(&mut self, ui: &mut egui::Ui) {
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let Some(q) = self.model.attitude.last().copied() else {
            return;
        };
        let Some(sun) = self.model.sun_body.last().copied() else {
            return;
        };
        let center = rect.center();
        let r = rect.height() * 0.3 * self.zoom;
        // Spacecraft square.
        let corners = [
            center + egui::vec2(-r, -r),
            center + egui::vec2(r, -r),
            center + egui::vec2(r, r),
            center + egui::vec2(-r, r),
        ];
        painter.add(egui::Shape::convex_polygon(
            corners.to_vec(),
            egui::Color32::from_rgb(60, 60, 90),
            egui::Stroke::new(1.5_f32, egui::Color32::LIGHT_BLUE),
        ));
        // Sun vector arrow (body frame).
        let tip = center + egui::vec2(sun[0] * r * 1.6, -sun[1] * r * 1.6);
        painter.line_segment(
            [center, tip],
            egui::Stroke::new(2.5_f32, egui::Color32::from_rgb(240, 200, 80)),
        );
        painter.circle_filled(tip, 5.0, egui::Color32::from_rgb(240, 200, 80));
        painter.text(
            center + egui::vec2(0.0, r + 24.0),
            egui::Align2::CENTER_TOP,
            format!("q = [{:.3}, {:.3}, {:.3}, {:.3}]", q[0], q[1], q[2], q[3]),
            egui::TextStyle::Monospace.resolve(ui.style()),
            egui::Color32::LIGHT_GRAY,
        );
    }

    fn draw_terrain(&mut self, ui: &mut egui::Ui) {
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let (w, h) = self.model.dem_dims;
        let cell = rect.width().min(rect.height()) / w.max(1) as f32 * self.zoom;
        let origin = rect.center() - egui::vec2(w as f32 * cell / 2.0, h as f32 * cell / 2.0);
        // Height shading.
        let mut max_h = 0.0f32;
        self.model.dem.iter().for_each(|v| max_h = max_h.max(*v));
        for y in 0..h {
            for x in 0..w {
                let v = self.model.dem[y * w + x];
                let shade = if max_h > 0.0 { v / max_h } else { 0.0 };
                let c = egui::Color32::from_gray(40 + (180.0 * shade) as u8);
                painter.rect_filled(
                    egui::Rect::from_min_size(
                        origin + egui::vec2(x as f32 * cell, y as f32 * cell),
                        egui::vec2(cell, cell),
                    ),
                    0.0,
                    c,
                );
            }
        }
        // Planned (dashed teal) vs actual (solid amber) paths.
        let at = |c: (usize, usize)| origin + egui::vec2(c.0 as f32 * cell, c.1 as f32 * cell);
        for pair in self.model.planned_path.windows(2) {
            painter.line_segment(
                [at(pair[0]), at(pair[1])],
                egui::Stroke::new(1.2_f32, egui::Color32::from_rgb(90, 220, 220)),
            );
        }
        for pair in self.model.actual_path.windows(2) {
            painter.line_segment(
                [at(pair[0]), at(pair[1])],
                egui::Stroke::new(2.2_f32, egui::Color32::from_rgb(240, 180, 60)),
            );
        }
    }

    fn draw_budgets(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().show(ui, |ui| {
            self.trace(ui, "Solar power (W)", &self.model.power_w, egui::Color32::from_rgb(240, 200, 80));
            self.trace(ui, "Battery SoC (%)", &self.model.soc_pct, egui::Color32::from_rgb(120, 220, 140));
            self.trace(ui, "Electronics temp (°C)", &self.model.temp_c, egui::Color32::from_rgb(230, 120, 100));
        });
    }

    fn trace(&self, ui: &mut egui::Ui, label: &str, data: &[f32], color: egui::Color32) {
        ui.label(label);
        let (rect, _) = ui.allocate_exact_size(
            egui::vec2(ui.available_width(), 90.0),
            egui::Sense::hover(),
        );
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 2.0, egui::Color32::from_black_alpha(60));
        if data.len() < 2 {
            return;
        }
        let (min, max) = data
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), v| (a.min(*v), b.max(*v)));
        let span = (max - min).max(1e-3);
        let at = |k: usize| {
            egui::pos2(
                rect.left() + rect.width() * k as f32 / (data.len() - 1) as f32,
                rect.bottom() - rect.height() * (data[k] - min) / span,
            )
        };
        for k in 1..data.len() {
            painter.line_segment([at(k - 1), at(k)], egui::Stroke::new(1.5_f32, color));
        }
    }
}
