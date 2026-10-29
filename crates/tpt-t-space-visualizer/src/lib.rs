//! `tpt-t-space-visualizer` — the egui renderer (spec §8).
//!
//! Four panes over a [`Replay`](tpt_t_space_sim::replay::Replay), the same
//! data the headless sim and the `tpts` CLI write and read:
//! * **Orbit**: 3D trajectory + ground track,
//! * **Attitude**: body rotated by the quaternion, sun vector in the body
//!   frame, array-to-sun alignment,
//! * **Terrain**: rover DEM with planned vs. actual path,
//! * **Budgets**: power/battery/thermal traces with a playback cursor,
//! * **Link**: light-time cues for a command on the Mars rover downlink.
//!
//! The top-bar slider scrubs the orbit/attitude/budget sample index.
//!
//! This crate is excluded from the workspace (the egui chain sits
//! outside the MIT-chain deny scope); it builds in isolation.

#![warn(missing_docs)]

use std::error::Error;
use std::path::Path;

use eframe::egui;
use tpt_t_space_sim::profile;
use tpt_t_space_sim::replay::{Replay, Terrain};
use tpt_t_space_sim::scenario;
use tpt_t_space_teleop::lighttime::{command_timing, cue_text};

/// Load a replay from `path`, or the sim's demo replay when `path` is
/// `None`.
///
/// # Errors
/// Returns an error if the file cannot be read or does not parse, or if
/// the demo scenario fails.
pub fn load_replay(path: Option<&Path>) -> Result<Replay, Box<dyn Error>> {
    match path {
        Some(p) => {
            let text = std::fs::read_to_string(p)?;
            Ok(Replay::parse(&text)?)
        }
        None => Ok(scenario::demo_replay()?),
    }
}

/// The viewer application.
pub struct ViewerApp {
    replay: Replay,
    source: String,
    selected_pane: Pane,
    zoom: f32,
    /// Playback cursor into the orbit/attitude/budget samples.
    frame: usize,
    /// Link pane: operator minute at which the command is sent.
    send_min: f32,
    /// Link pane: operator minute the readout is taken at.
    now_min: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Pane {
    Orbit,
    Attitude,
    Terrain,
    Budgets,
    Link,
}

impl ViewerApp {
    /// A viewer over `replay`, labelled `source` in the top bar.
    #[must_use]
    pub fn new(replay: Replay, source: impl Into<String>) -> Self {
        Self {
            replay,
            source: source.into(),
            selected_pane: Pane::Orbit,
            zoom: 1.0,
            frame: 0,
            send_min: 0.0,
            now_min: 0.0,
        }
    }
}

impl eframe::App for ViewerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let samples = self
            .replay
            .orbit
            .len()
            .max(self.replay.attitude.len())
            .max(self.replay.budgets.len());
        let last = samples.saturating_sub(1);
        self.frame = self.frame.min(last);

        egui::TopBottomPanel::top("tabs").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(format!("tpt-teleop-space — {}", self.source));
                ui.separator();
                for (pane, label) in [
                    (Pane::Orbit, "Orbit"),
                    (Pane::Attitude, "Attitude"),
                    (Pane::Terrain, "Terrain"),
                    (Pane::Budgets, "Budgets"),
                    (Pane::Link, "Link"),
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
                if samples > 0 {
                    ui.add(egui::Slider::new(&mut self.frame, 0..=last).text("sample"));
                }
            });
        });
        egui::CentralPanel::default().show(ctx, |ui| match self.selected_pane {
            Pane::Orbit => self.draw_orbit(ui),
            Pane::Attitude => self.draw_attitude(ui),
            Pane::Terrain => self.draw_terrain(ui),
            Pane::Budgets => self.draw_budgets(ui),
            Pane::Link => self.draw_link(ui),
        });
    }
}

/// Earth radius for drawing, metres.
const R_EARTH_M: f64 = 6_378_137.0;

impl ViewerApp {
    fn draw_orbit(&mut self, ui: &mut egui::Ui) {
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let track = &self.replay.orbit;
        let radius = track
            .iter()
            .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
            .fold(R_EARTH_M, f64::max);
        let to_screen = |p: [f64; 3]| {
            let scale = rect.width().min(rect.height()) * 0.4 * self.zoom / radius as f32;
            let (sin_y, cos_y) = 0.5_f32.sin_cos(); // fixed isometric-ish view
            egui::pos2(
                rect.center().x + (p[0] as f32 * cos_y - p[1] as f32 * sin_y) * scale,
                rect.center().y + (p[1] as f32 * sin_y + p[2] as f32) * scale,
            )
        };
        let earth_px =
            (R_EARTH_M as f32 / radius as f32) * rect.width().min(rect.height()) * 0.4 * self.zoom;
        painter.circle_filled(
            rect.center(),
            earth_px,
            egui::Color32::from_rgb(30, 60, 120),
        );

        // Ground track: project to the equatorial plane.
        for pair in track.windows(2) {
            painter.line_segment(
                [
                    to_screen([pair[0][0], pair[0][1], 0.0]),
                    to_screen([pair[1][0], pair[1][1], 0.0]),
                ],
                egui::Stroke::new(
                    0.8_f32,
                    egui::Color32::from_rgba_unmultiplied(200, 200, 120, 90),
                ),
            );
        }
        // Trajectory, then the current spacecraft position.
        for pair in track.windows(2) {
            painter.line_segment(
                [to_screen(pair[0]), to_screen(pair[1])],
                egui::Stroke::new(1.5_f32, egui::Color32::from_rgb(120, 220, 160)),
            );
        }
        if let Some(p) = track.get(self.frame) {
            painter.circle_filled(to_screen(*p), 4.0, egui::Color32::WHITE);
        }
        painter.text(
            rect.left_bottom() + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{} trajectory samples", track.len()),
            egui::TextStyle::Small.resolve(ui.style()),
            egui::Color32::GRAY,
        );
    }

    fn draw_attitude(&mut self, ui: &mut egui::Ui) {
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let Some(sample) = self.replay.attitude.get(self.frame).copied() else {
            return;
        };
        let q = sample.q;
        let sun = sample.sun_body;
        let center = rect.center();
        let size = rect.height().min(rect.width()) * 0.18 * self.zoom;

        // Body → view: attitude rotation, then a fixed view rotation so the
        // box reads in three dimensions.
        let rot = quat_to_matrix(q);
        let view = |v: [f32; 3]| {
            let w = mat_vec(&rot, v);
            let (yaw_s, yaw_c) = 0.6_f32.sin_cos();
            let (pitch_s, pitch_c) = 0.4_f32.sin_cos();
            let x1 = w[0] * yaw_c - w[1] * yaw_s;
            let y1 = w[0] * yaw_s + w[1] * yaw_c;
            let y2 = y1 * pitch_c - w[2] * pitch_s;
            egui::pos2(center.x + x1 * size, center.y - y2 * size)
        };

        // Spacecraft: a unit cube, twelve edges.
        let corners: Vec<[f32; 3]> = (0..8)
            .map(|i| {
                [
                    if i & 1 == 0 { -0.5 } else { 0.5 },
                    if i & 2 == 0 { -0.5 } else { 0.5 },
                    if i & 4 == 0 { -0.5 } else { 0.5 },
                ]
            })
            .collect();
        for i in 0..8usize {
            for bit in [1usize, 2, 4] {
                if i & bit == 0 {
                    painter.line_segment(
                        [view(corners[i]), view(corners[i | bit])],
                        egui::Stroke::new(1.5_f32, egui::Color32::LIGHT_BLUE),
                    );
                }
            }
        }
        // Body axes (+x red, +y green, +z blue; +z is the array normal).
        for (axis, color) in [
            ([1.0, 0.0, 0.0], egui::Color32::from_rgb(230, 90, 90)),
            ([0.0, 1.0, 0.0], egui::Color32::from_rgb(120, 220, 120)),
            ([0.0, 0.0, 1.0], egui::Color32::from_rgb(110, 150, 240)),
        ] {
            painter.line_segment(
                [view([0.0; 3]), view(axis)],
                egui::Stroke::new(2.0_f32, color),
            );
        }
        // Sun vector, body frame.
        let tip = view([sun[0] * 1.3, sun[1] * 1.3, sun[2] * 1.3]);
        painter.line_segment(
            [view([0.0; 3]), tip],
            egui::Stroke::new(2.5_f32, egui::Color32::from_rgb(240, 200, 80)),
        );
        painter.circle_filled(tip, 5.0, egui::Color32::from_rgb(240, 200, 80));

        let alignment_deg = sun[2].clamp(-1.0, 1.0).acos().to_degrees();
        painter.text(
            center + egui::vec2(0.0, size * 1.9),
            egui::Align2::CENTER_TOP,
            format!(
                "q = [{:.3}, {:.3}, {:.3}, {:.3}]\narray-to-sun {alignment_deg:.1}°  (sample {})",
                q[0], q[1], q[2], q[3], self.frame
            ),
            egui::TextStyle::Monospace.resolve(ui.style()),
            egui::Color32::LIGHT_GRAY,
        );
    }

    fn draw_terrain(&mut self, ui: &mut egui::Ui) {
        let (rect, _) = ui.allocate_exact_size(ui.available_size(), egui::Sense::hover());
        let painter = ui.painter_at(rect);
        let Terrain {
            width: w,
            height: h,
            heights,
            planned,
            actual,
        } = &self.replay.terrain;
        if *w == 0 || *h == 0 {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "no terrain in this replay",
                egui::TextStyle::Body.resolve(ui.style()),
                egui::Color32::GRAY,
            );
            return;
        }
        let cell = rect.width().min(rect.height()) / (*w).max(1) as f32 * self.zoom;
        let origin = rect.center() - egui::vec2(*w as f32 * cell / 2.0, *h as f32 * cell / 2.0);
        // Height shading, relative to the lowest and highest cells.
        let (lo, hi) = heights
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), v| {
                (a.min(*v), b.max(*v))
            });
        let span = (hi - lo).max(1e-6);
        for y in 0..*h {
            for x in 0..*w {
                let v = heights.get(y * w + x).copied().unwrap_or(lo);
                let shade = (v - lo) / span;
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
        // Planned (teal) vs actual (amber) paths, cell centres.
        let at = |c: &(usize, usize)| {
            origin + egui::vec2((c.0 as f32 + 0.5) * cell, (c.1 as f32 + 0.5) * cell)
        };
        for pair in planned.windows(2) {
            painter.line_segment(
                [at(&pair[0]), at(&pair[1])],
                egui::Stroke::new(1.2_f32, egui::Color32::from_rgb(90, 220, 220)),
            );
        }
        for pair in actual.windows(2) {
            painter.line_segment(
                [at(&pair[0]), at(&pair[1])],
                egui::Stroke::new(2.2_f32, egui::Color32::from_rgb(240, 180, 60)),
            );
        }
        painter.text(
            rect.left_bottom() + egui::vec2(8.0, -8.0),
            egui::Align2::LEFT_BOTTOM,
            format!("{w}×{h} cells · planned (teal) vs actual (amber) · heights {lo:.2}…{hi:.2} m"),
            egui::TextStyle::Small.resolve(ui.style()),
            egui::Color32::GRAY,
        );
    }

    /// Light-time cues for the Mars rover downlink: when a command sent at
    /// `send_min` arrives and when its result is known, read at `now_min`.
    fn draw_link(&mut self, ui: &mut egui::Ui) {
        let profile = profile::mars_rover();
        let one_way_us = (profile.link.one_way_delay_s * 1e6) as u64;
        let send_us = (f64::from(self.send_min) * 60e6) as u64;
        let now_us = (f64::from(self.now_min) * 60e6) as u64;
        let timing = command_timing(send_us, one_way_us);
        ui.heading(profile.summary);
        ui.label(format!(
            "one-way light time {:.0} s; the operator clock is in minutes",
            profile.link.one_way_delay_s
        ));
        ui.add(egui::Slider::new(&mut self.send_min, 0.0..=240.0).text("command sent at (min)"));
        ui.add(egui::Slider::new(&mut self.now_min, 0.0..=240.0).text("readout at (min)"));
        ui.separator();
        ui.monospace(format!(
            "arrives at  {:>8.2} min\nknown at    {:>8.2} min",
            timing.arrives_us as f64 / 60e6,
            timing.result_known_us as f64 / 60e6,
        ));
        ui.label(cue_text(&timing, now_us));
    }

    fn draw_budgets(&mut self, ui: &mut egui::Ui) {
        let power: Vec<f32> = self.replay.budgets.iter().map(|b| b.power_w).collect();
        let soc: Vec<f32> = self.replay.budgets.iter().map(|b| b.soc_pct).collect();
        let temp: Vec<f32> = self.replay.budgets.iter().map(|b| b.temp_c).collect();
        let cursor = self.frame;
        egui::ScrollArea::vertical().show(ui, |ui| {
            trace(
                ui,
                "Solar harvest (W)",
                &power,
                cursor,
                egui::Color32::from_rgb(240, 200, 80),
            );
            trace(
                ui,
                "Battery SoC (%)",
                &soc,
                cursor,
                egui::Color32::from_rgb(120, 220, 140),
            );
            trace(
                ui,
                "Battery temp (°C)",
                &temp,
                cursor,
                egui::Color32::from_rgb(230, 120, 100),
            );
        });
    }
}

/// One budget trace with a vertical playback cursor at `cursor`.
fn trace(ui: &mut egui::Ui, label: &str, data: &[f32], cursor: usize, color: egui::Color32) {
    ui.label(label);
    let (rect, _) =
        ui.allocate_exact_size(egui::vec2(ui.available_width(), 90.0), egui::Sense::hover());
    let painter = ui.painter_at(rect);
    painter.rect_filled(rect, 2.0, egui::Color32::from_black_alpha(60));
    if data.len() < 2 {
        return;
    }
    let (min, max) = data
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(a, b), v| {
            (a.min(*v), b.max(*v))
        });
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
    let x =
        rect.left() + rect.width() * cursor.min(data.len() - 1) as f32 / (data.len() - 1) as f32;
    painter.line_segment(
        [egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())],
        egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
    );
    painter.text(
        rect.left_top() + egui::vec2(4.0, 2.0),
        egui::Align2::LEFT_TOP,
        format!("{:.2}", data.get(cursor).copied().unwrap_or_default()),
        egui::TextStyle::Small.resolve(ui.style()),
        egui::Color32::LIGHT_GRAY,
    );
}

/// Rotation matrix (row-major) of a unit quaternion `[w, x, y, z]`.
fn quat_to_matrix(q: [f32; 4]) -> [[f32; 3]; 3] {
    let [w, x, y, z] = q;
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
        ],
        [
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
        ],
        [
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn mat_vec(m: &[[f32; 3]; 3], v: [f32; 3]) -> [f32; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}
