//! Canned scenarios shared by the integration test, the `tpts` CLI, and the
//! visualizer.
//!
//! * [`leo_survey`] — a J2/drag-perturbed LEO orbit with
//!   zenith-pointing attitude, eclipse-driven power and battery thermal
//!   traces. Produces the orbit, attitude, and budget panes.
//! * [`mars_sample`] — the spec §6 Mars Sample Collection flow across the
//!   workspace. Produces a timeline, measured budgets, and the terrain
//!   panes; the integration test asserts on the same run.
//! * [`demo_replay`] — both of the above merged, for the visualizer's
//!   default view.

use std::time::Instant;

use tpt_t_space_core::blackbox::{BlackBox, Event as BoxEvent, restore_each};
use tpt_t_space_core::wire::SpaceEvent;

/// Bus kind code the Mars flow records when the uplink gains custody.
pub const BUS_UPLINK_CUSTODY: u16 = 0x0001;
/// NVRAM bytes reserved for the black box in the Mars flow.
const NVRAM_BYTES: usize = 512;
use tpt_t_space_core::machine::{MissionState, StateMachine};
use tpt_t_space_core::safe_mode::{SafeModeActions, SafeModeCause, SafeModeController};
use tpt_t_space_dtn::bundle::{Bundle, BundleStore, CustodyState, EndpointId, Priority};
use tpt_t_space_dtn::link::{DtnAgent, TelemetryLog};
use tpt_t_space_power::{Battery, PowerSubsystem, SolarArray};
use tpt_t_space_predictive::dem::{DemConfig, DemGrid};
use tpt_t_space_predictive::planner::{Pose, RoverLimits, replan};
use tpt_t_space_radiation::seu::{SeuBook, SeuKind};
use tpt_t_space_radiation::tmr::TmrVoter;
use tpt_t_space_thermal::ThermalSubsystem;

use crate::clock::SimClock;
use crate::dtn_link::LinkProfile;
use crate::orbit::{Orbit, Perturbations, R_EARTH, Spacecraft};
use crate::replay::{Attitude, Budget, Replay, Terrain};

/// Terrain grid edge, cells.
const N: usize = 96;
/// Half the grid, cells (the planner's origin is the grid centre).
const HALF: f32 = N as f32 / 2.0;
const HOUR_S: f64 = 3_600.0;
/// Mean obliquity of the ecliptic, radians.
const OBLIQUITY: f64 = 0.409_092_6;
/// Seconds in a sidereal-ish year, for the sun's apparent motion.
const YEAR_S: f64 = 365.25 * 86_400.0;

/// A scenario step failed; the message names the step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScenarioError(pub &'static str);

impl std::fmt::Display for ScenarioError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "scenario step failed: {}", self.0)
    }
}

impl std::error::Error for ScenarioError {}

/// One timeline entry.
#[derive(Debug, Clone, PartialEq)]
pub struct Event {
    /// Simulation time, seconds.
    pub t_s: f64,
    /// Mission phase name.
    pub phase: &'static str,
    /// What happened.
    pub detail: String,
}

fn note(events: &mut Vec<Event>, t_s: f64, phase: &'static str, detail: impl Into<String>) {
    events.push(Event {
        t_s,
        phase,
        detail: detail.into(),
    });
}

/// Unit sun direction in ECI at time `t_s` (circular ecliptic motion).
fn sun_eci(t_s: f64) -> [f64; 3] {
    let lambda = std::f64::consts::TAU * t_s / YEAR_S;
    [
        lambda.cos(),
        lambda.sin() * OBLIQUITY.cos(),
        lambda.sin() * OBLIQUITY.sin(),
    ]
}

/// Whether the spacecraft at `r` (ECI, metres) is in Earth's cylindrical
/// shadow for sun direction `s` (unit).
fn shadowed(r: &[f64; 3], s: &[f64; 3]) -> bool {
    let along = r[0] * s[0] + r[1] * s[1] + r[2] * s[2];
    if along >= 0.0 {
        return false;
    }
    let perp2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] - along * along;
    perp2 < R_EARTH * R_EARTH
}

fn dot(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let n = dot(&v, &v).sqrt().max(1e-12);
    [v[0] / n, v[1] / n, v[2] / n]
}

/// Attitude quaternion `[w, x, y, z]` from a body→ECI rotation whose
/// columns are the body axes `x_b`, `y_b`, `z_b` (Shepperd's method).
fn quat_from_axes(x_b: &[f64; 3], y_b: &[f64; 3], z_b: &[f64; 3]) -> [f32; 4] {
    // R[row][col]: column j is body axis j expressed in ECI.
    let r = [
        [x_b[0], y_b[0], z_b[0]],
        [x_b[1], y_b[1], z_b[1]],
        [x_b[2], y_b[2], z_b[2]],
    ];
    let trace = r[0][0] + r[1][1] + r[2][2];
    let (w, x, y, z) = if trace > 0.0 {
        let s = (trace + 1.0).sqrt() * 2.0;
        (
            0.25 * s,
            (r[2][1] - r[1][2]) / s,
            (r[0][2] - r[2][0]) / s,
            (r[1][0] - r[0][1]) / s,
        )
    } else if r[0][0] > r[1][1] && r[0][0] > r[2][2] {
        let s = (1.0 + r[0][0] - r[1][1] - r[2][2]).sqrt() * 2.0;
        (
            (r[2][1] - r[1][2]) / s,
            0.25 * s,
            (r[0][1] + r[1][0]) / s,
            (r[0][2] + r[2][0]) / s,
        )
    } else if r[1][1] > r[2][2] {
        let s = (1.0 + r[1][1] - r[0][0] - r[2][2]).sqrt() * 2.0;
        (
            (r[0][2] - r[2][0]) / s,
            (r[0][1] + r[1][0]) / s,
            0.25 * s,
            (r[1][2] + r[2][1]) / s,
        )
    } else {
        let s = (1.0 + r[2][2] - r[0][0] - r[1][1]).sqrt() * 2.0;
        (
            (r[1][0] - r[0][1]) / s,
            (r[0][2] + r[2][0]) / s,
            (r[1][2] + r[2][1]) / s,
            0.25 * s,
        )
    };
    // Canonical sign: keep w non-negative.
    let sign = if w < 0.0 { -1.0 } else { 1.0 };
    [
        (sign * w) as f32,
        (sign * x) as f32,
        (sign * y) as f32,
        (sign * z) as f32,
    ]
}

/// Simple first-order battery thermal model: relaxes toward an
/// eclipse-dependent equilibrium (heaters keep the floor above survival).
fn relax_temp(temp_c: f32, target_c: f32, dt_s: f32, tau_s: f32) -> f32 {
    temp_c + (target_c - temp_c) * (dt_s / tau_s).min(1.0)
}

/// The LEO survey: propagate `orbits` revolutions at `step_s` sampling and
/// produce the orbit, attitude, and budget panes.
///
/// Attitude is zenith-pointing (body +z along the radius vector, +x along
/// the velocity), so the sun vector in the body frame sweeps through the
/// orbit and the array gimbal tracks it. Eclipses come from the cylindrical
/// shadow model; the power subsystem sheds loads and reports the survival
/// trigger, which the caller can read from the timeline.
#[must_use]
pub fn leo_survey(orbits: u32, step_s: f64) -> (Replay, Vec<Event>) {
    let alt_m = 550.0e3;
    let r0 = R_EARTH + alt_m;
    let v0 = (crate::orbit::MU_EARTH / r0).sqrt();
    let inc = 51.6_f64.to_radians();
    let mut state = [r0, 0.0, 0.0, 0.0, v0 * inc.cos(), v0 * inc.sin()];
    let mut prop = Orbit::new(Spacecraft::cubesat12(), Perturbations::full());
    let period_s = std::f64::consts::TAU * (r0.powi(3) / crate::orbit::MU_EARTH).sqrt();
    let steps = (f64::from(orbits) * period_s / step_s).ceil() as usize;

    let mut power = PowerSubsystem::new(
        SolarArray {
            rated_w: 350.0,
            travel: (
                -core::f32::consts::PI,
                core::f32::consts::PI,
                -core::f32::consts::FRAC_PI_2,
                core::f32::consts::FRAC_PI_2,
            ),
            steps: 24,
        },
        Battery::new(120.0),
    );
    let mut thermal = ThermalSubsystem::new();
    let mut battery_c = 15.0_f32;
    let mut events = Vec::new();
    let mut replay = Replay::default();
    let mut in_eclipse = false;
    let mut survival_latched = false;

    for k in 0..steps {
        let t_s = k as f64 * step_s;
        let sun = sun_eci(t_s);
        let r = [state[0], state[1], state[2]];
        let v = [state[3], state[4], state[5]];
        let r_hat = normalize(r);
        let x_hat = normalize([
            v[0] - dot(&v, &r_hat) * r_hat[0],
            v[1] - dot(&v, &r_hat) * r_hat[1],
            v[2] - dot(&v, &r_hat) * r_hat[2],
        ]);
        let y_hat = cross(&r_hat, &x_hat);
        let sun_body = [
            dot(&x_hat, &sun) as f32,
            dot(&y_hat, &sun) as f32,
            dot(&r_hat, &sun) as f32,
        ];
        let q = quat_from_axes(&x_hat, &y_hat, &r_hat);
        replay.orbit.push(r);
        replay.attitude.push(Attitude { q, sun_body });

        let eclipse = shadowed(&r, &sun);
        if eclipse != in_eclipse {
            let label = if eclipse {
                "eclipse entry"
            } else {
                "eclipse exit"
            };
            note(&mut events, t_s, "eclipse", label);
            in_eclipse = eclipse;
        }
        power.track_sun(sun_body);
        // The survival trigger stays asserted for as long as the output is
        // below the floor; log the onset once, not every sample.
        match power.step(t_s as u64, step_s as f32, eclipse) {
            Some(SafeModeCause::PowerSurvivalThreshold) if !survival_latched => {
                survival_latched = true;
                note(
                    &mut events,
                    t_s,
                    "power",
                    "survival threshold (array output below floor)",
                );
            }
            Some(_) => {}
            None => survival_latched = false,
        }
        let target_c = if eclipse {
            -12.0
        } else {
            18.0 + 0.02 * power.output_w()
        };
        battery_c = relax_temp(battery_c, target_c, step_s as f32, 900.0);
        if let Some(cause) = thermal.step_battery_temp(t_s as u64, battery_c) {
            note(&mut events, t_s, "thermal", format!("{cause:?}"));
        }
        replay.budgets.push(Budget {
            power_w: power.output_w(),
            soc_pct: power.battery().soc_pct(),
            temp_c: battery_c,
        });

        prop.step_rk4(&mut state, step_s);
    }
    note(
        &mut events,
        steps as f64 * step_s,
        "survey",
        format!(
            "{orbits} orbit(s) at {:.0} km, {steps} samples",
            alt_m / 1e3
        ),
    );
    (replay, events)
}

/// The rover terrain of the §6 scenario: benign undulation with the 25°
/// slope band ahead across the corridor.
#[must_use]
pub fn terrain() -> DemGrid<N, N> {
    /// Ingested half-span, in 0.25 m samples (fills the grid, minus a margin).
    const SPAN: i32 = N as i32 / 2 - 4;
    let mut dem = DemGrid::new(DemConfig::rover());
    let hazard = 25.0_f32.to_radians();
    for gy in -SPAN..SPAN {
        for gx in -SPAN..SPAN {
            let x = gx as f32 * 0.25;
            let y = gy as f32 * 0.25;
            let h = if (2.0..6.0).contains(&x) && y.abs() < 2.0 {
                hazard.tan() * (x - 2.0)
            } else {
                0.05 * (x * 0.3).sin()
            };
            dem.ingest_point(x, y, h);
        }
    }
    dem.finish();
    dem
}

/// Map a ground position to a DEM grid cell, clamped to the grid.
fn cell_of(x_m: f32, y_m: f32, cell_m: f32) -> (usize, usize) {
    let gx = (x_m / cell_m + HALF).floor().clamp(0.0, (N - 1) as f32);
    let gy = (y_m / cell_m + HALF).floor().clamp(0.0, (N - 1) as f32);
    (gx as usize, gy as usize)
}

/// The platform the Safe Mode controller drives (records the actions).
#[derive(Default)]
struct RoverPlatform {
    payloads_shed: bool,
    sun_pointing: bool,
    beacon_bps: u32,
}

impl SafeModeActions for RoverPlatform {
    fn shed_payloads(&mut self) {
        self.payloads_shed = true;
    }
    fn slew_to_sun_point(&mut self) {
        self.sun_pointing = true;
    }
    fn start_beacon(&mut self, rate_bps: u32) {
        self.beacon_bps = rate_bps;
    }
}

/// Result of the §6 Mars Sample Collection run.
#[derive(Debug, Clone)]
pub struct MarsRun {
    /// Mission timeline, in order.
    pub events: Vec<Event>,
    /// Length of the blackout the rover sat through, seconds.
    pub blackout_s: f64,
    /// Wall time of the first (detour) replan, milliseconds.
    pub replan_ms: f64,
    /// Sim time spent on the traverse arc, seconds.
    pub detour_arc_s: f64,
    /// Largest lateral excursion from the centreline, metres.
    pub max_clearance_m: f32,
    /// Whether the rover ever entered a hazard cell (must be `false`).
    pub hazard_entered: bool,
    /// Whether the rover halted during the traverse (must be `false`).
    pub halted: bool,
    /// Core masked by the TMR voter, if any.
    pub seu_masked_core: Option<u8>,
    /// Cores flagged for watchdog reset (bit per core).
    pub reset_mask: u8,
    /// Whether the voted quaternion kept the majority value.
    pub vote_intact: bool,
    /// Science steps verified by sensors.
    pub science_steps: usize,
    /// Whether Safe Mode was entered atomically.
    pub safe_mode_entered: bool,
    /// Safe Mode actions taken.
    pub safe_mode_payloads_shed: bool,
    /// Safe Mode slew to sun point.
    pub safe_mode_sun_pointing: bool,
    /// Beacon rate during Safe Mode, bits per second.
    pub safe_mode_beacon_bps: u32,
    /// Bundles queued when the link returned.
    pub queued_bundles: u32,
    /// Bundles delivered after reconnection.
    pub delivered_bundles: u32,
    /// Egress flits drained.
    pub flits: u32,
    /// Telemetry chunks logged.
    pub telemetry_chunks: u64,
    /// Final sim time, days.
    pub sim_days: f64,
    /// The encoded Safe Mode black-box image (`tpts blackbox` decodes it).
    pub blackbox: Vec<u8>,
    /// Records restored from the NVRAM region after the flow.
    pub blackbox_restored: usize,
    /// The rover's pose trace as a replay (terrain + planned/actual paths).
    pub replay: Replay,
}

/// Run the spec §6 Mars Sample Collection flow.
///
/// # Errors
/// Returns [`ScenarioError`] naming the step that could not complete
/// (e.g. a DTN custody ack refused).
pub fn mars_sample() -> Result<MarsRun, ScenarioError> {
    let dem = terrain();
    let cfg = DemConfig::rover();
    let mut clock = SimClock::new(1.0);
    let link = LinkProfile::mars_surface();
    let mut mission = StateMachine::new(MissionState::Cruise);
    let mut recorder = BlackBox::<16>::new();
    let mut events = Vec::new();

    // 1. The command: Earth uplinks "Proceed to Waypoint Alpha and collect
    //    sample" via DTN; the rover acknowledges custody.
    let mut agent = DtnAgent::new(BundleStore::new(16, 1024));
    let command = b"proceed-to-waypoint-alpha;collect-sample";
    let seq_sector = agent
        .store()
        .store_bundle(
            &Bundle::new(
                1,
                EndpointId::SURFACE_ASSET,
                0,
                command.len(),
                Priority::Expedited,
                1,
            ),
            command,
        )
        .map_err(|_| ScenarioError("command bundle store"))?;
    agent
        .store()
        .accept_custody(seq_sector)
        .map_err(|_| ScenarioError("custody ack"))?;
    let held = agent
        .store()
        .bundle_meta(seq_sector)
        .map_err(|_| ScenarioError("command bundle metadata"))?
        .custody
        == CustodyState::Accepted;
    if !held {
        return Err(ScenarioError(
            "rover does not hold custody of the uplinked command",
        ));
    }
    note(
        &mut events,
        0.0,
        "uplink",
        "command accepted under DTN custody",
    );
    recorder.record_bus(
        0,
        &SpaceEvent {
            kind: BUS_UPLINK_CUSTODY,
            seq: 1,
            timestamp_us: 0,
            payload: 0,
        },
    );
    mission
        .transition(MissionState::Approach)
        .map_err(|_| ScenarioError("cruise to approach"))?;
    recorder.record(
        0,
        BoxEvent::Transition {
            from: MissionState::Cruise,
            to: MissionState::Approach,
        },
    );
    mission
        .transition(MissionState::Operations)
        .map_err(|_| ScenarioError("approach to operations"))?;
    recorder.record(
        0,
        BoxEvent::Transition {
            from: MissionState::Approach,
            to: MissionState::Operations,
        },
    );

    // 2. The blackout: Mars rotates; the rover is alone for hours.
    let mut blackout_s = 0.0;
    while !link.link_up(blackout_s) && blackout_s < 5.0 * HOUR_S {
        blackout_s += 60.0;
    }
    clock.step_sim(blackout_s);
    note(
        &mut events,
        clock.sim_s(),
        "blackout",
        format!("link down for {:.2} h", blackout_s / HOUR_S),
    );

    // Telemetry generated during the blackout queues in NVRAM.
    let mut t_log = TelemetryLog::<64>::new();
    for chunk in 0..8u32 {
        let bytes = [(chunk * 11) as u8; 48];
        agent
            .store()
            .store_bundle(
                &Bundle::new(
                    100 + u64::from(chunk),
                    EndpointId::EARTH_OPS,
                    clock.sim_s() as u64,
                    48,
                    Priority::Normal,
                    2,
                ),
                &bytes,
            )
            .map_err(|_| ScenarioError("blackout telemetry store"))?;
        t_log.push(&bytes);
    }
    let queued_bundles = agent.pending();
    note(
        &mut events,
        clock.sim_s(),
        "telemetry",
        format!("{queued_bundles} bundles queued in NVRAM"),
    );

    // 3+4. Autonomous traverse: the DEM detects the 25° slope; the detour
    //      replans with no ground input.
    let limits = RoverLimits::flight_rules();
    let mut pose = Pose {
        x_m: 0.0,
        y_m: 0.0,
        yaw_rad: 0.0,
    };
    let start = Instant::now();
    let first = replan(&dem, &cfg, &limits, &pose);
    let replan_ms = start.elapsed().as_secs_f64() * 1000.0;
    let (c0, speed) = match &first {
        Ok(p) => (p.best.curvature, p.best.speed),
        Err(_) => return Err(ScenarioError("initial replan")),
    };
    note(
        &mut events,
        clock.sim_s(),
        "traverse",
        format!("hazard detected; detour κ={c0:.3} 1/m planned in {replan_ms:.2} ms"),
    );

    // Predicted (planned) arc at the detour curvature, for the terrain pane.
    let dt = 0.1_f32;
    let step_m = speed * dt;
    let mut planned_pose = pose;
    let mut planned = Vec::new();
    for _ in 0..400 {
        if planned_pose.x_m >= 8.0 {
            break;
        }
        planned_pose.yaw_rad += c0 * step_m;
        planned_pose.x_m += step_m * planned_pose.yaw_rad.cos();
        planned_pose.y_m += step_m * planned_pose.yaw_rad.sin();
        planned.push(cell_of(planned_pose.x_m, planned_pose.y_m, cfg.cell_m));
    }

    let mut actual = vec![cell_of(pose.x_m, pose.y_m, cfg.cell_m)];
    let mut arc_s = 0.0_f64;
    let mut max_clearance_m = 0.0_f32;
    let mut hazard_entered = false;
    let mut halted = false;
    while pose.x_m < 8.0 && arc_s < 300.0 {
        let (curvature, planned_speed) = match replan(&dem, &cfg, &limits, &pose) {
            Ok(p) => (p.best.curvature, p.best.speed),
            Err(_) => (0.0, 0.0),
        };
        if planned_speed <= 0.0 {
            halted = true;
            break;
        }
        // Unicycle integration under the planned curvature (10 Hz loop).
        pose.yaw_rad += curvature * planned_speed * dt;
        pose.x_m += planned_speed * dt * pose.yaw_rad.cos();
        pose.y_m += planned_speed * dt * pose.yaw_rad.sin();
        if dem.sample(pose.x_m, pose.y_m, &cfg).hazard {
            hazard_entered = true;
            break;
        }
        actual.push(cell_of(pose.x_m, pose.y_m, cfg.cell_m));
        max_clearance_m = max_clearance_m.max(pose.y_m.abs());
        arc_s += f64::from(dt);
        clock.step_sim(f64::from(dt));
    }
    note(
        &mut events,
        clock.sim_s(),
        "traverse",
        format!("cleared hazard band; max lateral {max_clearance_m:.1} m"),
    );

    // 5. The anomaly: a cosmic ray flips a bit mid-calculation; the TMR
    //    voter masks the corrupted core and flags it.
    let mut voter = TmrVoter::default();
    let mut seu_log = SeuBook::<32>::new();
    let q = core::f32::consts::FRAC_1_SQRT_2;
    let mut cores: [[f32; 4]; 3] = [[q, 0.0, q, 0.0]; 3];
    cores[1][0] = f32::from_bits(cores[1][0].to_bits() ^ (1 << 20));
    let vote = voter.submit(cores);
    seu_log.record(
        clock.sim_s() as u64,
        SeuKind::VoteMasked,
        vote.masked_core.map_or(0, u32::from),
    );
    let reset_mask = voter.take_reset_requests();
    let vote_intact = (vote.value[0] - q).abs() < 1e-3 && (vote.value[2] - q).abs() < 1e-3;
    note(
        &mut events,
        clock.sim_s(),
        "radiation",
        format!(
            "SEU masked on core {:?}; reset mask {reset_mask:#05b}",
            vote.masked_core
        ),
    );

    // 6. Sample collection at Waypoint Alpha: the science chain.
    const CHAIN: [&str; 5] = [
        "deploy-arm",
        "press-drill",
        "extract-core",
        "seal-tube",
        "stow",
    ];
    const TORQUE_NM: [f32; 5] = [6.0, 20.0, 12.0, 6.0, 4.0];
    const T_MIN: [f32; 5] = [0.5, 8.0, 5.0, 2.0, 1.0];
    const T_MAX: [f32; 5] = [12.0, 40.0, 30.0, 15.0, 10.0];
    let mut science_steps = 0;
    for (i, step) in CHAIN.iter().enumerate() {
        let torque = TORQUE_NM[i];
        if torque < T_MIN[i] || torque > T_MAX[i] {
            return Err(ScenarioError("science step torque out of band"));
        }
        science_steps += 1;
        clock.step_sim(90.0);
        t_log.push(&[i as u8; 24]);
        note(
            &mut events,
            clock.sim_s(),
            "science",
            format!("{step} verified ({torque} N·m)"),
        );
    }

    // 7. Safe Mode: a dust storm drops the solar output.
    let mut controller = SafeModeController::new();
    let mut platform = RoverPlatform::default();
    let safe_mode_entered = controller.enter(
        clock.sim_s() as u64,
        SafeModeCause::PowerSurvivalThreshold,
        &mut platform,
    );
    if safe_mode_entered {
        mission
            .transition(MissionState::SafeMode)
            .map_err(|_| ScenarioError("operations to safe mode"))?;
        let now = clock.sim_s() as u32;
        recorder.record(
            now,
            BoxEvent::Transition {
                from: MissionState::Operations,
                to: MissionState::SafeMode,
            },
        );
        recorder.record(
            now,
            BoxEvent::SafeMode(SafeModeCause::PowerSurvivalThreshold),
        );
    }
    clock.step_sim(2.0 * HOUR_S);
    note(
        &mut events,
        clock.sim_s(),
        "safe-mode",
        "payload off, rover halted, low-rate beacon on",
    );

    // 8. Reconnection: the storm clears, the orbiter returns; DTN resumes
    //    and the telemetry log + Safe Mode alert go downlink.
    let alert = b"SAFE_MODE;cause=power-survival;battery=ok";
    agent
        .store()
        .store_bundle(
            &Bundle::new(
                900,
                EndpointId::EARTH_OPS,
                clock.sim_s() as u64,
                alert.len(),
                Priority::Expedited,
                3,
            ),
            alert,
        )
        .map_err(|_| ScenarioError("alert store"))?;
    t_log.push(alert);
    let mut wait_s = 0.0;
    while !link.link_up(clock.sim_s()) && wait_s < link.blackout_period_s {
        clock.step_sim(60.0);
        wait_s += 60.0;
    }
    agent.set_link(true);
    let mut delivered_bundles = 0u32;
    let mut flits = 0u32;
    while let Some(Ok(egress)) = agent.poll() {
        if egress.final_flit {
            agent
                .deliver(egress.sector)
                .map_err(|_| ScenarioError("custody release"))?;
            delivered_bundles += 1;
        }
        flits += 1;
        if flits >= 200_000 {
            return Err(ScenarioError("downlink drain unbounded"));
        }
    }
    note(
        &mut events,
        clock.sim_s(),
        "reconnect",
        format!("{delivered_bundles} bundles in {flits} flits drained"),
    );

    let replay = Replay {
        terrain: Terrain {
            width: N,
            height: N,
            heights: (0..N * N)
                .map(|i| dem.cell(i % N, i / N, &cfg).height_m)
                .collect(),
            planned,
            actual,
        },
        ..Replay::default()
    };

    // Persist the box to an NVRAM region, then restore it, as after a reset.
    let mut nvram = [0xFFu8; NVRAM_BYTES];
    let image_len = recorder
        .encode(&mut nvram)
        .map_err(|_| ScenarioError("black-box image"))?;
    let mut blackbox_restored = 0usize;
    restore_each(&nvram, |_| blackbox_restored += 1)
        .map_err(|_| ScenarioError("black-box restore"))?;
    let blackbox = nvram[..image_len].to_vec();

    Ok(MarsRun {
        events,
        blackbox,
        blackbox_restored,
        blackout_s,
        replan_ms,
        detour_arc_s: arc_s,
        max_clearance_m,
        hazard_entered,
        halted,
        seu_masked_core: vote.masked_core,
        reset_mask,
        vote_intact,
        science_steps,
        safe_mode_entered,
        safe_mode_payloads_shed: platform.payloads_shed,
        safe_mode_sun_pointing: platform.sun_pointing,
        safe_mode_beacon_bps: platform.beacon_bps,
        queued_bundles,
        delivered_bundles,
        flits,
        telemetry_chunks: t_log.total_chunks(),
        sim_days: clock.sim_days(),
        replay,
    })
}

/// The visualizer's default view: the LEO survey merged with the Mars
/// terrain and paths.
pub fn demo_replay() -> Result<Replay, ScenarioError> {
    let (mut replay, _) = leo_survey(2, 30.0);
    replay.terrain = mars_sample()?.replay.terrain;
    Ok(replay)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leo_survey_attitude_is_unit_and_sun_is_unit_in_body_frame() {
        let (replay, _) = leo_survey(1, 60.0);
        assert!(!replay.attitude.is_empty());
        for a in &replay.attitude {
            let qn: f32 = a.q.iter().map(|v| v * v).sum::<f32>().sqrt();
            let sn: f32 = a.sun_body.iter().map(|v| v * v).sum::<f32>().sqrt();
            assert!((qn - 1.0).abs() < 1e-4, "quaternion norm {qn}");
            assert!((sn - 1.0).abs() < 1e-4, "sun vector norm {sn}");
        }
    }

    #[test]
    fn quaternion_rotates_body_axes_into_eci() {
        // Identity axes → identity quaternion.
        let q = quat_from_axes(&[1.0, 0.0, 0.0], &[0.0, 1.0, 0.0], &[0.0, 0.0, 1.0]);
        assert!((q[0] - 1.0).abs() < 1e-6 && q[1..].iter().all(|v| v.abs() < 1e-6));
        // 90° about z: body x → ECI +y.
        let q = quat_from_axes(&[0.0, 1.0, 0.0], &[-1.0, 0.0, 0.0], &[0.0, 0.0, 1.0]);
        let (w, z) = (q[0], q[3]);
        assert!((w - z).abs() < 1e-6 && (w * w - 0.5).abs() < 1e-5, "{q:?}");
    }

    #[test]
    fn leo_survey_sees_eclipses_each_revolution() {
        let (_, events) = leo_survey(2, 30.0);
        let entries = events
            .iter()
            .filter(|e| e.detail == "eclipse entry")
            .count();
        assert!(
            entries >= 2,
            "two revolutions, got {entries} eclipse entries"
        );
    }

    #[test]
    fn leo_survey_budgets_are_physical() {
        let (replay, _) = leo_survey(1, 30.0);
        for b in &replay.budgets {
            assert!(
                (0.0..=350.0).contains(&b.power_w),
                "harvest {} W",
                b.power_w
            );
            assert!((0.0..=100.0).contains(&b.soc_pct), "soc {}", b.soc_pct);
        }
        // Eclipse samples harvest nothing.
        assert!(replay.budgets.iter().any(|b| b.power_w == 0.0));
        assert!(replay.budgets.iter().any(|b| b.power_w > 100.0));
    }

    #[test]
    fn survey_replay_survives_the_file_format() {
        let (replay, _) = leo_survey(1, 120.0);
        let back = Replay::parse(&replay.to_text()).expect("parses");
        assert_eq!(back, replay);
    }

    #[test]
    fn mars_terrain_has_the_hazard_band() {
        let run = mars_sample().expect("flow runs");
        let t = &run.replay.terrain;
        assert_eq!((t.width, t.height), (N, N));
        assert_eq!(t.heights.len(), N * N);
        assert!(!t.planned.is_empty() && !t.actual.is_empty());
        assert!(
            t.heights.iter().any(|h| *h > 1.0),
            "the 25° ramp is present"
        );
    }

    #[test]
    fn mars_blackbox_records_the_safe_mode_episode() {
        let run = mars_sample().expect("flow runs");
        let mut records = Vec::new();
        tpt_t_space_core::blackbox::decode_each(&run.blackbox, |r| records.push(r))
            .expect("the image decodes");
        assert_eq!(
            records.len(),
            5,
            "uplink custody, two approach moves, safe-mode entry and cause"
        );
        assert_eq!(
            run.blackbox_restored, 5,
            "the NVRAM copy restores every record"
        );
        assert_eq!(records[0].event, BoxEvent::Bus(BUS_UPLINK_CUSTODY));
        assert!(matches!(
            records[3].event,
            BoxEvent::Transition {
                to: MissionState::SafeMode,
                ..
            }
        ));
        assert!(matches!(
            records[4].event,
            BoxEvent::SafeMode(SafeModeCause::PowerSurvivalThreshold)
        ));
    }
}
