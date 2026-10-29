#![warn(missing_docs)]
//! `tpt-t-space-teleop` — the space domain adapter (spec §5.2 / bridge
//! spec §6, "tpt-teleop-space → tpt-teleop Integration").
//!
//! The only crate that speaks DTI to the core teleop system, adapted for
//! the deep-space link:
//!
//! * the [`SpaceTeleopAdapter`] implements
//!   [`tpt_t_domain_bridge::dti::DomainTeleopInterface`], translating
//!   normalized [`ControlCommand`]s into body commands with per-axis slew
//!   limiting and a blend-ramped autonomy↔teleop handover;
//! * [`display`] — predictive display: the operator station sees the
//!   asset *extrapolated to now*, masking the 4–24 light-minute delay;
//! * [`sequencing`] — uploaded command sequences ("drive 5 m, turn 30°,
//!   take a photo") execute autonomously with hazard-avoidance checks
//!   from the `tpt-t-space-predictive` planner and per-step rollback;
//! * [`bandwidth`] — telemetry delta+RLE coding and image
//!   subsample/quantize coding for the deep-space bandwidth budget;
//! * [`transport`] — the DTN wiring: uplinked sequences arrive as
//!   custody-transferred bundles, downlink telemetry rides the
//!   hash-verified log.

pub mod bandwidth;
pub mod display;
pub mod lighttime;
pub mod lint;
pub mod sequencing;
pub mod transport;

use core::cell::Cell;

use tpt_t_domain_bridge::dti::{DomainTeleopInterface, DtiError};
use tpt_t_domain_bridge::safety::{SafetyState, SafetyStateMachine};
use tpt_t_domain_bridge::wire::{ControlCommand, DomainState, OperatorId, SensorFeed};
use tpt_t_space_core::wire::BodyCmd6;

/// Slew limits per axis (fraction/second at full rate): tx, ty, tz, rx,
/// ry, rz — the rover drives gently even under an excited operator.
pub const SLEW_LIMITS: [f32; 6] = [0.6, 0.6, 0.6, 1.5, 1.5, 1.0];

/// Handover ramp time, seconds (the blend autonomy↔teleop).
pub const BLEND_TIME_S: f32 = 2.0;

/// The adapter state. Not `Sync` — it lives on the teleop session's
/// loop thread; shared outputs go through the rings the caller wires
/// around it.
pub struct SpaceTeleopAdapter {
    machine: SafetyStateMachine,
    operator: Option<OperatorId>,
    blend: f32,
    /// Slew-limited output actually emitted.
    slewed: BodyCmd6,
    last_cmd_us: u64,
    /// Hazards flagged by the last sequencing hazard check (surfaced in
    /// [`DomainState::fault_bits`] bit 0).
    pub hazard_flag: Cell<bool>,
    pose: DomainPoseView,
}

/// The asset's view the adapter reports upward.
#[derive(Debug, Clone, Copy, Default)]
pub struct DomainPoseView {
    /// Position, metres (site/inertial frame per the session).
    pub xyz_m: [f64; 3],
    /// Heading, degrees.
    pub heading_deg: f32,
    /// Ground speed, m/s.
    pub speed_mps: f32,
    /// Battery SoC, %.
    pub battery_soc_pct: f32,
}

impl SpaceTeleopAdapter {
    /// New adapter, autonomous, blend zero.
    #[must_use]
    pub fn new() -> Self {
        Self {
            machine: SafetyStateMachine::default(),
            operator: None,
            blend: 0.0,
            slewed: BodyCmd6::default(),
            last_cmd_us: 0,
            hazard_flag: Cell::new(false),
            pose: DomainPoseView::default(),
        }
    }

    /// Update the vehicle view reported via [`DomainTeleopInterface::get_domain_state`].
    pub fn set_vehicle_view(&mut self, v: DomainPoseView) {
        self.pose = v;
    }

    /// Current teleop blend [0 = autonomy, 1 = teleop].
    #[must_use]
    pub fn blend(&self) -> f32 {
        self.blend
    }

    /// Universal safety machine state.
    #[must_use]
    pub fn safety_state(&self) -> SafetyState {
        self.machine.state()
    }

    /// The slew-limited body command produced by the last
    /// [`DomainTeleopInterface::on_control_command`].
    #[must_use]
    pub fn last_body_cmd(&self) -> BodyCmd6 {
        self.slewed
    }

    /// Drive the blend ramp: call once per loop tick with `teleop_active`
    /// from the safety machine and the loop period.
    pub fn tick_blend(&mut self, dt_s: f32) {
        let target = match self.machine.state() {
            SafetyState::TeleopActive => 1.0,
            SafetyState::ReturningToAutonomy => 0.0,
            SafetyState::EmergencyStop => 0.0,
            _ => self.blend,
        };
        let rate = (dt_s / BLEND_TIME_S).clamp(0.0, 1.0);
        self.blend += (target - self.blend).clamp(-rate, rate);
    }
}

impl Default for SpaceTeleopAdapter {
    fn default() -> Self {
        Self::new()
    }
}

impl DomainTeleopInterface for SpaceTeleopAdapter {
    fn on_teleop_engage(&mut self, operator_id: OperatorId) -> Result<(), DtiError> {
        if self.hazard_flag.get() {
            return Err(DtiError::Unavailable); // hazard freeze: no takeover
        }
        self.machine
            .dispatch(tpt_t_domain_bridge::safety::SafetyEvent::TeleopEngaged)
            .map_err(|_| DtiError::InvalidState)?;
        self.operator = Some(operator_id);
        Ok(())
    }

    fn on_teleop_disengage(&mut self) -> Result<(), DtiError> {
        self.machine
            .dispatch(tpt_t_domain_bridge::safety::SafetyEvent::TeleopReleased)
            .map_err(|_| DtiError::InvalidState)?;
        self.operator = None;
        Ok(())
    }

    fn on_control_command(&mut self, cmd: ControlCommand) -> Result<(), DtiError> {
        if self.operator.is_none() {
            return Err(DtiError::InvalidState);
        }
        // E-stop latches through the safety machine; output zeroes.
        if cmd.buttons.e_stop() {
            let _ = self
                .machine
                .dispatch(tpt_t_domain_bridge::safety::SafetyEvent::EmergencyStop);
            self.slewed = BodyCmd6::default();
            self.last_cmd_us = cmd.timestamp_us;
            return Ok(());
        }
        if self.machine.stopped() {
            return Err(DtiError::InvalidState);
        }
        // Translate axes: primary[0..3] → translation, primary[4..6] →
        // rotation (a driving+gazing convention; documented in the
        // operator manual, enforced here).
        let target = BodyCmd6 {
            tx: cmd.primary_input.axes[0],
            ty: cmd.primary_input.axes[1],
            tz: cmd.primary_input.axes[2],
            rx: cmd.primary_input.axes[4],
            ry: cmd.primary_input.axes[5],
            rz: cmd.secondary_input.axes[0],
        }
        .clamped();
        // Slew-limit each axis from the last emitted command. The first
        // command after engage has no timestamp reference; it assumes the
        // session's nominal 10 Hz period (a gentle first step).
        let dt = if self.last_cmd_us == 0 {
            0.1
        } else {
            ((cmd.timestamp_us.saturating_sub(self.last_cmd_us)) as f32 / 1.0e6).clamp(0.001, 1.0)
        };
        let axes = |c: &BodyCmd6| [c.tx, c.ty, c.tz, c.rx, c.ry, c.rz];
        let last = axes(&self.slewed);
        let want = axes(&target);
        let mut out = [0.0f32; 6];
        for i in 0..6 {
            let max_step = SLEW_LIMITS[i] * dt;
            out[i] = self.blend * (last[i] + (want[i] - last[i]).clamp(-max_step, max_step));
        }
        self.slewed = BodyCmd6 {
            tx: out[0],
            ty: out[1],
            tz: out[2],
            rx: out[3],
            ry: out[4],
            rz: out[5],
        };
        self.last_cmd_us = cmd.timestamp_us;
        Ok(())
    }

    fn get_domain_state(&self) -> DomainState {
        DomainState {
            timestamp_us: 0,
            lat_deg: 0.0,
            lon_deg: 0.0,
            alt_m: self.pose.xyz_m[2] as f32,
            yaw_deg: self.pose.heading_deg,
            speed_m_s: self.pose.speed_mps,
            safety_state: self.machine.state().as_u8(),
            fault_bits: u64::from(self.hazard_flag.get()),
            telemetry: tpt_t_domain_bridge::wire::DomainTelemetry {
                battery_soc_pct: self.pose.battery_soc_pct,
                ..Default::default()
            },
        }
    }

    fn get_sensor_feed(&mut self) -> SensorFeed {
        SensorFeed::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_t_domain_bridge::wire::{Button, ButtonState, InputState, OperatorId};

    fn cmd(axes: [f32; 8], us: u64) -> ControlCommand {
        let mut primary = InputState::default();
        primary.axes.copy_from_slice(&axes);
        ControlCommand {
            timestamp_us: us,
            operator_id: OperatorId([1; 16]),
            primary_input: primary,
            secondary_input: InputState::default(),
            buttons: ButtonState::default(),
            haptic_feedback: Default::default(),
            seq: 1,
        }
    }

    #[test]
    fn full_session_lifecycle_with_slew() {
        let mut a = SpaceTeleopAdapter::new();
        // Commands before engage are rejected.
        assert_eq!(
            a.on_control_command(cmd([0.5; 8], 1_000)),
            Err(DtiError::InvalidState)
        );
        a.on_teleop_engage(OperatorId([9; 16])).unwrap();
        assert_eq!(a.safety_state(), SafetyState::TeleopActive);

        // Blend ramps in over ticks.
        for _ in 0..25 {
            a.tick_blend(0.1);
        }
        assert!((a.blend() - 1.0).abs() < 0.01, "blend {}", a.blend());

        // Full-forward command slews up from zero, not a step.
        a.on_control_command(cmd([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1_100_000))
            .unwrap();
        let c1 = a.last_body_cmd();
        assert!(c1.tx > 0.0 && c1.tx < 0.1, "slew-limited: {}", c1.tx);
        for k in 0..30u64 {
            a.on_control_command(cmd(
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                1_200_000 + k * 100_000,
            ))
            .unwrap();
        }
        assert!(a.last_body_cmd().tx > 0.9, "reaches the demand");

        // Disengage ramps the blend back to autonomy.
        a.on_teleop_disengage().unwrap();
        for _ in 0..25 {
            a.tick_blend(0.1);
        }
        assert!(a.blend() < 0.01);
    }

    #[test]
    fn e_stop_latches_and_zeroes_output() {
        let mut a = SpaceTeleopAdapter::new();
        a.on_teleop_engage(OperatorId([2; 16])).unwrap();
        for _ in 0..25 {
            a.tick_blend(0.1);
        }
        a.on_control_command(cmd([0.8; 8], 1_000)).unwrap();
        assert!(a.last_body_cmd().tx != 0.0);

        let mut emergency = cmd([0.8; 8], 1_100_000);
        emergency.buttons.set(Button::EStop);
        a.on_control_command(emergency).unwrap();
        assert!(a.machine.stopped());
        assert!(a.last_body_cmd().is_zero(), "estop zeroes output");
        // Post-estop commands are refused.
        assert_eq!(
            a.on_control_command(cmd([0.8; 8], 1_200_000)),
            Err(DtiError::InvalidState)
        );
    }

    #[test]
    fn hazard_freeze_blocks_takeover() {
        let mut a = SpaceTeleopAdapter::new();
        a.hazard_flag.set(true);
        assert_eq!(
            a.on_teleop_engage(OperatorId([3; 16])),
            Err(DtiError::Unavailable)
        );
    }

    #[test]
    fn domain_state_reports_safety_and_hazards() {
        let mut a = SpaceTeleopAdapter::new();
        a.set_vehicle_view(DomainPoseView {
            xyz_m: [10.0, 20.0, -3.5],
            heading_deg: 122.0,
            speed_mps: 0.4,
            battery_soc_pct: 61.0,
        });
        let ds = a.get_domain_state();
        assert_eq!(ds.safety_state, SafetyState::Autonomous.as_u8());
        assert_eq!(ds.yaw_deg, 122.0);
        assert_eq!(ds.telemetry.battery_soc_pct, 61.0);
    }
}
