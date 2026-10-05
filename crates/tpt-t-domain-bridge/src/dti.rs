//! The Domain Teleoperation Interface trait.
//!
//! Every domain repo implements [`DomainTeleopInterface`] in its adapter
//! crate (`tpt-t-space-teleop`, `tpt-t-agri-teleop`, …). The core
//! teleop system is the only caller: it normalizes operator input into
//! [`ControlCommand`]s, applies safety limits *upstream* (including
//! human/AI arbitration), and streams [`SensorFeed`]s back to the operator
//! station.
//!
//! Contract notes (from `bridge spec.txt` §4–5):
//!
//! * [`DomainTeleopInterface::on_teleop_engage`] /
//!   [`DomainTeleopInterface::on_teleop_disengage`] must produce a smooth
//!   handover — the domain blends actuator setpoints so the machine does not
//!   jerk at the autonomy↔teleop boundary.
//! * [`DomainTeleopInterface::on_control_command`] receives *already
//!   arbitrated* commands: one `operator_id` per call, no competing streams.
//! * [`DomainTeleopInterface::get_sensor_feed`] returns metadata referencing
//!   out-of-band payloads (see [`crate::wire::SensorFeed`]); the session's
//!   frame buffer was negotiated at setup, so this call allocates nothing.

use crate::wire::{ControlCommand, DomainState, OperatorId, SensorFeed};

/// Error returned by domain adapter hooks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DtiError {
    /// The hook was called in a state where it cannot run (e.g. `engage`
    /// while a session is already active).
    InvalidState,
    /// The domain rejected the command on safety grounds (geofence, speed
    /// limit, interlock). The command is dropped; the domain remains under
    /// teleop but applies its own limits.
    SafetyReject,
    /// The domain is busy or degraded and cannot accept the transition now.
    Unavailable,
}

impl core::fmt::Display for DtiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let msg = match self {
            DtiError::InvalidState => "hook called in an invalid state",
            DtiError::SafetyReject => "command rejected by domain safety limits",
            DtiError::Unavailable => "domain unavailable or degraded",
        };
        f.write_str(msg)
    }
}

impl std::error::Error for DtiError {}

/// The universal adapter trait: tpt-teleop talks to *every* domain through
/// exactly these five methods.
///
/// Implementations are expected to be non-blocking: each hook performs
/// bounded, allocation-free work (enqueue to lock-free rings, latching
/// setpoints) so it can be called from the teleop session's realtime loop.
pub trait DomainTeleopInterface {
    /// Called when a human operator (or AI agent session) takes control.
    ///
    /// The domain should freeze its autonomy plan, blend the current actuator
    /// state toward the incoming control path, and start accepting
    /// [`Self::on_control_command`] traffic for `operator_id`.
    fn on_teleop_engage(&mut self, operator_id: OperatorId) -> Result<(), DtiError>;

    /// Called when control returns to autonomy.
    ///
    /// The domain smoothly transitions back: current teleop setpoints are
    /// handed to the autonomy controller as the new initial condition (no
    /// step change → no jerk).
    fn on_teleop_disengage(&mut self) -> Result<(), DtiError>;

    /// Receives one normalized, safety-arbitrated control command from the
    /// operator session. The domain translates it into domain-specific
    /// actuator commands (hydraulic valve positions, motor torques, …).
    fn on_control_command(&mut self, cmd: ControlCommand) -> Result<(), DtiError>;

    /// Provides the domain's current state to the teleop system
    /// (position, battery, fault status, …).
    fn get_domain_state(&self) -> DomainState;

    /// Provides the latest sensor batch to stream to the operator
    /// (camera frames, LiDAR, haptic force data).
    fn get_sensor_feed(&mut self) -> SensorFeed;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wire::{ButtonState, InputState, PixelFormat, VideoFrame};

    /// A trivial domain: latches the engaged operator, records the last
    /// command, and serves a one-frame feed.
    #[derive(Default)]
    struct DemoDomain {
        operator: Option<OperatorId>,
        last_speed: f32,
        engaged_count: u32,
    }

    impl DomainTeleopInterface for DemoDomain {
        fn on_teleop_engage(&mut self, operator_id: OperatorId) -> Result<(), DtiError> {
            if self.operator.is_some() {
                return Err(DtiError::InvalidState);
            }
            self.operator = Some(operator_id);
            self.engaged_count += 1;
            Ok(())
        }

        fn on_teleop_disengage(&mut self) -> Result<(), DtiError> {
            self.operator
                .take()
                .map(|_| ())
                .ok_or(DtiError::InvalidState)?;
            self.last_speed = 0.0;
            Ok(())
        }

        fn on_control_command(&mut self, cmd: ControlCommand) -> Result<(), DtiError> {
            if self.operator.is_none() {
                return Err(DtiError::InvalidState);
            }
            // A demo domain speed limit: refuse absurd commands.
            let speed = cmd.primary_input.axes[1];
            if speed.abs() > 10.0 {
                return Err(DtiError::SafetyReject);
            }
            self.last_speed = speed;
            Ok(())
        }

        fn get_domain_state(&self) -> DomainState {
            DomainState {
                speed_m_s: self.last_speed,
                ..DomainState::default()
            }
        }

        fn get_sensor_feed(&mut self) -> SensorFeed {
            let mut feed = SensorFeed::default();
            let _ = feed.push_video(VideoFrame {
                timestamp_us: 1,
                width: 64,
                height: 64,
                stride: 64 * 3,
                format: PixelFormat::Rgb8,
                data_offset: 0,
                data_len: 64 * 64 * 3,
            });
            feed
        }
    }

    fn sample_cmd(operator: OperatorId) -> ControlCommand {
        ControlCommand {
            timestamp_us: 100,
            operator_id: operator,
            primary_input: InputState {
                axes: [0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                pose: None,
                force_feedback: [0.0; 6],
            },
            secondary_input: InputState::default(),
            buttons: ButtonState::default(),
            haptic_feedback: crate::wire::HapticCmd::default(),
            seq: 1,
        }
    }

    #[test]
    fn full_session_lifecycle() {
        let mut domain = DemoDomain::default();
        let op = OperatorId([1; 16]);

        // Commands before engage are rejected.
        assert_eq!(
            domain.on_control_command(sample_cmd(op)),
            Err(DtiError::InvalidState)
        );

        domain.on_teleop_engage(op).expect("engage");
        assert_eq!(
            domain.on_teleop_engage(op),
            Err(DtiError::InvalidState),
            "double engage rejected"
        );

        domain
            .on_control_command(sample_cmd(op))
            .expect("cmd accepted");
        assert_eq!(domain.get_domain_state().speed_m_s, 2.0);

        let feed = domain.get_sensor_feed();
        assert_eq!(feed.video_count, 1);

        domain.on_teleop_disengage().expect("disengage");
        assert_eq!(domain.get_domain_state().speed_m_s, 0.0);
        assert_eq!(domain.on_teleop_disengage(), Err(DtiError::InvalidState));
    }

    #[test]
    fn safety_reject_is_reportable() {
        let mut domain = DemoDomain::default();
        let op = OperatorId([2; 16]);
        domain.on_teleop_engage(op).unwrap();
        let mut bad = sample_cmd(op);
        bad.primary_input.axes[1] = 42.0;
        assert_eq!(domain.on_control_command(bad), Err(DtiError::SafetyReject));
        // Domain remains engaged after a rejected command.
        assert!(domain.operator.is_some());
    }

    #[test]
    fn dti_error_display() {
        assert_eq!(
            DtiError::SafetyReject.to_string(),
            "command rejected by domain safety limits"
        );
    }
}
