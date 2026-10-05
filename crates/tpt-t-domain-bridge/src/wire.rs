//! Zero-copy rkyv wire types for the DTI data plane.
//!
//! Every type derives `rkyv::Archive` and is a fixed-size record: bounded
//! arrays with explicit length fields stand in for the `Vec`s sketched in
//! `bridge spec.txt`, and bulk payloads (video, point clouds, audio) are
//! *referenced* by `(offset, len)` into an out-of-band shared buffer rather
//! than inlined. An archived [`SensorFeed`] is therefore safe to map directly
//! over shared memory, a ring-buffer slot, or the wire with no rehydration
//! allocation at all.
//!
//! Serialization helpers ([`serialize`]/[`access`]/[`deserialize`]) mirror the
//! `tpt-t-space-core::wire` prelude so domain adapters use one consistent API.

use bytecheck::CheckBytes;
use rkyv::{Archive, Deserialize, Portable, Serialize};

/// Identity of the controlling party — a human operator *or* an AI agent.
///
/// Deliberately a plain 16-byte value instead of a UUID crate type: the wire
/// only needs opaque uniqueness, and the zero-bloat policy (spec §7) avoids
/// pulling a dependency for one type. Hex formatting for display is provided.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(C)]
pub struct OperatorId(
    /// Raw 16-byte identifier (UUID-compatible when generated from a UUID).
    pub [u8; 16],
);

impl OperatorId {
    /// Placeholder for sessions arriving over a legacy (pre-DTI) transport
    /// that carries no operator identity. See the crate docs for the legacy
    /// `ControlCommand` reconciliation policy.
    pub const LEGACY_ANON: OperatorId = OperatorId([0; 16]);

    /// Whether this is the legacy anonymous id.
    pub fn is_legacy_anon(self) -> bool {
        self.0 == [0; 16]
    }
}

impl core::fmt::Display for OperatorId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for byte in self.0 {
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

/// A rigid-body pose: position (metres) + unit quaternion (`[w, x, y, z]`).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct Pose6Dof {
    /// Position, metres (domain frame).
    pub position_m: [f32; 3],
    /// Orientation unit quaternion, `[w, x, y, z]`.
    pub quat_wxyz: [f32; 4],
}

/// One normalized input device sample: axes, optional 6-DOF pose, and the
/// device's own force-feedback state.
///
/// `axes` are normalized to `[-1.0, 1.0]` by the upstream input layer
/// (`tpt-teleop-input`); the domain adapter never rescales raw device counts.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct InputState {
    /// Normalized axis values, `[-1.0, 1.0]` (unused trailing axes are `0.0`).
    pub axes: [f32; 8],
    /// VR/AR controller pose, when the device provides one.
    pub pose: Option<Pose6Dof>,
    /// Force-feedback magnitudes the haptic device is currently rendering.
    pub force_feedback: [f32; 6],
}

impl Default for InputState {
    fn default() -> Self {
        Self {
            axes: [0.0; 8],
            pose: None,
            force_feedback: [0.0; 6],
        }
    }
}

impl InputState {
    /// Clamp all axes into `[-1.0, 1.0]`. The upstream input layer applies
    /// this before transmission; domain adapters may re-apply defensively.
    pub fn clamped(mut self) -> Self {
        for a in &mut self.axes {
            *a = a.clamp(-1.0, 1.0);
        }
        self
    }
}

/// Canonical button bit positions inside [`ButtonState::bits`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Button {
    /// Emergency stop — latched request to halt immediately.
    EStop = 0,
    /// Mode switch (autonomy / teleop toggle).
    ModeSwitch = 1,
    /// Implement raise/lower toggle.
    ImplementRaise = 2,
    /// Implement lower toggle.
    ImplementLower = 3,
    /// Horn / presence alert.
    Horn = 4,
}

/// Button state bitfield from the operator console.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(C)]
pub struct ButtonState(
    /// Raw bitfield; bit *n* is [`Button`] `n` where defined.
    pub u64,
);

impl ButtonState {
    /// Whether the given button bit is set.
    pub fn is_set(self, button: Button) -> bool {
        self.0 & (1 << button as u64) != 0
    }

    /// Set a button bit.
    pub fn set(&mut self, button: Button) {
        self.0 |= 1 << button as u64;
    }

    /// Clear a button bit.
    pub fn clear(&mut self, button: Button) {
        self.0 &= !(1 << button as u64);
    }

    /// Whether the emergency-stop bit is latched.
    pub fn e_stop(self) -> bool {
        self.is_set(Button::EStop)
    }
}

// Mirror accessors on the archived view so zero-copy consumers of a
// transmitted `ControlCommand` get the same ergonomic reads.
impl ArchivedButtonState {
    /// Whether the given button bit is set.
    pub fn is_set(&self, button: Button) -> bool {
        self.0 & (1 << button as u64) != 0
    }

    /// Whether the emergency-stop bit is latched.
    pub fn e_stop(&self) -> bool {
        self.is_set(Button::EStop)
    }
}

/// Force-feedback command rendered on the operator's haptic device.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct HapticCmd {
    /// Master enable; `false` mutes all output regardless of the gains.
    pub enabled: bool,
    /// Per-axis gain, `0.0..=1.0` (cartesian force + torque axes).
    pub gains: [f32; 6],
    /// Vibration period in microseconds (0 = steady force).
    pub period_us: u32,
}

impl Default for HapticCmd {
    fn default() -> Self {
        Self {
            enabled: false,
            gains: [0.0; 6],
            period_us: 0,
        }
    }
}

/// Force/torque measurements reported *by* the robot to the haptic device
/// (the sensor-side counterpart of [`HapticCmd`]).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct HapticFeedback {
    /// Contact forces in newtons `[x, y, z]`.
    pub forces_n: [f32; 3],
    /// Contact torques in newton-metres `[x, y, z]`.
    pub torques_nm: [f32; 3],
}

/// Pixel encoding of a referenced video frame.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PixelFormat {
    /// 8-bit RGB, 3 bytes per pixel.
    Rgb8,
    /// 8-bit BGR, 3 bytes per pixel.
    Bgr8,
    /// 8-bit grayscale / thermal, 1 byte per pixel.
    Luma8,
    /// 16-bit depth, millimetres, little-endian, 2 bytes per pixel.
    DepthMm16,
}

/// Metadata for one video frame; the pixel bytes live out-of-band.
///
/// [`data_offset`](VideoFrame::data_offset) indexes the shared frame buffer
/// the session negotiated at setup (ring slot, DMA region, or transmitted
/// blob), so the wire record stays a fixed 24-ish bytes instead of megabytes.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct VideoFrame {
    /// Capture timestamp, microseconds.
    pub timestamp_us: u64,
    /// Offset of the pixel data within the session frame buffer.
    pub data_offset: u64,
    /// Pixel data length in bytes.
    pub data_len: u32,
    /// Width in pixels.
    pub width: u16,
    /// Height in pixels.
    pub height: u16,
    /// Row stride in bytes (≥ width × bytes-per-pixel).
    pub stride: u32,
    /// Pixel encoding.
    pub format: PixelFormat,
}

/// Metadata for one point cloud; points live out-of-band like video pixels.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct PointCloud {
    /// Capture timestamp, microseconds.
    pub timestamp_us: u64,
    /// Offset of the point data within the session frame buffer.
    pub data_offset: u64,
    /// Point data length in bytes.
    pub data_len: u32,
    /// Number of points.
    pub point_count: u32,
    /// Bytes per point (e.g. 12 = packed `f32` XYZ).
    pub point_stride: u8,
}

/// Audio stream metadata; samples live out-of-band.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct AudioStream {
    /// Capture timestamp, microseconds.
    pub timestamp_us: u64,
    /// Offset of the sample data within the session frame buffer.
    pub data_offset: u64,
    /// Sample data length in bytes.
    pub data_len: u32,
    /// Sample rate in hertz.
    pub sample_rate_hz: u32,
    /// Channel count.
    pub channels: u8,
    /// Bytes per sample (per channel).
    pub sample_stride: u8,
}

/// Domain-agnostic robot telemetry (battery, link, health).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct DomainTelemetry {
    /// Battery state of charge, percent `0.0..=100.0`.
    pub battery_soc_pct: f32,
    /// Battery pack voltage.
    pub battery_v: f32,
    /// Instantaneous battery current (negative = charging).
    pub battery_a: f32,
    /// Representative drive/controller temperature, °C.
    pub temp_c: f32,
    /// Radio link RSSI, dBm.
    pub link_rssi_dbm: f32,
    /// Seconds since domain boot.
    pub uptime_s: u32,
    /// Domain-specific error bitfield (interpretation is domain's own).
    pub error_bits: u64,
}

/// The domain's full current state, as reported to the operator station.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct DomainState {
    /// Sample timestamp, microseconds.
    pub timestamp_us: u64,
    /// Latitude, degrees WGS-84 (`0.0` when the domain has no GNSS).
    pub lat_deg: f64,
    /// Longitude, degrees WGS-84.
    pub lon_deg: f64,
    /// Altitude, metres.
    pub alt_m: f32,
    /// Heading, degrees from true north.
    pub yaw_deg: f32,
    /// Ground speed, m/s (negative = reversing).
    pub speed_m_s: f32,
    /// [`safety::SafetyState`] discriminator (0 = Autonomous, …) so the
    /// operator UI renders the current autonomy/teleop mode without a
    /// domain-specific enum dependency.
    pub safety_state: u8,
    /// Domain fault bitfield (interpretation is domain's own).
    pub fault_bits: u64,
    /// Generic telemetry block.
    pub telemetry: DomainTelemetry,
}

/// Maximum video frames carried in one [`SensorFeed`].
pub const MAX_VIDEO_FRAMES: usize = 4;
/// Maximum point clouds carried in one [`SensorFeed`].
pub const MAX_POINT_CLOUDS: usize = 4;

/// The standardized sensor uplink: one fixed-size record bundling frame /
/// cloud *metadata* (payloads are out-of-band references), telemetry, haptics,
/// and optional audio.
///
/// This is the zero-alloc encoding of the `SensorFeed` sketched in
/// `bridge spec.txt` §4.2: instead of `Vec<VideoFrame>` there are bounded
/// arrays with live counts, so an archived feed can be mapped read-only over
/// shared memory without deserializing into heap structures.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct SensorFeed {
    /// Feed timestamp, microseconds.
    pub timestamp_us: u64,
    /// Video frame batch (RGB, thermal, depth) — metadata only.
    pub video_frames: [VideoFrame; MAX_VIDEO_FRAMES],
    /// Number of valid entries in [`SensorFeed::video_frames`].
    pub video_count: u8,
    /// Point-cloud batch (LiDAR, radar) — metadata only.
    pub point_clouds: [PointCloud; MAX_POINT_CLOUDS],
    /// Number of valid entries in [`SensorFeed::point_clouds`].
    pub cloud_count: u8,
    /// Robot telemetry.
    pub telemetry: DomainTelemetry,
    /// Force/torque data destined for the operator's haptic device.
    pub haptic_data: HapticFeedback,
    /// Microphone audio, when the session carries it.
    pub audio: Option<AudioStream>,
}

impl Default for SensorFeed {
    fn default() -> Self {
        Self {
            timestamp_us: 0,
            video_frames: [EMPTY_VIDEO_FRAME; MAX_VIDEO_FRAMES],
            video_count: 0,
            point_clouds: [EMPTY_POINT_CLOUD; MAX_POINT_CLOUDS],
            cloud_count: 0,
            telemetry: DomainTelemetry::default(),
            haptic_data: HapticFeedback::default(),
            audio: None,
        }
    }
}

const EMPTY_VIDEO_FRAME: VideoFrame = VideoFrame {
    timestamp_us: 0,
    data_offset: 0,
    data_len: 0,
    width: 0,
    height: 0,
    stride: 0,
    format: PixelFormat::Rgb8,
};

const EMPTY_POINT_CLOUD: PointCloud = PointCloud {
    timestamp_us: 0,
    data_offset: 0,
    data_len: 0,
    point_count: 0,
    point_stride: 0,
};

impl SensorFeed {
    /// Push a video frame into the batch. Returns `Err(frame)` if the batch
    /// is full ([`MAX_VIDEO_FRAMES`]).
    pub fn push_video(&mut self, frame: VideoFrame) -> Result<(), VideoFrame> {
        if self.video_count as usize >= MAX_VIDEO_FRAMES {
            return Err(frame);
        }
        self.video_frames[self.video_count as usize] = frame;
        self.video_count += 1;
        Ok(())
    }

    /// Push a point cloud into the batch. Returns `Err(cloud)` if the batch
    /// is full ([`MAX_POINT_CLOUDS`]).
    pub fn push_cloud(&mut self, cloud: PointCloud) -> Result<(), PointCloud> {
        if self.cloud_count as usize >= MAX_POINT_CLOUDS {
            return Err(cloud);
        }
        self.point_clouds[self.cloud_count as usize] = cloud;
        self.cloud_count += 1;
        Ok(())
    }
}

/// The standardized downlink control command (canonical form).
///
/// A session maps to a single [`operator_id`](ControlCommand::operator_id) per
/// call: shared human+AI control is arbitrated *upstream* (in
/// `tpt-teleop-core` / `tpt-teleop-safety`) and folded into this one struct —
/// the domain adapter is never handed two competing streams to reconcile.
/// See `bridge spec.txt` §4.1.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct ControlCommand {
    /// Microsecond timestamp of input capture.
    pub timestamp_us: u64,
    /// Who is controlling: a human operator or an AI agent session.
    pub operator_id: OperatorId,
    /// Primary input (joystick, VR hands, steering wheel).
    pub primary_input: InputState,
    /// Secondary input (implement control, camera gimbal).
    pub secondary_input: InputState,
    /// Discrete buttons (e-stop, mode switch, …).
    pub buttons: ButtonState,
    /// Force-feedback command for the operator's haptic device.
    pub haptic_feedback: HapticCmd,
    /// Sequence number for loss / duplicate detection.
    pub seq: u32,
}

impl ControlCommand {
    /// Build the canonical command from a legacy 56-byte flat POD transport
    /// encoding: `axes` are the legacy struct's axis words (primary plus
    /// secondary), split 8/8 across the two [`InputState`]s; `operator_id`
    /// has no legacy field, so [`OperatorId::LEGACY_ANON`] is used.
    ///
    /// See the crate docs for the full reconciliation policy.
    pub fn from_legacy_axes(axes: [f32; 16], timestamp_us: u64, seq: u32) -> Self {
        let mut primary = InputState::default();
        primary.axes.copy_from_slice(&axes[..8]);
        let mut secondary = InputState::default();
        secondary.axes.copy_from_slice(&axes[8..]);
        Self {
            timestamp_us,
            operator_id: OperatorId::LEGACY_ANON,
            primary_input: primary,
            secondary_input: secondary,
            buttons: ButtonState::default(),
            haptic_feedback: HapticCmd::default(),
            seq,
        }
    }
}

/// Serialize `value` into an owned byte buffer. Allocation occurs only for
/// the output; the read side stays zero-copy via [`access`].
pub fn serialize<T>(value: &T) -> Vec<u8>
where
    T: for<'a> Serialize<
        rkyv::api::high::HighSerializer<
            rkyv::util::AlignedVec,
            rkyv::ser::allocator::ArenaHandle<'a>,
            rkyv::rancor::Error,
        >,
    >,
{
    rkyv::api::high::to_bytes::<rkyv::rancor::Error>(value)
        .expect("rkyv serialize")
        .to_vec()
}

/// Zero-copy read: validate (`CheckBytes`) and return a borrowed archived
/// view, or `None` if validation fails.
pub fn access<T>(bytes: &[u8]) -> Option<&<T as Archive>::Archived>
where
    T: Archive,
    <T as Archive>::Archived:
        Portable + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
{
    rkyv::api::high::access::<<T as Archive>::Archived, rkyv::rancor::Error>(bytes).ok()
}

/// Deserialize into an owned `T`. Prefer [`access`] for zero-copy reads.
pub fn deserialize<T>(bytes: &[u8]) -> Option<T>
where
    T: Archive,
    <T as Archive>::Archived: Portable
        + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, rkyv::api::high::HighDeserializer<rkyv::rancor::Error>>,
{
    rkyv::api::high::from_bytes::<T, rkyv::rancor::Error>(bytes).ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn control_command_roundtrip() {
        let cmd = ControlCommand {
            timestamp_us: 1_725_000_000_000,
            operator_id: OperatorId([7; 16]),
            primary_input: InputState {
                axes: [0.5, -0.25, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0],
                pose: Some(Pose6Dof {
                    position_m: [1.0, 2.0, 3.0],
                    quat_wxyz: [1.0, 0.0, 0.0, 0.0],
                }),
                force_feedback: [0.1; 6],
            },
            secondary_input: InputState::default(),
            buttons: {
                let mut b = ButtonState::default();
                b.set(Button::ModeSwitch);
                b
            },
            haptic_feedback: HapticCmd {
                enabled: true,
                gains: [0.5; 6],
                period_us: 20_000,
            },
            seq: 42,
        };
        let bytes = serialize(&cmd);
        let view = access::<ControlCommand>(&bytes).expect("valid archived command");
        assert_eq!(view.seq, 42);
        assert_eq!(view.operator_id.0, [7; 16]);
        assert_eq!(view.primary_input.axes[0], 0.5);
        assert!(view.buttons.is_set(Button::ModeSwitch));
        assert!(!view.buttons.e_stop());
        assert_eq!(view.haptic_feedback.period_us, 20_000);

        let owned = deserialize::<ControlCommand>(&bytes).unwrap();
        assert_eq!(owned, cmd);
    }

    #[test]
    fn sensor_feed_batches_are_bounded() {
        let mut feed = SensorFeed::default();
        for i in 0..MAX_VIDEO_FRAMES {
            assert!(
                feed.push_video(VideoFrame {
                    timestamp_us: i as u64,
                    width: 1920,
                    height: 1080,
                    stride: 1920 * 3,
                    format: PixelFormat::Rgb8,
                    data_offset: 0,
                    data_len: 1920 * 1080 * 3,
                })
                .is_ok()
            );
        }
        let overflow = VideoFrame {
            timestamp_us: 99,
            ..feed.video_frames[0]
        };
        assert_eq!(feed.push_video(overflow), Err(overflow));
        assert_eq!(feed.video_count as usize, MAX_VIDEO_FRAMES);

        assert!(
            feed.push_cloud(PointCloud {
                timestamp_us: 1,
                data_offset: 4096,
                data_len: 1200,
                point_count: 100,
                point_stride: 12,
            })
            .is_ok()
        );

        let bytes = serialize(&feed);
        let view = access::<SensorFeed>(&bytes).expect("valid archived feed");
        assert_eq!(view.video_count, MAX_VIDEO_FRAMES as u8);
        assert_eq!(view.video_frames[2].width, 1920);
        assert_eq!(view.cloud_count, 1);
        assert_eq!(view.point_clouds[0].point_stride, 12);
        assert!(view.audio.is_none());
    }

    #[test]
    fn legacy_axes_mapping() {
        let mut axes = [0.0f32; 16];
        axes[0] = -0.5; // legacy steering
        axes[1] = 1.0; // legacy speed
        axes[8] = 0.75; // first secondary axis
        let cmd = ControlCommand::from_legacy_axes(axes, 123, 7);
        assert_eq!(cmd.primary_input.axes[0], -0.5);
        assert_eq!(cmd.primary_input.axes[1], 1.0);
        assert_eq!(cmd.secondary_input.axes[0], 0.75);
        assert_eq!(cmd.operator_id, OperatorId::LEGACY_ANON);
        assert!(cmd.operator_id.is_legacy_anon());
        assert_eq!(cmd.seq, 7);
    }

    #[test]
    fn operator_id_display_and_clamp() {
        let id = OperatorId([0xAB; 16]);
        assert_eq!(id.to_string(), "ab".repeat(16));
        assert!(!id.is_legacy_anon());

        let mut input = InputState::default();
        input.axes[0] = 1.5;
        input.axes[1] = -2.0;
        let clamped = input.clamped();
        assert_eq!(clamped.axes[0], 1.0);
        assert_eq!(clamped.axes[1], -1.0);
    }

    #[test]
    fn button_bits() {
        let mut b = ButtonState::default();
        b.set(Button::EStop);
        assert!(b.e_stop());
        b.clear(Button::EStop);
        assert!(!b.e_stop());
        b.set(Button::Horn);
        assert!(b.is_set(Button::Horn));
        assert_eq!(b.0, 1 << Button::Horn as u64);
    }
}
