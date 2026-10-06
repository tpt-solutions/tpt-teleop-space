//! Zero-copy rkyv wire-type prelude shared by the other space crates.
//!
//! All types here derive `rkyv::Archive` so they can be serialized to a
//! contiguous buffer and read back *in place* with zero allocations and zero
//! copies. They are plain-old-data-safe and contain no `Drop` glue, so
//! archived views are safe to map directly over shared memory, ring-buffer
//! slots, NVRAM sectors, or the wire.
//!
//! The prelude covers the position/command/health vocabulary every space
//! crate exchanges: inertial/site-frame poses, attitudes, normalized body
//! commands, subsystem health, and mission events. Sensor- and domain-
//! specific types (MEKF states, DEM grids, bundles) live in their own crates
//! and are designed to the same fixed-size, no-`Drop` rules.
//!
//! Serialization ([`serialize`]) allocates only the output buffer; the read
//! side is zero-copy via [`access`] (returns a borrowed archived view). Use
//! [`deserialize`] only when an owned `T` is genuinely required.

use bytecheck::CheckBytes;
use rkyv::{Archive, Deserialize, Portable, Serialize};

/// A rigid-body pose: metres + unit quaternion (`[w, x, y, z]`).
///
/// The frame is the *caller's* canonical frame: inertial (J2000-equivalent)
/// for orbital assets, site/local-level for surface assets. The wire record
/// is frame-agnostic; frames travel out-of-band with the session.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct Pose6Dof {
    /// Position, metres, caller's canonical frame.
    pub position_m: [f64; 3],
    /// Orientation unit quaternion, `[w, x, y, z]`, body → frame.
    pub quat_wxyz: [f64; 4],
    /// Pose timestamp, microseconds.
    pub timestamp_us: u64,
}

/// Attitude + rate: the ADCS-facing subset of [`Pose6Dof`] carried on the
/// telemetry bus every control cycle.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct Attitude {
    /// Orientation unit quaternion, `[w, x, y, z]`, body → frame.
    pub quat_wxyz: [f32; 4],
    /// Body angular rate, rad/s `[x, y, z]`.
    pub rate_rad_s: [f32; 3],
    /// Sample timestamp, microseconds.
    pub timestamp_us: u64,
}

/// Normalized 6DOF body command (translation `x/y/z`, rotation `x/y/z`).
/// All values `[-1.0, 1.0]`; this is the shared vocabulary between the
/// teleop adapter (`tpt-t-space-teleop`), the vehicle controllers
/// (`tpt-t-space-rover` / `-orbital` / `-satellite`), and the simulator.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct BodyCmd6 {
    /// Translation command, body X, normalized.
    pub tx: f32,
    /// Translation command, body Y, normalized.
    pub ty: f32,
    /// Translation command, body Z, normalized.
    pub tz: f32,
    /// Rotation moment, body X, normalized.
    pub rx: f32,
    /// Rotation moment, body Y, normalized.
    pub ry: f32,
    /// Rotation moment, body Z, normalized.
    pub rz: f32,
}

impl BodyCmd6 {
    /// Clamp every axis into `[-1.0, 1.0]`. Applied by the controller before
    /// actuator mixing; the teleop adapter re-applies defensively.
    pub fn clamped(mut self) -> Self {
        self.tx = self.tx.clamp(-1.0, 1.0);
        self.ty = self.ty.clamp(-1.0, 1.0);
        self.tz = self.tz.clamp(-1.0, 1.0);
        self.rx = self.rx.clamp(-1.0, 1.0);
        self.ry = self.ry.clamp(-1.0, 1.0);
        self.rz = self.rz.clamp(-1.0, 1.0);
        self
    }

    /// Whether every axis is (within epsilon) zero — the "hold attitude" /
    /// all-stop command.
    pub fn is_zero(self) -> bool {
        self.tx.abs() < f32::EPSILON
            && self.ty.abs() < f32::EPSILON
            && self.tz.abs() < f32::EPSILON
            && self.rx.abs() < f32::EPSILON
            && self.ry.abs() < f32::EPSILON
            && self.rz.abs() < f32::EPSILON
    }
}

/// Spacecraft/subsystem health summary carried on the telemetry bus.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Default)]
#[repr(C)]
pub struct SubsystemHealth {
    /// Battery state of charge, percent `0.0..=100.0`.
    pub battery_soc_pct: f32,
    /// Battery bus voltage.
    pub battery_v: f32,
    /// Instantaneous battery current (negative = charging).
    pub battery_a: f32,
    /// Worst-case electronics temperature, °C.
    pub temp_c: f32,
    /// Solar array output, watts (0 in eclipse / safe mode).
    pub solar_w: f32,
    /// Fault bitfield (interpretation is the subsystem's own; any set bit is
    /// triaged by fault management).
    pub fault_bits: u32,
    /// Seconds since boot.
    pub uptime_s: u32,
}

/// A space-asset event routed on the lock-free bus.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct SpaceEvent {
    /// Event kind discriminator (crate-local enum encoded as `u16`).
    pub kind: u16,
    /// Monotonic sequence number.
    pub seq: u32,
    /// Timestamp, microseconds.
    pub timestamp_us: u64,
    /// Opaque payload (interpretation depends on `kind`).
    pub payload: u64,
}

/// Serialize `value` into an owned byte buffer. Allocation occurs only
/// for the output; the read side stays zero-copy via [`access`]. Returns
/// `None` when rkyv cannot represent the value (never for the fixed-size
/// POD records in this prelude) — no `expect` in flight code.
pub fn serialize<T>(value: &T) -> Option<Vec<u8>>
where
    T: for<'a> Serialize<
        rkyv::api::high::HighSerializer<
            rkyv::util::AlignedVec,
            rkyv::ser::allocator::ArenaHandle<'a>,
            rkyv::rancor::Error,
        >,
    >,
{
    Some(
        rkyv::api::high::to_bytes::<rkyv::rancor::Error>(value)
            .ok()?
            .to_vec(),
    )
}

/// Zero-copy read: validate (`CheckBytes`) and return a borrowed archived
/// view.
///
/// Returns `None` if the bytes fail validation (wrong size, corrupt data, or
/// a type/schema mismatch). The returned reference borrows `bytes`, so no
/// copy or allocation occurs.
pub fn access<T>(bytes: &[u8]) -> Option<&<T as Archive>::Archived>
where
    T: Archive,
    <T as Archive>::Archived:
        Portable + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>,
{
    rkyv::api::high::access::<<T as Archive>::Archived, rkyv::rancor::Error>(bytes).ok()
}

/// Deserialize into an owned `T`. Use only when an owned value is required;
/// prefer [`access`] for zero-copy reads.
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
    fn pose_roundtrip_zero_copy() {
        let pose = Pose6Dof {
            position_m: [-2469.5, 1187.25, 32.125],
            quat_wxyz: [1.0, 0.0, 0.0, 0.0],
            timestamp_us: 1_725_000_000_123,
        };
        let bytes = serialize(&pose).unwrap();
        let view = access::<Pose6Dof>(&bytes).expect("valid archived pose");
        assert_eq!(view.position_m, pose.position_m);
        assert_eq!(view.quat_wxyz, pose.quat_wxyz);
        assert_eq!(view.timestamp_us, pose.timestamp_us);
    }

    #[test]
    fn attitude_and_body_cmd_roundtrip() {
        let att = Attitude {
            quat_wxyz: [
                core::f32::consts::FRAC_1_SQRT_2,
                0.0,
                core::f32::consts::FRAC_1_SQRT_2,
                0.0,
            ],
            rate_rad_s: [0.001, -0.002, 0.0],
            timestamp_us: 99,
        };
        let bytes = serialize(&att).unwrap();
        let back = deserialize::<Attitude>(&bytes).unwrap();
        assert_eq!(back, att);

        let cmd = BodyCmd6 {
            tx: 0.10,
            rz: -0.05,
            ..BodyCmd6::default()
        };
        let bytes = serialize(&cmd).unwrap();
        let view = access::<BodyCmd6>(&bytes).unwrap();
        assert_eq!(view.tx, 0.10);
        assert_eq!(view.rz, -0.05);
        assert!(!cmd.is_zero());
        assert!(BodyCmd6::default().is_zero());
    }

    #[test]
    fn body_cmd_clamp() {
        let cmd = BodyCmd6 {
            tx: 1.5,
            ty: -2.0,
            tz: 0.5,
            rx: 0.0,
            ry: 0.0,
            rz: -0.25,
        }
        .clamped();
        assert_eq!(cmd.tx, 1.0);
        assert_eq!(cmd.ty, -1.0);
        assert_eq!(cmd.tz, 0.5);
        assert_eq!(cmd.rz, -0.25);
    }

    #[test]
    fn corrupt_bytes_rejected() {
        let bytes = serialize(&SpaceEvent {
            kind: 1,
            seq: 2,
            timestamp_us: 3,
            payload: 4,
        })
        .unwrap();
        // A truncated buffer has no valid archived root, so validation must
        // reject it.
        let truncated = &bytes[..bytes.len().saturating_sub(1)];
        assert!(access::<SpaceEvent>(truncated).is_none());
    }

    #[test]
    fn health_roundtrip() {
        let health = SubsystemHealth {
            battery_soc_pct: 61.5,
            battery_v: 28.2,
            battery_a: -4.5,
            temp_c: -12.0,
            solar_w: 842.0,
            fault_bits: 0,
            uptime_s: 86_400,
        };
        let bytes = serialize(&health).unwrap();
        let view = access::<SubsystemHealth>(&bytes).unwrap();
        assert_eq!(view.battery_soc_pct, 61.5);
        assert_eq!(view.solar_w, 842.0);
    }
}
