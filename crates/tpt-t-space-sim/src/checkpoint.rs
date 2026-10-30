//! Time-travel checkpoints (Phase 20): the mission context saved at each
//! step of a deterministic run, so a viewer can rewind to any of them.
//!
//! Each checkpoint is a [`SnapshotStore`] image of a [`Context`], the same
//! CRC-guarded format the flight software persists across resets. Restoring
//! one therefore exercises the real recovery path, not a copy of the state.
//!
//! This rewinds a recorded run. It does not rewind a live sim: that needs a
//! running sim to take checkpoints as it goes, which this crate does not yet
//! provide.

use rkyv::{Archive, Deserialize, Serialize};
use tpt_t_space_core::blackbox::state_code;
use tpt_t_space_core::machine::MissionState;
use tpt_t_space_core::snapshot::SnapshotStore;

/// Bytes reserved for one checkpoint image in its NVRAM buffer.
pub const IMAGE_BYTES: usize = 64;

/// The mission context at one instant.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct Context {
    /// Simulation time, whole seconds.
    pub t_s: u32,
    /// Mission state, as [`state_code`] encodes it.
    pub mission_state: u8,
    /// Whether Safe Mode has been entered.
    pub safe_mode: bool,
    /// Telemetry bundles delivered over the link so far.
    pub bundles_delivered: u32,
}

/// One saved point in a run: a label for the timeline and its image.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Checkpoint {
    /// The timeline phase this checkpoint closes, e.g. `"blackout"`.
    pub label: &'static str,
    /// The CRC-guarded image, restorable with [`Checkpoint::restore`].
    pub image: Vec<u8>,
}

impl Checkpoint {
    /// Decode the saved context, or `None` if the image is corrupt.
    #[must_use]
    pub fn restore(&self) -> Option<Context> {
        SnapshotStore::new(IMAGE_BYTES, 64).restore::<Context>(&self.image)
    }
}

/// Failure to write a checkpoint image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CheckpointError;

impl std::fmt::Display for CheckpointError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "checkpoint image did not fit")
    }
}

impl std::error::Error for CheckpointError {}

/// Takes checkpoints in order, so each image carries an increasing sequence.
pub struct Checkpointer {
    store: SnapshotStore,
    taken: Vec<Checkpoint>,
}

impl Default for Checkpointer {
    fn default() -> Self {
        Self::new()
    }
}

impl Checkpointer {
    /// An empty checkpointer.
    #[must_use]
    pub fn new() -> Self {
        Self {
            store: SnapshotStore::new(IMAGE_BYTES, 64),
            taken: Vec::new(),
        }
    }

    /// Save the context at the end of `label`.
    ///
    /// # Errors
    /// [`CheckpointError`] if the context does not serialize into the image.
    pub fn capture(
        &mut self,
        label: &'static str,
        t_s: f64,
        state: MissionState,
        safe_mode: bool,
        bundles_delivered: u32,
    ) -> Result<(), CheckpointError> {
        let ctx = Context {
            t_s: t_s as u32,
            mission_state: state_code(state),
            safe_mode,
            bundles_delivered,
        };
        let mut image = vec![0u8; IMAGE_BYTES];
        self.store
            .save(&mut image, &ctx)
            .map_err(|_| CheckpointError)?;
        self.taken.push(Checkpoint { label, image });
        Ok(())
    }

    /// The checkpoints taken so far, oldest first.
    #[must_use]
    pub fn into_checkpoints(self) -> Vec<Checkpoint> {
        self.taken
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_captured_context_restores_exactly() {
        let mut cp = Checkpointer::new();
        cp.capture("blackout", 3_600.0, MissionState::Operations, false, 0)
            .expect("fits");
        cp.capture("safe-mode", 7_200.0, MissionState::SafeMode, true, 8)
            .expect("fits");
        let taken = cp.into_checkpoints();
        assert_eq!(taken.len(), 2);
        assert_eq!(taken[0].label, "blackout");
        assert_eq!(
            taken[1].restore(),
            Some(Context {
                t_s: 7_200,
                mission_state: state_code(MissionState::SafeMode),
                safe_mode: true,
                bundles_delivered: 8,
            })
        );
    }

    #[test]
    fn a_corrupt_image_restores_to_nothing() {
        let mut cp = Checkpointer::new();
        cp.capture("uplink", 0.0, MissionState::Cruise, false, 0)
            .expect("fits");
        let mut taken = cp.into_checkpoints();
        // Byte 24 is the first payload byte; the header is 24 bytes.
        taken[0].image[24] ^= 0xFF;
        assert_eq!(taken[0].restore(), None);
    }
}
