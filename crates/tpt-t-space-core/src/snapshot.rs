//! Persistent state snapshotting so the event loop survives processor resets
//! without losing mission context (spec §3, `tpt-t-space-core`).
//!
//! A rad-hard or watchdog reset wipes RAM; the mission context (mission
//! state, waypoint index, sequencing progress, custody bookkeeping) must
//! therefore live in non-volatile storage. [`SnapshotStore`] serializes a
//! caller-chosen context record with rkyv into a caller-owned NVRAM byte
//! buffer, guarded by a CRC32-stamped header and a monotonically increasing
//! sequence number:
//!
//! ```text
//! [0..8)   magic "TPTSNAP1"       [16..20) payload len (u32 LE)
//! [8..12)  layout version (u32)  [20..24) CRC32 of payload (u32 LE)
//! [12..16) snapshot seq (u32 LE) [24..)   rkyv-archived payload
//! ```
//!
//! After an unexpected reset, [`SnapshotStore::restore`] revalidates the
//! CRC and hands back a deserialized context — or `None`, in which case the
//! loop starts a fresh mission (a corrupt snapshot must never brick the
//! asset). Snapshots are taken at hibernate/transition boundaries, not per
//! tick; `save` allocates nothing per call (the scratch buffer is
//! pre-allocated at construction), honoring the no-dynamic-allocation-
//! after-initialization policy.

use crate::crc::crc32_one_shot;
use bytecheck::CheckBytes;
use rkyv::util::AlignedVec;
use rkyv::{Archive, Deserialize, Portable, Serialize};

const MAGIC: [u8; 8] = *b"TPTSNAP1";
const HEADER_LEN: usize = 24;
const LAYOUT_VERSION: u32 = 1;

/// Why a snapshot operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapshotError {
    /// The record's archived form does not fit the NVRAM buffer.
    TooLarge,
    /// The record's archived form exceeds the pre-allocated scratch.
    ScratchExhausted,
}

impl core::fmt::Display for SnapshotError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let msg = match self {
            SnapshotError::TooLarge => "snapshot does not fit the NVRAM buffer",
            SnapshotError::ScratchExhausted => "snapshot exceeds the pre-allocated scratch",
        };
        f.write_str(msg)
    }
}

impl std::error::Error for SnapshotError {}

/// CRC-guarded persistent snapshot store over a caller-owned NVRAM buffer.
pub struct SnapshotStore {
    nvram_len: usize,
    /// Maximum archived payload accepted by `save` (bounds the scratch).
    scratch_budget: usize,
    seq: u32,
    valid: bool,
    /// Serialization scratch, allocated once here at construction; every
    /// later `save` reuses it (zero allocation per call at steady size).
    scratch: AlignedVec,
}

impl SnapshotStore {
    /// Wrap an NVRAM buffer with the given maximum payload size (the
    /// scratch is pre-grown to that budget once, here).
    pub fn new(nvram_len: usize, scratch_bytes: usize) -> Self {
        let mut scratch = AlignedVec::new();
        scratch.reserve(scratch_bytes.max(64));
        Self {
            nvram_len,
            scratch_budget: scratch_bytes.max(64),
            seq: 0,
            valid: false,
            scratch,
        }
    }

    /// Serialize `state` into the NVRAM buffer and stamp the header.
    /// Returns the snapshot sequence number.
    pub fn save<T>(&mut self, nvram: &mut [u8], state: &T) -> Result<u32, SnapshotError>
    where
        T: Archive
            + for<'a> Serialize<
                rkyv::api::high::HighSerializer<
                    AlignedVec,
                    rkyv::ser::allocator::ArenaHandle<'a>,
                    rkyv::rancor::Error,
                >,
            >,
    {
        if nvram.len() < self.nvram_len || nvram.len() < HEADER_LEN {
            return Err(SnapshotError::TooLarge);
        }
        // Serialize into the pre-allocated scratch (capacity is reused
        // across saves; `to_bytes_in` hands the writer back).
        self.scratch.clear();
        let taken = core::mem::take(&mut self.scratch);
        match rkyv::api::high::to_bytes_in::<_, rkyv::rancor::Error>(state, taken) {
            Ok(written) => self.scratch = written,
            Err(_) => return Err(SnapshotError::ScratchExhausted),
        }
        let payload = self.scratch.as_slice();
        if payload.len() > self.scratch_budget {
            return Err(SnapshotError::ScratchExhausted);
        }
        if HEADER_LEN + payload.len() > nvram.len() {
            return Err(SnapshotError::TooLarge);
        }
        let crc = crc32_one_shot(payload);
        nvram[..HEADER_LEN + payload.len()].fill(0);
        nvram[..8].copy_from_slice(&MAGIC);
        nvram[8..12].copy_from_slice(&LAYOUT_VERSION.to_le_bytes());
        nvram[12..16].copy_from_slice(&self.seq.to_le_bytes());
        nvram[16..20].copy_from_slice(&(payload.len() as u32).to_le_bytes());
        nvram[20..24].copy_from_slice(&crc.to_le_bytes());
        nvram[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
        self.valid = true;
        let seq = self.seq;
        self.seq = self.seq.wrapping_add(1);
        Ok(seq)
    }

    /// Validate the header + CRC of the buffer and return the stored
    /// sequence number, or `None` if the snapshot is absent or corrupt.
    pub fn peek(&self, nvram: &[u8]) -> Option<u32> {
        if nvram.len() < HEADER_LEN {
            return None;
        }
        if nvram[..8] != MAGIC {
            return None;
        }
        let version = u32::from_le_bytes(nvram[8..12].try_into().ok()?);
        if version != LAYOUT_VERSION {
            return None;
        }
        let seq = u32::from_le_bytes(nvram[12..16].try_into().ok()?);
        let len = u32::from_le_bytes(nvram[16..20].try_into().ok()?) as usize;
        let crc = u32::from_le_bytes(nvram[20..24].try_into().ok()?);
        if HEADER_LEN + len > nvram.len() {
            return None;
        }
        let payload = &nvram[HEADER_LEN..HEADER_LEN + len];
        if crc32_one_shot(payload) != crc {
            return None;
        }
        Some(seq)
    }

    /// Deserialize the stored snapshot into an owned `T`, or `None` when it
    /// is absent, corrupt, or of the wrong type. This is the reset-recovery
    /// entry point.
    pub fn restore<T>(&self, nvram: &[u8]) -> Option<T>
    where
        T: Archive,
        <T as Archive>::Archived: Portable
            + for<'a> CheckBytes<rkyv::api::high::HighValidator<'a, rkyv::rancor::Error>>
            + Deserialize<T, rkyv::api::high::HighDeserializer<rkyv::rancor::Error>>,
    {
        self.peek(nvram)?;
        let len = u32::from_le_bytes(nvram[16..20].try_into().ok()?) as usize;
        let payload = &nvram[HEADER_LEN..HEADER_LEN + len];
        // Copy through the (aligned) scratch so validation sees the
        // alignment rkyv requires, then deserialize.
        let mut aligned: AlignedVec = AlignedVec::new();
        aligned.reserve(payload.len());
        aligned.extend_from_slice(payload);
        rkyv::api::high::from_bytes::<T, rkyv::rancor::Error>(aligned.as_slice()).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The mission context a loop persists across resets, in this test:
    /// mission state discriminator, waypoint index, and an ops counter.
    #[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq)]
    #[repr(C)]
    struct MissionContext {
        /// [`crate::machine::MissionState`] discriminator.
        pub mission_state: u8,
        /// Current route waypoint index.
        pub waypoint_idx: u16,
        /// Science operations completed.
        pub ops_done: u32,
        /// Last transition timestamp, µs.
        pub last_transition_us: u64,
    }

    #[test]
    fn save_peek_restore_roundtrip() {
        let mut nvram = [0u8; 512];
        let mut store = SnapshotStore::new(nvram.len(), 256);

        // Nothing stored yet.
        assert_eq!(store.peek(&nvram), None);
        assert_eq!(store.restore::<MissionContext>(&nvram), None);

        let ctx = MissionContext {
            mission_state: 2, // OPERATIONS
            waypoint_idx: 7,
            ops_done: 41,
            last_transition_us: 1_725_000_999_999,
        };
        let seq0 = store.save(&mut nvram, &ctx).expect("fits");
        assert_eq!(seq0, 0);
        assert_eq!(store.peek(&nvram), Some(0));

        let mut restored = store.restore::<MissionContext>(&nvram).expect("valid");
        assert_eq!(restored, ctx);

        // A second snapshot advances the sequence.
        restored.waypoint_idx = 8;
        assert_eq!(store.save(&mut nvram, &restored).expect("fits"), 1);
        assert_eq!(store.peek(&nvram), Some(1));
        assert_eq!(
            store
                .restore::<MissionContext>(&nvram)
                .unwrap()
                .waypoint_idx,
            8
        );
    }

    #[test]
    fn corrupt_crc_is_rejected_not_panicked() {
        let mut nvram = [0u8; 512];
        let mut store = SnapshotStore::new(nvram.len(), 256);
        let ctx = MissionContext {
            mission_state: 1,
            waypoint_idx: 2,
            ops_done: 3,
            last_transition_us: 4,
        };
        store.save(&mut nvram, &ctx).unwrap();

        // Flip one payload bit: the CRC must catch it.
        nvram[HEADER_LEN + 5] ^= 0x10;
        assert_eq!(store.peek(&nvram), None);
        assert_eq!(store.restore::<MissionContext>(&nvram), None);

        // A torn write (all zeros = erased flash) is "absent", not fatal.
        nvram.fill(0);
        assert_eq!(store.peek(&nvram), None);
    }

    #[test]
    fn stale_magic_and_version_rejected() {
        let mut nvram = [0u8; 512];
        let mut store = SnapshotStore::new(nvram.len(), 256);
        let ctx = MissionContext {
            mission_state: 0,
            waypoint_idx: 0,
            ops_done: 0,
            last_transition_us: 0,
        };
        store.save(&mut nvram, &ctx).unwrap();

        nvram[0] = b'X';
        assert_eq!(store.peek(&nvram), None);
        nvram[0] = b'T';
        nvram[11] = 9; // bogus version
        assert_eq!(store.peek(&nvram), None);
    }

    #[test]
    fn oversized_payload_is_reported() {
        let mut small = [0u8; 32]; // header + almost nothing
        let mut store = SnapshotStore::new(small.len(), 64);
        let ctx = MissionContext {
            mission_state: 0,
            waypoint_idx: 0,
            ops_done: 0,
            last_transition_us: 0,
        };
        assert_eq!(store.save(&mut small, &ctx), Err(SnapshotError::TooLarge));
    }
}
