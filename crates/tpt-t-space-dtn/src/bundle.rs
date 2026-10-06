//! BPv7-style bundle records with custody transfer, archived directly into
//! pre-allocated NVRAM sectors via rkyv (spec §4.2).
//!
//! The [`Bundle`] record is the metadata plane: endpoints, timestamps,
//! lifetime, custody state, and the payload *length*. Payload bytes are
//! stored out-of-band, appended after the archived metadata inside the
//! sector. A sector is self-describing and CRC-guarded, so a store scanned
//! after a processor reset revalidates every bundle without trusting RAM:
//!
//! ```text
//! [0..4)   magic "DTN1"          [16..20) CRC32 over meta+payload
//! [4..8)   meta archived len     [20..20+M) rkyv-archived Bundle
//! [8..12)  payload len           [20+M..)  payload bytes
//! [12..16) transmit cursor
//! ```

use rkyv::{Archive, Deserialize, Serialize};
use tpt_t_space_core::crc::crc32;

/// Endpoint identifier: a fixed 16-byte name (readable ASCII, zero padded —
/// the human-readable DTN name registry lives out-of-band).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct EndpointId(
    /// Raw endpoint bytes.
    pub [u8; 16],
);

impl EndpointId {
    /// The ground operations endpoint (Earth).
    pub const EARTH_OPS: EndpointId = EndpointId(*b"earth-ops\0\0\0\0\0\0\0");
    /// The orbiter relay endpoint.
    pub const ORBITER_RELAY: EndpointId = EndpointId(*b"orbiter-relay\0\0\0");
    /// A planetary surface asset (rover/probe).
    pub const SURFACE_ASSET: EndpointId = EndpointId(*b"surface-asset\0\0\0");
}

/// Custody-transfer state of a bundle.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[rkyv(compare(PartialEq), attr(derive(Debug)))]
#[repr(u8)]
pub enum CustodyState {
    /// No custody transfer configured (fire-and-forget).
    None = 0,
    /// This node holds custody: it must keep the bundle until a downstream
    /// custodian accepts (via a custody signal) or lifetime expires.
    HeldBySource = 1,
    /// A downstream custodian accepted custody; the source may expire its
    /// copy.
    Accepted = 2,
}

/// Priority classes from the Bundle Protocol profile.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Priority {
    /// Bulk (telemetry logs, imagery backhaul).
    Bulk = 0,
    /// Normal telemetry.
    Normal = 1,
    /// Operational commands and alerts.
    Expedited = 2,
}

/// The bundle metadata record (rkyv-archivable, fixed-size, no `Drop`).
#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(C)]
pub struct Bundle {
    /// Monotonic bundle id at the source (pairs with `src` for identity).
    pub bundle_id: u64,
    /// Source endpoint.
    pub src: EndpointId,
    /// Destination endpoint.
    pub dst: EndpointId,
    /// Bundle creation timestamp, microseconds.
    pub creation_us: u64,
    /// Lifetime, seconds — expired bundles may be discarded.
    pub lifetime_s: u32,
    /// Payload length, bytes (the bytes live in the sector body).
    pub payload_len: u32,
    /// Delivery priority.
    pub priority: Priority,
    /// Custody-transfer state.
    pub custody: CustodyState,
    /// Application payload kind (command sequence, telemetry, alert, …).
    pub kind: u16,
}

impl Bundle {
    /// Build a bundle originating at the surface asset.
    #[must_use]
    pub fn new(
        bundle_id: u64,
        dst: EndpointId,
        creation_us: u64,
        payload_len: usize,
        priority: Priority,
        kind: u16,
    ) -> Self {
        Self {
            bundle_id,
            src: EndpointId::SURFACE_ASSET,
            dst,
            creation_us,
            lifetime_s: 86_400,
            payload_len: payload_len as u32,
            priority,
            custody: CustodyState::None,
            kind,
        }
    }

    /// Whether the bundle is expired at `now_us`.
    #[must_use]
    pub fn expired(&self, now_us: u64) -> bool {
        now_us.saturating_sub(self.creation_us) > u64::from(self.lifetime_s) * 1_000_000
    }
}

/// Sector header layout constants.
pub mod sector {
    /// Magic at offset 0.
    pub const MAGIC: [u8; 4] = *b"DTN1";
    /// Header size (magic, meta len, payload len, cursor, crc).
    pub const HEADER: usize = 20;
    /// Offset of the archived-metadata length (u32 LE).
    pub const META_LEN: usize = 4;
    /// Offset of the payload length (u32 LE).
    pub const PAYLOAD_LEN: usize = 8;
    /// Offset of the transmit cursor (u32 LE, payload bytes sent).
    pub const CURSOR: usize = 12;
    /// Offset of the CRC32 (u32 LE over meta+payload).
    pub const CRC: usize = 16;
}

/// Why a store operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StoreError {
    /// Every sector is occupied (and none expired).
    Full,
    /// The bundle payload does not fit one sector.
    TooLarge,
    /// The requested sector id is out of range or unallocated.
    BadSector,
    /// The stored record failed its CRC.
    Corrupt,
}

impl core::fmt::Display for StoreError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let msg = match self {
            StoreError::Full => "bundle store full",
            StoreError::TooLarge => "payload exceeds one NVRAM sector",
            StoreError::BadSector => "bad sector id",
            StoreError::Corrupt => "sector CRC mismatch",
        };
        f.write_str(msg)
    }
}

impl std::error::Error for StoreError {}

/// The NVRAM bundle store: `sectors` fixed sectors of `sector_bytes` each,
/// CRC-guarded, allocation-free after construction. The backing image is
/// owned by the store (the NVRAM stand-in); a reboot is modeled by
/// serializing the image out and calling [`BundleStore::recover`] on a
/// fresh store with the same bytes.
pub struct BundleStore {
    sector_bytes: usize,
    sectors: usize,
    /// The whole NVRAM image (one allocation at construction).
    image: Vec<u8>,
    allocated: u64, // bitmap (up to 64 sectors)
    /// Metadata archive scratch: allocated once, reused per store call.
    scratch: rkyv::util::AlignedVec,
    /// Aligned view scratch for validating records out of the (unaligned)
    /// simulation image; single-threaded, so a `RefCell` suffices.
    aligned_view: core::cell::RefCell<rkyv::util::AlignedVec>,
}

impl BundleStore {
    /// Create a store over an erased (all-`0xFF`) NVRAM image.
    #[must_use]
    pub fn new(sectors: usize, sector_bytes: usize) -> Self {
        Self {
            sector_bytes,
            sectors,
            image: vec![0xFF; sectors * sector_bytes],
            allocated: 0,
            scratch: rkyv::util::AlignedVec::new(),
            aligned_view: core::cell::RefCell::new(rkyv::util::AlignedVec::new()),
        }
    }

    /// Attach to an existing NVRAM image after a reboot and revalidate
    /// every sector's CRC; corrupt sectors are reseeded as free. Returns
    /// the number of live bundles recovered.
    ///
    /// # Errors
    /// [`StoreError::BadSector`] when `image` does not match the store
    /// geometry.
    pub fn recover(&mut self, image: &[u8]) -> Result<u32, StoreError> {
        if image.len() != self.sectors * self.sector_bytes {
            return Err(StoreError::BadSector);
        }
        self.image.copy_from_slice(image);
        self.allocated = 0;
        let mut live = 0;
        for s in 0..self.sectors {
            if self.sector_present(s) && self.sector_crc_ok(s) {
                self.allocated |= 1 << s;
                live += 1;
            } else {
                self.erase(s);
            }
        }
        Ok(live)
    }

    /// Number of free sectors.
    #[must_use]
    pub fn free_sectors(&self) -> u32 {
        self.sectors as u32 - self.allocated.count_ones()
    }

    /// Store a bundle: archive the metadata with rkyv into the sector,
    /// append the payload, stamp cursor=0 and the CRC. Returns the sector
    /// id.
    ///
    /// # Errors
    /// [`StoreError::Full`] when no sector is free, [`StoreError::TooLarge`]
    /// when metadata + payload exceed one sector.
    pub fn store_bundle(&mut self, bundle: &Bundle, payload: &[u8]) -> Result<usize, StoreError> {
        if payload.len() != bundle.payload_len as usize {
            return Err(StoreError::TooLarge);
        }
        // Serialize with the scratch held locally so the NVRAM image can
        // be mutated while the archived bytes are read field-disjointly.
        let mut scratch = core::mem::take(&mut self.scratch);
        scratch.clear();
        let written = match rkyv::api::high::to_bytes_in::<_, rkyv::rancor::Error>(bundle, scratch)
        {
            Ok(w) => w,
            Err(_) => {
                self.scratch = rkyv::util::AlignedVec::new();
                return Err(StoreError::TooLarge);
            }
        };
        let meta_len = written.len();
        if sector::HEADER + meta_len + payload.len() > self.sector_bytes {
            self.scratch = written;
            return Err(StoreError::TooLarge);
        }
        let sector = match (0..self.sectors).find(|s| self.allocated & (1 << s) == 0) {
            Some(s) => s,
            None => {
                self.scratch = written;
                return Err(StoreError::Full);
            }
        };
        self.erase(sector);
        let base = sector * self.sector_bytes;
        let img = &mut self.image;
        img[base..base + 4].copy_from_slice(&sector::MAGIC);
        img[base + sector::META_LEN..base + sector::META_LEN + 4]
            .copy_from_slice(&(meta_len as u32).to_le_bytes());
        img[base + sector::PAYLOAD_LEN..base + sector::PAYLOAD_LEN + 4]
            .copy_from_slice(&(payload.len() as u32).to_le_bytes());
        img[base + sector::CURSOR..base + sector::CURSOR + 4].copy_from_slice(&0u32.to_le_bytes());
        let crc = crc32(crc32(0, written.as_slice()), payload);
        img[base + sector::CRC..base + sector::CRC + 4].copy_from_slice(&crc.to_le_bytes());
        for (i, b) in written.iter().enumerate() {
            img[base + sector::HEADER + i] = *b;
        }
        img[base + sector::HEADER + meta_len..base + sector::HEADER + meta_len + payload.len()]
            .copy_from_slice(payload);
        self.allocated |= 1 << sector;
        self.scratch = written;
        Ok(sector)
    }

    /// Borrow the archived metadata of a stored bundle (zero-copy view).
    ///
    /// # Errors
    /// [`StoreError::BadSector`] / [`StoreError::Corrupt`] on bad input.
    /// Read the stored bundle metadata. The fixed-size record (~48 bytes)
    /// deserializes onto the stack; the payload stays a zero-copy borrow
    /// of the sector via [`BundleStore::payload`]. (On target hardware the
    /// NVM mapping is aligned and the archived record maps in place; the
    /// simulation image is a plain byte vector, so validation runs through
    /// a copy.)
    ///
    /// # Errors
    /// [`StoreError::BadSector`] / [`StoreError::Corrupt`] on bad input.
    pub fn bundle_meta(&self, sector: usize) -> Result<Bundle, StoreError> {
        let (meta, _) = self.sector_parts(sector)?;
        // Validate + deserialize through the aligned scratch (rkyv's
        // validation checks alignment; the simulation image is a plain
        // byte vector). bytecheck here is defense in depth for custody
        // decisions — the sector CRC is the first line.
        let mut view = self.aligned_view.borrow_mut();
        view.clear();
        view.extend_from_slice(meta);
        rkyv::api::high::from_bytes::<Bundle, rkyv::rancor::Error>(view.as_slice())
            .map_err(|_| StoreError::Corrupt)
    }

    /// Borrow the stored payload bytes.
    ///
    /// # Errors
    /// [`StoreError::BadSector`] / [`StoreError::Corrupt`] on bad input.
    pub fn payload(&self, sector: usize) -> Result<&[u8], StoreError> {
        let (_, payload) = self.sector_parts(sector)?;
        Ok(payload)
    }

    /// Current transmit cursor (payload bytes already handed to the link).
    ///
    /// # Errors
    /// [`StoreError::BadSector`] on a bad sector id.
    pub fn cursor(&self, sector: usize) -> Result<u32, StoreError> {
        let base = self.sector_base(sector)?;
        Ok(u32::from_le_bytes(
            self.image[base + sector::CURSOR..base + sector::CURSOR + 4]
                .try_into()
                .map_err(|_| StoreError::BadSector)?,
        ))
    }

    /// Persist an advanced transmit cursor (called as payload bytes leave
    /// on the link). This is the exact-resume bookkeeping.
    ///
    /// # Errors
    /// [`StoreError::BadSector`] on a bad sector id.
    pub fn set_cursor(&mut self, sector: usize, cursor: u32) -> Result<(), StoreError> {
        let base = self.sector_base(sector)?;
        self.image[base + sector::CURSOR..base + sector::CURSOR + 4]
            .copy_from_slice(&cursor.to_le_bytes());
        Ok(())
    }

    /// Accept custody of a bundle (rewrite the archived record in place —
    /// the fixed-size record keeps its archived length, CRC refreshed).
    ///
    /// # Errors
    /// Propagated from sector access; [`StoreError::Corrupt`] when the
    /// rewrite changes the metadata size (cannot happen for this record).
    pub fn accept_custody(&mut self, sector: usize) -> Result<(), StoreError> {
        // Copy the record out (validated), flip custody, re-archive in.
        let mut bundle = self.bundle_meta(sector)?;
        bundle.custody = CustodyState::Accepted;
        let meta_len = self.meta_len(sector)?;
        let mut scratch = core::mem::take(&mut self.scratch);
        scratch.clear();
        let written = match rkyv::api::high::to_bytes_in::<_, rkyv::rancor::Error>(&bundle, scratch)
        {
            Ok(w) => w,
            Err(_) => {
                self.scratch = rkyv::util::AlignedVec::new();
                return Err(StoreError::Corrupt);
            }
        };
        if written.len() != meta_len as usize {
            self.scratch = written;
            return Err(StoreError::Corrupt);
        }
        let payload = self.sector_payload_slice(sector)?.to_vec();
        let base = self.sector_base(sector)?;
        self.image[base + sector::HEADER..base + sector::HEADER + written.len()]
            .copy_from_slice(written.as_slice());
        let crc = crc32(0, written.as_slice());
        let crc = crc32(crc, &payload);
        self.image[base + sector::CRC..base + sector::CRC + 4].copy_from_slice(&crc.to_le_bytes());
        self.scratch = written;
        Ok(())
    }

    /// Release a sector (bundle delivered or expired).
    ///
    /// # Errors
    /// [`StoreError::BadSector`] on a bad sector id.
    pub fn free(&mut self, sector: usize) -> Result<(), StoreError> {
        self.sector_base(sector)?;
        self.erase(sector);
        self.allocated &= !(1 << sector);
        Ok(())
    }

    /// Iterate allocated sector ids (unordered).
    pub fn allocated_sectors(&self) -> impl Iterator<Item = usize> + '_ {
        (0..self.sectors).filter(move |s| self.allocated & (1 << s) != 0)
    }

    /// Number of allocated (stored) sectors.
    #[must_use]
    pub fn allocated_count(&self) -> u32 {
        self.allocated.count_ones()
    }

    /// Snapshot the NVRAM image (the reboot boundary: bytes alone).
    #[must_use]
    pub fn image_snapshot(&self) -> Vec<u8> {
        self.image.clone()
    }

    /// Read-only borrow of the raw NVRAM image (for inspection/tests).
    #[must_use]
    pub fn image(&self) -> &[u8] {
        &self.image
    }

    // -- internals --------------------------------------------------------

    fn sector_base(&self, sector: usize) -> Result<usize, StoreError> {
        if sector >= self.sectors {
            return Err(StoreError::BadSector);
        }
        Ok(sector * self.sector_bytes)
    }

    fn meta_len(&self, sector: usize) -> Result<u32, StoreError> {
        let base = self.sector_base(sector)?;
        Ok(u32::from_le_bytes(
            self.image[base + sector::META_LEN..base + sector::META_LEN + 4]
                .try_into()
                .map_err(|_| StoreError::BadSector)?,
        ))
    }

    fn sector_present(&self, sector: usize) -> bool {
        match self.sector_base(sector) {
            Ok(base) => self.image[base..base + 4] == sector::MAGIC,
            Err(_) => false,
        }
    }

    fn sector_crc_ok(&self, sector: usize) -> bool {
        let (meta, payload) = match self.sector_parts(sector) {
            Ok(parts) => parts,
            Err(_) => return false,
        };
        crc32(crc32(0, meta), payload) == self.sector_crc(sector)
    }

    fn sector_crc(&self, sector: usize) -> u32 {
        let base = self.sector_base(sector).unwrap_or(0);
        u32::from_le_bytes(
            self.image[base + sector::CRC..base + sector::CRC + 4]
                .try_into()
                .unwrap_or([0; 4]),
        )
    }

    fn sector_parts(&self, sector: usize) -> Result<(&[u8], &[u8]), StoreError> {
        let base = self.sector_base(sector)?;
        if !self.sector_present(sector) {
            return Err(StoreError::BadSector);
        }
        let meta_len = self.meta_len(sector)? as usize;
        let payload_len = u32::from_le_bytes(
            self.image[base + sector::PAYLOAD_LEN..base + sector::PAYLOAD_LEN + 4]
                .try_into()
                .map_err(|_| StoreError::BadSector)?,
        ) as usize;
        let end = sector::HEADER + meta_len + payload_len;
        if end > self.sector_bytes {
            return Err(StoreError::Corrupt);
        }
        Ok((
            &self.image[base + sector::HEADER..base + sector::HEADER + meta_len],
            &self.image[base + sector::HEADER + meta_len..base + end],
        ))
    }

    fn sector_payload_slice(&self, sector: usize) -> Result<&[u8], StoreError> {
        let (_, payload) = self.sector_parts(sector)?;
        Ok(payload)
    }

    fn erase(&mut self, sector: usize) {
        let base = sector * self.sector_bytes;
        self.image[base..base + self.sector_bytes].fill(0xFF);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SECTORS: usize = 8;
    const SECTOR: usize = 256;

    fn demo_bundle(id: u64, payload_len: usize, now_us: u64) -> Bundle {
        Bundle::new(
            id,
            EndpointId::EARTH_OPS,
            now_us,
            payload_len,
            Priority::Normal,
            1,
        )
    }

    #[test]
    fn store_read_roundtrip() {
        let mut store = BundleStore::new(SECTORS, SECTOR);
        let payload: Vec<u8> = (0..100u32).map(|i| i as u8).collect();
        let b = demo_bundle(1, payload.len(), 1_000);
        let s = store.store_bundle(&b, &payload).expect("stored");
        let meta = store.bundle_meta(s).expect("valid");
        assert_eq!(meta.bundle_id, 1);
        assert_eq!(meta.payload_len, 100);
        assert_eq!(store.payload(s).expect("payload"), payload.as_slice());
        assert_eq!(store.cursor(s).unwrap(), 0);
        assert_eq!(store.free_sectors(), SECTORS as u32 - 1);
    }

    #[test]
    fn cursor_persists_for_exact_resume() {
        let mut store = BundleStore::new(SECTORS, SECTOR);
        let payload = [7u8; 64];
        let s = store
            .store_bundle(&demo_bundle(2, 64, 1_000), &payload)
            .unwrap();
        store.set_cursor(s, 40).unwrap();
        assert_eq!(store.cursor(s).unwrap(), 40);
        // Remaining payload is what a restored link transmits next.
        let rest = &store.payload(s).unwrap()[40..];
        assert_eq!(rest.len(), 24);
    }

    #[test]
    fn reboot_recovery_revalidates_crc() {
        let mut store = BundleStore::new(SECTORS, SECTOR);
        let s0 = store
            .store_bundle(&demo_bundle(1, 10, 1_000), &[1; 10])
            .unwrap();
        let s1 = store
            .store_bundle(&demo_bundle(2, 20, 2_000), &[2; 20])
            .unwrap();
        // Snapshot the NVRAM image (serialize → in-memory copy).
        let image = store.image_snapshot();
        // Corrupt one byte of bundle s0's payload before the "reboot".
        let mut rebooted = image.clone();
        let base0 = s0 * SECTOR + sector::HEADER;
        rebooted[base0 + 50] ^= 0xFF;
        // Fresh store recovers: s0 corrupt → erased, s1 live.
        let mut fresh = BundleStore::new(SECTORS, SECTOR);
        let live = fresh.recover(&rebooted).expect("geometry");
        assert_eq!(live, 1);
        let live_meta = fresh.bundle_meta(s1).unwrap();
        assert_eq!(live_meta.bundle_id, 2);
        // The corrupt sector is free again.
        assert_eq!(fresh.free_sectors(), SECTORS as u32 - 1);

        // An uncorrupted reboot keeps everything.
        let mut fresh2 = BundleStore::new(SECTORS, SECTOR);
        assert_eq!(fresh2.recover(&image).unwrap(), 2);
    }

    #[test]
    fn custody_acceptance_is_recorded_in_place() {
        let mut store = BundleStore::new(SECTORS, SECTOR);
        let mut b = demo_bundle(9, 8, 1_000);
        b.custody = CustodyState::HeldBySource;
        let s = store.store_bundle(&b, &[3; 8]).unwrap();
        store.accept_custody(s).expect("rewrite");
        let meta = store.bundle_meta(s).unwrap();
        assert_eq!(meta.custody, CustodyState::Accepted);
        // CRC still guards the updated record: a fresh recover keeps it.
        let image = store.image_snapshot();
        let mut fresh = BundleStore::new(SECTORS, SECTOR);
        assert_eq!(fresh.recover(&image).unwrap(), 1);
    }

    #[test]
    fn full_store_and_oversize_payload() {
        let mut small = BundleStore::new(2, 256);
        let _ = small
            .store_bundle(&demo_bundle(1, 16, 0), &[1; 16])
            .unwrap();
        let _ = small
            .store_bundle(&demo_bundle(2, 16, 0), &[2; 16])
            .unwrap();
        assert_eq!(
            small.store_bundle(&demo_bundle(3, 16, 0), &[3; 16]),
            Err(StoreError::Full)
        );
        // Oversize for the 64-byte sector.
        let mut b = demo_bundle(4, 200, 0);
        b.payload_len = 200;
        assert_eq!(small.store_bundle(&b, &[0; 200]), Err(StoreError::TooLarge));
    }

    #[test]
    fn expired_bundles_are_detectable() {
        let b = demo_bundle(1, 4, 0);
        assert!(!b.expired(3600 * 1_000_000));
        assert!(b.expired(86_400 * 1_000_000 + 1));
    }

    #[test]
    fn priority_orders_bulk_below_expedited() {
        assert!(Priority::Bulk < Priority::Normal);
        assert!(Priority::Normal < Priority::Expedited);
    }
}
