//! Safe Mode black-box recorder (Phase 20): the last `N` mission events in a
//! fixed ring, serialized behind a CRC for NVRAM persistence and decoded
//! after the fact (`tpts blackbox`).
//!
//! Recording never allocates: the ring is an array sized at compile time.
//! The image is a small header, the records oldest first, and a CRC-32:
//!
//! ```text
//! [0]     magic 0xBB
//! [1]     version 1
//! [2..4]  record count, little-endian u16
//! [4..]   count × 8-byte records: t_s (u32 LE), kind, a, b, reserved (0)
//! [end-4] CRC-32 (little-endian) over every byte before it
//! ```

use std::fmt;

use crate::crc::crc32;
use crate::machine::MissionState;
use crate::safe_mode::SafeModeCause;

const MAGIC: u8 = 0xBB;
const VERSION: u8 = 1;
const HEADER: usize = 4;
const CRC_LEN: usize = 4;
/// Bytes per serialized record.
pub const RECORD_BYTES: usize = 8;

const KIND_TRANSITION: u8 = 1;
const KIND_SAFE_MODE: u8 = 2;
const KIND_MARKER: u8 = 3;

/// Bytes needed to serialize a box that holds `n` records.
#[must_use]
pub const fn encoded_capacity(n: usize) -> usize {
    HEADER + RECORD_BYTES * n + CRC_LEN
}

/// One recorded mission event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// The mission FSM moved between states.
    Transition {
        /// State left.
        from: MissionState,
        /// State entered.
        to: MissionState,
    },
    /// Safe Mode was entered for this cause.
    SafeMode(SafeModeCause),
    /// A subsystem marker with a code the subsystem defines.
    Marker(u8),
}

/// One recorded event with its mission time.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record {
    /// Mission time, seconds (wraps after about 136 years).
    pub t_s: u32,
    /// What happened.
    pub event: Event,
}

impl Default for Record {
    fn default() -> Self {
        Self {
            t_s: 0,
            event: Event::Marker(0),
        }
    }
}

/// Why a black-box image could not be decoded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// Shorter than the header plus CRC, or than its declared record count.
    Short,
    /// The first byte is not the black-box magic.
    Magic,
    /// The image is from a format version this build does not read.
    Version(u8),
    /// The CRC does not match the contents.
    Crc,
    /// A record has an unknown kind or an out-of-range code.
    BadRecord {
        /// Zero-based record index.
        index: usize,
    },
}

impl fmt::Display for DecodeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Short => write!(f, "image is shorter than its declared length"),
            Self::Magic => write!(f, "not a black-box image (bad magic)"),
            Self::Version(v) => write!(f, "unsupported black-box version {v}"),
            Self::Crc => write!(f, "CRC mismatch: the image is corrupt"),
            Self::BadRecord { index } => write!(f, "record {index} has an unknown kind or code"),
        }
    }
}

impl std::error::Error for DecodeError {}

/// Why an image could not be written.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// The output buffer is smaller than [`encoded_capacity`].
    BufferTooSmall {
        /// Bytes needed.
        needed: usize,
    },
}

/// The last `N` events, oldest overwritten first.
#[derive(Debug, Clone)]
pub struct BlackBox<const N: usize> {
    ring: [Record; N],
    next: usize,
    len: usize,
}

impl<const N: usize> Default for BlackBox<N> {
    fn default() -> Self {
        Self {
            ring: [Record::default(); N],
            next: 0,
            len: 0,
        }
    }
}

impl<const N: usize> BlackBox<N> {
    /// An empty box.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Append an event, overwriting the oldest once the ring is full.
    pub fn record(&mut self, t_s: u32, event: Event) {
        if N == 0 {
            return;
        }
        self.ring[self.next] = Record { t_s, event };
        self.next = (self.next + 1) % N;
        self.len = (self.len + 1).min(N);
    }

    /// Records currently held.
    #[must_use]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether nothing has been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Records in chronological order, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = Record> + '_ {
        let start = if N == 0 {
            0
        } else {
            (self.next + N - self.len) % N
        };
        (0..self.len).map(move |k| self.ring[(start + k) % N])
    }

    /// Serialize into `out`, returning the number of bytes written.
    ///
    /// # Errors
    /// [`EncodeError::BufferTooSmall`] when `out` cannot hold the image.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, EncodeError> {
        let needed = encoded_capacity(self.len);
        if out.len() < needed {
            return Err(EncodeError::BufferTooSmall { needed });
        }
        out[0] = MAGIC;
        out[1] = VERSION;
        out[2..4].copy_from_slice(&(self.len as u16).to_le_bytes());
        for (k, rec) in self.iter().enumerate() {
            let at = HEADER + k * RECORD_BYTES;
            let (kind, a, b) = encode_event(rec.event);
            out[at..at + 4].copy_from_slice(&rec.t_s.to_le_bytes());
            out[at + 4] = kind;
            out[at + 5] = a;
            out[at + 6] = b;
            out[at + 7] = 0;
        }
        let body = HEADER + RECORD_BYTES * self.len;
        let crc = crc32(0, &out[..body]);
        out[body..body + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
        Ok(needed)
    }
}

/// The length of the image that starts at the front of an NVRAM region, read
/// from its header. The region may be larger: encoding writes only the image,
/// and whatever follows it is left alone.
///
/// Returns `None` when the region does not start with a black-box header or
/// the image does not fit inside it.
#[must_use]
pub fn image_len(nvram: &[u8]) -> Option<usize> {
    if nvram.len() < HEADER || nvram[0] != MAGIC {
        return None;
    }
    let count = usize::from(u16::from_le_bytes([nvram[2], nvram[3]]));
    let len = encoded_capacity(count);
    (len <= nvram.len()).then_some(len)
}

/// Decode the image at the front of an NVRAM region, as [`decode_each`] does
/// for an exact-length slice, so a box can be restored after a reset.
///
/// # Errors
/// [`DecodeError::Short`] when the region holds no complete image; otherwise
/// the errors of [`decode_each`].
pub fn restore_each(nvram: &[u8], visit: impl FnMut(Record)) -> Result<usize, DecodeError> {
    let len = image_len(nvram).ok_or(DecodeError::Short)?;
    decode_each(&nvram[..len], visit)
}

/// Decode an image written by [`BlackBox::encode`], calling `visit` for each
/// record, oldest first, and returning the record count. Allocation-free.
///
/// The whole image is checked first (header, length, CRC, and every record),
/// so a corrupt image visits nothing.
///
/// # Errors
/// Returns a [`DecodeError`] for a short, mis-tagged, corrupt, or
/// unrecognized image.
pub fn decode_each(bytes: &[u8], mut visit: impl FnMut(Record)) -> Result<usize, DecodeError> {
    let count = check(bytes)?;
    for index in 0..count {
        visit(record_at(bytes, index)?);
    }
    Ok(count)
}

/// Validate the whole image and return its record count.
fn check(bytes: &[u8]) -> Result<usize, DecodeError> {
    if bytes.len() < HEADER + CRC_LEN {
        return Err(DecodeError::Short);
    }
    if bytes[0] != MAGIC {
        return Err(DecodeError::Magic);
    }
    if bytes[1] != VERSION {
        return Err(DecodeError::Version(bytes[1]));
    }
    let count = usize::from(u16::from_le_bytes([bytes[2], bytes[3]]));
    let body = HEADER + RECORD_BYTES * count;
    if bytes.len() != body + CRC_LEN {
        return Err(DecodeError::Short);
    }
    let stored = u32::from_le_bytes([
        bytes[body],
        bytes[body + 1],
        bytes[body + 2],
        bytes[body + 3],
    ]);
    if crc32(0, &bytes[..body]) != stored {
        return Err(DecodeError::Crc);
    }
    for index in 0..count {
        record_at(bytes, index)?;
    }
    Ok(count)
}

/// The record at `index`. Only called on an image that [`check`] accepted.
fn record_at(bytes: &[u8], index: usize) -> Result<Record, DecodeError> {
    let at = HEADER + index * RECORD_BYTES;
    let t_s = u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]]);
    decode_event(bytes[at + 4], bytes[at + 5], bytes[at + 6])
        .map(|event| Record { t_s, event })
        .ok_or(DecodeError::BadRecord { index })
}

/// Stable numeric code for a mission state (the serialized form).
#[must_use]
pub fn state_code(state: MissionState) -> u8 {
    match state {
        MissionState::Cruise => 0,
        MissionState::Approach => 1,
        MissionState::Operations => 2,
        MissionState::SafeMode => 3,
        MissionState::Fault => 4,
        MissionState::Teleop => 5,
    }
}

fn state_from_code(code: u8) -> Option<MissionState> {
    Some(match code {
        0 => MissionState::Cruise,
        1 => MissionState::Approach,
        2 => MissionState::Operations,
        3 => MissionState::SafeMode,
        4 => MissionState::Fault,
        5 => MissionState::Teleop,
        _ => return None,
    })
}

/// Stable code for a Safe Mode cause, plus a detail byte. `Unknown` keeps
/// its domain byte in the detail.
fn cause_code(cause: SafeModeCause) -> (u8, u8) {
    match cause {
        SafeModeCause::ThermalLimit => (0, 0),
        SafeModeCause::PowerSurvivalThreshold => (1, 0),
        SafeModeCause::RadiationAnomaly => (2, 0),
        SafeModeCause::Watchdog => (3, 0),
        SafeModeCause::OperatorCommand => (4, 0),
        SafeModeCause::Unknown(domain) => (5, domain),
    }
}

fn cause_from_code(code: u8, detail: u8) -> Option<SafeModeCause> {
    Some(match code {
        0 => SafeModeCause::ThermalLimit,
        1 => SafeModeCause::PowerSurvivalThreshold,
        2 => SafeModeCause::RadiationAnomaly,
        3 => SafeModeCause::Watchdog,
        4 => SafeModeCause::OperatorCommand,
        5 => SafeModeCause::Unknown(detail),
        _ => return None,
    })
}

fn encode_event(event: Event) -> (u8, u8, u8) {
    match event {
        Event::Transition { from, to } => (KIND_TRANSITION, state_code(from), state_code(to)),
        Event::SafeMode(cause) => {
            let (code, detail) = cause_code(cause);
            (KIND_SAFE_MODE, code, detail)
        }
        Event::Marker(code) => (KIND_MARKER, code, 0),
    }
}

fn decode_event(kind: u8, a: u8, b: u8) -> Option<Event> {
    match kind {
        KIND_TRANSITION => Some(Event::Transition {
            from: state_from_code(a)?,
            to: state_from_code(b)?,
        }),
        KIND_SAFE_MODE => cause_from_code(a, b).map(Event::SafeMode),
        KIND_MARKER => Some(Event::Marker(a)),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decode_all(bytes: &[u8]) -> Result<Vec<Record>, DecodeError> {
        let mut out = Vec::new();
        decode_each(bytes, |r| out.push(r)).map(|_| out)
    }

    fn sample() -> BlackBox<4> {
        let mut bb = BlackBox::<4>::new();
        bb.record(
            10,
            Event::Transition {
                from: MissionState::Cruise,
                to: MissionState::Approach,
            },
        );
        bb.record(20, Event::Marker(7));
        bb.record(30, Event::SafeMode(SafeModeCause::Unknown(9)));
        bb
    }

    #[test]
    fn ring_keeps_the_last_n_in_order() {
        let mut bb = BlackBox::<4>::new();
        for t in 0..6u32 {
            bb.record(t, Event::Marker(t as u8));
        }
        let times: Vec<u32> = bb.iter().map(|r| r.t_s).collect();
        assert_eq!(times, vec![2, 3, 4, 5]);
    }

    #[test]
    fn image_round_trips_exactly() {
        let bb = sample();
        let mut buf = [0u8; 128];
        let n = bb.encode(&mut buf).expect("fits");
        assert_eq!(n, encoded_capacity(3));
        let decoded = decode_all(&buf[..n]).expect("decodes");
        let expected: Vec<Record> = bb.iter().collect();
        assert_eq!(decoded, expected);
    }

    #[test]
    fn corruption_is_caught_by_the_crc() {
        let mut buf = [0u8; 128];
        let n = sample().encode(&mut buf).expect("fits");
        buf[HEADER + 5] ^= 0x01;
        assert_eq!(decode_all(&buf[..n]), Err(DecodeError::Crc));
    }

    #[test]
    fn small_output_buffer_reports_the_need() {
        let mut buf = [0u8; 8];
        assert_eq!(
            sample().encode(&mut buf),
            Err(EncodeError::BufferTooSmall {
                needed: encoded_capacity(3)
            })
        );
    }

    #[test]
    fn header_and_length_are_checked() {
        assert_eq!(decode_all(&[0u8; 3]), Err(DecodeError::Short));
        let mut buf = [0u8; 128];
        let n = sample().encode(&mut buf).expect("fits");
        let mut bad = buf;
        bad[0] = 0x00;
        assert_eq!(decode_all(&bad[..n]), Err(DecodeError::Magic));
        let mut newer = buf;
        newer[1] = 2;
        assert_eq!(decode_all(&newer[..n]), Err(DecodeError::Version(2)));
        assert_eq!(decode_all(&buf[..n - 1]), Err(DecodeError::Short));
    }

    #[test]
    fn unknown_record_kind_is_reported_with_its_index() {
        // A valid-CRC image whose first record has kind 9.
        let mut buf = [0u8; 4 + 8 + 4];
        buf[0] = MAGIC;
        buf[1] = VERSION;
        buf[2] = 1;
        buf[4 + 4] = 9;
        let body = HEADER + RECORD_BYTES;
        let crc = crc32(0, &buf[..body]);
        buf[body..].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_all(&buf), Err(DecodeError::BadRecord { index: 0 }));
    }

    #[test]
    fn image_is_restored_from_a_larger_nvram_region() {
        let bb = sample();
        // An NVRAM region holding the image followed by unrelated data.
        let mut nvram = [0xEEu8; 512];
        let n = bb.encode(&mut nvram).expect("fits");
        assert_eq!(image_len(&nvram), Some(n));
        let mut restored = Vec::new();
        restore_each(&nvram, |r| restored.push(r)).expect("restores");
        let expected: Vec<Record> = bb.iter().collect();
        assert_eq!(restored, expected);
        assert!(
            nvram[n..].iter().all(|b| *b == 0xEE),
            "bytes past the image are untouched"
        );
    }

    #[test]
    fn a_blank_or_truncated_region_restores_nothing() {
        assert_eq!(image_len(&[0xFF; 64]), None);
        let mut nvram = [0u8; 64];
        let n = sample().encode(&mut nvram).expect("fits");
        assert_eq!(
            restore_each(&nvram[..n - 1], |_| {}),
            Err(DecodeError::Short)
        );
    }

    #[test]
    fn zero_capacity_box_records_nothing() {
        let mut bb = BlackBox::<0>::new();
        bb.record(1, Event::Marker(1));
        assert!(bb.is_empty());
        let mut buf = [0u8; 8];
        assert_eq!(bb.encode(&mut buf), Ok(encoded_capacity(0)));
        assert_eq!(decode_all(&buf[..encoded_capacity(0)]), Ok(vec![]));
    }
}
