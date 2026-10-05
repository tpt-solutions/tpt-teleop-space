//! The CCSDS Space Packet primary header (CCSDS 133.0-B-2) — the 6-byte
//! header shared by telemetry and telecommand packets, parsed zero-copy.
//!
//! ```text
//! byte 0     : [version:3][type:1][secondary-header:1][apid_hi:3]
//! byte 1     : [apid_lo:8]
//! byte 2     : [sequence-flags:2][sequence-count_hi:6]
//! byte 3     : [sequence-count_lo:8]
//! byte 4..6  : data length (u16 BE): data-field length minus one
//! ```
//!
//! [`PrimaryHeader::parse`] validates the fixed invariants (version 0,
//! declared length fits the buffer) and returns the decoded fields; the
//! payload is a borrowed sub-slice of the caller's buffer — no copy, no
//! allocation, no panic on any input.

/// CCSDS packet type: telemetry (downlink) or telecommand (uplink).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum PacketType {
    /// Telemetry packet (type bit 0).
    Telemetry = 0,
    /// Telecommand packet (type bit 1).
    Telecommand = 1,
}

/// Packet sequence flags (how this packet splits its user data).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum SequenceFlags {
    /// Continuation segment of user data.
    Continuation = 0,
    /// First segment of user data.
    First = 1,
    /// Last segment of user data.
    Last = 2,
    /// Unsegmented (complete user data).
    Unsegmented = 3,
}

/// Why a primary header (or packet) was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    /// Fewer bytes than the header requires.
    TooShort,
    /// Unsupported header version (only version 0 exists today).
    UnsupportedVersion(u8),
    /// The declared data length does not fit the provided buffer.
    LengthTruncated,
}

impl core::fmt::Display for ParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            ParseError::TooShort => f.write_str("buffer shorter than the 6-byte primary header"),
            ParseError::UnsupportedVersion(v) => write!(f, "unsupported header version {v}"),
            ParseError::LengthTruncated => f.write_str("declared data length exceeds the buffer"),
        }
    }
}

impl std::error::Error for ParseError {}

/// The decoded CCSDS Space Packet primary header plus a borrowed view of
/// the payload. `'a` borrows the input buffer; nothing is copied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PacketView<'a> {
    bytes: &'a [u8],
    /// Header version (3 bits) — 0 in every CCSDS release to date.
    pub version: u8,
    /// Packet type (telemetry / telecommand).
    pub packet_type: PacketType,
    /// Whether a packet secondary header follows the primary header.
    pub secondary_header: bool,
    /// Application process identifier (11 bits).
    pub apid: u16,
    /// Sequence flags (segmentation).
    pub sequence_flags: SequenceFlags,
    /// Packet sequence count (14 bits, wraps per APID).
    pub sequence_count: u16,
}

impl<'a> PacketView<'a> {
    /// The primary header is exactly 6 bytes.
    pub const HEADER_LEN: usize = 6;

    /// Parse the primary header from the start of `bytes` and borrow the
    /// packet payload (primary header + data field bounded by the declared
    /// length). Total packet length per the standard is `data_len + 1`
    /// bytes *including* the header, so the payload view is
    /// `6..(data_len + 1)` clipped to reality: a declared length shorter
    /// than the header is rejected.
    pub fn parse(bytes: &'a [u8]) -> Result<PacketView<'a>, ParseError> {
        if bytes.len() < Self::HEADER_LEN {
            return Err(ParseError::TooShort);
        }
        let b0 = bytes[0];
        let b1 = bytes[1];
        let version = b0 >> 5;
        if version != 0 {
            return Err(ParseError::UnsupportedVersion(version));
        }
        let packet_type = if b0 & 0x10 != 0 {
            PacketType::Telecommand
        } else {
            PacketType::Telemetry
        };
        let secondary_header = b0 & 0x08 != 0;
        let apid = ((u16::from(b0 & 0x07)) << 8) | u16::from(b1);
        let seq_bytes = u16::from_be_bytes([bytes[2], bytes[3]]);
        let sequence_flags_raw = (seq_bytes >> 14) as u8;
        let sequence_flags = match sequence_flags_raw {
            0 => SequenceFlags::Continuation,
            1 => SequenceFlags::First,
            2 => SequenceFlags::Last,
            _ => SequenceFlags::Unsegmented,
        };
        let sequence_count = seq_bytes & 0x3FFF;
        let data_len = u16::from_be_bytes([bytes[4], bytes[5]]) as usize;
        // The standard stores data-field length minus one: total packet
        // length = header + (data_len + 1). A declared length beyond the
        // buffer means a truncated transfer — reject, never read OOB.
        let total = Self::HEADER_LEN + data_len + 1;
        if total > bytes.len() {
            return Err(ParseError::LengthTruncated);
        }
        Ok(PacketView {
            bytes: &bytes[..total],
            version,
            packet_type,
            secondary_header,
            apid,
            sequence_flags,
            sequence_count,
        })
    }

    /// The payload (data field): everything after the primary header,
    /// optionally starting with the packet secondary header.
    #[must_use]
    pub fn payload(&self) -> &'a [u8] {
        &self.bytes[Self::HEADER_LEN..]
    }

    /// The complete packet bytes (header + data field), as transmitted.
    #[must_use]
    pub fn raw(&self) -> &'a [u8] {
        self.bytes
    }

    /// Total packet length in bytes (primary header + data field).
    #[must_use]
    pub fn total_len(&self) -> usize {
        self.bytes.len()
    }

    /// APID including the upper type/secondary bits, in wire form — useful
    /// when rebuilding a header byte-for-byte.
    #[must_use]
    pub fn apid_raw(&self) -> u16 {
        self.apid
    }
}

/// Build a primary header into `out` (exactly 6 bytes) for a packet whose
/// data field (payload after the header) is `payload_len` bytes. The
/// standard stores data-field length minus one, so `payload_len == 0`
/// encodes as the illegal `0xFFFF` (parse rejects it). Used by tests and
/// TC builders; the hot downlink path only ever parses.
pub fn build_header(
    out: &mut [u8; 6],
    packet_type: PacketType,
    secondary_header: bool,
    apid: u16,
    sequence_flags: SequenceFlags,
    sequence_count: u16,
    payload_len: usize,
) {
    let type_bit = match packet_type {
        PacketType::Telemetry => 0u8,
        PacketType::Telecommand => 0x10,
    };
    let sec_bit = u8::from(secondary_header) << 3;
    out[0] = type_bit | sec_bit | ((apid >> 8) as u8 & 0x07);
    out[1] = apid as u8;
    let flags = match sequence_flags {
        SequenceFlags::Continuation => 0u16,
        SequenceFlags::First => 1,
        SequenceFlags::Last => 2,
        SequenceFlags::Unsegmented => 3,
    };
    let seq = (flags << 14) | (sequence_count & 0x3FFF);
    out[2] = (seq >> 8) as u8;
    out[3] = seq as u8;
    let field = (payload_len as u16).wrapping_sub(1);
    out[4] = (field >> 8) as u8;
    out[5] = field as u8;
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header_for(payload_len: usize) -> ([u8; 6], Vec<u8>) {
        let mut out = [0u8; 6];
        build_header(
            &mut out,
            PacketType::Telemetry,
            false,
            100,
            SequenceFlags::Unsegmented,
            42,
            payload_len,
        );
        let mut packet = out.to_vec();
        packet.extend(std::iter::repeat_n(0xABu8, payload_len));
        (out, packet)
    }

    #[test]
    fn roundtrip_fields() {
        let mut out = [0u8; 6];
        build_header(
            &mut out,
            PacketType::Telecommand,
            true,
            0x7FF,
            SequenceFlags::First,
            0x3FFF,
            8,
        );
        let mut buf = out.to_vec();
        buf.extend_from_slice(&[0u8; 8]);
        let v = PacketView::parse(&buf).expect("valid");
        assert_eq!(v.packet_type, PacketType::Telecommand);
        assert!(v.secondary_header);
        assert_eq!(v.apid, 0x7FF);
        assert_eq!(v.sequence_flags, SequenceFlags::First);
        assert_eq!(v.sequence_count, 0x3FFF);
        assert_eq!(v.total_len(), 6 + 8);
        assert_eq!(v.payload().len(), 8);
    }

    #[test]
    fn zero_length_payload_is_illegal() {
        // data length field 0xFFFF..wrapping; payload_len 0 → field wraps
        // to 0xFFFF which parse must reject as truncated.
        let mut out = [0u8; 6];
        build_header(
            &mut out,
            PacketType::Telemetry,
            false,
            1,
            SequenceFlags::Unsegmented,
            1,
            0,
        );
        assert_eq!(PacketView::parse(&out), Err(ParseError::LengthTruncated));
    }

    #[test]
    fn single_byte_payload_is_minimal_packet() {
        let (_, packet) = header_for(1);
        assert_eq!(packet.len(), 7);
        let v = PacketView::parse(&packet).expect("valid");
        assert_eq!(v.payload().len(), 1);
    }

    #[test]
    fn too_short_buffers_rejected() {
        let (_, packet) = header_for(4);
        for cut in 0..6 {
            assert_eq!(
                PacketView::parse(&packet[..cut]),
                Err(ParseError::TooShort),
                "cut at {cut}"
            );
        }
    }

    #[test]
    fn declared_length_must_fit_buffer() {
        let (_, packet) = header_for(16);
        // Corrupt the length field to claim more data than present.
        let mut truncated = packet.clone();
        truncated.truncate(6 + 8);
        let stored = ((16u16).wrapping_sub(1)).to_be_bytes();
        truncated[4] = stored[0];
        truncated[5] = stored[1];
        assert_eq!(
            PacketView::parse(&truncated),
            Err(ParseError::LengthTruncated)
        );
    }

    #[test]
    fn version_bits_rejected() {
        let (_, packet) = header_for(2);
        let mut bad = packet.clone();
        bad[0] |= 0xE0; // version 7
        assert_eq!(
            PacketView::parse(&bad),
            Err(ParseError::UnsupportedVersion(7))
        );
    }

    #[test]
    fn display_messages() {
        assert_eq!(
            ParseError::TooShort.to_string(),
            "buffer shorter than the 6-byte primary header"
        );
    }
}
