//! TC packets (CCSDS 232.0-B-3): uplink command parsing with optional
//! CRC-16 error control.
//!
//! A telecommand packet is a Space Packet whose payload carries the command
//! data; uplink integrity is enforced with the CCSDS CRC-16 (polynomial
//! `0x1021`, init `0xFFFF`, no reflection, no final xor — "CRC-16/CCITT-
//! FALSE"). Whether the error control suffix is present is a mission
//! configuration, so [`TcPacket::parse`] takes an explicit flag; a CRC
//! mismatch rejects the command *before* it can reach an actuator.

use crate::packet::{PacketType, PacketView, ParseError};

/// CRC-16-CCITT (poly `0x1021`, init `0xFFFF`) over one byte.
const fn crc16_byte(crc: u16, byte: u8) -> u16 {
    let mut c = crc ^ ((byte as u16) << 8);
    let mut k = 0;
    while k < 8 {
        c = if c & 0x8000 != 0 {
            (c << 1) ^ 0x1021
        } else {
            c << 1
        };
        k += 1;
    }
    c
}

/// CRC-16-CCITT (CCSDS convention) over a byte slice.
#[must_use]
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc = 0xFFFF;
    for &b in data {
        crc = crc16_byte(crc, b);
    }
    crc
}

/// A parsed telecommand packet: borrowed view over the uplink buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TcPacket<'a> {
    /// The underlying Space Packet view (header fields included).
    pub packet: PacketView<'a>,
    /// Command data: the payload with any error-control suffix removed
    /// (4 bytes when `crc_checked`).
    pub command: &'a [u8],
    /// Whether a CRC-16 was present and verified.
    pub crc_checked: bool,
}

/// Why a TC packet was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TcError {
    /// Space Packet layer rejection (length/version).
    Packet(ParseError),
    /// The packet is a telemetry packet, not a telecommand.
    NotTelecommand,
    /// CRC-16 present but wrong — corrupt or injected command.
    CrcMismatch,
    /// CRC configured but fewer than 2 payload bytes exist.
    CrcMissing,
}

impl core::fmt::Display for TcError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            TcError::Packet(e) => write!(f, "packet layer: {e}"),
            TcError::NotTelecommand => f.write_str("packet is not a telecommand"),
            TcError::CrcMismatch => f.write_str("CRC-16 mismatch: corrupt command"),
            TcError::CrcMissing => f.write_str("CRC expected but payload too short"),
        }
    }
}

impl std::error::Error for TcError {}

impl<'a> TcPacket<'a> {
    /// Parse a telecommand packet from `bytes`. When `with_crc`, the last
    /// two payload bytes must be the big-endian CRC-16 over every preceding
    /// byte of the packet (header + payload-minus-crc); the command view
    /// excludes the suffix.
    pub fn parse(bytes: &'a [u8], with_crc: bool) -> Result<TcPacket<'a>, TcError> {
        let packet = PacketView::parse(bytes).map_err(TcError::Packet)?;
        if packet.packet_type != PacketType::Telecommand {
            return Err(TcError::NotTelecommand);
        }
        let payload = packet.payload();
        if with_crc {
            if payload.len() < 2 {
                return Err(TcError::CrcMissing);
            }
            let (body, suffix) = payload.split_at(payload.len() - 2);
            let expect = u16::from_be_bytes([suffix[0], suffix[1]]);
            // CRC covers the full packet up to (not including) the CRC.
            let covered = bytes.len() - 2;
            if crc16(&bytes[..covered]) != expect {
                return Err(TcError::CrcMismatch);
            }
            Ok(TcPacket {
                packet,
                command: body,
                crc_checked: true,
            })
        } else {
            Ok(TcPacket {
                packet,
                command: payload,
                crc_checked: false,
            })
        }
    }
}

/// Append a CCSDS CRC-16 over `packet_so_far` to the buffer (in place —
/// the builder's companion; allocation happens once in the caller).
pub fn append_crc16(packet_so_far: &mut Vec<u8>) {
    let crc = crc16(packet_so_far);
    packet_so_far.extend_from_slice(&crc.to_be_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::{SequenceFlags, build_header};

    fn tc_packet(command: &[u8], crc: bool) -> Vec<u8> {
        let mut p = Vec::new();
        let mut hdr = [0u8; 6];
        build_header(
            &mut hdr,
            PacketType::Telecommand,
            false,
            0x2A,
            SequenceFlags::Unsegmented,
            7,
            command.len() + if crc { 2 } else { 0 },
        );
        p.extend_from_slice(&hdr);
        p.extend_from_slice(command);
        if crc {
            append_crc16(&mut p);
        }
        p
    }

    #[test]
    fn crc_known_vector() {
        // CRC-16/CCITT-FALSE of "123456789" is 0x29B1.
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn clean_tc_with_crc_parses() {
        let buf = tc_packet(&[0xDE, 0xAD, 0xBE, 0xEF], true);
        let tc = TcPacket::parse(&buf, true).expect("valid");
        assert!(tc.crc_checked);
        assert_eq!(tc.command, &[0xDE, 0xAD, 0xBE, 0xEF]);
        assert_eq!(tc.packet.apid, 0x2A);
        assert_eq!(tc.packet.sequence_count, 7);
    }

    #[test]
    fn no_crc_mode_passes_payload_through() {
        let buf = tc_packet(&[1, 2, 3], false);
        let tc = TcPacket::parse(&buf, false).expect("valid");
        assert!(!tc.crc_checked);
        assert_eq!(tc.command, &[1, 2, 3]);
    }

    #[test]
    fn corrupted_command_is_rejected_by_crc() {
        let mut buf = tc_packet(&[1, 2, 3, 4], true);
        let bit = 2 + 1; // flip one bit inside the command
        buf[bit] ^= 0x01;
        assert_eq!(TcPacket::parse(&buf, true), Err(TcError::CrcMismatch));
    }

    #[test]
    fn corrupted_crc_itself_is_rejected() {
        let mut buf = tc_packet(&[1, 2, 3, 4], true);
        let last = buf.len() - 1;
        buf[last] ^= 0x80;
        assert_eq!(TcPacket::parse(&buf, true), Err(TcError::CrcMismatch));
    }

    #[test]
    fn telemetry_packet_rejected_as_tc() {
        let mut buf = Vec::new();
        let mut hdr = [0u8; 6];
        build_header(
            &mut hdr,
            PacketType::Telemetry,
            false,
            1,
            SequenceFlags::Unsegmented,
            1,
            4,
        );
        buf.extend_from_slice(&hdr);
        buf.extend_from_slice(&[0; 4]);
        assert_eq!(TcPacket::parse(&buf, false), Err(TcError::NotTelecommand));
    }

    #[test]
    fn crc_mode_with_short_payload_is_rejected() {
        // data field of 1 byte, crc configured → cannot hold the CRC.
        let mut hdr = [0u8; 6];
        build_header(
            &mut hdr,
            PacketType::Telecommand,
            false,
            1,
            SequenceFlags::Unsegmented,
            1,
            1,
        );
        let mut buf = hdr.to_vec();
        buf.push(0xAA);
        assert_eq!(TcPacket::parse(&buf, true), Err(TcError::CrcMissing));
    }

    #[test]
    fn truncated_packet_is_rejected_not_panicked() {
        let buf = tc_packet(&[9, 9, 9, 9, 9, 9], true);
        for cut in 0..buf.len() {
            let _ = TcPacket::parse(&buf[..cut], true);
            let _ = TcPacket::parse(&buf[..cut], false);
        }
    }

    #[test]
    fn error_display() {
        assert_eq!(
            TcError::CrcMismatch.to_string(),
            "CRC-16 mismatch: corrupt command"
        );
    }
}
