//! TM source packets (CCSDS 132.0-B-3) and loss-tolerant stream extraction.
//!
//! A downlink buffer carries consecutive TM source packets back to back.
//! [`TmStream`] walks them with zero copies: each accepted packet is a
//! borrowed [`PacketView`] over the input. When the stream is corrupted
//! mid-buffer (a burst error the frames layer did not catch), the walk
//! stops and reports the resume point — the caller re-syncs upstream; the
//! parser never panics and never reads past the buffer.

use crate::packet::{PacketType, PacketView, ParseError};

/// Statistics for one [`TmStream`] walk.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StreamStats {
    /// Packets successfully parsed.
    pub packets: u32,
    /// Bytes consumed as valid packets.
    pub bytes_consumed: u32,
    /// Offset at which parsing stopped (`buffer.len()` when the whole
    /// buffer parsed clean).
    pub stopped_at: u32,
}

/// Walk consecutive TM source packets in a byte buffer.
///
/// The iterator yields `Result<PacketView, ParseError>` and stops at the
/// first malformed packet (truncated tail or corrupt header); everything
/// before it was consumed cleanly. Iterate with a `while let` and inspect
/// [`Self::stats`] afterwards.
pub struct TmStream<'a> {
    buf: &'a [u8],
    pos: usize,
    stats: StreamStats,
}

impl<'a> TmStream<'a> {
    /// Start a walk over a downlink buffer.
    #[must_use]
    pub fn new(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            stats: StreamStats::default(),
        }
    }

    /// Walk statistics after (or during) iteration.
    #[must_use]
    pub fn stats(&self) -> StreamStats {
        self.stats
    }

    /// Whether every byte of the buffer was consumed as valid packets.
    #[must_use]
    pub fn drained(&self) -> bool {
        self.pos >= self.buf.len()
    }
}

impl<'a> Iterator for TmStream<'a> {
    type Item = Result<PacketView<'a>, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        if self.pos >= self.buf.len() {
            return None;
        }
        match PacketView::parse(&self.buf[self.pos..]) {
            Ok(v) => {
                // Only telemetry packets ride a TM stream; a TC packet
                // header here means misrouted bytes — stop the walk.
                if v.packet_type != PacketType::Telemetry {
                    self.stats.stopped_at = self.pos as u32;
                    self.pos = self.buf.len();
                    return None;
                }
                let len = v.total_len();
                self.pos += len;
                self.stats.packets += 1;
                self.stats.bytes_consumed += len as u32;
                self.stats.stopped_at = self.pos as u32;
                Some(Ok(v))
            }
            Err(e) => {
                self.stats.stopped_at = self.pos as u32;
                // The walk is over: report once, then drain so a `while
                // let` loop terminates. The caller re-syncs upstream.
                self.pos = self.buf.len();
                Some(Err(e))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::packet::{SequenceFlags, build_header};

    fn tm_packet(apid: u16, seq: u16, payload: &[u8]) -> Vec<u8> {
        let mut p = Vec::with_capacity(6 + payload.len());
        let mut hdr = [0u8; 6];
        build_header(
            &mut hdr,
            PacketType::Telemetry,
            false,
            apid,
            SequenceFlags::Unsegmented,
            seq,
            payload.len(),
        );
        p.extend_from_slice(&hdr);
        p.extend_from_slice(payload);
        p
    }

    #[test]
    fn walk_consecutive_packets() {
        let mut buf = Vec::new();
        for k in 0..5u16 {
            let payload = [k as u8; 16];
            let n = 8 + k as usize;
            buf.extend_from_slice(&tm_packet(100 + k, k, &payload[..n]));
        }
        let mut stream = TmStream::new(&buf);
        let mut seen = Vec::new();
        while let Some(v) = stream.next() {
            seen.push(v.expect("valid").sequence_count);
        }
        assert_eq!(seen, vec![0, 1, 2, 3, 4]);
        assert!(stream.drained());
        let s = stream.stats();
        assert_eq!(s.packets, 5);
        assert_eq!(s.bytes_consumed as usize, buf.len());
        assert_eq!(s.stopped_at as usize, buf.len());
    }

    #[test]
    fn empty_buffer_yields_nothing() {
        let mut stream = TmStream::new(&[]);
        assert!(stream.next().is_none());
        assert!(stream.drained());
        assert_eq!(stream.stats().packets, 0);
    }

    #[test]
    fn truncated_tail_stops_walk_with_resume_point() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&tm_packet(10, 0, &[1; 4]));
        let clean_len = buf.len();
        buf.extend_from_slice(&tm_packet(11, 1, &[2; 16]));
        buf.truncate(clean_len + 8); // tear the second packet
        let mut stream = TmStream::new(&buf);
        let first = stream.next().expect("first packet");
        assert_eq!(first.expect("valid").apid, 10);
        let second = stream.next().expect("second position");
        assert_eq!(second, Err(ParseError::LengthTruncated));
        assert!(stream.next().is_none(), "walk stops after the error");
        let s = stream.stats();
        assert_eq!(s.packets, 1);
        assert_eq!(s.stopped_at as usize, clean_len);
    }

    #[test]
    fn tc_packet_in_tm_stream_stops_the_walk() {
        let mut buf = Vec::new();
        buf.extend_from_slice(&tm_packet(10, 0, &[1; 4]));
        let clean_len = buf.len();
        buf.extend_from_slice(&tm_packet(11, 1, &[2; 4]));
        buf[clean_len] |= 0x10; // flip the type bit → telecommand
        let mut stream = TmStream::new(&buf);
        stream.next().expect("first packet").expect("valid");
        assert!(stream.next().is_none(), "TC in TM stream halts");
        assert_eq!(stream.stats().stopped_at as usize, clean_len);
    }

    #[test]
    fn zero_payload_packets_are_never_emitted() {
        // data-field length 0 is illegal per the standard (length field
        // counts length-1); the encoder wraps and parse rejects it.
        let packet = tm_packet(10, 0, &[]);
        assert_eq!(PacketView::parse(&packet), Err(ParseError::LengthTruncated));
    }
}
