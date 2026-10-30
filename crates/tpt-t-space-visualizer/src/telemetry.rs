//! Downlink telemetry for the visualizer (Phase 20): a CCSDS packet stream
//! decoded through the packet-definition table, keeping the newest packet
//! per APID.
//!
//! Pure and window-free, so it is tested without a display. The viewer
//! re-decodes a growing capture as its length changes.

use tpt_t_space_ccsds::defs::{self, Decoded};
use tpt_t_space_ccsds::packet::PacketView;

/// The newest decoded packet of one APID.
#[derive(Debug, Clone, PartialEq)]
pub struct Latest {
    /// Application process identifier.
    pub apid: u16,
    /// Packet name from the definition table.
    pub name: &'static str,
    /// Decoded fields, in payload order.
    pub fields: Vec<Decoded>,
}

/// What a downlink capture contained.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Downlink {
    /// Packets parsed, decodable or not.
    pub packets: usize,
    /// Packets whose APID has no definition, or whose payload is short.
    pub undecoded: usize,
    /// The newest decoded packet per APID, sorted by APID.
    pub latest: Vec<Latest>,
    /// Why parsing stopped early, if it did.
    pub stopped: Option<String>,
}

/// Decode a stream of back-to-back CCSDS Space Packets.
#[must_use]
pub fn decode_stream(bytes: &[u8]) -> Downlink {
    let mut out = Downlink::default();
    let mut off = 0usize;
    while off < bytes.len() {
        let view = match PacketView::parse(&bytes[off..]) {
            Ok(v) => v,
            Err(e) => {
                out.stopped = Some(format!("offset {off:#06x}: {e}"));
                break;
            }
        };
        out.packets += 1;
        let decoded = defs::lookup(view.apid).and_then(|def| {
            let mut fields = Vec::new();
            defs::decode_each(def, view.payload(), |f| fields.push(f))
                .ok()
                .map(|()| (def.name, fields))
        });
        match decoded {
            Some((name, fields)) => keep_latest(&mut out.latest, view.apid, name, fields),
            None => out.undecoded += 1,
        }
        off += view.total_len();
    }
    out
}

fn keep_latest(latest: &mut Vec<Latest>, apid: u16, name: &'static str, fields: Vec<Decoded>) {
    let entry = Latest { apid, name, fields };
    match latest.binary_search_by_key(&apid, |l| l.apid) {
        Ok(i) => latest[i] = entry,
        Err(i) => latest.insert(i, entry),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_t_space_ccsds::packet::{PacketType, SequenceFlags, build_header};

    /// One telemetry packet: a 6-byte header, then `payload`.
    fn packet(apid: u16, count: u16, payload: &[u8]) -> Vec<u8> {
        let mut header = [0u8; 6];
        build_header(
            &mut header,
            PacketType::Telemetry,
            false,
            apid,
            SequenceFlags::Unsegmented,
            count,
            payload.len(),
        );
        let mut out = header.to_vec();
        out.extend_from_slice(payload);
        out
    }

    /// A power packet (APID 0x102, 6 bytes): voltage u16 @0 ×0.01 V,
    /// current i16 @2 ×0.001 A, SoC u8 @4 ×0.5 %, shedding u8 @5.
    fn power(voltage_raw: u16) -> Vec<u8> {
        let v = voltage_raw.to_be_bytes();
        packet(0x102, 1, &[v[0], v[1], 0x00, 0x64, 0x64, 0x00])
    }

    #[test]
    fn a_power_packet_decodes_into_engineering_units() {
        let dl = decode_stream(&power(3000));
        assert_eq!(dl.packets, 1);
        assert_eq!(dl.undecoded, 0);
        let first = &dl.latest[0];
        assert_eq!(first.name, "power");
        assert_eq!(first.fields.len(), 4);
        assert_eq!(first.fields[0].name, "bus_voltage");
        assert!(
            (first.fields[0].value - 30.0).abs() < 1e-9,
            "{:?}",
            first.fields[0]
        );
        assert!(
            (first.fields[1].value - 0.1).abs() < 1e-9,
            "{:?}",
            first.fields[1]
        );
        assert!(
            (first.fields[2].value - 50.0).abs() < 1e-9,
            "{:?}",
            first.fields[2]
        );
    }

    #[test]
    fn only_the_newest_packet_per_apid_is_kept() {
        let mut stream = power(3000);
        stream.extend(power(1000));
        let dl = decode_stream(&stream);
        assert_eq!(dl.packets, 2);
        assert_eq!(dl.latest.len(), 1);
        assert!((dl.latest[0].fields[0].value - 10.0).abs() < 1e-9);
    }

    #[test]
    fn unknown_apids_and_short_payloads_are_counted_not_shown() {
        let mut stream = packet(0x7FF, 1, &[1, 2, 3]);
        stream.extend(packet(0x102, 2, &[0x00]));
        let dl = decode_stream(&stream);
        assert_eq!(dl.packets, 2);
        assert_eq!(dl.undecoded, 2);
        assert!(dl.latest.is_empty());
    }

    #[test]
    fn a_corrupt_header_stops_the_stream_and_says_where() {
        let mut stream = packet(0x102, 1, &[0; 9]);
        stream.extend_from_slice(&[0xFF, 0xFF, 0xC0, 0x00, 0xFF, 0xFF]);
        let dl = decode_stream(&stream);
        assert_eq!(dl.packets, 1);
        assert!(dl.stopped.expect("stops").starts_with("offset 0x"));
    }
}
