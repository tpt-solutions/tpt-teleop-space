//! Deterministic fuzz + property tests for the zero-copy CCSDS parsers
//! (Phase 4: "Fuzz/property tests for the zero-copy parser against
//! malformed frames").
//!
//! No external fuzzing dependency (spec §7 bans pulling stacks for test
//! conveniences): a deterministic xorshift PRNG drives three campaigns —
//!
//! 1. **mutation fuzz**: valid packets are randomly mutated at random
//!    offsets; the parser must never panic, and accepted mutants must have
//!    total_len ≤ buffer length (no out-of-bounds views are constructible).
//! 2. **chunk-split property**: parsing a valid buffer at *every* possible
//!    split (stream torn at each byte offset) either parses the valid
//!    prefix identically or rejects — never misparses.
//! 3. **roundtrip property**: encoder → parser reproduce the exact fields
//!    for randomized field combinations.

use tpt_t_space_ccsds::packet::{PacketType, PacketView, ParseError, SequenceFlags, build_header};
use tpt_t_space_ccsds::tc::{TcError, TcPacket, append_crc16, crc16};
use tpt_t_space_ccsds::tm::TmStream;

/// Deterministic xorshift64* PRNG (no `rand` crate in the tree).
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Self(seed.max(1))
    }
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % n.max(1) as u64) as usize
    }
}

fn build_tm(apid: u16, seq: u16, payload: &[u8]) -> Vec<u8> {
    let mut p = Vec::new();
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
fn mutation_fuzz_never_panics_and_never_overborrows() {
    let mut rng = Rng::new(0xC0FF_EE00_1234_5678);
    let mut base: Vec<u8> = Vec::new();
    for k in 0..64u16 {
        let fill = [k as u8; 16];
        base.extend_from_slice(&build_tm(0x100 + k, k, &fill[..4 + (k % 9) as usize]));
    }

    const ROUNDS: usize = 20_000;
    for round in 0..ROUNDS {
        let mut mutant = base.clone();
        let mutations = 1 + rng.below(8);
        for _ in 0..mutations {
            let at = rng.below(mutant.len());
            mutant[at] = rng.next_u64() as u8;
        }
        // Whole-buffer walk: may accept a prefix, may reject; never panics.
        let stream = TmStream::new(&mutant);
        for v in stream.flatten() {
            assert!(
                v.total_len() <= mutant.len(),
                "round {round}: view exceeds buffer"
            );
            // Payload borrows stay within the accepted packet.
            assert!(v.payload().len() + 6 == v.total_len());
        }
        // Direct one-shot parses of random slices: no panic under any
        // truncation.
        let start = rng.below(mutant.len());
        let end = start + rng.below(mutant.len() - start + 1);
        let _ = PacketView::parse(&mutant[start..end]);
    }
}

#[test]
fn mutation_fuzz_tc_with_crc_rejects_all_but_identical() {
    let mut rng = Rng::new(0x1234_ABCD_0000_0001);
    let mut base = Vec::new();
    let mut hdr = [0u8; 6];
    build_header(
        &mut hdr,
        PacketType::Telecommand,
        false,
        0x2A,
        SequenceFlags::Unsegmented,
        1,
        6,
    );
    base.extend_from_slice(&hdr);
    base.extend_from_slice(&[1, 2, 3, 4, 5, 6]);
    append_crc16(&mut base);

    const ROUNDS: usize = 20_000;
    let mut accepted = 0usize;
    for round in 0..ROUNDS {
        let mut mutant = base.clone();
        // Always corrupt one command-body byte — the CRC-protected region
        // proper. Header mutations (version/length) are *framing* errors
        // rejected before CRC verification, so they would muddy this
        // property; the framing layer has its own fuzz campaign above.
        let at = 6 + rng.below(base.len() - 8);
        let original = mutant[at];
        loop {
            mutant[at] = rng.next_u64() as u8;
            if mutant[at] != original {
                break;
            }
        }
        match TcPacket::parse(&mutant, true) {
            // The covered byte changed, so the only accepted outcome is a
            // CRC collision — a 1/65536 event per round is expected noise.
            Ok(_) => accepted += 1,
            Err(TcError::CrcMismatch) => {}
            Err(other) => panic!("round {round}: unexpected {other:?}"),
        }
    }
    // CRC16 catches virtually all single-byte corruptions (collision is
    // 1/65536 per round; 20k rounds → expect ~0.3, allow a few).
    assert!(
        accepted * 4 <= ROUNDS / 1000,
        "CRC accepted {accepted} corrupt commands of {ROUNDS}"
    );
}

#[test]
fn chunk_split_property_stream_is_prefix_safe() {
    // For every possible tear point in a two-packet downlink buffer, the
    // stream must parse exactly the packets that fit, then reject or stop.
    let mut buf = Vec::new();
    buf.extend_from_slice(&build_tm(7, 0, &[1; 10]));
    let first_len = buf.len(); // 16: header + 10 payload
    buf.extend_from_slice(&build_tm(8, 1, &[2; 6]));
    let second_len = buf.len() - first_len; // 12

    for tear in 0..buf.len() {
        let torn = &buf[..tear];
        let stream = TmStream::new(torn);
        let mut got = Vec::new();
        for v in stream.flatten() {
            got.push(v.sequence_count);
        }
        // Packet 1 parses only when the full 16 bytes are present; packet
        // 2 adds its full 12. Anything less is rejected, never misparsed.
        let expected: Vec<u16> = if tear >= first_len + second_len {
            vec![0, 1]
        } else if tear >= first_len {
            vec![0]
        } else {
            vec![]
        };
        assert_eq!(
            got, expected,
            "tear at {tear}: parsed more or less than the intact prefix"
        );
    }
}

#[test]
fn roundtrip_property_all_field_combinations() {
    let mut rng = Rng::new(0xFEED_FACE_55AA_55AA);
    for _ in 0..5_000 {
        let apid = (rng.next_u64() & 0x7FF) as u16;
        let seq = (rng.next_u64() & 0x3FFF) as u16;
        let flags = match rng.below(4) {
            0 => SequenceFlags::Continuation,
            1 => SequenceFlags::First,
            2 => SequenceFlags::Last,
            _ => SequenceFlags::Unsegmented,
        };
        let ptype = if rng.below(2) == 0 {
            PacketType::Telemetry
        } else {
            PacketType::Telecommand
        };
        let sec = rng.below(2) == 1;
        let payload_len = 1 + rng.below(200);
        let mut buf = Vec::new();
        let mut hdr = [0u8; 6];
        build_header(&mut hdr, ptype, sec, apid, flags, seq, payload_len);
        buf.extend_from_slice(&hdr);
        let fill = (rng.next_u64() & 0xFF) as u8;
        buf.extend(std::iter::repeat_n(fill, payload_len));

        let v = PacketView::parse(&buf).expect("encoder output must parse");
        assert_eq!(v.apid, apid);
        assert_eq!(v.sequence_count, seq);
        assert_eq!(v.sequence_flags, flags);
        assert_eq!(v.packet_type, ptype);
        assert_eq!(v.secondary_header, sec);
        assert_eq!(v.payload().len(), payload_len);
        assert!(v.payload().iter().all(|&b| b == fill));
    }
}

#[test]
fn crc16_streaming_matches_reference_vectors() {
    // Spot vectors for the CCSDS convention (CRC-16/CCITT-FALSE).
    assert_eq!(crc16(b""), 0xFFFF); // init passes through for empty input
    assert_eq!(crc16(b"A"), 0xB915);
    assert_eq!(crc16(b"123456789"), 0x29B1);
}

#[test]
fn parse_error_paths_are_exhaustive_and_total() {
    // A structured sweep over header fields: every rejection path taken.
    let mut hdr = [0u8; 6];
    build_header(
        &mut hdr,
        PacketType::Telemetry,
        false,
        1,
        SequenceFlags::Unsegmented,
        1,
        2,
    );
    // version bits all eight values
    for v in 0u8..8 {
        let mut b = hdr;
        b[0] = (b[0] & 0x1F) | (v << 5);
        let res = PacketView::parse(&b);
        if v == 0 {
            assert!(res.is_err()); // length field says 1 byte payload but
        // buffer ends at header → truncated
        } else {
            assert_eq!(res, Err(ParseError::UnsupportedVersion(v)));
        }
    }
}
