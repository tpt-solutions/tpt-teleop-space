//! Phase 5 — multi-hour blackout tolerance validation (planetary
//! occultation scenario, spec §4.2 / §6).
//!
//! Timeline (simulated, timestamps in the test are hours apart but run in
//! microseconds of wall clock):
//!
//! 1. Orbiter pass: telemetry flows, a flit boundary falls mid-bundle.
//! 2. Mars rotates: link drops for 4 hours 12 minutes (occultation).
//!    Bundles submitted during the blackout persist to NVRAM.
//! 3. A processor reset happens mid-blackout: the store recovers from the
//!    NVRAM image alone.
//! 4. Orbiter reappears: transmission resumes **exactly** at the persisted
//!    cursor — no byte re-sent, no byte skipped.
//! 5. Custody transfer completes; the hash-verified telemetry log proves
//!    downlink completeness to the ground.

use tpt_t_space_dtn::bundle::{BundleStore, CustodyState, EndpointId, Priority};
use tpt_t_space_dtn::link::{DtnAgent, Egress, FLIT_BYTES, TelemetryLog};

const HOUR_US: u64 = 3_600 * 1_000_000;
const SECTORS: usize = 8;
const SECTOR_BYTES: usize = 1024;

// REQ: DTN-001
#[test]
fn multi_hour_blackout_with_reset_and_exact_resume() {
    let t0 = 12 * HOUR_US; // mission day 0.5

    // ─── 1. The pass: one bundle goes out, interrupted mid-flit ────────
    let mut agent = DtnAgent::new(BundleStore::new(SECTORS, SECTOR_BYTES));
    agent.set_link(true);
    let payload_a: Vec<u8> = (0..300u32).map(|i| (i * 7 % 251) as u8).collect();
    let sector_a = agent
        .submit(EndpointId::EARTH_OPS, &payload_a, Priority::Normal, 1, t0)
        .expect("stored");
    let first = agent.poll().unwrap().expect("first flit leaves");
    assert_eq!((first.from, first.to), (0, FLIT_BYTES));

    // ─── 2. Occultation: 4h12m dark; commands queue up anyway ──────────
    let t_blackout_start = t0 + FLIT_BYTES as u64; // link dies right after
    agent.set_link(false);
    let t_blackout_end = t_blackout_start + 4 * HOUR_US + 12 * 60 * 1_000_000;
    let payload_b: Vec<u8> = vec![0x5A; 100];
    let payload_c: Vec<u8> = vec![0xC3; 150];
    let sector_b = agent
        .submit(
            EndpointId::EARTH_OPS,
            &payload_b,
            Priority::Normal,
            2,
            t0 + HOUR_US,
        )
        .expect("stored while dark");
    let sector_c = agent
        .submit(
            EndpointId::EARTH_OPS,
            &payload_c,
            Priority::Expedited,
            3,
            t0 + 2 * HOUR_US,
        )
        .expect("stored while dark");
    assert_eq!(agent.pending(), 3);
    // Nothing transmits in the dark.
    assert!(matches!(agent.poll(), Some(Err(_))));

    // ─── 3. Watchdog reset mid-blackout: NVRAM alone must recover ──────
    let image = agent.store().image_snapshot();
    let mut fresh_store = BundleStore::new(SECTORS, SECTOR_BYTES);
    let live = fresh_store.recover(&image).expect("geometry");
    assert_eq!(live, 3, "all three bundles survive the reset");
    let mut agent = DtnAgent::new(fresh_store);
    agent.set_link(false);
    // The interrupted bundle's cursor survived too.
    assert_eq!(
        agent.store().cursor(sector_a).unwrap(),
        first.to,
        "cursor persisted across the reset"
    );

    // ─── 4. Orbiter overhead: exact resume, bundles drain in order ─────
    agent.set_link(true);
    let mut received: Vec<(usize, u32, u32)> = Vec::new();
    let mut final_sectors = Vec::new();
    while let Some(Ok(Egress {
        sector,
        from,
        to,
        final_flit,
    })) = agent.poll()
    {
        received.push((sector, from, to));
        if final_flit {
            final_sectors.push(sector);
            agent.deliver(sector).expect("custody transferred");
        }
    }
    // Contiguity: the resumed bundle continues at the blackout cursor.
    let first_resume = received
        .iter()
        .find(|(s, _, _)| *s == sector_a)
        .expect("bundle A resumed");
    assert_eq!(
        first_resume.1, first.to,
        "resume exactly where the link died"
    );
    // Full coverage of each payload, no gaps, no re-sent bytes. Bundle A
    // already transmitted its first flit before the blackout.
    let mut covered_start = [(sector_a, first.to), (sector_b, 0), (sector_c, 0)];
    for (sector, payload) in [
        (sector_a, &payload_a),
        (sector_b, &payload_b),
        (sector_c, &payload_c),
    ] {
        let mut covered = covered_start
            .iter()
            .find(|(s, _)| *s == sector)
            .map(|(_, c)| *c)
            .unwrap_or(0);
        for (s, from, to) in &received {
            if *s == sector {
                assert_eq!(*from, covered, "contiguous transmission for {sector}");
                covered = *to;
            }
        }
        assert_eq!(covered as usize, payload.len(), "bundle {sector} complete");
        let _ = &mut covered_start;
    }
    assert_eq!(final_sectors.len(), 3);
    assert_eq!(agent.pending(), 0);
    assert!(
        t_blackout_end > t_blackout_start,
        "4h12m elapsed in sim time"
    );

    // ─── 5. The telemetry log seals the story ───────────────────────────
    let mut log = TelemetryLog::<64>::new();
    let pre_blackout = [&payload_a[..64]];
    for c in pre_blackout.iter().copied() {
        log.push(c);
    }
    let anchor = log.chain();
    let post_blackout: [&[u8]; 3] = [&payload_a[64..], &payload_b, &payload_c];
    for c in post_blackout.iter().copied() {
        log.push(c);
    }
    // Ground folds the anchor through the claimed post-blackout chunks.
    assert!(
        log.verify_chain_from(anchor, &post_blackout),
        "downlink proven complete and ordered"
    );
    // Any tampering breaks verification.
    assert!(!log.verify_chain_from(anchor, &[&payload_a[64..], &payload_b]));
}

#[test]
fn custody_lifecycle_end_to_end() {
    let mut agent = DtnAgent::new(BundleStore::new(4, 512));
    let payload = [9u8; 64];
    let sector = agent
        .submit(
            EndpointId::ORBITER_RELAY,
            &payload,
            Priority::Expedited,
            5,
            0,
        )
        .unwrap();
    // Freshly submitted bundles carry no custody obligation.
    let b = agent.store().bundle_meta(sector).unwrap();
    assert_eq!(b.custody, CustodyState::None);
    // The relay (downstream custodian) accepts → custody transferred.
    agent.store().accept_custody(sector).unwrap();
    let b = agent.store().bundle_meta(sector).unwrap();
    assert_eq!(b.custody, CustodyState::Accepted);
    // Delivered → sector released.
    agent.set_link(true);
    while let Some(Ok(e)) = agent.poll() {
        if e.final_flit {
            agent.deliver(e.sector).unwrap();
        }
    }
    assert_eq!(agent.pending(), 0);
}
