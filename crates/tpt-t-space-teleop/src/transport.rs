//! The DTN transport wiring (spec §5.2): uplinked command sequences ride
//! custody-transferred bundles; downlink telemetry rides the
//! hash-verified log — the operator's session survives every blackout by
//! construction.

use tpt_t_space_dtn::bundle::StoreError;
use tpt_t_space_dtn::bundle::{EndpointId, Priority};
use tpt_t_space_dtn::link::{DtnAgent, TelemetryLog};

/// The session transport over a DTN agent.
pub struct SessionTransport {
    agent: DtnAgent,
    log: TelemetryLog<64>,
    /// Chain digest the ground last confirmed (anchored handshake).
    ground_anchor: u64,
}

impl SessionTransport {
    /// New session transport.
    #[must_use]
    pub fn new(agent: DtnAgent) -> Self {
        Self {
            agent,
            log: TelemetryLog::new(),
            ground_anchor: 0xcbf2_9ce4_8422_2325, // FNV offset basis
        }
    }

    /// Borrow the DTN agent (link control, store recovery).
    #[must_use]
    pub fn agent(&mut self) -> &mut DtnAgent {
        &mut self.agent
    }

    /// Receive an uplinked command-sequence bundle: store it with
    /// expedited priority and acknowledge custody.
    ///
    /// # Errors
    /// Propagates the bundle-store errors (full / too large).
    pub fn receive_sequence(&mut self, payload: &[u8], now_us: u64) -> Result<usize, StoreError> {
        let sector = self.agent.store().store_bundle(
            &tpt_t_space_dtn::bundle::Bundle::new(
                0, // id assigned by the agent's counter via new()
                EndpointId::SURFACE_ASSET,
                now_us,
                payload.len(),
                Priority::Expedited,
                1, // kind: command sequence
            ),
            payload,
        )?;
        Ok(sector)
    }

    /// Queue a telemetry downlink (normal priority, hash-logged).
    ///
    /// # Errors
    /// Propagates the bundle-store errors.
    pub fn send_telemetry(&mut self, payload: &[u8], now_us: u64) -> Result<usize, StoreError> {
        self.log.push(payload);
        let sector = self.agent.store().store_bundle(
            &tpt_t_space_dtn::bundle::Bundle::new(
                0,
                EndpointId::EARTH_OPS,
                now_us,
                payload.len(),
                Priority::Normal,
                2, // kind: telemetry
            ),
            payload,
        )?;
        Ok(sector)
    }

    /// The chain digest to transmit on the next pass (the ground folds
    /// its anchor through the new chunks to verify completeness).
    #[must_use]
    pub fn log_chain(&self) -> u64 {
        self.log.chain()
    }

    /// The ground-side verification: fold the claimed new chunks onto the
    /// ground's anchor and compare with the asset's chain.
    #[must_use]
    pub fn verify_downlink(&self, new_chunks: &[&[u8]]) -> bool {
        self.log.verify_chain_from(self.ground_anchor, new_chunks)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tpt_t_space_dtn::bundle::BundleStore;

    #[test]
    fn sequences_arrive_by_custody_and_survive_blackouts() {
        let mut t = SessionTransport::new(DtnAgent::new(BundleStore::new(8, 512)));
        // Link is dark (asset on the far side) — the uplink still lands.
        t.agent().set_link(false);
        let seq = b"drive 5m; turn 30; capture cam2";
        let sector = t.receive_sequence(seq, 0).expect("stored while dark");
        // The sequence is resident in NVRAM with cursor 0: execution
        // starts when the asset's loop reads it, not when the link is up.
        assert_eq!(
            t.agent().store().bundle_meta(sector).unwrap().kind,
            1,
            "command-sequence kind"
        );
        assert_eq!(t.agent().store().payload(sector).unwrap(), seq);
    }

    #[test]
    fn telemetry_downlink_is_hash_verified() {
        let mut t = SessionTransport::new(DtnAgent::new(BundleStore::new(8, 512)));
        let t1 = [1u8, 2, 3];
        let t2 = [4u8, 5];
        t.send_telemetry(&t1, 0).unwrap();
        t.send_telemetry(&t2, 1_000).unwrap();
        // Ground folds its anchor through the received chunks.
        assert!(t.verify_downlink(&[&t1, &t2]));
        // A dropped or reordered chunk breaks the proof.
        assert!(!t.verify_downlink(&[&t2, &t1]));
        assert!(!t.verify_downlink(&[&t1]));
    }
}
