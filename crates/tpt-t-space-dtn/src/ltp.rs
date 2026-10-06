//! Licklider Transmission Protocol (LTP): reliable block transfer over the
//! lossy, high-latency space link (spec §4.2 / RFC 5326 flavor).
//!
//! The sender splits a block into segments; every `checkpoint_every`-th
//! segment carries a checkpoint. The receiver answers a checkpoint with a
//! report segment (RS) covering the byte ranges received; the sender
//! applies the report, resumes unacknowledged ranges, and closes the
//! session when the block is fully acknowledged. A session that hears
//! nothing for `timeout_us` re-marks its unresolved range for
//! retransmission — the exact-resume behavior the store-and-forward link
//! relies on across blackouts.
//!
//! Fixed-capacity session table, no allocation, no locks; all timers are
//! injected (`now_us`) so the whole protocol is simulated deterministically
//! in tests (multi-hour blackouts run in microseconds of wall clock).

/// Sender-side LTP session states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionState {
    /// Session slot free.
    Idle,
    /// Block still has untransmitted bytes.
    Transmitting,
    /// All segments sent; waiting for report segments.
    WaitingReport,
    /// Fully acknowledged; session complete.
    Closed,
}

/// One segment the sender hands to the link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Segment {
    /// Session this segment belongs to.
    pub session: u64,
    /// Block offset of the first payload byte.
    pub offset: usize,
    /// Payload length, bytes.
    pub len: usize,
    /// Whether this segment carries a checkpoint (the receiver must reply
    /// with a report segment).
    pub checkpoint: bool,
}

/// Why an LTP operation failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LtpError {
    /// Session table full — the oldest closed session is reclaimed first.
    SessionTableFull,
    /// Unknown or already-closed session.
    NoSuchSession,
}

impl core::fmt::Display for LtpError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let msg = match self {
            LtpError::SessionTableFull => "LTP session table full",
            LtpError::NoSuchSession => "no such LTP session",
        };
        f.write_str(msg)
    }
}

impl std::error::Error for LtpError {}

/// One session in the sender's table.
#[derive(Debug, Clone, Copy)]
struct Session {
    id: u64,
    block_len: usize,
    /// Next untransmitted byte offset.
    sent: usize,
    /// Lowest unacknowledged byte offset (reports advance this).
    acked: usize,
    /// Segments sent since the last checkpoint.
    since_checkpoint: u32,
    last_activity_us: u64,
    state: SessionState,
}

/// The LTP sender engine.
pub struct LtpSender {
    sessions: [Option<Session>; MAX_SESSIONS],
    segment_payload: usize,
    checkpoint_every: u32,
    timeout_us: u64,
    next_slot: usize,
}

/// Maximum concurrent LTP sessions (fixed table, no allocation).
pub const MAX_SESSIONS: usize = 8;

impl LtpSender {
    /// Create a sender: `segment_payload`-byte segments, a checkpoint
    /// every `checkpoint_every` segments, session timeout `timeout_us`.
    #[must_use]
    pub fn new(segment_payload: usize, checkpoint_every: u32, timeout_us: u64) -> Self {
        Self {
            sessions: [None; MAX_SESSIONS],
            segment_payload: segment_payload.max(1),
            checkpoint_every: checkpoint_every.max(1),
            timeout_us,
            next_slot: 0,
        }
    }

    /// Open a session for a `block_len`-byte block. Reclaims a closed or
    /// idle slot first.
    pub fn open(&mut self, id: u64, block_len: usize, now_us: u64) -> Result<(), LtpError> {
        if self.sessions.iter().any(|s| s.map(|x| x.id) == Some(id)) {
            return Ok(()); // idempotent re-open
        }
        let slot = (0..MAX_SESSIONS)
            .find(|i| {
                matches!(
                    self.sessions[*i],
                    None | Some(Session {
                        state: SessionState::Closed,
                        ..
                    }) | Some(Session {
                        state: SessionState::Idle,
                        ..
                    })
                )
            })
            .or_else(|| {
                // Round-robin steal from the oldest waiting session when
                // the table is saturated with live sessions.
                if self.sessions.iter().all(|s| s.is_some()) {
                    let i = self.next_slot % MAX_SESSIONS;
                    self.next_slot += 1;
                    Some(i)
                } else {
                    None
                }
            })
            .ok_or(LtpError::SessionTableFull)?;
        self.sessions[slot] = Some(Session {
            id,
            block_len,
            sent: 0,
            acked: 0,
            since_checkpoint: 0,
            last_activity_us: now_us,
            state: SessionState::Transmitting,
        });
        Ok(())
    }

    /// Produce the next segment to transmit, across all live sessions
    /// (round-robin). `None` when everything is sent and awaiting reports.
    pub fn next_segment(&mut self, now_us: u64) -> Option<Segment> {
        for slot in 0..MAX_SESSIONS {
            let s = self.sessions[slot].as_mut()?;
            if s.state != SessionState::Transmitting || s.sent >= s.block_len {
                if s.state == SessionState::Transmitting && s.sent >= s.block_len {
                    s.state = SessionState::WaitingReport;
                }
                continue;
            }
            let len = self.segment_payload.min(s.block_len - s.sent);
            s.since_checkpoint += 1;
            // The final segment of a block always carries a checkpoint
            // (LTP end-of-block semantics): without it a tail shorter than
            // the checkpoint interval would never be reported.
            let final_segment = s.sent + len >= s.block_len;
            let checkpoint = final_segment || s.since_checkpoint >= self.checkpoint_every;
            if checkpoint {
                s.since_checkpoint = 0;
            }
            s.last_activity_us = now_us;
            let seg = Segment {
                session: s.id,
                offset: s.sent,
                len,
                checkpoint,
            };
            s.sent += len;
            if s.sent >= s.block_len {
                s.state = SessionState::WaitingReport;
            }
            return Some(seg);
        }
        None
    }

    /// Apply a received report segment: bytes `[acked_to, _)` of the
    /// session are acknowledged. Fully-acked sessions close; partially
    /// acked sessions resume transmitting from the first unacked byte.
    pub fn on_report(
        &mut self,
        session: u64,
        acked_to: usize,
        now_us: u64,
    ) -> Result<bool, LtpError> {
        let s = self
            .sessions
            .iter_mut()
            .find(|s| matches!(s, Some(x) if x.id == session))
            .ok_or(LtpError::NoSuchSession)?
            .as_mut()
            .expect("matched Some");
        s.acked = s.acked.max(acked_to.min(s.block_len));
        s.last_activity_us = now_us;
        if s.acked >= s.block_len {
            s.state = SessionState::Closed;
            return Ok(true);
        }
        // Resume: transmit the unacked range again.
        s.sent = s.acked;
        s.state = SessionState::Transmitting;
        s.since_checkpoint = 0;
        Ok(false)
    }

    /// Periodic triage: sessions silent past the timeout resume
    /// retransmission from their checkpoint. Returns the retransmitting
    /// session ids in `out` (bounded); returns the number written.
    pub fn tick(&mut self, now_us: u64, out: &mut [u64]) -> usize {
        let mut n = 0;
        for slot in 0..MAX_SESSIONS {
            let Some(s) = self.sessions[slot].as_mut() else {
                continue;
            };
            if s.state == SessionState::WaitingReport
                && now_us.saturating_sub(s.last_activity_us) >= self.timeout_us
            {
                s.state = SessionState::Transmitting;
                s.sent = s.acked;
                s.since_checkpoint = 0;
                s.last_activity_us = now_us;
                if n < out.len() {
                    out[n] = s.id;
                    n += 1;
                }
            }
        }
        n
    }

    /// State of one session by id.
    #[must_use]
    pub fn state(&self, session: u64) -> Option<SessionState> {
        self.sessions
            .iter()
            .flatten()
            .find(|s| s.id == session)
            .map(|s| s.state)
    }

    /// Acknowledged byte count for one session by id.
    #[must_use]
    pub fn acked(&self, session: u64) -> Option<usize> {
        self.sessions
            .iter()
            .flatten()
            .find(|s| s.id == session)
            .map(|s| s.acked)
    }
}

/// The LTP receiver: tracks received byte ranges per session and issues
/// report segments in response to checkpoints. Ranges are tracked as a
/// fixed step bitmap over the block (`step` bytes per bit).
pub struct LtpReceiver {
    session: u64,
    block_len: usize,
    step: usize,
    bits: [u64; 2], // up to 128 steps per block
    received_max: usize,
    complete: bool,
}

impl LtpReceiver {
    /// Track one session's block (`step` bytes per tracking bit).
    #[must_use]
    pub fn new(session: u64, block_len: usize, step: usize) -> Self {
        Self {
            session,
            block_len,
            step: step.max(1),
            bits: [0; 2],
            received_max: 0,
            complete: false,
        }
    }

    /// Record one received segment range. Call for *every* arriving
    /// segment; report segments are generated at checkpoints by querying
    /// [`Self::contiguous`].
    pub fn receive(&mut self, offset: usize, len: usize) {
        if self.complete {
            return;
        }
        let start = offset / self.step;
        let end = (offset + len).min(self.block_len).div_ceil(self.step);
        for b in start..end.min(128) {
            self.bits[b / 64] |= 1 << (b % 64);
        }
        // Advance the contiguous prefix.
        let mut bit = 0;
        while bit < 128 && self.bits[bit / 64] & (1 << (bit % 64)) != 0 {
            bit += 1;
        }
        self.received_max = (bit * self.step).min(self.block_len);
        if self.received_max >= self.block_len {
            self.complete = true;
        }
    }

    /// The contiguous received prefix — the `(acked_to)` value a report
    /// segment carries when the sender checkpoints.
    #[must_use]
    pub fn contiguous(&self) -> usize {
        self.received_max
    }

    /// Whether the whole block has arrived.
    #[must_use]
    pub fn is_complete(&self) -> bool {
        self.complete
    }

    /// Session id.
    #[must_use]
    pub fn session(&self) -> u64 {
        self.session
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SEG: usize = 100;
    const BLOCK: usize = 1_000;

    fn sender() -> LtpSender {
        LtpSender::new(SEG, 3, 5_000_000)
    }

    #[test]
    fn full_block_transfers_and_closes() {
        let mut s = sender();
        s.open(1, BLOCK, 0).unwrap();
        let mut rx = LtpReceiver::new(1, BLOCK, 50);
        let mut now = 0u64;
        let mut delivered = 0usize;
        while let Some(seg) = s.next_segment(now) {
            delivered += seg.len;
            rx.receive(seg.offset, seg.len);
            if seg.checkpoint {
                s.on_report(1, rx.contiguous(), now).expect("report");
            }
            now += 1_000;
        }
        assert_eq!(delivered, BLOCK);
        assert_eq!(s.state(1), Some(SessionState::Closed));
        assert!(rx.is_complete());
    }

    #[test]
    fn timeout_retransmits_from_checkpoint() {
        let mut s = LtpSender::new(SEG, 1, 1_000_000); // checkpoint every segment
        s.open(2, 300, 0).unwrap();
        let mut rx = LtpReceiver::new(2, 300, 50);
        // All three segments leave; every report is lost (blackout).
        for k in 0..3u64 {
            let seg = s.next_segment(k * 1_000).unwrap();
            assert!(seg.checkpoint, "every segment carries a checkpoint here");
            // The report never makes it back to the sender.
            rx.receive(seg.offset, seg.len);
        }
        assert_eq!(s.state(2), Some(SessionState::WaitingReport));
        // Silence past the timeout (last activity t=2000, timeout 1s) →
        // retransmission triage fires.
        let mut resumed = [0u64; 4];
        let n = s.tick(1_002_000, &mut resumed);
        assert_eq!(n, 1);
        assert_eq!(resumed[0], 2);
        // The retransmitted segment starts from the acked offset (0).
        let seg = s.next_segment(1_002_000).unwrap();
        assert_eq!(seg.offset, 0);
        // A report now completes the block.
        s.on_report(2, 300, 1_002_000).unwrap();
        assert_eq!(s.state(2), Some(SessionState::Closed));
    }

    #[test]
    fn partial_report_resumes_mid_block() {
        let mut s = sender();
        s.open(3, BLOCK, 0).unwrap();
        let mut now = 0u64;
        // First three segments leave (checkpoint on the third).
        let segs = (0..3)
            .map(|_| s.next_segment(now).unwrap())
            .collect::<Vec<_>>();
        now += 1_000;
        // Only bytes 0..200 arrive; the report covers 200.
        assert!(s.on_report(3, 200, now).is_ok());
        // Next segment resumes at 200, not at 300.
        let seg = s.next_segment(now).unwrap();
        assert_eq!(seg.offset, 200);
        let _ = segs;
    }

    #[test]
    fn session_table_is_reclaimed() {
        let mut s = LtpSender::new(10, 1, 1_000);
        for id in 0..MAX_SESSIONS as u64 {
            s.open(id, 100, 0).unwrap();
        }
        // Close one, then opening another must succeed.
        s.on_report(3, 100, 1_000).unwrap();
        assert_eq!(s.state(3), Some(SessionState::Closed));
        s.open(99, 100, 1_000).unwrap();
        assert_eq!(s.state(99), Some(SessionState::Transmitting));
    }

    #[test]
    fn display_messages() {
        assert_eq!(
            LtpError::SessionTableFull.to_string(),
            "LTP session table full"
        );
    }
}
