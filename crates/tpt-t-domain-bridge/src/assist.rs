//! The `request_teleop_assistance()` API surface.
//!
//! A domain's safety subsystem calls this when autonomy can no longer safely
//! continue (stuck in mud, unmapped obstacle, sensor loss — `bridge spec.txt`
//! §5, "Fault Detection & Handover Trigger"). The requester is the domain
//! side of [`SafetyEvent::RequestAssistance`](crate::safety::SafetyEvent):
//!
//! 1. The domain builds an [`AssistRequest`] (reason, urgency, location).
//! 2. [`AssistRequester::request`] stamps it, enforces a re-request cooldown
//!    (so a flapping fault cannot flood the operator queue), and hands it to
//!    the session sink — in production that sink serializes the request into
//!    the `tpt-t-link` uplink; in tests it is a closure.
//!
//! The function is allocation-free and non-blocking: if the sink rejects the
//! request (ring full), the caller gets the request back as an error and may
//! retry on the next tick.

use crate::wire::OperatorId;

/// Why the domain is asking for help.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(u8)]
pub enum AssistReason {
    /// Drivetrain stalled (mud, ditch, embedded obstacle) with no viable plan.
    Stall,
    /// Unmapped obstacle blocks the current path and replanning failed.
    UnmappedObstacle,
    /// Machine left (or is about to leave) the approved work area.
    GeofenceViolation,
    /// A safety-critical sensor degraded below usable confidence.
    SensorFault,
    /// Autonomy confidence dropped below the go/no-go threshold.
    LowAutonomyConfidence,
    /// The (human or AI) supervisor explicitly asked to take over.
    OperatorRequested,
    /// Anything else; `detail` on [`AssistRequest`] carries a coarse code.
    Other,
}

/// How fast an operator must pick the session up.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
pub enum Urgency {
    /// Can wait for the current operator task to finish.
    Routine = 0,
    /// Machine is stopped and unproductive; minutes matter.
    Elevated = 1,
    /// Safety-relevant situation developing; seconds matter.
    Immediate = 2,
}

/// One teleop-assistance request on the wire.
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(C)]
pub struct AssistRequest {
    /// Request timestamp, microseconds.
    pub timestamp_us: u64,
    /// Monotonic sequence number (per [`AssistRequester`], loss/dup detection).
    pub seq: u32,
    /// Why help is needed.
    pub reason: AssistReason,
    /// How fast an operator must respond.
    pub urgency: Urgency,
    /// Machine latitude, degrees WGS-84 (`0.0` when unknown).
    pub lat_deg: f64,
    /// Machine longitude, degrees WGS-84 (`0.0` when unknown).
    pub lon_deg: f64,
    /// Domain-specific detail code (interpretation is domain's own).
    pub detail: u16,
    /// Operator/AI session this request is bound to, when re-requesting
    /// inside an active session (otherwise [`OperatorId::LEGACY_ANON`]).
    pub operator_id: OperatorId,
}

/// The sink every accepted request is delivered to. In production this
/// publishes onto the teleop uplink; the domain never opens a network
/// connection itself.
pub trait AssistSink {
    /// Deliver one request. Return `false` to signal rejection (e.g. ring
    /// full); the requester will surface that to the caller.
    fn deliver(&mut self, request: AssistRequest) -> bool;
}

impl<F: FnMut(AssistRequest) -> bool> AssistSink for F {
    fn deliver(&mut self, request: AssistRequest) -> bool {
        self(request)
    }
}

/// Errors from [`AssistRequester::request`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AssistError {
    /// The request was suppressed by the re-request cooldown window.
    Cooldown,
    /// The sink refused delivery (queue full); retry next tick.
    SinkRejected,
}

impl core::fmt::Display for AssistError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            AssistError::Cooldown => f.write_str("suppressed: within re-request cooldown"),
            AssistError::SinkRejected => f.write_str("sink rejected request (queue full)"),
        }
    }
}

impl std::error::Error for AssistError {}

/// Domain-side gate in front of the teleop uplink.
///
/// Holds the request sequence counter and the cooldown state. `request` is
/// the `request_teleop_assistance()` entry point wired to fault detection
/// (e.g. `tpt-t-space-thermal` leak detection → `tpt-t-space-teleop`).
#[derive(Debug, Clone, Copy)]
pub struct AssistRequester {
    next_seq: u32,
    last_sent_us: u64,
    cooldown_us: u64,
}

impl AssistRequester {
    /// Create a requester with the given minimum spacing between delivered
    /// requests (microseconds). Typical value: one second (`1_000_000`).
    pub fn new(cooldown_us: u64) -> Self {
        Self {
            next_seq: 0,
            last_sent_us: 0,
            cooldown_us,
        }
    }

    /// `request_teleop_assistance()`: build, stamp, and deliver one request.
    ///
    /// Returns the delivered request on success. Duplicate suppression is
    /// time-based only — a *changed* situation should be a new request, so
    /// callers retrying the same fault simply wait out the cooldown.
    pub fn request<S: AssistSink>(
        &mut self,
        now_us: u64,
        mut req: AssistRequest,
        sink: &mut S,
    ) -> Result<AssistRequest, AssistError> {
        if self.next_seq > 0 && now_us.saturating_sub(self.last_sent_us) < self.cooldown_us {
            return Err(AssistError::Cooldown);
        }
        req.timestamp_us = now_us;
        req.seq = self.next_seq;
        self.next_seq += 1;
        self.last_sent_us = now_us;
        if !sink.deliver(req) {
            return Err(AssistError::SinkRejected);
        }
        Ok(req)
    }

    /// Number of requests delivered so far (sequence number of the next one).
    pub fn delivered(&self) -> u32 {
        self.next_seq
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_is_stamped_and_delivered() {
        let mut requester = AssistRequester::new(1_000_000);
        let mut delivered = Vec::new();
        let mut sink = |r: AssistRequest| {
            delivered.push(r);
            true
        };

        let req = AssistRequest {
            timestamp_us: 0, // overwritten by requester
            seq: 999,        // overwritten by requester
            reason: AssistReason::Stall,
            urgency: Urgency::Immediate,
            lat_deg: -40.5,
            lon_deg: 175.25,
            detail: 0,
            operator_id: OperatorId::LEGACY_ANON,
        };

        let out = requester
            .request(5_000_000, req, &mut sink)
            .expect("first request delivered");
        assert_eq!(out.timestamp_us, 5_000_000);
        assert_eq!(out.seq, 0);
        assert_eq!(out.reason, AssistReason::Stall);
        assert_eq!(delivered.len(), 1);
        assert_eq!(requester.delivered(), 1);
    }

    #[test]
    fn cooldown_suppresses_flapping_fault() {
        let mut requester = AssistRequester::new(1_000_000);
        let count = core::cell::Cell::new(0usize);
        let mut sink = |_r: AssistRequest| {
            count.set(count.get() + 1);
            true
        };
        let req = AssistRequest {
            timestamp_us: 0,
            seq: 0,
            reason: AssistReason::LowAutonomyConfidence,
            urgency: Urgency::Elevated,
            lat_deg: 0.0,
            lon_deg: 0.0,
            detail: 0,
            operator_id: OperatorId::LEGACY_ANON,
        };

        requester.request(0, req, &mut sink).expect("first ok");
        // 100 ms later: suppressed.
        assert_eq!(
            requester.request(100_000, req, &mut sink),
            Err(AssistError::Cooldown)
        );
        assert_eq!(count.get(), 1);
        // After the cooldown window: delivered.
        requester
            .request(1_000_000, req, &mut sink)
            .expect("second ok");
        assert_eq!(count.get(), 2);
        assert_eq!(requester.delivered(), 2);
    }

    #[test]
    fn sink_rejection_is_reported_and_can_retry() {
        let mut requester = AssistRequester::new(0); // no cooldown for the test
        let mut full = true;
        let mut sink = |_r: AssistRequest| {
            let accept = !full;
            full = false;
            accept
        };
        let req = AssistRequest {
            timestamp_us: 0,
            seq: 0,
            reason: AssistReason::UnmappedObstacle,
            urgency: Urgency::Immediate,
            lat_deg: 1.0,
            lon_deg: 2.0,
            detail: 0,
            operator_id: OperatorId::LEGACY_ANON,
        };

        assert_eq!(
            requester.request(10, req, &mut sink),
            Err(AssistError::SinkRejected)
        );
        // Sequence still advanced so the operator can detect the dropped one.
        assert_eq!(requester.delivered(), 1);
        let out = requester.request(20, req, &mut sink).expect("retry ok");
        assert_eq!(out.seq, 1);
    }

    #[test]
    fn urgency_orders() {
        assert!(Urgency::Routine < Urgency::Elevated);
        assert!(Urgency::Elevated < Urgency::Immediate);
    }
}
