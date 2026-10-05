//! SEU (Single Event Upset) detection and recovery bookkeeping (spec §4.1).
//!
//! Every corrected bit, masked vote, and watchdog reset is recorded —
//! bounded, allocation-free — so the telemetry log tells the ground exactly
//! what the radiation environment did and what the software healed. The
//! log is a fixed-capacity ring: under an SEU storm the oldest events fall
//! off, but the *counters* never lose totals, so the ground health report
//! remains exact even when the event detail truncates.

/// One recorded radiation event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SeuKind {
    /// Single-bit error corrected by the memory scrubber.
    ScrubCorrected,
    /// Word failed ECC (multi-bit) and needs a golden reseed.
    ScrubUncorrectable,
    /// TMR voter masked a diverged core result.
    VoteMasked,
    /// TMR voter saw all three cores disagree.
    VoteDistrusted,
    /// Watchdog reset a flagged core.
    CoreReset,
}

/// A bounded SEU event log with exact totals.
#[derive(Debug, Clone, Copy)]
pub struct SeuBook<const N: usize> {
    ring: [Option<SeuEntry>; N],
    head: usize,
    len: usize,
    dropped: u64,
    totals: [u64; KIND_COUNT],
}

/// Number of [`SeuKind`] discriminators (bookkeeping array size).
const KIND_COUNT: usize = 5;

/// One entry in the SEU log.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeuEntry {
    /// Event timestamp, microseconds.
    pub timestamp_us: u64,
    /// What happened.
    pub kind: SeuKind,
    /// Coarse location (protected-region word index, or core id for TMR
    /// events — interpretation is the reporter's).
    pub whereabout: u32,
}

impl<const N: usize> SeuBook<N> {
    /// Create an empty book.
    #[must_use]
    pub fn new() -> Self {
        Self {
            ring: [None; N],
            head: 0,
            len: 0,
            dropped: 0,
            totals: [0; KIND_COUNT],
        }
    }

    /// Record one event. Timestamps must be monotonic (callers pass the
    /// loop clock). Oldest entries fall off when the ring is full.
    pub fn record(&mut self, timestamp_us: u64, kind: SeuKind, whereabout: u32) {
        let entry = SeuEntry {
            timestamp_us,
            kind,
            whereabout,
        };
        self.ring[(self.head + self.len) % N] = Some(entry);
        if self.len < N {
            self.len += 1;
        } else {
            self.head = (self.head + 1) % N;
            self.dropped += 1;
        }
        self.totals[kind as usize] += 1;
    }

    /// Iterate the retained events, oldest first.
    pub fn iter(&self) -> impl Iterator<Item = SeuEntry> + '_ {
        (0..self.len)
            .map(move |i| self.ring[(self.head + i) % N].expect("retained slots are always Some"))
    }

    /// Number of retained events.
    #[must_use]
    pub fn len(&self) -> usize {
        self.len
    }

    /// Whether nothing has been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Events evicted by ring overflow (the detail that was lost).
    #[must_use]
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Exact lifetime total for one event kind.
    #[must_use]
    pub fn total(&self, kind: SeuKind) -> u64 {
        self.totals[kind as usize]
    }

    /// Sum of all lifetime totals.
    #[must_use]
    pub fn total_all(&self) -> u64 {
        self.totals.iter().sum()
    }
}

impl<const N: usize> Default for SeuBook<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn records_and_iterates_in_order() {
        let mut book = SeuBook::<8>::new();
        book.record(10, SeuKind::ScrubCorrected, 0);
        book.record(20, SeuKind::VoteMasked, 1);
        book.record(30, SeuKind::CoreReset, 2);
        let entries: Vec<_> = book.iter().collect();
        assert_eq!(entries.len(), 3);
        assert_eq!(entries[0].timestamp_us, 10);
        assert_eq!(entries[2].kind, SeuKind::CoreReset);
        assert_eq!(book.total(SeuKind::ScrubCorrected), 1);
        assert_eq!(book.total_all(), 3);
        assert_eq!(book.dropped(), 0);
    }

    #[test]
    fn ring_overflow_drops_oldest_but_keeps_totals() {
        let mut book = SeuBook::<4>::new();
        for t in 0..10u64 {
            book.record(t, SeuKind::ScrubCorrected, t as u32);
        }
        assert_eq!(book.len(), 4);
        assert_eq!(book.dropped(), 6);
        let entries: Vec<_> = book.iter().collect();
        assert_eq!(entries[0].timestamp_us, 6, "oldest retained");
        assert_eq!(entries[3].timestamp_us, 9, "newest retained");
        // Exact totals survive the truncation.
        assert_eq!(book.total(SeuKind::ScrubCorrected), 10);
        assert_eq!(book.total_all(), 10);
    }

    #[test]
    fn mixed_kinds_are_counted_separately() {
        let mut book = SeuBook::<8>::new();
        book.record(1, SeuKind::ScrubUncorrectable, 3);
        book.record(2, SeuKind::VoteDistrusted, 0);
        book.record(3, SeuKind::ScrubUncorrectable, 4);
        assert_eq!(book.total(SeuKind::ScrubUncorrectable), 2);
        assert_eq!(book.total(SeuKind::VoteDistrusted), 1);
        assert_eq!(book.total(SeuKind::VoteMasked), 0);
    }
}
