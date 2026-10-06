//! Hardware watchdog management (spec §3): windowed feeding, bite
//! bookkeeping, and reset-cause latching.
//!
//! The watchdog contract on flight hardware is strict: the feed must
//! arrive inside a time *window* — too late bites (reset), too early is
//! also a fault signature (a runaway loop that spins faster than the
//! schedule). [`Watchdog::feed`] encodes exactly that window, tracks the
//! bite history, and hands the boot path the reset cause after a bite.
//! The device touch goes through a [`reg::RegCell`] (the WDT key/enable
//! register); the logic around it runs identically on host tests.

use crate::reg::RegCell;

/// Watchdog configuration.
#[derive(Debug, Clone, Copy)]
pub struct WatchdogConfig {
    /// Feed window: earliest feed after the last one, ticks.
    pub earliest_tick: u64,
    /// Deadline: the feed must arrive by this tick or the dog bites.
    pub deadline_tick: u64,
}

impl WatchdogConfig {
    /// A 1 Hz loop with a 2.5× margin: feed window 0.5–2.5 s.
    #[must_use]
    pub fn loop_1hz() -> Self {
        Self {
            earliest_tick: 500,
            deadline_tick: 2_500,
        }
    }
}

/// Why the dog wants attention.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeedError {
    /// Fed too early — the loop is running away from its schedule.
    TooEarly {
        /// Ticks since the last accepted feed.
        since_last: u64,
    },
    /// The deadline already passed: the bite is scheduled (hardware
    /// would reset within the bite latency).
    BitLate,
}

/// Cause latched after a bite (read by the boot path).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ResetCause {
    /// Power-on.
    PowerOn = 0,
    /// Watchdog deadline missed.
    WatchdogBite = 1,
    /// Software-requested reset.
    Software = 2,
}

/// The watchdog manager.
pub struct Watchdog {
    cfg: WatchdogConfig,
    /// Kick register (the magic-value feed on real devices).
    kick: RegCell,
    last_feed_tick: u64,
    armed: bool,
    /// Total bites (from this manager's history + latched boot cause).
    bites: u32,
    cause: ResetCause,
    /// Feed latencies (EWMA, ticks) for the health telemetry.
    latency_ewma: f32,
}

/// Magic value written to the kick register per feed (device-specific
/// sequence folded to one word for the abstraction).
pub const KICK_MAGIC: u32 = 0xCAFE_F00D;

impl Watchdog {
    /// Arm the watchdog over a kick register.
    #[must_use]
    pub fn arm(cfg: WatchdogConfig, kick: RegCell) -> Self {
        kick.write(KICK_MAGIC);
        Self {
            cfg,
            kick,
            last_feed_tick: 0,
            armed: true,
            bites: 0,
            cause: ResetCause::PowerOn,
            // Fractional window position, 0..=1; start mid-window.
            latency_ewma: 0.5,
        }
    }

    /// Feed at `now_tick`. Returns `Err` on window violations; a
    /// `TooEarly` feed is *not* written to the device (the schedule is
    /// the contract), a `BitLate` feed means the bite already happened —
    /// the caller is booting from after the reset.
    pub fn feed(&mut self, now_tick: u64) -> Result<(), FeedError> {
        if !self.armed {
            return Ok(());
        }
        let since = now_tick.saturating_sub(self.last_feed_tick);
        if since < self.cfg.earliest_tick {
            return Err(FeedError::TooEarly { since_last: since });
        }
        // The window is per-feed: the deadline is relative to the last
        // feed (the classic windowed watchdog), not an absolute clock.
        if since > self.cfg.deadline_tick {
            // The bite occurred at the deadline; this call is the
            // post-reset boot path noticing.
            self.bites += 1;
            self.cause = ResetCause::WatchdogBite;
            self.armed = false;
            return Err(FeedError::BitLate);
        }
        // Latency telemetry: where in the window the feed landed.
        let window = (self.cfg.deadline_tick - self.cfg.earliest_tick) as f32;
        let lateness = (since - self.cfg.earliest_tick) as f32 / window.max(1.0);
        self.latency_ewma = 0.9 * self.latency_ewma + 0.1 * lateness;
        self.kick.write(KICK_MAGIC);
        self.last_feed_tick = now_tick;
        Ok(())
    }

    /// Latch a software reset cause (called on commanded reboots).
    pub fn latch_software_reset(&mut self) {
        self.cause = ResetCause::Software;
        self.armed = false;
    }

    /// Boot-time cause view.
    #[must_use]
    pub fn cause(&self) -> ResetCause {
        self.cause
    }

    /// Bite count.
    #[must_use]
    pub fn bites(&self) -> u32 {
        self.bites
    }

    /// Whether armed (feeding expected).
    #[must_use]
    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Feed-latency EWMA `0..=1` (1 = always at the deadline — a loop
    /// barely keeping up).
    #[must_use]
    pub fn latency(&self) -> f32 {
        self.latency_ewma
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reg::Access;

    fn dog() -> (Watchdog, core::cell::Cell<u32>) {
        let m = core::cell::Cell::new(0u32);
        let w = Watchdog::arm(
            WatchdogConfig::loop_1hz(),
            RegCell::mirror(&m, Access::ReadWrite),
        );
        (w, m)
    }

    #[test]
    fn feeds_inside_the_window_reach_the_device() {
        let (mut w, m) = dog();
        // First feed at 1_000 (window 500..2_500).
        assert!(w.feed(1_000).is_ok());
        assert_eq!(m.get(), KICK_MAGIC);
        // Second feed 1_500 ticks later: inside.
        assert!(w.feed(2_500).is_ok());
        assert!(w.armed());
    }

    #[test]
    fn too_early_feed_is_rejected_and_not_written() {
        let (mut w, m) = dog();
        assert!(w.feed(1_000).is_ok());
        m.set(0);
        assert_eq!(w.feed(1_200), Err(FeedError::TooEarly { since_last: 200 }));
        assert_eq!(m.get(), 0, "early feed never reaches the device");
        // The proper next feed still works.
        assert!(w.feed(1_600).is_ok());
    }

    #[test]
    fn missed_deadline_latches_bite_and_disarms() {
        let (mut w, _m) = dog();
        assert!(w.feed(1_000).is_ok());
        // Next feed arrives past the relative deadline (since 3_000 > 2_500).
        assert_eq!(w.feed(4_000), Err(FeedError::BitLate));
        assert_eq!(w.cause(), ResetCause::WatchdogBite);
        assert_eq!(w.bites(), 1);
        assert!(!w.armed(), "post-bite the manager waits for the reboot");
        // Feeding a disarmed dog is a no-op until a fresh boot rearms.
        let _ = w.feed(3_100);
        assert!(!w.armed());
    }

    #[test]
    fn software_reset_latches_its_cause() {
        let (mut w, _m) = dog();
        w.latch_software_reset();
        assert_eq!(w.cause(), ResetCause::Software);
        assert!(!w.armed());
    }

    #[test]
    fn latency_ewma_tracks_late_feeds() {
        let (mut w, _m) = dog();
        // Feed at the very start of the window twice → latency ~0.05.
        w.feed(500).ok();
        w.feed(1_500).ok();
        w.feed(2_500).ok();
        assert!(
            w.latency() < 0.5,
            "feeds near the window start: {}",
            w.latency()
        );
        // Now feed right at the deadline repeatedly → the EWMA converges
        // toward the 0.75 lateness of deadline-edge feeds.
        let mut tick = 4_500u64;
        for _ in 0..30 {
            w.feed(tick).ok();
            tick += 2_000;
        }
        assert!(w.latency() > 0.5, "feeds at the deadline: {}", w.latency());
    }
}
