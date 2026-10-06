//! Wheel health monitoring: motor current trending + visual wear index
//! for impending-failure detection (spec §5.1, Curiosity-wheel-style
//! damage).
//!
//! Two independent evidence streams feed one health verdict:
//!
//! * **motor current**: an EWMA and a peak tracker over per-drive currents;
//!   a rising trend at constant speed means binding (regolith ingress,
//!   bearing wear);
//! * **visual wear index**: the camera inspection score per wheel
//!   (crack/derailment detector output, `0..=1` healthy → damaged).
//!
//! Either stream crossing its threshold (or both warning) flags the wheel
//! for the ground report and constrains the planner's speed.

/// Which wheel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum WheelId {
    /// Front left.
    FrontLeft = 0,
    /// Front right.
    FrontRight = 1,
    /// Middle left.
    MidLeft = 2,
    /// Middle right.
    MidRight = 3,
    /// Rear left.
    RearLeft = 4,
    /// Rear right.
    RearRight = 5,
}

/// Health verdict for one wheel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Health {
    /// Nominal.
    Healthy = 0,
    /// Elevated current or wear visible — watch it.
    Warning = 1,
    /// Impending failure: constrain speed, schedule inspection.
    ImpendingFailure = 2,
}

/// One wheel's monitors.
#[derive(Debug, Clone, Copy)]
pub struct WheelMonitor {
    id: WheelId,
    /// Current EWMA, A.
    current_ewma: f32,
    /// Baseline current at nominal load, A (learned at commissioning).
    baseline_a: f32,
    /// Peak current since last reset, A.
    peak_a: f32,
    /// Visual wear index `0..=1` (1 = pristine).
    wear: f32,
    health: Health,
}

/// Current EWMA above this multiple of baseline → Warning.
pub const CURRENT_WARNING_RATIO: f32 = 1.15;
/// Current EWMA above this multiple of baseline → ImpendingFailure.
pub const CURRENT_FAILURE_RATIO: f32 = 1.35;
/// Wear index below this → Warning.
pub const WEAR_WARNING: f32 = 0.75;
/// Wear index below this → ImpendingFailure.
pub const WEAR_FAILURE: f32 = 0.5;

impl WheelMonitor {
    /// New monitor with a learned baseline.
    #[must_use]
    pub fn new(id: WheelId, baseline_a: f32) -> Self {
        Self {
            id,
            current_ewma: baseline_a,
            baseline_a,
            peak_a: baseline_a,
            wear: 1.0,
            health: Health::Healthy,
        }
    }

    /// Wheel id.
    #[must_use]
    pub fn id(&self) -> WheelId {
        self.id
    }

    /// Feed one drive-current sample at a given speed fraction `0..=1`
    /// (current scales with load; the EWMA runs at matched speeds).
    pub fn step_current(&mut self, amps: f32) {
        self.current_ewma = 0.98 * self.current_ewma + 0.02 * amps;
        if amps > self.peak_a {
            self.peak_a = amps;
        }
        self.reassess();
    }

    /// Feed the visual wear index from the camera inspection pass.
    pub fn set_wear(&mut self, wear: f32) {
        self.wear = wear.clamp(0.0, 1.0);
        self.reassess();
    }

    fn reassess(&mut self) {
        let ratio = self.current_ewma / self.baseline_a.max(1e-3);
        let current_fail = ratio >= CURRENT_FAILURE_RATIO;
        let current_warn = ratio >= CURRENT_WARNING_RATIO;
        let wear_fail = self.wear <= WEAR_FAILURE;
        let wear_warn = self.wear <= WEAR_WARNING;
        self.health = if current_fail || wear_fail {
            Health::ImpendingFailure
        } else if current_warn || wear_warn {
            Health::Warning
        } else {
            Health::Healthy
        };
    }

    /// Verdict.
    #[must_use]
    pub fn health(&self) -> Health {
        self.health
    }

    /// Speed cap from the verdict, m/s (the planner's traction budget).
    #[must_use]
    pub fn speed_cap_mps(&self) -> f32 {
        match self.health {
            Health::Healthy => 1.0,
            Health::Warning => 0.7,
            Health::ImpendingFailure => 0.3,
        }
    }

    /// Current ratio (telemetry).
    #[must_use]
    pub fn current_ratio(&self) -> f32 {
        self.current_ewma / self.baseline_a.max(1e-3)
    }

    /// Wear index (telemetry).
    #[must_use]
    pub fn wear(&self) -> f32 {
        self.wear
    }
}

/// The rover's six-wheel health board.
pub struct WheelHealthBoard {
    wheels: [WheelMonitor; 6],
}

impl WheelHealthBoard {
    /// New board with per-wheel baselines from commissioning.
    #[must_use]
    pub fn new(baselines_a: [f32; 6]) -> Self {
        Self {
            wheels: [
                WheelMonitor::new(WheelId::FrontLeft, baselines_a[0]),
                WheelMonitor::new(WheelId::FrontRight, baselines_a[1]),
                WheelMonitor::new(WheelId::MidLeft, baselines_a[2]),
                WheelMonitor::new(WheelId::MidRight, baselines_a[3]),
                WheelMonitor::new(WheelId::RearLeft, baselines_a[4]),
                WheelMonitor::new(WheelId::RearRight, baselines_a[5]),
            ],
        }
    }

    /// Feed one current sample per wheel (a drive tick).
    pub fn step_currents(&mut self, amps: [f32; 6]) {
        for (w, a) in self.wheels.iter_mut().zip(amps.iter()) {
            w.step_current(*a);
        }
    }

    /// Feed the sol's camera inspection scores.
    pub fn set_wear_scores(&mut self, wear: [f32; 6]) {
        for (w, v) in self.wheels.iter_mut().zip(wear.iter()) {
            w.set_wear(*v);
        }
    }

    /// The fleet-minimum speed cap (constrains the planner).
    #[must_use]
    pub fn speed_cap_mps(&self) -> f32 {
        self.wheels
            .iter()
            .map(WheelMonitor::speed_cap_mps)
            .fold(1.0, f32::min)
    }

    /// Wheels at or beyond Warning, with their verdicts (ground report).
    #[must_use]
    pub fn flagged(&self) -> [(WheelId, Health); 6] {
        let mut out = [(WheelId::FrontLeft, Health::Healthy); 6];
        for (dst, w) in out.iter_mut().zip(self.wheels.iter()) {
            *dst = (w.id(), w.health());
        }
        out
    }

    /// Wheel monitor view.
    #[must_use]
    pub fn wheel(&self, i: usize) -> &WheelMonitor {
        &self.wheels[i]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rising_current_trend_flags_impending_failure() {
        let mut w = WheelMonitor::new(WheelId::MidLeft, 2.0);
        assert_eq!(w.health(), Health::Healthy);
        // Binding: current creeps up over thousands of ticks.
        for k in 0..4000 {
            let load = 2.0 + (k as f32 / 4000.0) * 1.2; // 2.0 → 3.2 A
            w.step_current(load);
        }
        assert_eq!(w.health(), Health::ImpendingFailure);
        assert!(w.current_ratio() >= CURRENT_FAILURE_RATIO);
        assert_eq!(w.speed_cap_mps(), 0.3);
    }

    #[test]
    fn visible_wear_alone_flags() {
        let mut w = WheelMonitor::new(WheelId::FrontRight, 2.0);
        w.set_wear(0.55);
        assert_eq!(w.health(), Health::Warning);
        w.set_wear(0.45);
        assert_eq!(w.health(), Health::ImpendingFailure, "crack progression");
    }

    #[test]
    fn noisy_current_does_not_flag() {
        let mut w = WheelMonitor::new(WheelId::RearLeft, 2.0);
        // Alternating spikes around the baseline (terrain noise).
        for k in 0..4000 {
            let load = if k % 7 == 0 { 2.6 } else { 2.0 };
            w.step_current(load);
        }
        assert_eq!(w.health(), Health::Healthy, "spikes are load, not binding");
    }

    #[test]
    fn board_caps_speed_for_the_worst_wheel() {
        let mut board = WheelHealthBoard::new([2.0; 6]);
        board.step_currents([2.0; 6]);
        assert_eq!(board.speed_cap_mps(), 1.0);
        // Rear-right binds hard.
        for _ in 0..4000 {
            board.step_currents([2.0, 2.0, 2.0, 2.0, 2.0, 3.4]);
        }
        assert_eq!(board.speed_cap_mps(), 0.3);
        let flags = board.flagged();
        assert_eq!(flags[5], (WheelId::RearRight, Health::ImpendingFailure));
        assert_eq!(board.wheel(5).health(), Health::ImpendingFailure);
    }
}
