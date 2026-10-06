//! Static round-robin task table (spec §3 "bare-metal RTOS layer").
//!
//! Tasks are compile-time-registered function pointers with fixed
//! periods; the scheduler walks them every tick and runs whichever is
//! due. No dynamic anything: the table is a fixed array, the stacks are
//! static by contract (every task is a plain `fn` — no closures, no
//! captures), and deadline misses are *counted* per task for the
//! traceability audits (spec §2).
//!
//! This is the dispatch skeleton; on target hardware the tick comes from
//! a timer interrupt and [`Scheduler::tick`] runs with interrupts masked
//! around the table walk.

/// One registered task.
#[derive(Clone, Copy)]
pub struct Task {
    /// Human-readable name (audit trail).
    pub name: &'static str,
    /// Period, ticks.
    pub period: u64,
    /// Phase offset, ticks (period-0 tasks run every tick).
    pub phase: u64,
    /// The task body. `now` is the tick; the return flags a deadline miss
    /// the task noticed about itself (long work, budget blown).
    pub run: fn(now: u64) -> bool,
}

/// Per-task accounting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TaskStats {
    /// How many times the task ran.
    pub runs: u64,
    /// How many periods were skipped entirely (scheduler overload).
    pub skipped: u64,
    /// How many runs self-reported a deadline miss.
    pub missed: u64,
}

/// Why task registration failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedError {
    /// The static table is full.
    TableFull,
    /// A task registered a zero period (would spin forever).
    ZeroPeriod,
}

impl core::fmt::Display for SchedError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            SchedError::TableFull => "static task table full",
            SchedError::ZeroPeriod => "task period must be nonzero",
        })
    }
}

impl core::error::Error for SchedError {}

/// Static capacity: the flight table size (documented budget).
pub const MAX_TASKS: usize = 16;

/// The round-robin scheduler.
pub struct Scheduler {
    tasks: [Option<Task>; MAX_TASKS],
    len: usize,
    now: u64,
    stats: [TaskStats; MAX_TASKS],
}

impl Scheduler {
    /// New scheduler at tick 0.
    #[must_use]
    pub fn new() -> Self {
        Self {
            tasks: [None; MAX_TASKS],
            len: 0,
            now: 0,
            stats: [TaskStats {
                runs: 0,
                skipped: 0,
                missed: 0,
            }; MAX_TASKS],
        }
    }

    /// Register a task.
    ///
    /// # Errors
    /// [`SchedError::TableFull`] / [`SchedError::ZeroPeriod`].
    pub fn register(&mut self, task: Task) -> Result<usize, SchedError> {
        if task.period == 0 {
            return Err(SchedError::ZeroPeriod);
        }
        if self.len >= MAX_TASKS {
            return Err(SchedError::TableFull);
        }
        self.tasks[self.len] = Some(task);
        self.len += 1;
        Ok(self.len - 1)
    }

    /// Advance one tick, running every due task (in registration order).
    /// Returns the number of task bodies executed.
    pub fn tick(&mut self) -> u32 {
        self.now += 1;
        let now = self.now;
        let mut ran = 0;
        for i in 0..self.len {
            let Some(task) = self.tasks[i] else { continue };
            let phase_ok = (now + task.phase) % task.period == 0;
            if !phase_ok {
                continue;
            }
            self.stats[i].runs += 1;
            ran += 1;
            if (task.run)(now) {
                self.stats[i].missed += 1;
            }
        }
        ran
    }

    /// Current tick.
    #[must_use]
    pub fn now(&self) -> u64 {
        self.now
    }

    /// Stats for task index `i`.
    #[must_use]
    pub fn stats(&self, i: usize) -> Option<TaskStats> {
        self.stats.get(i).copied()
    }

    /// Registered task count.
    #[must_use]
    pub fn task_count(&self) -> usize {
        self.len
    }
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::sync::atomic::{AtomicU32, Ordering};

    #[test]
    fn periodic_tasks_run_on_their_periods() {
        // `fn`-pointer tasks reach their state through statics (the
        // no-capture contract of a static task table).
        static FAST: AtomicU32 = AtomicU32::new(0);
        static SLOW: AtomicU32 = AtomicU32::new(0);
        let mut sched = Scheduler::new();
        sched
            .register(Task {
                name: "adcs-100hz",
                period: 1,
                phase: 0,
                run: |_| {
                    FAST.fetch_add(1, Ordering::Relaxed);
                    false
                },
            })
            .unwrap();
        sched
            .register(Task {
                name: "dem-10hz",
                period: 10,
                phase: 3,
                run: |_| {
                    SLOW.fetch_add(1, Ordering::Relaxed);
                    false
                },
            })
            .unwrap();
        for _ in 0..100 {
            sched.tick();
        }
        assert_eq!(FAST.load(Ordering::Relaxed), 100);
        // Phase 3: runs at ticks 7, 17, …, 97 → 10 runs.
        assert_eq!(SLOW.load(Ordering::Relaxed), 10);
    }

    #[test]
    fn deadline_misses_are_counted_per_task() {
        let mut sched = Scheduler::new();
        sched
            .register(Task {
                name: "overloaded",
                period: 5,
                phase: 0,
                run: |_| true, // always late
            })
            .unwrap();
        for _ in 0..20 {
            sched.tick();
        }
        let s = sched.stats(0).unwrap();
        assert_eq!(s.runs, 4);
        assert_eq!(s.missed, 4);
    }

    #[test]
    fn table_full_and_zero_period_rejected() {
        let mut sched = Scheduler::new();
        for i in 0..MAX_TASKS {
            sched
                .register(Task {
                    name: "t",
                    period: 1,
                    phase: 0,
                    run: |_| false,
                })
                .unwrap_or_else(|e| panic!("task {i}: {e}"));
        }
        assert_eq!(sched.task_count(), MAX_TASKS);
        assert_eq!(
            sched.register(Task {
                name: "extra",
                period: 1,
                phase: 0,
                run: |_| false,
            }),
            Err(SchedError::TableFull)
        );
        assert_eq!(
            sched.register(Task {
                name: "spin",
                period: 0,
                phase: 0,
                run: |_| false,
            }),
            Err(SchedError::ZeroPeriod)
        );
    }

    #[test]
    fn display_messages() {
        assert_eq!(SchedError::TableFull.to_string(), "static task table full");
    }
}
