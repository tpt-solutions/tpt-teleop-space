//! Lock-free single-producer / single-consumer ring buffer and the
//! fan-in message bus used for inter-crate hot-path routing.
//!
//! The ring is wait-free on the fast path and makes no heap allocations after
//! construction. It targets SPSC topologies (one producer thread, one consumer
//! thread) — the common case for per-core sensor → fusion → control
//! pipelines. [`Bus`] composes `P` SPSC rings into a multi-producer fan-in
//! with a single polling consumer, preserving lock-freedom (each producer
//! touches only its own ring).

use std::cell::{Cell, UnsafeCell};
use std::mem::MaybeUninit;
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

/// A fixed-capacity, lock-free SPSC ring buffer of `T` with `N` slots.
///
/// `N` must be a power of two. The ring is safe to share across exactly one
/// producer thread and one consumer thread.
pub struct SpscRing<T, const N: usize> {
    buffer: UnsafeCell<Box<[MaybeUninit<T>]>>,
    head: AtomicUsize, // producer write cursor (monotonic)
    tail: AtomicUsize, // consumer read cursor (monotonic)
}

// SAFETY: SPSC discipline — `head` is written only by the producer and `tail`
// only by the consumer. The producer publishes a written slot via
// `head.store(Release)` and the consumer acquires it via `head.load(Acquire)`;
// the symmetric pattern holds for `tail`. Thus concurrent access is data-race
// free given the single producer / single consumer discipline, so `Send` is
// sound for `T: Send`.
unsafe impl<T: Send, const N: usize> Send for SpscRing<T, N> {}
// SAFETY: same rationale as `Send` above — the buffer is only ever mutated
// through the atomic-guarded SPSC protocol, so sharing `&SpscRing` across the
// one producer and one consumer thread is race-free for `T: Send`.
unsafe impl<T: Send, const N: usize> Sync for SpscRing<T, N> {}

impl<T, const N: usize> SpscRing<T, N> {
    const MASK: usize = N - 1;

    /// Create a new empty ring. `N` must be a power of two.
    pub fn new() -> Self {
        assert!(
            N.is_power_of_two(),
            "SpscRing capacity must be a power of two"
        );
        let mut v = Vec::with_capacity(N);
        for _ in 0..N {
            v.push(MaybeUninit::uninit());
        }
        Self {
            buffer: UnsafeCell::new(v.into_boxed_slice()),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    /// Number of slots currently occupied.
    pub fn len(&self) -> usize {
        self.head.load(Ordering::Acquire) - self.tail.load(Ordering::Acquire)
    }

    /// Whether the ring is empty.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Try to push one item. Returns `Err(item)` (the item back) if full.
    pub fn try_push(&self, item: T) -> Result<(), T> {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);
        if head - tail == N {
            return Err(item);
        }
        let idx = head & Self::MASK;
        // SAFETY: `idx` is unique to this push until `head` is published via the
        // `Release` store below; the consumer cannot read this slot until then.
        unsafe {
            let buf = &mut *self.buffer.get();
            buf[idx].write(item);
        }
        self.head.store(head + 1, Ordering::Release);
        Ok(())
    }

    /// Try to pop one item. Returns `None` if empty.
    pub fn try_pop(&self) -> Option<T> {
        let tail = self.tail.load(Ordering::Relaxed);
        let head = self.head.load(Ordering::Acquire);
        if tail == head {
            return None;
        }
        let idx = tail & Self::MASK;
        // SAFETY: this slot was published by the producer (its `head` store with
        // `Release`) and has not yet been consumed, so reading it is exclusive.
        let item = unsafe {
            let buf = &mut *self.buffer.get();
            buf[idx].assume_init_read()
        };
        self.tail.store(tail + 1, Ordering::Release);
        Some(item)
    }

    /// Split into producer/consumer handles sharing the backing storage.
    pub fn split(self: Arc<Self>) -> (Producer<T, N>, Consumer<T, N>) {
        (Producer(self.clone()), Consumer(self.clone()))
    }

    /// Push, panicking on full. Use only when capacity is sized for the
    /// steady state and overflow is a programming error.
    pub fn push(&self, item: T)
    where
        T: Send,
    {
        self.try_push(item)
            .ok()
            .expect("SpscRing producer overflow")
    }
}

impl<T, const N: usize> Default for SpscRing<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Producer half of a [`SpscRing`] (send side).
pub struct Producer<T, const N: usize>(Arc<SpscRing<T, N>>);
/// Consumer half of a [`SpscRing`] (receive side).
pub struct Consumer<T, const N: usize>(Arc<SpscRing<T, N>>);

impl<T, const N: usize> Producer<T, N> {
    /// Try to push one item. Returns `Err(item)` if full.
    pub fn try_push(&self, item: T) -> Result<(), T> {
        self.0.try_push(item)
    }

    /// Number of items buffered.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether the ring is empty.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl<T, const N: usize> Consumer<T, N> {
    /// Try to pop one item. Returns `None` if empty.
    pub fn try_pop(&self) -> Option<T> {
        self.0.try_pop()
    }

    /// Number of items buffered.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether the ring is empty.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

impl<T: Send, const N: usize> Producer<T, N> {
    /// Push, panicking on full. Use only when capacity is sized for the steady
    /// state and overflow is a programming error.
    pub fn push(&self, item: T) {
        self.try_push(item)
            .ok()
            .expect("SpscRing producer overflow")
    }
}

/// A lock-free multi-producer / single-consumer message bus: `P` independent
/// SPSC rings fanned into one polling consumer.
///
/// Each producer owns a private [`SpscRing`], so the bus adds no
/// synchronization beyond the per-ring SPSC protocol. The consumer drains
/// rings round-robin (fairness without starvation). Producers that overflow
/// back-pressure their own ring only; one stalled producer cannot block
/// another.
pub struct Bus<T, const P: usize, const N: usize> {
    rings: [SpscRing<T, N>; P],
    next: Cell<usize>, // consumer round-robin cursor
}

impl<T, const P: usize, const N: usize> Bus<T, P, N> {
    /// Create an empty bus with `P` per-producer rings of `N` slots each.
    pub fn new() -> Self {
        assert!(P > 0, "Bus needs at least one producer ring");
        Self {
            rings: std::array::from_fn(|_| SpscRing::new()),
            next: Cell::new(0),
        }
    }

    /// Access producer `i`'s ring (send side). One producer thread per index.
    pub fn producer(&self, i: usize) -> &SpscRing<T, N> {
        &self.rings[i]
    }

    /// Pop one item from the next non-empty ring (round-robin). Returns the
    /// producing ring index with the item.
    pub fn try_pop_any(&self) -> Option<(usize, T)> {
        for k in 0..P {
            let i = (self.next.get() + k) % P;
            if let Some(item) = self.rings[i].try_pop() {
                self.next.set((i + 1) % P);
                return Some((i, item));
            }
        }
        None
    }

    /// Total items buffered across all rings.
    pub fn len(&self) -> usize {
        self.rings.iter().map(SpscRing::len).sum()
    }

    /// Whether every ring is empty.
    pub fn is_empty(&self) -> bool {
        self.rings.iter().all(SpscRing::is_empty)
    }
}

impl<T, const P: usize, const N: usize> Default for Bus<T, P, N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_pop_single() {
        let ring: SpscRing<u32, 8> = SpscRing::new();
        assert!(ring.is_empty());
        assert!(ring.try_push(42).is_ok());
        assert_eq!(ring.len(), 1);
        assert_eq!(ring.try_pop(), Some(42));
        assert!(ring.is_empty());
    }

    #[test]
    fn full_rejects() {
        let ring: SpscRing<u32, 4> = SpscRing::new();
        for i in 0..4 {
            assert!(ring.try_push(i).is_ok());
        }
        assert_eq!(ring.try_push(99), Err(99));
        assert_eq!(ring.len(), 4);
    }

    #[test]
    fn wraps_around_power_of_two() {
        let ring: SpscRing<u32, 4> = SpscRing::new();
        for round in 0..3 {
            for i in 0..4 {
                assert!(ring.try_push(round * 10 + i).is_ok());
            }
            for i in 0..4 {
                assert_eq!(ring.try_pop(), Some(round * 10 + i));
            }
            assert!(ring.is_empty());
        }
    }

    #[test]
    fn split_producer_consumer() {
        let ring = Arc::new(SpscRing::<u64, 16>::new());
        let (tx, rx) = ring.split();
        for i in 0..16 {
            tx.push(i);
        }
        assert_eq!(tx.try_push(999), Err(999));
        let mut seen = Vec::new();
        while let Some(v) = rx.try_pop() {
            seen.push(v);
        }
        assert_eq!(seen, (0..16).collect::<Vec<_>>());
    }

    #[test]
    fn bus_fans_in_from_all_producers() {
        let bus: Bus<u32, 3, 4> = Bus::new();
        bus.producer(0).push(1);
        bus.producer(1).push(2);
        bus.producer(2).push(3);
        bus.producer(0).push(4);
        assert_eq!(bus.len(), 4);

        let mut seen = Vec::new();
        while let Some((from, item)) = bus.try_pop_any() {
            seen.push((from, item));
        }
        // Round-robin: producer 0 first, then 1, then 2, then 0 again.
        assert_eq!(seen, vec![(0, 1), (1, 2), (2, 3), (0, 4)]);
        assert!(bus.is_empty());
    }

    #[test]
    fn bus_empty_producers_are_skipped() {
        let bus: Bus<u32, 4, 4> = Bus::new();
        bus.producer(2).push(7);
        assert_eq!(bus.try_pop_any(), Some((2, 7)));
        assert_eq!(bus.try_pop_any(), None);
    }

    #[test]
    fn spsc_cross_thread() {
        let ring = Arc::new(SpscRing::<u32, 8>::new());
        let (tx, rx) = ring.split();
        let producer = std::thread::spawn(move || {
            for i in 0..100u32 {
                while tx.try_push(i).is_err() {
                    std::hint::spin_loop();
                }
            }
        });
        let mut count = 0u32;
        while count < 100 {
            if rx.try_pop().is_some() {
                count += 1;
            } else {
                std::hint::spin_loop();
            }
        }
        producer.join().unwrap();
        assert_eq!(count, 100);
    }
}
