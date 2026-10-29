//! Central event loop for routing spacecraft events off the hot
//! path. Drains a [`SpscRing`](crate::bus::SpscRing) (or a whole
//! [`Bus`](crate::bus::Bus)) and dispatches each event to a handler.
//! Intentionally allocation-free in [`EventLoop::step`] and
//! [`EventLoop::drain`].

use crate::bus::{Bus, Consumer, Producer, SpscRing};
use std::sync::Arc;

/// An event loop that consumes `E` events from a [`Consumer`] and dispatches
/// them via a caller-supplied handler.
pub struct EventLoop<E, const N: usize> {
    rx: Consumer<E, N>,
}

impl<E, const N: usize> EventLoop<E, N> {
    /// Wrap a consumer half of an event ring.
    pub fn new(rx: Consumer<E, N>) -> Self {
        Self { rx }
    }

    /// Process a single buffered event, returning `false` when the ring is
    /// empty (no work done this tick).
    pub fn step<H: FnMut(E)>(&self, mut handler: H) -> bool {
        match self.rx.try_pop() {
            Some(e) => {
                handler(e);
                true
            }
            None => false,
        }
    }

    /// Drain all currently-buffered events, invoking `handler` for each.
    /// Returns the number of events processed.
    pub fn drain<H: FnMut(E)>(&self, mut handler: H) -> usize {
        let mut n = 0;
        while let Some(e) = self.rx.try_pop() {
            handler(e);
            n += 1;
        }
        n
    }
}

/// Build a paired producer + [`EventLoop`] over a shared `N`-slot event ring.
///
/// The returned [`Producer`] is the hot-path send side (e.g. fed by sensor /
/// safety threads); the [`EventLoop`] is driven on the dispatcher thread.
pub fn event_channel<E, const N: usize>() -> (Producer<E, N>, EventLoop<E, N>) {
    let ring = Arc::new(SpscRing::new());
    let (tx, rx) = ring.split();
    (tx, EventLoop::new(rx))
}

/// An event loop over a whole [`Bus`]: drains every producer ring
/// round-robin and reports the source ring with each event.
pub struct BusLoop<T, const P: usize, const N: usize> {
    bus: Arc<Bus<T, P, N>>,
}

impl<T, const P: usize, const N: usize> BusLoop<T, P, N> {
    /// Wrap a shared bus as the consumer side.
    pub fn new(bus: Arc<Bus<T, P, N>>) -> Self {
        Self { bus }
    }

    /// Process one event from any producer ring, returning `Ok((source,
    /// handled))` — `Ok(None)`-style via `Option` when the bus is empty.
    pub fn step<H: FnMut(usize, T)>(&self, mut handler: H) -> Option<usize> {
        let (from, e) = self.bus.try_pop_any()?;
        handler(from, e);
        Some(from)
    }

    /// Drain all currently-buffered events across all rings. Returns the
    /// number of events processed.
    pub fn drain<H: FnMut(usize, T)>(&self, mut handler: H) -> usize {
        let mut n = 0;
        while let Some((from, e)) = self.bus.try_pop_any() {
            handler(from, e);
            n += 1;
        }
        n
    }
}

/// Build a shared [`Bus`] plus its [`BusLoop`] consumer.
pub fn bus_channel<T, const P: usize, const N: usize>() -> (Arc<Bus<T, P, N>>, BusLoop<T, P, N>) {
    let bus = Arc::new(Bus::new());
    let l = BusLoop::new(bus.clone());
    (bus, l)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_channel_drains_in_order() {
        let (tx, loop_) = event_channel::<u32, 8>();
        for i in 0..5 {
            tx.push(i);
        }
        let mut collected = Vec::new();
        let processed = loop_.drain(|e| collected.push(e));
        assert_eq!(processed, 5);
        assert_eq!(collected, vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn step_reports_empty() {
        let (_tx, loop_) = event_channel::<u32, 8>();
        let mut count = 0;
        assert!(!loop_.step(|_| count += 1));
        assert_eq!(count, 0);
    }

    #[test]
    fn bus_loop_reports_source_ring() {
        let (bus, loop_) = bus_channel::<u32, 3, 4>();
        bus.producer(0).push(10);
        bus.producer(2).push(20);
        let mut seen = Vec::new();
        let n = loop_.drain(|from, e| seen.push((from, e)));
        assert_eq!(n, 2);
        assert_eq!(seen, vec![(0, 10), (2, 20)]);
    }
}
