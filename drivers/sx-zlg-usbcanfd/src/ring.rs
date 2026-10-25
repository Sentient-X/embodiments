//! A fixed-capacity ring, allocated once: the receive and transmit queues never allocate per
//! frame.

/// First in, first out, of at most `capacity` values.
pub struct Ring<T: Copy> {
    slots: Box<[T]>,
    head: usize,
    len: usize,
}

impl<T: Copy> Ring<T> {
    /// A ring of `capacity` slots, each set to `empty`.
    #[must_use]
    pub fn new(capacity: usize, empty: T) -> Self {
        Self {
            slots: vec![empty; capacity.max(1)].into_boxed_slice(),
            head: 0,
            len: 0,
        }
    }

    #[must_use]
    pub const fn len(&self) -> usize {
        self.len
    }

    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    #[must_use]
    pub fn capacity(&self) -> usize {
        self.slots.len()
    }

    #[must_use]
    pub fn free(&self) -> usize {
        self.capacity() - self.len
    }

    /// Append `value`, or hand it back when the ring is full.
    ///
    /// # Errors
    ///
    /// Returns `value` when there is no free slot.
    pub fn push(&mut self, value: T) -> Result<(), T> {
        if self.len == self.slots.len() {
            return Err(value);
        }
        let tail = (self.head + self.len) % self.slots.len();
        self.slots[tail] = value;
        self.len += 1;
        Ok(())
    }

    /// The oldest value.
    pub fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        let value = self.slots[self.head];
        self.head = (self.head + 1) % self.slots.len();
        self.len -= 1;
        Some(value)
    }

    /// Move up to `out.len()` of the oldest values into `out`, returning how many.
    pub fn drain_into(&mut self, out: &mut [T]) -> usize {
        let count = out.len().min(self.len);
        for slot in out.iter_mut().take(count) {
            *slot = self.slots[self.head];
            self.head = (self.head + 1) % self.slots.len();
        }
        self.len -= count;
        count
    }

    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn values_come_out_in_order_across_the_wrap_and_a_full_ring_refuses() {
        let mut ring = Ring::new(3, 0_u32);
        for round in 0..10 {
            ring.push(round * 2).expect("room");
            ring.push(round * 2 + 1).expect("room");
            assert_eq!(ring.pop(), Some(round * 2));
            assert_eq!(ring.pop(), Some(round * 2 + 1));
        }
        ring.push(1).expect("room");
        ring.push(2).expect("room");
        ring.push(3).expect("room");
        assert_eq!(ring.push(4), Err(4));
        assert_eq!(ring.free(), 0);
        let mut out = [0; 2];
        assert_eq!(ring.drain_into(&mut out), 2);
        assert_eq!(out, [1, 2]);
        assert_eq!(ring.len(), 1);
        assert_eq!(ring.drain_into(&mut out), 1);
        assert!(ring.is_empty());
        assert_eq!(ring.pop(), None);
    }
}
