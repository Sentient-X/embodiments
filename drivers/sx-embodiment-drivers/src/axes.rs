//! One value per axis, held inline: what a control tick returns without touching the heap.
//!
//! A chain answers every command with one observed position and one feedback per axis, and a
//! governor steps every axis each tick. Held in a `Vec`, each of those is an allocation per
//! tick, which a real-time control path does not make. [`Axes`] holds up to [`MAX_AXES`]
//! values in a fixed array, is `Copy`, and reads as a slice. A chain wider than [`MAX_AXES`]
//! is refused when it is validated ([`crate::validate_chain`]), so no tick ever has more
//! values than fit.

use std::fmt;
use std::ops::{Deref, DerefMut};

/// The most axes one chain carries.
pub const MAX_AXES: usize = 64;

/// Up to [`MAX_AXES`] values, one per axis in native state order, inline.
#[derive(Clone, Copy)]
pub struct Axes<T: Copy + Default> {
    len: usize,
    values: [T; MAX_AXES],
}

impl<T: Copy + Default> Axes<T> {
    /// No values.
    #[must_use]
    pub fn new() -> Self {
        Self {
            len: 0,
            values: [T::default(); MAX_AXES],
        }
    }

    /// `values`, the first [`MAX_AXES`] of them.
    #[must_use]
    pub fn from_slice(values: &[T]) -> Self {
        let mut axes = Self::new();
        axes.extend_from_slice(values);
        axes
    }

    /// `value` for each of `len` axes, at most [`MAX_AXES`].
    #[must_use]
    pub fn filled(value: T, len: usize) -> Self {
        let mut axes = Self::new();
        axes.len = len.min(MAX_AXES);
        axes.values[..axes.len].fill(value);
        axes
    }

    /// Append `value`; a value past [`MAX_AXES`] is dropped, which no validated chain reaches.
    pub fn push(&mut self, value: T) {
        if let Some(slot) = self.values.get_mut(self.len) {
            *slot = value;
            self.len += 1;
        }
    }

    /// Append `values`, as many as fit.
    pub fn extend_from_slice(&mut self, values: &[T]) {
        for value in values {
            self.push(*value);
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    /// The values, as a slice.
    #[must_use]
    pub fn as_slice(&self) -> &[T] {
        self
    }
}

impl<T: Copy + Default> Default for Axes<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: Copy + Default> Deref for Axes<T> {
    type Target = [T];

    fn deref(&self) -> &[T] {
        &self.values[..self.len]
    }
}

impl<T: Copy + Default> DerefMut for Axes<T> {
    fn deref_mut(&mut self) -> &mut [T] {
        &mut self.values[..self.len]
    }
}

impl<T: Copy + Default + PartialEq> PartialEq for Axes<T> {
    fn eq(&self, other: &Self) -> bool {
        **self == **other
    }
}

impl<T: Copy + Default + PartialEq> PartialEq<Vec<T>> for Axes<T> {
    fn eq(&self, other: &Vec<T>) -> bool {
        **self == **other
    }
}

impl<T: Copy + Default + PartialEq> PartialEq<[T]> for Axes<T> {
    fn eq(&self, other: &[T]) -> bool {
        **self == *other
    }
}

impl<T: Copy + Default + PartialEq, const N: usize> PartialEq<[T; N]> for Axes<T> {
    fn eq(&self, other: &[T; N]) -> bool {
        **self == *other
    }
}

impl<T: Copy + Default + fmt::Debug> fmt::Debug for Axes<T> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Debug::fmt(&**self, formatter)
    }
}

impl<T: Copy + Default> FromIterator<T> for Axes<T> {
    fn from_iter<I: IntoIterator<Item = T>>(values: I) -> Self {
        let mut axes = Self::new();
        for value in values {
            axes.push(value);
        }
        axes
    }
}

impl<T: Copy + Default> Extend<T> for Axes<T> {
    fn extend<I: IntoIterator<Item = T>>(&mut self, values: I) {
        for value in values {
            self.push(value);
        }
    }
}

impl<T: Copy + Default> From<Vec<T>> for Axes<T> {
    fn from(values: Vec<T>) -> Self {
        Self::from_slice(&values)
    }
}

impl<T: Copy + Default> From<&[T]> for Axes<T> {
    fn from(values: &[T]) -> Self {
        Self::from_slice(values)
    }
}

impl<'a, T: Copy + Default> IntoIterator for &'a Axes<T> {
    type Item = &'a T;
    type IntoIter = std::slice::Iter<'a, T>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<T: Copy + Default> IntoIterator for Axes<T> {
    type Item = T;
    type IntoIter = std::iter::Take<std::array::IntoIter<T, MAX_AXES>>;

    fn into_iter(self) -> Self::IntoIter {
        self.values.into_iter().take(self.len)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn axes_read_as_the_slice_they_hold_and_never_grow_past_their_capacity() {
        let mut axes: Axes<f64> = [1.0, 2.0].into_iter().collect();
        assert_eq!(&*axes, &[1.0, 2.0]);
        assert_eq!(axes, vec![1.0, 2.0]);
        axes.push(3.0);
        assert_eq!(axes.len(), 3);
        let full = Axes::filled(0.5, MAX_AXES + 3);
        assert_eq!(full.len(), MAX_AXES);
        let mut more = full;
        more.push(9.0);
        assert_eq!(more.len(), MAX_AXES);
        assert_eq!(format!("{:?}", Axes::from_slice(&[1_u8])), "[1]");
    }
}
