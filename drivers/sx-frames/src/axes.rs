//! Directions, axes and the basis change between two axes, computed — never typed in.

use std::fmt;

use crate::FrameError;
use crate::linalg::{Matrix3, mat_mul, transpose};

/// A physical direction relative to the facing of the thing a frame is attached to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Direction {
    Forward,
    Back,
    Left,
    Right,
    Up,
    Down,
}

impl Direction {
    pub const ALL: [Self; 6] = [
        Self::Forward,
        Self::Back,
        Self::Left,
        Self::Right,
        Self::Up,
        Self::Down,
    ];

    /// This direction in the facing's own forward-left-up coordinates.
    #[must_use]
    #[inline]
    pub const fn vector(self) -> [f64; 3] {
        match self {
            Self::Forward => [1.0, 0.0, 0.0],
            Self::Back => [-1.0, 0.0, 0.0],
            Self::Left => [0.0, 1.0, 0.0],
            Self::Right => [0.0, -1.0, 0.0],
            Self::Up => [0.0, 0.0, 1.0],
            Self::Down => [0.0, 0.0, -1.0],
        }
    }

    /// The wire and document spelling: `"forward"`, `"left"`, ...
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Forward => "forward",
            Self::Back => "back",
            Self::Left => "left",
            Self::Right => "right",
            Self::Up => "up",
            Self::Down => "down",
        }
    }

    /// The letter in a three-letter axes name: `F B L R U D`.
    #[must_use]
    pub const fn letter(self) -> char {
        match self {
            Self::Forward => 'F',
            Self::Back => 'B',
            Self::Left => 'L',
            Self::Right => 'R',
            Self::Up => 'U',
            Self::Down => 'D',
        }
    }

    /// The direction a document spells.
    ///
    /// # Errors
    ///
    /// [`FrameError::Axes`] for any other word.
    pub fn parse(word: &str) -> Result<Self, FrameError> {
        Self::ALL
            .into_iter()
            .find(|direction| direction.as_str() == word)
            .ok_or_else(|| FrameError::Axes(format!("unknown direction {word:?}")))
    }

    const fn line(self) -> usize {
        match self {
            Self::Forward | Self::Back => 0,
            Self::Left | Self::Right => 1,
            Self::Up | Self::Down => 2,
        }
    }
}

/// Whether a set of axes is right- or left-handed. Derived from the axes, never stored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Handedness {
    Right,
    Left,
}

impl Handedness {
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Right => "right",
            Self::Left => "left",
        }
    }
}

/// Which physical direction each of x, y and z points, on the thing's facing.
///
/// `Axes(RIGHT, DOWN, FORWARD)` is `OpenCV`'s optical frame; `Axes(FORWARD, LEFT, UP)` is a
/// ROS/URDF body. The named values below are values, not concepts.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Axes {
    x: Direction,
    y: Direction,
    z: Direction,
}

/// ROS REP-103 body and URDF link axes.
pub const FLU: Axes = Axes::known(Direction::Forward, Direction::Left, Direction::Up);
/// `OpenCV` / pinhole optical axes: x image-right, y image-down, z along the view.
pub const RDF: Axes = Axes::known(Direction::Right, Direction::Down, Direction::Forward);
/// `OpenGL`, USD, `MuJoCo` cameras and WebXR/OpenXR spaces: the view looks along -z.
pub const RUB: Axes = Axes::known(Direction::Right, Direction::Up, Direction::Back);
/// The Z-up world the Quest converter has written since 2026-05 (x right, y forward).
pub const RFU: Axes = Axes::known(Direction::Right, Direction::Forward, Direction::Up);
/// glTF assets and Aria's central pupil frame: x left, y up, z forward.
pub const LUF: Axes = Axes::known(Direction::Left, Direction::Up, Direction::Forward);
/// Unity world and Unity XR tracking space (`RUF`): left-handed, only ever a recording source.
pub const UNITY: Axes = Axes::known(Direction::Right, Direction::Up, Direction::Forward);

/// Every named value, by its conventional spelling, in the Python kernel's order.
pub const NAMED_AXES: [(&str, Axes); 6] = [
    ("FLU", FLU),
    ("RDF", RDF),
    ("RUB", RUB),
    ("RFU", RFU),
    ("LUF", LUF),
    ("RUF", UNITY),
];

impl Axes {
    /// Three directions, one per axis.
    ///
    /// # Errors
    ///
    /// [`FrameError::Axes`] when two axes lie on one line.
    #[inline]
    pub const fn new(x: Direction, y: Direction, z: Direction) -> Result<Self, FrameError> {
        if x.line() == y.line() || y.line() == z.line() || x.line() == z.line() {
            return Err(FrameError::AxesRepeatLine);
        }
        Ok(Self { x, y, z })
    }

    /// A named constant; the lines are distinct by inspection.
    const fn known(x: Direction, y: Direction, z: Direction) -> Self {
        Self { x, y, z }
    }

    /// `"RDF"` → `Axes(Right, Down, Forward)`.
    ///
    /// # Errors
    ///
    /// [`FrameError::Axes`] for anything but three of `F B L R U D` on distinct lines.
    pub fn parse(spelling: &str) -> Result<Self, FrameError> {
        let refuse = || {
            FrameError::Axes(format!(
                "axes spelling {spelling:?} is not three of F B L R U D"
            ))
        };
        let mut directions = [Direction::Forward; 3];
        let mut letters = spelling.chars();
        for slot in &mut directions {
            let letter = letters.next().ok_or_else(refuse)?.to_ascii_uppercase();
            *slot = Direction::ALL
                .into_iter()
                .find(|direction| direction.letter() == letter)
                .ok_or_else(refuse)?;
        }
        if letters.next().is_some() {
            return Err(refuse());
        }
        Self::new(directions[0], directions[1], directions[2])
    }

    #[must_use]
    #[inline]
    pub const fn directions(self) -> [Direction; 3] {
        [self.x, self.y, self.z]
    }

    /// Columns are x, y, z expressed in the facing's forward-left-up coordinates.
    #[must_use]
    #[inline]
    pub const fn facing_from_axes(self) -> Matrix3 {
        let (x, y, z) = (self.x.vector(), self.y.vector(), self.z.vector());
        [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]
    }

    #[must_use]
    #[inline]
    pub fn handedness(self) -> Handedness {
        if crate::linalg::determinant(&self.facing_from_axes()) > 0.0 {
            Handedness::Right
        } else {
            Handedness::Left
        }
    }
}

impl fmt::Display for Axes {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}{}{}",
            self.x.letter(),
            self.y.letter(),
            self.z.letter()
        )
    }
}

/// `parent_from_child` for two right-handed axes on one thing at one origin: a signed
/// permutation.
///
/// # Errors
///
/// [`FrameError::LeftHanded`] when either is left-handed: left-handed numbers enter only by
/// [`crate::recorded`].
pub fn basis(parent: Axes, child: Axes) -> Result<Matrix3, FrameError> {
    for axes in [parent, child] {
        if axes.handedness() == Handedness::Left {
            return Err(FrameError::LeftHanded(axes));
        }
    }
    Ok(basis_unchecked(parent, child))
}

pub(crate) fn basis_unchecked(parent: Axes, child: Axes) -> Matrix3 {
    mat_mul(
        &transpose(&parent.facing_from_axes()),
        &child.facing_from_axes(),
    )
}
