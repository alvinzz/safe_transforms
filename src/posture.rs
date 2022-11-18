// TODO: This file needs a better name

use std::{f32::consts::PI, fmt::Display, hash::Hash, ops::Mul};

use nalgebra::{Isometry2, Translation2, UnitComplex};
use serde::{Deserialize, Serialize};

use crate::{
    types::{Direction, Timestamped},
    util::at_facing,
};

/// Motion planning coordinate primitive.
pub type Coord = f32;

/// Distances in the `Coord` space are measured in meters.
///
/// Eg. the distance between two points (x1,0) and (x2,0) is (x2 - x1).abs() meters.
pub type Meters = f32;

/// `Orientation` representation in SE(2) space for planning and control.
pub type Orientation = nalgebra::UnitComplex<f32>;

/// Internal `Point` representation for 2D Euclidean space positioning.
pub type Point = nalgebra::Point2<Coord>;

/// `Posture` representation in SE(2) space for behavior control.
/// Will not contain NaN or -0.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Posture(nalgebra::Isometry2<f32>);

impl Eq for Posture {}

impl PartialOrd for Posture {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Posture {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.compare_on(other, |t| t.translation.x)
            .then_with(|| self.compare_on(other, |t| t.translation.y))
            .then_with(|| self.compare_on(other, |t| t.rotation.re))
            .then_with(|| self.compare_on(other, |t| t.rotation.im))
    }
}

// The input-validation ensures that Hash and Eq line up.
#[allow(clippy::derive_hash_xor_eq)]
impl Hash for Posture {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        self.0.translation.vector.x.to_bits().hash(state);
        self.0.translation.vector.y.to_bits().hash(state);
        self.0.rotation.re.to_bits().hash(state);
        self.0.rotation.im.to_bits().hash(state);
    }
}

pub type TimestampedPosture = Timestamped<Posture>;

/// `Pose` representation in SE(3) space for global positioning.
pub type Pose = nalgebra::Isometry3<f32>;
pub type TimestampedPose = Timestamped<Pose>;

pub type Radians = f32;

impl Posture {
    pub fn new(position: Point, angle: Radians) -> Self {
        Self(Isometry2::new(position.coords, angle)).validate()
    }
    pub fn isometry(self) -> Isometry2<Coord> {
        self.0
    }
    pub fn origin() -> Self {
        Self::new(Point::origin(), 0.)
    }
    pub fn position(self) -> Point {
        Point { coords: self.0.translation.vector }
    }
    pub fn x(self) -> Coord {
        self.0.translation.vector.x
    }
    pub fn y(self) -> Coord {
        self.0.translation.vector.y
    }
    pub fn rotation(self) -> UnitComplex<Meters> {
        self.0.rotation
    }
    pub fn angle(self) -> Radians {
        self.0.rotation.angle()
    }
    pub fn inv_mul(self, rhs: Self) -> Isometry2<Meters> {
        self.0.inv_mul(&rhs.0)
    }
    pub fn inv_mul_point(&self, point: Point) -> Translation2<Coord> {
        Translation2 { vector: self.0.inverse_transform_point(&point).coords }
    }
    pub fn from_pt_rot(point: Point, rotation: UnitComplex<f32>) -> Self {
        Self::from_isometry(Isometry2 { translation: Translation2 { vector: point.coords }, rotation })
    }
    pub fn from_isometry(isometry: Isometry2<Coord>) -> Self {
        Self(isometry).validate()
    }
    pub fn translate(self, translation: &Translation2<Coord>) -> Self {
        let mut isometry = self.isometry();
        isometry.append_translation_mut(translation);
        Self::from_isometry(isometry)
    }
    /// Traverse the segment from here to the target in whichever direction induces the least total rotation
    /// (counting both the initial rotation towards the target and the final rotation upon reaching the target).
    pub fn natural_direction(self, target: Self) -> Direction {
        if self.position() == target.position() {
            return Direction::Forward;
        }
        let forward_orientation = at_facing(self.position(), target.position()).rotation();
        let forward_initial_turn = forward_orientation.angle_to(&self.rotation()).abs();
        let forward_final_turn = forward_orientation.angle_to(&target.rotation()).abs();
        let total_forward_turning = forward_initial_turn + forward_final_turn;
        let total_backward_turning = PI - forward_initial_turn + PI - forward_final_turn;
        if total_forward_turning <= total_backward_turning {
            Direction::Forward
        } else {
            Direction::Backward
        }
    }
    fn validate(mut self) -> Self {
        validate(&mut self.0.translation.x);
        validate(&mut self.0.translation.y);
        validate(&mut self.0.rotation.as_mut_unchecked().re);
        validate(&mut self.0.rotation.as_mut_unchecked().im);
        self
    }

    fn compare_on(&self, other: &Posture, f: impl Fn(Isometry2<f32>) -> f32) -> std::cmp::Ordering {
        f(self.0).partial_cmp(&f(other.0)).unwrap()
    }
}

fn validate(x: &mut f32) {
    assert!(!x.is_nan());
    if *x == -0. {
        *x = 0.;
    }
}

impl Mul<Isometry2<Meters>> for Posture {
    type Output = Posture;

    fn mul(self, rhs: Isometry2<Meters>) -> Self::Output {
        Posture(self.0 * rhs).validate()
    }
}

impl Mul<Translation2<Meters>> for Posture {
    type Output = Point;

    fn mul(self, rhs: Translation2<Meters>) -> Self::Output {
        Point { coords: (self.0 * rhs).translation.vector }
    }
}

impl Display for Posture {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}, {}]", self.x(), self.y(), self.angle())
    }
}
