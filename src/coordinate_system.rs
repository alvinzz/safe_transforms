//! The core framework, which provides [`CoordinateSystem`]s and [`Point`]s.

use serde::Serialize;
use std::{fmt::Debug, hash::Hash, marker::PhantomData};

/// Marker Trait for Coordinate System IDs.
pub trait IsCoordinateSystemId: Debug + Default + Copy + Eq + Hash + Serialize {}

/// A Coordinate System. [`Point`] coordinates are written relative to a [`CoordinateSystem`].
/// [`CoordinateSystem`]s are defined by three attributes:
///  - an `id`, which is known at compile-time (e.g, "LeftCameraSE3" or "RightCameraImage")
///  - a `time`, which is known only at run-time
///  - a Representation (`Repr`) (e.g., [`nalgebra::Isometry3`] or [`nalgebra::Vector2`])
///
/// [`Point`]s written in this [`CoordinateSystem`] have their `coordinates` expressed in its Representation `Repr`.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct CoordinateSystem<Id: IsCoordinateSystemId, Repr: Debug + Copy + Serialize> {
    id: Id,
    time: u64,
    _r: PhantomData<Repr>,
}

impl<Id: IsCoordinateSystemId, Repr: Debug + Copy + Serialize> PartialEq
    for CoordinateSystem<Id, Repr>
{
    fn eq(&self, other: &Self) -> bool {
        self.id == other.id && self.time == other.time
    }
}

impl<Id: IsCoordinateSystemId, Repr: Debug + Copy + Serialize> Eq for CoordinateSystem<Id, Repr> {}

impl<Id: IsCoordinateSystemId, Repr: Debug + Copy + Serialize> Hash for CoordinateSystem<Id, Repr> {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        (self.id, self.time).hash(state)
    }
}

impl<Id: IsCoordinateSystemId, Repr: Debug + Copy + Serialize> CoordinateSystem<Id, Repr> {
    #[allow(dead_code)]
    fn id(&self) -> Id {
        self.id
    }

    #[allow(dead_code)]
    fn time(&self) -> u64 {
        self.time
    }

    /// Get the [`CoordinateSystem`] with the defined `Id` at the target time.
    pub fn at_time(time: u64) -> Self {
        Self {
            id: Id::default(),
            time,
            _r: PhantomData,
        }
    }
}

/// A Point, written relative to some [`CoordinateSystem`].
#[derive(Debug, Clone, Copy, Serialize)]
pub struct Point<Id: IsCoordinateSystemId, Repr: Debug + Copy + Serialize> {
    coordinate_system: CoordinateSystem<Id, Repr>,
    coordinates: Repr,
}

impl<Id: IsCoordinateSystemId, Repr: Debug + Copy + Serialize> Point<Id, Repr> {
    pub fn new(coordinate_system: CoordinateSystem<Id, Repr>, coordinates: Repr) -> Self {
        Self {
            coordinate_system,
            coordinates,
        }
    }

    pub fn coordinate_system(&self) -> CoordinateSystem<Id, Repr> {
        self.coordinate_system
    }

    pub fn coordinates(&self) -> Repr {
        self.coordinates
    }
}
