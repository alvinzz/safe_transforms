use serde::Serialize;
use std::{fmt::Debug, hash::Hash, marker::PhantomData};

use super::{Differentiable, IsManifold};

pub trait IsCoordinateFrameId: Debug + Default + Copy + Eq + Hash + Serialize {}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize)]
pub struct CoordinateFrame<Id: IsCoordinateFrameId> {
    pub time: u64,
    pub id: Id,
}

impl<Id: IsCoordinateFrameId> CoordinateFrame<Id> {
    pub fn at_time(time: u64) -> Self {
        Self {
            time,
            id: Id::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct CoordinateSystem<
    Id: IsCoordinateFrameId,
    Manifold: IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
> {
    _repr: PhantomData<Repr>,
    pub frame: CoordinateFrame<Id>,
    pub manifold: Manifold,
}

impl<
        Id: IsCoordinateFrameId,
        Manifold: IsManifold + Differentiable,
        Repr: Debug + Copy + Serialize,
    > CoordinateSystem<Id, Manifold, Repr>
{
    pub fn at_frame(id: CoordinateFrame<Id>) -> Self {
        Self {
            _repr: PhantomData,
            frame: id,
            manifold: Manifold::default(),
        }
    }

    pub fn at_time(time: u64) -> Self {
        Self::at_frame(CoordinateFrame::at_time(time))
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct ManifoldElement<
    Id: IsCoordinateFrameId,
    Manifold: IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
> {
    pub coordinate_system: CoordinateSystem<Id, Manifold, Repr>,
    pub coordinates: Repr,
}

impl<
        Id: IsCoordinateFrameId,
        Manifold: IsManifold + Differentiable,
        Repr: Debug + Copy + Serialize,
    > ManifoldElement<Id, Manifold, Repr>
{
    pub fn new(coordinate_system: CoordinateSystem<Id, Manifold, Repr>, coordinates: Repr) -> Self {
        Self {
            coordinate_system,
            coordinates,
        }
    }
}

pub trait IsManifoldElement<
    Id: IsCoordinateFrameId,
    Manifold: IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
>: Debug + Copy + Serialize
{
    fn coordinate_system(self) -> CoordinateSystem<Id, Manifold, Repr>;
}

impl<
        Id: IsCoordinateFrameId,
        Manifold: IsManifold + Differentiable,
        Repr: Debug + Copy + Serialize,
    > IsManifoldElement<Id, Manifold, Repr> for ManifoldElement<Id, Manifold, Repr>
{
    fn coordinate_system(self) -> CoordinateSystem<Id, Manifold, Repr> {
        self.coordinate_system
    }
}

impl<
        Id: IsCoordinateFrameId,
        Manifold: IsManifold + Differentiable,
        Repr: Debug + Copy + Serialize,
    > PartialEq for CoordinateSystem<Id, Manifold, Repr>
{
    fn eq(&self, other: &Self) -> bool {
        self.frame == other.frame && self.manifold == other.manifold
    }
}
impl<
        Id: IsCoordinateFrameId,
        Manifold: IsManifold + Differentiable,
        Repr: Debug + Copy + Serialize,
    > Eq for CoordinateSystem<Id, Manifold, Repr>
{
}
impl<
        Id: IsCoordinateFrameId,
        Manifold: IsManifold + Differentiable,
        Repr: Debug + Copy + Serialize,
    > Hash for CoordinateSystem<Id, Manifold, Repr>
{
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        (self.frame, self.manifold).hash(state)
    }
}
