//! Provides the framework for transforming [`Point`]s between different [`CoordinateSystem`]s.

use std::fmt::Debug;

use nalgebra::{Isometry3, Matrix3, RealField, Vector2};
use serde::Serialize;

use super::{CoordinateSystem, IsCoordinateSystemId, Point};

/// Trait for Transforms between [`CoordinateSystem`]s.
pub trait IsTransform<DstId, DstRepr, SrcId, SrcRepr>: Debug + Copy + Serialize
where
    DstId: IsCoordinateSystemId,
    DstRepr: Debug + Copy + Serialize,
    SrcId: IsCoordinateSystemId,
    SrcRepr: Debug + Copy + Serialize,
{
    /// [`CoordinateSystem`] of the [`Point`] after applying the Transform.
    fn dst(&self) -> CoordinateSystem<DstId, DstRepr>;
    /// [`CoordinateSystem`] of the [`Point`] before applying the Transform.
    fn src(&self) -> CoordinateSystem<SrcId, SrcRepr>;
    /// Function which applies the Transform, converting a [`Point`] in the `src`
    /// [`CoordinateSystem`] to a [`Point`] in the `dst` [`CoordinateSystem`].
    ///
    /// This function should not be implemented; it merely peforms a check and then
    /// calls `transform_inner`. Instead implement `transform_inner`.
    fn transform(&self, point: Point<SrcId, SrcRepr>) -> Point<DstId, DstRepr> {
        assert!(
            self.src() == point.coordinate_system(),
            "Transform source coordinate system {:?} does not match Point coordinate system {:?}.",
            self.src(),
            point.coordinate_system(),
        );
        self.transform_inner(point)
    }
    /// Performs the Transform after performing a run-time check.
    /// Should not be called by an external user, instead call `transform`.
    fn transform_inner(&self, point: Point<SrcId, SrcRepr>) -> Point<DstId, DstRepr>;
}

/// Represents a Transform between two SE3 [`CoordinateSystem`]s.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct SE3Transform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    dst: CoordinateSystem<DstId, Isometry3<T>>,
    src: CoordinateSystem<SrcId, Isometry3<T>>,
    transform: Isometry3<T>,
}

impl<DstId, SrcId, T> IsTransform<DstId, Isometry3<T>, SrcId, Isometry3<T>>
    for SE3Transform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    fn dst(&self) -> CoordinateSystem<DstId, Isometry3<T>> {
        self.dst
    }
    fn src(&self) -> CoordinateSystem<SrcId, Isometry3<T>> {
        self.src
    }
    fn transform_inner(&self, point: Point<SrcId, Isometry3<T>>) -> Point<DstId, Isometry3<T>> {
        Point::new(self.dst(), self.transform * point.coordinates())
    }
}

impl<DstId, SrcId, T> SE3Transform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    pub fn new(
        dst: CoordinateSystem<DstId, Isometry3<T>>,
        src: CoordinateSystem<SrcId, Isometry3<T>>,
        transform: Isometry3<T>,
    ) -> Self {
        // TODO: figure out some way to prevent setting identity transform
        Self {
            dst,
            src,
            transform,
        }
    }

    /// Invert a Transform between two SE3 [`CoordinateSystem`]s.
    pub fn invert(&self) -> SE3Transform<SrcId, DstId, T> {
        SE3Transform::new(self.src, self.dst, self.transform.inverse())
    }

    /// Compose two [`SE3Transform`]s.
    pub fn compose_with<RhsSrcId>(
        &self,
        rhs: SE3Transform<SrcId, RhsSrcId, T>,
    ) -> SE3Transform<DstId, RhsSrcId, T>
    where
        RhsSrcId: IsCoordinateSystemId,
    {
        assert!(
            self.src() == rhs.dst(),
            "Source coordinate system of `self` {:?} does not match Destination coordinate system of `rhs` {:?}.",
            self.src(),
            rhs.dst(),
        );
        SE3Transform::new(self.dst, rhs.src(), self.transform * rhs.transform)
    }
}

/// Represents a Transform from an SE3 [`CoordinateSystem`] to an Image-Plane [`CoordinateSystem`].
#[derive(Debug, Clone, Copy, Serialize)]
pub struct ProjectiveTransform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    dst: CoordinateSystem<DstId, Vector2<T>>,
    src: CoordinateSystem<SrcId, Isometry3<T>>,
    k: Matrix3<T>,
}

impl<DstId, SrcId, T> IsTransform<DstId, Vector2<T>, SrcId, Isometry3<T>>
    for ProjectiveTransform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    fn dst(&self) -> CoordinateSystem<DstId, Vector2<T>> {
        self.dst
    }
    fn src(&self) -> CoordinateSystem<SrcId, Isometry3<T>> {
        self.src
    }
    fn transform_inner(&self, point: Point<SrcId, Isometry3<T>>) -> Point<DstId, Vector2<T>> {
        let unnormalized_coords = self.k * point.coordinates().translation.vector;
        if unnormalized_coords[2] <= T::zero() {
            log::warn!("Projection had z-coordinate <= 0. Thus the Point may be phyically behind the Camera.");
        }
        let normalized_coords = Vector2::new(
            unnormalized_coords[0] / unnormalized_coords[2],
            unnormalized_coords[1] / unnormalized_coords[2],
        );
        Point::new(self.dst(), normalized_coords)
    }
}

impl<DstId, SrcId, T> ProjectiveTransform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    pub fn new(
        dst: CoordinateSystem<DstId, Vector2<T>>,
        src: CoordinateSystem<SrcId, Isometry3<T>>,
        k: Matrix3<T>,
    ) -> Self {
        assert!(
            k[(2, 0)] == T::zero() && k[(2, 1)] == T::zero() && k[(2, 2)] == T::one(),
            "Last row of camera intrinsics matrix must be [0, 0, 1], got [{}, {}, {}].",
            k[(2, 0)],
            k[(2, 1)],
            k[(2, 2)],
        );
        Self { dst, src, k }
    }
}
