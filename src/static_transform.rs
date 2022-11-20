//! Provides utilities for **static** transforms between [`CoordinateSystem`]s
//! that do not change with time.

use std::{fmt::Debug, marker::PhantomData};

use nalgebra::{Isometry3, Matrix3, RealField};
use serde::Serialize;

use crate::{CoordinateSystem, IsCoordinateSystemId, ProjectiveTransform, SE3Transform};

/// Static version of [`SE3Transform`] that does not change with time.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct StaticSE3Transform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    _src: PhantomData<DstId>,
    _dst: PhantomData<SrcId>,
    transform: Isometry3<T>,
}

impl<DstId, SrcId, T> StaticSE3Transform<DstId, SrcId, T>
where
    T: Copy + RealField + Serialize,
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
{
    pub fn new(transform: Isometry3<T>) -> Self {
        // TODO: figure out some way to prevent setting identity transform
        Self {
            _src: PhantomData,
            _dst: PhantomData,
            transform,
        }
    }

    pub fn transform(&self) -> Isometry3<T> {
        self.transform
    }

    pub fn at_time(&self, time: u64) -> SE3Transform<DstId, SrcId, T> {
        SE3Transform::new(
            CoordinateSystem::at_time(time),
            CoordinateSystem::at_time(time),
            self.transform,
        )
    }

    pub fn invert(&self) -> StaticSE3Transform<SrcId, DstId, T> {
        StaticSE3Transform::new(self.transform.inverse())
    }

    pub fn compose_with<RhsSrcId>(
        &self,
        rhs: StaticSE3Transform<SrcId, RhsSrcId, T>,
    ) -> StaticSE3Transform<DstId, RhsSrcId, T>
    where
        RhsSrcId: IsCoordinateSystemId,
    {
        StaticSE3Transform::new(self.transform * rhs.transform)
    }
}

/// Static version of [`ProjectiveTransform`] that does not change with time.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct StaticProjectiveTransform<DstId, SrcId, T>
where
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
    T: Copy + RealField + Serialize,
{
    _src: PhantomData<DstId>,
    _dst: PhantomData<SrcId>,
    k: Matrix3<T>,
}

impl<DstId, SrcId, T> StaticProjectiveTransform<DstId, SrcId, T>
where
    T: Copy + RealField + Serialize,
    DstId: IsCoordinateSystemId,
    SrcId: IsCoordinateSystemId,
{
    pub fn new(k: Matrix3<T>) -> Self {
        // TODO: figure out some way to prevent setting identity transform
        Self {
            _src: PhantomData,
            _dst: PhantomData,
            k,
        }
    }

    pub fn k(&self) -> Matrix3<T> {
        self.k
    }

    pub fn at_time(&self, time: u64) -> ProjectiveTransform<DstId, SrcId, T> {
        ProjectiveTransform::new(
            CoordinateSystem::at_time(time),
            CoordinateSystem::at_time(time),
            self.k,
        )
    }
}
