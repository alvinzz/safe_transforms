use nalgebra::{Isometry3, RealField};
use serde::Serialize;
use std::fmt::Debug;

use super::{CoordinateSystem, IsCoordinateFrameId, Transform, SE3};

#[derive(Debug, Clone, Copy, Serialize)]
pub struct StaticTransform<T, DstCoordinateFrameId, SrcCoordinateFrameId>
where
    T: Copy + RealField + Serialize,
    DstCoordinateFrameId: IsCoordinateFrameId,
    SrcCoordinateFrameId: IsCoordinateFrameId,
{
    pub dst: DstCoordinateFrameId,
    pub src: SrcCoordinateFrameId,
    pub transform: Isometry3<T>,
}

impl<T, DstCoordinateFrameId, SrcCoordinateFrameId>
    StaticTransform<T, DstCoordinateFrameId, SrcCoordinateFrameId>
where
    T: Copy + RealField + Serialize,
    DstCoordinateFrameId: IsCoordinateFrameId,
    SrcCoordinateFrameId: IsCoordinateFrameId,
{
    pub fn new(
        dst: DstCoordinateFrameId,
        src: SrcCoordinateFrameId,
        transform: Isometry3<T>,
    ) -> Self {
        Self {
            dst,
            src,
            transform,
        }
    }

    pub fn invert(self) -> StaticTransform<T, SrcCoordinateFrameId, DstCoordinateFrameId> {
        StaticTransform::new(self.src, self.dst, self.transform.inverse())
    }

    pub fn to_transform_at_time(
        self,
        time: u64,
    ) -> Transform<
        Isometry3<T>,
        DstCoordinateFrameId,
        SE3,
        Isometry3<T>,
        SrcCoordinateFrameId,
        SE3,
        Isometry3<T>,
    > {
        Transform::new(
            CoordinateSystem::at_time(time),
            CoordinateSystem::at_time(time),
            self.transform,
        )
    }
}

impl<T, CoordinateFrameId> StaticTransform<T, CoordinateFrameId, CoordinateFrameId>
where
    T: Copy + RealField + Serialize,
    CoordinateFrameId: IsCoordinateFrameId,
{
    pub fn identity() -> Self {
        Self::new(
            CoordinateFrameId::default(),
            CoordinateFrameId::default(),
            Isometry3::identity(),
        )
    }
}

impl<T, CoordinateFrameIdA, CoordinateFrameIdB>
    StaticTransform<T, CoordinateFrameIdA, CoordinateFrameIdB>
where
    T: Copy + RealField + Serialize,
    CoordinateFrameIdA: IsCoordinateFrameId,
    CoordinateFrameIdB: IsCoordinateFrameId,
{
    pub fn compose_with<CoordinateFrameIdC>(
        self,
        rhs: StaticTransform<T, CoordinateFrameIdB, CoordinateFrameIdC>,
    ) -> StaticTransform<T, CoordinateFrameIdA, CoordinateFrameIdC>
    where
        CoordinateFrameIdC: IsCoordinateFrameId,
    {
        StaticTransform::new(self.dst, rhs.src, self.transform * rhs.transform)
    }
}
