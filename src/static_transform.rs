use nalgebra::{Isometry3, RealField};
use serde::Serialize;
use std::fmt::Debug;

use super::{CoordinateSystem, IsRobotPart, Transform, SE3};

#[derive(Debug, Clone, Copy, Serialize)]
pub struct StaticTransform<T, DstRobotFrame, SrcRobotFrame>
where
    T: Copy + RealField + Serialize,
    DstRobotFrame: IsRobotPart,
    SrcRobotFrame: IsRobotPart,
{
    pub dst: DstRobotFrame,
    pub src: SrcRobotFrame,
    pub transform: Isometry3<T>,
}

impl<T, DstRobotFrame, SrcRobotFrame> StaticTransform<T, DstRobotFrame, SrcRobotFrame>
where
    T: Copy + RealField + Serialize,
    DstRobotFrame: IsRobotPart,
    SrcRobotFrame: IsRobotPart,
{
    pub fn new(dst: DstRobotFrame, src: SrcRobotFrame, transform: Isometry3<T>) -> Self {
        Self { dst, src, transform }
    }

    pub fn invert(self) -> StaticTransform<T, SrcRobotFrame, DstRobotFrame> {
        StaticTransform::new(self.src, self.dst, self.transform.inverse())
    }

    pub fn to_transform_at_time(
        self,
        time: u64,
    ) -> Transform<Isometry3<T>, DstRobotFrame, SE3, Isometry3<T>, SrcRobotFrame, SE3, Isometry3<T>> {
        Transform::new(
            CoordinateSystem::at_time(time),
            CoordinateSystem::at_time(time),
            self.transform,
        )
    }
}

impl<T, RobotFrame> StaticTransform<T, RobotFrame, RobotFrame>
where
    T: Copy + RealField + Serialize,
    RobotFrame: IsRobotPart,
{
    pub fn identity() -> Self {
        Self::new(RobotFrame::default(), RobotFrame::default(), Isometry3::identity())
    }
}

impl<T, RobotFrameA, RobotFrameB> StaticTransform<T, RobotFrameA, RobotFrameB>
where
    T: Copy + RealField + Serialize,
    RobotFrameA: IsRobotPart,
    RobotFrameB: IsRobotPart,
{
    pub fn compose_with<RobotFrameC>(
        self,
        rhs: StaticTransform<T, RobotFrameB, RobotFrameC>,
    ) -> StaticTransform<T, RobotFrameA, RobotFrameC>
    where
        RobotFrameC: IsRobotPart,
    {
        StaticTransform::new(self.dst, rhs.src, self.transform * rhs.transform)
    }
}
