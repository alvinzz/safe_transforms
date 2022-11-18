use serde::Serialize;
use std::{fmt::Debug, marker::PhantomData};

use super::{CoordinateSystem, IsManifoldElement, IsRobotFrame, IsManifold, Differentiable, ManifoldElement};

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Transform<TransformRepr, DstRobotFrame, DstManifold, DstRepr, SrcRobotFrame, SrcManifold, SrcRepr>
where
    DstRobotFrame: IsRobotFrame,
    DstManifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcRobotFrame: IsRobotFrame,
    SrcManifold: IsManifold + Differentiable,
    SrcRepr: Debug + Copy + Serialize,
{
    pub dst: CoordinateSystem<DstRobotFrame, DstManifold, DstRepr>,
    pub src: CoordinateSystem<SrcRobotFrame, SrcManifold, SrcRepr>,
    pub transform: TransformRepr,
}

impl<TransformRepr, DstRobotFrame, DstManifold, DstRepr, SrcRobotFrame, SrcManifold, SrcRepr>
    Transform<TransformRepr, DstRobotFrame, DstManifold, DstRepr, SrcRobotFrame, SrcManifold, SrcRepr>
where
    DstRobotFrame: IsRobotFrame,
    DstManifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcRobotFrame: IsRobotFrame,
    SrcManifold: IsManifold + Differentiable,
    SrcRepr: Debug + Copy + Serialize,
{
    pub fn new(
        dst: CoordinateSystem<DstRobotFrame, DstManifold, DstRepr>,
        src: CoordinateSystem<SrcRobotFrame, SrcManifold, SrcRepr>,
        transform: TransformRepr,
    ) -> Self {
        Self {
            dst,
            src,
            transform,
        }
    }
}

pub trait IsTransform<DstRobotFrame, DstManifold, DstRepr, SrcRobotFrame, SrcManifold, SrcRepr>:
    Debug + Copy + Serialize
where
    DstRobotFrame: IsRobotFrame,
    DstManifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcRobotFrame: IsRobotFrame,
    SrcManifold: IsManifold + Differentiable,
    SrcRepr: Debug + Copy + Serialize,
{
    fn dst(self) -> CoordinateSystem<DstRobotFrame, DstManifold, DstRepr>;
    fn src(self) -> CoordinateSystem<SrcRobotFrame, SrcManifold, SrcRepr>;
    fn transform(
        self,
        point: ManifoldElement<SrcRobotFrame, SrcManifold, SrcRepr>,
    ) -> ManifoldElement<DstRobotFrame, DstManifold, DstRepr> {
        assert!(self.src() == point.coordinate_system());
        todo!()
    }
}

pub trait IdentityTransform<TRepr, RobotFrame, Manifold, DstRepr, SrcRepr>: Debug + Copy + Serialize
where
    TRepr: Debug + Copy + Serialize,
    RobotFrame: IsRobotFrame,
    Manifold: IsManifold + Differentiable,
    DstRepr: Debug + Copy + Serialize,
    SrcRepr: Debug + Copy + Serialize,
{
    fn identity_at(
        dst: CoordinateSystem<RobotFrame, Manifold, DstRepr>,
    ) -> Transform<TRepr, RobotFrame, Manifold, DstRepr, RobotFrame, Manifold, SrcRepr>;
}

pub trait InvertableTransform<TRepr, DstRobotFrame, SrcRobotFrame, Manifold, Repr>:
    Debug + Copy + Serialize + IsTransform<DstRobotFrame, Manifold, Repr, SrcRobotFrame, Manifold, Repr>
where
    TRepr: Debug + Copy + Serialize,
    DstRobotFrame: IsRobotFrame,
    SrcRobotFrame: IsRobotFrame,
    Manifold: IsManifold + Differentiable,
    Repr: Debug + Copy + Serialize,
{
    fn invert(self) -> Transform<TRepr, SrcRobotFrame, Manifold, Repr, DstRobotFrame, Manifold, Repr>;
}

pub trait ComposableTransform<
    // TReprBC,
    // RobotFrameA,
    // ManifoldA,
    // ReprA,
    // RobotFrameB,
    // ManifoldB,
    // ReprB,
    // RobotFrameC,
    // ManifoldC,
    // ReprC,
    Transform2
>//: //Debug + Copy + Serialize + IsTransform<RobotFrameA, ManifoldA, ReprA, RobotFrameB, ManifoldB, ReprB> where
    // Transform2: Trans
    //RobotFrameA: IsRobotFrame,
    //ManifoldA: IsManifold + Differentiable,
    //ReprA: Debug + Copy + Serialize,
    // RobotFrameB: IsRobotFrame,
    // ManifoldB: IsManifold + Differentiable,
    // ReprB: Debug + Copy + Serialize,
    // TReprBC: Debug + Copy + Serialize,
    // RobotFrameC: IsRobotFrame,
    // ManifoldC: IsManifold + Differentiable,
    // ReprC: Debug + Copy + Serialize,
{
    // type TReprAC: Debug + Copy + Serialize;
    type Output;

    fn compose_with(
        self,
        rhs: Transform2 //Transform<TReprBC, RobotFrameB, ManifoldB, ReprB, RobotFrameC, ManifoldC, ReprC>,
    ) -> Self::Output//Transform<Self::TReprAC, RobotFrameA, ManifoldA, ReprA, RobotFrameC, ManifoldC, ReprC>
    // where
        // Transform<TReprBC, RobotFrameB, ManifoldB, ReprB, RobotFrameC, ManifoldC, ReprC>:
            // IsTransform<RobotFrameB, ManifoldB, ReprB, RobotFrameC, ManifoldC, ReprC>,
    {
        assert!(self.src() == rhs.dst());
        todo!()
    }
}
